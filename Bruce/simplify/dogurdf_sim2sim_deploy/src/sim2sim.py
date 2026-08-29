"""Sim2sim: run a MJX-trained dogurdf policy in the *native* MuJoCo engine.

The training/play pipeline runs on MJX (GPU batched sim).  This script loads
the same policy checkpoint and the same MJCF, but steps the physics with the
native MuJoCo C engine (``mujoco.mj_step``) in a single environment.  Matching
behaviour across the two engines is the standard sim2sim validation done before
going to hardware.

Everything that touches the policy input/output is a faithful re-implementation
of the training pipeline (observation layout, PD control law, joint ordering).
See the plan/notes for the exact alignment points; the three that matter most:

  1. ``base_lin_vel`` (obs[0:3]) is ALWAYS ZERO.  Training feeds ``odom_data =
     zeros(3)`` every step, so the policy never saw real linear velocity.
  2. Deterministic inference uses the actor mean (``network.act``), no noise.
  3. Torques are injected via ``qfrc_applied`` (the MJCF has no actuators).

Usage
-----
    conda activate MJX && cd code
    python src/sim2sim.py --cmd_vel_x 1.0 \
        --checkpoint checkpoints/dogurdf_velocity/checkpoints_.../iteration_250.pkl
    # add --save_video --video_path /tmp/out.mp4 for offscreen rendering
"""

from __future__ import annotations

import argparse
import os
import pickle
import sys
import time
from pathlib import Path
from typing import Any

# Sim2sim is a single-env CPU workload; force JAX onto CPU so it never fights
# a running GPU training job for VRAM.  Honour an explicit override if set.
os.environ.setdefault("JAX_PLATFORMS", "cpu")

# Match train.py/play.py: make ``src`` importable regardless of invocation dir.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import jax
import jax.numpy as jnp
import numpy as np
import mujoco
from brax import math as brax_math

from cfg.experiments import get_experiment
from networks import ActorCriticMLP
from robots.dogurdf import (
    DOGURDF_JOINT_NAMES,
    JOINT_LIMITS,
    LEG_PREFIXES,
    NOMINAL_CALF,
    NOMINAL_HIP,
    NOMINAL_THIGH,
    NOMINAL_TORSO_HEIGHT,
    NUM_JOINTS,
    NUM_LEG_JOINTS,
    NUM_WHEELS,
    WHEEL_VEL_LIMIT,
)

try:
    import mediapy as media
    HAS_MEDIAPY = True
except ImportError:
    HAS_MEDIAPY = False


# --------------------------------------------------------------------------
# Static control constants (mirror dogurdf_config.get_dogurdf_config)
# --------------------------------------------------------------------------
ACTION_SCALE = 0.25
WHEEL_VEL_SCALE = WHEEL_VEL_LIMIT          # 12.5
LEG_KP, LEG_KD = 250.0, 4.0
WHEEL_KP, WHEEL_KD = 0.0, 2.0
LEG_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT = 250.0, 53.0
DECIMATION = 4
SIM_DT = 0.005
CONTROL_DT = SIM_DT * DECIMATION           # 0.02 s -> 50 Hz

# Gait phase clock, must stay in sync with cfg/dogurdf_config.py rewards:
#   gait_cycle_time, gait_phase_offsets.
GAIT_CYCLE_TIME = 0.6
GAIT_PHASE_OFFSETS = (0.0, 0.5, 0.5, 0.0)


def _default_pose_policy() -> np.ndarray:
    """Default joint angles in POLICY order (12 legs + 4 wheels).

    Mirrors ``GenericEnv._build_default_pose`` which orders by
    ``RobotSpec.joint_names`` == ``DOGURDF_JOINT_NAMES``.
    """
    angles = {}
    for leg in LEG_PREFIXES:
        angles[f"{leg}_hip_joint"] = NOMINAL_HIP
        angles[f"{leg}_thigh_joint"] = NOMINAL_THIGH
        angles[f"{leg}_calf_joint"] = NOMINAL_CALF
        angles[f"{leg}_wheel_joint"] = 0.0
    return np.array([angles[name] for name in DOGURDF_JOINT_NAMES], dtype=np.float64)


def _policy_joint_limits() -> tuple[np.ndarray, np.ndarray]:
    """(lower, upper) leg-joint limits in POLICY order, wheels unbounded."""
    lower = np.array(
        [JOINT_LIMITS[j][0] for _ in LEG_PREFIXES for j in ("hip", "thigh", "calf")]
        + [-1.0e6] * NUM_WHEELS
    )
    upper = np.array(
        [JOINT_LIMITS[j][1] for _ in LEG_PREFIXES for j in ("hip", "thigh", "calf")]
        + [1.0e6] * NUM_WHEELS
    )
    return lower, upper


class JointIndexer:
    """Maps between MuJoCo qpos/qvel slots and the policy joint ordering.

    Rather than trust a hard-coded permutation, we look every joint up by name
    in the loaded model.  ``qpos_adr[k]`` / ``dof_adr[k]`` give the qpos / qvel
    address of the k-th policy joint; the free joint occupies qpos[0:7],
    qvel[0:6] and is skipped.
    """

    def __init__(self, model: mujoco.MjModel):
        self.qpos_adr = np.empty(NUM_JOINTS, dtype=int)
        self.dof_adr = np.empty(NUM_JOINTS, dtype=int)
        for k, name in enumerate(DOGURDF_JOINT_NAMES):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise RuntimeError(f"Joint '{name}' not found in MJCF.")
            self.qpos_adr[k] = model.jnt_qposadr[jid]
            self.dof_adr[k] = model.jnt_dofadr[jid]

    def joint_pos(self, data: mujoco.MjData) -> np.ndarray:
        return data.qpos[self.qpos_adr]

    def joint_vel(self, data: mujoco.MjData) -> np.ndarray:
        return data.qvel[self.dof_adr]

    def set_default_pose(self, data: mujoco.MjData, pose_policy: np.ndarray) -> None:
        data.qpos[self.qpos_adr] = pose_policy


def _build_observation(
    data: mujoco.MjData,
    indexer: JointIndexer,
    default_pose: np.ndarray,
    last_action: np.ndarray,
    command: np.ndarray,
    step: int = 0,
    use_phase: bool = True,
) -> np.ndarray:
    """Assemble the actor observation in policy order.

    Layout (see components/tasks/velocity/observations.py):
        base_lin_vel(3)=0 | base_ang_vel(3) | projected_gravity(3)
        | joint_pos_rel(12) | joint_vel(16) | last_action(16) | command(3)
        [+ gait_phase(8: sin/cos per foot) when use_phase]

    ``use_phase=True`` yields the 64-D new-format observation (gait-phase
    clock); ``False`` reproduces the legacy 56-D layout so old checkpoints
    (pre-gait-phase, e.g. gentleturn/phasegait runs) can be driven too.

    ``step`` is the episode step counter; the gait-phase clock
    (phi = step*CONTROL_DT/GAIT_CYCLE_TIME + offset mod 1) is the SAME
    reference the phase_gait reward uses during training, so the policy's
    phase observation stays consistent at deployment.  Reset step to 0 on env
    reset, exactly like the training episode counter.
    """
    base_quat = np.array(data.qpos[3:7])                     # (w,x,y,z), MuJoCo order
    base_ang_vel_world = np.array(data.qvel[3:6])

    inv_rot = brax_math.quat_inv(jnp.asarray(base_quat))
    base_ang_vel = np.asarray(brax_math.rotate(jnp.asarray(base_ang_vel_world), inv_rot))
    projected_gravity = np.asarray(
        brax_math.rotate(jnp.array([0.0, 0.0, -1.0]), inv_rot)
    )

    joint_pos = indexer.joint_pos(data)                      # policy order, 16
    joint_vel = indexer.joint_vel(data)                      # policy order, 16
    pos_rel = (joint_pos - default_pose)[:NUM_LEG_JOINTS]    # legs only, 12

    obs_parts = [
        np.zeros(3),               # base_lin_vel  -> ALWAYS ZERO (see module docstring)
        base_ang_vel,              # 3
        projected_gravity,         # 3
        pos_rel,                   # 12
        joint_vel,                 # 16 (all joints incl. wheels)
        last_action,               # 16
        command,                   # 3
    ]
    if use_phase:
        # Gait phase clock, same reference as the phase_gait reward (see above).
        phi = (step * CONTROL_DT / GAIT_CYCLE_TIME
               + np.array(GAIT_PHASE_OFFSETS)) % 1.0
        obs_parts.append(np.concatenate([np.sin(2.0 * np.pi * phi),
                                         np.cos(2.0 * np.pi * phi)]))
    obs = np.concatenate(obs_parts)
    return np.clip(obs, -100.0, 100.0)


def _compute_torques_policy(
    action: np.ndarray,
    joint_pos: np.ndarray,
    joint_vel: np.ndarray,
    default_pose: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    kp: np.ndarray,
    kd: np.ndarray,
    torque_limit: np.ndarray,
) -> np.ndarray:
    """PD control law in POLICY order (control.py build_targets + compute_pd_torques).

    Legs : q_target = clip(default + action_scale*a, limits); tau = kp(q_target-q) - kd*qd
    Wheels: w_target = wheel_vel_scale*a;                      tau = kd*(w_target - qd)   (kp=0)
    """
    # Position target (legs); for wheels this term is inert because kp=0.
    pos_target = np.clip(default_pose + action * ACTION_SCALE, lower, upper)
    # Velocity target: wheels get wheel_vel_scale*a, legs get 0.
    vel_target = np.zeros(NUM_JOINTS)
    vel_target[NUM_LEG_JOINTS:] = action[NUM_LEG_JOINTS:] * WHEEL_VEL_SCALE

    torques = kp * (pos_target - joint_pos) + kd * (vel_target - joint_vel)
    return np.clip(torques, -torque_limit, torque_limit)


def _resolve_checkpoint(checkpoint: str | None) -> str:
    if not checkpoint:
        raise SystemExit("Please pass --checkpoint <path to .pkl>")
    path = checkpoint if os.path.isabs(checkpoint) else os.path.abspath(checkpoint)
    if not os.path.exists(path):
        raise FileNotFoundError(f"Checkpoint not found: {path}")
    return path


def _iteration_num(path: str) -> int:
    """Extract N from a '.../iteration_N.pkl' path (-1 if it doesn't match)."""
    stem = os.path.basename(path)
    if stem.startswith("iteration_") and stem.endswith(".pkl"):
        try:
            return int(stem[len("iteration_"):-len(".pkl")])
        except ValueError:
            return -1
    return -1


def _newest_checkpoint(watch_dir: str) -> tuple[str | None, int]:
    """Return (path, iteration) of the highest-iteration checkpoint, or (None,-1)."""
    import glob
    best_path, best_iter = None, -1
    for p in glob.glob(os.path.join(watch_dir, "iteration_*.pkl")):
        it = _iteration_num(p)
        if it > best_iter:
            best_path, best_iter = p, it
    return best_path, best_iter


def _load_params(path: str):
    """Load the 'params' pytree from a checkpoint pickle."""
    with open(path, "rb") as f:
        return pickle.load(f)["params"]


def _reset_pose(
    mj_model: mujoco.MjModel,
    mj_data: mujoco.MjData,
    indexer: "JointIndexer",
    default_pose: np.ndarray,
) -> None:
    """Torso upright at nominal height, legs at NOMINAL, everything else zero."""
    mujoco.mj_resetData(mj_model, mj_data)
    mj_data.qpos[0:3] = [0.0, 0.0, NOMINAL_TORSO_HEIGHT]
    mj_data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]                  # identity quat (w,x,y,z)
    indexer.set_default_pose(mj_data, default_pose)
    mujoco.mj_forward(mj_model, mj_data)


class GamepadReader:
    """Reads velocity commands from a gamepad via pygame's SDL joystick backend.

    Default mapping targets an Xbox / XInput-style controller:
        left-stick vertical  (axis 1)  -> forward/back  (cmd_vel_x)   up = +x
        right-stick horizontal(axis 3)  -> yaw           (cmd_vel_yaw) left = +yaw

    SDL reports stick up / left as NEGATIVE, so both axes are negated to make
    "push up = go forward" and "push left = turn left (CCW, +yaw)".  Axis indices
    differ across controllers/drivers -- run ``--gamepad_debug`` to find yours,
    then override with ``--axis_x`` / ``--axis_yaw``.
    """

    def __init__(self, axis_x: int, axis_yaw: int, deadzone: float, index: int = 0):
        import pygame  # local import: avoids the noisy banner unless gamepad used
        self._pg = pygame
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError(
                "No gamepad detected. Plug one in and retry, or drop --gamepad."
            )
        self._js = pygame.joystick.Joystick(index)
        self._js.init()
        self.name = self._js.get_name()
        self.num_axes = self._js.get_numaxes()
        self.axis_x = axis_x
        self.axis_yaw = axis_yaw
        self.deadzone = deadzone

    def _axis(self, idx: int) -> float:
        if idx < 0 or idx >= self.num_axes:
            return 0.0
        v = self._js.get_axis(idx)
        return 0.0 if abs(v) < self.deadzone else v

    def read(self, max_vx: float, max_vyaw: float) -> np.ndarray:
        """Poll the pad and return command = [cmd_vel_x, 0, cmd_vel_yaw]."""
        self._pg.event.pump()
        vx = -self._axis(self.axis_x) * max_vx          # stick up -> +x
        vyaw = -self._axis(self.axis_yaw) * max_vyaw     # stick left -> +yaw (CCW)
        return np.array([vx, 0.0, vyaw])                 # lin_vel_y stays locked at 0


def _gamepad_debug_loop() -> None:
    """Print live axis / button values so the user can identify their mapping."""
    try:
        import pygame
    except ImportError:
        raise SystemExit("pygame not installed: pip install pygame")
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise SystemExit("  ! No gamepad detected. Plug one in and retry.")
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"  Gamepad: {js.get_name()!r}  "
          f"axes={js.get_numaxes()} buttons={js.get_numbuttons()}")
    print("  Move the sticks / press buttons to find your axis numbers. "
          "Ctrl+C to quit.\n")
    try:
        while True:
            pygame.event.pump()
            axes = " ".join(f"{js.get_axis(i):+.2f}" for i in range(js.get_numaxes()))
            pressed = [i for i in range(js.get_numbuttons()) if js.get_button(i)]
            print(f"\r  axes=[{axes}]  pressed={pressed}      ", end="", flush=True)
            time.sleep(0.08)
    except KeyboardInterrupt:
        print("\n  done.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Sim2sim dogurdf policy in native MuJoCo")
    parser.add_argument("--exp", default="dogurdf_velocity")
    parser.add_argument("--checkpoint", type=str, default=None)
    parser.add_argument("--cmd_vel_x", type=float, default=1.0)
    parser.add_argument("--cmd_vel_y", type=float, default=0.0)
    parser.add_argument("--cmd_vel_yaw", type=float, default=0.0)
    parser.add_argument("--episode_length", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--record", type=str, default=None,
                        help="Export per-step trajectory CSV (qrel/vel/action/pgr, "
                             "POLICY order, 50Hz) for sim2real comparison.")
    parser.add_argument("--save_video", action="store_true")
    parser.add_argument("--video_path", type=str, default="/tmp/sim2sim.mp4")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    # ---- Real-time viewer + gamepad --------------------------------------
    parser.add_argument("--viewer", action="store_true",
                        help="Open a real-time interactive mujoco.viewer window.")
    parser.add_argument("--gamepad", action="store_true",
                        help="Drive cmd_vel from a gamepad (implies --viewer).")
    parser.add_argument("--gamepad_debug", action="store_true",
                        help="Print live gamepad axis/button values, then exit.")
    parser.add_argument("--axis_x", type=int, default=1,
                        help="Gamepad axis for forward/back (default: left-stick Y).")
    parser.add_argument("--axis_yaw", type=int, default=3,
                        help="Gamepad axis for yaw (default: right-stick X).")
    parser.add_argument("--deadzone", type=float, default=0.12,
                        help="Stick deadzone; values below are treated as 0.")
    parser.add_argument("--max_vx", type=float, default=1.0,
                        help="cmd_vel_x at full stick (training range was +/-1.0).")
    parser.add_argument("--max_vyaw", type=float, default=1.0,
                        help="cmd_vel_yaw at full stick (training range was +/-1.0).")
    # ---- Live-watch: hot-reload newest checkpoint during training ---------
    parser.add_argument("--watch_dir", type=str, default=None,
                        help="Poll this dir for newer iteration_*.pkl and hot-reload "
                             "the policy live (implies --viewer). Point it at the "
                             "active training checkpoints_<ts>/ dir.")
    parser.add_argument("--watch_interval", type=float, default=5.0,
                        help="Seconds between --watch_dir checkpoint polls.")
    parser.add_argument("--no_follow", action="store_true",
                        help="Disable the follow-cam; use a fixed free camera.")
    parser.add_argument("--follow_dist", type=float, default=3.0,
                        help="Follow-cam distance from the torso (m). Smaller = closer.")
    parser.add_argument("--cam_elevation", type=float, default=-20.0,
                        help="Follow-cam elevation angle in deg (video path); "
                             "-90 is top-down, 0 is level.")
    parser.add_argument("--cam_azimuth", type=float, default=90.0,
                        help="Follow-cam azimuth angle in deg (video path).")
    args = parser.parse_args()

    # Gamepad debug is a standalone utility: identify axes, then exit.
    if args.gamepad_debug:
        _gamepad_debug_loop()
        return

    print("=" * 80)
    print(f"Sim2Sim (native MuJoCo) - Experiment: {args.exp}")
    print(f"  JAX backend: {jax.default_backend()}")
    print("=" * 80)

    # ---- Experiment spec (for model path + network hyperparams) ------------
    spec = get_experiment(args.exp)
    ppo_cfg = spec.build_ppo_config()
    xml_path = str(spec.robot.asset_path)
    print(f"  MJCF: {xml_path}")

    # ---- Native MuJoCo model ----------------------------------------------
    mj_model = mujoco.MjModel.from_xml_path(xml_path)
    mj_data = mujoco.MjData(mj_model)
    print(f"  nq={mj_model.nq}  nv={mj_model.nv}  timestep={mj_model.opt.timestep}")
    if abs(mj_model.opt.timestep - SIM_DT) > 1e-9:
        print(f"  ! WARNING: MJCF timestep {mj_model.opt.timestep} != SIM_DT {SIM_DT}")

    indexer = JointIndexer(mj_model)
    default_pose = _default_pose_policy()
    lower, upper = _policy_joint_limits()

    # Per-joint gains / torque limits in POLICY order.
    kp = np.array([LEG_KP] * NUM_LEG_JOINTS + [WHEEL_KP] * NUM_WHEELS)
    kd = np.array([LEG_KD] * NUM_LEG_JOINTS + [WHEEL_KD] * NUM_WHEELS)
    torque_limit = np.array(
        [LEG_TORQUE_LIMIT] * NUM_LEG_JOINTS + [WHEEL_TORQUE_LIMIT] * NUM_WHEELS
    )

    # ---- Initial pose: torso upright at nominal height, legs at NOMINAL ----
    _reset_pose(mj_model, mj_data, indexer, default_pose)

    # ---- Network + weights -------------------------------------------------
    network = ActorCriticMLP(
        action_size=NUM_JOINTS,
        actor_hidden_dims=tuple(ppo_cfg.network.policy_hidden_layer_sizes),
        critic_hidden_dims=tuple(ppo_cfg.network.value_hidden_layer_sizes),
        init_noise_std=ppo_cfg.network.init_noise_std,
        activation=ppo_cfg.network.activation,
    )
    # In --watch_dir mode we may start from the newest checkpoint in that dir;
    # otherwise use --checkpoint.  ``current`` holds the live params + iteration
    # so the watcher can swap them under the running control loop.
    if args.watch_dir:
        init_path, init_iter = _newest_checkpoint(args.watch_dir)
        if init_path is None:
            # Training may not have written its first checkpoint yet
            # (save_interval). Fall back to --checkpoint for a starting policy
            # if given, else wait for the first checkpoint to appear.
            if args.checkpoint:
                init_path = _resolve_checkpoint(args.checkpoint)
                init_iter = _iteration_num(init_path)
                print(f"  --watch_dir empty; starting from --checkpoint until "
                      f"the first one appears.")
            else:
                print(f"  --watch_dir has no iteration_*.pkl yet; waiting for the "
                      f"first checkpoint (poll every {args.watch_interval:.0f}s)...")
                while init_path is None:
                    time.sleep(args.watch_interval)
                    init_path, init_iter = _newest_checkpoint(args.watch_dir)
        ckpt_path = init_path
    else:
        ckpt_path = _resolve_checkpoint(args.checkpoint)
        init_iter = _iteration_num(ckpt_path)
    current = {"params": _load_params(ckpt_path), "iter": init_iter}
    print(f"  Loaded checkpoint: {ckpt_path}  (iteration {init_iter})")

    # Auto-detect the actor input dimension from the loaded weights so both
    # the legacy 56-D runs (pre gait-phase clock) and the new 64-D runs
    # (with gait-phase clock) can be driven with the matching observation.
    actor_dim = int(current["params"]["params"]["actor_mlp_0"]["kernel"].shape[0])
    use_phase = actor_dim == 64
    print(f"  actor obs dim: {actor_dim}"
          f"  ({'with gait-phase clock' if use_phase else 'legacy, no gait-phase'} )")

    @jax.jit
    def policy_mean(p, obs):
        return network.apply(p, obs, method=network.act)

    # Warm up so any shape error surfaces before the loop.  Build a real
    # observation (width follows the checkpoint's actor input dim, 56 or 64).
    warmup_obs = _build_observation(
        mj_data, indexer, default_pose,
        np.zeros(NUM_JOINTS), np.zeros(3), step=0, use_phase=use_phase,
    )
    _ = policy_mean(current["params"], jnp.asarray(warmup_obs[None]))

    last_action = np.zeros(NUM_JOINTS)

    # One 50 Hz control step: build obs, run policy, step physics DECIMATION
    # times with the SAME action target (matches mjx_step_custom_pd).  Returns
    # (obs, action) and mutates last_action via the caller's assignment.  Reads
    # ``current["params"]`` fresh each step so --watch_dir hot-reloads take
    # effect immediately.
    def control_step(command: np.ndarray, last_action: np.ndarray, step: int = 0) -> tuple[np.ndarray, np.ndarray]:
        obs = _build_observation(mj_data, indexer, default_pose, last_action, command,
                                 step=step, use_phase=use_phase)
        action = np.asarray(policy_mean(current["params"], jnp.asarray(obs[None]))[0])
        action = np.where(np.isfinite(action), action, 0.0)
        for _ in range(DECIMATION):
            joint_pos = indexer.joint_pos(mj_data)
            joint_vel = indexer.joint_vel(mj_data)
            torques = _compute_torques_policy(
                action, joint_pos, joint_vel, default_pose,
                lower, upper, kp, kd, torque_limit,
            )
            mj_data.qfrc_applied[:] = 0.0
            mj_data.qfrc_applied[indexer.dof_adr] = torques
            mujoco.mj_step(mj_model, mj_data)
        return obs, action

    # ---- Real-time interactive path (viewer, optionally gamepad-driven) ----
    # 统一记录初始化：headless 与 viewer/gamepad 都支持 --record（sim2real 对比）
    record_f, record_w = (None, None)
    if args.record:
        record_f, record_w = _open_record(args.record)

    if args.viewer or args.gamepad or args.watch_dir:
        _run_viewer(args, mj_model, mj_data, indexer, default_pose,
                    control_step, current, record_f, record_w)
        if record_f is not None:
            record_f.close()
        return

    # ---- Headless / video path (fixed command, fall detection) -------------
    command = np.array([args.cmd_vel_x, args.cmd_vel_y, args.cmd_vel_yaw])
    print(f"  command: lin_x={args.cmd_vel_x} lin_y={args.cmd_vel_y} "
          f"ang_yaw={args.cmd_vel_yaw}")

    renderer = None
    frames: list[np.ndarray] = []
    video_cam: Any = -1
    if args.save_video:
        renderer = mujoco.Renderer(mj_model, height=args.height, width=args.width)
        # Follow-cam that tracks the torso (same as the interactive viewer),
        # so the robot stays centred and close instead of the default top-down
        # free camera.  Falls back to -1 if the torso body isn't found.
        torso_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        if not args.no_follow and torso_id >= 0:
            video_cam = mujoco.MjvCamera()
            video_cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            video_cam.trackbodyid = torso_id
            video_cam.distance = args.follow_dist
            video_cam.elevation = args.cam_elevation
            video_cam.azimuth = args.cam_azimuth

    print("-" * 60)
    print(f"Running {args.episode_length} control steps (Ctrl+C to stop)")
    print("-" * 60)
    render_every = max(int(round((1.0 / CONTROL_DT) / 50.0)), 1)  # ~50 fps video
    start = time.perf_counter()
    fell = False

    # record_f/record_w 已在上方统一初始化（_open_record），headless 复用
    try:
        for step in range(args.episode_length):
            obs, last_action = control_step(command, last_action, step)

            if record_w is not None:
                _record_row(record_w, mj_data, indexer, obs, last_action, command, step)

            if renderer is not None and (step % render_every == 0):
                renderer.update_scene(mj_data, camera=video_cam)
                frames.append(renderer.render())

            # Fall detection: projected-gravity z tips past ~70 deg.
            proj_g = obs[6:9]
            if proj_g[2] > -0.34 and step > 5:
                print(f"  ! step {step}: robot tipped over (proj_grav_z={proj_g[2]:.2f})")
                fell = True
                break
    except KeyboardInterrupt:
        print("\n  Interrupted.")

    if record_f is not None:
        record_f.close()

    elapsed = time.perf_counter() - start
    steps_done = step + 1
    print("-" * 60)
    print(f"  Ran {steps_done} steps in {elapsed:.2f}s "
          f"({steps_done / elapsed:.0f} ctrl-steps/s)")
    print(f"  Final torso pos: {mj_data.qpos[0:3]}")
    print(f"  Result: {'FELL' if fell else 'survived to end'}")

    if renderer is not None and frames:
        renderer.close()
        _write_video(frames, args.video_path)


# ---- 轨迹记录（sim2real 对比，2026-08-30）----
# 统一字段：wall_ms(系统时间戳，与真机对齐) + step + cmd +
#           qpos_00..15(16 编码器绝对位置, POLICY order) +
#           qrel_0..11(腿相对位) + vel_00..15(16 关节速) + act_00..15 + pgr_x/y/z
def _open_record(path):
    import csv
    fields = (["wall_ms", "step", "cmd_vx", "cmd_vy", "cmd_wz"] +
              [f"qpos_{i:02d}" for i in range(16)] +
              [f"qrel_{i}" for i in range(12)] +
              [f"vel_{i:02d}" for i in range(16)] +
              [f"act_{i:02d}" for i in range(16)] +
              ["pgr_x", "pgr_y", "pgr_z"])
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(fields)
    return f, w


def _record_row(w, mj_data, indexer, obs, action, command, step):
    import time
    joint_pos = indexer.joint_pos(mj_data)   # 16 编码器绝对位置 (POLICY order, rad)
    w.writerow([f"{time.time()*1000:.0f}", step, command[0], command[1], command[2]] +
               [f"{x:.5f}" for x in joint_pos] +
               [f"{x:.4f}" for x in obs[9:21]] +
               [f"{x:.4f}" for x in obs[21:37]] +
               [f"{x:.4f}" for x in action] +
               [f"{x:.4f}" for x in obs[6:9]])


def _run_viewer(
    args: argparse.Namespace,
    mj_model: mujoco.MjModel,
    mj_data: mujoco.MjData,
    indexer: "JointIndexer",
    default_pose: np.ndarray,
    control_step,
    current: dict,
    record_f=None,
    record_w=None,
) -> None:
    """Real-time interactive loop: passive viewer, 50 Hz wall-clock pacing.

    Command source is either a live gamepad (``--gamepad``) or the fixed
    ``--cmd_vel_*`` CLI values.  On a fall the robot auto-resets so you can
    keep driving without restarting the process.

    With ``--watch_dir`` the loop polls that directory every
    ``--watch_interval`` seconds and hot-swaps ``current["params"]`` whenever a
    higher-iteration checkpoint appears -- so you watch the policy improve (or
    keep falling) live as training writes new checkpoints.  The reload is a
    plain pickle read on the CPU JAX backend; it never touches the training
    GPU.
    """
    import mujoco.viewer

    pad = None
    if args.gamepad:
        pad = GamepadReader(args.axis_x, args.axis_yaw, args.deadzone)
        print(f"  Gamepad: {pad.name!r}  ({pad.num_axes} axes)")
        print(f"    left-stick Y (axis {args.axis_x}) -> forward/back  "
              f"[+/-{args.max_vx} m/s]")
        print(f"    right-stick X (axis {args.axis_yaw}) -> turn        "
              f"[+/-{args.max_vyaw} rad/s]")
        print("    (override axes with --axis_x/--axis_yaw; "
              "find them via --gamepad_debug)")
    else:
        print(f"  Fixed command: lin_x={args.cmd_vel_x} ang_yaw={args.cmd_vel_yaw} "
              "(pass --gamepad to drive live)")

    if args.watch_dir:
        print(f"  Watching {args.watch_dir} every {args.watch_interval:.0f}s "
              f"for newer checkpoints (currently iteration {current['iter']}).")

    last_action = np.zeros(NUM_JOINTS)
    fixed_cmd = np.array([args.cmd_vel_x, args.cmd_vel_y, args.cmd_vel_yaw])
    sim_step = 0  # episode step counter for the gait-phase observation clock

    print("-" * 60)
    print("  Real-time viewer running. Close the window or Ctrl+C to stop.")
    print("-" * 60)

    last_poll = time.perf_counter()

    with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
        # Follow-cam: track the torso so it stays centred as it drives around.
        if not args.no_follow:
            torso_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, "torso")
            if torso_id >= 0:
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                viewer.cam.trackbodyid = torso_id
                viewer.cam.distance = args.follow_dist
                viewer.cam.elevation = -20.0
                viewer.cam.azimuth = 90.0

        while viewer.is_running():
            try:
                tic = time.perf_counter()

                # Poll for a newer checkpoint and hot-swap params in place.
                if args.watch_dir and (tic - last_poll) >= args.watch_interval:
                    last_poll = tic
                    path, it = _newest_checkpoint(args.watch_dir)
                    if path is not None and it > current["iter"]:
                        try:
                            current["params"] = _load_params(path)
                            current["iter"] = it
                            print(f"  -> hot-loaded iteration {it}")
                        except (EOFError, pickle.UnpicklingError, OSError):
                            # Checkpoint still being written; retry next poll.
                            pass

                command = pad.read(args.max_vx, args.max_vyaw) if pad is not None else fixed_cmd
                obs, last_action = control_step(command, last_action, sim_step)

                if record_w is not None:
                    _record_row(record_w, mj_data, indexer, obs, last_action, command, sim_step)

                sim_step += 1

                # Auto-reset on fall so the session keeps going.
                proj_g = obs[6:9]
                if proj_g[2] > -0.34:
                    print(f"  ! tipped over (proj_grav_z={proj_g[2]:.2f}) -> resetting")
                    _reset_pose(mj_model, mj_data, indexer, default_pose)
                    last_action = np.zeros(NUM_JOINTS)
                    sim_step = 0

                viewer.sync()

                # Pace to real time: one control step == CONTROL_DT seconds.
                dt = CONTROL_DT - (time.perf_counter() - tic)
                if dt > 0:
                    time.sleep(dt)
            except KeyboardInterrupt:
                print("\n  Interrupted by user.")
                break
    print("  Viewer closed.")


def _write_video(frames: list[np.ndarray], path: str) -> None:
    fps = 50.0
    # ffmpeg won't create parent dirs; make sure they exist.
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)
    if HAS_MEDIAPY:
        media.write_video(path, frames, fps=fps)
        print(f"  Saved video ({len(frames)} frames) -> {path}")
        return
    try:
        import imageio
        imageio.mimsave(path, frames, fps=fps)
        print(f"  Saved video ({len(frames)} frames) -> {path}")
    except ImportError:
        print("  ! Neither mediapy nor imageio available; skipping video. "
              "Install with: pip install imageio[ffmpeg]")


if __name__ == "__main__":
    main()
