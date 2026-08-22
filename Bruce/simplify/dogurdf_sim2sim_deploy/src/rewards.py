"""Velocity task reward registry and extracted reward terms."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping

import jax
import jax.numpy as jp
from brax import math


RewardFn = Callable[["VelocityRewardContext", Any], jax.Array]

# Diagonal wheel pairs for step-to-turn gait, in foot_body_names order
# (FL, FR, RL, RR).  Diagonal A = FL + RR, diagonal B = FR + RL -- the two
# trot support pairs.  A diagonal support line passes near the CoM, so lifting
# one diagonal and rotating on the other is dynamically balanceable (unlike
# front/rear-axle pairs, whose support line is 0.33 m from the CoM).
DIAG_A_IDX = (0, 3)
DIAG_B_IDX = (1, 2)

# Position-joint indices in policy order (FL, FR, RL, RR) x (hip, thigh, calf).
# hip = abduction/roll (M20 "hipx"), thigh = pitch (M20 "hipy"), calf (M20 "knee").
HIP_IDX = (0, 3, 6, 9)
THIGH_IDX = (1, 4, 7, 10)
CALF_IDX = (2, 5, 8, 11)
# Per-leg joint slices for the diagonal joint_mirror term.
FL_IDX = (0, 1, 2)
FR_IDX = (3, 4, 5)
RL_IDX = (6, 7, 8)
RR_IDX = (9, 10, 11)

# Nominal wheel-centre offsets in the BODY frame, in (FL, FR, RL, RR) order,
# measured at the default stance (torso at 0.40 m).  These are the centre of
# the phase foot trajectory.
NOMINAL_FOOT_OFFSETS = (
    (0.3020, 0.2558, -0.2863),
    (0.3020, -0.2558, -0.2863),
    (-0.3510, 0.2558, -0.2863),
    (-0.3510, -0.2558, -0.2863),
)


@dataclass(frozen=True)
class VelocityRewardContext:
    """Inputs shared by the first registry-backed velocity reward terms."""

    command: jax.Array
    action: jax.Array
    last_action: jax.Array
    joint_pos: jax.Array
    default_pose: jax.Array
    soft_joint_lower: jax.Array
    soft_joint_upper: jax.Array
    base_lin_vel: jax.Array
    base_ang_vel: jax.Array
    projected_gravity: jax.Array
    data: Any | None = None
    # Base orientation quaternion (w,x,y,z), world->rotate.  Lets a reward
    # rotate world-frame foot velocities into the body frame (feet_slide_turn).
    base_quat: jax.Array | None = None
    feet_body_ids: jax.Array | None = None
    feet_air_time: jax.Array | None = None
    first_contact: jax.Array | None = None
    # Air-time duration of the swing that just ended, captured BEFORE
    # ``feet_air_time`` is zeroed on landing (see feet.py).  Non-zero exactly on
    # the touchdown frame, so ``last_air_time * first_contact`` credits the
    # completed swing -- which ``feet_air_time * first_contact`` cannot (it is
    # always 0 at landing).  Used by feet_air_time_turn.
    last_air_time: jax.Array | None = None
    base_height: jax.Array | None = None
    # All-joint velocities in POLICY order (position joints then wheels); wheels
    # are the tail [num_position_joints:]. Used by stand_still to damp wheel spin
    # under a zero command. Optional so other constructors still work.
    joint_vel: jax.Array | None = None
    # Number of leading entries in ``joint_pos`` that are position-controlled.
    # Posture terms only apply to those; velocity-controlled wheels have no
    # meaningful nominal angle and no joint limits.
    num_position_joints: int | None = None


@dataclass(frozen=True)
class RewardTermSpec:
    name: str
    description: str
    fn: RewardFn | None = None

    @property
    def extracted(self) -> bool:
        return self.fn is not None


def tracking_lin_vel(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Track commanded planar velocity and damp vertical base velocity."""
    tracking_sigma_lin = getattr(reward_config, "tracking_sigma_lin", 0.5)
    xy_error = jp.sum(jp.square(context.command[:2] - context.base_lin_vel[:2]))
    z_error = jp.square(context.base_lin_vel[2])
    return jp.exp(-(xy_error + z_error) / (tracking_sigma_lin**2))


def tracking_ang_vel(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Track commanded yaw velocity only.

    Roll/pitch stability is handled by the dedicated ``body_ang_vel`` term so
    the two objectives can be weighted independently (we want yaw to follow the
    command while roll/pitch are held as close to zero as possible).
    """
    tracking_sigma_ang = getattr(reward_config, "tracking_sigma_ang", 0.707)
    z_error = jp.square(context.command[2] - context.base_ang_vel[2])
    return jp.exp(-z_error / (tracking_sigma_ang**2))


def body_ang_vel(context: VelocityRewardContext, _: Any) -> jax.Array:
    """Penalize body-frame roll/pitch angular velocity to keep the torso level.

    During a step-to-turn the legs should do the turning while the body stays
    flat; any rocking about the roll/pitch axes is penalized (yaw is left free
    for the tracking term).  ``base_ang_vel`` is body-frame, so ``[:2]`` are the
    roll/pitch rates.
    """
    return jp.sum(jp.square(context.base_ang_vel[:2]))


def lateral_vel(context: VelocityRewardContext, _: Any) -> jax.Array:
    """Penalize body-frame lateral velocity drift."""
    return jp.square(context.base_lin_vel[1])


def yaw_rate_error(context: VelocityRewardContext, _: Any) -> jax.Array:
    """Penalize direct yaw-rate tracking error."""
    return jp.square(context.command[2] - context.base_ang_vel[2])


def upright(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Keep projected gravity close to the upright axis."""
    upright_sigma = getattr(reward_config, "upright_sigma", 0.447)
    xy_squared = jp.sum(jp.square(context.projected_gravity[:2]))
    return jp.exp(-xy_squared / (upright_sigma**2))


def joint_mirror(context: VelocityRewardContext, _: Any) -> jax.Array:
    """Penalize diagonal-leg posture asymmetry (M20 ``joint_mirror``).

    A left/right-symmetric trot has the two diagonal legs (FL<->RR, FR<->RL)
    holding mirror-image joint angles.  Penalize the squared angle difference
    within each diagonal pair over (hip, thigh, calf), summed and halved, then
    scale by ``clamp(-proj_g_z, 0, 0.7)/0.7`` so a tipping/fallen robot (whose
    projected gravity z is no longer ~-1) is not penalized.  This is M20's
    native cure for the left>>right leg-lift asymmetry, complementary to the
    training-pipeline mirror augmentation.
    """
    q = context.joint_pos
    fl, fr = jp.array(FL_IDX), jp.array(FR_IDX)
    rl, rr = jp.array(RL_IDX), jp.array(RR_IDX)
    diff = jp.sum(jp.square(q[fl] - q[rr])) + jp.sum(jp.square(q[fr] - q[rl]))
    diff = diff / 2.0
    upright_factor = jp.clip(-context.projected_gravity[2], 0.0, 0.7) / 0.7
    return diff * upright_factor


def bad_orientation(context: VelocityRewardContext, _: Any) -> jax.Array:
    """1.0 when the base orientation is bad (M20 ``bad_orientation_penalty``).

    Fires when projected gravity points even slightly up (``proj_g_z > 0``, i.e.
    past horizontal) or the planar tilt exceeds 0.7.  A hard indicator penalty;
    at M20's -1000 scale it dominates the reward, strongly discouraging tipping.
    """
    g = context.projected_gravity
    tipped = (g[2] > 0.0) | (jp.linalg.norm(g[:2]) > 0.7)
    return tipped.astype(jp.float32)


def _joint_pos_penalty(
    context: VelocityRewardContext,
    joint_idx: tuple[int, ...],
    reward_config: Any,
) -> jax.Array:
    """M20 ``joint_pos_penalty``: L2 posture error, amplified when near-still.

    ``running = ||q[group] - default[group]||`` (L2 norm, not squared sum).
    When the command is near zero AND the base is barely moving, multiply by
    ``stand_still_scale`` so a resting robot is held tightly to its nominal
    posture.  Matches M20 base params (stand_still_scale=5, velocity_threshold=
    0.5, command_threshold=0.1).  M20's projected-gravity clamp is commented out
    there, so no orientation gating here either.
    """
    idx = jp.array(joint_idx)
    err = context.joint_pos[idx] - context.default_pose[idx]
    running = jp.linalg.norm(err)

    scale = getattr(reward_config, "pose_penalty_stand_still_scale", 5.0)
    cmd_thr = getattr(reward_config, "pose_penalty_command_threshold", 0.1)
    vel_thr = getattr(reward_config, "pose_penalty_velocity_threshold", 0.5)
    cmd = jp.linalg.norm(context.command)
    body_vel = jp.linalg.norm(context.base_lin_vel[:2])
    near_still = (cmd <= cmd_thr) & (body_vel <= vel_thr)
    return jp.where(near_still, scale * running, running)


def pose_hipx(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Hip (abduction/roll) posture penalty, always active (M20 hipx, -3.0)."""
    return _joint_pos_penalty(context, HIP_IDX, reward_config)


def pose_hipy(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Thigh (pitch) posture penalty, relaxed during pure yaw (M20 hipy, -1.5).

    Under a pure-yaw step-to-turn command the thigh must bend to lift a wheel,
    so the penalty is relaxed (not fully disabled) there -- keeps the leg from
    flailing while still allowing enough bend to step-turn.
    """
    relax = getattr(reward_config, "pose_turn_relax", 0.3)
    weight = 1.0 - (1.0 - relax) * _pure_yaw_gate(context, reward_config)
    return _joint_pos_penalty(context, THIGH_IDX, reward_config) * weight


def pose_knee(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Calf posture penalty, relaxed during pure yaw (M20 knee, -0.75)."""
    relax = getattr(reward_config, "pose_turn_relax", 0.3)
    weight = 1.0 - (1.0 - relax) * _pure_yaw_gate(context, reward_config)
    return _joint_pos_penalty(context, CALF_IDX, reward_config) * weight


def _position_joint_slice(context: VelocityRewardContext) -> slice:
    """Slice selecting the position-controlled head of the policy vector."""
    n = context.num_position_joints
    return slice(None) if n is None else slice(0, n)


def dof_pos_limits(context: VelocityRewardContext, _: Any) -> jax.Array:
    """Penalize excursions beyond the configured soft joint bounds.

    Wheels are excluded: their joints are unbounded, so a limit penalty is
    meaningless (and their sentinel limits would swamp the sum).
    """
    sel = _position_joint_slice(context)
    joint_pos = context.joint_pos[sel]
    out_of_limits = -jp.clip(joint_pos - context.soft_joint_lower[sel], max=0.0)
    out_of_limits += jp.clip(joint_pos - context.soft_joint_upper[sel], min=0.0)
    return jp.sum(out_of_limits)


def base_height(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Reward holding the torso near its nominal ride height."""
    if context.base_height is None:
        return jp.array(0.0)
    target = getattr(reward_config, "base_height_target", 0.35)
    sigma = getattr(reward_config, "base_height_sigma", 0.05)
    return jp.exp(-jp.square(context.base_height - target) / (sigma**2))


def action_rate(context: VelocityRewardContext, _: Any) -> jax.Array:
    """Penalize policy action changes from the previous step."""
    return jp.sum(jp.square(context.action - context.last_action))


def stand_still(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Penalize actual base motion when the command is (near) zero.

    The tracking reward can't hold a true stop here: the policy never observes
    its own linear velocity (``odom_data`` is fed as zeros), so it has no
    closed-loop feedback to converge to rest.  Meanwhile every gait-shaping
    term is gated off below ``command_threshold``, leaving the zero-command
    regime almost unconstrained -- the robot drifts (mostly the wheels keep
    rolling).  This term fills that gap: when the command is ~zero, penalize
    the squared base velocity (planar drift + vertical bounce).  It is exactly
    zero for any non-zero command, so it does not directly touch the learned
    locomotion gaits.
    """
    command_threshold = getattr(reward_config, "command_threshold", 0.05)
    total_command = jp.linalg.norm(context.command[:2]) + jp.abs(context.command[2])
    is_zero_cmd = (total_command < command_threshold).astype(jp.float32)

    # Deadzone: only penalize steady-state motion, not the physical braking
    # transient. The policy cannot observe its own speed (base_lin_vel is fed
    # as 0), so when the command snaps to zero it cannot actively brake; a
    # deadzone leaves the decel ramp unpenalized and only charges a sustained
    # creep (which is what "stand still" is really about).
    speed_deadzone = getattr(reward_config, "stand_still_speed_deadzone", 0.3)
    base_speed = jp.linalg.norm(context.base_lin_vel[:3])
    base_pen = jp.square(jp.clip(base_speed - speed_deadzone, 0.0, None))

    wheel_pen = jp.array(0.0)
    if context.joint_vel is not None and context.num_position_joints is not None:
        wheel_deadzone = getattr(reward_config, "stand_still_wheel_deadzone", 1.0)
        wheel_vel = context.joint_vel[context.num_position_joints:]
        wheel_speed = jp.linalg.norm(wheel_vel)
        wheel_pen = jp.square(jp.clip(wheel_speed - wheel_deadzone, 0.0, None))

    wheel_weight = getattr(reward_config, "stand_still_wheel_weight", 0.05)
    return (base_pen + wheel_weight * wheel_pen) * is_zero_cmd


def wheel_slip_longitudinal(
    context: VelocityRewardContext, reward_config: Any,
) -> jax.Array:
    """Penalize longitudinal wheel slip (spin-up / lock-up) under any command.

    For a rolling wheel the surface speed ``omega * r`` should match the body's
    forward speed.  The mismatch ``omega * r - v_forward`` is the longitudinal
    slip: positive means the wheel spins faster than the robot moves (burnout),
    negative means it drags (lock-up).  Both waste energy and grind the tyre,
    and both are controllable (the wheel-velocity command sets omega), so this
    is penalized globally -- for every command, not just turning.

    Sign verified empirically: a positive wheel joint velocity drives a positive
    base x-velocity, so ``omega * r - base_lin_vel[x]`` is the correct slip.
    Lateral scrub is deliberately excluded: it is geometrically forced for this
    skid-steer platform and cannot be removed by any wheel-speed choice.
    """
    if context.joint_vel is None or context.num_position_joints is None:
        return jp.array(0.0)

    radius = getattr(reward_config, "wheel_radius", 0.113)
    wheel_vel = context.joint_vel[context.num_position_joints:]
    v_forward = context.base_lin_vel[0]
    slip = wheel_vel * radius - v_forward
    return jp.sum(jp.square(slip))


def _pure_yaw_gate(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """1.0 when the command is a pure in-place yaw, else 0.0.

    "Pure yaw" = small planar command AND significant yaw command, matching
    M20's ``feet_slide_ang_z_cmd`` gate.  This is the regime where the robot
    should turn by lifting a diagonal pair of wheels and stepping around,
    rather than scrubbing all four wheels sideways.
    """
    lin_thr = getattr(reward_config, "turn_lin_threshold", 0.1)
    ang_thr = getattr(reward_config, "turn_ang_threshold", 0.1)
    lin = jp.linalg.norm(context.command[:2])
    yaw = jp.abs(context.command[2])
    return ((lin < lin_thr) & (yaw > ang_thr)).astype(jp.float32)


def _yaw_turn_weight(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Soft 0..1 turn-gait gain: the pure-yaw gate scaled by |cmd_yaw|.

    The hard gate alone makes the step-to-turn gait all-or-nothing: a tiny yaw
    command gets no stepping incentive, while a large one gets the full (often
    violent) gait.  Scaling by |cmd_yaw| / ``turn_yaw_scale`` makes the stepping
    incentive proportional to the demanded turn rate, so the policy learns to
    modulate its step size instead of always stepping at maximum.
    """
    yaw_scale = getattr(reward_config, "turn_yaw_scale", 1.0)
    yaw = jp.abs(context.command[2])
    return _pure_yaw_gate(context, reward_config) * jp.minimum(yaw / yaw_scale, 1.0)


def feet_air_time_turn(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Reward completed wheel swings while turning in place (M20 air-time).

    Ported from M20's ``feet_air_time_ang_z_cmd_M20``: on the landing frame of
    each wheel, credit ``(last_air_time - threshold)`` so longer genuine swings
    pay more, gated to significant yaw command.  We use ``last_air_time`` (the
    pre-reset air time, non-zero only at touchdown) because ``feet_air_time`` is
    already zeroed on landing.  M20's ground-clearance gate is dropped: their
    config sets ``foot_height_threshold=0.0``, disabling it.
    """
    if context.last_air_time is None or context.first_contact is None:
        return jp.array(0.0)
    threshold = getattr(reward_config, "air_time_turn_threshold", 0.2)
    credit = (context.last_air_time - threshold) * context.first_contact.astype(jp.float32)
    return jp.sum(credit) * _yaw_turn_weight(context, reward_config)


def _wheel_contact(context: VelocityRewardContext) -> jax.Array:
    """Per-wheel in-contact boolean, recomputed from external forces.

    Uses the same 1.0 N vertical-force threshold as feet.py; ``contact`` is not
    threaded into the reward context, so stateless rewards recompute it here
    (as ``slip``/``soft_landing`` already do).
    """
    feet_forces = context.data.cfrc_ext[context.feet_body_ids, 3:6]
    return feet_forces[:, 2] > 1.0


def rotation_gait_status(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Reward a clean diagonal support pattern while turning in place.

    Ported from M20's ``rotation_gait_status``.  Under a pure-yaw command,
    reward 1.0 when one diagonal is fully grounded and the other diagonal's mean
    wheel height exceeds it by more than ``rotation_target_height`` (or the
    mirror case).  M20's ``exp`` kernel line is dead code (overwritten by the
    boolean test); only the boolean version is ported.
    """
    if context.data is None or context.feet_body_ids is None:
        return jp.array(0.0)
    target_h = getattr(reward_config, "rotation_target_height", 0.05)

    in_contact = _wheel_contact(context)
    foot_z = context.data.xpos[context.feet_body_ids, 2]
    a = jp.array(DIAG_A_IDX)
    b = jp.array(DIAG_B_IDX)

    a_grounded = jp.all(in_contact[a]).astype(jp.float32)
    b_grounded = jp.all(in_contact[b]).astype(jp.float32)
    mean_z_a = jp.mean(foot_z[a])
    mean_z_b = jp.mean(foot_z[b])

    # A planted while B lifted, or the mirror.
    pattern_1 = a_grounded * (mean_z_b - mean_z_a > target_h).astype(jp.float32)
    pattern_2 = b_grounded * (mean_z_a - mean_z_b > target_h).astype(jp.float32)
    reward = jp.maximum(pattern_1, pattern_2)
    return reward * _pure_yaw_gate(context, reward_config)


def feet_slide_turn(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Penalize grounded-wheel lateral sliding while turning in place.

    Ported from M20's ``feet_slide_ang_z_cmd`` (which wraps ``feet_slide``):
    take each wheel's velocity relative to the base, rotate into the body frame,
    and penalize the planar (xy) magnitude while the wheel is in contact --
    gated to pure-yaw commands.  Complements the global longitudinal
    ``wheel_slip_longitudinal``: this term targets the *lateral* scrub that a
    grounded wheel makes as the body rotates around it.
    """
    if context.data is None or context.feet_body_ids is None or context.base_quat is None:
        return jp.array(0.0)
    inv_rot = math.quat_inv(context.base_quat)
    # cvel[:, 3:6] is the world-frame linear velocity of each foot body.
    foot_vel_world = context.data.cvel[context.feet_body_ids, 3:6]
    # Foot velocity relative to the base, expressed in the body frame.  Rotation
    # is linear, so rotate(v_foot) - rotate(v_base) = rotate(v_foot) - base_lin_vel.
    foot_vel_body = jax.vmap(lambda v: math.rotate(v, inv_rot))(foot_vel_world)
    rel_vel_body = foot_vel_body - context.base_lin_vel
    lateral = jp.linalg.norm(rel_vel_body[:, :2], axis=1)
    in_contact = _wheel_contact(context).astype(jp.float32)
    cost = jp.sum(lateral * in_contact)
    return cost * _pure_yaw_gate(context, reward_config)


def flat_stance(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Penalize any lifted wheel EXCEPT under a pure-yaw command.

    The turn stack rewards lifting a diagonal wheel pair to step-turn, but only
    inside the pure-yaw gate.  Because the policy is a single shared network, that
    diagonal-lift posture leaks across the gate into the zero-command and
    straight-line regimes, where the robot should keep all four wheels planted --
    it was standing on one diagonal at a zero command.  This term is the exact
    complement of the turn-stack gate: outside pure-yaw, every airborne wheel
    costs reward, so a flat four-wheel stance is the only reward-maximal
    configuration there.  It is precisely zero inside the pure-yaw gate, so it
    never opposes the step-to-turn lift it is meant to contain.
    """
    if context.data is None or context.feet_body_ids is None:
        return jp.array(0.0)
    in_contact = _wheel_contact(context).astype(jp.float32)
    lifted = jp.sum(1.0 - in_contact)  # number of airborne wheels, 0..4
    not_turning = 1.0 - _pure_yaw_gate(context, reward_config)
    return lifted * not_turning


def feet_lift_turn(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Reward lifting a swing wheel HIGH while turning in place.

    Complements ``feet_air_time_turn`` (which pays swing *duration* at
    touchdown): this pays swing *height* densely, every frame a wheel is in the
    air under a pure-yaw command.  Crucially the clearance is weighted by the
    wheel's ACTIVE-SWING speed, i.e. its velocity relative to the body with the
    base's rigid-body motion removed.  A wheel merely held aloft while the body
    yaws still has world-frame velocity ``omega x r`` -- the old world-frame
    gate counted that as "swinging" and paid densely for a static diagonal
    stance.  Subtracting ``v_base + omega x r`` leaves only the leg's motion
    relative to the body, so a static hold earns ~0 and only a genuine step
    pays.  Clearance is measured above the grounded wheel-centre height
    (~wheel radius) and capped at ``lift_turn_target`` so it can't be gamed by
    flinging a leg absurdly high.
    """
    if context.data is None or context.feet_body_ids is None or context.base_quat is None:
        return jp.array(0.0)
    ground_h = getattr(reward_config, "wheel_radius", 0.113)
    target = getattr(reward_config, "lift_turn_target", 0.12)
    foot_z = context.data.xpos[context.feet_body_ids, 2]
    clearance = jp.clip(foot_z - ground_h, 0.0, target)

    # Active-swing velocity in the body frame, with the base's rigid-body motion
    # removed.  qvel[:3] / qvel[3:6] are the world-frame base linear / angular
    # velocity; a foot fixed to the body has world velocity v_base + omega x r,
    # which this subtracts away to zero.
    base_pos = context.data.xpos[0]
    base_lin_world = context.data.qvel[:3]
    base_ang_world = context.data.qvel[3:6]
    foot_pos = context.data.xpos[context.feet_body_ids]
    r = foot_pos - base_pos
    foot_vel_world = context.data.cvel[context.feet_body_ids, 3:6]
    swing_vel_world = foot_vel_world - (base_lin_world + jp.cross(base_ang_world, r))
    inv_rot = math.quat_inv(context.base_quat)
    swing_vel_body = jax.vmap(lambda v: math.rotate(v, inv_rot))(swing_vel_world)
    planar_speed = jp.linalg.norm(swing_vel_body[:, :2], axis=1)

    airborne = 1.0 - _wheel_contact(context).astype(jp.float32)
    reward = jp.sum(clearance * planar_speed * airborne)
    return reward * _yaw_turn_weight(context, reward_config)


def turn_drift(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Penalize the base wandering off-centre while turning in place.

    Under a pure-yaw command the robot should spin about a fixed point, not
    translate across the floor.  ``stand_still`` only fires at a ~zero command
    (a pure yaw is non-zero, so it stays off there) and ``lateral_vel`` only
    damps body-y, leaving forward/backward drift during a turn essentially
    unconstrained.  This penalizes the squared planar base velocity (vx^2 +
    vy^2) -- integrated drift away from the turn centre -- gated to pure-yaw so
    it never touches translating gaits.
    """
    planar_speed_sq = jp.sum(jp.square(context.base_lin_vel[:2]))
    return planar_speed_sq * _pure_yaw_gate(context, reward_config)


def feet_air_time(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Reward feet whose current air time is in the configured target range."""
    threshold_min = getattr(reward_config, "feet_air_time_target_min", 0.05)
    threshold_max = getattr(reward_config, "feet_air_time_target_max", 0.5)
    command_threshold = getattr(reward_config, "command_threshold", 0.5)

    current_air_time = context.feet_air_time
    if current_air_time is None:
        return jp.array(0.0)

    in_range = (current_air_time > threshold_min) & (current_air_time < threshold_max)
    rew_air = jp.sum(in_range.astype(jp.float32))
    total_command = jp.linalg.norm(context.command[:2]) + jp.abs(context.command[2])
    return rew_air * (total_command > command_threshold).astype(jp.float32)


def foot_clearance(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Penalize swing-foot height error weighted by planar foot velocity."""
    if context.data is None or context.feet_body_ids is None:
        return jp.array(0.0)

    target_height = getattr(reward_config, "foot_clearance_target", 0.1)
    command_threshold = getattr(reward_config, "command_threshold", 0.05)

    foot_z = context.data.xpos[context.feet_body_ids, 2]
    feet_vel = context.data.cvel[context.feet_body_ids, 3:6]
    vel_norm = jp.linalg.norm(feet_vel[:, :2], axis=1)
    cost = jp.sum(jp.abs(foot_z - target_height) * vel_norm)

    total_command = jp.linalg.norm(context.command[:2]) + jp.abs(context.command[2])
    active = (total_command > command_threshold).astype(jp.float32)
    # Turn off during pure-yaw: step-to-turn REQUIRES lifting wheels, which this
    # term would otherwise penalize, cancelling the feet_air_time_turn reward.
    active = active * (1.0 - _pure_yaw_gate(context, reward_config))
    return cost * active


def slip(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Penalize planar foot velocity while the foot is in contact."""
    if context.data is None or context.feet_body_ids is None:
        return jp.array(0.0)

    command_threshold = getattr(reward_config, "command_threshold", 0.05)
    feet_forces = context.data.cfrc_ext[context.feet_body_ids, 3:6]
    in_contact = (feet_forces[:, 2] > 1.0).astype(jp.float32)
    feet_vel = context.data.cvel[context.feet_body_ids, 3:6]
    vel_xy_norm_sq = jp.square(jp.linalg.norm(feet_vel[:, :2], axis=1))
    cost = jp.sum(vel_xy_norm_sq * in_contact)

    total_command = jp.linalg.norm(context.command[:2]) + jp.abs(context.command[2])
    return cost * (total_command > command_threshold).astype(jp.float32)


def soft_landing(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """Penalize first-contact impact magnitude."""
    if context.data is None or context.feet_body_ids is None or context.first_contact is None:
        return jp.array(0.0)

    command_threshold = getattr(reward_config, "command_threshold", 0.05)
    feet_forces = context.data.cfrc_ext[context.feet_body_ids, 3:6]
    force_magnitude = jp.linalg.norm(feet_forces, axis=1)
    cost = jp.sum(force_magnitude * context.first_contact.astype(jp.float32))

    total_command = jp.linalg.norm(context.command[:2]) + jp.abs(context.command[2])
    return cost * (total_command > command_threshold).astype(jp.float32)


def compute_foot_swing_height_reward(
    *,
    data: Any,
    feet_body_ids: jax.Array,
    contact_filt: jax.Array,
    first_contact: jax.Array,
    peak_heights: jax.Array,
    command: jax.Array,
    reward_config: Any,
) -> tuple[jax.Array, jax.Array]:
    """Compute swing-height penalty and the next peak-height state."""
    target_height = getattr(reward_config, "foot_clearance_target", 0.1)
    command_threshold = getattr(reward_config, "command_threshold", 0.05)

    foot_heights = data.xpos[feet_body_ids, 2]
    in_air = contact_filt == False
    next_peak_heights = jp.where(in_air, jp.maximum(peak_heights, foot_heights), peak_heights)

    error = next_peak_heights / target_height - 1.0
    cost = jp.sum(jp.square(error) * first_contact.astype(jp.float32))

    total_command = jp.linalg.norm(command[:2]) + jp.abs(command[2])
    active = (total_command > command_threshold).astype(jp.float32)
    reward = cost * active
    next_peak_heights = jp.where(first_contact, 0.0, next_peak_heights)
    return reward, next_peak_heights


def compute_rotation_gait_symmetry_reward(
    *,
    data: Any,
    feet_body_ids: jax.Array,
    command: jax.Array,
    reward_config: Any,
    contact_buffer: jax.Array,
    buffer_idx: jax.Array,
    buffer_filled: jax.Array,
) -> tuple[jax.Array, jax.Array, jax.Array, jax.Array]:
    """Reward a balanced diagonal duty cycle while turning in place.

    Ported from M20's ``RotationGaitSymmetry``.  A clean step-to-turn gait
    alternates the two diagonal support pairs, so over a multi-second window
    each pair should be in contact ~50% of the time.  We keep a per-foot ring
    buffer of contact flags, estimate each diagonal's duty cycle, and reward
    both duties being near ``symmetry_target_duty`` (0.5) with a Gaussian
    kernel.  Gated to pure-yaw commands so it never reshapes the walking gait.

    Returns ``(reward, next_buffer, next_idx, next_filled)``; the caller writes
    the three buffer values back into ``info``.
    """
    buffer_size = contact_buffer.shape[1]

    feet_forces = data.cfrc_ext[feet_body_ids, 3:6]
    in_contact = (feet_forces[:, 2] > 1.0).astype(jp.float32)  # (F,)

    # Write the current contact into the ring buffer, then advance the cursor.
    next_buffer = contact_buffer.at[:, buffer_idx].set(in_contact)
    next_idx = (buffer_idx + 1) % buffer_size
    next_filled = buffer_filled | (next_idx == 0)

    # Only average over columns that have actually been written.  Before the
    # ring wraps, that is columns 0..buffer_idx (inclusive); afterwards, all.
    col = jp.arange(buffer_size)
    valid = jp.where(
        next_filled, jp.ones(buffer_size), (col <= buffer_idx).astype(jp.float32)
    )
    count = jp.where(next_filled, buffer_size, buffer_idx + 1)
    per_foot_duty = jp.sum(next_buffer * valid[None, :], axis=1) / count

    a = jp.array(DIAG_A_IDX)
    b = jp.array(DIAG_B_IDX)
    duty_a = jp.mean(per_foot_duty[a])
    duty_b = jp.mean(per_foot_duty[b])

    target = getattr(reward_config, "symmetry_target_duty", 0.5)
    std = getattr(reward_config, "symmetry_std", 0.2)
    reward = (
        jp.exp(-jp.square(duty_a - target) / (std**2))
        * jp.exp(-jp.square(duty_b - target) / (std**2))
    )

    lin_thr = getattr(reward_config, "turn_lin_threshold", 0.1)
    ang_thr = getattr(reward_config, "turn_ang_threshold", 0.1)
    yaw_scale = getattr(reward_config, "turn_yaw_scale", 1.0)
    lin = jp.linalg.norm(command[:2])
    yaw = jp.abs(command[2])
    gate = ((lin < lin_thr) & (yaw > ang_thr)).astype(jp.float32)
    gate = gate * jp.minimum(yaw / yaw_scale, 1.0)
    return reward * gate, next_buffer, next_idx, next_filled


def compute_phase_gait_reward(
    *,
    data,
    feet_body_ids,
    command,
    reward_config,
    step,
    dt,
):
    """Full phase foot-trajectory reward (diagonal trot).

    Port of M20's ``phase_foot_trajectory_exp``: each foot carries a clock phase
    and must track a reference trajectory in the BODY frame -- during stance the
    foot moves back at ground level, during swing it lifts and moves forward.
    Diagonal pairs get opposite phase offsets (trot).  Because the reference
    POSITION moves, a static hold cannot satisfy it; this is what breaks the
    static-diagonal local optimum that the contact-only phase reward and the
    duty-cycle reward could not.

    ``step`` is the per-env episode step counter and ``dt`` the control period;
    ``phase = (step*dt / cycle_time + offset) mod 1``.
    """
    cycle_time = getattr(reward_config, "gait_cycle_time", 0.6)
    stance_duty = getattr(reward_config, "gait_stance_duty", 0.5)
    offsets = jp.array(
        getattr(reward_config, "gait_phase_offsets", (0.0, 0.5, 0.5, 0.0))
    )
    step_amp = getattr(reward_config, "gait_step_amp", 0.06)
    swing_height = getattr(reward_config, "gait_swing_height", 0.08)
    velocity_weight = getattr(reward_config, "gait_velocity_weight", 0.5)
    std = getattr(reward_config, "gait_track_std", 0.2)
    nominal = jp.array(
        getattr(reward_config, "gait_nominal_foot_offsets", NOMINAL_FOOT_OFFSETS)
    )

    time = step * dt  # scalar per env (the step is vmapped over envs)
    S = jp.mod(time / cycle_time + offsets, 1.0)  # (F,)
    stance = S < stance_duty
    s_stance = S / stance_duty
    t_swing = jp.clip((S - stance_duty) / (1.0 - stance_duty), 0.0, 1.0)

    # Fore-aft (q) and vertical (z) reference trajectory.
    q_stance = step_amp * (1.0 - 2.0 * s_stance)
    q_swing = -step_amp + 2.0 * step_amp * t_swing
    q = jp.where(stance, q_stance, q_swing)
    z_stance = jp.zeros_like(S)
    z_swing = swing_height * jp.sin(jp.pi * t_swing)
    z = jp.where(stance, z_stance, z_swing)
    ref_pos = nominal + jp.stack([q, jp.zeros_like(q), z], axis=-1)  # (F, 3)

    # Reference velocity (d/dt of the trajectory).
    dqdt_stance = (-2.0 * step_amp / stance_duty) / cycle_time
    dqdt_swing = (2.0 * step_amp / (1.0 - stance_duty)) / cycle_time
    dqdt = jp.where(stance, dqdt_stance, dqdt_swing)
    dzdt_swing = (
        swing_height * jp.pi * jp.cos(jp.pi * t_swing) / (1.0 - stance_duty) / cycle_time
    )
    dzdt = jp.where(stance, 0.0, dzdt_swing)
    ref_vel = jp.stack([dqdt, jp.zeros_like(dqdt), dzdt], axis=-1)  # (F, 3)

    # Actual foot state in the body frame.
    base_pos = data.qpos[:3]
    base_quat = data.qpos[3:7]
    inv_rot = math.quat_inv(base_quat)
    foot_pos_body = jax.vmap(lambda v: math.rotate(v - base_pos, inv_rot))(
        data.xpos[feet_body_ids]
    )
    base_lin_world = data.qvel[:3]
    foot_vel_body = jax.vmap(lambda v: math.rotate(v - base_lin_world, inv_rot))(
        data.cvel[feet_body_ids, 3:6]
    )

    pos_err = jp.sum(jp.square(foot_pos_body - ref_pos))
    vel_err = jp.sum(jp.square(foot_vel_body - ref_vel))
    total_err = pos_err + velocity_weight * vel_err
    reward = jp.exp(-total_err / (std * std))

    # Gate to pure-yaw (same as the other step-to-turn rewards) so the
    # already-working straight-line gait is left untouched.
    lin_thr = getattr(reward_config, "turn_lin_threshold", 0.1)
    ang_thr = getattr(reward_config, "turn_ang_threshold", 0.1)
    lin = jp.linalg.norm(command[:2])
    yaw = jp.abs(command[2])
    gate = ((lin < lin_thr) & (yaw > ang_thr)).astype(jp.float32)
    return reward * gate


def stand_success(context: VelocityRewardContext, reward_config: Any) -> jax.Array:
    """1.0 while the torso is upright and near nominal height (stand-up task).

    A sharp, dense success signal for the stand-up skill: once the robot is
    standing it keeps earning this bonus every step, which both rewards
    reaching the posture and maintaining it.
    """
    if context.base_height is None:
        return jp.array(0.0)
    target = getattr(reward_config, "base_height_target", 0.35)
    upright = context.projected_gravity[2] < -0.98
    height_ok = jp.abs(context.base_height - target) < 0.02
    return (upright & height_ok).astype(jp.float32)


def wheel_spin(context: VelocityRewardContext, _: Any) -> jax.Array:
    """Penalize wheel rotation (stand-up task keeps the wheels locked).

    During stand-up the wheels act as static feet; any rolling is wasted motion
    that shifts the body around while the legs try to push it up.
    """
    if context.joint_vel is None or context.num_position_joints is None:
        return jp.array(0.0)
    wheel_vel = context.joint_vel[context.num_position_joints:]
    return jp.sum(jp.square(wheel_vel))


VELOCITY_REWARD_TERMS: Mapping[str, RewardTermSpec] = {
    "tracking_lin_vel": RewardTermSpec(
        "tracking_lin_vel",
        "Track commanded linear velocity.",
        tracking_lin_vel,
    ),
    "tracking_ang_vel": RewardTermSpec(
        "tracking_ang_vel",
        "Track commanded yaw velocity.",
        tracking_ang_vel,
    ),
    "body_ang_vel": RewardTermSpec(
        "body_ang_vel",
        "Penalize body-frame roll/pitch angular velocity.",
        body_ang_vel,
    ),
    "lateral_vel": RewardTermSpec(
        "lateral_vel",
        "Penalize body-frame lateral drift.",
        lateral_vel,
    ),
    "yaw_rate_error": RewardTermSpec(
        "yaw_rate_error",
        "Penalize direct yaw-rate tracking error.",
        yaw_rate_error,
    ),
    "upright": RewardTermSpec(
        "upright",
        "Keep projected gravity close to upright.",
        upright,
    ),
    "pose_hipx": RewardTermSpec(
        "pose_hipx",
        "Hip (roll) posture penalty, always active.",
        pose_hipx,
    ),
    "pose_hipy": RewardTermSpec(
        "pose_hipy",
        "Thigh posture penalty, disabled during pure yaw.",
        pose_hipy,
    ),
    "pose_knee": RewardTermSpec(
        "pose_knee",
        "Calf posture penalty, disabled during pure yaw.",
        pose_knee,
    ),
    "joint_mirror": RewardTermSpec(
        "joint_mirror",
        "Penalize diagonal-leg posture asymmetry.",
        joint_mirror,
    ),
    "bad_orientation": RewardTermSpec(
        "bad_orientation",
        "Penalize a tipped/fallen base orientation.",
        bad_orientation,
    ),
    "dof_pos_limits": RewardTermSpec(
        "dof_pos_limits",
        "Penalize soft joint limit violations.",
        dof_pos_limits,
    ),
    "base_height": RewardTermSpec(
        "base_height",
        "Hold the torso near its nominal ride height.",
        base_height,
    ),
    "action_rate": RewardTermSpec("action_rate", "Penalize action changes.", action_rate),
    "feet_air_time": RewardTermSpec(
        "feet_air_time",
        "Reward valid swing timing.",
        feet_air_time,
    ),
    "feet_air_time_turn": RewardTermSpec(
        "feet_air_time_turn",
        "Reward completed wheel swings while turning in place.",
        feet_air_time_turn,
    ),
    "foot_clearance": RewardTermSpec(
        "foot_clearance",
        "Penalize swing-foot height error.",
        foot_clearance,
    ),
    "foot_swing_height": RewardTermSpec(
        "foot_swing_height",
        "Penalize peak swing height error.",
    ),
    "slip": RewardTermSpec("slip", "Penalize foot slip in contact.", slip),
    "rotation_gait_status": RewardTermSpec(
        "rotation_gait_status",
        "Reward diagonal support pattern while turning in place.",
        rotation_gait_status,
    ),
    "feet_slide_turn": RewardTermSpec(
        "feet_slide_turn",
        "Penalize grounded-wheel lateral slide while turning in place.",
        feet_slide_turn,
    ),
    "rotation_gait_symmetry": RewardTermSpec(
        "rotation_gait_symmetry",
        "Reward a balanced diagonal duty cycle while turning in place.",
    ),
    "phase_gait": RewardTermSpec(
        "phase_gait",
        "Phase-based diagonal-trot contact gait (stateful).",
    ),
    "flat_stance": RewardTermSpec(
        "flat_stance",
        "Penalize lifted wheels except under a pure-yaw command.",
        flat_stance,
    ),
    "feet_lift_turn": RewardTermSpec(
        "feet_lift_turn",
        "Reward high, actively-stepping swing wheels while turning in place.",
        feet_lift_turn,
    ),
    "turn_drift": RewardTermSpec(
        "turn_drift",
        "Penalize base planar drift while turning in place.",
        turn_drift,
    ),
    "stand_still": RewardTermSpec(
        "stand_still",
        "Penalize base motion when the command is ~zero.",
        stand_still,
    ),
    "wheel_slip_longitudinal": RewardTermSpec(
        "wheel_slip_longitudinal",
        "Penalize longitudinal wheel slip (omega*r vs forward speed).",
        wheel_slip_longitudinal,
    ),
    "soft_landing": RewardTermSpec(
        "soft_landing",
        "Penalize high first-contact impact.",
        soft_landing,
    ),
    "stand_success": RewardTermSpec(
        "stand_success",
        "Reward being upright and near nominal height.",
        stand_success,
    ),
    "wheel_spin": RewardTermSpec(
        "wheel_spin",
        "Penalize wheel rotation while standing up.",
        wheel_spin,
    ),
}


def compute_extracted_velocity_rewards(
    *,
    reward_scales: Mapping[str, float],
    reward_config: Any,
    context: VelocityRewardContext,
) -> dict[str, jax.Array]:
    """Compute active registry-backed velocity terms in registry order."""
    rewards = {}
    for name, spec in VELOCITY_REWARD_TERMS.items():
        if spec.fn is not None and name in reward_scales:
            rewards[name] = spec.fn(context, reward_config) * reward_scales[name]
    return rewards
