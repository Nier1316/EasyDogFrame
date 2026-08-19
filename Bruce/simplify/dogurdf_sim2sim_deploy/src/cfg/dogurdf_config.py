"""
Configuration for the dogurdf wheel-legged quadruped — Project 3.

Flat-terrain velocity tracking with minimal domain randomization.

16 DoF: 4 legs x (hip, thigh, calf, wheel).  Legs are position-controlled,
wheels are velocity-controlled:

    action[0:12]  -> leg position offset:  q_target = default + action_scale * a
    action[12:16] -> wheel angular rate:   w_target = wheel_vel_scale * a

A single torque law covers both, with the wheels' kp entry set to 0:

    tau = kp * (q_target - q) + kd * (w_target - qd)

Gains and stance below were calibrated against the model, not guessed; see
the comments at each value for the measurement.
"""

import math

from ml_collections import config_dict

from robots.dogurdf import (
    LEG_PREFIXES,
    NOMINAL_CALF,
    NOMINAL_HIP,
    NOMINAL_THIGH,
    NOMINAL_TORSO_HEIGHT,
    NUM_JOINTS,
    NUM_LEG_JOINTS,
    NUM_WHEELS,
    WHEEL_RADIUS,
    WHEEL_VEL_LIMIT,
)


# Observation layout (see components/tasks/velocity/observations.py):
#   actor      = lin_vel(3) + ang_vel(3) + gravity(3)
#              + leg_pos_rel(12) + leg_vel(12) + wheel_vel(4)
#              + last_action(16) + command(3)
#              + gait_phase(8: sin/cos per foot)                    = 64
#   privileged = actor(64) + wheel_height(4) + wheel_air_time(4)
#              + wheel_contact(4) + contact_forces(12)             = 88
NUM_ACTOR_OBS = 3 + 3 + 3 + NUM_LEG_JOINTS * 2 + NUM_WHEELS + NUM_JOINTS + 3 + 8
NUM_PRIVILEGED_OBS = NUM_ACTOR_OBS + NUM_WHEELS * 3 + NUM_WHEELS * 3
# Odom: rpy[:2] + leg_pos(12) + all_vel(16) + ang_vel_body(3)
ODOM_OBS_SIZE = 2 + NUM_LEG_JOINTS + NUM_JOINTS + 3

# Wheel radius 0.113 m, motor limit 12.5 rad/s -> 1.41 m/s top speed.
MAX_WHEEL_SPEED_MS = WHEEL_VEL_LIMIT * WHEEL_RADIUS


def get_dogurdf_config():
    config = config_dict.ConfigDict()

    # ========== Environment settings ==========
    config.env = config_dict.ConfigDict()
    config.env.num_envs = 4096
    config.env.num_observations = NUM_ACTOR_OBS
    config.env.num_actions = NUM_JOINTS
    config.env.obs_history_length = 1

    # episode_length_s=20.0, decimation=4, dt=0.005 -> 20/(0.005*4) = 1000 steps
    config.env.episode_length = 2000

    # ========== Commands configuration ==========
    config.commands = config_dict.ConfigDict()
    config.commands.num_commands = 3  # lin_vel_x, lin_vel_y, ang_vel_z
    config.commands.resampling_time = 5.5

    config.commands.ranges = config_dict.ConfigDict()
    # 1.0 m/s is ~71% of the 1.41 m/s ceiling, leaving control headroom.
    config.commands.ranges.lin_vel_x = (-1.0, 1.0)
    # Differential-drive platform: all four wheel axes are fixed along body y,
    # and the hip joints only tilt the wheel plane.  Lateral motion is not
    # achievable, so commanding it would train against physics.
    config.commands.ranges.lin_vel_y = (0.0, 0.0)
    # M20 dogurdf commands ang_vel_z in (-2.0, 2.0).
    config.commands.ranges.ang_vel_yaw = (-2.0, 2.0)
    config.commands.ranges.heading = (-math.pi, math.pi)
    config.commands.heading_command = False
    config.commands.heading_control_stiffness = 0.5

    config.commands.zero_cmd_probability = 0.25
    config.commands.rel_heading_envs = 0.3
    # Force a fraction of samples to be pure in-place yaw (zero planar, keep the
    # sampled yaw) so the step-to-turn rewards fire often during training.
    # Mirrors M20's rel_only_ang_z_envs=0.2.  Zero-command takes precedence.
    config.commands.pure_yaw_probability = 0.35

    # ========== Initial state ==========
    # Nominal stance sits at z=0.45; spawn slightly above it.
    config.init_state = config_dict.ConfigDict()
    config.init_state.pos = (0.0, 0.0, 0.46)
    config.init_state.rot = (1.0, 0.0, 0.0, 0.0)

    config.init_state.default_joint_angles = config_dict.ConfigDict()
    for leg in LEG_PREFIXES:
        setattr(config.init_state.default_joint_angles, f"{leg}_hip_joint", NOMINAL_HIP)
        setattr(config.init_state.default_joint_angles, f"{leg}_thigh_joint", NOMINAL_THIGH)
        setattr(config.init_state.default_joint_angles, f"{leg}_calf_joint", NOMINAL_CALF)
        setattr(config.init_state.default_joint_angles, f"{leg}_wheel_joint", 0.0)

    # ========== Control configuration ==========
    config.control = config_dict.ConfigDict()
    config.control.control_type = 'P'
    # Leg gains: measured 2 cm stance sag with 13 N.m peak torque at kp=150/kd=4.
    # kp=300 halves the sag but stiffens the contact; 150 is the better trade.
    config.control.stiffness = 150.0
    config.control.damping = 4.0
    # Wheel velocity loop: kd=2.0 tracks 8.85 rad/s to within 0.01 rad/s at a
    # 17.7 N.m peak (limit 53).  kd>=10 saturates and destabilizes.
    config.control.wheel_stiffness = 0.0
    config.control.wheel_damping = 2.0
    config.control.action_scale = 0.25
    config.control.wheel_vel_scale = WHEEL_VEL_LIMIT
    config.control.decimation = 4
    config.control.use_custom_pd = True
    config.control.action_clip = 1.0

    # ========== Robot geometry ==========
    config.robot = config_dict.ConfigDict()
    config.robot.model_path = "../assets/bot_model/dogurdf/dogurdf.xml"
    config.robot.num_joints = NUM_JOINTS
    config.robot.num_leg_joints = NUM_LEG_JOINTS
    config.robot.num_wheels = NUM_WHEELS
    config.robot.wheel_radius = WHEEL_RADIUS

    # Limits from urdf/dogurdf.urdf; wheels are continuous joints (unbounded).
    config.robot.joint_lower = (
        (-0.6, -0.7, -1.0) * 4 + (-1.0e6,) * NUM_WHEELS
    )
    config.robot.joint_upper = (
        (0.6, 1.75, 0.35) * 4 + (1.0e6,) * NUM_WHEELS
    )
    config.robot.torque_limits = (150.0,) * NUM_LEG_JOINTS + (53.0,) * NUM_WHEELS
    config.robot.dof_vel_limits = (14.0,) * NUM_LEG_JOINTS + (WHEEL_VEL_LIMIT,) * NUM_WHEELS

    # The wheels are this robot's ground-contact bodies.
    config.robot.foot_body_names = tuple(f"{leg}_wheel_link" for leg in LEG_PREFIXES)
    config.robot.foot_site_names = tuple(f"{leg.lower()}_wheel_site" for leg in LEG_PREFIXES)
    config.robot.nonfoot_body_names = ("torso",) + tuple(
        f"{leg}_{part}_link"
        for leg in LEG_PREFIXES
        for part in ("hip", "thigh", "calf")
    )

    config.robot.init_jitter_height = 0.15
    config.robot.init_dof_noise = 0.1
    config.robot.init_dof_vel_noise = 0.01
    config.robot.init_base_vel_noise = 0.5

    config.robot.num_actor_obs = NUM_ACTOR_OBS
    config.robot.num_privileged_obs = NUM_PRIVILEGED_OBS
    config.robot.odom_obs_size = ODOM_OBS_SIZE

    # ========== Simulation configuration ==========
    config.sim = config_dict.ConfigDict()
    config.sim.dt = 0.005
    config.sim.substeps = 1
    config.use_soft_contact = False
    config.sim.nconmax = 4 * 8192
    config.sim.njmax = 300
    config.sim.impl = "jax"

    # ========== Domain randomization (minimal for Project 3) ==========
    config.domain_rand = config_dict.ConfigDict()

    config.domain_rand.randomize_friction = True
    config.domain_rand.friction_range = (0.3, 1.2)

    config.domain_rand.randomize_com_offset = True
    config.domain_rand.com_offset_range = ((-0.025, 0.025), (-0.025, 0.025), (-0.03, 0.03))

    config.domain_rand.randomize_encoder_bias = True
    config.domain_rand.encoder_bias_range = (-0.015, 0.015)

    config.domain_rand.push_robots = True
    config.domain_rand.push_interval_s = 4.0
    config.domain_rand.push_duration_s = 0.0
    config.domain_rand.max_push_vel_xy = 0.5
    config.domain_rand.max_push_vel_z = 0.4
    config.domain_rand.max_push_ang_vel = 0.52
    config.domain_rand.max_push_ang_vel_yaw = 0.78

    config.domain_rand.randomize_dof_init = True

    # Disabled DR items (kept for code compatibility)
    config.domain_rand.randomize_base_mass = False
    config.domain_rand.randomize_mass = False
    config.domain_rand.randomize_motor_strength = False
    config.domain_rand.randomize_Kp_factor = False
    config.domain_rand.randomize_Kd_factor = False
    config.domain_rand.randomize_inertia = False
    config.domain_rand.randomize_dof_damping = False
    config.domain_rand.randomize_action_latency = False
    config.domain_rand.randomize_obs_latency = False
    config.domain_rand.randomize_torques_latency = False
    config.domain_rand.randomize_restitution = False

    config.domain_rand.obs_dof_his_num = 1
    config.domain_rand.obs_imu_his_num = 1
    config.domain_rand.obs_latency_steps = 1
    config.domain_rand.torque_his_num = 3

    # Push-FM disabled
    config.domain_rand.push_robots_FM = False
    config.domain_rand.push_FM_interval_s = 15.0
    config.domain_rand.push_FM_duration_s = 1.0
    config.domain_rand.max_push_force_FM = 50.0
    config.domain_rand.max_push_torque_FM = 5.0
    config.domain_rand.kick_vel = 1.5

    # ========== Terrain (flat only for Project 3) ==========
    config.terrain = config_dict.ConfigDict()
    config.terrain.mesh_type = 'plane'
    config.terrain.measure_heights = False
    config.terrain.static_friction = 1.0
    config.terrain.dynamic_friction = 1.0

    # ========== Rewards configuration ==========
    config.rewards = config_dict.ConfigDict()
    config.rewards.tracking_sigma_lin = 0.5
    config.rewards.tracking_sigma_ang = math.sqrt(0.5)
    config.rewards.upright_sigma = math.sqrt(0.2)
    config.rewards.base_height_target = NOMINAL_TORSO_HEIGHT
    config.rewards.base_height_sigma = 0.05
    config.rewards.feet_air_time_target_min = 0.05
    config.rewards.feet_air_time_target_max = 0.5
    config.rewards.foot_clearance_target = 0.1
    config.rewards.command_threshold = 0.05
    # stand_still: relative weight of wheel-spin penalty vs base-motion penalty
    # under a zero command. Wheel rates (rad/s) are numerically larger than body
    # speed (m/s), so this is <1 to keep the two terms comparable.
    config.rewards.stand_still_wheel_weight = 0.01

    # Pose (variable posture)
    config.rewards.pose_walking_threshold = 0.05
    config.rewards.pose_running_threshold = 1.5
    config.rewards.pose_std_standing_hip = 0.05
    config.rewards.pose_std_standing_calf = 0.1
    config.rewards.pose_std_walking_hip = 0.3
    config.rewards.pose_std_walking_calf = 0.6
    config.rewards.pose_std_running_hip = 0.3
    config.rewards.pose_std_running_calf = 0.6

    # ========== Reward Scales ==========
    config.rewards.scales = config_dict.ConfigDict()

    # Velocity tracking (M20: track_lin_vel_xy_exp=5.0, track_ang_vel_z_exp=3.0)
    config.rewards.scales.tracking_lin_vel = 5.0
    config.rewards.scales.tracking_ang_vel = 3.0
    # Heavier than the legged config: this platform cannot move sideways, so
    # any lateral velocity is pure tyre scrub and should be discouraged.
    config.rewards.scales.lateral_vel = -0.5
    config.rewards.scales.yaw_rate_error = -0.2

    # Pose / orientation (ported from M20; see components/.../rewards.py).
    config.rewards.scales.upright = 1.0
    # base_height raised 1.0 -> 3.0 (same weight as the stand-up task): manual
    # driving of the 08-16 runs showed they learned the diagonal step-turn but
    # crouched too low (CoG too low).  A 1.0 weight could not hold the torso
    # height against the turn-gait rewards that favour a crouched stance.
    config.rewards.scales.base_height = 3.0
    # M20 split posture penalty: L2 joint-pos error, x5 when near-still. hipx
    # (roll) is always on to keep wheels pointing forward; thigh/calf are gated
    # off during pure yaw so a leg can bend to lift its wheel to step-turn.
    config.rewards.scales.pose_hipx = -3.0
    config.rewards.scales.pose_hipy = -1.5
    config.rewards.scales.pose_knee = -0.75
    config.rewards.pose_penalty_stand_still_scale = 5.0
    config.rewards.pose_penalty_velocity_threshold = 0.5
    config.rewards.pose_penalty_command_threshold = 0.1
    # Diagonal-symmetry penalty (M20 joint_mirror): the reward-level cure for the
    # left>>right leg-lift asymmetry, complementary to mirror data augmentation.
    config.rewards.scales.joint_mirror = -0.03
    # Hard tip/fall indicator penalty (M20 bad_orientation_penalty). M20 uses
    # -1000, but at that scale the policy learned to die instantly to dodge the
    # per-step penalty (mean_episode_length collapsed to ~2.7 steps). Backed off
    # to -10 to keep the "don't tip" signal without dominating the reward.
    config.rewards.scales.bad_orientation = -10.0

    # Joint limits
    config.rewards.soft_dof_pos_limit = 0.9
    config.rewards.scales.dof_pos_limits = -1.0

    # Action smoothness (M20 action_rate_l2 = -0.01).
    config.rewards.scales.action_rate = -0.01

    # Swing-leg gait shaping: not applicable to rolling locomotion.
    # `slip` in particular penalizes contact-point horizontal velocity, which
    # is exactly what a correctly rolling wheel produces.
    config.rewards.scales.feet_air_time = 0.0
    config.rewards.scales.foot_clearance = -2.0
    config.rewards.scales.foot_swing_height = 0.0
    config.rewards.scales.slip = 0.0
    config.rewards.scales.soft_landing = 0.0

    # Disabled rewards (weight=0)
    config.rewards.scales.body_ang_vel = -1.0  # roll/pitch rate penalty: keep torso level while turning
    config.rewards.scales.angular_momentum = 0.0
    config.rewards.scales.orientation = 0.0
    config.rewards.scales.vel_mismatch_exp = 0.0
    config.rewards.scales.base_lin_acc = 0.0
    config.rewards.scales.dof_pos_abad = 0.0
    config.rewards.scales.torques = 0.0
    config.rewards.scales.power = 0.0
    config.rewards.scales.dof_acc = 0.0
    config.rewards.scales.power_distribution = 0.0
    config.rewards.scales.smoothness = 0.0
    config.rewards.scales.dof_vel_limits = 0.0
    config.rewards.scales.torque_limits = 0.0
    config.rewards.scales.collision = 0.0
    # Penalize base motion AND wheel spin under a ~zero command so the policy
    # learns a true stop (it can't otherwise: it never observes its own linear
    # velocity, and the gait terms are gated off below command_threshold). Only
    # active for near-zero commands, so locomotion gaits are left alone. Trained
    # from scratch with this term present, so it can be weighted firmly.
    config.rewards.scales.stand_still = -2.0
    # Penalize longitudinal wheel slip (omega*r != forward speed) globally, for
    # every command. This is the controllable half of "grinding the tyres":
    # spin-up/lock-up that wastes energy. Lateral scrub is NOT penalized -- it is
    # geometrically forced for this skid-steer platform. Small weight so it does
    # not fight tracking (a wheel-vel loop always has some tracking lag).
    config.rewards.scales.wheel_slip_longitudinal = -0.02
    config.rewards.wheel_radius = WHEEL_RADIUS

    # Keep all four wheels planted whenever NOT turning in place. The turn stack
    # rewards lifting a diagonal pair, but only inside the pure-yaw gate; that
    # lift posture leaked across the gate (shared policy) and the robot stood on
    # one diagonal at a zero command. This penalizes each airborne wheel outside
    # pure-yaw (exact complement of the turn gate), so a flat stance is the only
    # reward-maximal stand there. Two wheels up -> -1.0/step, enough to overcome
    # the leaked prior. Exactly zero inside pure-yaw, so step-to-turn is untouched.
    config.rewards.scales.flat_stance = -0.5

    # ---- Step-to-turn (pure-yaw) gait, ported from DeepRobotics M20 ----------
    # This platform is skid-steer: no joint rotates about vertical z, so turning
    # with all four wheels planted forces lateral tyre scrub.  Instead, under a
    # pure-yaw command (near-zero planar speed, significant |yaw|) we reward the
    # robot for lifting one diagonal wheel pair and stepping around on the other.
    # All four terms are gated to the pure-yaw regime, so the walking/straight
    # gaits are untouched.
    #
    # CONFIG RESTORED to the 2026-08-16 gentleturn/liftfix recipe (commit
    # 242a371) which produced a clean diagonal step-turn (diagnose_turn on
    # gentleturn_v2 iter2250: diag-A 0.79 / diag-B 0.83 both-airborne, peak lift
    # 0.05 m, yaw ~= cmd, drift 0.06).  The phase_gait trajectory reward added on
    # 08-17 (and the simultaneous disabling of feet_lift_turn /
    # rotation_gait_symmetry) REGRESSED the gait to a static single-wheel lift;
    # that route is abandoned (phase_gait -> 0).
    #
    # Reward each completed wheel swing at touchdown (M20 feet_air_time_ang_z=50).
    config.rewards.scales.feet_air_time_turn = 50.0
    # Reward a clean diagonal support pattern (one pair grounded, other lifted).
    config.rewards.scales.rotation_gait_status = 0.0
    # Penalize grounded-wheel lateral scrub while turning in place (M20 -2.0).
    config.rewards.scales.feet_slide_turn = -2.0
    # Balanced ~50/50 diagonal duty cycle (M20 rotation_gait_symmetry=15).
    config.rewards.scales.rotation_gait_symmetry = 15.0
    # Phase foot-trajectory reward DISABLED: this route regressed the gait
    # (see the restore note above); replaced by the 08-16 turn recipe.
    config.rewards.scales.phase_gait = 0.0
    # Speed-weighted lift reward (restored; the 08-16 recipe used 3.0).  Its
    # "fast leg vibration" failure mode only appeared alongside phase_gait.
    config.rewards.scales.feet_lift_turn = 3.0
    config.rewards.lift_turn_target = 0.12   # clearance (m) above ground where lift reward saturates
    # Penalize the base wandering off the turn centre during pure-yaw. vx^2+vy^2
    # in (m/s)^2; a 0.3 m/s drift -> 0.09 * scale per step. Keep centred spins.
    config.rewards.scales.turn_drift = -2.0

    # Pure-yaw gate + shaping thresholds.
    config.rewards.turn_lin_threshold = 0.1      # ‖cmd_xy‖ below this = "no translation"
    config.rewards.turn_ang_threshold = 0.05     # |cmd_yaw| above this = "turning"
    # |cmd_yaw| at which the step-to-turn shaping reaches full strength.  Below
    # this the stepping incentive scales down, so the policy modulates step size
    # instead of always stepping at maximum (fixes "small stick can't turn, full
    # stick is violent").
    config.rewards.turn_yaw_scale = 1.0
    # Posture penalty retained on thigh/calf during pure yaw (0.0 = fully off).
    # Keeps the legs from flailing while still allowing enough bend to lift.
    config.rewards.pose_turn_relax = 0.3
    config.rewards.air_time_turn_threshold = 0.15  # min swing time (s) that starts earning credit
    config.rewards.rotation_target_height = 0.05  # lifted-pair clearance (m) for a "clean" pattern
    config.rewards.symmetry_target_duty = 0.5     # ideal per-diagonal contact fraction
    config.rewards.symmetry_std = 0.2             # Gaussian width around the target duty
    config.rewards.symmetry_window_s = 5.0        # duty-cycle averaging window (M20 uses 5 s)
    # Phase-based contact gait (diagonal trot) shaping.
    config.rewards.gait_cycle_time = 0.6          # gait cycle period (s)
    config.rewards.gait_stance_duty = 0.5         # fraction of cycle in stance
    config.rewards.gait_phase_offsets = (0.0, 0.5, 0.5, 0.0)  # FL, FR, RL, RR -> diagonal trot
    # Phase foot-trajectory shaping parameters (body-frame, metres).
    config.rewards.gait_step_amp = 0.06           # fore-aft stepping amplitude
    config.rewards.gait_swing_height = 0.08       # swing lift height
    config.rewards.gait_velocity_weight = 0.5     # velocity-error weight in the kernel
    config.rewards.gait_track_std = 1.5           # Gaussian width of the tracking kernel
    config.rewards.scales.termination = 0.0
    config.rewards.scales.stumble = 0.0
    config.rewards.scales.feet_contact_forces = 0.0

    config.rewards.only_positive_rewards = False

    # ========== Observation scales ==========
    config.obs_scales = config_dict.ConfigDict()
    config.obs_scales.lin_vel = 1.0
    config.obs_scales.ang_vel = 1.0
    config.obs_scales.dof_pos = 1.0
    config.obs_scales.dof_vel = 1.0
    config.obs_scales.projected_gravity = 1.0

    # ========== Noise configuration ==========
    config.noise = config_dict.ConfigDict()
    config.noise.add_noise = True
    config.noise.noise_level = 1.0
    config.noise.scales = config_dict.ConfigDict()
    config.noise.scales.dof_pos = 0.01
    config.noise.scales.dof_vel = 1.5
    config.noise.scales.wheel_vel = 1.5
    config.noise.scales.lin_vel = 0.5
    config.noise.scales.ang_vel = 0.2
    config.noise.scales.gravity = 0.05

    # ========== Perturbation configuration ==========
    config.pert_config = config_dict.ConfigDict()
    config.pert_config.enable = False
    config.pert_config.velocity_kick = (3.0, 6.0)
    config.pert_config.kick_wait_times = (5.0, 15.0)

    # ========== Termination configuration ==========
    config.termination = config_dict.ConfigDict()
    config.termination.fell_over_limit_angle = math.radians(70.0)
    config.termination.illegal_contact_force_threshold = 10.0

    return config


def get_dogurdf_ppo_config():
    config = config_dict.ConfigDict()

    # ========== Training parameters ==========
    config.max_iterations = 3000
    config.num_timesteps = 200_000_000
    config.num_envs = 4096

    config.num_evals = 100

    # ========== Reward Scaling ==========
    config.use_isaac_reward_scaling = False
    config.reward_scaling_gamma = 0.99
    config.reward_scaling = 1.0

    config.episode_length = 2000
    config.normalize_observations = False
    config.action_repeat = 1

    # ========== PPO parameters ==========
    config.unroll_length = 24
    config.num_minibatches = 8
    config.num_updates_per_batch = 8
    config.discounting = 0.99
    config.learning_rate = 1.5e-3
    config.entropy_cost = 0.01
    config.max_grad_norm = 1.0
    config.gae_lambda = 0.95
    config.clip_param = 0.2
    config.desired_kl = 0.01
    config.value_loss_coef = 1.0
    config.use_clipped_value_loss = True
    config.lr_schedule = "adaptive"
    # Sagittal (left/right) mirror augmentation: double each minibatch with its
    # mirror to enforce a left/right-symmetric policy.  On by default now that
    # the untrained left>right turn asymmetry has surfaced.
    config.mirror_augment = True

    # ========== Network architecture ==========
    config.network = config_dict.ConfigDict()
    config.network.policy_hidden_layer_sizes = (512, 256, 128)
    config.network.value_hidden_layer_sizes = (512, 256, 128)
    config.network.activation = 'elu'
    config.network.init_noise_std = 1.0

    # ========== Checkpointing ==========
    config.save_interval = 50
    config.checkpoint_save_interval_steps = 10_000_000

    return config


def get_dogurdf_standup_config():
    """Stand-up skill config: rise from a belly-down prone pose within 6 s.

    Reuses the velocity-task machinery but overrides:
    - episode length 300 steps (6 s @ 50 Hz),
    - prone initial state (low torso, bent legs, small random tilt),
    - timeout-only termination (falling is the starting state, not a failure),
    - locked wheels (wheel velocity target is always 0),
    - a reward set that only cares about standing upright and reaching the
      nominal torso height (plus a sharp ``stand_success`` bonus).
    """
    config = get_dogurdf_config()

    # 6 s episode at 50 Hz.
    config.env.episode_length = 300

    # ---- Prone initial state (belly down, legs under body) ----
    config.init_state.pos = (0.0, 0.0, 0.15)
    config.init_state.rot = (1.0, 0.0, 0.0, 0.0)
    config.init_state.rot_noise = 0.2  # rad, small random tilt around belly-down
    # Crouched legs in policy order: (hip, thigh, calf) x4 + (wheel) x4.
    config.init_state.crouch_joint_angles = (0.0, 0.55, -0.95) * 4 + (0.0,) * 4
    config.robot.init_jitter_height = 0.03
    config.robot.init_dof_noise = 0.3
    config.robot.init_dof_vel_noise = 0.0
    config.robot.init_base_vel_noise = 0.0

    # ---- Termination: only timeout (falling / torso contact is the start) ----
    config.termination.fell_over_limit_angle = 3.14159       # ~180 deg, never
    config.termination.illegal_contact_force_threshold = 1e9  # never

    # ---- Lock the wheels (velocity target always 0; kd damps rolling) ----
    config.control.wheel_vel_scale = 0.0

    # ---- Rewards: only stand-up related ----
    scales = config.rewards.scales
    for k in list(scales.keys()):
        scales[k] = 0.0
    scales.upright = 2.0           # torso upright
    scales.base_height = 3.0       # reach nominal height
    scales.action_rate = -0.01     # smooth actions
    scales.dof_pos_limits = -1.0   # respect joint limits
    scales.pose_hipx = -0.5        # keep roll stable while pushing up
    scales.stand_success = 5.0     # sharp success bonus (new term)
    scales.wheel_spin = -0.5       # wheels locked (new term)

    return config
