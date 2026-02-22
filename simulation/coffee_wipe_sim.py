"""
OpenK Coffee Wiping Simulation with Genesis
=============================================
Robot arm wipes spilled coffee on a table using an alkaline electrolysis sheet
wrapped around its hand (wipe_pad link).

Uses the real OpenK robot URDF with STL meshes.

4 parallel environments with distinct wiping patterns:
  Env 0: Linear back-and-forth
  Env 1: Circular spiral outward
  Env 2: Zigzag pattern
  Env 3: Figure-eight pattern
"""

import os
import sys
import math
import platform
import numpy as np

import genesis as gs

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF_PATH = os.path.join(PROJECT_ROOT, "simulation", "robot_genesis.urdf")
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "simulation", "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)

N_ENVS = 4
TABLE_HEIGHT = 0.50          # Table top surface height
TABLE_TOP_THICK = 0.03       # Thickness of table top
TABLE_LEG_W = 0.04           # Table leg width
TABLE_W = 0.60               # Table width (X)
TABLE_D = 0.50               # Table depth (Y)

# Robot is mounted on a stand behind the table
STAND_HEIGHT = 0.40          # Stand pillar above the table
ROBOT_BASE_Z = TABLE_HEIGHT + STAND_HEIGHT  # ~0.90m
ROBOT_BASE_X = -0.10         # Slightly behind table center
ROBOT_BASE_Y = 0.0

WIPE_PAD_OFFSET = 0.055 + 0.003     # hand2_1 -> wipe_pad center distance
WIPE_HEIGHT = TABLE_HEIGHT + WIPE_PAD_OFFSET + 0.005  # hand2_1 target so pad touches table
SIM_DT = 4e-3
SIM_SUBSTEPS = 10
HORIZON = 300
VIDEO_FPS = 30

# Wiping area on the table (in front of robot)
WIPE_CENTER_X = 0.12
WIPE_CENTER_Y = 0.0
WIPE_RADIUS = 0.08

IS_MACOS = platform.system() == "Darwin"
_gs_initialized = False


def ensure_gs_init():
    global _gs_initialized
    if not _gs_initialized:
        backend = gs.cpu if IS_MACOS else gs.gpu
        gs.init(seed=0, precision="32", logging_level="info", backend=backend)
        _gs_initialized = True


# ---------------------------------------------------------------------------
# Wiping trajectories
# ---------------------------------------------------------------------------
def _trajectory_linear(phase):
    sweep = math.sin(phase * 6 * math.pi) * WIPE_RADIUS
    return [WIPE_CENTER_X + sweep,
            WIPE_CENTER_Y + (phase - 0.5) * WIPE_RADIUS * 0.5,
            WIPE_HEIGHT]


def _trajectory_spiral(phase):
    r = WIPE_RADIUS * 0.2 + WIPE_RADIUS * 0.8 * phase
    angle = phase * 8 * math.pi
    return [WIPE_CENTER_X + r * math.cos(angle),
            WIPE_CENTER_Y + r * math.sin(angle),
            WIPE_HEIGHT]


def _trajectory_zigzag(phase):
    row = int(phase * 5) % 5
    col_phase = (phase * 5) % 1.0
    direction = 1 if row % 2 == 0 else -1
    return [WIPE_CENTER_X + direction * (col_phase - 0.5) * WIPE_RADIUS * 2,
            WIPE_CENTER_Y + (row / 4.0 - 0.5) * WIPE_RADIUS * 2,
            WIPE_HEIGHT]


def _trajectory_figure8(phase):
    angle8 = phase * 4 * math.pi
    return [WIPE_CENTER_X + WIPE_RADIUS * math.sin(angle8),
            WIPE_CENTER_Y + WIPE_RADIUS * 0.5 * math.sin(2 * angle8),
            WIPE_HEIGHT]


TRAJECTORY_FNS = [_trajectory_linear, _trajectory_spiral,
                  _trajectory_zigzag, _trajectory_figure8]


def create_wiping_trajectories(n_envs: int, n_steps: int) -> np.ndarray:
    """Generate distinct wiping trajectories per environment. Shape: (n_steps, n_envs, 3)."""
    trajectories = np.zeros((n_steps, n_envs, 3))
    t = np.linspace(0, 1, n_steps)
    for step in range(n_steps):
        phase = t[step]
        for env_idx in range(n_envs):
            fn = TRAJECTORY_FNS[env_idx % len(TRAJECTORY_FNS)]
            trajectories[step, env_idx] = fn(phase)
    return trajectories


# ---------------------------------------------------------------------------
# Table builder: thin top + 4 legs
# ---------------------------------------------------------------------------
def add_table(scene, cx=0.0, cy=0.0):
    """Add a realistic table (tabletop + 4 legs) to the scene."""
    # Tabletop
    scene.add_entity(
        morph=gs.morphs.Box(
            pos=(cx, cy, TABLE_HEIGHT - TABLE_TOP_THICK / 2),
            size=(TABLE_W, TABLE_D, TABLE_TOP_THICK),
            fixed=True,
        ),
        surface=gs.surfaces.Default(color=(0.45, 0.28, 0.15, 1.0)),  # Dark wood
    )
    # 4 legs
    leg_h = TABLE_HEIGHT - TABLE_TOP_THICK
    leg_half_h = leg_h / 2.0
    inset_x = TABLE_W / 2 - TABLE_LEG_W / 2 - 0.02
    inset_y = TABLE_D / 2 - TABLE_LEG_W / 2 - 0.02
    for sx, sy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
        scene.add_entity(
            morph=gs.morphs.Box(
                pos=(cx + sx * inset_x, cy + sy * inset_y, leg_half_h),
                size=(TABLE_LEG_W, TABLE_LEG_W, leg_h),
                fixed=True,
            ),
            surface=gs.surfaces.Default(color=(0.40, 0.24, 0.12, 1.0)),
        )


# ---------------------------------------------------------------------------
# Robot mounting stand
# ---------------------------------------------------------------------------
def add_stand(scene, bx, by):
    """Add a cylindrical pillar on the table for mounting the robot."""
    pillar_h = STAND_HEIGHT
    scene.add_entity(
        morph=gs.morphs.Cylinder(
            pos=(bx, by, TABLE_HEIGHT + pillar_h / 2),
            radius=0.04,
            height=pillar_h,
            fixed=True,
        ),
        surface=gs.surfaces.Default(color=(0.3, 0.3, 0.3, 1.0)),
    )


# ---------------------------------------------------------------------------
# Scene builders
# ---------------------------------------------------------------------------
def build_scene(n_envs: int = N_ENVS):
    """Build Genesis scene with robot, table, coffee particles, and camera."""
    ensure_gs_init()

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=SIM_DT,
            substeps=SIM_SUBSTEPS,
        ),
        sph_options=gs.options.SPHOptions(
            lower_bound=(-1.0, -1.0, 0.0),
            upper_bound=(2.0, 2.0, 2.0),
            particle_size=0.01,
        ),
        viewer_options=gs.options.ViewerOptions(
            res=(1280, 720),
            camera_pos=(1.5, -1.0, 1.5),
            camera_lookat=(0.3, 0.0, 0.5),
            camera_fov=45,
            max_FPS=60,
        ),
        vis_options=gs.options.VisOptions(
            show_world_frame=False,
            ambient_light=(0.5, 0.5, 0.5),
        ),
        show_viewer=False,
    )

    # Ground plane
    scene.add_entity(gs.morphs.Plane())

    # Table and stand
    add_table(scene, cx=WIPE_CENTER_X, cy=WIPE_CENTER_Y)
    add_stand(scene, bx=ROBOT_BASE_X, by=ROBOT_BASE_Y)

    # Robot arm (real STL meshes)
    robot = scene.add_entity(
        morph=gs.morphs.URDF(
            file=URDF_PATH,
            pos=(ROBOT_BASE_X, ROBOT_BASE_Y, ROBOT_BASE_Z),
            fixed=True,
            requires_jac_and_IK=True,
        ),
    )

    # Coffee spill - brown liquid droplets on the table
    coffee = scene.add_entity(
        material=gs.materials.SPH.Liquid(sampler="regular"),
        morph=gs.morphs.Box(
            pos=(WIPE_CENTER_X, WIPE_CENTER_Y, TABLE_HEIGHT + 0.015),
            size=(0.12, 0.10, 0.008),
        ),
        surface=gs.surfaces.Default(
            color=(0.35, 0.18, 0.07, 1.0),
            vis_mode="particle",
        ),
    )

    # Camera
    cam = scene.add_camera(
        res=(1280, 720),
        pos=(0.8, -0.6, 1.2),
        lookat=(0.1, 0.0, 0.55),
        fov=50,
        GUI=False,
    )

    return scene, robot, coffee, cam


def run_simulation():
    """Main simulation: 4 environments, each with different wiping pattern."""
    scene, robot, coffee, cam = build_scene(N_ENVS)

    print(f"Building scene with {N_ENVS} parallel environments...")
    scene.build(n_envs=N_ENVS, env_spacing=(2.0, 2.0))

    # End-effector = hand2_1 (wipe_pad is fixed-jointed to it)
    ee_link = robot.get_link("hand2_1")
    print(f"Robot: {robot.n_dofs} DOFs, links: {[l.name for l in robot.links]}")

    # Target orientation: wipe pad facing downward
    target_quat = np.array([0.0, 1.0, 0.0, 0.0])

    trajectories = create_wiping_trajectories(N_ENVS, HORIZON)

    print(f"Starting simulation ({HORIZON} steps, {N_ENVS} environments)...")
    print("Patterns: linear | spiral | zigzag | figure-eight")

    cam.start_recording()

    for step in range(HORIZON):
        target_pos = trajectories[step]           # (N_ENVS, 3)
        target_quats = np.tile(target_quat, (N_ENVS, 1))

        try:
            q = robot.inverse_kinematics(
                link=ee_link,
                pos=target_pos,
                quat=target_quats,
            )
            robot.set_dofs_position(q)
        except Exception as e:
            if step == 0:
                print(f"IK issue at step 0: {e}")

        scene.step()
        cam.render()

        # Gentle camera orbit
        t_norm = step / HORIZON
        angle = t_norm * 2.0 * math.pi * 0.25
        cam_dist = 3.5
        cam_x = cam_dist * math.cos(angle) + 0.5
        cam_y = cam_dist * math.sin(angle) + 0.5
        cam.set_pose(
            pos=(cam_x, cam_y, 2.0),
            lookat=(0.5, 0.5, 0.5),
        )

        if step % 50 == 0:
            print(f"  Step {step}/{HORIZON}")

    video_path = os.path.join(OUTPUT_DIR, "coffee_wipe_simulation.mp4")
    cam.stop_recording(save_to_filename=video_path, fps=VIDEO_FPS)
    print(f"\nVideo saved to: {video_path}")
    return video_path


# ---------------------------------------------------------------------------
# Single-environment demo
# ---------------------------------------------------------------------------
def run_single_env_demo():
    """Single-environment demo for quick testing."""
    ensure_gs_init()

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=SIM_DT, substeps=SIM_SUBSTEPS),
        sph_options=gs.options.SPHOptions(
            lower_bound=(-0.5, -0.5, 0.0),
            upper_bound=(0.8, 0.8, 2.0),
            particle_size=0.01,
        ),
        vis_options=gs.options.VisOptions(
            show_world_frame=False,
            ambient_light=(0.5, 0.5, 0.5),
        ),
        show_viewer=False,
    )

    # Ground
    scene.add_entity(gs.morphs.Plane())

    # Table and stand
    add_table(scene, cx=WIPE_CENTER_X, cy=WIPE_CENTER_Y)
    add_stand(scene, bx=ROBOT_BASE_X, by=ROBOT_BASE_Y)

    # Robot (real meshes)
    robot = scene.add_entity(
        morph=gs.morphs.URDF(
            file=URDF_PATH,
            pos=(ROBOT_BASE_X, ROBOT_BASE_Y, ROBOT_BASE_Z),
            fixed=True,
            requires_jac_and_IK=True,
        ),
    )

    # Coffee spill
    coffee = scene.add_entity(
        material=gs.materials.SPH.Liquid(sampler="regular"),
        morph=gs.morphs.Box(
            pos=(WIPE_CENTER_X, WIPE_CENTER_Y, TABLE_HEIGHT + 0.015),
            size=(0.12, 0.10, 0.008),
        ),
        surface=gs.surfaces.Default(
            color=(0.35, 0.18, 0.07, 1.0),
            vis_mode="particle",
        ),
    )

    # Camera: angled view to clearly show table, robot, and coffee
    cam = scene.add_camera(
        res=(1280, 720),
        pos=(0.55, -0.45, 1.05),
        lookat=(0.10, 0.0, 0.50),
        fov=50,
        GUI=False,
    )

    print("Building single-env scene...")
    scene.build()

    ee_link = robot.get_link("hand2_1")
    print(f"Robot: {robot.n_dofs} DOFs, links: {[l.name for l in robot.links]}")

    target_quat = np.array([0.0, 1.0, 0.0, 0.0])

    n_steps = 200
    trajectories = create_wiping_trajectories(1, n_steps)

    print(f"Running single-env demo ({n_steps} steps)...")
    cam.start_recording()

    for step in range(n_steps):
        target_pos = trajectories[step, 0]

        try:
            q = robot.inverse_kinematics(
                link=ee_link,
                pos=target_pos,
                quat=target_quat,
            )
            robot.set_dofs_position(q)
        except Exception:
            pass

        scene.step()
        cam.render()

        if step % 50 == 0:
            print(f"  Step {step}/{n_steps}")
            particles = coffee.get_particles()
            print(f"  Coffee particles shape: {particles.shape}")

    video_path = os.path.join(OUTPUT_DIR, "coffee_wipe_single_env.mp4")
    cam.stop_recording(save_to_filename=video_path, fps=VIDEO_FPS)
    print(f"\nVideo saved to: {video_path}")
    return video_path


if __name__ == "__main__":
    if "--single" in sys.argv:
        run_single_env_demo()
    else:
        run_simulation()
