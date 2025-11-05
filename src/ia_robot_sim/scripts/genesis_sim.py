import genesis as gs
import numpy as np
import os


def draw_bound(scene, entity):
    visualbound = entity.get_vAABB()
    collisionbound = entity.get_AABB()
    scene.draw_debug_box(visualbound, color=(1.0, 0.0, 0.0, 0.5))
    scene.draw_debug_box(collisionbound, color=(0.0, 1.0, 0.0, 0.5))

def test_vel(robot, scene, pos_dofs_idx, vel_dofs_idx):
    input("Enter to continue...")
    for i in range(1000):
        robot.set_dofs_position(
            np.zeros(len(pos_dofs_idx)),
            pos_dofs_idx,
        )
        robot.set_dofs_velocity(
            np.array([5.0, -5.0, 5.0, -5.0]), # right side needs to be negative
            vel_dofs_idx,
        )
        scene.step()


def ctrl(robot, scene):
    target_pos_1 = np.zeros(len(dofs_idx))
    target_pos_1[12] = -2.0  # ARM1_LEFT
    target_pos_1[16] = 2.0   # ARM1_RIGHT

    target_pos_2 = np.zeros(len(dofs_idx))
    target_pos_2[10] = 1.0
    target_pos_2[12] = -2.0  # ARM1_LEFT
    target_pos_2[16] = 2.0   # ARM1_RIGHT

    target_pos_3 = np.zeros(len(dofs_idx))
    target_pos_3[10] = -1.0   # CHEST1
    target_pos_3[12] = -2.0  # ARM1_LEFT
    target_pos_3[16] = 2.0   # ARM1_RIGHT

    # Hard reset
    # for i in range(500):
    #     if i < 50:
    #         robot.set_dofs_position(target_pos_1, dofs_idx)
    #     elif i < 300:
    #         robot.set_dofs_position(target_pos_2, dofs_idx)
    #     else:
    #         robot.set_dofs_position(target_pos_3, dofs_idx)


    print("Starting PD control...")

    # PD control
    for i in range(150):
        if i == 0:
            print("Setting initial position...")
            robot.control_dofs_position(
                target_pos_1,
                dofs_idx,
            )
        elif i == 200:
            print("Moving to target position 2...")
            robot.control_dofs_position(
                target_pos_2,
                dofs_idx,
            )
        elif i == 400:
            print("Moving to target position 3...")
            robot.control_dofs_position(
                target_pos_3,
                dofs_idx,
            )
        elif i == 100:
            print("Controlling with velocity...")
            robot.control_dofs_velocity(
                np.array([5.0, -5.0, 5.0, -5.0]), # right side needs to be negative
                vel_dofs_idx,
            )

        # print('control force:', robot.get_dofs_control_force(chest_idx))

        # This is the actual force experienced by the dof
        # print('internal force:', robot.get_dofs_force(chest_idx))

        scene.step()


########################## init ##########################
gs.init(
    seed                = None,
    precision           = '32',
    debug               = False,
    eps                 = 1e-12,
    logging_level       = None,
    backend             = gs.gpu,
    theme               = 'dark',
    logger_verbose_time = False
)

########################## create a scene ##########################
scene = gs.Scene(
    show_viewer    = True,
    viewer_options = gs.options.ViewerOptions(
        res           = (1280, 960),
        camera_pos    = (3.5, 0.0, 2.5),
        camera_lookat = (0.0, 0.0, 0.5),
        camera_fov    = 40,
        max_FPS       = 60,
    ),
    vis_options = gs.options.VisOptions(
        show_world_frame = True, # visualize the coordinate frame of `world` at its origin
        world_frame_size = 1.0, # length of the world frame in meter
        show_link_frame  = True, # do not visualize coordinate frames of entity links
        show_cameras     = False, # do not visualize mesh and frustum of the cameras added
        plane_reflection = True, # turn on plane reflection
        ambient_light    = (0.9, 0.9, 0.9), # ambient light setting
        shadow           = False,
        visualize_sph_boundary = True
    ),
    renderer = gs.renderers.Rasterizer(), # using rasterizer for camera rendering
    # rigid_options = gs.options.RigidOptions(
    #     enable_self_collision = False,
    # )
)

########################## entities ##########################
plane = scene.add_entity(gs.morphs.Plane())
house = scene.add_entity(
    gs.morphs.Mesh(
        file = '/home/hanxi/code/ia_robot_sim/src/ia_robot_sim/glb/house_nosofa.glb',
        pos = (3.0, 8.0, 0.0),
        euler = (90.0, 0.0, 0.0),
        scale = 1.0,
        fixed = True,  # make the house static
    ),
    # vis_mode="collision"
)
sofa = scene.add_entity(
    gs.morphs.Mesh(
        file = '/home/hanxi/code/ia_robot_sim/src/ia_robot_sim/glb/sofa.glb',
        pos = (-0.45, 0.0, -0.2),
        euler = (90.0, 0.0, 90.0),
        scale = 1.0,
        fixed = True,  # make the sofa static
    ),
    # vis_mode="collision"
)
robot = scene.add_entity(
    gs.morphs.URDF(
        file  = os.path.join(os.path.dirname(__file__), '../../ia_robot_urdf/urdf/ia_robot.absolute.urdf'),
        pos   = (2.0, 1.0, 0.0),
        euler = (0, 0, 180), # we follow scipy's extrinsic x-y-z rotation convention, in degrees,
        # quat  = (1.0, 0.0, 0.0, 0.0), # we use w-x-y-z convention for quaternions,
        scale = 1.0,
        fixed = False,  # TODO: the base cannot move now
        links_to_keep = ["base_footprint", "camera_link", "base_link"]
    )
)

human = scene.add_entity(
    gs.morphs.MJCF(
        file = "/home/hanxi/code/Genesis_IA/genesis_loco/skeleton/human_compact.xml",
        pos = (1.0, 0.0, 0.0),
        euler = (0, 0, 0),
        scale = 1.0,
    ),
)

rgbd_cam0 = gs.vis.camera.Camera(
            visualizer = scene.visualizer, 
            model = 'pinhole',
            res = (640, 400),
            pos = (0.5, 2.5, 3.5),
            lookat = (0.5, 0.5, 0.5),
            fov = 120,
            denoise = True,
            near = 0.05,
            far = 100.0,
            GUI=False
        )

rgbd_cam1 = gs.vis.camera.Camera(
            visualizer = scene.visualizer, 
            model = 'pinhole',
            res = (640, 400),
            pos = (3.5, 2.5, 3.5),
            lookat = (3.5, 0.5, 0.5),
            fov = 120,
            denoise = True,
            near = 0.05,
            far = 100.0,
            GUI=False
        )

lidar = gs.vis.camera.Camera(
            visualizer = scene.visualizer, 
            model = 'pinhole',
            res = (320, 200),
            pos = (2.0, 1.0, 3.0),
            lookat = (2.0, 1.0, 0.5),
            fov = 170,
            denoise = True,
            near = 0.05,
            far = 100.0,
            GUI=False
        )

attach_link = robot.get_link("camera_link")
T = np.array([[  0.00,   0.00,  -1.00,   0.00],
                  [ -1.00,   0.00,   0.00,   0.00],
                  [  0.00,   1.00,   0.00,   0.00],
                  [  0.00,   0.00,   0.00,   1.00]])
scene._visualizer._cameras.append(rgbd_cam0)
scene._visualizer._cameras.append(rgbd_cam1)
scene._visualizer._cameras.append(lidar)
rgbd_cam0.attach(attach_link, T)
rgbd_cam1.attach(attach_link, T)
lidar.attach(attach_link, T)

########################## build ##########################
scene.build()



########################### ctrl ##########################
jnt_names = ['Leg_front_left_3', 'Leg_front_left_2', 'Leg_front_left_1', 'Leg_front_right_3', 'Leg_front_right_2', 'Leg_front_right_1', 'Leg_back_left_2', 'Leg_back_left_1', 'Leg_back_right_2', 'Leg_back_right_1', 'CHEST1', 'ARM0_LEFT', 'ARM1_LEFT', 'ARM2_LEFT', 'ARM3_LEFT', 'ARM0_RIGHT', 'ARM1_RIGHT', 'ARM2_RIGHT', 'ARM3_RIGHT', ]

upper_jnt_names = ['CHEST1', 'ARM1_LEFT', 'ARM2_LEFT', 'ARM3_LEFT']

dofs_idx = [robot.get_joint(name).dof_idx_local for name in jnt_names]
chest_idx = robot.get_joint('CHEST1').dof_idx_local

# Set control gains for all joints
vel_dofs_idx = [robot.get_joint(name).dof_idx_local for name in jnt_names if '_1' in name]
pos_dofs_idx = [robot.get_joint(name).dof_idx_local for name in jnt_names if '_1' not in name]
upper_dofs_idx = [robot.get_joint(name).dof_idx_local for name in upper_jnt_names]


robot.set_dofs_kp(
    np.ones(len(dofs_idx)) * 400.0,
    dofs_idx_local = dofs_idx,
)
robot.set_dofs_kv(
    np.ones(len(dofs_idx)) * 50.0,
    dofs_idx_local = dofs_idx,
)
init_pos = robot.get_dofs_position()


for i in range(100):
    rgbd_cam0.move_to_attach()
    rgbd_cam1.move_to_attach()
    lidar.move_to_attach()
    rgb, depth, _, _ = rgbd_cam0.render(rgb=True, depth=True)
    rgb1, depth1, _, _ = rgbd_cam1.render(rgb=True, depth=True)
    _, lidarout, _, _ = lidar.render(depth=True)
    scene.step()

# import IPython
# IPython.embed()
input('Press Enter to continue...')

# ctrl(robot, scene)
target_pos_1 = np.zeros(len(dofs_idx))
target_pos_1[10] = 1.0   # CHEST1
target_pos_1[12] = -3.0  # ARM1_LEFT
target_pos_1[13] = 2.0   # ARM2_LEFT

# test_vel(robot, scene, pos_dofs_idx, vel_dofs_idx)

############################# IK ##########################

target_pos = np.array([0.2106, -1.0769,  1.0365])
scene.draw_debug_spheres(
    poss = target_pos,
    radius=0.05
)


end_effector = robot.get_link('ARM3_LEFT')
draw_bound(scene, end_effector)

import time
start_time = time.time()
qpos = robot.inverse_kinematics(
    link = end_effector,
    pos  = target_pos,
    # init_qpos = init_pos,
    dofs_idx_local = upper_dofs_idx
)
elapsed_time = time.time() - start_time
print(f"inverse_kinematics took {elapsed_time:.4f} seconds")
print("ik result: ", qpos)
print("current pose:", init_pos)

input("Enter to set pos")
# import IPython; IPython.embed()
robot.set_qpos(qpos)
draw_bound(scene, end_effector)
scene.step()
print("ee pos after set:", end_effector.get_pos())

######################### Path Planning ##########################
input("Enter to start path planning...")

start_time = time.time()
path = robot.plan_path(
    qpos_goal = qpos,
    qpos_start = init_pos,
    planner = 'RRT',
)
elapsed_time = time.time() - start_time
print(f"Path planning took {elapsed_time:.4f} seconds")

# plot the trajectory of end effector along the path as debug spheres in genesis
for waypoint in path:
    robot.set_dofs_position(waypoint)
    scene.draw_debug_spheres(
        poss = robot.get_link('ARM3_LEFT').get_pos().cpu().numpy(),
        radius = 0.01,
        color = (0.0, 1.0, 0.0, 0.5),
    )
    # draw_bound(scene, end_effector)
    scene.step()

robot.set_dofs_position(init_pos)
scene.step()


input("Enter to excute the path...")

for waypoint in path:
    robot.control_dofs_position(waypoint)
    scene.step()

for i in range(1000):
    robot.control_dofs_position(qpos)
    scene.step()


# for i in range(1000):
#     # robot.set_dofs_position(qpos)
#     robot.control_dofs_position(qpos, dofs_idx)
#     # draw_bound(scene, end_effector)
#     scene.step()

