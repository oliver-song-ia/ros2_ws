import genesis as gs
import torch
import numpy as np
import json
import math
from pathlib import Path
import time
import os
from mobile_ik_controller import MobileIKController

class IARobot:
    def __init__(self, scene, urdf_path=None):
        self.scene = scene
        if urdf_path is None:
            urdf_path = os.path.join(os.path.dirname(__file__), '../../ia_robot_urdf/urdf/ia_robot_ik.absolute.urdf')
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(file=urdf_path,
                           links_to_keep=["base_link", "camera_link"],
                           fixed=True),
        )
        self.rgbd_cam = gs.vis.camera.Camera(
            visualizer = self.scene.visualizer, 
            model = 'pinhole',
            res = (640, 480),
            pos = (0.5, 2.5, 3.5),
            lookat = (0.5, 0.5, 0.5),
            fov = 120,
            denoise = True,
            near = 0.05,
            far = 100.0,
            GUI=True
        )
        self.joint_names = [joint.name for joint in self.robot.joints]
        self.motor_dofs = self._get_dofs_idx(self.robot, self.joint_names)

        ## camera
        self.T = np.array([[  0.00,   0.00,  -1.00,   0.00],
                  [ -1.00,   0.00,   0.00,   0.00],
                  [  0.00,   1.00,   0.00,   0.00],
                  [  0.00,   0.00,   0.00,   1.00]])
        self.attached_link = self.robot.get_link("camera_link")
        self.scene._visualizer._cameras.append(self.rgbd_cam)
        self.rgbd_cam.attach(self.attached_link, self.T)
    
    def render(self):
        self.rgbd_cam.move_to_attach()
        color, depth, _, _ = self.rgbd_cam.render(rgb=True, depth=True)
        return color, depth
    
    def get_robot(self):
        return self.robot
    
    def _get_dofs_idx(self, robot, joint_names):
        motor_dofs=[]
        if joint_names is not None:
            # Create a mapping for faster lookup
            joint_name_to_idx = {joint.name: joint.dofs_idx_local[0] for joint in robot.joints if joint.name != "root_joint"}
            
            for joint_name in joint_names:
                if joint_name in joint_name_to_idx:
                    motor_dofs.append(joint_name_to_idx[joint_name])
            # print("motor dofs:",motor_dofs)
        return motor_dofs


def main(urdf_path=None):
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
    # house = scene.add_entity(
    #     gs.morphs.Mesh(
    #         file = '/home/hanxi/code/ia_robot_sim/src/ia_robot_sim/glb/house_00000.glb',
    #         pos = (0, 0, 0),
    #         euler = (90, 0, 0),
    #         scale = 1.0,
    #         fixed = True,  # make the house static
    #     ),
    # )
    ia_robot = IARobot(scene, urdf_path=urdf_path)
    # human = scene.add_entity(
    #     gs.morphs.MJCF(
    #         file = os.path.join(os.path.dirname(__file__), '../../human_mesh/human_compact.xml'),
    #         pos = (1.0, 0.0, 0.0),
    #         euler = (0, 0, 0),
    #         scale = 1.0,
    #     ),
    # )

    ########################## build ##########################
    scene.build()
    input('Press Enter to continue...')
    
    for i in range(100):
        scene.step()
        color, depth = ia_robot.render()

        print('Color and depth image shape:', color.shape, depth.shape)


if __name__ == '__main__':
    urdf_path="/home/hanxi/code/ia_robot_sim/src/ia_robot_urdf/urdf/ia_robot_ik.absolute.urdf"
    main(urdf_path=urdf_path)