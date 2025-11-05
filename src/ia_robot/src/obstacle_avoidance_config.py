#!/usr/bin/env python3

"""
Parameter tuning and configuration for obstacle avoidance system.
Adjust these parameters to balance between task completion and obstacle avoidance.

Note: SDF_BOUNDS now define the workspace relative to the robot's current position
(robot-centered bounds) rather than absolute world coordinates.
"""

import numpy as np

class ObstacleAvoidanceConfig:
    """Configuration parameters for obstacle avoidance system"""
    
    # SDF Parameters (robot-centered relative bounds)
    SDF_BOUNDS = ([-2.0, -2.0, -0.5], [2.0, 2.0, 2.0])  # Workspace bounds relative to robot [min_xyz, max_xyz]
    SDF_RESOLUTION = 0.05  # Grid resolution in meters (smaller = more accurate, more memory)
    SDF_MAX_DISTANCE = 2.0  # Maximum distance to compute
    
    # Point Cloud Processing
    POINTCLOUD_MAX_RANGE = 4.0  # Maximum range for valid points
    POINTCLOUD_MIN_RANGE = 0.05  # Minimum range for valid points
    POINTCLOUD_DOWNSAMPLE_SIZE = 3000  # Max points to process (for performance)
    SDF_UPDATE_RATE = 5  # Update SDF every N point cloud messages (for performance)
    
    # Collision Checking
    COLLISION_MARGIN = 0.15  # Safety margin in meters
    COLLISION_THRESHOLD = 0.8  # Distance threshold for penalty activation
    
    # IK Optimization Weights
    POSITION_WEIGHT = 80.0 # 8.0  # Weight for end-effector position error
    ORIENTATION_WEIGHT = 15.0  # Weight for end-effector orientation error
    REGULARIZATION_WEIGHT = 0.1  # Weight for joint angle regularization
    CHASSIS_XY_PENALTY = 1.0  # Penalty for chassis translation
    CHASSIS_Z_PENALTY = 0.5  # Penalty for chassis rotation
    CHEST_PENALTY = -5.0  # Reward for extending chest (negative = reward)
    CHEST_PENALTY_APPROACHING = 5.0 # Penalty for extending chest when approaching human
    ARM1_PENALTY = 0.5  # Penalty for deviating ARM1 from 90 degrees
    OBSTACLE_AVOIDANCE_WEIGHT = 0.01  # Weight for obstacle avoidance (increase for stronger avoidance)
    SYMMETRY_WEIGHT = 0.1  # Weight for symmetry between left and right arms (prismatic joints)

    # IK Constraints
    MAX_POSITION_ERROR = 1.0  # Maximum allowed total position error (meters)
    CHEST_MAX_ANGLE = 0.7  # Maximum chest extension angle (radians)
    MAX_JOINT_STEP = 0.03  # Maximum joint position change per IK iteration (radians/meters)
    MAX_CHASSIS_STEP = 0.2  # Maximum chassis position change per IK iteration (meters/radians)
    
    # Collision Cost Function Parameters
    COLLISION_COST_EXPONENTIAL_RATE = 8.0  # Exponential rate for collision cost
    COLLISION_COST_LINEAR_RATE = 2.0  # Linear rate for collision cost
    
    # Robot Link Geometry (for collision checking)
    LINK_GEOMETRY = {
        'chassis_link': {'type': 'box', 'size': [0.6, 0.4, 0.3]},
        'CHEST1': {'type': 'cylinder', 'radius': 0.15, 'height': 0.3},
        'ARM0_LEFT': {'type': 'sphere', 'radius': 0.09},
        'ARM1_LEFT': {'type': 'cylinder', 'radius': 0.07, 'height': 0.25},
        'ARM2_LEFT': {'type': 'cylinder', 'radius': 0.06, 'height': 0.20},
        'ARM3_LEFT': {'type': 'sphere', 'radius': 0.08},
        'ARM0_RIGHT': {'type': 'sphere', 'radius': 0.09},
        'ARM1_RIGHT': {'type': 'cylinder', 'radius': 0.07, 'height': 0.25},
        'ARM2_RIGHT': {'type': 'cylinder', 'radius': 0.06, 'height': 0.20},
        'ARM3_RIGHT': {'type': 'sphere', 'radius': 0.08},
    }
    
    # Visualization Parameters
    SDF_VISUALIZATION_THRESHOLD = 0.3  # Only show SDF points closer than this
    SDF_VISUALIZATION_SAMPLE_RATE = 25  # Sample every N grid points for visualization
    VISUALIZATION_MAX_MARKERS = 3000  # Maximum markers for performance
    
    @classmethod
    def get_collision_cost_function(cls):
        """Return collision cost function based on distance"""
        def collision_cost(distance_with_margin):
            if distance_with_margin > cls.COLLISION_THRESHOLD:
                return 0.0  # No cost if far enough
            elif distance_with_margin > 0:
                # Exponential cost as we get closer
                normalized_dist = distance_with_margin / cls.COLLISION_THRESHOLD
                return cls.COLLISION_COST_EXPONENTIAL_RATE * np.exp(-normalized_dist * 5)
            else:
                # Very high cost if in collision
                return cls.COLLISION_COST_EXPONENTIAL_RATE * np.exp(5) + cls.COLLISION_COST_LINEAR_RATE * abs(distance_with_margin)
        
        return collision_cost
    
    @classmethod
    def print_config(cls):
        """Print current configuration"""
        print("=== Obstacle Avoidance Configuration ===")
        print(f"SDF Resolution: {cls.SDF_RESOLUTION}m")
        print(f"SDF Bounds: {cls.SDF_BOUNDS}")
        print(f"Collision Margin: {cls.COLLISION_MARGIN}m")
        print(f"Obstacle Avoidance Weight: {cls.OBSTACLE_AVOIDANCE_WEIGHT}")
        print(f"Collision Threshold: {cls.COLLISION_THRESHOLD}m")
        print(f"Point Cloud Max Range: {cls.POINTCLOUD_MAX_RANGE}m")
        print("========================================")


# Preset configurations for different scenarios
class PresetConfigs:
    """Preset configurations for different use cases"""
    
    @staticmethod
    def high_precision():
        """High precision mode - slow but very accurate"""
        ObstacleAvoidanceConfig.SDF_RESOLUTION = 0.05
        ObstacleAvoidanceConfig.POINTCLOUD_DOWNSAMPLE_SIZE = 5000
        ObstacleAvoidanceConfig.SDF_UPDATE_RATE = 2
        ObstacleAvoidanceConfig.OBSTACLE_AVOIDANCE_WEIGHT = 15.0
        ObstacleAvoidanceConfig.COLLISION_MARGIN = 0.2
        print("✓ Applied HIGH PRECISION configuration")
    
    @staticmethod
    def performance_mode():
        """Performance mode - faster but less accurate"""
        ObstacleAvoidanceConfig.SDF_RESOLUTION = 0.12
        ObstacleAvoidanceConfig.POINTCLOUD_DOWNSAMPLE_SIZE = 1500
        ObstacleAvoidanceConfig.SDF_UPDATE_RATE = 8
        ObstacleAvoidanceConfig.OBSTACLE_AVOIDANCE_WEIGHT = 7.0
        ObstacleAvoidanceConfig.COLLISION_MARGIN = 0.12
        print("✓ Applied PERFORMANCE configuration")
    
    @staticmethod
    def aggressive_avoidance():
        """Very aggressive obstacle avoidance"""
        ObstacleAvoidanceConfig.OBSTACLE_AVOIDANCE_WEIGHT = 25.0
        ObstacleAvoidanceConfig.COLLISION_MARGIN = 0.25
        ObstacleAvoidanceConfig.COLLISION_THRESHOLD = 1.0
        ObstacleAvoidanceConfig.COLLISION_COST_EXPONENTIAL_RATE = 15.0
        print("✓ Applied AGGRESSIVE AVOIDANCE configuration")
    
    @staticmethod
    def conservative_avoidance():
        """Conservative avoidance - prioritizes task completion"""
        ObstacleAvoidanceConfig.OBSTACLE_AVOIDANCE_WEIGHT = 3.0
        ObstacleAvoidanceConfig.COLLISION_MARGIN = 0.1
        ObstacleAvoidanceConfig.COLLISION_THRESHOLD = 0.5
        ObstacleAvoidanceConfig.COLLISION_COST_EXPONENTIAL_RATE = 5.0
        print("✓ Applied CONSERVATIVE AVOIDANCE configuration")


def tune_parameters_interactive():
    """Interactive parameter tuning"""
    print("=== Interactive Parameter Tuning ===")
    print("Choose a preset configuration:")
    print("1. High Precision (slow, very accurate)")
    print("2. Performance Mode (fast, good accuracy)")
    print("3. Aggressive Avoidance (strong obstacle avoidance)")
    print("4. Conservative Avoidance (prioritize task completion)")
    print("5. Custom tuning")
    print("6. Use current settings")
    
    try:
        choice = input("Enter choice (1-6): ").strip()
        
        if choice == "1":
            PresetConfigs.high_precision()
        elif choice == "2":
            PresetConfigs.performance_mode()
        elif choice == "3":
            PresetConfigs.aggressive_avoidance()
        elif choice == "4":
            PresetConfigs.conservative_avoidance()
        elif choice == "5":
            print("\nCustom tuning not implemented yet. Using current settings.")
        elif choice == "6":
            print("✓ Using current settings")
        else:
            print("Invalid choice. Using current settings.")
            
    except (KeyboardInterrupt, EOFError):
        print("\nUsing current settings.")
    
    ObstacleAvoidanceConfig.print_config()


if __name__ == "__main__":
    # Test different configurations
    print("Testing different configurations...\n")
    
    print("Default configuration:")
    ObstacleAvoidanceConfig.print_config()
    
    print("\nHigh precision configuration:")
    PresetConfigs.high_precision()
    ObstacleAvoidanceConfig.print_config()
    
    print("\nPerformance configuration:")
    PresetConfigs.performance_mode()
    ObstacleAvoidanceConfig.print_config()
    
    print("\nAggressive avoidance configuration:")
    PresetConfigs.aggressive_avoidance()
    ObstacleAvoidanceConfig.print_config()
