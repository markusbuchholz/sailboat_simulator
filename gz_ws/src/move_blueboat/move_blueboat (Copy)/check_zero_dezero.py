import numpy as np

class PositionMonitor:
    def __init__(self, initial_dvl, initial_robot_world, angle_degrees):
        self.initial_dvl = np.array(initial_dvl)
        self.initial_robot_world = np.array(initial_robot_world)
        self.angle_radians = np.radians(angle_degrees)
        self.R = self.compute_rotation_matrix(self.angle_radians)

    def compute_rotation_matrix(self, theta):
        # 2D rotation matrix extended to 3D (only rotating in the XY plane)
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])
        return R

    def process_position(self, target_world_position):
        # Calculate the required movement in the world frame
        movement_world = np.array(target_world_position) - self.initial_robot_world

        # Apply rotation to movement to get it in the robot frame
        movement_robot_frame = np.dot(np.linalg.inv(self.R), movement_world)

        # Compute the final DVL position by adding movement in the robot frame to the initial DVL position
        final_dvl_position = self.initial_dvl + movement_robot_frame

        # Print results
        print(f'Initial Robot Position in World Frame: {self.initial_robot_world}')
        print(f'Target Position in World Frame: {target_world_position}')
        print(f'Movement in World Frame: {movement_world}')
        print(f'Movement in Robot Frame: {movement_robot_frame}')
        print(f'Final DVL Position: {final_dvl_position}')
        return final_dvl_position

def main():
    initial_dvl = [-1, 0, 0]  # Initial DVL position This include the drift
    initial_robot_world = [0, 0, 0]  # Initial robot position in the world frame after zeroing
    angle_degrees = 0  # Angle between robot frame and world frame

    monitor = PositionMonitor(initial_dvl, initial_robot_world, angle_degrees)

    # Test data: Target positions in the world frame
    test_positions = [
        [2, 0, 0],
        [5, 5, -1],
        [10, 10, 0]
    ]

    for pos in test_positions:
        print(f'\nInput Target Position (World Frame): {pos}')
        final_pos = monitor.process_position(np.array(pos))
        print(f'Output Final DVL Position: {final_pos}')

if __name__ == "__main__":
    main()
