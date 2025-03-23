import numpy as np
import matplotlib.pyplot as plt
from math import atan2, cos, sin, pi

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class BoatSimulator:
    def __init__(self, initial_position, initial_orientation, dt=0.1):
        self.position = np.array(initial_position)
        self.orientation = initial_orientation
        self.dt = dt
        self.trajectory = [np.append(self.position, self.orientation).copy()]
        self.linear_velocities = []
        self.angular_velocities = []

        self.linear_pid = PIDController(1.0, 0.0, 0.2)
        self.angular_pid = PIDController(1.0, 0.0, 0.2)

    def update(self, target_position):
        error_vector = target_position - self.position
        distance_error = np.linalg.norm(error_vector)
        target_orientation = atan2(error_vector[1], error_vector[0])
        orientation_error = self._normalize_angle(target_orientation - self.orientation)

        linear_output = self.linear_pid.compute(distance_error, self.dt)
        angular_output = self.angular_pid.compute(orientation_error, self.dt)

        self.orientation += angular_output * self.dt
        self.position += linear_output * np.array([cos(self.orientation), sin(self.orientation)]) * self.dt

        self.trajectory.append(np.append(self.position, self.orientation).copy())
        self.linear_velocities.append(linear_output)
        self.angular_velocities.append(angular_output)

    def _normalize_angle(self, angle):
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def plot_results(self, target_position):
        trajectory = np.array(self.trajectory)
        time_steps = np.arange(0, len(self.linear_velocities) * self.dt, self.dt)
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

        # Plot trajectory
        ax1.plot(trajectory[:, 0], trajectory[:, 1], label='Trajectory')
        ax1.scatter(*target_position, color='red', label='Target')
        ax1.quiver(trajectory[:, 0], trajectory[:, 1], np.cos(trajectory[:, 2]), np.sin(trajectory[:, 2]), angles='xy', scale_units='xy', scale=1, color='blue', alpha=0.5)
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.legend()
        ax1.set_title('Boat Trajectory')
        ax1.grid()
        ax1.axis('equal')

        # Plot velocities
        ax2.plot(time_steps, self.linear_velocities, label='Linear Velocity (z)')
        ax2.plot(time_steps, self.angular_velocities, label='Angular Velocity (y)')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity')
        ax2.legend()
        ax2.set_title('Velocity Commands')
        ax2.grid()

        plt.tight_layout()
        plt.show()

def main():
    initial_position = np.array([0.0, 0.0])
    initial_orientation = 0.0
    simulator = BoatSimulator(initial_position, initial_orientation)

    while True:
        target_input = input("Enter target X and Y positions separated by a space or comma: ")
        target_x, target_y = map(float, target_input.replace(',', ' ').split())
        target_position = np.array([target_x, target_y])

        # Simulate until the boat is close to the target position
        while np.linalg.norm(target_position - simulator.position) > 0.1:
            simulator.update(target_position)

        simulator.plot_results(target_position)

if __name__ == '__main__':
    main()
