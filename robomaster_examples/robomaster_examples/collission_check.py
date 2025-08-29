from ament_index_python import get_package_share_directory
import rclpy
from rclpy.node import Node
import numpy as np
from os.path import join
from typing import List
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from robomaster_examples.modules.Trajectory import Trajectory

class collission_check(Node):
      
    def __init__(self):
        super().__init__('collission_check')
        self.declare_parameters(
        namespace='',
        parameters=[
            ('trajectory', join(get_package_share_directory('robomaster_examples'), 'waypoints', 'multi_drone_pyramid_circle.yml')),
            ('safe_distance', 0.2),
            ('nb_drones', 2),
            ('quality', 50),
        ])
        self.trajectory = self.get_parameter('trajectory').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.nb_drones = self.get_parameter('nb_drones').value
        self.get_logger().info(f"nb_drones: {self.nb_drones}")
        self.quality = self.get_parameter('quality').value
        self.trajs: List[Trajectory] = []

        for i in range(1, self.nb_drones + 1):
            try:
                traj = Trajectory()
                traj.load(self.trajectory, i)
                self.duration = traj.duration
                self.trajs.append(traj)
            except Exception as e:
                self.get_logger().error(f"trajectory not loaded for drone {i}; {e}")
                
        self.trajectories = np.zeros((self.nb_drones, self.quality, 3))
        self.yaw_trajectories = np.zeros((self.nb_drones, self.quality))
        
        for i in range(1, self.quality):
            t = i * self.duration / self.quality
            for j in range(1, self.nb_drones + 1):
                eval_result = self.trajs[j - 1].eval(t)
                if eval_result is not None:
                    self.trajectories[j - 1][i] = eval_result.pos
                    self.yaw_trajectories[j - 1][i] = eval_result.yaw
                
        # Collision detection
        for i in range(1, self.nb_drones):
            for j in range(i + 1, self.nb_drones + 1):
                for k in range(1, self.quality):
                    if np.linalg.norm(self.trajectories[i - 1][k] - self.trajectories[j - 1][k]) < self.safe_distance:
                        t = k * self.duration / self.quality
                        print(f"Collision detected between drone {i} and drone {j} at time {t:.2f} seconds")
                        break

        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        ax.set_xlim(np.min(self.trajectories[:, :, 0]) - 1, np.max(self.trajectories[:, :, 0]) + 1)
        ax.set_ylim(np.min(self.trajectories[:, :, 1]) - 1, np.max(self.trajectories[:, :, 1]) + 1)
        ax.set_zlim(np.min(self.trajectories[:, :, 2]) - 1, np.max(self.trajectories[:, :, 2]) + 1)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Multi-Drone Trajectory Visualization')

        arrow_length = 0.15

        def update(frame):
            ax.cla()
            ax.set_xlim(np.min(self.trajectories[:, :, 0]) - 1, np.max(self.trajectories[:, :, 0]) + 1)
            ax.set_ylim(np.min(self.trajectories[:, :, 1]) - 1, np.max(self.trajectories[:, :, 1]) + 1)
            ax.set_zlim(np.min(self.trajectories[:, :, 2]) - 1, np.max(self.trajectories[:, :, 2]) + 1)
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            
            current_time = frame * self.duration / self.quality
            ax.set_title(f'Multi-Drone Trajectory Visualization - t={current_time:.2f}s')
            
            trail_duration = 1.5  # seconds
            start_time = max(0, current_time - trail_duration)

            for i in range(self.nb_drones):
                trail_times = np.linspace(start_time, current_time, 50)
                p = np.array([self.trajs[i].eval(t).pos for t in trail_times if self.trajs[i].eval(t) is not None])
                
                if len(p) > 0:
                    # Plot trajectory trail
                    drone_color = plt.cm.hsv(i / self.nb_drones)
                    ax.plot(p[:, 0], p[:, 1], p[:, 2], color=drone_color, linewidth=2, alpha=0.7)
                    
                    # Get current position and yaw
                    eval_result = self.trajs[i].eval(current_time)
                    if eval_result is not None:
                        pos = eval_result.pos
                        yaw = eval_result.yaw
                        
                        # Plot drone position
                        ax.scatter(pos[0], pos[1], pos[2], color=drone_color, marker='o', s=100, label=f"Drone {i+1}")
                        
                        dx = arrow_length * np.cos(yaw)
                        dy = arrow_length * np.sin(yaw)
                        
                        # Draw a 3D arrow to represent yaw orientation
                        ax.quiver(pos[0], pos[1], pos[2], 
                                 dx, dy, 0,
                                 color=drone_color, 
                                 arrow_length_ratio=0.3, 
                                 linewidth=2)
                        
                        ax.text(pos[0], pos[1], pos[2] + 0.2, f"{i+1}", color=drone_color, fontsize=12)
                
            ax.legend(loc='upper right')
            return ax
            
        ani = FuncAnimation(fig, update, frames=self.quality, repeat=True, interval=100)
        plt.show()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = collission_check()
    rclpy.spin_once(node)
