#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd
import threading

class OdomPlotter:
    def __init__(self, save_path, csv_path):
        rospy.init_node('odom_plotter', anonymous=True)
        rospy.Subscriber("/gem/base_footprint/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/cte", Float64, self.cte_callback)
        self.odom_data = {'x': [], 'y': []}
        self.cte_data = []
        self.lock = threading.Lock()
        self.save_path = save_path
        self.csv_path = csv_path
        self.fig, self.axs = plt.subplots(2, 1, figsize=(12, 8))
        self.fig.canvas.mpl_connect('close_event', self.handle_close)
        self.path_data = pd.read_csv(self.csv_path)

    def odom_callback(self, msg):
        with self.lock:
            self.odom_data['x'].append(msg.pose.pose.position.x)
            self.odom_data['y'].append(msg.pose.pose.position.y)

    def cte_callback(self, msg):
        with self.lock:
            self.cte_data.append(msg.data)

    def animate(self, i):
        with self.lock:
            # Plot Odometry and Path Trajectory
            self.axs[0].clear()
            self.axs[0].plot(self.odom_data['x'], self.odom_data['y'], label='Odometry', color='blue', linewidth=2)
            self.axs[0].plot(self.path_data['path_x'], self.path_data['path_y'], label='Path Trajectory', color='red', linestyle='--', linewidth=2)
            self.axs[0].set_xlabel('X Coordinate')
            self.axs[0].set_ylabel('Y Coordinate')
            self.axs[0].set_title('Odometry and Path Trajectory')
            self.axs[0].legend()
            self.axs[0].grid(True)

            # Plot CTE
            self.axs[1].clear()
            self.axs[1].plot(self.cte_data, label='CTE', color='green', linewidth=2)
            self.axs[1].set_xlabel('Time Step')
            self.axs[1].set_ylabel('CTE')
            self.axs[1].set_title('Cross-Track Error over Time')
            self.axs[1].legend()
            self.axs[1].grid(True)

    def handle_close(self, evt):
        print("Saving plot to", self.save_path)
        self.fig.savefig(self.save_path, dpi=300, format='png')

    def run(self):
        ani = animation.FuncAnimation(self.fig, self.animate, interval=1000)
        plt.show()

if __name__ == '__main__':
    save_path = '/home/oem/gem2_ws/src/mpc_ros/mpc_gen/odom_trajectory_plot.png'
    csv_path = '/home/oem/gem2_ws/src/mpc_ros/mpc_gen/src/wps_plot.csv'
    plotter = OdomPlotter(save_path, csv_path)
    try:
        plotter.run()
    except rospy.ROSInterruptException:
        pass

