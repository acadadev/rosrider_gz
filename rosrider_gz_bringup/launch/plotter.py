#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import signal
import sys

# Lists to store the trajectory data
x_data = []
y_data = []

class OdometryPlotter(Node):
    """
    A ROS 2 node that subscribes to an Odometry topic and plots the X/Y position.
    """
    def __init__(self):
        """
        Initializes the node, subscriber, plot, and data storage.
        """
        super().__init__('odometry_plotter_node')
        self.get_logger().info('Odometry Plotter Node started.')

        # --- Parameters ---
        # Declare and get the topic name parameter, default to '/odometry/filtered'
        self.declare_parameter('topic_name', '/next/odometry')
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.get_logger().info(f"Subscribing to topic: {topic_name}")

        # --- Subscriber ---
        # Create a subscriber for the Odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            topic_name,
            self.odometry_callback,
            10)  # QoS profile depth

        # --- Plotting Setup ---
        plt.ion()  # Turn on interactive mode for plotting
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot(x_data, y_data, 'r-') # Initial plot setup (red line)

        # Configure plot appearance
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('Real-time Odometry Plot (X vs Y)')
        self.ax.grid(True)
        self.ax.axis('equal') # Ensure aspect ratio is equal

        # --- Initial Plot Update ---
        # Draw the initial empty plot
        self.fig.canvas.draw()
        plt.show(block=False) # Display the plot without blocking

    def odometry_callback(self, msg: Odometry):
        """
        Callback function executed when a new Odometry message is received.
        Extracts position data, appends it to lists, and updates the plot.
        """
        # Extract X and Y position from the message
        # Note: Odometry provides position (x, y, z) and orientation (quaternion)
        # We plot the x and y components of the position.
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Append new data points
        x_data.append(x)
        y_data.append(y)

        # --- Update Plot Data ---
        # Set the new data for the line plot
        self.line.set_xdata(x_data)
        self.line.set_ydata(y_data)

        # --- Rescale Plot Axes ---
        # Adjust plot limits dynamically to fit the data
        self.ax.relim()  # Recalculate limits
        self.ax.autoscale_view(True, True, True) # Autoscale axes

        # --- Redraw Plot ---
        # Redraw the canvas
        self.fig.canvas.draw()
        # Pause briefly to allow the plot to update and process GUI events
        plt.pause(0.01) # Very important for real-time plotting

    def shutdown_plot(self):
        """
        Closes the matplotlib plot window.
        """
        self.get_logger().info('Closing plot window...')
        plt.close(self.fig)

def main(args=None):
    """
    Main function to initialize ROS, create the node, spin, and handle shutdown.
    """
    rclpy.init(args=args)
    odometry_plotter_node = None
    try:
        odometry_plotter_node = OdometryPlotter()

        # Define a signal handler for graceful shutdown (Ctrl+C)
        def signal_handler(sig, frame):
            print('\nCtrl+C detected. Shutting down node...')
            if odometry_plotter_node:
                odometry_plotter_node.shutdown_plot()
                odometry_plotter_node.destroy_node()
            rclpy.shutdown()
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)

        # Keep the node running until shutdown is requested
        rclpy.spin(odometry_plotter_node)

    except KeyboardInterrupt:
        # This handles the case where spin is interrupted before the signal handler
        pass
    except Exception as e:
        if odometry_plotter_node:
            odometry_plotter_node.get_logger().error(f"An error occurred: {e}")
        else:
            print(f"An error occurred during initialization: {e}")
    finally:
        # Ensure resources are released even if errors occur
        if odometry_plotter_node and rclpy.ok():
            odometry_plotter_node.shutdown_plot()
            odometry_plotter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS Cleanup complete.")


if __name__ == '__main__':
    main()
