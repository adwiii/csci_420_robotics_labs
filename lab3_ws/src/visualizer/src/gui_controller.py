#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import numpy as np

from gui import GUI
from geometry_msgs.msg import PoseStamped, Pose
from transforms3d._gohlketransforms import euler_from_quaternion

class GUI_Controller(Node):

  def __init__(self):
    # When this node shutsdown
    super().__init__('GUIControllerNode')

    # Set the rate
    self.rate = 2.0
    self.dt = 1.0 / self.rate

    # Create the position
    self.position = np.zeros(3, dtype=np.float64)
    self.quaternion = np.zeros(4)

    gui_data = {'quad1':{'position':[0, 0, 0],'orientation':[0, 0, 0], 'L':1}}

    # Create the gui object
    self.gui_object = GUI(quads=gui_data)

    # Create the subscribers and publishers
    self.gps_sub = self.create_subscription(PoseStamped, "uav/sensors/gps", self.get_gps, 10)

    # Run the communication node
    self._timer = self.create_timer(self.dt, self.UpdateLoop)


  # This is the main loop of this class
  def UpdateLoop(self):
    # Display the position
    self.gui_object.quads['quad1']['position'] = list(self.position)
    self.gui_object.quads['quad1']['orientation'] = list(euler_from_quaternion(self.quaternion))
    self.gui_object.update()

  # Call back to get the gps data
  def get_gps(self, msg):
    self.position[0] = msg.pose.position.x
    self.position[1] = msg.pose.position.y
    self.position[2] = msg.pose.position.z

    self.quaternion = (msg.pose.orientation.x,
                       msg.pose.orientation.y,
                       msg.pose.orientation.z,
                       msg.pose.orientation.w)

# Main function
if __name__ == '__main__':
    rclpy.init()
    try:
        rclpy.spin(GUI_Controller())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()
