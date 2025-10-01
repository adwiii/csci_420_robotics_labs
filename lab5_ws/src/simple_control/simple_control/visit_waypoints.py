import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Vector3, PoseStamped, Point
import numpy as np
import time

# A class used to visit different waypoints on a map
class VisitWaypoints(Node):
    # On node initialization
    def __init__(self):
        super().__init__('VisitWaypoints')
        # Init the waypoints [x,y] pairs, we will later obtain them from the command line
        waypoints_str = "[[2,6],[2,-4],[-2,-2],[4,-2],[-5,5],[4,4],[-6,-6]]"
        # Convert from string to numpy array
        try:
            self.waypoints=waypoints_str.replace('[','')
            self.waypoints=self.waypoints.replace(']','')
            self.waypoints=np.fromstring(self.waypoints,dtype=int,sep=',')
            self.waypoints=np.reshape(self.waypoints, (-1, 2))
        except:
            print("Error: Could not convert waypoints to Numpy array")
            print(waypoints_str)
            exit()

        # Create the position message we are going to be sending
        self.pos = Vector3()
        self.pos.z = 2.0

        # TODO Checkpoint 1:
        # Subscribe to `uav/sensors/gps`

        # Create the publisher and subscriber
        self.position_pub = self.create_publisher(Vector3, '/uav/input/position_request', 1)

        # Wait 10 seconds before starting
        time.sleep(10)

        # Call the mainloop of our class
        # Set the timer to call the mainloop of our class
        self.rate = 1
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.mainloop)

    def mainloop(self):
        # TODO Checkpoint 1
        # Update mainloop() to fly between the different positions

        # TODO Checkpoint 1
        # Check that we are at the waypoint for 5 seconds

        # Publish the position
        self.position_pub.publish(self.pos)



def main():
    rclpy.init()
    try:
        rclpy.spin(VisitWaypoints())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        rclpy.try_shutdown()

# Main function
if __name__ == '__main__':
    main()