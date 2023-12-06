#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Path
from transforms3d.euler import euler2quat

class MarkerArrayPublisher(Node):

    def __init__(self):
        super().__init__('marker_array_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.publisherPath = self.create_publisher(Path, 'visualization_waypoint_parth', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        marker_array = MarkerArray()
        path = Path()
        j = 0
        filename = '/home/tianhao/sim_ws/src/f1tenth_lab7/mpc/waypoint1atrium4M.csv'
        with open (filename, 'r') as f:
            lines = f.readlines()
            self.wp = []

            for line in lines:
                count = 0
                tempx = ''
                tempy = ''
                tempyaw = ''
                tempv = ''
                for i in range (0, len(line) -1):
                    if count == 0 and line[i] != ',':
                        tempx += line[i]
                    elif count == 1 and line[i] != ',':
                        tempy += line[i]
                    elif count == 2 and line[i] != ',':
                        tempyaw += line[i]
                    elif count == 3 and line[i] != ',':
                        tempv += line[i]
                    else:
                        count += 1


                if tempx != '' and tempy != '':
                    path.header.frame_id = "map"
                    poseStamped = PoseStamped()
                    quaternion = Quaternion()
                    poseStamped.pose.position = Point(x=float(tempx), y=float(tempy), z=0.0)
                    quat = euler2quat(0.0, 0.0, float(tempyaw))
                    quaternion.w = quat[0]
                    quaternion.x = quat[1]
                    quaternion.y = quat[2]
                    quaternion.z = quat[3]

                    poseStamped.pose.orientation = quaternion

                    path.poses.append(poseStamped)



                # if tempx != '' and tempy != '':
                #     marker = Marker()
                #     marker.header.frame_id = "map"
                #     marker.id = j
                #     marker.type = Marker.SPHERE
                #     marker.action = Marker.ADD
                #     marker.pose.position = Point(x=float(tempx), y=float(tempy), z=0.0)
                #     marker.pose.orientation.w = 1.0
                #     marker.scale.x = 0.2
                #     marker.scale.y = 0.2
                #     marker.scale.z = 0.2
                #     marker.color.r = 1.0
                #     marker.color.a = 1.0

                #     marker_array.markers.append(marker)
                #     j = j+1

        self.publisherPath.publish(path)
        self.get_logger().info('Publishing path')

        # self.publisher_.publish(marker_array)
        # self.get_logger().info('Publishing marker array')


def main(args=None):
    rclpy.init(args=args)
    marker_array_publisher = MarkerArrayPublisher()
    rclpy.spin(marker_array_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
