import rospy
import cv2
import json
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import os
import math

class ImageConverterPub:

    def __init__(self):
        self.id_pub = rospy.Publisher("/detected_markers", String, queue_size=10)
        print("publishing")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        print("sub init")
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.waypoints = {}  # Dictionary to store waypoints
        self.processed_ids = set()  # Set to keep track of processed IDs
        self.Parameters = aruco.DetectorParameters_create()
        self.Parameters.adaptiveThreshWinSizeMin = 10
        print("init end")

    def odom_callback(self, data):
        self.odom_data = data

    def callback(self, data):
        print("callback")
        try:
            print("callback")
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgra8")
        except CvBridgeError as e:
            print(e)

        markers_img, ids_list, corners = self.detect_aruco(cv_image)

        if ids_list is not None:
            ids_str = ' '.join(str(e) for e in ids_list)
            self.id_pub.publish(ids_str)
            cv2.imshow("detected", markers_img)
            self.process_markers(corners, ids_list)

        cv_image_resized = cv2.resize(cv_image, (640, 480))  # Resize for better visualization
        cv2.imshow("image window", cv_image_resized)
        cv2.waitKey(3)

    def detect_aruco(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=self.Parameters)
        print(ids)
        output = aruco.drawDetectedMarkers(gray, corners, ids)
        return output, ids, corners

    def process_markers(self, corners, ids_list):
        if self.odom_data is not None and corners is not None:
            x, y, z = round(self.odom_data.pose.pose.position.x, 3), round(self.odom_data.pose.pose.position.y, 3), round(self.odom_data.pose.pose.position.z, 3)
            for i, corner_set in enumerate(corners):
                marker_x = (corner_set[0][0] + corner_set[0][2]) / 2
                marker_y = (corner_set[0][1] + corner_set[0][3]) / 2
                marker_distance = math.sqrt((marker_x[0] - x) ** 2 + (marker_y[0] - y) ** 2 + z ** 2)
                print(marker_distance)
                aruco_id = int(ids_list[i][0])

                if aruco_id not in self.processed_ids:
                    print("Waypoint added for ArUco ID:", aruco_id)
                    self.waypoints[aruco_id] = {
                        'id': aruco_id,
                        'pose': {
                            'position': {
                                'x': float(x),
                                'y': float(y),
                                'z': float(z)
                            }
                        }
                    }
                    self.processed_ids.add(aruco_id)
                    self.save_waypoints_to_json()

    def save_waypoints_to_json(self):
        with open('/home/shrey/catkin_ws/src/Tasks/aruco_marker_teleop/waypoints.json', 'w') as json_file:
            json.dump({'waypoints': list(self.waypoints.values())}, json_file, indent=4)
            print("Waypoints saved to JSON")



def main():
    print("Initializing ROS-node")
    rospy.init_node('detect_markers', anonymous=True)
    print("Bring the ArUco markers in front of the camera")
    ic = ImageConverterPub()
    rospy.spin()

if __name__ == '__main__':
    main()
