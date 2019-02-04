#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('image_snapshot')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_snapshot.srv import *


#usage for trx:
# roslaunch mobilican trx.launch gazebo:=true cams:=true world:="/home/hila/catkin_ws/src/mobilican/mobilican_gazebo/worlds/simplified_market.world"
# rosrun rqt_image_view rqt_image_view
# rosrun rqt_robot_steering rqt_robot_steering
# rosrun image_snapshot image_snapshot_node.py
# rosservice call /image_snapshot_server // call the server from the command line
IMAGES_DIRECTORY = '/home/hila/catkin_ws/src/mobilican/mobilican_demos/image_snapshot/images/'
camera_dict = {0: "bottom", 1: "middle", 2: "top"}

class image_snapshot_node:
  # A node that uses CvBridge to convert ROS images (sensor_msgs/Image) into OpenCV cv::Mat format.
  # The node listens to a ROS image message topics, converts the images into an OpenCV cv::Mat format,
  # and holds the current frame from each camera. Upon a client request, it saves the images into files.

  def __init__(self):
    self.counter = 0
    self.current_images = [None] * 3  # the current image from each camera
    self.bridge = CvBridge()
    self.init_camera_listeners()
    self.init_snapshot_server()

  def init_camera_listeners(self):
    # Initialize listeners for each camera topic.
    # When new messages are received, callback functions are invoked with the message as the first argument.
    self.bottom_cam_sub_ = rospy.Subscriber("/bottom_cam/image_raw", Image, self.update_image_callback, 0)
    self.middle_cam_sub_ = rospy.Subscriber("/middle_cam/image_raw", Image, self.update_image_callback, 1)
    self.top_cam_sub_ = rospy.Subscriber("/top_cam/image_raw", Image, self.update_image_callback, 2)

  def init_snapshot_server(self):
    # Initialize a server named "image_snapshot_server", that receives a snapshot request from the client.
    # When a request is received, the current images are saved to a file.
    self.snapshot_server = rospy.Service("image_snapshot_server", ImageSnapshot, self.handle_image_snapshot)
    print("Ready to save images.")

  def update_image_callback(self, data, camera_index):
    try:
      # Converting an image message pointer to an OpenCV message:
      self.current_images[camera_index] = self.bridge.imgmsg_to_cv2(data, "bgr8")
      if self.current_images[camera_index] is None:
        print("Error: no image received by the topic.")

    except CvBridgeError as e:  # catch conversion errors.
      print(e)

    # cv2.imshow("Image window", self.cv_image)
    # cv2.waitKey(3)

  def handle_image_snapshot(self, req):
    # Save the current image into a file.
    print("server called")
    for image, i in enumerate(self.current_images):
      status = cv2.imwrite(IMAGES_DIRECTORY + camera_dict[i] + '_image_' + str(self.counter) + '.png', image)
      print(camera_dict[i] + " image written to file-system : ", status)
    self.counter += 1
    return "succeed"

def main(args):
  ic = image_snapshot_node()
  rospy.init_node('image_snapshot_node', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)