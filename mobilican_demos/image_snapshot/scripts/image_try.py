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
# rosrun image_snapshot image_try.py
# rosservice call /image_snapshot_server // call the server from the command line

class image_try:
  # A node that uses CvBridge to convert ROS images (sensor_msgs/Image) into OpenCV cv::Mat format.
  # The listens to a ROS image message topics, converts the images into an OpenCV cv::Mat format,
  # and holds the current frame from each camera. Upon a client request, it saves the images into files.

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/top_cam/image_raw", Image, self.callback)
    self.init_snapshot_server()

  def init_snapshot_server(self):
    # Initialize a server named "image_snapshot", that receives a snapshot request from the client.
    # When a request is received, the current images are saved to a file.
    self.snapshot_server = rospy.Service("image_snapshot_server", ImageSnapshot, self.handle_image_snapshot)
    print("Ready to save images.")

  def callback(self, data):
    try:
      # Converting an image message pointer to an OpenCV message
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      if self.cv_image is None:
        print("Error: no image received by the topic.")
    except CvBridgeError as e:  # catch conversion errors.
      print(e)

    # cv2.imshow("Image window", self.cv_image)
    # cv2.waitKey(3)

  def handle_image_snapshot(self, req):
    # print("server called")
    status = cv2.imwrite('/home/hila/catkin_ws/src/mobilican/mobilican_demos/image_snapshot/images/image.png', self.cv_image)
    print("Image written to file-system : ", status)
    return "succeed"

def main(args):
  ic = image_try()
  rospy.init_node('image_try', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)