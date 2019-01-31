#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_snapshot.srv import *

class image_snapshot:
  # A node that uses CvBridge to convert ROS images (sensor_msgs/Image) into OpenCV cv::Mat format.
  # A node that listens to a ROS image message topics, converts the images into an OpenCV cv::Mat format,
  # and holds the current frame from each camera. Upon a client request, it save the images into files.

  def __init__(self):
    self.init_snapshot_server()
    self.init_image_converter()
    self.init_camera_listeners()

  def init_image_converter(self):
    #
    self.image_pub = rospy.Publisher("image_topic_2", Image)
    self.bridge = CvBridge()

  def init_camera_listeners(self):
    # Initialize a listener for each camera topic.

    # The node subscribes to the chatter topic
    # When new messages are received, listener_callback is invoked with the message as the first argument.
    self.bottom_cam_sub_ = rospy.Subscriber("/bottom_cam/image_raw", Image, self.update_image_callback)
    self.middle_cam_sub_ = rospy.Subscriber("/middle_cam/image_raw", Image, self.update_image_callback)
    self.top_cam_sub_ = rospy.Subscriber("/top_cam/image_raw", Image, self.update_image_callback)

  def init_snapshot_server(self):
    # Initialize a server named "save_image", that receives a snapshot request from the client.
    # When a request is received, the current images are saved to a file.
    self.snapshot_server = rospy.Service("image_snapshot", ImageSnapshot, self.handle_image_snapshot())
    print("Ready to save images.")

  def handle_image_snapshot(self, data):
    try:
      # todo - should be a global variable which is changed each time we have a new frame.
      # todo - when the client ask, we will save the frame. - im_write?
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape

    cv2.imshow("Image window", cv_image)  # todo - instead of show, we want to save the image.
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
    return SaveImageResponse(req.a + req.b)

  def update_image_callback(self, data):
      # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
      self.cur_bottom_image = data  #todo - how to send the specific image to update? and how? by =?



def main(args):
  image_snapshot_creator = image_snapshot()

  # The anonymous=True flag means that rospy will choose a unique
  # name for our 'listener' node so that multiple listeners can
  # run simultaneously.
  rospy.init_node('image_snapshot', anonymous=True) # todo - to change to image_snapshot_node
  try:
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)