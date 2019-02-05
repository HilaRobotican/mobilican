#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('navigation_goal')
roslib.load_manifest("rosparam")
import sys
import rospy
import cv2
import rosparam
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from navigation_goal.srv import *

#usage for trx:
# roslaunch mobilican trx.launch gazebo:=true cams:=true world:="/home/hila/catkin_ws/src/mobilican/mobilican_gazebo/worlds/simplified_market.world"
# rosrun rqt_image_view rqt_image_view
# rosrun rqt_robot_steering rqt_robot_steering
# rosrun navigation_goal image_snapshot_node.py
# rosservice call /image_snapshot_server  # call the server from the command line

TOPICS_CONFIG_NAME = 'cameras_topics_armadillo2.yaml'
TOPICS_CONFIG_PATH = os.path.join(os.path.dirname(sys.argv[0]), '../config/' + TOPICS_CONFIG_NAME)
IMAGES_DIRECTORY = os.path.join(os.path.dirname(sys.argv[0]), '../images/')
IMAGE_FORMAT = '.png'

class image_snapshot_node:
  # A node that uses CvBridge to convert ROS images (sensor_msgs/Image) into OpenCV cv::Mat format.
  # The node listens to a ROS image message topics, converts the images into an OpenCV cv::Mat format,
  # and holds the current frame from each camera. Upon a client request, it saves the images into files.

  def __init__(self):
    self.load_yaml()
    self.counter = 0  # used for numbering the saved images.
    number_of_cameras = len(self.topic_list)
    self.current_images = [None] *  number_of_cameras # the current image from each camera
    self.last_topic_time = [None] * number_of_cameras  # the last time each camera topic was called
    self.camera_subscribers_ = [None] * number_of_cameras  # the subscribers to each camera topic
    self.first_time_timer = True  # True if it is the first time that the timer is called

    self.bridge = CvBridge()
    self.init_camera_listeners()
    self.init_snapshot_server()

  def init_camera_listeners(self):
    # Initialize listeners for each camera topic.
    # When new messages are received, callback functions are invoked with the message as the first argument.
    for i, topic in enumerate(self.topic_list):
      self.camera_subscribers_[i] = rospy.Subscriber(topic, Image, self.update_image_callback, i)
      self.last_topic_time[i] = False
    rospy.Timer(rospy.Duration(1), self.on_timer)  # set on the timer.

  def init_snapshot_server(self):
    # Initialize a server named "image_snapshot_server", that receives a snapshot request from the client.
    # When a request is received, the current images are saved to a file.
    self.snapshot_server = rospy.Service("image_snapshot_server", ImageSnapshot, self.handle_image_snapshot)
    print("Ready to save images.")

  def load_yaml(self):
    # The yaml file contains the topics of the robot's cameras.
    try:
      paramlist = rosparam.load_file(TOPICS_CONFIG_PATH)
      self.topic_list = paramlist[0][0]["cameras_topics"]
    except RosParamException as exc:  # unable to load contents of filename
      print(exc)

  def on_timer(self, event):
    # Validate that each one of the camera's topics is updating each second.
    if self.first_time_timer:
      self.first_time_timer = False
      return
    # print('Timer called at ' + str(event.current_real))

    for i, topic in enumerate(self.topic_list):
      if self.last_topic_time[i] is False:
        print("[image_snapshot_node] Error: one of the camera's topics is not updated.")
      else:
        self.last_topic_time[i] = False

  def update_image_callback(self, data, camera_index):
    self.last_topic_time[camera_index] = True
    try:
      # Converting an image message pointer to an OpenCV message:
      self.current_images[camera_index] = self.bridge.imgmsg_to_cv2(data, "bgr8")
      if self.current_images[camera_index] is None:
        print("[image_snapshot_node] Error: no image received by the topic.")

    except CvBridgeError as e:  # catch conversion errors.
      print(e)

  def handle_image_snapshot(self, req):
    # Save the current images into files.
    print("server called")

    for i, image in enumerate(self.current_images):
      if image is None:
        print("[image_snapshot_node] Error: no image exists.")
      else:
        status = cv2.imwrite(IMAGES_DIRECTORY + self.topic_list[i].split("/")[1] + '_goal' + str(self.counter) + IMAGE_FORMAT, image)
        print("image from " + self.topic_list[i] + " was written to file-system: ", status)

    self.counter += 1
    return "succeed"

def main(args):
  rospy.init_node('image_snapshot_node', anonymous=True)
  im_snapshot = image_snapshot_node()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)