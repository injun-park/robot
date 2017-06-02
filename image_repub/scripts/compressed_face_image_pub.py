#!/usr/bin/python
import roslib
import sys
import time
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/human_tracking/face_image_display/compressed",CompressedImage, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/human_tracking/face_image_display", Image, self.callback)

  def callback(self,ros_data):
    '''Callback function of subscribed topic.
            Here images get converted and features detected'''

    try:
      cv_image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
      #### Create CompressedIamge ####
      msg = CompressedImage()
      msg.header.stamp = rospy.Time.now()
      msg.format = "jpeg"
      msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
      # Publish new image
      self.image_pub.publish(msg)
    except CvBridgeError as e:
      print(e)


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

