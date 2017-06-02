#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Shows how to show live images from Nao using PyQt"""

import rospy
import cv2
import sys

from image_repub.srv import window_op
from image_repub.srv import window_opRequest
from image_repub.srv import window_opResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import PyQt4.QtGui as QtGui
from PyQt4.QtGui import QWidget, QImage, QApplication, QPainter

class ImageWidget(QWidget):
    """
    Tiny widget to display camera images from Naoqi.
    """
    def __init__(self):
        """
        Initialization.
        """
        self.initGUI()
        self.image = None
        self.image_sub = rospy.Subscriber("/facial/image", Image, self.callback)
        self.window_op = rospy.Service('/image_repub/window_op', window_op, self.windowOperation)
        self.bridge = CvBridge()

    def initGUI(self):
        QWidget.__init__(self)
        self.setWindowTitle('Robot')
        grid = QtGui.QGridLayout()
        grid.setSpacing(10)
        self.label = QtGui.QLabel()
        grid.addWidget(self.label, 0, 0)
        self.setLayout(grid)

        # self._imgWidth = 768
        # self._imgHeight = 1024
        (imgwidth, imgheight) = (768, 1024)
        self.resize(imgwidth, imgheight)

        # Trigget 'timerEvent' every 30 ms.
        self.startTimer(30)
        self.showFlag = 1


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            cv_image = cv2.resize(cv_image, (0, 0), fx=768. / 640., fy=1024 / 480)
            height, width = cv_image.shape[:2]
            q_image = QtGui.QImage(cv_image,
                               width,
                               height,
                               QtGui.QImage.Format_RGB888)
            self.image = q_image
        except CvBridgeError as e:
            print(e)

    def _updateImage(self):
        if self.image is not None :
            img = QtGui.QPixmap.fromImage(self.image)
            self.label.setPixmap(img)

    def windowOperation(self, req):
        self.showFlag = req.show_flag
        response = window_opResponse()
        response.result = window_opRequest.RESULT_OK
        response.response = "OK"
        return response

    def timerEvent(self, event):
        """
        Called periodically. Retrieve a nao image, and update the widget.
        """
        self._updateImage()
        self.update()
        if(self.showFlag == window_opRequest.HIDE) :
            self.hide()
            #self.image = None
            #self.image_sub.unregister()
        else :
            #self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
            self.show()


    def __del__(self):
        """
        When the widget is deleted, we unregister our naoqi video module.
        """
        #self._unregisterImageClient()
        pass

if __name__ == "__main__" :
    rospy.init_node("image_window_qt", anonymous=False)

    # Get the service ALVideoDevice.
    app = QApplication(sys.argv)
    myWidget = ImageWidget()
    myWidget.show()
    sys.exit(app.exec_())