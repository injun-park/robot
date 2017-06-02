#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Shows how to show live images from Nao using PyQt"""

import rospy
import cv2
import sys
import numpy as np

from image_repub.srv import window_op
from image_repub.srv import window_opRequest
from image_repub.srv import window_opResponse
from sensor_msgs.msg import CompressedImage
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

        self.image = None
        self.bridge = CvBridge()
        self.subscriber = None

        self.initGUI()
        # self.image = None
        # self.image_sub = rospy.Subscriber("/facial/image", Image, self.callback)
        # self.window_op = rospy.Service('/image_repub/window_op', window_op, self.windowOperation)
        # self.bridge = CvBridge()


    def initGUI(self):
        QWidget.__init__(self)
        self.setWindowTitle('RobotImage')
        grid = QtGui.QGridLayout()
        grid.setSpacing(10)

        self.comboboxItmes = [
            "/facial/image/compressed",
            "/human_tracking/face_image_display/compressed",
        ]
        self.combobox = QtGui.QComboBox()
        for item in self.comboboxItmes :
            self.combobox.addItem(item)
        self.combobox.currentIndexChanged.connect(self.imageTopicChanged)
        grid.addWidget(self.combobox)

        self.label = QtGui.QLabel()
        self.label.resize(640, 480)
        grid.addWidget(self.label)
        self.setLayout(grid)

        # self._imgWidth = 768
        # self._imgHeight = 1024
        (imgwidth, imgheight) = (640, 580)
        self.resize(imgwidth, imgheight)

        # Trigget 'timerEvent' every 30 ms.
        self.startTimer(30)
        self.showFlag = 1


    def imageTopicChanged(self, index):

        self.image = None

        if self.subscriber is not None :
            self.subscriber.unregister()

        self.subscriber = rospy.Subscriber(self.comboboxItmes[index], CompressedImage, self.callback, queue_size=10)
        print "new subscriber started : ", self.comboboxItmes[index]

    def callback(self, data):
        try:
            np_arr = np.fromstring(data.data, np.uint8)
            # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
            q_image = QtGui.QImage(image_np,
                               640,
                               480,
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
    rospy.init_node("image_window_qt", anonymous=True)

    # Get the service ALVideoDevice.
    app = QApplication(sys.argv)
    myWidget = ImageWidget()
    myWidget.show()
    sys.exit(app.exec_())
