#-*- coding: utf-8 -*-
import sys
import rospy
from ccm import CCM
from motion_expression import  Motion
from motion_expression import MotionExpression
from PyQt4 import QtGui
from PyQt4.QtCore import QTextCodec
from PyQt4.QtGui import QListWidget

class Example(QtGui.QWidget):

    def __init__(self):
        super(Example, self).__init__()
        self.ccm = CCM()
        self.ccm.loadComponentAll()

        self.songList = [
            ['dance', 'gangnam', 'final'],
            ['dance', 'tienmimi', 'long'],
            ['dance', 'updown1', '1'],
            ['dance', 'moonheart', 'final'],
        ]

        self.btnPart1 = None
        self.btnPart2 = None
        self.motionExpression = MotionExpression(self.motionCallback)
        self.initUI()

    def initUI(self):
        part1 = QtGui.QLabel('Part1')
        part2 = QtGui.QLabel('Part2')

        grid = QtGui.QGridLayout()
        grid.setSpacing(10)

        grid.addWidget(part1, 1, 0)
        grid.addWidget(part2, 1, 1)

        self.listWidgetLeft = QListWidget()
        self.listWidgetRight = QListWidget()
        grid.addWidget(self.listWidgetLeft, 2, 0)
        grid.addWidget(self.listWidgetRight, 2, 1)

        (self.motionPart1, self.motionPart2) = self.motionExpression.readMotionFiles()
        codec = QTextCodec.codecForName('utf-8')
        for m in self.motionPart1 :
            str = codec.toUnicode(m.toString())
            self.listWidgetLeft.addItem(str)

        for m in self.motionPart2 :
            str = codec.toUnicode(m.toString())
            self.listWidgetRight.addItem(str)

        self.btnPart1 = QtGui.QPushButton('execute part1')
        self.btnPart1.clicked.connect(self.handlePart1Button)
        grid.addWidget(self.btnPart1, 3, 0)

        self.btnPart2 = QtGui.QPushButton('execute part2')
        self.btnPart2.clicked.connect(self.handlePart2Button)
        grid.addWidget(self.btnPart2, 3, 1)


        self.combobox = QtGui.QComboBox()
        for song in self.songList :
            self.combobox.addItem(song[1])
        grid.addWidget(self.combobox, 4, 0)

        self.btnDance = QtGui.QPushButton('execute dance')
        self.btnDance.clicked.connect(self.executeDance)
        grid.addWidget(self.btnDance, 5, 0)

        self.btnStop = QtGui.QPushButton('stop')
        self.btnStop.clicked.connect(self.behaviorStop)
        grid.addWidget(self.btnStop, 5, 1)


        self.setLayout(grid)
        self.setGeometry(100, 100, 500, 300)
        self.setWindowTitle('Review')
        self.show()

    def handlePart1Button(self):
        index = self.listWidgetLeft.currentRow()
        if index >= 0 :
            self.motionExpression.callMotion(self.motionPart1[index])

        #self.demo.callMotions(self.motionPart1)
    def handlePart2Button(self):
        index = self.listWidgetRight.currentRow()
        if index >= 0 :
            self.motionExpression.callMotion(self.motionPart2[index])

    def executeDance(self):
        index = self.combobox.currentIndex()
        sentenceType = self.songList[index][0]
        emotionId = self.songList[index][1]
        expressionLength = self.songList[index][2]
        self.motionExpression.callDance(Motion(sentenceType, emotionId, expressionLength, ''))

    def behaviorStop(self):
        self.motionExpression.stop()

    def motionCallback(self, msg):
        if msg.state == "INIT_STATE" :
            if self.btnPart1 is not None : self.btnPart1.setEnabled(True)
            if self.btnPart2 is not None : self.btnPart2.setEnabled(True)
        else :
            self.btnPart1.setEnabled(False)
            self.btnPart2.setEnabled(False)

        pass#rospy.loginfo("motion callback : " + str(msg))

def main():
    rospy.init_node("mero3_demo_ui", anonymous=False)
    app = QtGui.QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
