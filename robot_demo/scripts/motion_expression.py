#-*- coding: utf-8 -*-
import rospy
import threading
import Queue
from cir_msgs.srv import BehaviorRos_Expression_Srv
from cir_msgs.srv import BehaviorRos_Expression_SrvRequest
from cir_msgs.msg import BehaviorRos_State_Msg
from cir_msgs.srv import BehaviorRos_Stop_Srv
from cir_msgs.srv import BehaviorRos_Stop_SrvRequest
from cir_msgs.srv import BehaviorRos_Song_Srv
from cir_msgs.srv import BehaviorRos_Song_SrvRequest

class Motion :
    def __init__(self, sentenceType, emotionId, expLength, sentence):
        self.sentenceType = sentenceType
        self.emotionId = emotionId
        self.expressonLength = expLength
        self.sentence = sentence

    def __str__(self):
        return self.sentenceType +" : " + self.emotionId + " : " + self.expressonLength +" : " + self.sentence
    def toString(self):
        return self.sentenceType + " : " + self.emotionId + " : " + self.expressonLength + " : " + self.sentence

    @staticmethod
    def splitSentence(sentence):
        arr = sentence.split('|')
        motion = Motion(arr[0], arr[1], arr[2], arr[3].rstrip())
        return motion

class MotionExpression :
    def __init__(self, callbackFunction=None):
        self.motionLock = threading.Lock()
        self.motionQueue = Queue.Queue()
        self.motionPlayingNow = True
        self.motion = rospy.ServiceProxy('/behavior_winros/expression', BehaviorRos_Expression_Srv)
        self.callbackFunction = callbackFunction

        rospy.loginfo("Wait : motion service :")
        self.motion.wait_for_service()
        rospy.loginfo("Found :  motion service :")

        self.subscriber = rospy.Subscriber("/behavior_winros/state", BehaviorRos_State_Msg, self.motionCallback)
        while self.subscriber.get_num_connections() < 1 :
            rospy.loginfo("Wait : motion publisher : count[" + str(self.subscriber.get_num_connections()))
            rospy.sleep(0.1)
        rospy.loginfo("Found : motion publisher")

        success = False
        while success is not True :
            rospy.loginfo("wait behavior INIT")
            msg = self.motionQueue.get()
            if msg.state == "INIT_STATE":
                success = True

        self.song = rospy.ServiceProxy("/behavior_winros/song", BehaviorRos_Song_Srv)
        self.stopBehavior = rospy.ServiceProxy("/behavior_winros/stop", BehaviorRos_Stop_Srv)

    def callMotion(self, motion):
        req = BehaviorRos_Expression_SrvRequest()
        req.sentenceType = motion.sentenceType
        req.emotionID = motion.emotionId
        req.expressionLength = motion.expressonLength
        req.sentence = motion.sentence
        self.motionQueue.queue.clear()
        res = self.motion(req)
        rospy.loginfo("motion call result : " + str(res))


    def callMotionSync(self, motion):
        self.motionLock.acquire()
        try :
            self.motionPlayingNow = True
            self.callMotion(motion)
            self.waitMotionFinish()
            self.motionPlayingNow = False
        finally :
            self.motionLock.release()

    def waitMotionFinish(self):
        success = False
        while success is not True :
            try :
                msg = self.motionQueue.get()
                rospy.loginfo("motion status received " + str(msg))
                if msg.state == "INIT_STATE" :
                    success = True
            except :
                rospy.loginfo("Queue is empty")

    def callDance(self, motion):
        req = BehaviorRos_Song_SrvRequest()
        req.sentenceType = motion.sentenceType
        req.emotionID = motion.emotionId
        req.expressionLength = motion.expressonLength
        req.sentence = motion.sentence

        rospy.loginfo("song : " + str(motion))

        res = self.song(req)

    def motionCallback(self, msg):
        rospy.loginfo("motion status : " + str(msg))
        self.motionQueue.put(msg)
        if self.callbackFunction is not None :
            self.callbackFunction(msg)

    def stop(self):
        req = BehaviorRos_Stop_SrvRequest()
        req.command = 'stop'
        res = self.stopBehavior(req)
        rospy.loginfo("stop result : " + str(res))


    def readMotionFiles(self):
        motions1 = []
        motions2 = []
        file = open('part1.txt', 'r')
        lines = file.readlines()
        for line in lines:
            motion = Motion.splitSentence(line)
            motions1.append(motion)
        file.close()

        file = open('part2.txt', 'r')
        lines = file.readlines()
        for line in lines:
            motion = Motion.splitSentence(line)
            motions2.append(motion)
        file.close()

        # for m in self.motions1 :
        #     rospy.loginfo(m.sentenceType + ", " + m.emotionId +", " + m.expressonLength +", " + m.sentence)
        #     #self.motion.callMotionSync(m)
        #
        # for m in self.motions2:
        #     rospy.loginfo(m.sentenceType + ", " + m.emotionId + ", " + m.expressonLength + ", " + m.sentence)
        #     #self.motion.callMotionSync(m)
        # # for m in self.motions2 :
        # #     rospy.loginfo(m.sentenceType + ", " + m.emotionId + ", " + m.expressonLength + ", " + m.sentence)

        return (motions1, motions2)