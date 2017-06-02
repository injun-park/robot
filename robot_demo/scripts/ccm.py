#-*- coding: utf-8 -*-
#!/usr/bin/python
import rospy
from motion_expression import Motion
from motion_expression import MotionExpression
from cir_msgs.srv import  CCM_Control_Srv
from cir_msgs.srv import  CCM_Control_SrvRequest
from cir_msgs.srv import CCM_Component_Status_Srv
from cir_msgs.srv import CCM_Component_Status_SrvRequest
from cir_msgs.srv import BehaviorRos_Song_Srv
from cir_msgs.srv import BehaviorRos_Song_SrvRequest
from cir_msgs.srv import BehaviorRos_Stop_Srv
from cir_msgs.srv import BehaviorRos_Stop_SrvRequest


class CCM :
    def __init__(self):
        self.components_list = [
        ['devicenode', 'devicenode.exe'],
        ['behavior_winros', 'behavior_winros.exe'],
        ['lipsync', 'lipsync.exe'],
        ['sound', 'sound.exe'],
        ]
        self.ccm_control = rospy.ServiceProxy('/robot/ccm/component_control', CCM_Control_Srv)
        self.ccm_status = rospy.ServiceProxy('/robot/ccm/component_status', CCM_Component_Status_Srv)

        rospy.loginfo("wating for ccm control service")
        self.ccm_control.wait_for_service()
        rospy.loginfo("Found : ccm control service")

        rospy.loginfo("wating for ccm status service")
        self.ccm_status.wait_for_service()
        rospy.loginfo("Found : ccm status service")

    def loadComponentAll(self):
        req = CCM_Control_SrvRequest()
        for [pkg, executable] in self.components_list :
            req.pkgname = pkg
            req.executable = executable
            req.command = "LOAD"
            if self.getComponentStatus(pkg, executable) != 1 :
                rospy.loginfo("Load component : [" + pkg +", " + executable +"]")
                self.ccm_control(req)
            else :
                rospy.loginfo("Component [" + pkg +", " + executable +"] has been already Loaded")
            rospy.sleep(0.1)
        self.motion = MotionExpression()

        # self.song = rospy.ServiceProxy("/behavior_winros/song", BehaviorRos_Song_Srv)
        # self.stopBehavior = rospy.ServiceProxy("/behavior_winros/stop", BehaviorRos_Stop_Srv)

    def unloadComonentAll(self):
        req = CCM_Control_SrvRequest()
        for [pkg, executable] in self.components_list:
            rospy.loginfo("Unload component[" + pkg +", " + executable +"]")
            req.pkgname = pkg
            req.executable = executable
            req.command = "UNLOAD"
            self.ccm_control(req)

    def getComponentStatus(self, pkg, executable):
        req = CCM_Component_Status_SrvRequest()
        req.pkgname = pkg
        req.executable = executable
        res = self.ccm_status(req)
        return res.status