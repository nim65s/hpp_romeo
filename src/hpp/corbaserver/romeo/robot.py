#
# Copyright (c) 2014 CNRS
# Authors: Renaud Viry
#
#
# This file is part of hpp_romeo
# hpp_romeo is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp_romeo is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp_romeo  If not, see
# <http://www.gnu.org/licenses/>.

from hpp.corbaserver.robot import Robot as Parent

##
#  Control of robot Romeo in hpp
#
#  This class implements a client to the corba server implemented in
#  hpp-corbaserver. It derive from class hpp.corbaserver.robot.Robot.
#
#  This class is also used to initialize a client to rviz in order to
#  display configurations of the Romeo robot.
#
#  At creation of an instance, the urdf and srdf files are loaded using
#  idl interface hpp::corbaserver::Robot::loadRobotModel.
class Robot (Parent):
    ##
    #  Information to retrieve urdf and srdf files.
    packageName = "romeo_description"
    ##
    #  Information to retrieve urdf and srdf files.
    urdfName = "romeo"
    urdfSuffix = ""
    srdfSuffix = ""

    halfSitting = {'LEyePitch': 0,
                   'LWristYaw': -0.3,
                   'base_joint_SO3': (1, 0, 0, 0),
                   'LEyeYaw': 0,
                   'RWristYaw': -0.3,
                   'LHipYaw': 0,
                   'RHipPitch': -0.3490658,
                   'RElbowYaw': 1.05,
                   'LShoulderYaw': 0.6,
                   'TrunkYaw': 0,
                   'RShoulderPitch': 1.5,
                   'LShoulderPitch': 1.5,
                   'LWristPitch': -0.2,
                   'HeadRoll': 0,
                   'LKneePitch': 0.6981317,
                   'RAnkleRoll': 0,
                   'LHipPitch': -0.3490658,
                   'LElbowYaw': -1.05,
                   'RHipYaw': 0,
                   'LAnklePitch': -0.3490658,
                   'RAnklePitch': -0.3490658,
                   'LToePitch': 0,
                   'RKneePitch': 0.6981317,
                   'HeadPitch': 0,
                   'LWristRoll': -0.4,
                   'RShoulderYaw': -0.6,
                   'RWristPitch': -0.2,
                   'LElbowRoll': -0.5,
                   'RWristRoll': -0.4,
                   'LAnkleRoll': 0,
                   'REyeYaw': 0,
                   'NeckPitch': 0,
                   'REyePitch': 0,
                   'RToePitch': 0,
                   'LHipRoll': 0,
                   'RHipRoll': 0,
                   'RElbowRoll': 0.5,
                   'base_joint_xyz': (0, 0, 0.840252),
                   #'base_joint_z': 0.840252,
                   #'base_joint_y': 0,
                   #'base_joint_x': 0,
                   'NeckYaw': 0,
                   # Here start romeo -full- specifics
                   'LFinger11': 0,
                   'LFinger12': 0,
                   'LFinger13': 0,
                   'LFinger21': 0,
                   'LFinger22': 0,
                   'LFinger23': 0,
                   'LFinger31': 0,
                   'LFinger32': 0,
                   'LFinger33': 0,
                   'LThumb1': 0,
                   'LThumb2': 0,
                   'LThumb3': 0,

                   'RFinger11': 0,
                   'RFinger12': 0,
                   'RFinger13': 0,
                   'RFinger21': 0,
                   'RFinger22': 0,
                   'RFinger23': 0,
                   'RFinger31': 0,
                   'RFinger32': 0,
                   'RFinger33': 0,
                   'RThumb1': 0,
                   'RThumb2': 0,
                   'RThumb3': 0,

                   'LEyeYaw': 0,
                   'LEyePitch': 0,
                   'REyeYaw': 0,
                   'REyePitch': 0}

    leftHandClosed = {"LFinger11": 0.0,
                      "LFinger12": 0.0,
                      "LFinger13": 0.0,
                      "LFinger21": 0.0,
                      "LFinger22": 0.0,
                      "LFinger23": 0.0,
                      "LFinger31": 0.0,
                      "LFinger32": 0.0,
                      "LFinger33": 0.0,
                      "LThumb1": 0.0,
                      "LThumb2": 0.0,
                      "LThumb3": 0.0}
    leftHandOpen = {"LFinger11": 1.06,
                    "LFinger12": 1.06,
                    "LFinger13": 1.06,
                    "LFinger21": 1.06,
                    "LFinger22": 1.06,
                    "LFinger23": 1.06,
                    "LFinger31": 1.06,
                    "LFinger32": 1.06,
                    "LFinger33": 1.06,
                    "LThumb1": -1.06,
                    "LThumb2": 1.06,
                    "LThumb3": 1.06}
    rightHandClosed = {"RFinger11": 0.0,
                       "RFinger12": 0.0,
                       "RFinger13": 0.0,
                       "RFinger21": 0.0,
                       "RFinger22": 0.0,
                       "RFinger23": 0.0,
                       "RFinger31": 0.0,
                       "RFinger32": 0.0,
                       "RFinger33": 0.0,
                       "RThumb1": 0.0,
                       "RThumb2": 0.0,
                       "RThumb3": 0.0}
    rightHandOpen = {"RFinger11": 1.06,
                     "RFinger12": 1.06,
                     "RFinger13": 1.06,
                     "RFinger21": 1.06,
                     "RFinger22": 1.06,
                     "RFinger23": 1.06,
                     "RFinger31": 1.06,
                     "RFinger32": 1.06,
                     "RFinger33": 1.06,
                     "RThumb1": -1.06,
                     "RThumb2": 1.06,
                     "RThumb3": 1.06}

    def __init__ (self, robotName, load = True):
        Parent.__init__ (self, robotName, "freeflyer", load)
        self.tf_root = "base_link"
        self.leftAnkle = "l_sole_joint"
        self.rightAnkle = "r_sole_joint"

    def getInitialConfig (self):
        q = []
        for n in self.jointNames:
            dof = self.halfSitting [n]
            if type (dof) is tuple:
                q += dof
            else:
                q.append (dof)
        return q

    def getJointDofValue(self, jointName):
      i = 0
      for n in self.jointNames:
        if (n == jointName):
          return self.getCurrentConfig()[i + 3]
        i += 1

    def getHandConfig (self, side, conf):
      q = []
      if side == "left":
        if conf == "open":
          q = self.getLeftHandOpenConfig()
        elif conf == "closed":
          q = self.getLeftHandClosedConfig()
      elif side == "right":
        if conf == "open":
          q = self.getRightHandOpenConfig()
        elif conf == "closed":
          q = self.getRightHandClosedConfig()
      elif side == "both":
        if conf == "open":
          q = self.getRightHandOpenConfig()
          self.setCurrentConfig(q)
          q = self.getLeftHandOpenConfig()
        elif conf == "closed":
          q = self.getRightHandClosedConfig()
          self.setCurrentConfig(q)
          q = self.getLeftHandClosedConfig()
      return q


    def getLeftHandClosedConfig (self):
        q = self.getCurrentConfig()
        for n in self.jointNames:
          if (n in self.leftHandClosed):
            q[self.jointNames.index(n) + 3] = self.leftHandClosed[n]
        return q

    def getLeftHandOpenConfig (self):
        q = self.getCurrentConfig()
        for n in self.jointNames:
          if (n in self.leftHandOpen):
            q[self.jointNames.index(n) + 3] = self.leftHandOpen[n]
        return q

    def getRightHandClosedConfig (self):
        q = self.getCurrentConfig()
        for n in self.jointNames:
          if (n in self.rightHandClosed):
            q[self.jointNames.index(n) + 3] = self.rightHandClosed[n]
        return q

    def getRightHandOpenConfig (self):
        q = self.getCurrentConfig()
        for n in self.jointNames:
          if (n in self.rightHandOpen):
            q[self.jointNames.index(n) + 3] = self.rightHandOpen[n]
        return q
