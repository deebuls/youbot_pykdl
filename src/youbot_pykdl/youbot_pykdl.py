#!/usr/bin/python

# Copyright (c) 2013-2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import numpy as np
import PyKDL

import rospy

#import youbot_interface

from youbot_kdl.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF

class youbot_kinematics(object):
    """
    Youbot Kinematics with PyKDL
    """
    def __init__(self ,urdf=None):
        if urdf is None:
            print 'FROM PARAM SERVER'
            self._youbot = URDF.from_parameter_server(key='robot_description')
        else:
            print 'FROM STRING'
            self._youbot = URDF.from_xml_string(urdf)

        self._kdl_tree = kdl_tree_from_urdf_model(self._youbot)
        self._base_link = 'arm_link_0'
        print "ROOT : ",self._base_link
        self._tip_link = 'arm_link_5'
        print "self._tip_link : ", self._tip_link
        self._tip_frame = PyKDL.Frame()
        self._arm_chain = self._kdl_tree.getChain(self._base_link,
                                                  self._tip_link)

        # Baxter Interface Limb Instances
        #self._limb_interface = youbot_interface.Limb(limb)
        #self._joint_names = self._limb_interface.joint_names()
        self._joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
        self._num_jnts = len(self._joint_names)

        # KDL Solvers
        self._fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(self._arm_chain)
        self._fk_v_kdl = PyKDL.ChainFkSolverVel_recursive(self._arm_chain)
        self._ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self._arm_chain)
        self._ik_p_kdl = PyKDL.ChainIkSolverPos_NR(self._arm_chain,
                                                   self._fk_p_kdl,
                                                   self._ik_v_kdl)
        self._jac_kdl = PyKDL.ChainJntToJacSolver(self._arm_chain)
        self._dyn_kdl = PyKDL.ChainDynParam(self._arm_chain,
                                            PyKDL.Vector.Zero())

    def print_robot_description(self):
        nf_joints = 0
        for j in self._youbot.joints:
            if j.type != 'fixed':
                nf_joints += 1
        print "URDF non-fixed joints: %d;" % nf_joints
        print "URDF total joints: %d" % len(self._youbot.joints)
        print "URDF links: %d" % len(self._youbot.links)
        print "KDL joints: %d" % self._kdl_tree.getNrOfJoints()
        print "KDL segments: %d" % self._kdl_tree.getNrOfSegments()

    def print_kdl_chain(self):
        for idx in xrange(self._arm_chain.getNrOfSegments()):
            print '* ' + self._arm_chain.getSegment(idx).getName()

    def joints_to_kdl(self, type, joints_val=None):
        kdl_array = PyKDL.JntArray(self._num_jnts)
        print "Not Implemented "
        print "Interface with the youbot driver not implemented"
        print "Please provide the Joint values (position, velocity ,efforts) in api"       

        '''
        if type == 'positions':
            cur_type_values = self._limb_interface.joint_angles()
        elif type == 'velocities':
            cur_type_values = self._limb_interface.joint_velocities()
        elif type == 'torques':
            cur_type_values = self._limb_interface.joint_efforts()
        '''
        if type == 'velocities':
            kdl_array = PyKDL.JntArrayVel(kdl_array)
        return kdl_array

    def kdl_to_mat(self, data):
        mat =  np.mat(np.zeros((data.rows(), data.columns())))
        for i in range(data.rows()):
            for j in range(data.columns()):
                mat[i,j] = data[i,j]
        return mat

    def forward_position_kinematics(self, joints_val=None):
        end_frame = PyKDL.Frame()
        # Populate seed with current angles if not provided
        seed_array = PyKDL.JntArray(self._num_jnts)
        if joints_val != None:
            seed_array.resize(len(joints_val))
            for idx, jnt in enumerate(joints_val):
                seed_array[idx] = jnt
        else:
            seed_array = self.joints_to_kdl('positions')

        self._fk_p_kdl.JntToCart(seed_array, end_frame)
        pos = end_frame.p
        rot = PyKDL.Rotation(end_frame.M)
        rot = rot.GetQuaternion()
        return np.array([pos[0], pos[1], pos[2],
                         rot[0], rot[1], rot[2], rot[3]])

    def forward_velocity_kinematics(self, joints_val=None):
        end_frame = PyKDL.FrameVel()
        # Populate seed with current angles if not provided
        seed_array = PyKDL.JntArray(self._num_jnts)
        if joints_val != None:
            seed_array.resize(len(joints_val))
            for idx, jnt in enumerate(joints_val):
                seed_array[idx] = jnt
            seed_array = PyKDL.JntArrayVel(seed_array)
        else:
            seed_array = self.joints_to_kdl('velocities')

        self._fk_v_kdl.JntToCart(seed_array, end_frame)
        return end_frame.GetTwist()

    def inverse_kinematics(self, position, orientation=None, seed=None):
        ik = PyKDL.ChainIkSolverVel_pinv(self._arm_chain)
        pos = PyKDL.Vector(position[0], position[1], position[2])
        if orientation != None:
            rot = PyKDL.Rotation()
            rot = rot.Quaternion(orientation[0], orientation[1],
                                 orientation[2], orientation[3])
        # Populate seed with current angles if not provided
        seed_array = PyKDL.JntArray(self._num_jnts)
        if seed != None:
            seed_array.resize(len(seed))
            for idx, jnt in enumerate(seed):
                seed_array[idx] = jnt
        else:
            seed_array = self.joints_to_kdl('positions')

        # Make IK Call
        if orientation:
            goal_pose = PyKDL.Frame(rot, pos)
        else:
            goal_pose = PyKDL.Frame(pos)
        result_angles = PyKDL.JntArray(self._num_jnts)

        if self._ik_p_kdl.CartToJnt(seed_array, goal_pose, result_angles) >= 0:
            result = np.array(result_angles)
            return result
        else:
            print 'No IK Solution Found'
            return None

    def jacobian(self, seed=None):
        jacobian = PyKDL.Jacobian(self._num_jnts)
        # Populate seed with current angles if not provided
        seed_array = PyKDL.JntArray(self._num_jnts)
        if seed != None:
            seed_array.resize(len(seed))
            for idx, jnt in enumerate(seed):
                seed_array[idx] = jnt
        else:
            seed_array = self.joints_to_kdl('positions')

        self._jac_kdl.JntToJac(seed_array, jacobian)
        return self.kdl_to_mat(jacobian)

    def jacobian_transpose(self, seed=None):
        return self.jacobian(seed).T

    def jacobian_pseudo_inverse(self, seed=None):
        return np.linalg.pinv(self.jacobian(seed))


    def inertia(self, seed=None):
        inertia = PyKDL.JntSpaceInertiaMatrix(self._num_jnts)
        # Populate seed with current angles if not provided
        seed_array = PyKDL.JntArray(self._num_jnts)
        if seed != None:
            seed_array.resize(len(seed))
            for idx, jnt in enumerate(seed):
                seed_array[idx] = jnt
        else:
            seed_array = self.joints_to_kdl('positions')

        self._dyn_kdl.JntToMass(seed_array, inertia)
        return self.kdl_to_mat(inertia)

    def cart_inertia(self, seed=None):
        js_inertia = self.inertia(seed)
        jacobian = self.jacobian(seed)
        return np.linalg.inv(jacobian * np.linalg.inv(js_inertia) * jacobian.T)
