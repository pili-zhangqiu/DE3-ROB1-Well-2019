#!/usr/bin/env python
# Copyright (c) 2013-2015, Rethink Robotics
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

"""
Baxter RSDK Inverse Kinematics Pick and Place Demo
"""
import argparse
import os
import struct
import sys
import copy
import math
import time
from utils import * 
import numpy as np

from tf.transformations import *

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True): 
        self._limb_name = limb # limb has to be string 
        self._hover_distance = hover_distance
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService" # initialise IKService for the given limb
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION) # enable De Niro
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None): 
		# takes a dictionary of joint angles, position the limb to those joint angles
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose): 
		# takes a cartesian pose to be positioned, returns a dicionary of joint angles
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles): 
		# moves the joints to the given joint angles if no errors.
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self): 
		# open the gripper
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
		# close the gripper
        self._gripper.close()
        rospy.sleep(1.0)
	
    def _approach(self, pose):
		# approach with a pose with the current retracted z-position so that it moves in a straight line above the bricks to avoid collisions
        approach = copy.deepcopy(pose)
    	approach.position.z = self._limb.endpoint_pose()['position'].z
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose): 
		# function to move directly to a desired pose
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose): 
		# function to pick a brick up at a specific pose
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
		
		#Detects if the gripper action actually succeeds in gripping an object
        gripped = False #Boolean for whether the gripper grips an object
        gripcount = 0 #count for how many grip attempts were done.
        while gripped == False: 
            print("Failed to grip, trying again...")
            gripcount += 1

			if self._gripper.position() < 4.5 and gripcount < 2: 
				#If the brick is picked up, the position will be a certain threshold value. If it is not picked up, the position will be very small.
				#So if the gripper is below the threshold value, it failed to grip a brick, so retry gripping. Also only try a certain number of times.
                self.gripper_open()
                rospy.sleep(3)
                self.gripper_close()
                rospy.sleep(1)

            else:
                gripped = True
		#after picking up, raise the end effector vertically.
        self._retract()
	
    def place(self, pose):
		# function to place the brick
        start = self._limb.endpoint_pose()
        start = [start['position'].x,start['position'].y,start['position'].z]
		# path planning start location is the current gripper location
        end = [pose.position.x, pose.position.y, pose.position.z]
		# path planning end location is pose
        self._approach(pose, to_spawn=False)
        path = plan_path(start, end)
        # generate a set of stopping points

        for i in range(1,len(path)):
            stop_point = Pose(position=Point(x=path[i,0],y=path[i,1],z=path[i,2]),orientation=pose.orientation)
            self._servo_to_pose(stop_point)
			# stop at these stop points for better path, avoids colliding with other bricks
			
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

 #Functions to load the table and bricks in gazebo. Only required for simulations

def load_gazebo_models(table_pose=Pose(position=Point(x=0.8, y=0.4, z=0.0)), 
                       table_reference_frame="world",
                       brick_pose=Pose(position=Point(x=0.7, y=0, z=0.8210)),
                       brick_reference_frame="world"):
    # load models of the table and the brick at the given cartesian positions
	
	# Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
    brick_xml = ''
    with open (model_path + "block/model.urdf", "r") as brick_file:
        brick_xml=brick_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF with the spawn_urdf_model service
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("brick", brick_xml, "/",
                               brick_pose, brick_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_brick(idx,brick_pose=Pose(position=Point(x=0.7, y=0, z=0.8210)),brick_reference_frame="world"): 
	#Brick name is unique and is equal to idx which iterates from 0 to 14
	
	model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
	brick_xml = ''
	with open (model_path + "block/model.urdf", "r") as brick_file:
		brick_xml=brick_file.read().replace('\n', '')
    # Spawn Table SDF
	rospy.wait_for_service('/gazebo/spawn_sdf_model')
	try:
		spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
		resp_urdf = spawn_urdf("%s" %(idx), brick_xml, "/",
                               brick_pose, brick_reference_frame)
	except rospy.ServiceException, e:
		rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def main():

    """RSDK Inverse Kinematics Pick and Place Example
    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Baxter will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    
	rospy.init_node("ik_pick_and_place_demo") # Initialise the node for subscribing to and utilising topics and services.

    brick_locations = calculate_brick_locations() # Execute the utility function to calculate the brick positions for the well.

    limb = 'left' # Select which limb to use
    hover_distance = 0.15 # Drop height of the brick in meters
	
    # Starting Joint angles for left arm
    starting_joint_angles = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': 0.41999602073,
                             'left_s1': -1.49997811669}
	
    pnp = PickAndPlace(limb, hover_distance) # Initialise the class for the limb = 'left' arm

    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011) # Quaternion array to change the orientation of the gripper to be parallel to the world xy plane
	
    focus_loc = Pose(
        position=Point(x=0.75, y=0, z=-0.04),
        orientation= overhead_orientation) # Position and orientation of the end-effector for taking the picture

    pnp.move_to_start(starting_joint_angles) # Move to an arbitrary start position defined in starting_joint_angles
	
	# load_gazebo_models() # for gazebo simulation, spawn table and the first brick
	
    for i in range(len(brick_locations)):
    	pose = Pose(
        position=Point(x=brick_locations[i,0], y=brick_locations[i,1], z=brick_locations[i,2]),
        orientation=Quaternion(x=brick_locations[i,3],y=brick_locations[i,4],z=brick_locations[i,5],w=brick_locations[i,6]))
		# remap pose to the position of the brick to be placed
        print("Placing brick ",i)
        print("Location to be placed:",pose)
        pnp.move_to(focus_loc)	# end-effector moves to picture-taking position

        os.system('python take_photo_l1.py') # use the camera in the arm to take a picture
        image=cv2.imread('/home/petar/catkin_ws/src/WELL/scripts/final/perception_test/l1.jpg') # Read the image from the specific directory that the image was saved in
		#MAKE SURE THAT THE DIRECTORY IS CORRECT.
        dx, dy, theta= brick_boi(image) # process the image to find the brick's offset relative to the picture frame
        os.system('rosrun baxter_examples xdisplay_image.py -f /home/petar/catkin_ws/src/WELL/scripts/final/perception_test/plot.png') # Put the processed image onto Baxter's screen
        

        orientation_orig = np.array([-0.0249590815779,0.999649402929,0.00737916180073,0.00486450832011]) # Same as original orientation in which the arm takes a picture
        orientation_new = quaternion_multiply(quaternion_from_euler(0,0,-theta),orientation_orig) # normalised orientation for picking up the brick with the brick offset
        spwan_loc= Pose(
        position=Point(x=0.75+dx, y=0.0+dy, z=-0.17), # Pick the brick at the original location + the brick's offset = the brick's current location
        orientation=Quaternion(x=orientation_new[0],y=orientation_new[1],z=orientation_new[2],w=orientation_new[3])) # normalise the end effector to the brick

    	pnp.pick(spwan_loc) # pick and place
    	pnp.place(pose)
    	#load_brick(i) # for gazebo simulation, spawn a new brick at spawn location

    return 0

if __name__ == '__main__':

    try:
        main()

    except KeyboardInterrupt:
        cleanupOnExit()
