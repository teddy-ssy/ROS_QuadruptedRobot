#!/usr/bin/env python

import gym
import rospy
import numpy as np
import time
from gym import utils, spaces
from geometry_msgs.msg import Pose
from gym.utils import seeding
from gym.envs.registration import register
from gazebo_connection import GazeboConnection
from joint_publisher import JointPub
from hyq_state import HyqState
from controllers_connection import ControllersConnection

# register the training environment in the gym as an available one
reg = register(
    id='Hyq-v0',
    entry_point='hyq_env:HyqEnv',
    timestep_limit=1000,
)


class HyqEnv(gym.Env):

    def __init__(self):
        # We assume that a ROS node has already been created
        # before initialising the environment

        # gets training parameters from param server
        #self.step = 0

        self.desired_pose = Pose()
        self.desired_pose.position.x = rospy.get_param("/desired_pose/x")
        self.desired_pose.position.y = rospy.get_param("/desired_pose/y")
        self.desired_pose.position.z = rospy.get_param("/desired_pose/z")
        self.running_step = rospy.get_param("/running_step")
        self.max_incl = rospy.get_param("/max_incl")
        self.max_height = rospy.get_param("/max_height")
        self.min_height = rospy.get_param("/min_height")
        self.joint_increment_value = rospy.get_param("/joint_increment_value")
        self.done_reward = rospy.get_param("/done_reward")
        self.alive_reward = rospy.get_param("/alive_reward")
        self.fail_reward = rospy.get_param("/fail_reward")
        self.desired_force = rospy.get_param("/desired_force")
        self.desired_yaw = rospy.get_param("/desired_yaw")

        self.weight_r1 = rospy.get_param("/weight_r1")
        self.weight_r2 = rospy.get_param("/weight_r2")
        self.weight_r3 = rospy.get_param("/weight_r3")
        self.weight_r4 = rospy.get_param("/weight_r4")
        self.weight_r5 = rospy.get_param("/weight_r5")

        # stablishes connection with simulator
        self.gazebo = GazeboConnection()

        self.controllers_object = ControllersConnection(namespace="hyq")

        self.hyq_state_object = HyqState(max_height=self.max_height,
                                                 min_height=self.min_height,
                                                 abs_max_roll=self.max_incl,
                                                 abs_max_pitch=self.max_incl,
                                                 abs_max_yaw=self.max_incl,
                                                 joint_increment_value=self.joint_increment_value,
                                                 done_reward=self.done_reward,
                                                 alive_reward=self.alive_reward,
                                                 fail_reward = self.fail_reward,
                                                 desired_force=self.desired_force,
                                                 desired_yaw=self.desired_yaw,
                                                 weight_r1=self.weight_r1,
                                                 weight_r2=self.weight_r2,
                                                 weight_r3=self.weight_r3,
                                                 weight_r4=self.weight_r4,
                                                 weight_r5=self.weight_r5
                                                 )

        self.hyq_state_object.set_desired_world_point(self.desired_pose.position.x,
                                                          self.desired_pose.position.y,
                                                          self.desired_pose.position.z)

        self.hyq_joint_pubisher_object = JointPub()

        """
        For this version, we consider 6 actions
        1-2) Increment/Decrement haa_joint
        3-4) Increment/Decrement hfe_joint
        5-6) Increment/Decrement kfe_joint
        """
        self.action_space = spaces.Discrete(12)
        self.reward_range = (-np.inf, np.inf)

        self.state_space = len(self.hyq_state_object._list_of_observations)

        self._seed()

    # A function to initialize the random generator
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # Resets the state of the environment and returns an initial observation.
    def _reset(self):
        # 0st: We pause the Simulator
        rospy.logdebug("Pausing SIM...")
        self.gazebo.pauseSim()

        # 1st: resets the simulation to initial values
        rospy.logdebug("Reset SIM...")
        self.gazebo.resetSim()

        # 2nd: We Set the gravity to 0.0 so that we dont fall when reseting joints
        # It also UNPAUSES the simulation
        rospy.logdebug("Remove Gravity...")
        self.gazebo.change_gravity(0.0, 0.0, 0.0)

        # EXTRA: Reset JoinStateControlers because sim reset doesnt reset TFs, generating time problems
        rospy.logdebug("reset_hyq_joint_controllers...")
        self.controllers_object.reset_hyq_joint_controllers()

        # 3rd: resets the robot to initial conditions
        rospy.logdebug("set_init_pose...")
        self.hyq_joint_pubisher_object.set_init_pose()

        # 5th: Check all subscribers work.
        # Get the state of the Robot defined by its RPY orientation, distance from
        # desired point, contact force and JointState of the three joints
        rospy.logdebug("check_all_systems_ready...")
        self.hyq_state_object.check_all_systems_ready()
        rospy.logdebug("get_observations...")
        observation = self.hyq_state_object.get_observations()

        # 6th: We restore the gravity to original
        rospy.logdebug("Restore Gravity...")
        self.gazebo.change_gravity(0.0, 0.0, -9.81)

        # 7th: pauses simulation
        rospy.logdebug("Pause SIM...")
        self.gazebo.pauseSim()

        # Get the State Discrete Stringuified version of the observations
        state = self.get_state(observation)
        #self.step = 0

        return state

    def _step(self, action):
        # Given the action selected by the learning algorithm,
        # we perform the corresponding movement of the robot

        # 1st, decide which action corresponsd to which joint is incremented
        #next_action_position = self.hyq_state_object.get_action_to_position(action)

        # We move it to that pos
        self.gazebo.unpauseSim()
        #print "next action position ==" + str(next_action_position)
        #self.hyq_joint_pubisher_object.move_joints(next_action_position)
        self.hyq_joint_pubisher_object.move_joints(action[0])
        # Then we send the command to the robot and let it go
        # for running_step seconds
        #time.sleep(action[1])
        time.sleep(self.running_step)
        self.gazebo.pauseSim()

        # We now process the latest data saved in the class state to calculate
        # the state and the rewards. This way we guarantee that they work
        # with the same exact data.
        # Generate State based on observations
        observation = self.hyq_state_object.get_observations()

        # finally we get an evaluation based on what happened in the sim
        reward, done = self.hyq_state_object.process_data()

        #print "hyqenv 1 "+ str(done)

        # Get the State Discrete Stringuified version of the observations
        state = self.get_state(observation)

        #print "hyqenv 2 "+ str(done)

        return state, reward, done, {}

    def get_state(self, observation):
        """
        We retrieve the Stringuified-Discrete version of the given observation
        :return: state
        """
        #return self.hyq_state_object.get_state_as_string(observation)
        return np.array(observation)
