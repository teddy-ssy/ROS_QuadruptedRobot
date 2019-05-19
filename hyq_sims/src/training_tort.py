#!/usr/bin/env python

import gym
import time
import numpy
import random
import DDPG
import hyq_env
from gym import wrappers
from std_msgs.msg import Float64
# ROS packages required
import rospy
import rospkg
import math

# import our training environment

ON_TRAIN = True


# Create the Gym environment
env = gym.make('Hyq-v0')
rospy.logdebug("Gym environment done")
reward_pub = rospy.Publisher('/hyq/reward', Float64, queue_size=1)
episode_reward_pub = rospy.Publisher('hyq/episode_reward', Float64, queue_size=1)
addr1 = 'tort1'
addr2 = 'tort2'


def train():

    last_time_steps = numpy.ndarray(0)
    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    LR_A = rospy.get_param("/LR_A")
    LR_C = rospy.get_param("/LR_C")
    GAMMA = rospy.get_param("/GAMMA")
    TAU = rospy.get_param("/TAU")
    MEMORY_CAPACITY = rospy.get_param("/MEMORY_CAPACITY")
    BATCH_SIZE = rospy.get_param("/BATCH_SIZE")
    MAX_EPSIODES = rospy.get_param("/MAX_EPSIODES")
    MAX_STEPS = rospy.get_param("/MAX_STEPS")

    A_DIM =env.action_space.n#24
    S_DIM =20#20
    A_BOUND = [-1.57, 1.57]
    # Initialises the algorithm that we are going to use for learning
    #qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                           #alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
    ddpg1 = DDPG.DDPG(a_dim=A_DIM, s_dim=S_DIM, a_bound=A_BOUND, MEMORY_CAPACITY=MEMORY_CAPACITY, TAU=TAU, GAMMA=GAMMA, LR_A = LR_A, LR_C=LR_C)
    ddpg2 = DDPG.DDPG(a_dim=A_DIM, s_dim=S_DIM, a_bound=A_BOUND, MEMORY_CAPACITY=MEMORY_CAPACITY, TAU=TAU, GAMMA=GAMMA, LR_A = LR_A, LR_C=LR_C)
    #initial_epsilon = qlearn.epsilon

    start_time = time.time()
    D1_highest_reward = 0
    D2_highest_reward = 0

    # Starts the main training loop: the one about the episodes to do
    for x in range(MAX_EPSIODES):
        rospy.loginfo("STARTING Episode #" + str(x))

        D1_cumulated_reward = 0
        D1_cumulated_reward_msg = Float64()
        D2_cumulated_reward = 0
        D2_cumulated_reward_msg = Float64()
        episode_reward_msg = Float64()
        done = False
        #if qlearn.epsilon > 0.05:
            #qlearn.epsilon *= epsilon_discount

        # Initialize the environment and get first state of the robot
        rospy.logdebug("env.reset...")
        # Now We return directly the stringuified observations called state
        state = env.reset()

        #print "state =" + str(state)

        rospy.logdebug("env.get_state...==>" + str(state))

        # for each episode, we test the robot for nsteps
        i= 0
        DONE =False
        while not(DONE):



            #print "step "+ str(i)

            # Pick an action based on the current state
            #action = qlearn.chooseAction(state)

            if mode ==0:
                action = ddpg1.choose_action(state)
                action =[action[0], action[1], 0.0,
                        0.0, 3.14, -3.14,
                        0.0, -3.14, 3.14,
                        action[2], action[3], 0.0]
                rospy.logdebug("###################### Start Step...[" + str(i) + "]")
                rospy.logdebug("haa+,haa-,hfe+,hfe-,kfe+,kfe- >> [0,1,2,3,4,5]")
                rospy.logdebug("Action to Perform >> " + str(action))
                nextState, reward, done, info = env.step(action)
                ddpg1.store_transition(state, action, reward, nextState, MEMORY_CAPACITY)

                cumulated_reward += reward
                if D1_highest_reward < D1_cumulated_reward:
                    D1_highest_reward = D1_cumulated_reward
                mode =1
                rate = rospy.Rate(int(action[4]))
                rate.sleep()
            elif mode ==1:
                action =[0.0, 0.8, -0.8,
                        0.0, 0.8, -0.8, 
                        0.0, 0.8, -0.8, 
                        0.0, 0.8, -0.8]
                nextstate, reward, done, info = env.step(action)
                mode =2
                rate = rospy.Rate(2)
                rate.sleep()

            elif mode ==2:
                action = ddpg2.choose_action(state)
                action =[0.0, 1.6, -1.6,
                        action[0], action[1], 0.0,
                        action[2], action[3], 0.0,
                        0.0, -3.14, 3.14]
                nextstate, reward, done, info = env.step(action)
                ddpg2.store_transition(state, action, reward, nextState, MEMORY_CAPACITY)

                cumulated_reward += reward
                if D2_highest_reward < D2_cumulated_reward:
                    D2_highest_reward = D2_cumulated_reward
                mode =3
                rate = rospy.Rate(int(action[4]))
                rate.sleep()
            elif mode ==3:
                action =[0.0, 1.57, -1.57,
                        0.0, 1.57, -1.57, 
                        0.0, -1.57, 1.57, 
                        0.0, -1.57, 1.57]
                nextstate, reward, done, info = env.step(action)
                mode =0
                rate = rospy.Rate(2)
                rate.sleep()
            
            rospy.logdebug("END Step...")
            rospy.logdebug("Reward ==> " + str(reward))
            

            rospy.logdebug(
                "env.get_state...[distance_from_desired_point,base_roll,base_pitch,base_yaw,contact_force,joint_states_haa,joint_states_hfe,joint_states_kfe]==>" + str(
                    nextState))

            if ddpg1.memory_full:
                ddpg1.learn(MEMORY_CAPACITY, BATCH_SIZE)
            if ddpg2.memory_full:
                ddpg2.learn(MEMORY_CAPACITY, BATCH_SIZE)
            # Make the algorithm learn based on the results
            #qlearn.learn(state, action, reward, nextState)

            # We publish the cumulated reward
            D1_cumulated_reward_msg.data = reward
            D1_reward_pub.publish(cumulated_reward_msg)
            D2_cumulated_reward_msg.data = reward
            D2_reward_pub.publish(cumulated_reward_msg)

            #print "train2" + str(done)

            if not (done):
                state = nextState
                #rospy.loginfo("NOT DONE")
            else:
                rospy.logdebug("DONE")
                rospy.loginfo("DONE")
                rospy.loginfo("STEP" + str(i))
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break

            rospy.logdebug("###################### END Step...[" + str(i) + "]")
            i = i+1

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        episode_reward_msg.data = reward
        episode_reward_pub.publish(episode_reward_msg)
        rospy.loginfo(("EP: " + str(x + 1) + " - epsilon: " + str(round(x)) + "] - Reward: " + str(
            reward) + "     Time: %d:%02d:%02d" % (h, m, s)))

    ddpg1.save(addr1)
    ddpg2.save(addr2)

    rospy.loginfo(("\n|" + str(MAX_EPSIODES) + "|" + str(D1_highest_reward) + "| PICTURE |"))
    rospy.loginfo(("\n|" + str(MAX_EPSIODES) + "|" + str(D2_highest_reward) + "| PICTURE |"))

    l = last_time_steps.tolist()
    l.sort()

    rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
    rospy.loginfo("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))

def train02():


    last_time_steps = numpy.ndarray(0)
    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    LR_A = rospy.get_param("/LR_A")
    LR_C = rospy.get_param("/LR_C")
    GAMMA = rospy.get_param("/GAMMA")
    TAU = rospy.get_param("/TAU")
    MEMORY_CAPACITY = rospy.get_param("/MEMORY_CAPACITY")
    BATCH_SIZE = rospy.get_param("/BATCH_SIZE")
    MAX_EPSIODES = rospy.get_param("/MAX_EPSIODES")
    MAX_STEPS = rospy.get_param("/MAX_STEPS")

    A_DIM =5#24
    S_DIM =20#20
    A_BOUND = [-1.57, 1.57]
    # Initialises the algorithm that we are going to use for learning
    #qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                           #alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
    ddpg = DDPG.DDPG(a_dim=A_DIM, s_dim=S_DIM, a_bound=A_BOUND, MEMORY_CAPACITY=MEMORY_CAPACITY, TAU=TAU, GAMMA=GAMMA, LR_A = LR_A, LR_C=LR_C)
    #initial_epsilon = qlearn.epsilon

    start_time = time.time()
    highest_reward = 0

    # Starts the main training loop: the one about the episodes to do
    for x in range(MAX_EPSIODES):
        rospy.loginfo("STARTING Episode #" + str(x))

        cumulated_reward = 0
        cumulated_reward_msg = Float64()
        episode_reward_msg = Float64()
        done = False
        #if qlearn.epsilon > 0.05:
            #qlearn.epsilon *= epsilon_discount

        # Initialize the environment and get first state of the robot
        rospy.logdebug("env.reset...")
        # Now We return directly the stringuified observations called state
        state = env.reset()

        #print "state =" + str(state)

        rospy.logdebug("env.get_state...==>" + str(state))

        # for each episode, we test the robot for nsteps
        i= 0
        DONE =False
        mode =0
        while not(DONE):
            actionlist=[]
            #action = ddpg.choose_action(state)
            rospy.logdebug("###################### Start Step...[" + str(i) + "]")
            rospy.logdebug("haa+,haa-,hfe+,hfe-,kfe+,kfe- >> [0,1,2,3,4,5]")
            #rospy.logdebug("Action to Perform >> " + str(action))
            if mode ==0:
                action = ddpg.choose_action(state)
                actionpos =[action[0], action[1], 0.0,
                        0.0, 1.6, -1.6,
                        0.0, 1.6, -1.6,
                        action[2], action[3], 0.0]
                actionlist.append(actionpos)
                actionlist.append(math.fabs(action[4]))

                nextState, reward, done, info = env.step(actionlist)
                mode =1
                
            elif mode ==1:
                action = ddpg.choose_action(state)
                actionpos =[0.0, 1.6, -1.6,
                        action[0], action[1], 0.0,
                        action[2], action[3], 0.0,
                        0.0, 1.6, -1.6]
                actionlist.append(actionpos)
                actionlist.append(math.fabs(action[4]))
                nextState, reward, done, info = env.step(actionlist)
                mode =0

            DONE =done
            ddpg.store_transition(state, action, reward, nextState, MEMORY_CAPACITY)
            rospy.logdebug("END Step...")
            rospy.logdebug("Reward ==> " + str(reward))
            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            rospy.logdebug(
                "env.get_state...[distance_from_desired_point,base_roll,base_pitch,base_yaw,contact_force,joint_states_haa,joint_states_hfe,joint_states_kfe]==>" + str(
                    nextState))

            if ddpg.memory_full:
                ddpg.learn(MEMORY_CAPACITY, BATCH_SIZE)
            # Make the algorithm learn based on the results
            #qlearn.learn(state, action, reward, nextState)

            # We publish the cumulated reward
            cumulated_reward_msg.data = cumulated_reward
            reward_pub.publish(cumulated_reward_msg)

            #print "train2" + str(done)

            if not (done):
                state = nextState
                #rospy.loginfo("NOT DONE")
            else:
                rospy.logdebug("DONE")
                rospy.loginfo("DONE")
                rospy.loginfo("STEP" + str(i))
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break

            rospy.logdebug("###################### END Step...[" + str(i) + "]")
            i = i+1
            #time.sleep(1)

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        episode_reward_msg.data = cumulated_reward
        episode_reward_pub.publish(episode_reward_msg)
        rospy.loginfo(("EP: " + str(x + 1) + " - epsilon: " + str(round(x)) + "] - Reward: " + str(
            cumulated_reward) + "     Time: %d:%02d:%02d" % (h, m, s)))

    ddpg.save(addr1)

    rospy.loginfo(("\n|" + str(MAX_EPSIODES) + "|" + str(highest_reward) + "| PICTURE |"))

    l = last_time_steps.tolist()
    l.sort()

    rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
    rospy.loginfo("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))



def evalation():
    
    last_time_steps = numpy.ndarray(0)
    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    LR_A = rospy.get_param("/LR_A")
    LR_C = rospy.get_param("/LR_C")
    GAMMA = rospy.get_param("/GAMMA")
    TAU = rospy.get_param("/TAU")
    MEMORY_CAPACITY = rospy.get_param("/MEMORY_CAPACITY")
    BATCH_SIZE = rospy.get_param("/BATCH_SIZE")
    MAX_EPSIODES = rospy.get_param("/MAX_EPSIODES")
    MAX_STEPS = rospy.get_param("/MAX_STEPS")

    A_DIM =5#24
    S_DIM =20#20
    A_BOUND = 1.57
    # Initialises the algorithm that we are going to use for learning
    #qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                           #alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
    ddpg = DDPG.DDPG(a_dim=A_DIM, s_dim=S_DIM, a_bound=A_BOUND, MEMORY_CAPACITY=MEMORY_CAPACITY, TAU=TAU, GAMMA=GAMMA, LR_A = LR_A, LR_C=LR_C)
    #ddpg2 = DDPG.DDPG(a_dim=A_DIM, s_dim=S_DIM, a_bound=A_BOUND, MEMORY_CAPACITY=MEMORY_CAPACITY, TAU=TAU, GAMMA=GAMMA, LR_A = LR_A, LR_C=LR_C)
    #initial_epsilon = qlearn.epsilon

    start_time = time.time()  

    ddpg.restore(addr1)
    #ddpg2.restore(addr2)

    state = env.reset()

    while True:
        actionlist=[]

        #actionlist.clear()
        if mode ==0:
            action = ddpg.choose_action(state)
            actionpos =[action[0], action[1], 0.0,
                    0.0, 1.6, -1.6,
                    0.0, 1.6, -1.6,
                    action[2], action[3], 0.0]
            actionlist.append(actionpos)
            actionlist.append(math.fabs(action[4]))

            state, reward, done, info = env.step(actionlist)
            mode =1
            
        elif mode ==1:
            action = ddpg.choose_action(state)
            actionpos =[0.0, 1.6, -1.6,
                    action[0], action[1], 0.0,
                    action[2], action[3], 0.0,
                    0.0, 1.6, -1.6]
            actionlist.append(actionpos)
            actionlist.append(math.fabs(action[4]))
            state, reward, done, info = env.step(actionlist)
            mode =0


def evalation02():
    last_time_steps = numpy.ndarray(0)
    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    LR_A = rospy.get_param("/LR_A")
    LR_C = rospy.get_param("/LR_C")
    GAMMA = rospy.get_param("/GAMMA")
    TAU = rospy.get_param("/TAU")
    MEMORY_CAPACITY = rospy.get_param("/MEMORY_CAPACITY")
    BATCH_SIZE = rospy.get_param("/BATCH_SIZE")
    MAX_EPSIODES = rospy.get_param("/MAX_EPSIODES")
    MAX_STEPS = rospy.get_param("/MAX_STEPS")

    A_DIM =5#24
    S_DIM =20#20
    A_BOUND = 1.57
    # Initialises the algorithm that we are going to use for learning
    #qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                           #alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
    ddpg = DDPG.DDPG(a_dim=A_DIM, s_dim=S_DIM, a_bound=A_BOUND, MEMORY_CAPACITY=MEMORY_CAPACITY, TAU=TAU, GAMMA=GAMMA, LR_A = LR_A, LR_C=LR_C)
    #initial_epsilon = qlearn.epsilon

    start_time = time.time()  

    ddpg.restore(addr1)

    state = env.reset()

    while True:
        action = ddpg.choose_action(state)
        state, reward, done, info = env.step(action)


if __name__ == '__main__':

    rospy.init_node('hyq_gym', anonymous=True, log_level=rospy.INFO)

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('hyq_sims')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.logdebug("Monitor Wrapper started")

    if ON_TRAIN:
        #train()
        train02()
    else:
        #evalation()
        evalation02()

    env.close()