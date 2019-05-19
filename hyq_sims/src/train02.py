#!/usr/bin/env python

'''
    Original Training code made by Ricardo Tellez <rtellez@theconstructsim.com>
    Moded by Miguel Angel Rodriguez <duckfrost@theconstructsim.com>
    Visit our website at www.theconstructsim.com
'''
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

# import our training environment

ON_TRAIN = True


# Create the Gym environment
env = gym.make('Hyq-v0')
rospy.logdebug("Gym environment done")
reward_pub = rospy.Publisher('/hyq/reward', Float64, queue_size=1)
episode_reward_pub = rospy.Publisher('hyq/episode_reward', Float64, queue_size=1)





def training_01():

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
        while not(DONE):

            #print "step "+ str(i)

            # Pick an action based on the current state
            #action = qlearn.chooseAction(state)
            action = ddpg.choose_action(state)

            #print "action =" + str(action)

            # Execute the action in the environment and get feedback
            rospy.logdebug("###################### Start Step...[" + str(i) + "]")
            rospy.logdebug("haa+,haa-,hfe+,hfe-,kfe+,kfe- >> [0,1,2,3,4,5]")
            rospy.logdebug("Action to Perform >> " + str(action))
            nextState, reward, done, info = env.step(action)
            DONE=done
            
            #print "train1" + str(done)
            #print "nextstate =" + str(nextState)
            #print "reward =" + str(reward)

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
            cumulated_reward_msg.data = reward
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

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        episode_reward_msg.data = reward
        episode_reward_pub.publish(episode_reward_msg)
        rospy.loginfo(("EP: " + str(x + 1) + " - epsilon: " + str(round(x)) + "] - Reward: " + str(
            reward) + "     Time: %d:%02d:%02d" % (h, m, s)))

    ddpg.save()

    rospy.loginfo(("\n|" + str(MAX_EPSIODES) + "|" + str(highest_reward) + "| PICTURE |"))

    l = last_time_steps.tolist()
    l.sort()

    rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
    rospy.loginfo("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))


def eval_01():
    
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
    A_BOUND = 1.57
    # Initialises the algorithm that we are going to use for learning
    #qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                           #alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
    ddpg = DDPG.DDPG(a_dim=A_DIM, s_dim=S_DIM, a_bound=A_BOUND, MEMORY_CAPACITY=MEMORY_CAPACITY, TAU=TAU, GAMMA=GAMMA, LR_A = LR_A, LR_C=LR_C)
    #initial_epsilon = qlearn.epsilon

    start_time = time.time()  

    ddpg.restore()

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
        training_01()
    else:
        eval_01()

    env.close()