#!/usr/bin/env python

import gym
import time
import numpy
import random
import Qlearn
from gym import wrappers
from std_msgs.msg import Float64
# ROS packages required
import rospy
import rospkg

# import our training environment
import hyq_env


if __name__ == '__main__':
    
    rospy.init_node('hyq_gym', anonymous=True, log_level=rospy.INFO)

    # Create the Gym environment
    env = gym.make('Hyq-v0')
    rospy.logdebug ( "Gym environment done")
    reward_pub = rospy.Publisher('/hyq/reward', Float64, queue_size=1)
    episode_reward_pub = rospy.Publisher('/hyq/episode_reward', Float64, queue_size=1)

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('hyq_sims')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.logdebug("Monitor Wrapper started")
    
    last_time_steps = numpy.ndarray(0)

    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    Alpha = rospy.get_param("/alpha")
    Epsilon = rospy.get_param("/epsilon")
    Gamma = rospy.get_param("/gamma")
    epsilon_discount = rospy.get_param("/epsilon_discount")
    nepisodes = rospy.get_param("/nepisodes")
    nsteps = rospy.get_param("/nsteps")

    # Initialises the algorithm that we are going to use for learning
    qlearn = Qlearn.QLearn(actions=range(env.action_space.n),
                    alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
    initial_epsilon = qlearn.epsilon

    start_time = time.time()
    highest_reward = 0
    
    # Starts the main training loop: the one about the episodes to do
    for x in range(nepisodes):
        rospy.loginfo ("STARTING Episode #"+str(x))
        
        cumulated_reward = 0
        cumulated_reward_msg = Float64()
        episode_reward_msg = Float64()
        done = False
        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount
        
        # Initialize the environment and get first state of the robot
        rospy.logdebug("env.reset...")
        # Now We return directly the stringuified observations called state
        state = env.reset()

        rospy.logdebug("env.get_state...==>"+str(state))
        #print "env.get_state...==>"+str(state)
        
        # for each episode, we test the robot for nsteps
        for i in range(nsteps):

            # Pick an action based on the current state
            action = qlearn.chooseAction(state)
            #print "action= "+ str(action)
            
            # Execute the action in the environment and get feedback
            rospy.logdebug("###################### Start Step...["+str(i)+"]")
            rospy.logdebug("haa+,haa-,hfe+,hfe-,kfe+,kfe- >> [0,1,2,3,4,5]")
            rospy.logdebug("Action to Perform >> "+str(action))
            #print "Action to Perform >> "+str(action)
            nextState, reward, done, info = env.step(action)
            #print "state= "+ str(nextState)
            #print "reward= "+ str(reward)
            rospy.logdebug("END Step...")
            rospy.logdebug("Reward ==> " + str(reward))
            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            rospy.logdebug("env.get_state...[distance_from_desired_point,base_roll,base_pitch,base_yaw,contact_force,joint_states_haa,joint_states_hfe,joint_states_kfe]==>" + str(nextState))

            # Make the algorithm learn based on the results
            qlearn.learn(state, action, reward, nextState)

            # We publish the cumulated reward
            cumulated_reward_msg.data = cumulated_reward
            reward_pub.publish(cumulated_reward_msg)

            if not(done):
                state = nextState
            else:
                rospy.logdebug ("DONE")
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break

            rospy.logdebug("###################### END Step...["+str(i)+"]")

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        episode_reward_msg.data = cumulated_reward
        episode_reward_pub.publish(episode_reward_msg)
        rospy.loginfo( ("EP: "+str(x+1)+" - [alpha: "+str(round(qlearn.alpha,2))+" - gamma: "+str(round(qlearn.gamma,2))+" - epsilon: "+str(round(qlearn.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s)))

    rospy.loginfo ( ("\n|"+str(nepisodes)+"|"+str(qlearn.alpha)+"|"+str(qlearn.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |"))

    l = last_time_steps.tolist()
    l.sort()

    rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
    rospy.loginfo("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))

    env.close()
