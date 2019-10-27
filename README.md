# ROS_QuadruptedRobot
simulate quadrupted Robot in ROS + Gazebo with reinforcement learning
<img src="https://github.com/teddy-ssy/ROS_QuadruptedRobot/blob/master/readme/bot1.jpeg">

|project start date|6/ 1/ 2019|
|:---:|:---:|

This is a personal practice project, for my own intersting, I hope to create a quadrupted robot with reinforment learning method. after review the article of the reinforment learning method, and open source project, especially the preactice of OpenAI. I start this project.
the project based on the Unbunt system, the reinforcment learning method is implment with tensorflow, I design the reward sysem, the hardware driver, and tested the joint driver. I found, the static movment is better for the high deminsaion action robot. the current project can only preform as the preview in the next part.

## Dependency Packages for Chatbox
|package|version|
|:---:|:---:|
|openAI||
|tensorflow|1.13.1|
|Gazebo||
|ROS||
|Unbunt||

## Preview

![the simulation for joint moving](https://github.com/teddy-ssy/ROS_QuadruptedRobot/blob/master/readme/demo/balance and motion.mp4)
morion plan

![the simulation for joint moving](https://github.com/teddy-ssy/ROS_QuadruptedRobot/blob/master/readme/demo/laser.mp4)
laser with motion
  
## Architecture
<img src="https://github.com/teddy-ssy/ROS_QuadruptedRobot/blob/master/readme/code%20structue.png">
<pdf src="https://github.com/teddy-ssy/ROS_QuadruptedRobot/blob/master/readme/urdf structure/hyq.pdf">
  <pdf src="https://github.com/teddy-ssy/ROS_QuadruptedRobot/blob/master/readme/transform info/frames.pdf">


## feature

### URDF

### reward function

### joint publisher 

### hardware publisher

### DDPG

## Roadmap

- [x] **20/1/2019** 
    - ~~start project~~
- [x] **1/2/2019**
    - ~~finish URDF~~
- [x] **20/2/2019**
    - ~~finish reward system~~
- [x] **3/4/2019**
    - ~~test Q learning~~
- [x]**25/5/2019**
    - ~~test DDPG with experience replay~~



## Discussion
- [submit issue](https://github.com/teddy-ssy/ROS_QuadruptedRobot/issues/new)

