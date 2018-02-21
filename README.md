# ROS_Projects
The asl_turtlebot contains two programs for running a robot simulation using Gazebo. 

The first program uses an control law developed through optimal control theory to move
the robot from a given desired postion to a goal position. It can be run with the 
following commands: 
 $ roslaunch asl_turtlebot turtlebot3_sim.launch 
and in another terminal 
 $ rosrun asl_turtlebot controller.py 


The second program incorporates computer vision into the turtlebot so that it can detect
a stop sign. Using a state machine, the bot stops in front of the stop sign for 3 seconds
before continuing forward. The program can be run with the following commands: 
 $ roslaunch asl_turtlebot turtlebot3_signs.launch 
 and in another terminal 
 $ rosrun asl_turtlebot supervisor.py 


These programs were part of an assignment for AA274 (Principles of Robotic Autonomy) at Stanford. 

The code that I was responsible for writing was most of controller.py and supervisor.py
along with small parts of pose_controller.py and detector.py, which can be found  under the scripts directory. 
For the last two, sections of code have been commented for which I wrote.   
