# wall-following-ROS-Cpp
Control a two-wheels differential drive robot with a virtual car-like robot  

-x while doing a ROS implementation for a wall follower robot, I invented a method to control this two-wheel differential drive robot indirectly by a car-like modleing with steering and throttle rather than linear and angular command that used in two-wheels differential drive  

-x the idea is that using a car-like modeling with steering and throttle, it's much more flexibleto optimze and tune the controller  

-x when determined steering and throttle (speed) for the car-like robot, we can use kinematic modeling to calculate linear and angular speed needed to send to the two-wheels differential drive robot  

-x while applying the control on the car-like robot, we can implement many kind of advances in autonomous driving, for my  ROS project I tried with PID, with Stanley, to control cross-track-error and heading-error of my robot  

*youtube livestream : https://www.youtube.com/watch?v=h8XQrfBhjpU&ab_channel=BecomingaROSDeveloper  
**presentation : https://drive.google.com/file/d/1Go-H_I698mGTNF5yPivYcZZUdz6BSLz5/view  
***blog : https://www.nguyenrobot.com/autonomous-driving  

##Build  
catkin_make --only-package-with-deps find_wall_node  
catkin_make --only-package-with-deps odom_record_node  
catkin_make --only-package-with-deps follow_wall_node  

*dependencies : ROS Noetic, catkin  
