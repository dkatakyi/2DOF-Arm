Project: 2DOF Robot Arm
Source code: project.m
Programmers: Daniel Takyi, Ufuoma Aya

Built and programmed a robot arm with two dimension freedom. The arm has four 
functionalities.

1. Measure Distance
   Inside the robot working space, a user wants to measure the distance between 
   two points. The user moves the end effector to the first point, clicks a touch 
   sensor for recording the first point, then the user locates the end effector 
   over the second location and clicks for storing the second point. The user 
   gets as output of the distance between the two points and it is displayed in 
   the EV3 screen.

2. Measure Angle
   Inside the robot working space, a user wants to measure the angle between two 
   lines that intersect. The user moves the end effector to three points the 
   first point in the line intersection and the other two on the different lines. 
   The output the user gets is the angle between the two lines and it is displayed 
   in the EV3 screen.

3. Move the end effector of the arm to a specified point
   Given a point (x,y), the end effector can go to the point from a predetermined 
   point (x0,y0), quickly with ‘no’ or ‘little’ overshoot. The settling time, the 
   percentage overshoot and steady state error all get recorded within this function.

4. Draw a straight line
   The user is able to enter the two points into the controller and the robot moves 
   to the first point and then draws a straight line to the next point. A pen is 
   attached to the end effector to show the trajectory.

Video link: https://www.dropbox.com/s/kuqmrb8ph9q64hi/ENEL%20484%20Project%20Demo.mp4?dl=0
