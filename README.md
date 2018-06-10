# **Term 3 Project 1: Path Planning Project**
Self-Driving Car Engineer Nanodegree Program

The goals of this project are the following:

* Implement a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic
* Using locaalization, sensor fusion and map data, the path planner will be able to keep the car inside its lane, avoid hitting other cars, and pass slower moving traffic
* The car is able to drive at least 4.32 miles without incident
* The car drives according to the speed limit
* The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3
* Car does not have collisions
* The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road

[//]: # (Image References)

[image1]: ./images/LaneChange_Close_Rear.png "P1"
[image2]: ./images/NotChangingLane_OtherLanesOccupied.png "P2"
[image3]: ./images/Prioritize_Left_Lane_Change.png "P3"
[image4]: ./images/RelativeVelocityConsidered_CloseDistance.png "P4"
[image5]: ./images/RightLaneChange_2ndPriority_RelativeSpeed_Considered.png "P5"
[image6]: ./images/RightLaneChange_Left_Lane_Occupied.png "P6"
[image7]: ./images/RightLaneChange_Lf_RR_occupied.png "P7"
[image8]: ./images/Simple_LLC.png "P8"
[image9]: ./images/Simple_RLC.png "P9"


The Rubric Points are listed in this following [link](https://review.udacity.com/#!/rubrics/1020/view)   

---

### Path Planner Implementation

The code model for generating paths that satisfy the listed goals is described in detail in this report.  This project uses the Term 3 Simulator to provide visualization of the vehicle opeation, as well as feeding data to the C++ path planning code for path generation.  The focus of this project is to generate a path that the vehicle can follow while satisfying the given goals.  The control aspect of autonomous vehicle operation is assumed to be handled by the simulator and it will not be addressed in this project.   It is also assumed that other vehicles in the simulator will obey the traffic laws and will not drive in a dangerous manner to cause accident.

### General Motion

The vehicle in the simulator will move from one path point to the next one every 20 ms.  Therefore, in order to move the vehicle on the road, a series of x,y pair path points need to be created, and these path points will form the trajectory of the vehicle.  Velocity of the vehicle will be controlled by the spacing between these path points.  The goal is to achieve 50 MPH without passing this limit in all situation while maximizing the speed.  Localization information is provided by the simulator and they are loaded into main.cpp from line 231 to 236.  Both 2D cartesian coordinates and Frenet coordinates are given to describe the state of the vehicle. While the cartesian coordinates are used to calculate path points, Frenet coordinates are used to compute lane changing logic since it is invariant to the curvature of the highway. Waypoints for the map are provided in a CSV file in both coordinate systems.  These waypoints were used to tranform path points between the two coordinate systems (See main.cpp, line 519 to 521).  The path points are passed to the simulator as 2 vector quantities, next_x_vals and next_y_vals, they are passed to two JSON parameters at line 608 and 609.  Since it is required that the acceleration and jerk does not exceed the prescribed limit, the starting velocity of the vehicle is set to 0 (main.cpp, line 210), and the linear acceleration of the vehicle is set to 0.224 m/s^2 (main.cpp, line 470 to 473), which was found to be acceptable for the requirements.  Angular acceleration and jerk is controlled by the speed and the how the trajectory is created.  These will be discussed in the following sections.

### Complex Path on Highway

In order to keep the vehicle to follow the road, which is not a straight line, Frenet coordinate can be used since the s-coordinate follows the curvature of the road and the d-coordinate how far the vehicle is from the center of the road.  These information are inherently useful for lane following calculation.  To keep the vehicle in the targeted lane, a constant d-value can be used.  The s-value of the path points will need to be calculated based on the desired velocity since the vehicle will move from one path point to the next in 20 ms in the simulator.  

While staying in one lane is important but if there's slower traffic at your lane, lane changing is an option and should be executed if it is safe to do so.  In order to create a path for changing lanes, a C++ spline tool, [spline.h](http://kluge.in-chemnitz.de/opensource/spline/), was used.  Keypoints on the spline were defined, and the spline tool will interpolate the points and create a trajectory for the car to follow.  Three keypoints are 30m apart from each other and they are created in line 519 to 520, which depends on the car's current position and the desired lane that it wants to arrive after a lane change.  After the general path is defined, additional path points are generated along the spline and this is implemented in line 567 to 587.  These points are generated so that the desired reference velocity is achieved and maintained.  The reference velocity was set to 49.5 MPH, which is close to the 50 MPH limit but it also has a safety margin to ensure vehicle does not go over the speed limit.  This is set in the condition for acceleration/deceleration in lines 466 to 473.  

It is also important minimize jerk experienced by the vehicle therefore the transition between the sequential path points need to be smooth.  In order to do that, the previous path points of the vehicle need to be taken into account when creating future path points.  The simulator provides previous path points and this information is loaded to main.cpp in line 239 and 240.  In the start of the path, not too many previous points are available, therefore path points that are tangential to the vehicle is used (see line 490 and 491). As more previous points are generated, the previous path's end points can be used as starting reference for future points (see line 501 to 516).





### Sensor Fusion and Lane Changing

Localization information about other cars on the road is provided by the data JSON object from the simulator, which is loaded to main.cpp at line 247.  This information is used to determine where the other cars are and their velocity, so that our vehicle can react to traffic to achieve the objectives, such as avoid collision and lane changing. 

Avoid collision - trigger lane change

Wrapping Back
