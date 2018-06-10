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

While staying in one lane is important but if there's slower traffic at your lane, lane changing is an option and should be executed if it is safe to do so.  In order to create a path for changing lanes, a C++ spline tool, [spline.h](http://kluge.in-chemnitz.de/opensource/spline/), was used.  Keypoints on the spline were defined, and the spline tool will interpolate the points and create a trajectory for the car to follow.  Three keypoints are 30m apart from each other and they are created in line 519 to 520, which depends on the car's current position and the desired lane that it wants to arrive after a lane change.  After the general path is defined, additional path points are generated along the spline using a horizon value of 30m and a reference velocity of 49.5 MPH. This is implemented in line 567 to 587.  These points are generated so that the desired reference velocity is achieved and maintained.  The reference velocity was set to 49.5 MPH, which is close to the 50 MPH limit but it also has a safety margin to ensure vehicle does not go over the speed limit.  This is set in the condition for acceleration/deceleration in lines 466 to 473.  Local vehicle coordinate transformation is performed in line 531 to 540 to ensure the trajectory created is always relative to the vehicle orientation and to make path point computation easier to manage.  After the path points are generated, a basis transformation will take the coordinates back to the global coordinate system, this is implemented in line 579 to 583 of the code.

It is also important minimize jerk experienced by the vehicle therefore the transition between the sequential path points need to be smooth.  In order to do that, the previous path points of the vehicle need to be taken into account when creating future path points.  The simulator provides previous path points and this information is loaded to main.cpp in line 239 and 240.  In the start of the path, not too many previous points are available, therefore path points that are tangent to the vehicle is used (see line 490 and 491). As more previous points are generated, the previous path's end points can be used as starting reference for future points (see line 506 to 508). Using 2 previous points and the 3 future points, which are 30m, 60m and 90m in front of the car, a smooth path can be generated to ensure jerk is minimize due to path's curvature.  Additional future path points are generated as the car move down the road instead of generating the entire path every time when the simulator update the path. 50 points are generated in the path and the code will create new path points according to how many of the points are consumed.

### Sensor Fusion and Lane Changing

Localization information about other cars on the road is provided by the sensor fusion data JSON object from the simulator, which is loaded to main.cpp at line 247.  This information is used to determine where the other cars are and their velocity, so that our vehicle can react to traffic to achieve the objectives, such as avoid collision and lane changing.  

In order to avoid colliding into the car in front of our vehicle, a for loop is used to go through all the sensor fusion data that indicate the state of all the cars on the road and determine if the other cars are in the lane that we currently occupied and within a collision safety distance of 20m in front of our vehicle.  Once another vehicle is detected to be less than 20m ahead of our vehicle(line 280) in the same lane(line 269), a flag will be raised, the vehicle will decelerate to avoid collision (line 466 to 469), then the vehicle will enter into a state to decide if lane change is possible.

Triggering the lane change state, the algorithm will first determine which lane that our vehicle is currently located.  If the vehicle is located in the left-most lane(lane 0)(line 288) or the right-most lane(lane 2)(line 335), the only option for lane change is to transistioning to the middle lane, lane 1. See Figure 1 and 2 for illustrations of lane changes to the middle lane.  On the other hand, if the vehicle is in the middle lane, it can go into the left lane or the right lane.  Left-lane change is prioritized over right-lane change due to regulation in many countries(see Figure 3).  

![alt text][image9]

*Figure 1: Simple Lane Change from Left-most Lane to Middle Lane*
<br><br>

![alt text][image8]

*Figure 2: Simple Lane Change from Right-most Lane to Middle Lane*
<br><br>

![alt text][image3]

*Figure 3: Lane Change - Prioritizing Right-side Passing of Other Vehicle*
<br><br>

In order to safely execute a lane change, a number of factors need to be considered.  First of all, the there's another car occupying the target lane, it will not be safe to change lane.  In addition, if there is a vehicle in the target lane, which is situated not too much further ahead of the vehicle in front of us, a lane change will not be effective since one will need to perform another lane change immediately again once our vehicle arrived in our targeted lane.  This will cause an unsafe driving behaviour and it should be avoided.  Therefore it is important to consider if there's a vehicle located in the target lane that is a little further ahead (the safe distanced defined in the code is set to 25m, which is 5m more than the frontal car safe distance).  The car that is closest to our car and it is in the target lane in front of us will be identified and its distance from our vehicle will be used to decide if a lane change should be commenced. This is achieved by another for loop will go through all the other vehicles on the road.  See code from line 294 to 317 for left-most lane lane-changes operation.  In addition, if there's another vehicle approaching from behind on the targeted lane, a lane change will also not be safe.  Not only the position of the other vehicle is important but its velocity is also important for predicting its future state.  Its velocity will reduce the effective safe distance for car approach from behind.  These consideration are implemented in line 327 to 331 for lane 0, line 374 to 379 for lane 2, and 449 to 459 for lane 1.  In Figure 4, even though left lane change is prioritized due to the right-side passing best-practices, since there's a car in the left lane, the targeted lane change is moved to the right lane. In Figure 5, you can see that a lane change is not commenced since there's a car occupying the target lane.

![alt text][image7]

*Figure 4: Right Lane Change Instead of Left Lane Change due to Occupied Target Lane*
<br><br>

![alt text][image2]

*Figure 5: Lane Change not Commenced due to Busy Traffic at the Target Lane*
<br><br>


Wrapping Back
