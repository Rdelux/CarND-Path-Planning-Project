# **Term 3 Project 1: Path Planning Project**
Self-Driving Car Engineer Nanodegree Program

The goals / steps of this project are the following:

* Implement a PID controller in C++ to maneuver the vehicle around the lake race track from the Behavioral Cloning Project
* The PID procedure follows what was taught in the lessons
* Calculate the steering angle based on the cross track error (CTR) and the chosen vehicle velocity value
* The car should not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe


[//]: # (Image References)

[image1]: ./images/High_Speed_Cornering.png "HSC"
[image2]: ./images/Recovering.png "Recover"



The Rubric Points are listed in this following [link](https://review.udacity.com/#!/rubrics/824/view)   

---

### PID Controller Implementation

Using the simulator provided in the Behavioral Cloning Project, a vehicle is to be maneuvered by a PID controller algorithm, implemented in C++ code.  The PID class is implemented in the PID.cpp and PID.h files.  The simulator provides the cross track error (CTE) to the code and the PID algorithm will compute the steering angle in order to minimize the deviation or error of the vehicle relative to the track.  
