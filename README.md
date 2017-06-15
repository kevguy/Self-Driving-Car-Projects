# Self-Driving-Car-Projects
I enrolled into Udacity's Self-Driving Car Engineer Nanodegree in October 2016 and was put into the December cohort. This is an overview of all the projects I've worked on along the way.

## Table of contents
- [Semester 1: Computer Vision and Deep Learning](#semester-1-computer-vision-and-deep-learning)
  - [Project 1: Finding Lane Lines](#project-1-finding-lane-lines)
  - [Project 2: Traffic Sign Classifier Project](#project-2-traffic-sign-classifier-project)
  - [Project 3: Behavioral Cloning](#project-3-behavioral-cloning)
    - [Demo 1: Comma.ai](#demo-1-commaai)
    - [Demo 2: nVidia](#demo-2-nvidia)
  - [Project 4: Advanced Lane Finding](#project-4-advanced-lane-finding)
    - [Demo: Advanced Lane Finding](#demo-advanced-lane-finding)
  - [Project 5: Vehicle Detection and Tracking](#project-5-vehicle-detection-and-tracking)
    - [Demo: Vehicle Detection and Tracking](#demo-vehicle-detection-and-tracking)
- [Semester 2: Sensor Fusion, Localization and Control](#semester-2-sensor-fusion-localization-and-control)
  - [Project 1: Extended Kalman Filter](#project-1-extended-kalman-filter)
  - [Project 2: Unscented Kalman Filter](#project-2-unscented-kalman-filter)
  - [Project 3: Kidnapped Vehicle](#project-3-kidnapped-vehicle)
  - [Project 4: PID Controller](#project-4-pid-controller)
- [Semester 3: Path Planning, Concentrations and Systems](#semester-3-path-planning-concentrations-and-systems)

## Semester 1: Computer Vision and Deep Learning

### Project 1: Finding Lane Lines
Identifies lane lines on the road, first in an image, and later in a video stream. Since this project only works on straight lines and the repo is too big, I didn't put it on Github. I suggest you look at Project 4 instead.


### Project 2: Traffic Sign Classifier Project

Uses deep neural networks and convolutional neural networks to classify traffic signs. Specifically, the mode is trained to classify traffic signs from the [German Traffic Sign Dataset](http://benchmark.ini.rub.de/?section=gtsrb&subsection=dataset). Then the model is tested on new images of traffic signs found on the web.

- **Code**: [Github](https://github.com/kevguy/CarND-Traffic-Sign-Classifier-Project)
- **Explanation (ipynb)**: [writeup](https://github.com/kevguy/CarND-Traffic-Sign-Classifier-Project/blob/master/Traffic_Sign_Classifier.ipynb)

### Project 3: Behavioral Cloning

This project clones driving behavior (specifically mine) also by using deep neural networks and convolutional neural network. The model is trained using Keras and will output a steering angle to an autonomous vehicle running in the simulator Udacity provided.

- **Code**: [Github](https://github.com/kevguy/CarND-Behavioral-Cloning-P3)
- **Explanation**: [writeup](https://github.com/kevguy/CarND-Behavioral-Cloning-P3/blob/master/writeup_template.md)

#### Demo 1: Comma.ai

[YouTube](https://www.youtube.com/watch?v=DZmIwV8ADGw)  
I suggest you watching it in 2x speed, the car moves really slow.  
To see the part where the car is actually moving, please skip to 0:28.  
[![IMAGE ALT TEXT](http://img.youtube.com/vi/DZmIwV8ADGw/0.jpg)](https://www.youtube.com/watch?v=DZmIwV8ADGw "Self-driving Car: Behavior Cloning (Comma.AI)")

#### Demo 2: nVidia

[YouTube](https://www.youtube.com/watch?v=S9x58PpZP7M&t=6s)  
I suggest you watching it in 2x speed, the car moves really slow.  
To see the part where the car is actually moving, please skip to 1:08.  
[![IMAGE ALT TEXT](http://img.youtube.com/vi/S9x58PpZP7M/0.jpg)](https://www.youtube.com/watch?v=S9x58PpZP7M&t=6s "Self-driving Car: Behavior Cloning (nVidia)")

### Project 4: Advanced Lane Finding

To write a software pipeline to identify the lane boundaries in a video from a front-facing camera on a car. The camera calibration images, test road images, and project videos are available in the project repository.

- **Code**: [Github](https://github.com/kevguy/CarND-Advanced-Lane-Lines)
- **Explanation**: [writeup](https://github.com/kevguy/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md)

#### Demo: Advanced Lane Finding

- [YouTube](https://youtu.be/RxcDBK14jNc)
- [Download](https://github.com/kevguy/CarND-Advanced-Lane-Lines/raw/master/output_images/project_image.mp4)

[![IMAGE ALT TEXT](http://img.youtube.com/vi/RxcDBK14jNc/0.jpg)](https://www.youtube.com/watch?v=RxcDBK14jNc "Advanced Lane Finding")


### Project 5: Vehicle Detection and Tracking

To write a software pipeline to identify vehicles in a video from a front-facing camera on a car based on some test images and a video.

- **Code**: [Github](https://github.com/kevguy/CarND-Vehicle-Detection)
- **Explanation**: [writeup](https://github.com/kevguy/CarND-Vehicle-Detection/blob/master/writeup_template.md)

#### Demo: Vehicle Detection and Tracking

- [YouTube](https://youtu.be/S1V5Ia6M5Q0)
- [Download](https://github.com/kevguy/CarND-Vehicle-Detection/raw/master/output_images/processed_project_video.mp4)

[![IMAGE ALT TEXT](http://img.youtube.com/vi/S1V5Ia6M5Q0/0.jpg)](https://www.youtube.com/watch?v=S1V5Ia6M5Q0 "Self-Driving Car: Vehicle Detection and Tracking ")

## Semester 2: Sensor Fusion, Localization and Control

### Project 1: Extended Kalman Filter

Implements an extended Kalman filter in C++ that detects a bicycle that travels around the vehicle using lidar and radar measurements and track the bicycle's position and velocity.

- **Code**: [Github](https://github.com/kevguy/CarND-Extended-Kalman-Filter-Project)

### Project 2: Unscented Kalman Filter

Implements an unscented Kalman filter in C++ using the CTRV model and does the same thing as the extended Kalman filter, but should give different results.

- **Code**: [Github](https://github.com/kevguy/CarND-Unscented-Kalman-Filter-Project)

### Project 3: Kidnapped Vehicle

Implements a 2 dimensional particle filter in C++, that given a map and some initial localization information (analogous to what a GPS would provide) and at each time step given observation and control data, it can localize the vehicle's position.

- **Code**: [Github](https://github.com/kevguy/CarND-Kidnapped-Vehicle-Project)

### Project 4: PID Controller

Ongoing

- **Code**: [Github](https://github.com/kevguy/CarND-PID-Control-Project)

## Semester 3: Path Planning, Concentrations and Systems
