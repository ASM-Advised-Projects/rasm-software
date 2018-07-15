# rasm-v2
#### Version 2 of the Robotic Adaptive Screen Mount (RASM).
This is a mechanical engineering senior design project at the University of Utah.
The project is being advised by Dr. Andrew Merryweather.

The RASM is an autonomous robot arm that keeps a [Tobii Dynavox Screen](https://www.tobiidynavox.com/en-US/devices/eye-gaze-devices/i-12/#Specifications) in front of a user's face.
The most common users will be tetraplegics and quadriplegics in a hospital environment.

Its software system is organized into eight subsystems:
Subsystem | Source Files
--------- | ------------
Configuration | src/configuration.hpp
Logging | src/logging.hpp
HTTP Server | src/http/*
Shell Server | src/shell/*
Pose Estimation | src/vision/*
Control | src/control/*
Peripheral Interfacing | src/peripheral
Battery Monitoring | src/battery.hpp

The configuration, logging, and battery monitoring subsystems are simple and
self-explanatory enough to not need a description.

##### HTTP Server
The HTTP server hosts a website which allows the user to configure the RASM
while its operating. The Tobii screen itself is used to connect to the RASM's
wifi access point and browse the configuration website.

##### Shell Server
The shell server provides a command-line interface that allows for software
engineers to interact with the RASM while it is running in order to gather
log files, run tests, change configurations, et cetera.

##### Pose Estimation
This subsystem uses a camera mounted next to the Tobii screen to detect and
estimate the pose of human faces and certain markers.

##### Control


##### Peripheral Interfacing

