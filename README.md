# RASMv2
### Version 2 of the Robotic Adaptive Screen Mount (RASM)

This code is for the RASM mechanical engineering senior design project at the University of Utah.

Engineering team:
* Far Zainal
* Nathan Dibb
* Hakimi Nazlie
* Joe Pollard
* Joshua Call

Advisor: Dr. Andrew Merryweather

The RASM is an autonomous 6-degree-of-freedom robot arm that keeps a [Tobii Dynavox Screen](https://www.tobiidynavox.com/en-US/devices/eye-gaze-devices/i-12/#Specifications) positioned in front of a user's face. The most common users will be tetraplegics and quadriplegics in a hospital environment.

{image}

---

The software system is written in C++ and is organized into eight subsystems:

Subsystem | Source Files | Dependencies
:-------- |:------------ |:------------
Configuration | src/configuration.hpp | Poco
Logging | src/logging.hpp | Poco
HTTP Server | src/http/* | Poco
Shell Server | src/shell/* | Poco
Pose Estimation | src/vision/* | openCV and dlib
Control | src/control/* | none
Peripheral Interfacing | src/peripheral/* | Poco and [c-periphery](https://github.com/vsergeev/c-periphery)
Battery Monitoring | src/battery.hpp | none

Poco version: 1.9.x

openCV version: 3.4.x

dlib version: 19.xx

**Configuration**

Provides a simple access/modify interface to all configurations which are loaded from, and synchronized with, a set of .properties configuration files. This interface is provided by a singleton that can be used by all subsystems.

**Logging**

Defines a single logging interface (as a singleton) that can be used by all subsystems. Manages the log file system and provides four log message levels: notice, warning, error, and critical.

**Battery Monitoring**

Predicts the battery system's SOC (state of charge) and SOH (state of health) using a filtering state estimation algorithm.

**HTTP Server**

The HTTP server hosts a website which allows the user to configure the RASM while it's operating. The Tobii screen or any other wifi-capable device can connect to the RASM's wifi access point and use the configuration website. Note that the access point doesn't provide any bridge to the internet. The website files are located in the test/filesysroot/rasm_2_x/web directory.

**Shell Server**

This subsystem provides a shell server and client that communicate via the trasport socket layer. A program that uses a command-line interface is also implemented which allows for engineers to easily interact with the RASM's primary program while it is running in order to read log files, run control tests, change configurations, et cetera.

**Pose Estimation**

Uses a camera mounted next to the Tobii screen to detect and estimate the pose of human faces and certain markers.

**Control**

Generates joint-space trajectories, tracks trajectories with feedforward and feedback control, and processes sensor signals. The overall RASM behavior is defined via the control routines implemented in the Controller class. These control routines utilize the pose estimation, peripheral interfacing, and battery monitoring subsystems. 

![alt text](blaaa "control system block diagram")

**Peripheral Interfacing**

Provides an abstractive interface for interacting with the peripherals connected to the single-board computer. The two peripherals in use are an 8-bit AVR microcontroller (connected via SPI) and a gen4 HMI display module (connected via UART).

---

#### Unit Testing
Unit testing is done with the [lest](https://github.com/martinmoene/lest) framework. The tests are located in the test/src directory. Note that some of them are interactive tests due to the nature of what is being tested.

#### System Setup
The linux distribution running on the RASM's single-board computer needs to have certain things configured like an access point and process initialization. The 'setup' diretory holds scripts and configuration files for these things along with general notes.

#### Build System
The build system uses CMake. The unit tests are natively compiled while the rasmexec and rasmshell programs are cross-compiled for the RASM's single-board computer.

#### MatLab Analysis
MatLab scripts have been written for various tasks such as analysis of state space models and digital filters. These scripts are located in the 'analyze' directory.
