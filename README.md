# RASMv2
### Version 2 of the Robotic Adaptive Screen Mount (RASM)

This is a mechanical engineering senior design project at the University of Utah. The project is being advised by Dr. Andrew Merryweather.

The RASM is an autonomous robot arm that keeps a [Tobii Dynavox Screen](https://www.tobiidynavox.com/en-US/devices/eye-gaze-devices/i-12/#Specifications) in front of a user's face. The most common users will be tetraplegics and quadriplegics in a hospital environment.

---

The software system is organized into eight subsystems:

Subsystem | Source Files | Dependencies
:-------- |:------------ |:------------
Configuration | src/configuration.hpp | Poco
Logging | src/logging.hpp | Poco
HTTP Server | src/http/* | Poco
Shell Server | src/shell/* | Poco
Pose Estimation | src/vision/* | openCV and dlib
Control | src/control/* | none
Peripheral Interfacing | src/peripheral/* | Poco and c-periphery
Battery Monitoring | src/battery.hpp | none

**Configuration**

Provides a simple access/modify interface to all configurations which are loaded from, and synchronized with, a set of configuration files. This interface is provided by a singleton that can be used by all subsystems.

**Logging**

Defines a single logging interface (as a singleton) that can be used by all subsystems. Manages the log file system and provides four log message levels: notice, warning, error, and critical.

**Battery Monitoring**

Predicts the battery system's SOC (state of charge) and SOH (state of health) using a filtering state estimation algorithm.

**HTTP Server**

The HTTP server hosts a website which allows the user to configure the RASM while its operating. The Tobii screen itself is used to connect to the RASM's wifi access point and browse the configuration website.

**Shell Server**

This subsystem provides a shell server and client that communicate via the trasport socket layer. A program that uses a command-line interface is also defined which allows for software engineers to easily interact with the RASM while it is running in order to read log files, run tests, change configurations, et cetera.

**Pose Estimation**

Uses a camera mounted next to the Tobii screen to detect and estimate the pose of human faces and certain markers.

**Control**

Generates joint-space trajectories, tracks trajectories with feedforward and feedback control, and filters, integrates, and differentiates sensor signals. The overall RASM behavior is defined via the control routines implemented in the Controller class. These control routines utilize the pose estimation, peripheral interfacing, and battery monitoring subsystems.

**Peripheral Interfacing**

Provides an abstractive interface for interacting with the peripherals connected to the single-board computer. The two peripherals in use are an 8-bit AVR microcontroller (SPI connection) and a gen4 HMI display module (UART connection).

---

#### Unit Testing


#### Build System
