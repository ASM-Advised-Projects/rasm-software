The four cpp files defined here each run their own program.

* rasm_main.cpp -- Operates the RASM in a hospital environment.

* local_shell.cpp -- Provides a text-based front end for using a shell session hosted by RASM's shell server.

* system_modeling.cpp -- Runs a variety of system identification routines that give the RASM a series of control inputs while reading the encoders. The encoder data is stored in the file system. This data is meant to be used in MatLab to form a state-space model of the RASM.

* camera_calibration.cpp -- Runs routines for camera 'calibration' where the camera being used will have it's distortion and camera matrices identified.
