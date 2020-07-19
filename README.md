# LH2-positioning

This project was my end of engineering school project (HEIG-VD, Switzerland).

The goal of this project was to create a device that can use one LH2 (Vive lighthouse v2.0) to determine his own position.

This repository has all the source code developed for this project :
 - The firmware for the sensor (TS4231 and efm8lb1 based)
 - The firmware for the central unit (nRF52840 based, including a BNO055 IMU)
 - The server used to process all the data (python, run on a raspberry pi in this case)
 - The web GUI used to visualize the data computed by the server and display a 3d model of the device
 - Additionally, the python code (Jupyter lab notebook) used to process the signals before developing the hardware is also available.
