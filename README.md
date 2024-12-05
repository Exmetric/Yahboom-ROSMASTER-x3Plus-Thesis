# Yahboom_Rosmaster_x3Plus_Thesis

This is a brief github repo for all codes, some guides, and specific functions for my thesis at Metropolia UAS. Metropolia has purchased a robot from Yahboom China to be used in a course for students, where the task would be to be operate this robot from home, while the robot is at the university, exchange data between the robot and the user, video stream and additional sensor data exchange.

The robot is a Yahboom Rosmaster x3Plus, running on a jetson orin nano 8gb dev kit provided by Metropolia. 

Specific functions that I have managed to implement:

Implementing a python script to control the 6 servo robotic arm as well as the wheels over a different LAN (with VPN) using a remote keyboard.
Exchanging lidar data between the robot and the user over a different WLAN
Exchanging voltage data between the robot and the user over a different WLAN
Exchanging velocity data between the robot and the user over a different WLAN

To be done: 

Video streaming, depth camera and front camera dont function properly since they came from the factory
Save lidar data as a .txt file to comprehend the data being sent
Add an imu sensor and exchange data over a different WLAN


In each folder, I will include the relevant codes and the step-by-step instructions on how to run the codes so that they work.

Before I get into the introduction of setting up the robot, I would like to thank Patrik Asikainen, Samuli Ahokas, and Niko Hutri for helping/guiding me in this project.


Please also have the [Yahboom Rosmaster X3 Plus repo](http://www.yahboom.net/study/ROSMASTER-X3-PLUS) open along when going through this guide for reference, they may update their repo in the future.
