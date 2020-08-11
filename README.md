# Cooperative Relative Localization

In this project, we have created python codes to read data from multiple turtlebots on a single computer (using the ROS Master Slave configuration), read motion capture data and also read decawave radio data.
In the launch folder, two different launch files are provided. One is used to read data from the turtlebots. This file is kept separate as it can be used standalone to collect data for any offline experiement as well. The CooperativeRelativeEKF.launch is used to launch the sensor nodes, the EKF and the error calculation node. The file is setup in such a way that it can run on live data or it can run using previous bagged data. 
