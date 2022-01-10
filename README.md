# ROS-orchestration-and-super-clustering
This repository contains the source code for the ROS node that collects data on the topics for the computation of the Bandwidth, then builds the communication graph to cluster the ROS nodes with a modified Hierarchical Clustering Algorithm.

## Executing the code
With the ROS enviroment to containerize running, execute the following command:

```
rosrun rbkairos_demo_pkg_sim collect_data.py
```

The output will represent the containerized nodes. The node also saves the data to file for later analysis.

This repository also contains the data collected for the RB-Kairos robot.
