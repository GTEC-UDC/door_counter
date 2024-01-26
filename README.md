# README

Set of ROS nodes to monitor the occupancy of a room with FMCW mmWave radars.

- **time_occupancy_node**: returns the occupancy in some deffined areas.
- **zone_occupancy_to_mqtt**: allows the publish of occupancy measurements through MQTT.
- **zone_ocuppancy_to_file**: save the occupancy measurements to a local file.

The ```launch``` folder contains some launch files to launch the algorithms with some parameters.

This repository is related with the next paper. Please cite us if this code is useful to you.

Barral, V., Dominguez-Bolano, T., Escudero, C. J., & Garcia-Naya, J. A. *An IoT System for Smart Building Combining Multiple mmWave FMCW Radars Applied to People Counting.*

