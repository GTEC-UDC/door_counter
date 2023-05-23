# README

Set of ROS nodes to monitor the occupancy of people in a room and the crossing of a door.

## Door Node

This node is responsible for counting people passing through a door. To do so, the node needs to define the passage zone, and within it, two other sub-zones called *high* and *low*, each located at one end of the passage zone. Each time a person passes from the *high* zone to the *low* zone or vice versa, a message is sent. 




