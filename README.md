# Contact Interface
This repository acts as an interface between the physical contact-related parts (such as approach, contact, etc.) and the rest of the project. 
The current version is only a template where after receiving the command to make a contact, the UAV flies in the direction of its yaw for a defined distance, stops for 5 seconds and then flies back. The recent updated also allows flying forward based on the distance to the wall received from the tracking node.

## Instructions:

After compiling the repository using `catkin build`, run the following launch file to start the node (don't forget to restart the terminal or source the workspace after the first time compilation):

```
roslaunch contact_interface contact_interface_node.launch
```

Contact commands can be sent through the `controller/contact/command` topic. More information about the message fields is available in the message definition file inside the `msg` folder. 

*Currently, the command fields are not important and just sending any command on this topic would initiate the sequence. Sending only one command is enough to start the whole sequence.*

The distance going forward is defined as the `distance_step` parameter instead of the fixed 2 meters (see launch file).

It reads the distance to the wall from the `/point_cloud/pose` topic. The z element is read as the distance to wall.
Every time the distance to wall is received, it will go forward distance_step meters until some stopping distance defined by `stop_distance` parameter (see launch file). 

*An example:* To just go forward fixed 2 meters, set the `distance_step` to 2 meters and don't send the depth data. 
If also sending with the depth data, I would suggest setting distance_step to something small like 0.1 meters.

**Important**: Note that the current version is not fully tested with the distance to wall data.

The status of the contact sequence can be received from the `controller/contact/status` topic. There are four status types for the whole task and four status types for the contact. 
More information about the message fields and status types can be found at the message definition file inside the `msg` folder. 

## Contact

Author: Azarakhsh Keipour

Email: keipour@cmu.edu
