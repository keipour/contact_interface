### Contact Interface
This repository acts as an interface between the physical contact-related parts (such as approach, contact, etc.) and the rest of the project. 
The current version is only a template where after receiving the command to make a contact, the UAV flies North for 2 meters, stops for 5 seconds and then flies back.

## Instructions:

After compiling the repository using `catkin build`, run the following launch file to start the node (don't forget to restart the terminal or source the workspace after the first time compilation):

```
roslaunch contact_interface contact_interface_node.launch
```

Contact commands can be sent through the `controller/contact/command` topic. More information about the message fields is available in the message definition file inside the `msg` folder. Sending only one command is enough to start the whole sequence.

The status of the contact sequence can be received from the `controller/contact/status` topic. There are four status types for the whole task and four status types for the contact. More information about the message fields and status types can be fount at the message definition file inside the `msg` folder. 

## Contact

Author: Azarakhsh Keipour

Email: keipour@cmu.edu
