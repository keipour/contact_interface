Header header
float32 applied_force       # The amount of force currently being applied on the contact point
float32 applied_moment      # The amount of torque currently being on the contact point
geometry_msgs/Pose end_effector_pose    # The coordinates of the end effector
int8 ContactStatus          # The status of the contact controller
int8 TaskStatus             # The status of the current or last commanded task

int8 CONTACT_NONE=0         # The drone is not in the contact control mode
int8 CONTACT_APPROACH=1     # The drone is approaching the contact point
int8 CONTACT_CONTACT=2      # The end effector is in contact
int8 CONTACT_DEPART=3       # The drone is backing off after the contact

int8 TASK_NONE=0            # No task is received yet
int8 TASK_IN_PROGRESS=1     # The current task is in progress
int8 TASK_COMPLETED=2       # The last task has successfully completed 
int8 TASK_ABORTED=3         # The last task was not successful
