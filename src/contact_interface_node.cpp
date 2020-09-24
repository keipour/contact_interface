#include <contact_interface/contact_interface.h>

ContactInterfaceNode::ContactInterfaceNode(std::string node_name)
  : BaseNode(node_name){
}

bool ContactInterfaceNode::initialize(){
  ros::NodeHandle* nh = get_node_handle();
  ros::NodeHandle* pnh = get_private_node_handle();

  // init parameters
  status_pub_rate = pnh->param("contact_status_pub_rate", 20.);
  distance_step = pnh->param("distance_step", 5.0F);
  stop_distance = pnh->param("stop_distance", 0.5F);
  tilt_lock_distance = pnh->param("tilt_lock_distance", 2.0F);
  normal_xy_speed = pnh->param("normal_xy_speed", 12.0F);
  approach_xy_speed = pnh->param("approach_xy_speed", 0.5F);
  is_dummy_test = pnh->param("is_dummy_test", false);

  // init subscribers
  pose_sub = nh->subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &ContactInterfaceNode::pose_callback, this);
  mode_sub = nh->subscribe<std_msgs::Bool>("has_control", 10, &ContactInterfaceNode::mode_callback, this);
  arm_sub = nh->subscribe<std_msgs::Bool>("is_armed", 10, &ContactInterfaceNode::arm_callback, this);
  heading_sub = nh->subscribe<std_msgs::Float32>("depth_estimation/point_cloud/heading_angle", 10, &ContactInterfaceNode::heading_callback, this);
  depth_sub = nh->subscribe<geometry_msgs::Vector3Stamped>("depth_estimation/point_cloud/pose", 10, &ContactInterfaceNode::depth_callback, this);
  contact_command_sub = nh->subscribe<contact_interface::ContactCommand>("controller/contact/command", 10, &ContactInterfaceNode::contact_command_callback, this);
  status_publisher_timer = nh->createTimer(ros::Duration(1. / status_pub_rate), &ContactInterfaceNode::status_publisher_timer_callback, this);

  // init publishers
  pose_command_pub = nh->advertise<geometry_msgs::PoseStamped>("pose_command", 10);
  contact_status_pub = nh->advertise<contact_interface::ContactStatus>("controller/contact/status", 10);
  att_mode_pub = nh->advertise<core_px4_interface::AttMode>("controller/commands/set_att_mode", 10);
  max_xy_vel_pub = nh->advertise<std_msgs::Float32>("controller/commands/set_max_xy_speed", 10);
  return true;
}

bool ContactInterfaceNode::execute()
{
  // return if it's not in offboard mode or if it's not armed
  if (!is_offboard || !is_armed)
  {
    // just return if all the parameters are already set correctly
    if (contact_status == ContactStatus::None && task_status != TaskStatus::InProgress)
      return true;
    
    contact_status = ContactStatus::None;
    if (task_status == TaskStatus::InProgress)
      task_status = TaskStatus::Aborted;

    current_force = 0;
    current_moment = 0;

    publish_status();
    return true;
  }

  switch (contact_status)
  {
  case ContactStatus::Approaching :
    approach();
    break;
  case ContactStatus::InContact :
    contact();
    break;
  case ContactStatus::Departing :
    depart();
    break;
  default:
    break;
  }
  return true;
}

// ------------------------------------------------------------------------------------------- //
// -------------------------- Callbacks ------------------------------------------------------ //
// ------------------------------------------------------------------------------------------- //

void ContactInterfaceNode::mode_callback(const std_msgs::Bool::ConstPtr &msg)
{
  is_offboard = msg->data;
}

void ContactInterfaceNode::arm_callback(const std_msgs::Bool::ConstPtr &msg)
{
  is_armed = msg->data;
}

void ContactInterfaceNode::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  current_pose = *msg;
}

void ContactInterfaceNode::contact_command_callback(const contact_interface::ContactCommand::ConstPtr &msg)
{
  // return if it's not in offboard mode or if it's not armed
  if (!is_offboard || !is_armed)
  {
    contact_status = ContactStatus::None;
    task_status = TaskStatus::Aborted;
    publish_status();
    return;
  }
  
  // Set the starting point to return to it later
  starting_pose = current_pose;
  current_command = *msg;

  // Set the task and contact statuses
  task_status = TaskStatus::InProgress;
  contact_status = ContactStatus::Waiting; 
  approach_status = ApproachStatus::GettingClose;

  if (is_dummy_test)
  {
    contact_status = ContactStatus::Approaching; 
    approach_status = ApproachStatus::FinalStage;
    process_depth_reading(stop_distance + distance_step);
  }
}

void ContactInterfaceNode::depth_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
  process_depth_reading(msg->vector.z / 100.F);
}

void ContactInterfaceNode::process_depth_reading(float depth)
{
    if ((contact_status != ContactStatus::Approaching && contact_status != ContactStatus::Waiting)
   || task_status != TaskStatus::InProgress)
    return;

  // Read the current distance from the wall (in meters)
  float dist = depth;

  if (isnan(dist)) 
    return;

  if (dist <= stop_distance)
  {
      change_status(ApproachStatus::GettingClose, ContactStatus::InContact, TaskStatus::InProgress);
      return;
  }
  else if (dist <= tilt_lock_distance && approach_status == ApproachStatus::GettingClose)
  {
      change_status(ApproachStatus::LockingTilt, ContactStatus::Approaching, TaskStatus::InProgress);
      return;
  }

  change_status(approach_status, ContactStatus::Approaching, TaskStatus::InProgress);

  double dist_forward;

  switch (approach_status)
  {
  case ApproachStatus::GettingClose:
      dist_forward = std::min(dist - tilt_lock_distance, distance_step);
      fly_forward(dist_forward);
      break;
  case ApproachStatus::LockingTilt:
      // Do nothing
      break;
  case ApproachStatus::FinalStage:
      dist_forward = std::min(dist - stop_distance, distance_step);
      fly_forward(dist_forward);
      break;
  }  
}

void ContactInterfaceNode::heading_callback(const std_msgs::Float32::ConstPtr &msg)
{
  // Do nothing for now
}

void ContactInterfaceNode::status_publisher_timer_callback(const ros::TimerEvent &te)
{
  publish_status();
}

// ------------------------------------------------------------------------------------------- //
// ----------------------------- Misc Functions ---------------------------------------------- //
// ------------------------------------------------------------------------------------------- //

void ContactInterfaceNode::fly_forward(const double dist_forward)
{
  current_command.ContactPoint = current_pose.pose;

  // Get the current yaw
  tf::Quaternion q(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
  double roll, pitch, yaw_enu;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw_enu);
  double yaw = M_PI / 2 - yaw_enu;

  // Set the destination as a bit forward in the direction of the current yaw
  current_command.ContactPoint.position.x += dist_forward * std::sin(yaw);
  current_command.ContactPoint.position.y += dist_forward * std::cos(yaw);

  // Don't change the altitude during the sequence
  current_command.ContactPoint.position.z = starting_pose.pose.position.z;
}

void ContactInterfaceNode::change_status(ApproachStatus as, ContactStatus cs, TaskStatus ts)
{
  if (cs == ContactStatus::InContact && contact_status != ContactStatus::InContact)
  {
    delay_started = ros::Time::now();
  }
  else if (as == ApproachStatus::LockingTilt && approach_status != ApproachStatus::LockingTilt)
  {
    delay_started = ros::Time::now();
    publish_att_mode(core_px4_interface::AttMode::ESTIMATE);
  }
  else if (as == ApproachStatus::FinalStage && approach_status != ApproachStatus::FinalStage)
  {
    publish_att_mode(core_px4_interface::AttMode::LOCK_TILT);
    publish_max_xy_vel(approach_xy_speed);
  }
  else if (as == ApproachStatus::LockingTilt && approach_status != ApproachStatus::LockingTilt)
  {
      delay_started = ros::Time::now();
  }
  else if (ts == TaskStatus::Aborted && ts != task_status)
  {
    publish_att_mode(core_px4_interface::AttMode::ZERO_TILT);
    publish_max_xy_vel(normal_xy_speed);
  }
  else if (cs == ContactStatus::Departing && contact_status != ContactStatus::Departing)
  {
      publish_att_mode(core_px4_interface::AttMode::ZERO_TILT);
      publish_max_xy_vel(normal_xy_speed);
  }

  // if (as != approach_status)
  //   ROS_WARN("AS: %d", (int)as);

  contact_status = cs;
  task_status = ts;
  approach_status = as;
}

// ------------------------------------------------------------------------------------------- //
// ------------------------------ Publisher Functions ---------------------------------------- //
// ------------------------------------------------------------------------------------------- //

void ContactInterfaceNode::publish_status()
{
  // initialize the message
  contact_interface::ContactStatus msg;
  msg.Header.stamp = ros::Time::now();
  msg.Header.frame_id = current_command.Header.frame_id;
  msg.Header.seq = contact_status_seq++;
  msg.AppliedForce = current_force;
  msg.AppliedMoment = current_moment;
  msg.EndEffectorPose = current_pose.pose;
  msg.ContactStatus = contact_status;
  msg.TaskStatus = task_status;

  // publish the message
  contact_status_pub.publish(msg);
}

void ContactInterfaceNode::publish_pose_command(const geometry_msgs::Pose &pose)
{
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = current_command.Header.frame_id;
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();
  msg.pose = pose;

  pose_command_pub.publish(msg);
}

void ContactInterfaceNode::publish_att_mode(const int mode)
{
  core_px4_interface::AttMode msg;
  msg.Header.frame_id = "";
  msg.Header.seq = 0;
  msg.Header.stamp = ros::Time::now();
  msg.Mode = mode;
  ROS_INFO("Att: %d", mode);
  att_mode_pub.publish(msg);
}

void ContactInterfaceNode::publish_max_xy_vel(const float speed)
{
  std_msgs::Float32 msg;
  msg.data = speed;
  ROS_INFO("Speed: %0.2f", speed);
  max_xy_vel_pub.publish(msg);
}

// ------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------- //
// ------------------------------------------------------------------------------------------- //

void ContactInterfaceNode::approach()
{
  switch (approach_status)
  {
  case ApproachStatus::GettingClose :
    approach_getting_close();
    break;
  case ApproachStatus::LockingTilt:
    approach_locking_tilt();
    break;
  case ApproachStatus::FinalStage:
    approach_final_stage();
    break;
  }
}

void ContactInterfaceNode::approach_getting_close()
{
  current_force = 0;
  current_moment = 0;
  
  double dx = current_pose.pose.position.x - current_command.ContactPoint.position.x;
  double dy = current_pose.pose.position.y - current_command.ContactPoint.position.y;
  double dz = current_pose.pose.position.z - current_command.ContactPoint.position.z;
  double dist = dx*dx + dy*dy + dz*dz;

  const double dist_threshold = 0.25F; // 25 cm
  if (dist > dist_threshold * dist_threshold)
  {
    publish_pose_command(current_command.ContactPoint);
  }
  else
  {
    if (is_dummy_test)
      change_status(ApproachStatus::GettingClose, ContactStatus::InContact, TaskStatus::InProgress);
    else
      change_status(ApproachStatus::LockingTilt, ContactStatus::Approaching, TaskStatus::InProgress);
  }
}

void ContactInterfaceNode::approach_locking_tilt()
{
  const float estimation_duration = 10; // in seconds

  if (ros::Time::now() - delay_started > ros::Duration(estimation_duration))
  {
    change_status(ApproachStatus::FinalStage, ContactStatus::Approaching, TaskStatus::InProgress);
  }
}

void ContactInterfaceNode::approach_final_stage()
{
    current_force = 0;
    current_moment = 0;

    double dx = current_pose.pose.position.x - current_command.ContactPoint.position.x;
    double dy = current_pose.pose.position.y - current_command.ContactPoint.position.y;
    double dz = current_pose.pose.position.z - current_command.ContactPoint.position.z;
    double dist = dx*dx + dy*dy + dz*dz;

    const double dist_threshold = 0.25F; // 25 cm
    if (dist > dist_threshold * dist_threshold)
    {
        publish_pose_command(current_command.ContactPoint);
    }
    else
    {
      change_status(ApproachStatus::GettingClose, ContactStatus::InContact, TaskStatus::InProgress);
    }
}

void ContactInterfaceNode::contact()
{
  const float contact_duration = 5; // in seconds
  current_force = current_command.Force; // in Newtons
  current_moment = current_command.Moment; // in N.m

  if (ros::Time::now() - delay_started > ros::Duration(contact_duration))
  {
    change_status(ApproachStatus::GettingClose, ContactStatus::Departing, TaskStatus::InProgress);
  }
}

void ContactInterfaceNode::depart()
{
  current_force = 0;
  current_moment = 0;

  double dx = current_pose.pose.position.x - starting_pose.pose.position.x;
  double dy = current_pose.pose.position.y - starting_pose.pose.position.y;
  double dz = current_pose.pose.position.z - starting_pose.pose.position.z;
  double dist = dx * dx + dy * dy + dz * dz;

  const double dist_threshold = 0.25F; // 25 cm
  if (dist > dist_threshold * dist_threshold)
  {
    publish_pose_command(starting_pose.pose);
  }
  else
  {
    change_status(ApproachStatus::GettingClose, ContactStatus::None, TaskStatus::Completed);
  }
}




ContactInterfaceNode::~ContactInterfaceNode() = default;

BaseNode* BaseNode::get()
{
  ContactInterfaceNode* control_interface_node = new ContactInterfaceNode("ControlInterfaceNode");
  return control_interface_node;
}
