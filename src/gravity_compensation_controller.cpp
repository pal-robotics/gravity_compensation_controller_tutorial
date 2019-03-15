#include <XmlRpc.h>
#include <gravity_compensation_controller_tutorial/gravity_compensation_controller.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

namespace force_control
{
bool GravityCompensationTutorial::initRequest(hardware_interface::RobotHW* robot_hw,
                                              ros::NodeHandle& root_nh,
                                              ros::NodeHandle& controller_nh,
                                              ClaimedResources& claimed_resources)
{
  // Check if construction finished cleanly
  if (state_ != CONSTRUCTED)
  {
    ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
    return false;
  }

  // Get a pointer to the joint effort control interface
  hardware_interface::EffortJointInterface* effort_iface =
      robot_hw->get<hardware_interface::EffortJointInterface>();

  if (!effort_iface)
  {
    ROS_ERROR("This controller requires a hardware interface of type EffortJointInterface."
              " Make sure this is registered in the hardware_interface::RobotHW class.");
    return false;
  }

  // Get a pointer to the joint position control interface
  hardware_interface::JointStateInterface* joint_state_iface =
      robot_hw->get<hardware_interface::JointStateInterface>();
  if (!joint_state_iface)
  {
    ROS_ERROR("This controller requires a hardware interface of type JointStateInterface."
              " Make sure this is registered in the hardware_interface::RobotHW class.");
    return false;
  }

  // Clear resources associated at both interfaces
  effort_iface->clearClaims();
  joint_state_iface->clearClaims();


  if (!init(effort_iface, joint_state_iface, root_nh, controller_nh))
  {
    ROS_ERROR("Failed to initialize the controller");
    return false;
  }

  // Saves the resources claimed by this controller
  claimed_resources.push_back(hardware_interface::InterfaceResources(
      hardware_interface::internal::demangledTypeName<hardware_interface::EffortJointInterface>(),
      effort_iface->getClaims()));
  effort_iface->clearClaims();

  // Changes state to INITIALIZED
  state_ = INITIALIZED;
  return true;
}

bool GravityCompensationTutorial::init(hardware_interface::EffortJointInterface* effort_iface,
                                       hardware_interface::JointStateInterface* joint_state_iface,
                                       ros::NodeHandle& /*root_nh*/, ros::NodeHandle& control_nh)
{
  ROS_INFO_STREAM("Loading public gravity compensation controller");

  // Check int the param server if subchains specified
  std::vector<std::string> tip_links;
  control_nh.getParam("robot_model_chains", tip_links);

  std::vector<double> joint_position_min;
  std::vector<double> joint_position_max;
  std::vector<double> joint_vel_min;
  std::vector<double> joint_vel_max;
  std::vector<double> joint_damping;
  std::vector<double> joint_friction;
  std::vector<double> joint_max_effort;

  if (tip_links.size() > 0)
  {
    // Parse the robot if subchains specified
    RigidBodyDynamics::Addons::URDFReadFromParamServer(
        &rbdl_model_, RigidBodyDynamics::FloatingBaseType::FixedBase, tip_links,
        joint_names_, joint_position_min, joint_position_max, joint_vel_min,
        joint_vel_max, joint_damping, joint_friction, joint_max_effort);
  }
  else
  {
    // Parse the full robot if there is no subchain specified
    RigidBodyDynamics::Addons::URDFReadFromParamServer(
        &rbdl_model_, RigidBodyDynamics::FloatingBaseType::FixedBase, joint_names_,
        joint_position_min, joint_position_max, joint_vel_min, joint_vel_max,
        joint_damping, joint_friction, joint_max_effort);
  }

  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    // Checks joint type from param server
    std::string control_type;
    if (!control_nh.getParam("joints/" + joint_names_[i] + "/type", control_type))
    {
      ROS_ERROR_STREAM("Could not find joint " << joint_names_[i] << " interface type");
      return false;
    }

    if (control_type == "actuated" ||
        control_type == "no_control")  // If joint is actuated or constantly commanded to zero
    {
      // Read the actuator parameters from param server
      ActuatorParameters actuator_parameters;
      if (!control_nh.getParam("joints/" + joint_names_[i] + "/motor_torque_constant",
                               actuator_parameters.motor_torque_constant))
      {
        ROS_ERROR_STREAM("Could not find motor torque constant for joint " << joint_names_[i]);
        return false;
      }
      if (!control_nh.getParam("joints/" + joint_names_[i] + "/reduction_ratio",
                               actuator_parameters.reduction_ratio))
      {
        ROS_ERROR_STREAM("Could not find reduction ratio for joint " << joint_names_[i]);
        return false;
      }

      // Reads the optional gravity compensation parameters
      GravityCompensationParameters friction_parameters;
      if (!control_nh.getParam("viscous_friction", friction_parameters.viscous_friction))
        ROS_WARN_STREAM("No viscous friction defined for joint "
                        << joint_names_[i] << ". Setting it to 0.0");
      if (!control_nh.getParam("velocity_tolerance", friction_parameters.velocity_tolerance))
        ROS_WARN_STREAM("No velocity tolerance defined for joint "
                        << joint_names_[i] << ". Setting it to 0.0");
      if (!control_nh.getParam("static_friction", friction_parameters.static_friction))
        ROS_WARN_STREAM("No static friction defined for joint " << joint_names_[i]
                                                                << ". Setting it to 0.0");

      try
      {
        // Try to get an effort interface handle to command the joint in effort
        hardware_interface::JointHandle joint_handle =
            effort_iface->getHandle(joint_names_[i]);
        // Creates an actuated joint and insert in the map of actuated joints
        ActuatedJoint actuated_joint;
        actuated_joint.joint_handle = joint_handle;
        actuated_joint.actuator_parameters = actuator_parameters;
        actuated_joint.friction_parameters = friction_parameters;
        actuated_joints_.insert(std::make_pair(joint_names_[i], actuated_joint));
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Could not find joint " << joint_names_[i] << " with Effort interface");
        return false;
      }
      // Insert the joint in the map of joint types according to his type
      if (control_type == "actuated")
        joint_types_.insert(std::make_pair(joint_names_[i], JointType::ACTUATED));
      else if (control_type == "no_control")
        joint_types_.insert(std::make_pair(joint_names_[i], JointType::ACTUATED_NO_CONTROL));
    }
    else  // If static joint
    {
      try
      {
        // Try to get a joint state handle which only allows us to read the current states
        // of the joint
        hardware_interface::JointStateHandle joint_state_handle =
            joint_state_iface->getHandle(joint_names_[i]);
        // Insert this handle in the map of static joints
        static_joints_.insert(std::make_pair(joint_names_[i], joint_state_handle));
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Could not find joint " << joint_names_[i] << " with Position interface");
        return false;
      }
      // Insert the joint in the map of joint types
      joint_types_.insert(std::make_pair(joint_names_[i], JointType::STATIC));
    }
  }

  assert(joint_types_.size() == joint_names_.size());
  assert(joint_types_.size() == actuated_joints_.size() + static_joints_.size());

  // Iinitializa q_act_, q_zero_, tau_cmd_
  q_act_.resize(joint_names_.size());
  q_zero_.resize(joint_names_.size());
  tau_cmd_.resize(joint_names_.size());

  q_act_.setZero();
  q_zero_.setZero();
  tau_cmd_.setZero();

  // Allows to modify the gravity compensation parameters from the rqt reconfigure
  for (auto it = actuated_joints_.begin(); it != actuated_joints_.end(); it++)
  {
    ddr_.reset(new ddynamic_reconfigure::DDynamicReconfigure(control_nh));
    ddr_->RegisterVariable(&it->second.friction_parameters.viscous_friction,
                           it->first + "/viscous_friction_gain");
    ddr_->RegisterVariable(&it->second.friction_parameters.static_friction,
                           it->first + "/static_friction_gain");
    ddr_->RegisterVariable(&it->second.friction_parameters.velocity_tolerance,
                           it->first + "/velocity_tolerance_gain");
    ddr_->PublishServicesTopics();
  }

  return true;
}

void GravityCompensationTutorial::update(const ros::Time& time, const ros::Duration& period)
{
  // Read the current position of all the joints
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    if (joint_types_[joint_names_[i]] == JointType::ACTUATED ||
        joint_types_[joint_names_[i]] == JointType::ACTUATED_NO_CONTROL)
      q_act_[i] = actuated_joints_[joint_names_[i]].joint_handle.getPosition();
    else
      q_act_[i] = static_joints_[joint_names_[i]].getPosition();
  }

  // Calculate the minimum torque to maintain the robot at the current position
  tau_cmd_.setZero();
  RigidBodyDynamics::InverseDynamics(rbdl_model_, q_act_, q_zero_, q_zero_, tau_cmd_);

  // For all the joints...
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    // ...check those one that are actuated
    if (joint_types_[joint_names_[i]] == JointType::ACTUATED)
    {
      // Translate the calculated torque to desired effort by integrating the frictions of
      // the motor + the ger ration + motor constants
      ActuatedJoint& actuated_joint = actuated_joints_[joint_names_[i]];
      double desired_torque = tau_cmd_[i];
      double actual_velocity = actuated_joint.joint_handle.getVelocity();

      desired_torque += actuated_joint.friction_parameters.viscous_friction * actual_velocity;
      if (actual_velocity > actuated_joint.friction_parameters.velocity_tolerance)
        desired_torque += actuated_joint.friction_parameters.static_friction;
      else
        desired_torque -= actuated_joint.friction_parameters.static_friction;

      double desired_effort =
          desired_torque / (actuated_joint.actuator_parameters.motor_torque_constant *
                            actuated_joint.actuator_parameters.reduction_ratio);

      if (std::isnan(desired_effort))  // If desired effort is not valid
      {
        ROS_ERROR_STREAM("Desired effort is not valid for joint "
                         << joint_names_[i] << " = " << desired_effort);
        return;
      }
      else
      {
        // Command an effort to the joint via ros_cotrol interface
        actuated_joint.joint_handle.setCommand(desired_effort);
      }
    }
    else if (joint_types_[joint_names_[i]] == JointType::ACTUATED_NO_CONTROL)
    {
      // ...and those one that are constantly commanded to zero
      // Send zero effort command via ros control interce
      actuated_joints_[joint_names_[i]].joint_handle.setCommand(0);
    }
  }
}

void GravityCompensationTutorial::starting(const ros::Time& time)
{
}

void GravityCompensationTutorial::stopping(const ros::Time& time)
{
  // Set desired effort to zero for the actuated joints
  for (auto it = actuated_joints_.begin(); it != actuated_joints_.end(); it++)
  {
    it->second.joint_handle.setCommand(0);
  }
}
}
