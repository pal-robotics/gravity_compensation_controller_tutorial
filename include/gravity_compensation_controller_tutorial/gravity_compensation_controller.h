#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <rbdl/Dynamics.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace force_control
{
/*! Enum that defines the type of joint */
enum JointType
{
  STATIC, /*!< Joint not actuated in the controller */
  ACTUATED, /*!< Joint actuaded in the controller. Resource of the controller */
  ACTUATED_NO_CONTROL /*!< Joint commanded constantly with zero effort. Resource of the controller */
};


//! Gravity Compensation Parameters
/*!
  Parameters related with the gravity compensation controller.
    - static_friction: friction of the motor that needs to be overcome to start moving the joint
    - viscous_friction: friction of the motor when the joint is being moved.
    - velocity_tolerance: velocity at which static friction is not taken in account.

  This parameters are specific for each actuated joint, and could be modify by the dynamic reconfigure.
*/
struct GravityCompensationParameters
{
  double static_friction = 0.0;
  double viscous_friction = 0.0;
  double velocity_tolerance = 0.0;
};

//! Actuator Parameters
/*!
  Parameters related with the actuators/motor of the actuated joints.
    - motor_torque_constant: motor torque constatn specs.
    - reduction_ratio: gear reduction ratio.

  This parameters are specific for each motor and should not be modfied. Those are loaded in the param server from the config files.
*/
struct ActuatorParameters
{
  double motor_torque_constant = 0.0;
  double reduction_ratio = 0.0;
};

//! Actuated Joint
/*!
  Struct that contains the parameters and the joint handle interface to send commands to the actuator.
    - joint_handle: control interface used to read the actual the actual values of the joint (in terms of position, velocity and effort),
    and that allows to send commands to the joint to send it to a specific position velocity or effort.
    - actuator_parameters: parameters of the actuator used to calculate the desired effort.
    - friction_parameters: parameters of the gravity compensation controller used to calculate the desired effort.
*/
struct ActuatedJoint
{
  hardware_interface::JointHandle joint_handle;
  ActuatorParameters actuator_parameters;
  GravityCompensationParameters friction_parameters;
};

//! Gravity Compensation Tutorial
/*!
  Gravity compensation controller which serves as a tutorial to the user on how to implement his own controllers. 
  This class derives from the controller_interface::ControllerBase class from ros_control.
*/
class GravityCompensationTutorial : public controller_interface::ControllerBase
{
public:

  //! initRequest
  /*!
    Function to initialize the controller that is launched when the controller is loaded
      - robot_hw: the robot hardware interface
      - root_nh
      - controller_nh: NodeHandle in the namespace of the controller
      - claimed_resources: actuated joints for this controller
      returns true if initializon was successful, false otherwise 

  */
  bool initRequest(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                   ros::NodeHandle& controller_nh, ClaimedResources& claimed_resources);

  //! update
  /*!
    Function that performs the control loop at every control cycle once it is started and before it is stopped.
      - time: the current time
      - period: the time passed since thje last time call
  */
  void update(const ros::Time& time, const ros::Duration& period);
  
  //! starting
  /*!
    Function to start the controller once it has been loaded. It is call before the first call to update.
      - time: the current time
  */
  void starting(const ros::Time& time);

  //! starting
  /*!
    Function to stop the controller once it has been loaded. It is call after the last call to update
      - time: the current time
  */
  void stopping(const ros::Time& time);

private:

   //! init
  /*!
    Function to initialize controller that is called from the initRequest function
  */
  bool init(hardware_interface::EffortJointInterface* effort_iface,
            hardware_interface::JointStateInterface* joint_state_iface,
            ros::NodeHandle& root_nh, ros::NodeHandle& control_nh);


  RigidBodyDynamics::Model rbdl_model_;  /*!< Robot model from RBDL */

  Eigen::VectorXd q_zero_;    /*!< Zero vector with joint_names size */
  Eigen::VectorXd tau_cmd_;   /*!< Vector with the necessary torque to maintain gravity */
  Eigen::VectorXd q_act_;     /*!< Vector with the current position of the joint states */

  std::vector<std::string> joint_names_;                  /*!< Vector with the joint names of all the joints, including the actuated and the static ones */
  std::map<std::string, JointType> joint_types_;          /*!< Map to define which joint are actuated and which one are static */
  std::map<std::string, ActuatedJoint> actuated_joints_;  /*!< Map with the actuated joints and his parameters + hardware interface to read and write commands*/
  std::map<std::string, hardware_interface::JointStateHandle> static_joints_;   /*!< Map with the static joints and his hardware interface to read the current position */

  ddynamic_reconfigure::DDynamicReconfigurePtr ddr_;     /*!< Dyanic reconfigure */
};

}

// Exports the GravityCompensationTutorial as a plugin of ros_control
PLUGINLIB_EXPORT_CLASS(force_control::GravityCompensationTutorial, controller_interface::ControllerBase)
