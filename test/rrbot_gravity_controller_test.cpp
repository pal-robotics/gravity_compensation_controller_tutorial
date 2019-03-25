#include <gtest/gtest.h>
#include <ros/ros.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>
#include <gravity_compensation_controller_tutorial/gravity_compensation_controller.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>

class GravityCompensationTutorialTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    load_srv_ = nh_.serviceClient<controller_manager_msgs::LoadController>(
        "controller_manager/load_controller");
    unload_srv_ = nh_.serviceClient<controller_manager_msgs::UnloadController>(
        "controller_manager/unload_controller");
    list_srv_ = nh_.serviceClient<controller_manager_msgs::ListControllers>(
        "controller_manager/list_controllers");
    switch_srv_ = nh_.serviceClient<controller_manager_msgs::SwitchController>(
        "controller_manager/switch_controller");
    effort_gazebo_ =
        nh_.serviceClient<gazebo_msgs::ApplyJointEffort>("gazebo/apply_joint_effort");
    joint_properties_gazebo_ =
        nh_.serviceClient<gazebo_msgs::GetJointProperties>("gazebo/get_joint_properties");

    load_msg_.request.name = controller_name_;
    unload_msg_.request.name = controller_name_;
    switch_msg_.request.start_controllers.push_back(controller_name_);
  }

  bool areServicesReady(const ros::Duration &timeout)
  {
    if ((!load_srv_.waitForExistence(timeout)) || (!unload_srv_.waitForExistence(timeout)) ||
        (!list_srv_.waitForExistence(timeout)) || (!switch_srv_.waitForExistence(timeout)) ||
        (!effort_gazebo_.waitForExistence(timeout)) ||
        (!joint_properties_gazebo_.waitForExistence(timeout)))
      return false;

    return true;
  }

  ros::NodeHandle nh_;
  ros::ServiceClient load_srv_;
  ros::ServiceClient unload_srv_;
  ros::ServiceClient list_srv_;
  ros::ServiceClient switch_srv_;
  ros::ServiceClient effort_gazebo_;
  ros::ServiceClient joint_properties_gazebo_;
  controller_manager_msgs::LoadController load_msg_;
  controller_manager_msgs::UnloadController unload_msg_;
  controller_manager_msgs::ListControllers list_msg_;
  controller_manager_msgs::SwitchController switch_msg_;

  const std::string controller_name_ = "gravity_compensation_tutorial";
};

TEST_F(GravityCompensationTutorialTest, SpawnController)
{
  EXPECT_TRUE(areServicesReady(ros::Duration(15.0)));

  EXPECT_TRUE(list_srv_.call(list_msg_));
  EXPECT_TRUE(list_msg_.response.controller.empty());

  EXPECT_TRUE(load_srv_.call(load_msg_));
  EXPECT_TRUE(load_msg_.response.ok);
  EXPECT_TRUE(list_srv_.call(list_msg_));
  EXPECT_FALSE(list_msg_.response.controller.empty());
  EXPECT_EQ(list_msg_.response.controller[0].name, controller_name_);
  EXPECT_EQ(list_msg_.response.controller[0].state, "stopped");

  EXPECT_TRUE(switch_srv_.call(switch_msg_));
  EXPECT_TRUE(switch_msg_.response.ok);
  EXPECT_TRUE(list_srv_.call(list_msg_));
  EXPECT_FALSE(list_msg_.response.controller.empty());
  EXPECT_EQ(list_msg_.response.controller[0].name, controller_name_);
  EXPECT_EQ(list_msg_.response.controller[0].state, "running");

  switch_msg_.request.start_controllers.clear();
  switch_msg_.request.stop_controllers.push_back(controller_name_);
  EXPECT_TRUE(switch_srv_.call(switch_msg_));
  EXPECT_TRUE(switch_msg_.response.ok);
  EXPECT_TRUE(list_srv_.call(list_msg_));
  EXPECT_FALSE(list_msg_.response.controller.empty());
  EXPECT_EQ(list_msg_.response.controller[0].name, controller_name_);
  EXPECT_EQ(list_msg_.response.controller[0].state, "stopped");

  EXPECT_TRUE(unload_srv_.call(unload_msg_));
  EXPECT_TRUE(unload_msg_.response.ok);
  EXPECT_TRUE(list_srv_.call(list_msg_));
  EXPECT_TRUE(list_msg_.response.controller.empty());
}

TEST_F(GravityCompensationTutorialTest, MultiplePositions)
{
  EXPECT_TRUE(areServicesReady(ros::Duration(15.0)));

  gazebo_msgs::ApplyJointEffort effort_msg;
  effort_msg.request.effort = 40.0;
  effort_msg.request.joint_name = "single_rrbot_joint1";
  effort_msg.request.duration = ros::Duration(2.0);
  EXPECT_TRUE(effort_gazebo_.call(effort_msg));
  ros::Duration(6.0).sleep();

  gazebo_msgs::GetJointProperties joint_properties_msg;
  joint_properties_msg.request.joint_name = "single_rrbot_joint1";
  EXPECT_TRUE(joint_properties_gazebo_.call(joint_properties_msg));
  EXPECT_NEAR(joint_properties_msg.response.position[0], M_PI / 2, 1.e-3);

  joint_properties_msg.request.joint_name = "single_rrbot_joint2";
  EXPECT_TRUE(joint_properties_gazebo_.call(joint_properties_msg));
  EXPECT_NEAR(joint_properties_msg.response.position[0], M_PI / 2, 1.e-3);

  EXPECT_TRUE(load_srv_.call(load_msg_));
  EXPECT_TRUE(load_msg_.response.ok);
  EXPECT_TRUE(switch_srv_.call(switch_msg_));
  EXPECT_TRUE(switch_msg_.response.ok);

  effort_msg.request.effort = -40.0;
  effort_msg.request.joint_name = "single_rrbot_joint2";
  EXPECT_TRUE(effort_gazebo_.call(effort_msg));

  ros::Duration(8.0).sleep();

  joint_properties_msg.request.joint_name = "single_rrbot_joint1";
  EXPECT_TRUE(joint_properties_gazebo_.call(joint_properties_msg));
  EXPECT_NEAR(joint_properties_msg.response.position[0], M_PI / 2, 1.e-3);

  joint_properties_msg.request.joint_name = "single_rrbot_joint2";
  EXPECT_TRUE(joint_properties_gazebo_.call(joint_properties_msg));
  EXPECT_NEAR(joint_properties_msg.response.position[0], -0.3, 2.e-1);

  double previous_position = joint_properties_msg.response.position[0];

  ros::Duration(10.0).sleep();

  joint_properties_msg.request.joint_name = "single_rrbot_joint1";
  EXPECT_TRUE(joint_properties_gazebo_.call(joint_properties_msg));
  EXPECT_NEAR(joint_properties_msg.response.position[0], M_PI / 2, 1.e-3);

  joint_properties_msg.request.joint_name = "single_rrbot_joint2";
  EXPECT_TRUE(joint_properties_gazebo_.call(joint_properties_msg));
  EXPECT_NEAR(joint_properties_msg.response.position[0], previous_position, 1.e-1);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "rrbot_gravity_controller_test");

  ros::NodeHandle nh;
  ros::Time::waitForValid();

  return RUN_ALL_TESTS();
}
