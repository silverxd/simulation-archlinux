#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>

#include <thread>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <math.h>

namespace gazebo
{
class GrabberPlugin : public ModelPlugin
{

private:
  physics::JointPtr poleJoint;
  physics::JointPtr leftGrabberJoint;
  physics::JointPtr rightGrabberJoint;

  double polePos;
  double leftPos;
  double rightPos;

  int updateCount = 0;

  // A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> rosNode;

  // ROS subscribers
  ros::Subscriber rosSubClosePercentage;
  ros::Subscriber rosSubHeightPercentage;

  // A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue;

  // A thread the keeps running the rosQueue
  std::thread rosQueueThread;

  // Pointer to the model
  physics::ModelPtr model;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;

public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Store the pointer to the model
    this->model = _parent;

    // Initialize ros, if it has not already been initialized.
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");
      return;
    }

    // Create our ROS node.
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    this->poleJoint = this->model->GetJoint("grabber_pole");
    this->leftGrabberJoint = this->model->GetJoint("grabber_left_arm_joint_to_grabber_mount");
    this->rightGrabberJoint = this->model->GetJoint("grabber_right_arm_joint_to_grabber_mount");

    this->polePos = 0.314;
    this->leftPos = 0.25 * M_PI * 0.7;
    this->rightPos = -0.25 * M_PI * 0.7;

    // Create named topics, and subscribe to them.
    ros::SubscribeOptions soHeight = ros::SubscribeOptions::create<std_msgs::Float32>(
        "/robot/grabber/height_cmd",
        1,
        boost::bind(&GrabberPlugin::OnHeightCmd, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    ros::SubscribeOptions soClose =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/robot/grabber/close_cmd",
            1,
            boost::bind(&GrabberPlugin::OnCloseCmd, this, _1),
            ros::VoidPtr(), &this->rosQueue);

    this->rosSubHeightPercentage = this->rosNode->subscribe(soHeight);
    this->rosSubClosePercentage = this->rosNode->subscribe(soClose);

    // Spin up the queue helper thread.
    this->rosQueueThread =
        std::thread(std::bind(&GrabberPlugin::QueueThread, this));

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GrabberPlugin::OnUpdate, this));
  }

  void OnHeightCmd(const std_msgs::Float32ConstPtr &msg)
  {
    float percentage = msg->data;
    this->polePos = (-0.005 * 0.8 * percentage + 0.5) * M_PI;
  }

  void OnCloseCmd(const std_msgs::Float32ConstPtr &msg)
  {
    float percentage = msg->data;
    this->leftPos = 0.0025 * M_PI * 0.7 * percentage;
    this->rightPos = -this->leftPos;
  }

  void setJointPos(physics::JointPtr joint, double destination, double force) {
    double currentPos = joint->Position();
    
    if (destination - currentPos > 0) {
      joint->SetForce(0, force);
    } else if (currentPos - destination > 0) {
      joint->SetForce(0, -force);
    } else {
      joint->SetForce(0, 0);
    }
  }

  // Called by the world update start event
  void OnUpdate()
  {
    double force = 0.1;
    setJointPos(this->poleJoint, this->polePos, force);
    //this->poleJoint->SetPosition(0, this->polePos);
    setJointPos(this->leftGrabberJoint, this->leftPos, force);
    //this->leftGrabberJoint->SetPosition(0, this->leftPos);
    setJointPos(this->rightGrabberJoint, this->rightPos, force);
    //this->rightGrabberJoint->SetPosition(0, this->rightPos);
  }

private:
  void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }
};
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GrabberPlugin)
} // namespace gazebo
