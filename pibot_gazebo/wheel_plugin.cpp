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

#include <stdlib.h>
#include <time.h>

#include <math.h>

namespace gazebo
{
class WheelPlugin : public ModelPlugin
{

private:
  physics::JointPtr leftWheelJoint;
  physics::JointPtr rightWheelJoint;
  double rightVel;
  double leftVel;
  double rightVelPercentage;
  double leftVelPercentage;

  // A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> rosNode;

  // ROS subscribers
  ros::Subscriber rosSubRightVel;
  ros::Subscriber rosSubLeftVel;

  //Joint state publisher
  ros::Publisher rightWheelJointStatePublisher;
  ros::Publisher leftWheelJointStatePublisher;

  // A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue;

  // A thread the keeps running the rosQueue
  std::thread rosQueueThread;

  // Pointer to the model
  physics::ModelPtr model;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  int realmotors = -1;
  double leftWheelCoefficient = 1.0;
  double rightWheelCoefficient = 1.0;

public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Store the pointer to the model
    this->model = _parent;
    srand(time(NULL));
    // Initialize ros, if it has not already been initialized.
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");
      return;
    }

    this->leftWheelJoint = this->model->GetJoint("left_wheel_hinge");
    this->rightWheelJoint = this->model->GetJoint("right_wheel_hinge");

    this->rightVel = 0.0;
    this->leftVel = 0.0;
    this->rightVelPercentage = 0.0;
    this->leftVelPercentage = 0.0;

    // Create our ROS node.
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    addCoefficent();

    // Create named topics, and subscribe to them.
    ros::SubscribeOptions soRightVel =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/robot/wheel/right/vel_cmd",
            1,
            boost::bind(&WheelPlugin::OnRightVelCmd, this, _1),
            ros::VoidPtr(), &this->rosQueue);

    ros::SubscribeOptions soLeftVel =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/robot/wheel/left/vel_cmd",
            1,
            boost::bind(&WheelPlugin::OnLeftVelCmd, this, _1),
            ros::VoidPtr(), &this->rosQueue);

    this->rosSubRightVel = this->rosNode->subscribe(soRightVel);
    this->rosSubLeftVel = this->rosNode->subscribe(soLeftVel);

    //Create joint state publishers
    this->rightWheelJointStatePublisher = this->rosNode->advertise<std_msgs::Int32>("/robot/wheel/right/position", 1000);
    this->leftWheelJointStatePublisher = this->rosNode->advertise<std_msgs::Int32>("/robot/wheel/left/position", 1000);

    // Spin up the queue helper thread.
    this->rosQueueThread =
        std::thread(std::bind(&WheelPlugin::QueueThread, this));

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&WheelPlugin::OnUpdate, this));
  }

  void OnLeftVelCmd(const std_msgs::Float32ConstPtr &msg)
  {
    //ROS_INFO_STREAM("msg->data is" << msg->data);
    this->leftVel = this->getVelocity(leftWheelCoefficient * msg->data);
    this->leftVelPercentage = msg->data;
    //ROS_INFO_STREAM("leftvel is " << this->leftVel);
  }

  void OnRightVelCmd(const std_msgs::Float32ConstPtr &msg)
  {
    //ROS_INFO_STREAM("msg->data is" << msg->data);
    this->rightVel = this->getVelocity(rightWheelCoefficient * msg->data);
    this->rightVelPercentage = msg->data;
    //ROS_INFO_STREAM("rightvel is " << this->rightVel);
  }

  void addCoefficent() {
    int rm = getRealMotors();
    if (rm == 1 && leftWheelCoefficient == 1 && rightWheelCoefficient == 1) {
      int probability = rand() % 100;
      double coefficient = (rand() % 20 + 80) / 100.0;
      if (0 <= probability && probability < 50) {
        leftWheelCoefficient = coefficient;
        ROS_INFO_STREAM("Left wheel coefficient: " << leftWheelCoefficient);
      } else if (50 <= probability && probability < 100) {
        rightWheelCoefficient = coefficient;
        ROS_INFO_STREAM("Right wheel coefficient: " << rightWheelCoefficient);
      }
    }
  }

  int getRealMotors() {
    if (realmotors == -1) {
      rosNode->getParam("/realmotors", realmotors);
      ROS_INFO_STREAM("Realmotors: " << realmotors);
    }
    if (realmotors == -1) return 0;
    return realmotors;
  }

  void publishJointStates()
  {
    std_msgs::Int32 rightWheelPosition;
    std_msgs::Int32 leftWheelPosition;

    float radInDeg = 180 / 3.14;

    rightWheelPosition.data = (((int)(this->rightWheelJoint->Position() * radInDeg))/4)*4;
    leftWheelPosition.data = (((int)(this->leftWheelJoint->Position() * radInDeg))/4)*4;

    this->rightWheelJointStatePublisher.publish(rightWheelPosition);
    this->leftWheelJointStatePublisher.publish(leftWheelPosition);
  }

  template <typename T>
  int getSign(T x)
  {
    return (x > 0) - (x < 0);
  }

  double getVelocity(float percentage)
  {
    float x = abs(percentage);
    if (x < 11)
      return 0.0;

    double y = 143.7422 + (-97.04175 - 143.7422) / (1 + pow((x / 20.43845), 0.9319634));

    return getSign(percentage) * y;
  }

  // Called by the world update start event
  void OnUpdate()
  {
    
    addCoefficent();
    // Apply velocity to wheel joints
    this->leftWheelJoint->SetVelocity(0, this->leftVel);
    this->rightWheelJoint->SetVelocity(0, this->rightVel);
    publishJointStates();
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
GZ_REGISTER_MODEL_PLUGIN(WheelPlugin)
} // namespace gazebo
