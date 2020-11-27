#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <ignition/math/Pose3.hh>

#include <thread>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <chrono>
#include <tuple>

namespace gazebo
{
class WheelPlugin : public ModelPlugin
{

private:
  physics::JointPtr leftWheelJoint;
  physics::JointPtr rightWheelJoint;
  double rightVel = 0.0;
  double leftVel = 0.0;
  double rightVelPercentage = 0.0;
  double leftVelPercentage = 0.0;
  double leftVelInput = 0.0;
  double rightVelInput = 0.0;
  const double ROBOT_START_MOVING_SPEED = 8.0;

  // A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> rosNode;

  // ROS subscribers
  ros::Subscriber rosSubRightVel;
  ros::Subscriber rosSubLeftVel;

  //Joint state publisher
  ros::Publisher rightWheelJointStatePublisher;
  ros::Publisher leftWheelJointStatePublisher;

  // TEMP vel pub
  ros::Publisher rightWheelVelocityPublisher;
  ros::Publisher leftWheelVelocityPublisher;

  ros::Publisher modelPosePublisher;

  // A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue;
  // A thread the keeps running the rosQueue
  std::thread rosQueueThread;
  // Pointer to the model
  physics::ModelPtr model;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  int realmotors = -1;
  int debug = -1;
  double leftWheelCoefficient = 1.0;
  double rightWheelCoefficient = 1.0;
  std::chrono::time_point<std::chrono::system_clock> lastTime = std::chrono::system_clock::now();
  int throttle_count = 49;
  int throttle = 49;

public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    this->model = _parent;
    initializeRos();
    initializeRobotVariables();
    setCoefficients();
    getDebug();
    subscribe();
    createPublishers();
    initializeUpdate();
  }

  void subscribe() {
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
  }

  void createPublishers() {
    //Create joint state publishers
    this->rightWheelJointStatePublisher = this->rosNode->advertise<std_msgs::Int32>("/robot/wheel/right/position", 100);
    this->leftWheelJointStatePublisher = this->rosNode->advertise<std_msgs::Int32>("/robot/wheel/left/position", 100);

    this->rightWheelVelocityPublisher = this->rosNode->advertise<std_msgs::Float32>("/robot/wheel/right/velocity", 100);
    this->leftWheelVelocityPublisher = this->rosNode->advertise<std_msgs::Float32>("/robot/wheel/left/velocity", 100);

    this->modelPosePublisher = this->rosNode->advertise<geometry_msgs::Pose>("/robot/pose", 100);
  }

  void initializeUpdate() {
    // Spin up the queue helper thread.
    this->rosQueueThread =
        std::thread(std::bind(&WheelPlugin::QueueThread, this));

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&WheelPlugin::OnUpdate, this));
  }

  void initializeRos() {
    srand(time(NULL));
    // Initialize ros, if it has not already been initialized.
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");
      return;
    }
    // Create our ROS node.
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
  }

  void initializeRobotVariables() {
    this->leftWheelJoint = this->model->GetJoint("left_wheel_hinge");
    this->rightWheelJoint = this->model->GetJoint("right_wheel_hinge");
  }

  void OnLeftVelCmd(const std_msgs::Float32ConstPtr &msg)
  {
    //ROS_INFO_STREAM("left msg->data is " << msg->data);
    this->leftVelInput = msg->data;
    this->leftVelPercentage = this->leftWheelCoefficient * msg->data;
    this->leftVel = this->getVelocity(this->leftVelPercentage);
    //ROS_INFO_STREAM("leftvel is " << this->leftVel);
  }

  void OnRightVelCmd(const std_msgs::Float32ConstPtr &msg)
  {
    //ROS_INFO_STREAM("right msg->data is " << msg->data);
    this->rightVelInput = msg->data;
    this->rightVelPercentage = this->rightWheelCoefficient * msg->data;
    this->rightVel = this->getVelocity(this->rightVelPercentage);
    //ROS_INFO_STREAM("rightvel is " << this->rightVel);
  }

  double addNoise(double velocity)
  {
    int probability = rand() % 100;
    int random = (rand() + 1) % std::max(3, ((int)(velocity / 10)));

    if (0 <= probability && probability < 10)
    {
      velocity += random;
    }
    else if (10 <= probability && probability < 20)
    {
      velocity -= random;
    }
    return velocity;
  }

  void setCoefficients()
  {
    if (getRealMotors() == 1 && leftWheelCoefficient == 1 && rightWheelCoefficient == 1)
    {
      int probability = rand() % 100;
      double coefficient = (rand() % 30 + 70.0) / 100.0;

      if (0 <= probability && probability < 50)
      {
        leftWheelCoefficient = coefficient;
        ROS_INFO_STREAM("Left wheel coefficient: " << leftWheelCoefficient);
      }
      else if (50 <= probability && probability < 100)
      {
        rightWheelCoefficient = coefficient;
        ROS_INFO_STREAM("Right wheel coefficient: " << rightWheelCoefficient);
      }
    }
  }

  int getRealMotors()
  {
    if (realmotors == -1)
    {
      rosNode->getParam("/realmotors", realmotors);
      ROS_INFO_STREAM("Realmotors: " << realmotors);
    }
    if (realmotors == -1)
      return 0;
    return realmotors;
  }

  void publishJointStates()
  {
    std_msgs::Int32 rightWheelPosition;
    std_msgs::Int32 leftWheelPosition;

    float radInDeg = 180 / 3.14;

    rightWheelPosition.data = (int)(this->rightWheelJoint->Position() * radInDeg);
    leftWheelPosition.data = (int)(this->leftWheelJoint->Position() * radInDeg);

    this->rightWheelJointStatePublisher.publish(rightWheelPosition);
    this->leftWheelJointStatePublisher.publish(leftWheelPosition);
    
    std_msgs::Float32 velocity;
    velocity.data = this->leftWheelJoint->GetVelocity(0);
    this->leftWheelVelocityPublisher.publish(velocity);
    velocity.data = this->rightWheelJoint->GetVelocity(0);
    this->rightWheelVelocityPublisher.publish(velocity);

    geometry_msgs::Pose pose;
    ignition::math::Pose3d modelPose = this->model->WorldPose();
    pose.position.x = modelPose.Pos().X();
    pose.position.y = modelPose.Pos().Y();
    pose.position.z = modelPose.Pos().Z();
    pose.orientation.x = modelPose.Rot().X();
    pose.orientation.y = modelPose.Rot().Y();
    pose.orientation.z = modelPose.Rot().Z();
    pose.orientation.w = modelPose.Rot().W();
    this->modelPosePublisher.publish(pose);
  }

  template <typename T>
  int getSign(T x)
  {
    return (x > 0) - (x < 0);
  }

  double getVelocity(float percentage)
  {
    float x = abs(percentage);
    double y = 0.0;
    if (x < ROBOT_START_MOVING_SPEED)
      return 0.0;
    else
      y = 123.5 + (-17.48109 - 115.48)/(1 + pow((x/33.14506), 1.694765));
    
    if (y < 0) ROS_INFO_STREAM("!!! Something wrong !!!");
      
    return getSign(percentage) * y;
  }

  std::tuple<double, double> addNoiseToVelocities()
  {
    double lvp = leftVelPercentage;
    double rvp = rightVelPercentage;

    if (getRealMotors() == 1)
    {
      double lvp = addNoise(leftVelPercentage);
      double rvp = addNoise(rightVelPercentage);
      leftVel = getVelocity(lvp);
      rightVel = getVelocity(rvp);
    }
    return std::make_tuple(lvp, rvp);
  }

  int getDebug()
  {
    if (debug == -1)
    {
      rosNode->getParam("/debug", debug);
      ROS_INFO_STREAM("Debug: " << debug);
    }
    if (debug == -1)
      return 0;
    return debug;
  }

  void printVelocities(std::tuple<double, double> velocities) {
    double left;
    double right;
    std::tie(left, right) = velocities;
    ROS_INFO_STREAM("Right wheel velocity: " << right);
    ROS_INFO_STREAM("Left wheel velocity: " << left);
  }

  void addNoisePrintVelocities()
  {
    int db = getDebug();
    if (db == 1)
    {
      std::chrono::duration<double> diff = std::chrono::system_clock::now() - lastTime;
      if (diff.count() >= 1)
      {
        printVelocities(addNoiseToVelocities());
        lastTime = std::chrono::system_clock::now();
      }
    }
  }

  void setVelocities()
  {
    this->leftWheelJoint->SetVelocity(0, this->leftVel);
    this->rightWheelJoint->SetVelocity(0, this->rightVel);
  }

  // Called by the world update start event
  void OnUpdate()
  {
    if (throttle_count++ >= throttle)
    {
      throttle_count = 0;
      setCoefficients();
      addNoisePrintVelocities();
      publishJointStates();
    }
    setVelocities();
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
