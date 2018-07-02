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

namespace gazebo
{
  class WheelPlugin : public ModelPlugin
  {

    private:
      physics::JointPtr leftWheelJoint;
      physics::JointPtr rightWheelJoint;
      float rightVel;
      float leftVel;

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

    public:
      void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
        // Store the pointer to the model
        this->model = _parent;

        // Initialize ros, if it has not already been initialized.
        if (!ros::isInitialized()) {
          ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");
          return;
        }

        this->leftWheelJoint = this->model->GetJoint("left_wheel_hinge");
        this->rightWheelJoint = this->model->GetJoint("right_wheel_hinge");

        this->rightVel = 0.0;
        this->leftVel = 0.0;

        // Create our ROS node.
        this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

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

      void OnLeftVelCmd(const std_msgs::Float32ConstPtr &msg) {
        this->leftVel = msg->data;
      }

      void OnRightVelCmd(const std_msgs::Float32ConstPtr &msg) {
        this->rightVel = msg->data;
      }

      void publishJointStates() {
        std_msgs::Int32 rightWheelPosition;
        std_msgs::Int32 leftWheelPosition;

        float radInDeg = 180 / 3.14;

        rightWheelPosition.data = (int) (this->rightWheelJoint->Position() * radInDeg);
        leftWheelPosition.data = (int) (this->leftWheelJoint->Position() * radInDeg);

        this->rightWheelJointStatePublisher.publish(rightWheelPosition);
        this->leftWheelJointStatePublisher.publish(leftWheelPosition);
      }

      // Called by the world update start event
      void OnUpdate() {
        // Apply velocity to wheel joints
        this->leftWheelJoint->SetVelocity(0, this->leftVel);
        this->rightWheelJoint->SetVelocity(0, this->rightVel);
        publishJointStates();
      }

    private:
      void QueueThread() {
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
          this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
      }
  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(WheelPlugin)
}
