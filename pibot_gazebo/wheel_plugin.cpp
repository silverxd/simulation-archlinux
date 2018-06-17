#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include <ros/console.h>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

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
        if (!ros::isInitialized())
        {
          int argc = 0;
          char **argv = NULL;
          ros::init(argc, argv, "gazebo_client",
              ros::init_options::NoSigintHandler);
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
              "/" + this->model->GetName() + "/right_vel_cmd",
              1,
              boost::bind(&WheelPlugin::OnRightVelCmd, this, _1),
              ros::VoidPtr(), &this->rosQueue);

        ros::SubscribeOptions soLeftVel =
          ros::SubscribeOptions::create<std_msgs::Float32>(
              "/" + this->model->GetName() + "/left_vel_cmd",
              1,
              boost::bind(&WheelPlugin::OnLeftVelCmd, this, _1),
              ros::VoidPtr(), &this->rosQueue);

        this->rosSubRightVel = this->rosNode->subscribe(soRightVel);
        this->rosSubLeftVel = this->rosNode->subscribe(soLeftVel);

        // Spin up the queue helper thread.
        this->rosQueueThread =
          std::thread(std::bind(&WheelPlugin::QueueThread, this));

        // Listen to the update event. This event is broadcast every simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&WheelPlugin::OnUpdate, this));
        ROS_INFO("Load function finished");
      }

      void OnLeftVelCmd(const std_msgs::Float32ConstPtr &msg) {
        this->leftVel = msg->data;
        ROS_INFO("left_vel_cmd received");
      }

      void OnRightVelCmd(const std_msgs::Float32ConstPtr &msg) {
        this->rightVel = msg->data;
        ROS_INFO("right_vel_cmd received");
      }

      // Called by the world update start event
      void OnUpdate() {
        // Apply velocity to wheel joints
        this->leftWheelJoint->SetVelocity(0, this->leftVel);
        this->rightWheelJoint->SetVelocity(0, this->rightVel);
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
