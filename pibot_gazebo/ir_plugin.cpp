#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <thread>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <functional>
#include <limits>

namespace gazebo
{
class IRPlugin : public SensorPlugin
{
private:
  std::unique_ptr<ros::NodeHandle> rosNode;
  sensors::RaySensorPtr raySensor;
  ros::Publisher rawPublisher;
  ros::Publisher publisher;
  event::ConnectionPtr updateConnection;

  std::string topic;
  int noise = -1;
  int blind = -1;
  int realSensors = -1;
  double frontMaxDistance = 0.5;
  double rearRawConstant = 350;
  int throttle_count = 0;
  int throttle = 99;

public:
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
  {
    initializeRos();
    initializeOutsideVariables(sdf);
    initializeRosVariables(sensor);
  }

  void initializeRos() {
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");
      return;
    }
    rosNode.reset(new ros::NodeHandle("gazebo_client"));
  }

  void initializeOutsideVariables(sdf::ElementPtr& sdf) {
    topic = sdf->GetElement("topic")->GetValue()->GetAsString();
    std::hash<std::string> hasher;
    auto seed = hasher(topic);
    ROS_INFO_STREAM("seed " << seed);
    srand((seed + time(NULL)) % std::numeric_limits<unsigned int>::max());
    noise = getNoise();
    rearRawConstant += rand() % 150;
    ROS_INFO_STREAM("rearRawConstant " << rearRawConstant);
  }

  void initializeRosVariables(sensors::SensorPtr& sensor) {
    raySensor = std::dynamic_pointer_cast<sensors::RaySensor>(sensor);
    rawPublisher = rosNode->advertise<std_msgs::Float64MultiArray>("/robot/" + topic + "/raw", 1);
    publisher = rosNode->advertise<std_msgs::Float64>("/robot/" + topic + "/value", 1);
    updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&IRPlugin::OnUpdate, this));
  }

  void fillRawData(std_msgs::Float64MultiArray &msg)
  {
    std::vector<double> data;
    raySensor->Ranges(data);

    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = data.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "ranges";

    msg.data.clear();
    msg.data.insert(msg.data.end(), data.begin(), data.end());
  }

  void addNoise(double &minRange) {
    int probability = rand() % 100;
    double random = rand() % 100 + 1; // 1 .. 100
    if (0 <= probability && probability < 10) {
      minRange += (random / 1000.0);
    } else if (10 <= probability && probability < 13) {
      minRange = (random / 100.0);
    }
  }

  void addRealSensorNoise(double &minRange) {
    int probability = rand() % 100;
    double random = rand() % 100 + 1;
    if (0 <= probability && probability < 100 * minRange * 3) {
      ROS_INFO_STREAM("Adding noise:" << minRange * (random / 1000.0));
      minRange += minRange * (random / 1000.0);
    } else if (10 <= probability && probability < 20) {
      minRange = (random / 100.0);
    }
  }

  void addNoiseIfNoiseEnabled(double &minRange) {
    if (getNoise() == 1) {
      addNoise(minRange);
    }
  }

  void addRealSensorsNoiseIfEnabled(double &minRange) {
    if (getRealSensors() == 1 && !isRearSensor()) {
      addRealSensorNoise(minRange);
    }
  }

  void addBlindIfBlindEnabled(double& minRange) {
    if (getBlind() == 1 && !isRearSensor()) {
      minRange = std::min(minRange, frontMaxDistance);
    }
  }

  int getNoise() {
    if (noise == -1) {
      rosNode->getParam("/noise", noise);
      ROS_INFO_STREAM("Noise: " << noise);
    }
    if (noise == -1) return 0;
    return noise;
  }

  int getRealSensors() {
    if (realSensors == -1) {
      rosNode->getParam("/realsensors", realSensors);
      ROS_INFO_STREAM("Realsensors: " << realSensors);
    }
    if (realSensors == -1) return 0;
    return realSensors;
  }

  int getBlind() {
    if (blind == -1) {
      rosNode->getParam("/blind", blind);
      ROS_INFO_STREAM("Rear blindness: " << blind);
    }
    if (blind == -1) return 0;
    return blind;
  }

  bool isRearSensor() {
    return raySensor->RangeMax() < 0.5;
  }

  void convertToRaw(double &minRange) {
    minRange = 503.36712 - 24.36869 * minRange * 100 + 0.2946128 * pow(minRange * 100, 2) + rearRawConstant;
  }

  void convertToRawIfRearSensor(double &minRange) {
    if (isRearSensor()) {
      convertToRaw(minRange);
    }
  }

  void addModifiers(double &minRange) {
    addBlindIfBlindEnabled(minRange);
    addNoiseIfNoiseEnabled(minRange);
  }

  double findMinimumRange() {
    double minRange = raySensor->RangeMax();
    for (int i = 0; i < raySensor->RangeCount(); i++)
    {
      double range = raySensor->Range(i);
      minRange = std::min(range, minRange);
    }
    return minRange;
  }

  void publishRawData() {
    std_msgs::Float64MultiArray rawMsg;
    fillRawData(rawMsg);
    rawPublisher.publish(rawMsg);
  }

  void publishMinValue() {
    std_msgs::Float64 msg;
    msg.data = getModifiedMinimumRange();
    publisher.publish(msg);
  }

  double getModifiedMinimumRange() {
    double minRange = findMinimumRange();
    minRange = std::max(minRange, raySensor->RangeMin());
    addModifiers(minRange);
    convertToRawIfRearSensor(minRange);
    return minRange;
  }

  void OnUpdate()
  {
    if (throttle_count++ >= throttle)
    {
      throttle_count = 0;
      raySensor->SetActive(false);

      publishRawData();
      publishMinValue();

      raySensor->SetActive(true);
    }
  }
};
GZ_REGISTER_SENSOR_PLUGIN(IRPlugin)
}
