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

namespace gazebo
{
class IRPlugin : public SensorPlugin
{
private:
  std::unique_ptr<ros::NodeHandle> rosNode;
  sensors::RaySensorPtr raySensor;
  ros::Publisher rawPublisher;
  ros::Publisher publisher;
  std::string topic;
  event::ConnectionPtr updateConnection;
  int noise = -1;
  int blind = -1;
  double rearBlindMinimum = 0.05;

public:
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
  {
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");
      return;
    }

    srand(time(NULL));

    rosNode.reset(new ros::NodeHandle("gazebo_client"));
    raySensor = std::dynamic_pointer_cast<sensors::RaySensor>(sensor);
    topic = sdf->GetElement("topic")->GetValue()->GetAsString();

    noise = getNoise();

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
    double random = rand() % 100 + 1;
    if (0 <= probability && probability < 10) {
      minRange += (random / 1000.0);
    } else if (10 <= probability && probability < 20) {
      minRange = (random / 100.0);
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

  int getBlind() {
    if (!isRearSensor()) {
      blind = 0;
    }
    if (blind == -1) {
      rosNode->getParam("/blind", blind);
      ROS_INFO_STREAM("Rear blindness: " << noise);
    }
    if (blind == -1) return 0;
    return blind;
  }

  bool isRearSensor() {
    return raySensor->RangeMax() < 0.2;
  }

  void addModifiers(double &minRange) {
    int ns = getNoise();
    if (ns == 1) {
      addNoise(minRange);
    }

    int bl = getBlind();
    if (bl == 1 && isRearSensor()) {
      minRange = std::min(minRange, rearBlindMinimum);
    }
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

  void OnUpdate()
  {
    raySensor->SetActive(false);

    double minRange = findMinimumRange();
    addModifiers(minRange);
    minRange = std::max(minRange, raySensor->RangeMin());

    //Publish raw data
    std_msgs::Float64MultiArray rawMsg;
    fillRawData(rawMsg);
    rawPublisher.publish(rawMsg);

    //Publish min value
    std_msgs::Float64 msg;
    msg.data = minRange;
    publisher.publish(msg);

    raySensor->SetActive(true);
  }
};
GZ_REGISTER_SENSOR_PLUGIN(IRPlugin)
} // namespace gazebo
