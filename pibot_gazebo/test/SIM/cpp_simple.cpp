#include <gtest/gtest.h>
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <include/RoboticsGtestExtenstion.hpp>
//#include <robot_test/master.h>
std::vector<double> pibot_loc{0, 0};

TEST(SIM, CPP_TopicTest)
{
    ros::master::V_TopicInfo master_topics;

    ros::Time timeout_time = ros::Time::now() + ros::Duration(20.0);
    std::set<std::string> found_topics;
    while (ros::ok() && ros::Time::now() < timeout_time)
    {
        ros::master::getTopics(master_topics);

        for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++)
        {
            const ros::master::TopicInfo &info = *it;
            std::string found_topic = info.name + " " + info.datatype;
            found_topics.insert(found_topic);
        }
        if (found_topics.size() > 6)
        {
            break;
        }

        ros::Duration(1.0).sleep();
    }

    std::set<std::string>::iterator it = found_topics.begin();
    int i = 0;
    while (it != found_topics.end())
    {
        std::cerr << "topic_" << i++ << ": " << (*it) << "\n";
        it++;
    }
    std::cerr << std::flush;

    if (found_topics.size() > 6)
        return;

    FAIL() << "No topics found";
}

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    int states = std::min({msg->name.size(), msg->pose.size(), msg->twist.size()});
    for (int i = 0; i < states; i++)
    {
        if (msg->name[i].compare("pibot") == 0)
        {
            pibot_loc = {msg->pose[i].position.x, msg->pose[i].position.y};
        }
    }
}

TEST(SIM, CPP_ReachedWall)
{
    std::vector<std::vector<double>> polygon{{-3, 0.75}, {3, 0.75}, {3, 1.28}, {-3, 1.28}};
    ros::Time timeout_time = ros::Time::now() + ros::Duration(20.0);
    testing::AssertionResult result = testing::AssertionFailure() << "Robot didn't reach wall";
    while (ros::ok() && ros::Time::now() < timeout_time)
    {
        std::cerr << "LOC: " << pibot_loc[0] << " " << pibot_loc[1] << "\n";
        testing::AssertionResult current_result = RoboticsGtestExtension::pointInPolygon(pibot_loc, polygon);
        if (current_result)
        {
            std::cerr << "ARRIVED\n";
            result = current_result;
            break;
        }
        ros::Duration(0.1).sleep();
    }
    std::cerr << std::flush;
    ASSERT_TRUE(result);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpp_simple_test");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/gazebo/model_states", 1, modelStatesCallback);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}
