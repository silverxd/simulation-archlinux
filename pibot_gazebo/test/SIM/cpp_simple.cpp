#include <gtest/gtest.h>
#include <ros/ros.h>
#include <include/RoboticsGtestExtenstion.hpp>
//#include <robot_test/master.h>

TEST(CPP_TopicTest, SIM)
{
    ros::master::V_TopicInfo master_topics;

    ros::Time timeout_time = ros::Time::now() + ros::Duration(30.0);
    std::set<std::string> found_topics;
    while (ros::ok() && ros::Time::now() < timeout_time)
    {
        ros::master::getTopics(master_topics);

        for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++)
        {
            const ros::master::TopicInfo &info = *it;
            //std::cerr << "topic_" << it - master_topics.begin() << ": " << info.name << " " << info.datatype << std::endl;
            std::string found_topic = info.name + " " + info.datatype;
            found_topics.insert(found_topic);
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

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "cpp_simple_test");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}
