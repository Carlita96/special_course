/*
This class can be used to record a cloud from a topic in a pcd file.
    * ROS Parameters that can be used:
        * For save file:
            - outputDirectory: directory where the output will be saved (IT NEEDS TO EXISTS ALREADY)
            - outputFileName: name that the ouput file will have. The date time of the recording will be
                added to avoid overwritting it.
        * For topic listening:
            - topicName: topic that will be listened and which cloud will be recorded.
            - numberOfMessages: number of messages that will be saved in the pcd file.
*/

#ifndef RECORDER_HPP
#define RECORDER_HPP

// Import necessary libraries
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <experimental/filesystem>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>    
#include <boost/foreach.hpp>

class RecordClass
{
    public:
        RecordClass();

        ~RecordClass();

        void initParameters(ros::NodeHandle &nodeHangle); 

        void topicCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg);

    private:
        // Create ros objects necessary for the class
        ros::NodeHandle nodeHandle;
        ros::Subscriber subscriber;

        // Create attributes necessary
        // For saving the file
        std::string outputDirectory;
        std::string outputFileName;
        // To listen to the topic
        std::string topicName;
        int numberOfMessagesRecorded = 0;
        int numberOfMessages;

        // Create clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::PCLPointCloud2 concatentedPointCloud2;

        std::string datetime();
        bool hasEnding (std::string const &fullString, std::string const &ending);
        void saveFile();
}; 

#endif // RECORDER_HPP 