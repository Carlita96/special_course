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

#include "recorder.hpp"

RecordClass::RecordClass()
{
    // Read the ros parameters explained in the intro
    initParameters(nodeHandle);

    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr( new pcl::PointCloud<pcl::PointXYZ>() );

    // Create a subscriber to the topic
    subscriber = nodeHandle.subscribe<sensor_msgs::PointCloud2>( topicName, 10000, &RecordClass::topicCallBack, this );
    ROS_INFO_STREAM("Recorder init OK");
}

RecordClass::~RecordClass(){
}

void RecordClass::initParameters(ros::NodeHandle &nodeHangle) 
{
    /*
    Method that initialize the parameters declared in ROS. These are:
        - outputDirectory: directory where the output will be saved (IT NEEDS TO EXISTS ALREADY)
        - outputFileName: name that the ouput file will have. The date time of the recording will be
            added to avoid overwritting it.
        - topicName: topic that will be listened and which cloud will be recorded.
        - numberOfMessages: number of messages that will be saved in the pcd file.
    */
    nodeHandle.param<std::string>( "outputFileName", outputFileName, "pcdFile");
    nodeHandle.param<std::string>( "outputDirectory", outputDirectory, std::string( "./" ));
    nodeHandle.param<std::string>( "topicName", topicName, std::string( "/zed2/zed_node/point_cloud/cloud_registered" ));
    nodeHandle.param<int>( "numberOfMessages", numberOfMessages, 1 );
}

void RecordClass::topicCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    /*
    Callback for the subscriber to the topic:
    * Input parameters:
        - msg: It is the pointer to the cloud sent by the publisher. 
    */

    // Transform the message to a pcl::PCLPointCloud2
    pcl::PCLPointCloud2 pointCloud2;
    pcl_conversions::toPCL(*msg, pointCloud2);
    // Concatenate the cloud obtained into concatenatedPointCloud2
    pcl::concatenatePointCloud (concatentedPointCloud2, pointCloud2, concatentedPointCloud2); 
    // Listen to an exact amount of messages
    numberOfMessagesRecorded += 1;
    if (numberOfMessagesRecorded == numberOfMessages) {
        // Shutdown the subscriber
        subscriber.shutdown();
        // Transform the pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZ>
        pcl::fromPCLPointCloud2(concatentedPointCloud2 , *cloud);
        ROS_INFO_STREAM("Total pointcloud read of size: " << cloud->points.size());
        // Save the file
        saveFile();
    }
}

std::string RecordClass::datetime()
{
    /*
    Method that retrieves the datetime of the moment with format std::string. 
    */
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%Y-%m-%d_%H-%M-%S",timeinfo);
    return std::string(buffer);
}

bool RecordClass::hasEnding (std::string const &fullString, std::string const &ending) 
{
    /*
    Method that checks if the string received finishes as other string.
    * Input parameters:
        - fullString: string which end will be checked.
        - ending: string to check if is the end of the other string.
    * Returns:
        - It will return true in case the first string finishes with the second one
            and false otherwise.
    */
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

void RecordClass::saveFile()
{
    /*
    Method that saves the cloud into a PCD file with name and directory defined in ROS parameters
    - TODO: Check directory start or create it
    */
    // Add the date time to the file name and the termination .pcd afterwards
    if (not hasEnding(outputFileName, ".pcd")){
        outputFileName.append("_");
        outputFileName.append(datetime());
        outputFileName.append(".pcd");
    } else {
        // In case it already has termination .pcd delete it before
        std::string outputFileName = outputFileName.substr(0, outputFileName.length()-4);
        outputFileName.append("_");
        outputFileName.append(datetime());
        outputFileName.append(".pcd");
    }
    // Add the last slash in case the directory does not include one as the name 
    // will be appended to the directory
    if (not hasEnding(outputDirectory, "/")){
        outputDirectory.append("/");
    }
    outputDirectory.append(outputFileName);

    // Save file
    ROS_INFO_STREAM("Saving file to: " << outputDirectory);
    pcl::io::savePCDFileASCII (outputDirectory, *cloud);

    // Terminate ROS so the terminal is left free
    ros::shutdown();
}


int main (int argc, char** argv)
{
    ros::init( argc, argv, "topicCloudRecorder" );

    // Create the class to go through the process
    RecordClass recorder;

    ros::spin();

    return 0;
}