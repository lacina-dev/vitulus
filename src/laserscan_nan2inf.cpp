#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath> // For std::isnan and std::numeric_limits

ros::Publisher modified_scan_pub;
ros::Publisher scan_5m_pub; // New publisher for /rplidar/scan_5m

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& original_scan)
{
    // First modification: NaN to 16.0f for /scan topic
    sensor_msgs::LaserScan modified_scan_for_scan_topic = *original_scan;
    for (size_t i = 0; i < modified_scan_for_scan_topic.ranges.size(); ++i)
    {
        if (std::isnan(modified_scan_for_scan_topic.ranges[i]))
        {
            modified_scan_for_scan_topic.ranges[i] = std::numeric_limits<float>::infinity();
            // modified_scan_for_scan_topic.ranges[i] = 16.0f; 
        }
    }
    modified_scan_pub.publish(modified_scan_for_scan_topic);

    // Second modification: NaN to 5.0f for /rplidar/scan_5m topic
    sensor_msgs::LaserScan modified_scan_for_5m_topic = *original_scan;
    for (size_t i = 0; i < modified_scan_for_5m_topic.ranges.size(); ++i)
    {
        // if (modified_scan_for_5m_topic.ranges[i] == std::numeric_limits<float>::infinity())
        if (std::isnan(modified_scan_for_5m_topic.ranges[i]))
        {
            // modified_scan_for_5m_topic.ranges[i] = std::numeric_limits<float>::infinity();
            modified_scan_for_5m_topic.ranges[i] = 5.0f; 
        }
        
        
    }
    // Set all intensities to 50.0f for the /rplidar/scan_5m topic
    for (size_t i = 0; i < modified_scan_for_5m_topic.intensities.size(); ++i)
    {
        modified_scan_for_5m_topic.intensities[i] = 50.0f; // Set all intensities to 50.0f
    }
    scan_5m_pub.publish(modified_scan_for_5m_topic);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserscan_nan_to_inf_node");
    ros::NodeHandle nh;

    modified_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 10);
    scan_5m_pub = nh.advertise<sensor_msgs::LaserScan>("/rplidar/scan_5m", 10); // Initialize the new publisher
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/rplidar/scan_filtered", 10, laserScanCallback);

    ROS_INFO("LaserScan NaN to Inf/5m node started. Subscribing to /rplidar/scan_filtered, publishing to /scan and /rplidar/scan_5m");

    ros::spin();

    return 0;
}
