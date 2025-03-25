#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LaserScanOverride
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher scan_pub_;

public:
    LaserScanOverride()
    {
        // Publisher for modified laser scan
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_ovr", 10);
        
        ROS_INFO("LaserScan override node initialized");
        ROS_INFO("Waiting for scan topic to be published...");
        
        // Wait for the scan topic to be published
        ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan", nh_);
        
        ROS_INFO("Scan topic detected! Subscribing now.");
        
        // Subscribe to original laser scan topic after confirmed it exists
        scan_sub_ = nh_.subscribe("scan", 10, &LaserScanOverride::scanCallback, this);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
        // Create a copy of the incoming message
        sensor_msgs::LaserScan modified_scan = *scan_msg;
        
        // Override the range_min value to 0.1 because the rplidar C1 have a minimum range of 0.1m, but rplidar_ros publishes 0.15m
        modified_scan.range_min = 0.1;
        
        // Publish the modified message
        scan_pub_.publish(modified_scan);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserscan_override_node");
    LaserScanOverride laser_override;
    ros::spin();
    return 0;
}
