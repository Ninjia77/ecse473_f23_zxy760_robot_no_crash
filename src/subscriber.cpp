#include <ros/ros.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher* p_pub;   
double wall_dist;
sensor_msgs::LaserScan* lidar_out;
geometry_msgs::Twist desired_velocity;
geometry_msgs::Twist output_velocity;

void desvelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    p_pub->publish(msg);
}
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg2)
{
    ROS_INFO("Laser?: [%d]", msg2->header.seq);

    bool obstacle = false;
    for (float range : msg2->ranges)
    {
        if (range < wall_dist)
        {
            obstacle = true;
            break;
        }
    }
    if (obstacle)
    {	desired_velocity.linear.x = 0.0;
        desired_velocity.angular.z = 1.0;
    }
    else {
    	desired_velocity.linear.x = 0.5;
        desired_velocity.angular.z = 0.0;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "subscriber_node");
    ros::NodeHandle nh;
    ros::Publisher publisher_handle = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    
    p_pub = &publisher_handle;

    ros::Subscriber desvelSubscriber = nh.subscribe<geometry_msgs::Twist>("des_vel", 10,desvelCallback);
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("laser_1", 10, lidarCallback);
    ros::param::param("wall_dist", wall_dist, 0.1);
    ros::Rate loop_rate(10);
    ROS_INFO_ONCE("wall_dist began with: [%2.2f]", wall_dist);
	std::string param_name = "/robot0/wall_dist";
    ros::spin();
    while (ros::ok())
    {
        cmd_vel_pub.publish(desired_velocity);
        if (nh.getParamCached(param_name, wall_dist)) 
	        ROS_INFO("wall_dist was updated to: [%2.2f]", wall_dist);
	    ROS_INFO_ONCE("wall_dist is now: [%2.2f]", wall_dist);
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
