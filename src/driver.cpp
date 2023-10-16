#include <ros/ros.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <random>

ros::Publisher* p_pub;   
double wall_dist;
sensor_msgs::LaserScan* lidar_out;
geometry_msgs::Twist desired_velocity;
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<int> dist(1, 10);
int random_num=0;
void desvelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    p_pub->publish(desired_velocity);
}
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg2)
{
    ROS_INFO("Laser?: [%d]", msg2->header.seq);
    ROS_INFO("desired_velocity_linear: [%f]", desired_velocity.linear.x);
    random_num=dist(gen);
    ROS_INFO("randomnum: [%d]", random_num);
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
    {	
        if(random_num<=6)
        {
            desired_velocity.linear.x = 0.0;
            desired_velocity.angular.z = 2;
        }
        else
        {
            desired_velocity.linear.x = -0.5;
            desired_velocity.angular.z = 1.0;
        }
    }
    else {
    	desired_velocity.linear.x = 0.5;
        desired_velocity.angular.z = 0.0;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "driver_node");
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    p_pub = &cmd_vel_pub;
    ros::Subscriber des_vel_sub = nh.subscribe<geometry_msgs::Twist>("des_vel", 10,desvelCallback);
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("laser_0", 10, lidarCallback);
    ros::param::param("wall_dist", wall_dist, 0.1);
    ros::Rate loop_rate(10);
    ROS_INFO("wall_dist began with: [%2.2f]", wall_dist);
    std::string param_name = "/robot0/wall_dist";

    while (ros::ok())
    {
        if (nh.getParamCached(param_name, wall_dist)) 
	        ROS_INFO("wall_dist was updated to: [%2.2f]", wall_dist);
        else 
            ROS_ERROR("Failed to get param 'wall_dist.'");
	    ROS_INFO_ONCE("wall_dist is now: [%2.2f]", wall_dist);
        ros::spinOnce();
        ros::spin();
        loop_rate.sleep();
    }
    return 0;
}
