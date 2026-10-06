#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <cmath>
#include <vector>


struct Point { double x, y, z; };

double current_x = 0.0, current_y = 0.0, current_z = 0.0;
double total_path_length = 0.0;
bool odom_received = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double new_x = msg->pose.pose.position.x;
    double new_y = msg->pose.pose.position.y;
    double new_z = msg->pose.pose.position.z;

    if (odom_received) {
        total_path_length += std::sqrt(std::pow(new_x - current_x, 2) + 
                                       std::pow(new_y - current_y, 2) + 
                                       std::pow(new_z - current_z, 2));
    }

    current_x = new_x;
    current_y = new_y;
    current_z = new_z;
    odom_received = true;
}

double getDistance(Point target) {
    return std::sqrt(std::pow(target.x - current_x, 2) + 
                     std::pow(target.y - current_y, 2) + 
                     std::pow(target.z - current_z, 2));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::vector<Point> route(5);

    nh_private.param("point_a_x", route[0].x, 0.0);
    nh_private.param("point_a_y", route[0].y, 0.0);
    nh_private.param("point_a_z", route[0].z, 1.0);

    nh_private.param("point_b_x", route[1].x, 10.0);
    nh_private.param("point_b_y", route[1].y, 0.0);
    nh_private.param("point_b_z", route[1].z, 1.0);

    nh_private.param("point_c_x", route[2].x, 10.0);
    nh_private.param("point_c_y", route[2].y, 10.0);
    nh_private.param("point_c_z", route[2].z, 1.0);

    nh_private.param("point_d_x", route[3].x, 0.0);
    nh_private.param("point_d_y", route[3].y, 10.0);
    nh_private.param("point_d_z", route[3].z, 1.0);

    route[4] = route[0];

    ros::Subscriber sub_odom = nh.subscribe("/drone_0_visual_slam/odom", 10, odomCallback);
    ros::Publisher pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    std::ofstream csv_file("flight_log.csv");
    csv_file << "Time,X,Y,Z,Target_ID,Distance\n";

    ros::Rate rate(10); 
    
    int current_target_idx = 0;
    int hold_counter = 0;

    ROS_INFO("Waiting for odometry data...");
    while (ros::ok() && !odom_received) {
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time mission_start_time = ros::Time::now();
    ros::Time segment_start_time = ros::Time::now();

    ROS_INFO("Mission started! Flying to Target 0.");

    while (ros::ok() && current_target_idx < route.size()) {
        ros::spinOnce(); 

        double time_from_start = (ros::Time::now() - mission_start_time).toSec();
        Point target = route[current_target_idx];
        double distance = getDistance(target);

        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header.stamp = ros::Time::now();
        goal_msg.header.frame_id = "world";
        goal_msg.pose.position.x = target.x;
        goal_msg.pose.position.y = target.y;
        goal_msg.pose.position.z = target.z;
        pub_goal.publish(goal_msg);

        csv_file << time_from_start << "," << current_x << "," << current_y << "," 
                 << current_z << "," << current_target_idx << "," << distance << "\n";

        if (distance < 0.5) {
            hold_counter++;
            if (hold_counter >= 10) { 
                double segment_time = (ros::Time::now() - segment_start_time).toSec();
                ROS_INFO("Target %d reached! Time on segment: %.2f seconds.", current_target_idx, segment_time);
                
                current_target_idx++;
                hold_counter = 0;
                segment_start_time = ros::Time::now();

                if (current_target_idx < route.size()) {
                    ROS_INFO("Moving to Target %d...", current_target_idx);
                }
            }
        } else {
            hold_counter = 0; 
        }

        rate.sleep(); 
    }

    double total_mission_time = (ros::Time::now() - mission_start_time).toSec();
    csv_file.close();

    ROS_INFO("----------------------------------");
    ROS_INFO("MISSION COMPLETE!");
    ROS_INFO("Total flight time: %.2f seconds", total_mission_time);
    ROS_INFO("Total path length: %.2f meters", total_path_length);
    ROS_INFO("----------------------------------");

    return 0;
}
