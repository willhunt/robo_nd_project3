#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//TODO: Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"

class RobotDrive {
    private:
    // ROS::Publisher motor commands;
    ros::Publisher motor_command_publisher_;
    ros::ServiceServer command_robot_service_;

    public:
    RobotDrive(ros::NodeHandle *n) {
        // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
        motor_command_publisher_ = n->advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Service server to respond to /ball_chaser/command_robot service (from client) with a handle_drive_request callback function
        command_robot_service_ = n->advertiseService("/ball_chaser/command_robot", &RobotDrive::handle_drive_request, this);
        ROS_INFO("Ready to send drive commands");

    }

    // Callback function that executes whenever a drive_bot service is requested
    // This function publishes the requested linear x and angular velocities to the robot wheel joints
    // After publishing the requested velocities, a message feedback is returned with the requested wheel velocities
    bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res) {

        ROS_INFO("DriveToTarget received - linear_x:%1.2f, angular_z:%1.2f", (float)req.linear_x, (float)req.angular_z);

        // Create a motor_command object of type geometry_msgs::Twist
        geometry_msgs::Twist motor_command;
        // Set wheel velocities
        motor_command.linear.x = req.linear_x;
        motor_command.angular.z = req.angular_z;
        // Publish angles to drive the robot
        motor_command_publisher_.publish(motor_command);

        // Wait for robot to move
        ros::Duration(0.5).sleep();

        // Return a response message
        res.msg_feedback = "Velocity set - linear_x: " + std::to_string(req.linear_x) + " , angular:z: " + std::to_string(req.angular_z);
        ROS_INFO_STREAM(res.msg_feedback);

        return true;
    }
};

int main(int argc, char** argv) {
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");
    // Create a ROS NodeHandle object
    ros::NodeHandle n;
    // Instantiate RobotDrive object
    RobotDrive robodrive = RobotDrive(&n);

    ros::spin();

    return 0;
}