#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include "image_processing_helper.h"

class RobotImageProcessor {
    private:
    // Client capable of requesting services from command_robot
    ros::ServiceClient command_robot_client_;
    // Camera subscriber
    ros::Subscriber camera_subscriber_;

    public:
    RobotImageProcessor(ros::NodeHandle *n) {
        // Client makes request to server over /ball_chaser/command_robot topic
        command_robot_client_ = n->serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
        
        camera_subscriber_ = n->subscribe("/camera/rgb/image_raw", 10, 
            &RobotImageProcessor::process_image_callback, this);
    }

    // This function calls the command_robot service to drive the robot in the specified direction
    void drive_robot(float lin_x, float ang_z)
    {
        // TODO: Request a service and pass the velocities to it to drive the robot
        ROS_INFO_STREAM("Driving robot");

        ball_chaser::DriveToTarget srv;
        srv.request.linear_x = lin_x;
        srv.request.angular_z = ang_z;

        // Call the safe_move service and pass the requested joint angles
        if (!command_robot_client_.call(srv))
            ROS_ERROR("Failed to call service drive_request");
    }

    // This callback function continuously executes and reads the image data
    void process_image_callback(const sensor_msgs::Image img)
    {
        std::vector<int> object_location = white_object_location(img);
        
        int ball_present = std::accumulate(object_location.begin(), object_location.end(), 0);
        // Check if no ball found
        if (ball_present == 0) {
            drive_robot(0, 0);  // Stop
        } else {
            // Look for position of white ball
            int index_max = std::max_element(object_location.begin(), object_location.end()) - object_location.begin();
            if (index_max == 0) {
                drive_robot(0, 0.3);  // Turn left
            } else if (index_max == 2) {
                drive_robot(0, -0.3);  // Turn right
            } else {
                drive_robot(0.2, 0);  // Go forwards
            }
        }   
    }
};

int main (int argc, char **argv) {
    // Initialize a ROS node
    ros::init(argc, argv, "process_image");
    // Create a ROS NodeHandle object
    ros::NodeHandle n;
    // Instantiate RobotDrive object
    RobotImageProcessor roboimage = RobotImageProcessor(&n);

    ros::spin();
}