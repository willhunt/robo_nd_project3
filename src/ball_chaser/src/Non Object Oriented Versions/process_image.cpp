#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <vector>
#include <numeric>
#include <algorithm>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Driving robot");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_request");
}

// identifies white pixels given different lighting
bool white_pixel_identifier(int r, int g, int b) {
    // R, G & B channels must have the same value
    if (r != g || r != b) {
        return false;
    }
    // Check pixels are bright enough to identify a white object
    if (r < 220) {
        return false;
    } else {
        return true;
    }
}


// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;  // Not used anymore

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    // Limits for left, right (and therefore centre)
    int left_lim = img.width / 3;
    int right_lim = 2 * img.width / 3;
    // Store count of left, centre & right located white pixels
    std::vector<int> white_pixel_count = { 0, 0, 0 };
    // Store upper limit of white pixel count to stop moving at (too close) to avoid recalculating
    int n_white_pixels_close = 0.4 * 3 * img.width * img.height;

    // Loopthrough image pixels and record location of white pixels
    for (int row_i = 0; row_i < img.height; row_i++) {
        for (int col_i = 0; col_i < img.width; col_i++) {
            // Sum three RGB elements, will equal 3 * 255 if white
            int r_pixel_index = (col_i * 3) + (row_i * img.width * 3);;
            // ROS_INFO("Pixel brightness: %d / %d", pixel_brightness, white_brightness);
            if (white_pixel_identifier(img.data[r_pixel_index], img.data[r_pixel_index + 1], img.data[r_pixel_index + 2])) {
                if (col_i < left_lim) {
                    white_pixel_count[0] = white_pixel_count[0] + 1;
                } else if (col_i > right_lim) {
                    white_pixel_count[2] = white_pixel_count[2] + 1;
                } else {
                    white_pixel_count[1] = white_pixel_count[1] + 1;
                }      
            }
        }
    }
    // ROS_INFO("White pixel distribution: [%d, %d, %d]", white_pixel_count[0], white_pixel_count[1], white_pixel_count[2]);
    
    int n_white_pixels = std::accumulate(white_pixel_count.begin(), white_pixel_count.end(), 0);
    // Check if no white pixels found (no white ball) or too many are found (too close to ball)
    if (n_white_pixels == 0 || n_white_pixels > n_white_pixels_close) {
        drive_robot(0, 0);
    } else {
        // Look for position of white ball
        int maxElementIndex = std::max_element(white_pixel_count.begin(), white_pixel_count.end()) - white_pixel_count.begin();
        if (maxElementIndex == 0) {
            drive_robot(0, 0.4);
        } else if (maxElementIndex == 2) {
            drive_robot(0, -0.4);
        } else {
            drive_robot(0.2, 0);
        }
    }   
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}