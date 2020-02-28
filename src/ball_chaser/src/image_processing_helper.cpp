#include "image_processing_helper.h"
// Helper functions for robot camera image processing

// This function identifies white pixels given different lighting
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

// This function identifies the location of white objects as either on the left, centre or right (or none)
std::vector<int> white_object_location(const sensor_msgs::Image img) {
    // White pixels detected mainly to left, centre or right of image to determine location
    // Limits for left, right (and therefore centre) of image columns
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
    // Location of white ball shown as 1 (there) in vector [left, centre, right] and 0 (not there)
    std::vector<int> white_object_location = { 0, 0, 0 };
    int n_white_pixels = std::accumulate(white_pixel_count.begin(), white_pixel_count.end(), 0);
    // Check that white pixels are found (white ball detected) and not too many are found (too close to ball)
    if (n_white_pixels != 0 && n_white_pixels < n_white_pixels_close) {
        // Look for position of white ball
        int index_max = std::max_element(white_pixel_count.begin(), white_pixel_count.end()) - white_pixel_count.begin();
        white_object_location[index_max] = 1;

    }
    return white_object_location;
}
