#ifndef IMAGE_PROCESSING_HELPER_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define IMAGE_PROCESSING_HELPER_H

#include <vector>
#include <numeric>
#include <algorithm>
#include <sensor_msgs/Image.h>

// This function identifies white pixels given different lighting
bool white_pixel_identifier(int r, int g, int b);

// This function identifies the location of white objects as either on the left, centre or right (or none)
std::vector<int> white_object_location(const sensor_msgs::Image img);

#endif  // IMAGE_PROCESSING_HELPER_H