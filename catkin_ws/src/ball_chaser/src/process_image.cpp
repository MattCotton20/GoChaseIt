#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request centered joint angles [1.57, 1.57]
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service /ball_chaser/command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    // Tuneable settings
    const float turn_speed = 0.1;
    const float drive_speed = 0.1;
    const int left_threshold = img.width / 3;
    const int right_threshold = img.width / 3 * 2;

    // Other variable declarations
    const int pixel_count = img.height * img.width;
    const int white_pixel = 255;
    bool ball_found = false;
    int ball_pos;

    // Loop through every pixel in the image
    for (int i = 0; i < pixel_count; ++i) {
        if (img.data[3*i]   == white_pixel and
            img.data[3*i+1] == white_pixel and
            img.data[3*i+2] == white_pixel) {

            ball_found = true; // If found, log it and stop searching
	    ball_pos = i % img.width;
	    break;
        }
    }

    if (!ball_found) // ball not in image - stop robot
        drive_robot(0, 0);
    else {
        if (ball_pos < left_threshold) // ball in left sector - turn left
	    drive_robot(0, turn_speed);
        else if (ball_pos < right_threshold) // ball in middle sector - drive forward
	    drive_robot(drive_speed, 0);
	else
	    drive_robot(0, -turn_speed); // ball in right sector - turn right
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
