#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
	ball_chaser::DriveToTarget srv;
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;
	// Call the service:
	if (!client.call(srv)){
		ROS_ERROR("Failed to call service command_robot");
	}

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

	// Iterate through image to find w white ball:
	int white_pixel = 255;
	bool white_pixel_found = false;
	int white_pixel_row = 0;
	int white_pixel_col = 0;
   	for (int row_num = 0; row_num < img.height; ++row_num){
		for (int col_num = 0; col_num < img.width; ++col_num){
			if ( img.data[row_num * img.step + col_num] == white_pixel){
				white_pixel_found = true;
				white_pixel_row = row_num;
				white_pixel_col = col_num;			
			}		
		}	
	}
	
	// If robot found white ball than determine, if it is in the left, right or center of the image:
	float forward_vel = 0.1;
	float angular_vel = 0.1;
	int left_region_end = img.width/3;
	int right_region_start = img.width - img.width/3; 
	if (white_pixel_found == true){
		if (white_pixel_col < left_region_end){
			// The ball in on the left side:	
			drive_robot(0, angular_vel);	
		}
		else if (white_pixel_col < right_region_start){
			// The ball in in the mid region:
			drive_robot(forward_vel, 0);		
		}
		else {
			// The ball is in the right region:
			drive_robot(0, -1 * angular_vel);
		}
	} 
	else {
		// The robot doesn't see a white ball, thus stop:
		drive_robot(0, 0);
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
