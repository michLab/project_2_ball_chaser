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
	// Iterate through image to find a white ball:
	int white_pixel = 255;			// A white pixel has three bytes: RGB, each should equals 255
	bool white_blob_found = false;		// Initially assume, that we didn't find a white blob (white ball)
	bool white_pixel_found = false;		// Initially assume, that we didn't find a white pixel
	int white_blob_row_min = img.height;	// A white blob minimum and maximum row and column
	int white_blob_row_max = 0;
	int white_blob_col_min = img.width;
	int white_blob_col_max = 0;

	// Go through image rows, columns and bytes in RGB data:
   	for (int row_num = 0; row_num < img.height; ++row_num){
		for (int col_num = 0; col_num < img.width; ++col_num){
			// For RGB triade check if it describes a white pixel (each byte should be 255):
			white_pixel_found = true; 	// Firstly assume, that this is a white pixel and iterate through RGB bytes:
			for (int rgb_byte_num = 0; rgb_byte_num < 3; ++rgb_byte_num){
				if (img.data[row_num * img.step + col_num * 3 + rgb_byte_num] != white_pixel) {
					white_pixel_found = false;	// if any of the pixel in RGB triade isn't 'white' then change white_pixel_found flag to false
				}
			}
			// If white pixel was found then check, if the row and column associated with this pixel are extremums (most left, right, top, bottom) to determine 
			// white blob square-shaped boundaries on the image:
			if ( white_pixel_found == true){
				white_blob_found = true;
				if (row_num < white_blob_row_min){
					white_blob_row_min = row_num;				
				}
				else if (row_num > white_blob_row_max){
					white_blob_row_max = row_num;
				}

				if (col_num < white_blob_col_min){
					white_blob_col_min = col_num;				
				}
				else if (col_num > white_blob_col_max){
					white_blob_col_max = col_num;
				}
			}		
		}	
	}
	
	// If robot found white ball than determine if it is in the left, right or center of the image:
	float forward_vel = 0.1;				// Robot forward speed
	float angular_vel = 0.2;				// Robot angular speed
	int left_region_end = img.width/3;			// A column at which left region of the image ends
	int right_region_start = img.width - img.width/3; 	// A column at which right region of the image starts

	// If a white_blob was found:	
	if (white_blob_found == true){
		// Find a center of white blob:
		int white_blob_row_center = 0.5 * (white_blob_row_max + white_blob_row_min);	// Center row is an arithmetic mean of min and max rows of blob
		int white_blob_col_center = 0.5 * (white_blob_col_max + white_blob_col_min);	// Center col is an arithmetic mean of min and max cols of blob
		ROS_INFO("Found white blob: row = %d, col = %d", white_blob_row_center, white_blob_col_center);
		//ROS_INFO("white_blob_row_min = %d, white_blob_row_max = %d", white_blob_row_min, white_blob_row_max);

		// If white blob fills image from top to bottom then stop the motors:
		if (white_blob_row_min == 0 and white_blob_row_max == img.height-1){
			drive_robot(0, 0);
		}
		// else if blob center is in the left region of image:
		else if (white_blob_col_center < left_region_end){
			// then move counter-clockwise:	
			drive_robot(0, angular_vel);	
		}
		// else if blob center is in the moddle region of image:
		else if (white_blob_col_center < right_region_start){
			// then move forward
			drive_robot(forward_vel, 0);		
		}
		// else the blob center is in the right region of image:
		else {
			// so move clockwise:
			drive_robot(0, -1 * angular_vel);
		}
	} 
	else {
		ROS_INFO("Didn't found white blob");
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
