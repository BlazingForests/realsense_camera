#include <stdio.h>
#include <termios.h>            //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>
#include <signal.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <iostream>

#include "realsense_camera/realsenseConfig.h"




float depth_raw_nuit_default;
float depth_raw_unit;


float config_step;
ros::Publisher config_pub;


void 
showCurConfig()
{
    printf("config:\n"
    		"depth_raw_unit = %f\n"
    		"config_step = %f\n",

			depth_raw_unit,
			config_step
			);
}



void 
defaultConfig()
{
	depth_raw_unit = depth_raw_nuit_default;
	config_step = 0.001f;
}


void 
help()
{
    puts("9 - help");
    puts("-----------------------------------");
    puts("0 - set step 1.0");
    puts("1 - set step 0.1");
    puts("2 - set step 0.01");
    puts("3 - set step 0.001");
    puts("4 - set step 0.0001");
    puts("5 - set step 0.00001");
    puts("6 - set step 0.000001");
    puts("7 - set step 0.0000001");
    puts("8 - set step 0.00000001");
    puts("-----------------------------------");
    puts("c/C - depth_raw_unit -/+ step");
    puts("-----------------------------------");
    
}


void sendConfigMsg()
{
    realsense_camera::realsenseConfig config;

    config.depth_raw_unit = depth_raw_unit;
    
    config_pub.publish (config);
    
    //showCurSetting();
    //std::cout << std::endl << "sendConfigMsg" << std::endl;
}



int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

void keyLoop()
{
    char c = 0;
    bool send = false;
 
 
    // get the console in raw mode                                                              
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file                         
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    help();


    while(1)
    {
        // get the next event from the keyboard  
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        //ROS_INFO("value: 0x%02X", c);

        switch(c)
        {
            case 0x39:
            {
                help();
            }
            break;

            case 0x30:
            {
                config_step = 1.0;
                std::cout << std::endl << "config_step = " << config_step << std::endl;
            }
            break;

            case 0x31:
            {
                config_step = 0.1;
                std::cout << std::endl << "config_step = " << config_step << std::endl;
            }
            break;

            case 0x32:
            {
                config_step = 0.01;
                std::cout << std::endl << "config_step = " << config_step << std::endl;
            }
            break;

            case 0x33:
            {
                config_step = 0.001;
                std::cout << std::endl << "config_step = " << config_step << std::endl;
            }
            break;

            case 0x34:
            {
                config_step = 0.0001;
                std::cout << std::endl << "config_step = " << config_step << std::endl;
            }
            break;

            case 0x35:
            {
                config_step = 0.00001;
                std::cout << std::endl << "config_step = " << config_step << std::endl;
            }
            break;

            case 0x36:
            {
                config_step = 0.000001;
                std::cout << std::endl << "config_step = " << config_step << std::endl;
            }
            break;

            case 0x37:
            {
                config_step = 0.0000001;
                std::cout << std::endl << "config_step = " << config_step << std::endl;
            }
            break;

            case 0x38:
            {
                config_step = 0.00000001;
                std::cout << std::endl << "config_step = " << config_step << std::endl;
            }
            break;
            
            case 0x63:
            {
            	depth_raw_unit -= config_step;
                std::cout << std::endl << "depth_raw_unit = " << depth_raw_unit << std::endl;
                send = true;
            }
            break;
            
            case 0x43:
            {
            	depth_raw_unit += config_step;
                std::cout << std::endl << "depth_raw_unit = " << depth_raw_unit << std::endl;
                send = true;
            }
            break;
            
            case 0x0A:
            {
                showCurConfig();
            }
            break;

        }

        if(send == true)
        {
            sendConfigMsg();
            send = false;
        }
    }

    return;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_camera_config_pub");
    ros::NodeHandle n;


    //param
//    <arg name="depth_unit" default="33.05" />
//
//    <arg name="depth_fx" default="463.888885" />
//    <arg name="depth_fy" default="463.888885" />
//
//    <arg name="depth_cx" default="320.0" />
//    <arg name="depth_cy" default="240.0" />


    ros::NodeHandle private_node_handle_("~");

	double depth_unit_d;
	private_node_handle_.param("depth_unit", depth_unit_d, 31.25);
	depth_raw_nuit_default = depth_unit_d;

//	double depth_fx, depth_fy;
//	private_node_handle_.param("depth_fx", depth_fx, 463.888885);
//	private_node_handle_.param("depth_fy", depth_fy, 463.888885);
//	depth_fxinv = 1.0f / depth_fx;
//	depth_fyinv = 1.0f / depth_fy;
//
//	double depth_cx_d, depth_cy_d;
//	private_node_handle_.param("depth_cx", depth_cx_d, 320.0);
//	private_node_handle_.param("depth_cy", depth_cy_d, 240.0);
//	depth_cx = depth_cx_d;
//	depth_cy = depth_cy_d;




    config_pub = n.advertise<realsense_camera::realsenseConfig>("/realsense_camera_config", 1);

    defaultConfig();

    signal(SIGINT,quit);

    keyLoop();


    return 0;
}
