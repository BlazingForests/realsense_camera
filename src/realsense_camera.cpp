
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <map>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <ros/package.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


#include "capturer_mmap.h"

#include "v4l2-ctl.h"



/*
The unit of depth values in micrometer if PIXEL_FORMAT_DEPTH_RAW
31.250000

The depth-sensor horizontal and vertical field of view parameters, in degrees.
x, y = 72.000000, 55.000000

The depth-sensor, sensing distance parameters, in millimeters.
min, max = 0.000000, 2047.000000

The depth-sensor focal length in pixels. The parameters vary with the resolution setting.
x, y = 463.888885, 463.888885

The depth-sensor focal length in mm.
1.670000

The depth-sensor principal point in pixels. The parameters vary with the resolution setting.
x, y = 320.000000, 240.000000
  */



#define SHOW_RGBD_FRAME 1


using namespace cv;
using namespace std;

typedef pcl::PointXYZ       PointXYZT;
typedef pcl::PointXYZRGB    PointXYZRGBT;
typedef pcl::PointXYZRGBA   PointXYZRGBAT;

typedef PointXYZRGBT PointType;

bool show_use_times = false;

struct timeval start, all_start;
struct timeval end, all_end;
double timeuse, all_timeuse;

//x: start timeval
#define USE_TIMES_START( x ) if(show_use_times){gettimeofday( &x, NULL );}

//x: start timeval
//y: end timeval
//z: show string
#define USE_TIMES_END_SHOW( x, y, z ) \
        if(show_use_times) \
        { \
            gettimeofday( &y, NULL ); \
            timeuse = 1000000 * ( y.tv_sec - x.tv_sec ) + y.tv_usec - x.tv_usec; \
            timeuse /= 1000000; \
            printf(z": [%f s]\n", timeuse); \
        }





VideoStream     rgb_stream;
VideoStream     depth_stream;


unsigned char *rgb_frame_buffer = NULL;
//unsigned short *depth_value_buffer = NULL;

#if SHOW_RGBD_FRAME
unsigned char *depth_frame_buffer = NULL;
#endif


int sensor_depth_max = 1200;
float depth_scale = 0.001;

float depth_fxinv = 1.0f / 463.888885f;
float depth_fyinv = 1.0f / 463.888885f;

float depth_cx = 320.0f;
float depth_cy = 240.0f;

ros::Publisher realsense_points_pub;
ros::Publisher realsense_reg_points_pub;

int depth_min = 99999;
int depth_max = -1;


int depth_enable_min = 0;
int depth_enable_max = 2047;

typedef struct
{
    int depthValue;
    float *uvmap;
}DepthToRGBUVMap;

std::map<int, DepthToRGBUVMap> depthToRGBUVMapALL;




void
initDepthToRGBUVMap()
{
    std::string packagePath = ros::package::getPath("realsense_camera");
    std::string uvmapPath = packagePath + "/data/uvmap/";

    for(int i=depth_enable_min; i <= depth_enable_max; ++i)
    {
        char uvmapFileName[64] = {0};
        sprintf(uvmapFileName, "uvmap_%04d.bin", i);
        std::string uvmapFullName = uvmapPath + uvmapFileName;

        std::ifstream uvmapFile;

        uvmapFile.open(uvmapFullName.c_str(), std::ios::binary | std::ios::ate);

        if(uvmapFile.is_open())
        {
            int len = uvmapFile.tellg();
            char *uvmapValue = new char[len];
            uvmapFile.seekg(0, ios::beg);
            uvmapFile.read(uvmapValue, len);
            uvmapFile.close();

            //printf("uvmap file len = %d\n", len);

            DepthToRGBUVMap d_to_rgb_uvmap;
            d_to_rgb_uvmap.depthValue = i;
            d_to_rgb_uvmap.uvmap = (float*)uvmapValue;

            //for(int j=0; j<(len/(sizeof(float)*2)); ++j)
            //{
            //    printf("%04d = %f, %f\n", j, d_to_rgb_uvmap.uvmap[2*j], d_to_rgb_uvmap.uvmap[2*j+1]);
            //}

            depthToRGBUVMapALL[i] = d_to_rgb_uvmap;

            printf("init Depth To RGB UVMap dist = %04d\n", i);
        }
    }

}


int getUVWithDXY(int depth, int xy, float &uvx, float &uvy)
{
    std::map<int, DepthToRGBUVMap>::iterator itr = depthToRGBUVMapALL.find(depth);

    if(itr == depthToRGBUVMapALL.end())
    {
        return 1;
    }

    DepthToRGBUVMap uvmap = itr->second;

    if(uvmap.depthValue != depth)
    {
        return 1;
    }

    uvx = uvmap.uvmap[2*xy];
    uvy = uvmap.uvmap[2*xy+1];

    return 0;
}


void
pubRealSenseTF()
{
    tf::Transform tf;
    tf.setIdentity();
    tf::Quaternion q;
    q.setEuler(0.0f, -1.57f, 0.0f);
    tf.setRotation(q);
    tf.setOrigin(tf::Vector3(0.0f, 0.1f, 0.0f));

    static tf::TransformBroadcaster tf_br;
    tf_br.sendTransform(tf::StampedTransform(tf, ros::Time::now(),
                                             "/realsense_frame", "/realsense_depth_frame"));
}


void
pubRealSensePointsCloudMsg(pcl::PointCloud<PointType>::Ptr &input,
							pcl::PointCloud<pcl::PointXYZ>::Ptr &xyz_input)
{
    input->header.frame_id = "/realsense_depth_frame";
    xyz_input->header.frame_id = "/realsense_depth_frame";

    //pub reg points
    pcl::PCLPointCloud2 pcl_xyzrgb_pc2;
    pcl::toPCLPointCloud2 (*input, pcl_xyzrgb_pc2);

    sensor_msgs::PointCloud2 realsense_xyzrgb_cloud2;
    pcl_conversions::moveFromPCL(pcl_xyzrgb_pc2, realsense_xyzrgb_cloud2);
    realsense_reg_points_pub.publish (realsense_xyzrgb_cloud2);



    //pub points
    //pcl::PointCloud<PointXYZT> msg_xyz_cloud;
    //pcl::copyPointCloud(*input, msg_xyz_cloud);

    pcl::PCLPointCloud2 pcl_xyz_pc2;
    pcl::toPCLPointCloud2 (*xyz_input, pcl_xyz_pc2);

    sensor_msgs::PointCloud2 realsense_xyz_cloud2;
    pcl_conversions::moveFromPCL(pcl_xyz_pc2, realsense_xyz_cloud2);
    realsense_points_pub.publish (realsense_xyz_cloud2);
}


void initVideoStream()
{
    memset(&rgb_stream, 0, sizeof(VideoStream));
    memset(&depth_stream, 0, sizeof(VideoStream));

    strncpy(rgb_stream.videoName, "/dev/video", 10);
    rgb_stream.width = 1920;
    rgb_stream.height = 1080;
    rgb_stream.pixelFormat = V4L2_PIX_FMT_YUYV;
    rgb_stream.fd = -1;

    strncpy(depth_stream.videoName, "/dev/video", 10);
    depth_stream.width = 640;
    depth_stream.height = 480;
    depth_stream.pixelFormat = V4L2_PIX_FMT_RGB565;
    depth_stream.fd = -1;
}

void processRGB()
{
    struct timeval rgb_start, rgb_end;
    USE_TIMES_START( rgb_start );
    capturer_mmap_get_frame(&rgb_stream);
    USE_TIMES_END_SHOW ( rgb_start, rgb_end, "capturer_mmap_get_frame RGB time" );
}

void processDepth()
{
    struct timeval depth_start, depth_end;
    USE_TIMES_START( depth_start );
    capturer_mmap_get_frame(&depth_stream);
    USE_TIMES_END_SHOW ( depth_start, depth_end, "capturer_mmap_get_frame depth time" );
}


// This is probably the most acceptable conversion from camera YUYV to RGB
//
// Wikipedia has a good discussion on the details of various conversions and cites good references:
// http://en.wikipedia.org/wiki/YUV
//
// Also http://www.fourcc.org/yuv.php
//
// What's not clear without knowing more about the camera in question is how often U & V are sampled compared
// to Y.
//
// E.g. YUV444, which is equivalent to RGB, where both require 3 bytes for each pixel
//      YUV422, which we assume here, where there are 2 bytes for each pixel, with two Y samples for one U & V,
//              or as the name implies, 4Y and 2 UV pairs
//      YUV420, where for every 4 Ys, there is a single UV pair, 1.5 bytes for each pixel or 36 bytes for 24 pixels

void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
   int r1, g1, b1;

   // replaces floating point coefficients
   int c = y-16, d = u - 128, e = v - 128;

   // Conversion that avoids floating point
   r1 = (298 * c           + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d           + 128) >> 8;

   // Computed values may need clipping.
   if (r1 > 255) r1 = 255;
   if (g1 > 255) g1 = 255;
   if (b1 > 255) b1 = 255;

   if (r1 < 0) r1 = 0;
   if (g1 < 0) g1 = 0;
   if (b1 < 0) b1 = 0;

//   *r = r1 ;
//   *g = g1 ;
//   *b = b1 ;

   *r = b1 ;
   *g = g1 ;
   *b = r1 ;
}


void
processRGBD()
{

    processRGB();
    processDepth();


#if SHOW_RGBD_FRAME
    Mat rgb_frame(rgb_stream.height, rgb_stream.width, CV_8UC3, rgb_frame_buffer);
    Mat depth_frame(depth_stream.height, depth_stream.width, CV_8UC1, depth_frame_buffer);
#endif



    USE_TIMES_START( start );

    //RGB frame
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)rgb_stream.frameBuffer.data;
    for(int i=0, newi=0; i<rgb_stream.frameBuffer.length; i=i+4, newi=newi+6)
    {
        y_temp=(int)pptr[i]; u_temp=(int)pptr[i+1]; y2_temp=(int)pptr[i+2]; v_temp=(int)pptr[i+3];
        yuv2rgb(y_temp, u_temp, v_temp, &rgb_frame_buffer[newi], &rgb_frame_buffer[newi+1], &rgb_frame_buffer[newi+2]);
        yuv2rgb(y2_temp, u_temp, v_temp, &rgb_frame_buffer[newi+3], &rgb_frame_buffer[newi+4], &rgb_frame_buffer[newi+5]);
    }


    pcl::PointCloud<PointType>::Ptr realsense_cloud (new pcl::PointCloud<PointType>());

    realsense_cloud->width = depth_stream.width;
    realsense_cloud->height = depth_stream.height;
    realsense_cloud->is_dense = false;
    realsense_cloud->points.resize(depth_stream.width * depth_stream.height);


    pcl::PointCloud<pcl::PointXYZ>::Ptr realsense_xyz_cloud (new pcl::PointCloud<pcl::PointXYZ>());

    realsense_xyz_cloud->width = depth_stream.width;
    realsense_xyz_cloud->height = depth_stream.height;
    realsense_xyz_cloud->is_dense = false;
    realsense_xyz_cloud->points.resize(depth_stream.width * depth_stream.height);



    //depth value
    for(int i=0; i<depth_stream.width * depth_stream.height; ++i)
    {
        unsigned short depth_raw = *((unsigned short*)(depth_stream.frameBuffer.data) + i);
        int depth = depth_raw / 31.25f;

        if(depth)
        {
            int new_min = std::min(depth_min, depth);
            int new_max = std::max(depth_max, depth);

            if(new_min != depth_min)
            {
                printf("new depth min = %d, %d\n", depth_raw, new_min);
            }

            if(new_max != depth_max)
            {
                printf("new depth max = %d, %d\n", depth_raw, new_max);
            }

            depth_min = new_min;
            depth_max = new_max;
        }

#if SHOW_RGBD_FRAME
        depth_frame_buffer[i] = depth ? 255 * (sensor_depth_max - depth) / sensor_depth_max : 0;
#endif

        float uvx = -1.0f;
        float uvy = -1.0f;

        if(depth)
        {
            float zz = depth * depth_scale;
            realsense_cloud->points[i].x = ((i % depth_stream.width) - depth_cx) * zz * depth_fxinv;
            realsense_cloud->points[i].y = ((i / depth_stream.width) - depth_cy) * zz * depth_fyinv;
            realsense_cloud->points[i].z = zz;

            realsense_xyz_cloud->points[i].x = realsense_cloud->points[i].x;
            realsense_xyz_cloud->points[i].y = realsense_cloud->points[i].y;
            realsense_xyz_cloud->points[i].z = realsense_cloud->points[i].z;


            if(!getUVWithDXY(depth, i, uvx, uvy))
            {
            	int cx = (int)(uvx * rgb_stream.width + 0.5f);
				int cy = (int)(uvy * rgb_stream.height + 0.5f);
				if (cx >= 0 && cx < rgb_stream.width && cy >= 0 && cy < rgb_stream.height)
				{
					unsigned char *rgb = rgb_frame_buffer + (cx+cy*rgb_stream.width)*3;
					unsigned char r = rgb[2];
					unsigned char g = rgb[1];
					unsigned char b = rgb[0];

					realsense_cloud->points[i].rgba = (0 << 24) | (r << 16) | (g << 8) | b;

				}
				else
				{
					realsense_cloud->points[i].x = nanf("");
					realsense_cloud->points[i].y = nanf("");
					realsense_cloud->points[i].z = nanf("");
				}
            }
            else
            {
            	realsense_cloud->points[i].x = nanf("");
				realsense_cloud->points[i].y = nanf("");
				realsense_cloud->points[i].z = nanf("");
            }

        }
        else
        {
            realsense_cloud->points[i].x = nanf("");
            realsense_cloud->points[i].y = nanf("");
            realsense_cloud->points[i].z = nanf("");

            realsense_xyz_cloud->points[i].x = nanf("");
            realsense_xyz_cloud->points[i].y = nanf("");
            realsense_xyz_cloud->points[i].z = nanf("");

        }

    }




    USE_TIMES_END_SHOW ( start, end, "process result time" );

#if SHOW_RGBD_FRAME
    cv::imshow("RGB frame view", rgb_frame);
    cv::imshow("depth frame view", depth_frame);
#endif

    pubRealSenseTF();
    pubRealSensePointsCloudMsg(realsense_cloud, realsense_xyz_cloud);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "realsense_camera_node");
    ros::NodeHandle n;

    //std::string video_rgb_idx;
    //std::string video_depth_idx;
    //ros::NodeHandle private_node_handle_("~");
    //private_node_handle_.param("video_rgb_idx", video_rgb_idx, std::string("0"));
    //private_node_handle_.param("video_depth_idx", video_depth_idx, std::string("2"));


    initDepthToRGBUVMap();

    //find realsense video device
    std::string target_device_name = "Intel(R) RealSense(TM) 3D Camer";
    std::vector<std::string> video_lists;
    list_devices(target_device_name, video_lists);

    if(video_lists.empty())
    {
        printf("can not find Intel(R) RealSense(TM) 3D Camer video device!!!!!!!!!\n");
        ros::shutdown();
        return 0;
    }

    if(video_lists.size() < 2)
    {
        printf("Intel(R) RealSense(TM) 3D Camer video error!!\n");
        for (std::vector<std::string>::iterator iter = video_lists.begin();
                iter != video_lists.end(); ++iter) {
            printf("    found %s\n", (*iter).c_str());
        }
        ros::shutdown();
        return 0;
    }

    initVideoStream();
    strncpy(rgb_stream.videoName, video_lists[0].c_str(), video_lists[0].length());
    strncpy(depth_stream.videoName, video_lists[1].c_str(), video_lists[1].length());

    printf("video rgb name is %s\n", rgb_stream.videoName);
    printf("video depth name is %s\n", depth_stream.videoName);


    if(capturer_mmap_init(&rgb_stream))
    {
        printf("open %s error!!!!!!!!\n", rgb_stream.videoName);
        ros::shutdown();
        return 0;
    }
    else
    {
        printf("video rgb w,h - %d, %d\n", rgb_stream.width, rgb_stream.height);
    }

    if(capturer_mmap_init(&depth_stream))
    {
        printf("open %s error!!!!!!!!\n", depth_stream.videoName);
        ros::shutdown();
        return 0;
    }
    else
    {
        printf("video depth w,h - %d, %d\n", depth_stream.width, depth_stream.height);
    }


    rgb_frame_buffer = new unsigned char[rgb_stream.width * rgb_stream.height * 3];
    //depth_value_buffer = new unsigned short[depth_stream.width * depth_stream.height];

#if SHOW_RGBD_FRAME
    depth_frame_buffer = new unsigned char[depth_stream.width * depth_stream.height];
#endif


    realsense_points_pub = n.advertise<sensor_msgs::PointCloud2> ("/realsense/depth/points", 1);
    realsense_reg_points_pub = n.advertise<sensor_msgs::PointCloud2>("/realsense/depth_registered/points", 1);

    ros::Rate loop_rate(30);

    while(ros::ok())
    {
        //ros::spinOnce();
        processRGBD();
        cv::waitKey(10);
        loop_rate.sleep();
    }

    capturer_mmap_exit(&rgb_stream);
    capturer_mmap_exit(&depth_stream);

    delete[] rgb_frame_buffer;
    //delete[] depth_value_buffer;
    delete[] depth_frame_buffer;

    return 0;
}
