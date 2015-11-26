
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <map>
#include <sys/stat.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <ros/package.h>

#include <opencv2/opencv.hpp>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


#include "capturer_mmap.h"

#include "v4l_unit.h"

#include "realsense_camera/realsenseConfig.h"
#include "realsense_camera/get_rgb_uv.h"

#include <dynamic_reconfigure/server.h>
#include <realsense_camera/RealsenseCameraConfig.h>

#include <libusb-1.0/libusb.h>


#define SHOW_RGBD_FRAME 0

#define USE_BGR24 0


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



//debug info
bool		debug_depth_unit = false;



VideoStream     rgb_stream;
VideoStream     depth_stream;
std::string		useDeviceSerialNum;

unsigned char *rgb_frame_buffer = NULL;
unsigned char *depth_frame_buffer = NULL;
#ifdef V4L2_PIX_FMT_INZI
unsigned char *ir_frame_buffer = NULL;
#endif


const int sensor_depth_max = 1200;

//"Intel(R) RealSense(TM) 3D Camer"   	F200
//"Intel RealSense 3D Camera R200"		R200
std::string realsense_camera_type = "Intel(R) RealSense(TM) 3D Camer";

std::string rgb_frame_id = "_rgb_optical_frame";
std::string depth_frame_id = "_depth_optical_frame";

int rgb_frame_w = 1280;
int rgb_frame_h = 720;

float depth_unit = 31.25f;
float depth_scale = 0.001f;

float depth_fxinv = 1.0f / 463.888885f;
float depth_fyinv = 1.0f / 463.888885f;

float depth_cx = 320.0f;
float depth_cy = 240.0f;

int depth_uv_enable_min = 0;
int depth_uv_enable_max = 2047;

std::string topic_depth_points_id = "/depth/points";
std::string topic_depth_registered_points_id = "/depth_registered/points";

std::string topic_image_rgb_raw_id = "/image/rgb_raw";
std::string topic_image_depth_raw_id = "/image/depth_raw";
std::string topic_image_infrared_raw_id = "/image/ir_raw";


//point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr realsense_xyz_cloud;
pcl::PointCloud<PointType>::Ptr realsense_xyzrgb_cloud;
bool resize_point_cloud = false;

//msgs head
unsigned int head_sequence_id = 0;
ros::Time head_time_stamp;

ros::Publisher realsense_points_pub;
ros::Publisher realsense_reg_points_pub;

image_transport::CameraPublisher realsense_rgb_image_pub;
image_transport::CameraPublisher realsense_depth_image_pub;
#ifdef V4L2_PIX_FMT_INZI
image_transport::CameraPublisher realsense_infrared_image_pub;
#endif

// used to read and publish camera calibration parameters
sensor_msgs::CameraInfoPtr rgb_camera_info;
sensor_msgs::CameraInfoPtr ir_camera_info;

ros::ServiceServer getRGBUVService;



typedef struct
{
    int depthValue;
    float *uvmap;
}DepthToRGBUVMap;

std::map<int, DepthToRGBUVMap> depthToRGBUVMapALL;
bool isHaveD2RGBUVMap = false;





float	center_z = 0.0f;
int		center_z_count = 0;
float	center_offset_pixel = 5.f;


// get temperature
libusb_context *usbContext = NULL;
libusb_device_handle *usbHandle = NULL;
unsigned char realsenseTemperature = 0;

void getRealsenseUSBHandle(libusb_context*& context, libusb_device_handle*& handle, std::string& useSerialNumber)
{
	libusb_device **dev_list = NULL;
	libusb_device_handle *dh = NULL;

	int status = -1;
	status = libusb_init(&context);
	if(status < 0)
	{
		printf("libusb init error\n");
		return;
	}

	ssize_t count = -1;
	count = libusb_get_device_list(context, &dev_list);
	if(count < 0)
	{
		printf("get device list error\n");
	}

	for(int i = 0 ; i < count ; ++i)
	{
		struct libusb_device_descriptor descriptor;
		status = libusb_get_device_descriptor(dev_list[i], &descriptor);
		if(status < 0)
		{
		    printf("error getting descriptor\n");
		    continue;
		}

		if(0x8086 == descriptor.idVendor && 0x0a66 == descriptor.idProduct)
		{
			printf("found realsense camera usb device\n");
			status = libusb_open(dev_list[i], &dh);
			if(status < 0){
				printf("failed to open realsense camera usb device\n");
				continue;
			}
			else
			{
				unsigned char serialNumberStr[256];
				status = libusb_get_string_descriptor_ascii(dh, descriptor.iSerialNumber, serialNumberStr, sizeof(serialNumberStr));
				std::string usbSNName = (const char *)serialNumberStr;
				if(useSerialNumber == usbSNName)
				{
					status = libusb_claim_interface(dh,4);
					if(status < 0)
					{
						printf("could not claim interface\n");
						libusb_close(dh);
						continue;
					}
					else
					{
						handle = dh;
						libusb_free_device_list(dev_list, 1);
						break;
					}
				}
				else
				{
					libusb_close(dh);
					continue;
				}
			}
		}
	}

}


unsigned char getRealsenseTemperature(libusb_device_handle *dh){
  int transferred,status;

  unsigned char data[24] = {
    0x14, 0x00, 0xab, 0xcd, 0x52, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  unsigned char buffer[1024];
  status = libusb_bulk_transfer(dh,1,data,24,&transferred,0);
  if(status < 0){
    fprintf(stderr, "bulk out failed\n");
  }
  status = libusb_bulk_transfer(dh,0x81,buffer,1024,&transferred,0);
  if(status < 0){
    fprintf(stderr, "bulk in failed\n");
  } else {
    return buffer[4];
  }
}


void
initDepthToRGBUVMap()
{
	isHaveD2RGBUVMap = false;

    std::string packagePath = ros::package::getPath("realsense_camera");
    std::string uvmapPath = packagePath + "/data/uvmap/" + useDeviceSerialNum;

    struct stat st;
	if (stat(uvmapPath.c_str(), &st) != 0)
	{
		/* Directory does not exist. EEXIST for race condition */
		printf("Directory %s does not exist!!!!\n", uvmapPath.c_str());
		return;
	}

    printf("\n============ start read UVMap\n");

    for(int i=depth_uv_enable_min; i <= depth_uv_enable_max; ++i)
    {
        char uvmapFileName[64] = {0};
        sprintf(uvmapFileName, "/uvmap_%04d.bin", i);
        std::string uvmapFullName = uvmapPath + uvmapFileName;

        std::ifstream uvmapFile;

        uvmapFile.open(uvmapFullName.c_str(), std::ios::binary | std::ios::ate | std::ios::in);

        if(uvmapFile.is_open())
        {
            int len = uvmapFile.tellg();
            char *uvmapValue = new char[len];
            uvmapFile.seekg(0, std::ios::beg);
            uvmapFile.read(uvmapValue, len);
            uvmapFile.close();

            DepthToRGBUVMap d_to_rgb_uvmap;
            d_to_rgb_uvmap.depthValue = i;
            d_to_rgb_uvmap.uvmap = (float*)uvmapValue;

            depthToRGBUVMapALL[i] = d_to_rgb_uvmap;

            //printf("%04d, ", i);
            printf(".");
        }
    }

    if(depthToRGBUVMapALL.size() == (depth_uv_enable_max - depth_uv_enable_min + 1))
    {
    	isHaveD2RGBUVMap = true;
    }

    printf("\n============ end read UVMap\n");
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
pubRealSensePointsXYZCloudMsg(pcl::PointCloud<pcl::PointXYZ>::Ptr &xyz_input)
{
    pcl::PCLPointCloud2 pcl_xyz_pc2;
    pcl::toPCLPointCloud2 (*xyz_input, pcl_xyz_pc2);

    sensor_msgs::PointCloud2 realsense_xyz_cloud2;
    pcl_conversions::moveFromPCL(pcl_xyz_pc2, realsense_xyz_cloud2);

    realsense_xyz_cloud2.header.seq = head_sequence_id;
    realsense_xyz_cloud2.header.stamp = head_time_stamp;
    realsense_xyz_cloud2.header.frame_id = depth_frame_id;

    realsense_points_pub.publish (realsense_xyz_cloud2);
}


void
pubRealSensePointsXYZRGBCloudMsg(pcl::PointCloud<PointType>::Ptr &xyzrgb_input)
{
    pcl::PCLPointCloud2 pcl_xyzrgb_pc2;
    pcl::toPCLPointCloud2 (*xyzrgb_input, pcl_xyzrgb_pc2);

    sensor_msgs::PointCloud2 realsense_xyzrgb_cloud2;
    pcl_conversions::moveFromPCL(pcl_xyzrgb_pc2, realsense_xyzrgb_cloud2);

    realsense_xyzrgb_cloud2.header.seq = head_sequence_id;
    realsense_xyzrgb_cloud2.header.stamp = head_time_stamp;
    realsense_xyzrgb_cloud2.header.frame_id = depth_frame_id;

    realsense_reg_points_pub.publish (realsense_xyzrgb_cloud2);
}

void
pubRealSenseDepthImageMsg(cv::Mat& depth_mat)
{
	sensor_msgs::ImagePtr depth_img(new sensor_msgs::Image);

	depth_img->header.seq = head_sequence_id;
	depth_img->header.stamp = head_time_stamp;
	depth_img->header.frame_id = depth_frame_id;

	depth_img->width = depth_mat.cols;
	depth_img->height = depth_mat.rows;

	depth_img->encoding = sensor_msgs::image_encodings::MONO8;
	depth_img->is_bigendian = 0;

	int step = sizeof(unsigned char) * depth_img->width;
	int size = step * depth_img->height;
	depth_img->step = step;
	depth_img->data.resize(size);
	memcpy(&depth_img->data[0], depth_mat.data, size);

	ir_camera_info->header.frame_id = depth_frame_id;
	ir_camera_info->header.stamp = head_time_stamp;
	ir_camera_info->header.seq = head_sequence_id;

	realsense_depth_image_pub.publish(depth_img, ir_camera_info);
}


#ifdef V4L2_PIX_FMT_INZI
void
pubRealSenseInfraredImageMsg(cv::Mat& ir_mat)
{
	sensor_msgs::ImagePtr ir_img(new sensor_msgs::Image);;

	ir_img->header.seq = head_sequence_id;
	ir_img->header.stamp = head_time_stamp;
	ir_img->header.frame_id = depth_frame_id;


	ir_img->width = ir_mat.cols;
	ir_img->height = ir_mat.rows;

	ir_img->encoding = sensor_msgs::image_encodings::MONO8;
	ir_img->is_bigendian = 0;

	int step = sizeof(unsigned char) * ir_img->width;
	int size = step * ir_img->height;
	ir_img->step = step;
	ir_img->data.resize(size);
	memcpy(&ir_img->data[0], ir_mat.data, size);

	ir_camera_info->header.frame_id = depth_frame_id;
	ir_camera_info->header.stamp = head_time_stamp;
	ir_camera_info->header.seq = head_sequence_id;

	realsense_infrared_image_pub.publish(ir_img, ir_camera_info);
}
#endif

void
pubRealSenseRGBImageMsg(cv::Mat& rgb_mat)
{
	sensor_msgs::ImagePtr rgb_img(new sensor_msgs::Image);

	rgb_img->header.seq = head_sequence_id;
	rgb_img->header.stamp = head_time_stamp;
	rgb_img->header.frame_id = rgb_frame_id;

	rgb_img->width = rgb_mat.cols;
	rgb_img->height = rgb_mat.rows;

	rgb_img->encoding = sensor_msgs::image_encodings::BGR8;
	rgb_img->is_bigendian = 0;

	int step = sizeof(unsigned char) * 3 * rgb_img->width;
	int size = step * rgb_img->height;
	rgb_img->step = step;
	rgb_img->data.resize(size);
	memcpy(&(rgb_img->data[0]), rgb_mat.data, size);

    rgb_camera_info->header.frame_id = rgb_frame_id;
    rgb_camera_info->header.stamp = head_time_stamp;
    rgb_camera_info->header.seq = head_sequence_id;

	realsense_rgb_image_pub.publish(rgb_img, rgb_camera_info);


	//save rgb img
//	static int count = 0;
//	count++;
//	if(count > 0)
//	{
//	    struct timeval save_time;
//        gettimeofday( &save_time, NULL );
//        char save_name[256];
//        sprintf(save_name, "~/temp/realsense_rgb_%d.jpg", (int)save_time.tv_sec);
//        printf("\nsave realsense rgb img: %s\n", save_name);
//	    cv::imwrite(save_name, rgb_mat);
//	    count = 0;
//	}
}


void initVideoStream()
{
    memset(&rgb_stream, 0, sizeof(VideoStream));
    memset(&depth_stream, 0, sizeof(VideoStream));

    strncpy(rgb_stream.videoName, "/dev/video", 10);
    rgb_stream.width = rgb_frame_w;//640;//1280;//1920;
    rgb_stream.height = rgb_frame_h;//480;//720;//1080;
#if USE_BGR24
    rgb_stream.pixelFormat = V4L2_PIX_FMT_BGR24;
#else
    rgb_stream.pixelFormat = V4L2_PIX_FMT_YUYV;
#endif
    rgb_stream.fd = -1;

    strncpy(depth_stream.videoName, "/dev/video", 10);
    depth_stream.width = 640;
    depth_stream.height = 480;
#ifdef V4L2_PIX_FMT_INZI
    depth_stream.pixelFormat = V4L2_PIX_FMT_INZI;
#else
    depth_stream.pixelFormat = 0;
#endif
    depth_stream.fd = -1;
}

int processRGB()
{
    int stream_state = 0;
    struct timeval rgb_start, rgb_end;
    USE_TIMES_START( rgb_start );
    stream_state = capturer_mmap_get_frame(&rgb_stream);
    USE_TIMES_END_SHOW ( rgb_start, rgb_end, "capturer_mmap_get_frame RGB time" );
    return stream_state;
}

int processDepth()
{
    int stream_state = 0;
    struct timeval depth_start, depth_end;
    USE_TIMES_START( depth_start );
    stream_state = capturer_mmap_get_frame(&depth_stream);
    USE_TIMES_END_SHOW ( depth_start, depth_end, "capturer_mmap_get_frame depth time" );
    return stream_state;
}



//capturer_mmap_get_frame depth time: [0.000108 s]
//capturer_mmap_get_frame RGB time: [0.000268 s]
//get RGBD time: [0.000410 s]
//CV_YUV2BGR_YUYV time: [0.000776 s]
//new cv::Mat object RGBD time: [0.000986 s]
//new point cloud object time: [0.000000 s]
//fill point cloud data time: [0.035777 s]     <----  need optimize
//process result time: [0.037211 s]

void
processRGBD()
{
	USE_TIMES_START( start );

	struct timeval process_start, process_end;

	USE_TIMES_START( process_start );

    int stream_depth_state = 0;
    int stream_rgb_state = 0;

    stream_depth_state = processDepth();
    stream_rgb_state = processRGB();

    USE_TIMES_END_SHOW ( process_start, process_end, "get RGBD time" );

    if(stream_depth_state || stream_rgb_state)
    {
        printf("\nstream state error  depth = %d, rgb = %d\n", stream_depth_state, stream_rgb_state);
        return;
    }

    USE_TIMES_START( process_start );

	cv::Mat depth_frame(depth_stream.height, depth_stream.width, CV_8UC1, depth_frame_buffer);

#ifdef V4L2_PIX_FMT_INZI
	cv::Mat ir_frame(depth_stream.height, depth_stream.width, CV_8UC1, ir_frame_buffer);
#endif

	cv::Mat rgb_frame;
#if USE_BGR24
	rgb_frame = cv::Mat(rgb_stream.height, rgb_stream.width, CV_8UC3, rgb_frame_buffer);
	memcpy(rgb_frame_buffer, rgb_stream.fillbuf, rgb_stream.buflen);
#else
	cv::Mat rgb_frame_yuv(rgb_stream.height, rgb_stream.width, CV_8UC2, rgb_frame_buffer);
	memcpy(rgb_frame_buffer, rgb_stream.fillbuf, rgb_stream.buflen);

	struct timeval cvt_start, cvt_end;
	USE_TIMES_START( cvt_start );
	cv::cvtColor(rgb_frame_yuv,rgb_frame,CV_YUV2BGR_YUYV);
	USE_TIMES_END_SHOW ( cvt_start, cvt_end, "CV_YUV2BGR_YUYV time" );

#endif

	USE_TIMES_END_SHOW ( process_start, process_end, "new cv::Mat object RGBD time" );


	USE_TIMES_START( process_start );

	if(!resize_point_cloud)
	{
		realsense_xyz_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
		realsense_xyz_cloud->width = depth_stream.width;
		realsense_xyz_cloud->height = depth_stream.height;
		realsense_xyz_cloud->is_dense = false;
		realsense_xyz_cloud->points.resize(depth_stream.width * depth_stream.height);

		if(isHaveD2RGBUVMap)
		{
			realsense_xyzrgb_cloud.reset(new pcl::PointCloud<PointType>());
			realsense_xyzrgb_cloud->width = depth_stream.width;
			realsense_xyzrgb_cloud->height = depth_stream.height;
			realsense_xyzrgb_cloud->is_dense = false;
			realsense_xyzrgb_cloud->points.resize(depth_stream.width * depth_stream.height);
		}

		resize_point_cloud = true;
	}

    USE_TIMES_END_SHOW ( process_start, process_end, "new point cloud object time" );


    USE_TIMES_START( process_start );

    //depth value
	//#pragma omp parallel for
    for(int i=0; i<depth_stream.width * depth_stream.height; ++i)
    {
    	float depth = 0;
#ifdef V4L2_PIX_FMT_INZI
			unsigned short* depth_ptr = (unsigned short*)((unsigned char*)(depth_stream.fillbuf) + i*3);
			unsigned char* ir_ptr = (unsigned char*)(depth_stream.fillbuf) + i*3+2;

			unsigned char ir_raw = *ir_ptr;
			ir_frame_buffer[i] = ir_raw;

			unsigned short depth_raw = *depth_ptr;
			depth = (float)depth_raw / depth_unit;
#else
    		unsigned short depth_raw = *((unsigned short*)(depth_stream.fillbuf) + i);
			depth = (float)depth_raw / depth_unit;
#endif

        depth_frame_buffer[i] = depth ? 255 * (sensor_depth_max - depth) / sensor_depth_max : 0;

        float uvx = -1.0f;
        float uvy = -1.0f;

        if(depth>0.000001f)
        {
        	float pixel_x = (i % depth_stream.width) - depth_cx;
        	float pixel_y = (i / depth_stream.width) - depth_cy;
            float zz = depth * depth_scale;
            realsense_xyz_cloud->points[i].x = pixel_x * zz * depth_fxinv;
            realsense_xyz_cloud->points[i].y = pixel_y * zz * depth_fyinv;
            realsense_xyz_cloud->points[i].z = zz;

            if(isHaveD2RGBUVMap)
            {
				realsense_xyzrgb_cloud->points[i].x = realsense_xyz_cloud->points[i].x;
				realsense_xyzrgb_cloud->points[i].y = realsense_xyz_cloud->points[i].y;
				realsense_xyzrgb_cloud->points[i].z = realsense_xyz_cloud->points[i].z;

				if(!getUVWithDXY(depth, i, uvx, uvy))
				{
					int cx = (int)(uvx * rgb_stream.width + 0.5f);
					int cy = (int)(uvy * rgb_stream.height + 0.5f);
					if (cx >= 0 && cx < rgb_stream.width && cy >= 0 && cy < rgb_stream.height)
					{
						unsigned char *rgb = rgb_frame.data + (cx+cy*rgb_stream.width)*3;
						unsigned char r = rgb[2];
						unsigned char g = rgb[1];
						unsigned char b = rgb[0];


						if(debug_depth_unit &&
							pixel_x > -center_offset_pixel &&
							pixel_x < center_offset_pixel &&
							pixel_y > -center_offset_pixel &&
							pixel_y < center_offset_pixel )
						{
							center_z += zz;
							center_z_count++;
							realsense_xyzrgb_cloud->points[i].rgba = (0 << 24) | (0 << 16) | (0 << 8) | 255;
						}
						else
						{
							realsense_xyzrgb_cloud->points[i].rgba = (0 << 24) | (r << 16) | (g << 8) | b;
						}

					}
					else
					{
						realsense_xyzrgb_cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
						realsense_xyzrgb_cloud->points[i].y = std::numeric_limits<float>::quiet_NaN();
						realsense_xyzrgb_cloud->points[i].z = std::numeric_limits<float>::quiet_NaN();
						realsense_xyzrgb_cloud->points[i].rgba = 0;
					}
				}
				else
				{
					realsense_xyzrgb_cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
					realsense_xyzrgb_cloud->points[i].y = std::numeric_limits<float>::quiet_NaN();
					realsense_xyzrgb_cloud->points[i].z = std::numeric_limits<float>::quiet_NaN();
					realsense_xyzrgb_cloud->points[i].rgba = 0;
				}
            }

        }
        else
        {
        	realsense_xyz_cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
			realsense_xyz_cloud->points[i].y = std::numeric_limits<float>::quiet_NaN();
			realsense_xyz_cloud->points[i].z = std::numeric_limits<float>::quiet_NaN();

        	if(isHaveD2RGBUVMap)
        	{
				realsense_xyzrgb_cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
				realsense_xyzrgb_cloud->points[i].y = std::numeric_limits<float>::quiet_NaN();
				realsense_xyzrgb_cloud->points[i].z = std::numeric_limits<float>::quiet_NaN();
				realsense_xyzrgb_cloud->points[i].rgba = 0;
        	}
        }

    }

    USE_TIMES_END_SHOW ( process_start, process_end, "fill point cloud data time" );

    if(debug_depth_unit && center_z_count)
    {
    	if(usbContext && usbHandle)
    	{
    		realsenseTemperature = getRealsenseTemperature(usbHandle);
    	}
    	center_z /= center_z_count;
    	printf("average center z value = %f    temp = %d    depth_unit = %f\n", center_z, realsenseTemperature, depth_unit);
    	center_z_count = 0;
    }


    USE_TIMES_END_SHOW ( start, end, "process result time" );

#if SHOW_RGBD_FRAME
    cv::imshow("depth frame view", depth_frame);

#ifdef V4L2_PIX_FMT_INZI
    cv::imshow("ir frame view", ir_frame);
#endif
    cv::imshow("RGB frame view", rgb_frame);

#endif


    //pub msgs
    head_sequence_id++;
    head_time_stamp = ros::Time::now();

    pubRealSenseRGBImageMsg(rgb_frame);
#ifdef V4L2_PIX_FMT_INZI
    pubRealSenseInfraredImageMsg(ir_frame);
#endif
    pubRealSenseDepthImageMsg(depth_frame);

    pubRealSensePointsXYZCloudMsg(realsense_xyz_cloud);
    if(isHaveD2RGBUVMap)
    {
    	pubRealSensePointsXYZRGBCloudMsg(realsense_xyzrgb_cloud);
    }

}


int getNumRGBSubscribers()
{
    return realsense_reg_points_pub.getNumSubscribers() + realsense_rgb_image_pub.getNumSubscribers();
}
 
int getNumDepthSubscribers()
{
    int n = realsense_points_pub.getNumSubscribers() + realsense_reg_points_pub.getNumSubscribers() + realsense_depth_image_pub.getNumSubscribers();
#ifdef V4L2_PIX_FMT_INZI
    n += realsense_infrared_image_pub.getNumSubscribers();
#endif
    return n;
}

void
realsenseConfigCallback(const realsense_camera::realsenseConfig::ConstPtr &config)
{
	if(debug_depth_unit)
	{
		depth_unit = config->depth_raw_unit;
		printf("depth_unit = %f\n", depth_unit);
	}
}

void
dynamicReconfigCallback(realsense_camera::RealsenseCameraConfig &config, uint32_t level)
{
    if (capturer_mmap_set_control(&depth_stream, "Laser Power", config.laser_power))
    {
        printf("Could not set Laser Power to %i\n", config.laser_power);
    }
    if (capturer_mmap_set_control(&depth_stream, "Accuracy", config.accuracy))
    {
        printf("Could not set Accuracy to %i\n", config.accuracy);
    }
    if (capturer_mmap_set_control(&depth_stream, "Motion Range Trade Off", config.motion_range_trade_off))
    {
        printf("Could not set Motion Range Trade Off to %i\n", config.motion_range_trade_off);
    }
    if (capturer_mmap_set_control(&depth_stream, "Filter Option", config.filter_option))
    {
        printf("Could not set Filter Option to %i\n", config.filter_option);
    }
    if (capturer_mmap_set_control(&depth_stream, "Confidence Threshold", config.confidence_threshold))
    {
        printf("Could not set Confidence Threshold to %i\n", config.confidence_threshold);
    }
}

bool getRGBUV(realsense_camera::get_rgb_uv::Request  &req,
                realsense_camera::get_rgb_uv::Response &res)
{
    float uvx, uvy;

    if(getUVWithDXY(req.x_min_depth, req.x_min_xy, uvx, uvy))
    {
        return false;
    }
    res.x_min_uv = (int)(uvx * rgb_stream.width + 0.5f);

    if(getUVWithDXY(req.y_min_depth, req.y_min_xy, uvx, uvy))
    {
        return false;
    }
    res.y_min_uv = (int)(uvy * rgb_stream.height + 0.5f);

    if(getUVWithDXY(req.x_max_depth, req.x_max_xy, uvx, uvy))
    {
        return false;
    }
    res.x_max_uv = (int)(uvx * rgb_stream.width + 0.5f);

    if(getUVWithDXY(req.y_max_depth, req.y_max_xy, uvx, uvy))
    {
        return false;
    }
    res.y_max_uv = (int)(uvy * rgb_stream.height + 0.5f);

    return true;
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "realsense_camera_node");
    ros::NodeHandle n;
    image_transport::ImageTransport image_transport(n);

    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("realsense_camera_type", realsense_camera_type, std::string("Intel(R) RealSense(TM) 3D Camer"));

    private_node_handle_.param("rgb_frame_id", rgb_frame_id, std::string("_rgb_optical_frame"));
    private_node_handle_.param("depth_frame_id", depth_frame_id, std::string("_depth_optical_frame"));

    private_node_handle_.param("rgb_frame_w", rgb_frame_w, 1280);
    private_node_handle_.param("rgb_frame_h", rgb_frame_h, 720);

    double depth_unit_d, depth_scale_d;
    private_node_handle_.param("depth_unit", depth_unit_d, 31.25);
    private_node_handle_.param("depth_scale", depth_scale_d, 0.001);
    depth_unit = depth_unit_d;
    depth_scale = depth_scale_d;

    double depth_fx, depth_fy;
    private_node_handle_.param("depth_fx", depth_fx, 463.888885);
    private_node_handle_.param("depth_fy", depth_fy, 463.888885);
    depth_fxinv = 1.0f / depth_fx;
    depth_fyinv = 1.0f / depth_fy;

    double depth_cx_d, depth_cy_d;
    private_node_handle_.param("depth_cx", depth_cx_d, 320.0);
    private_node_handle_.param("depth_cy", depth_cy_d, 240.0);
    depth_cx = depth_cx_d;
    depth_cy = depth_cy_d;

    private_node_handle_.param("depth_uv_enable_min", depth_uv_enable_min, 0);
    private_node_handle_.param("depth_uv_enable_max", depth_uv_enable_max, 2047);

    private_node_handle_.param("topic_depth_points_id", topic_depth_points_id, std::string("/depth/points"));
    private_node_handle_.param("topic_depth_registered_points_id", topic_depth_registered_points_id, std::string("/depth_registered/points"));

    private_node_handle_.param("topic_image_rgb_raw_id", topic_image_rgb_raw_id, std::string("/rgb/image_raw"));
    private_node_handle_.param("topic_image_depth_raw_id", topic_image_depth_raw_id, std::string("/depth/image_raw"));

    private_node_handle_.param("topic_image_infrared_raw_id", topic_image_infrared_raw_id, std::string("/ir/image_raw"));

    private_node_handle_.param("debug_depth_unit", debug_depth_unit, false);

    std::string rgb_info_url;
    private_node_handle_.param("rgb_camera_info_url", rgb_info_url, std::string());

    std::string ir_camera_info_url;
    private_node_handle_.param("ir_camera_info_url", ir_camera_info_url, std::string());

    printf("\n\n===================\n"
    		"realsense_camera_type = %s\n"
    		"rgb_frame_id = %s\n"
    		"depth_frame_id = %s\n"
    		"depth_unit = %f\n"
    		"depth_scale = %f\n"
    		"depth_fxinv = %f\n"
    		"depth_fyinv = %f\n"
    		"depth_cx = %f\n"
    		"depth_cy = %f\n"
    		"depth_uv_enable_min = %d\n"
    		"depth_uv_enable_max = %d\n"
    		"topic_depth_points_id = %s\n"
    		"topic_depth_registered_points_id = %s\n"
    		"topic_image_rgb_raw_id = %s\n"
    		"topic_image_depth_raw_id = %s\n"
    		"topic_image_infrared_raw_id = %s\n"
            "debug_depth_unit = %d\n"
            "rgb_camera_info_url = %s\n"
    		"ir_camera_info_url = %s\n"
    		"=======================\n\n",

			realsense_camera_type.c_str(),
			rgb_frame_id.c_str(),
			depth_frame_id.c_str(),
			depth_unit,
			depth_scale,
			depth_fxinv,
			depth_fyinv,
			depth_cx,
			depth_cy,
			depth_uv_enable_min,
			depth_uv_enable_max,
			topic_depth_points_id.c_str(),
			topic_depth_registered_points_id.c_str(),
            topic_image_rgb_raw_id.c_str(),
			topic_image_depth_raw_id.c_str(),
			topic_image_infrared_raw_id.c_str(),
            debug_depth_unit,
            rgb_info_url.c_str(),
			ir_camera_info_url.c_str()

    		);



#ifdef V4L2_PIX_FMT_INZI
    printf("\ndepthWithIRStream - YEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEES\n");
#else
    printf("\ndepthWithIRStream - NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO\n");
    printf("if you want IR stream, please visit\n"
    		"http://solsticlipse.com/2015/03/31/intel-real-sense-3d-on-linux-macos.html\n"
    		"https://github.com/teknotus/depthview/tree/kernelpatchfmt\n");
#endif

    //find realsense video device
    std::vector<VIDEO_DEVICE> video_lists;
    list_devices(realsense_camera_type, video_lists);

    if(video_lists.empty())
    {
        printf("\n\n""can not find Intel(R) RealSense(TM) 3D Camera video device!!!!!!!!! - %s\n\n", realsense_camera_type.c_str());
        ROS_ERROR("can not find Intel(R) RealSense(TM) 3D Camera video device!!!!!!!!! - %s", realsense_camera_type.c_str());
        ros::shutdown();
        return 0;
    }

    if(1)
    {
    	printf("\n===========================================\n");
    	printf("Intel(R) RealSense(TM) 3D Camera lists\n");

    	for(int i=0; i<video_lists.size(); ++i)
    	{
    		printf("\nPCI: %s\n", video_lists[i].card_name.c_str());
    		printf("Serial: %s\n", video_lists[i].serial_number.c_str());
    		for(int j=0; j<video_lists[i].video_names.size(); ++j)
    		{
    			printf("\t%s\n", video_lists[i].video_names[j].c_str());
    		}
    	}
    	printf("===========================================\n\n");
    }

    //return 0;

    if(video_lists[0].video_names.size() < 2)
	{
		printf("Intel(R) RealSense(TM) 3D Camera video device count error!!!!!!!!!!!\n");
		ros::shutdown();
		return 0;
	}
    else
    {
    	useDeviceSerialNum = video_lists[0].serial_number;
    	printf("use camera %s\n", useDeviceSerialNum.c_str());
    }

    initDepthToRGBUVMap();

    initVideoStream();
    strncpy(rgb_stream.videoName, video_lists[0].video_names[0].c_str(), video_lists[0].video_names[0].length());
    strncpy(depth_stream.videoName, video_lists[0].video_names[1].c_str(), video_lists[0].video_names[1].length());

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

    if (!rgb_info_url.empty())
	{
		std::string camera_name_rgb = "realsense_camera_rgb_" + useDeviceSerialNum;
		camera_info_manager::CameraInfoManager rgb_info_manager(n, camera_name_rgb, rgb_info_url);
		if (rgb_info_manager.isCalibrated())
		{
			rgb_camera_info = boost::make_shared<sensor_msgs::CameraInfo>(rgb_info_manager.getCameraInfo());
			if (rgb_camera_info->width != rgb_frame_w || rgb_camera_info->height != rgb_frame_h)
			{
				ROS_WARN("RGB image resolution does not match calibration file");
				rgb_camera_info.reset(new sensor_msgs::CameraInfo());
				rgb_camera_info->width = rgb_frame_w;
				rgb_camera_info->height = rgb_frame_h;
			}
		}
	}
	if (!rgb_camera_info)
	{
		rgb_camera_info.reset(new sensor_msgs::CameraInfo());
		rgb_camera_info->width = rgb_frame_w;
		rgb_camera_info->height = rgb_frame_h;
	}

	if (!ir_camera_info_url.empty())
	{
		std::string camera_name_ir = "realsense_camera_ir_" + useDeviceSerialNum;
		camera_info_manager::CameraInfoManager ir_camera_info_manager(n, camera_name_ir, ir_camera_info_url);
		if (ir_camera_info_manager.isCalibrated())
		{
			ir_camera_info = boost::make_shared<sensor_msgs::CameraInfo>(ir_camera_info_manager.getCameraInfo());
			if (ir_camera_info->width != depth_stream.width || ir_camera_info->height != depth_stream.height)
			{
				ROS_WARN("IR image resolution does not match calibration file");
				ir_camera_info.reset(new sensor_msgs::CameraInfo());
				ir_camera_info->width = depth_stream.width;
				ir_camera_info->height = depth_stream.height;
			}
		}
	}
	if (!ir_camera_info)
	{
		ir_camera_info.reset(new sensor_msgs::CameraInfo());
		ir_camera_info->width = depth_stream.width;
		ir_camera_info->height = depth_stream.height;
	}

	if(debug_depth_unit && realsense_camera_type == "Intel(R) RealSense(TM) 3D Camer")
	{
		getRealsenseUSBHandle(usbContext, usbHandle, useDeviceSerialNum);
		if(usbContext && usbHandle)
		{
			printf("getRealsenseUSBHandle OK!\n");
		}
	}

    printf("RealSense Camera is running!\n");

#if USE_BGR24
    rgb_frame_buffer = new unsigned char[rgb_stream.width * rgb_stream.height * 3];
#else
    rgb_frame_buffer = new unsigned char[rgb_stream.width * rgb_stream.height * 2];
#endif
    depth_frame_buffer = new unsigned char[depth_stream.width * depth_stream.height];

#ifdef V4L2_PIX_FMT_INZI
    ir_frame_buffer = new unsigned char[depth_stream.width * depth_stream.height];
#endif

    realsense_points_pub = n.advertise<sensor_msgs::PointCloud2> (topic_depth_points_id, 1);
    realsense_reg_points_pub = n.advertise<sensor_msgs::PointCloud2>(topic_depth_registered_points_id, 1);

    realsense_rgb_image_pub = image_transport.advertiseCamera(topic_image_rgb_raw_id, 1);
    realsense_depth_image_pub = image_transport.advertiseCamera(topic_image_depth_raw_id, 1);

#ifdef V4L2_PIX_FMT_INZI
    realsense_infrared_image_pub = image_transport.advertiseCamera(topic_image_infrared_raw_id, 1);
#endif

    ros::Subscriber config_sub = n.subscribe("/realsense_camera_config", 1, realsenseConfigCallback);


    getRGBUVService = n.advertiseService("get_rgb_uv", getRGBUV);

    capturer_mmap_init_v4l2_controls();
    dynamic_reconfigure::Server<realsense_camera::RealsenseCameraConfig> dynamic_reconfigure_server;
    dynamic_reconfigure_server.setCallback(boost::bind(&dynamicReconfigCallback, _1, _2));

    ros::Rate loop_rate(60);
    ros::Rate idle_rate(1);

    while(ros::ok())
    {
        while ((getNumRGBSubscribers() + getNumDepthSubscribers()) == 0 && ros::ok())
        {
            ros::spinOnce();
            idle_rate.sleep();
        }
        processRGBD();

#if SHOW_RGBD_FRAME
        cv::waitKey(10);
#endif

        ros::spinOnce();

        loop_rate.sleep();
    }

    capturer_mmap_exit(&rgb_stream);
    capturer_mmap_exit(&depth_stream);

    delete[] rgb_frame_buffer;
    delete[] depth_frame_buffer;
#ifdef V4L2_PIX_FMT_INZI
    delete[] ir_frame_buffer;
#endif

    if(debug_depth_unit)
    {
    	libusb_close(usbHandle);
    	libusb_exit(usbContext);
    }

    printf("RealSense Camera is shutdown!\n");

    return 0;
}
