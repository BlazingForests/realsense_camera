#ifndef _V4L2_CTL_H
#define _V4L2_CTL_H


#include <string>
#include <vector>

#include <linux/videodev2.h>

typedef struct video_device
{
	std::string					card_name;
	std::string					serial_number;
	std::vector<std::string>	video_names;
}VIDEO_DEVICE;

void list_devices(std::string &target_name, std::vector<VIDEO_DEVICE> &device_lists);

#endif
