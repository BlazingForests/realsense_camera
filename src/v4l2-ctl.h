#ifndef _V4L2_CTL_H
#define _V4L2_CTL_H


#include <string>
#include <vector>

#include <linux/videodev2.h>


void list_devices(std::string &target_name, std::vector<std::string> &device_lists);

#endif
