/*
 * udev_unit.h
 *
 *  Created on: Feb 26, 2015
 *      Author: ddz
 */

#ifndef REALSENSE_CAMERA_SRC_UDEV_UNIT_H_
#define REALSENSE_CAMERA_SRC_UDEV_UNIT_H_

#include <string>
#include <cstring>
#include <libudev.h>

std::string getDeviceSerialNumber(const char *device);



static inline char *startswith(const char *s, const char *prefix) {
        size_t l;

        l = strlen(prefix);
        if (strncmp(s, prefix, l) == 0)
                return (char*) s + l;

        return NULL;
}

struct udev_device *find_device(struct udev *udev, const char *id);

#endif /* REALSENSE_CAMERA_SRC_UDEV_UNIT_H_ */
