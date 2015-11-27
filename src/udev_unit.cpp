/*
 * udev_unit.cpp
 *
 *  Created on: Feb 26, 2015
 *      Author: ddz
 */



#include "udev_unit.h"

#include <assert.h>
#include <sys/stat.h>






struct udev_device *find_device(struct udev *udev, const char *id)
{

        assert(udev);
        assert(id);

        if (startswith(id, "/dev/")) {
                struct stat statbuf;
                char type;

                if (stat(id, &statbuf) < 0)
                        return NULL;

                if (S_ISBLK(statbuf.st_mode))
                        type = 'b';
                else if (S_ISCHR(statbuf.st_mode))
                        type = 'c';
                else
                        return NULL;

                return udev_device_new_from_devnum(udev, type, statbuf.st_rdev);
        } else if (startswith(id, "/sys/"))
                return udev_device_new_from_syspath(udev, id);
        else
                return NULL;
}


//static void print_record(struct udev_device *device) {
//        const char *str;
//        int i;
//        struct udev_list_entry *list_entry;
//
//        printf("P: %s\n", udev_device_get_devpath(device));
//
//        str = udev_device_get_devnode(device);
//        if (str != NULL)
//                printf("N: %s\n", str + strlen("/dev/"));
//
//        i = udev_device_get_devlink_priority(device);
//        if (i != 0)
//                printf("L: %i\n", i);
//
//        udev_list_entry_foreach(list_entry, udev_device_get_devlinks_list_entry(device))
//                printf("S: %s\n", udev_list_entry_get_name(list_entry) + strlen("/dev/"));
//
//        udev_list_entry_foreach(list_entry, udev_device_get_properties_list_entry(device))
//                printf("E: %s=%s\n",
//                       udev_list_entry_get_name(list_entry),
//                       udev_list_entry_get_value(list_entry));
//        printf("\n");
//}



std::string getDeviceSerialNumber(const char *device)
{
	std::string output = "";

	struct udev *udev_obj;
	struct udev_device *udev_dev;

	udev_obj = udev_new();
	if(!udev_obj)
	{
		//printf("Can't create udev\n");
		return "";
	}

	udev_dev = find_device(udev_obj, device);

	if(!udev_dev)
	{
		//printf("Can't find udev device = %s\n", device);
		return "";
	}

	struct udev_list_entry *list_entry;

	udev_list_entry_foreach(list_entry, udev_device_get_properties_list_entry(udev_dev))
	{
		if(strncmp(udev_list_entry_get_name(list_entry), "ID_SERIAL_SHORT", strlen("ID_SERIAL_SHORT")) == 0)
		{
			output = udev_list_entry_get_value(list_entry);
			break;
		}
	}

	udev_device_unref(udev_dev);
	udev_unref(udev_obj);

	return output;
}
