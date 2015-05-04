#include "v4l_unit.h"

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <getopt.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <ctype.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <dirent.h>
#include <math.h>

#include <list>
#include <vector>
#include <map>
#include <algorithm>

#include "udev_unit.h"



int doioctl_name(int fd, unsigned long int request, void *parm, const char *name);
#define doioctl(n, r, p) doioctl_name(n, r, p, #r)


typedef std::vector<std::string> dev_vec;
typedef std::map<std::string, std::string> dev_map;
typedef std::map<std::string, std::vector<std::string> > card_map;




int doioctl_name(int fd, unsigned long int request, void *parm, const char *name)
{
    int retval = ioctl(fd, request, parm);

    if (retval < 0)
        printf("%s: failed: %s\n", name, strerror(errno));

    return retval;
}


bool is_v4l_dev(const char *name)
{
    return !memcmp(name, "video", 5);
}

int calc_node_val(const char *s)
{
	int n = 0;

	s = strrchr(s, '/') + 1;
    if (!memcmp(s, "video", 5)) n = 0;
	n += atol(s + (n >= 0x200 ? 3 : 5));
	return n;
}

bool sort_on_device_name(const std::string &s1, const std::string &s2)
{
	int n1 = calc_node_val(s1.c_str());
	int n2 = calc_node_val(s2.c_str());

	return n1 < n2;
}

void list_devices(std::string &target_name, std::vector<VIDEO_DEVICE> &device_lists)
{
    device_lists.clear();

	DIR *dp;
	struct dirent *ep;
	dev_vec files;
	dev_map links;
	card_map cards;
	struct v4l2_capability vcap;

	dp = opendir("/dev");
	if (dp == NULL) {
		perror ("Couldn't open the directory");
		return;
	}
	while ((ep = readdir(dp)))
		if (is_v4l_dev(ep->d_name))
			files.push_back(std::string("/dev/") + ep->d_name);
	closedir(dp);

	/* Find device nodes which are links to other device nodes */
	for (dev_vec::iterator iter = files.begin();
			iter != files.end(); ) {
		char link[64+1];
		int link_len;
		std::string target;

		link_len = readlink(iter->c_str(), link, 64);
		if (link_len < 0) {	/* Not a link or error */
			iter++;
			continue;
		}
		link[link_len] = '\0';

		/* Only remove from files list if target itself is in list */
		if (link[0] != '/')	/* Relative link */
			target = std::string("/dev/");
		target += link;
		if (find(files.begin(), files.end(), target) == files.end()) {
			iter++;
			continue;
		}

		/* Move the device node from files to links */
		if (links[target].empty())
			links[target] = *iter;
		else
			links[target] += ", " + *iter;
		files.erase(iter);
	}

	std::sort(files.begin(), files.end(), sort_on_device_name);

	for (dev_vec::iterator iter = files.begin();
			iter != files.end(); ++iter)
	{
		//printf("process %s\n", iter->c_str());

		int fd = open(iter->c_str(), O_RDWR);

		if (fd < 0)
			continue;
		doioctl(fd, VIDIOC_QUERYCAP, &vcap);
		close(fd);

//		if(1)
//		{
//			printf("\nVIDIOC_QUERYCAP - %s\n"
//					"	driver = %s\n"
//					"	card = %s\n"
//					"	bus_info = %s\n"
//					"	version = %d\n"
//					"	capabilities = %d\n"
//					"	device_caps = %d\n"
//					"	reserved[3] = [%d, %d, %d]\n\n",
//					iter->c_str(),
//					vcap.driver,
//					vcap.card,
//					vcap.bus_info,
//					vcap.version,
//					vcap.capabilities,
//					vcap.device_caps,
//					vcap.reserved[0], vcap.reserved[1], vcap.reserved[2] );
//		}

        std::string card_name = (const char *)vcap.card;

        if(target_name == card_name)
        {
        	std::string bus_info((const char *)vcap.bus_info);
			cards[bus_info].push_back(*iter);
        }

	}


	for (card_map::iterator iter = cards.begin();
			iter != cards.end(); ++iter) {

		VIDEO_DEVICE device;
		device.card_name = iter->first;

		//get serial number
		if(1)
		{
			device.serial_number = getDeviceSerialNumber((iter->second)[0].c_str());
		}


		for(int i=0; i<(iter->second).size(); ++i)
		{
			device.video_names.push_back((iter->second)[i]);
		}

		device_lists.push_back(device);
	}
}




