
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <linux/videodev2.h>
#include <libv4l1.h>
#include <libv4l2.h>
#include <limits.h>

#include <string>

#define RESULT_SUCCESS 0
#define RESULT_FAILURE 1

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define BUFFER_COUNT 1


// v4l2 control ids
// dependent on https://github.com/teknotus/depthview/tree/udev
#define REALSENSE_LASER_CTRL V4L2_CID_PRIVATE_BASE
#define REALSENSE_PATTERN_CTRL (V4L2_CID_PRIVATE_BASE + 1)
#define REALSENSE_EXPOSURE_CTRL (V4L2_CID_PRIVATE_BASE + 2)
#define REALSENSE_FILTER_CTRL (V4L2_CID_PRIVATE_BASE + 3)
#define REALSENSE_CONFIDENCE_CTRL (V4L2_CID_PRIVATE_BASE + 4)

//one video frame in memory
typedef struct video_frame_buffer {
    size_t      length;
    void*       mmapbuf;
}FrameBuffer, * PFrameBuffer;


typedef struct video_stream
{
    char            videoName[32];
    int             width;
    int             height;
    unsigned int    pixelFormat;
    int             fd;
    FrameBuffer     frameBuffer[BUFFER_COUNT];
    void*           fillbuf;
    int             buflen;
}VideoStream, * PVideoStream;



int capturer_mmap_init(PVideoStream p_video_stream);

int capturer_mmap_get_frame(PVideoStream p_video_stream);

void capturer_mmap_exit(PVideoStream p_video_stream);

void capturer_mmap_init_v4l2_controls();

int capturer_mmap_set_control(PVideoStream p_video_stream, const std::string &control, int value);
