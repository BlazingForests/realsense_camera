
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
#include <limits.h>

#define CLEAR(x) memset (&(x), 0, sizeof (x))


//one video frame in memory
typedef struct video_frame_buffer {
    void *                  data;
    size_t                  length;
}FrameBuffer, * PFrameBuffer;


typedef struct video_stream
{
    char            videoName[32];
    int             width;
    int             height;
    unsigned int    pixelFormat;
    int             fd;
    FrameBuffer     frameBuffer;
}VideoStream, * PVideoStream;



#ifdef __cplusplus
extern "C" {
#endif


int capturer_mmap_init(PVideoStream p_video_stream);

int capturer_mmap_get_frame(PVideoStream p_video_stream);

void capturer_mmap_exit(PVideoStream p_video_stream);


#ifdef __cplusplus
}
#endif

