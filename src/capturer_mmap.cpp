
#include "capturer_mmap.h"

#include <map>
std::map<std::string, int> v4l2_controls_mapping;

void errno_exit (const char *s)
{
	printf("\n%s error %d, %s\n", s, errno, strerror (errno));
}

//a blocking wrapper of the ioctl function
int xioctl (int fd, int request, void *arg)
{
	int r;

	r = ioctl (fd, request, arg);

	return r;
}

void fcc2s(char *str, unsigned int val)
{

    str[0] = val & 0xff;
    str[1] = (val >> 8) & 0xff;
    str[2] = (val >> 16) & 0xff;
    str[3] = (val >> 24) & 0xff;

}


int open_device (int * fd, char * dev_name)
{

    *fd = open (dev_name, O_RDWR | O_NONBLOCK);

    if (-1 == *fd)
    {
        errno_exit (dev_name);
        return RESULT_FAILURE;
    }

    return RESULT_SUCCESS;
}


//configure and initialize the hardware device
int init_device(int *fd, char *dev_name,
                int *width, int *height, unsigned int *pixel_format )
{
    struct v4l2_format fmt;
    char pixel_str[5] = {0};

    memset(&fmt, 0, sizeof(struct v4l2_format));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    //get
    if (-1 == xioctl (*fd, VIDIOC_G_FMT, &fmt))
    {
        errno_exit ("VIDIOC_G_FMT");
        return RESULT_FAILURE;
    }

    fcc2s(pixel_str, fmt.fmt.pix.pixelformat);
    printf ("\n======================== get\n"
            "%s \n"
            "v4l2_pix_format\n"
            "   width = %d\n"
            "   height = %d\n"
            "   pixelformat_value = 0x%08X\n"
            "   pixelformat = %s\n"
            "   field = %d\n"
            "   bytesperline = %d\n"
            "   sizeimage = %d\n"
            "   colorspace = %d\n"
            "   priv = %d\n\n",
            dev_name,
            fmt.fmt.pix.width,
            fmt.fmt.pix.height,
            fmt.fmt.pix.pixelformat,
            pixel_str,
            fmt.fmt.pix.field,
            fmt.fmt.pix.bytesperline,
            fmt.fmt.pix.sizeimage,
            fmt.fmt.pix.colorspace,
            fmt.fmt.pix.priv );

    //set
    fmt.fmt.pix.width       = *width;
    fmt.fmt.pix.height      = *height;
    fmt.fmt.pix.pixelformat = *pixel_format;

    if (-1 == xioctl (*fd, VIDIOC_S_FMT, &fmt))
    {
        errno_exit ("VIDIOC_S_FMT");
        return RESULT_FAILURE;
    }

    memset(&pixel_str, 0, sizeof(pixel_str));
    fcc2s(pixel_str, fmt.fmt.pix.pixelformat);
    printf ("\n======================== set\n"
            "%s \n"
            "v4l2_pix_format\n"
            "   width = %d\n"
            "   height = %d\n"
            "   pixelformat_value = 0x%08X\n"
            "   pixelformat = %s\n"
            "   field = %d\n"
            "   bytesperline = %d\n"
            "   sizeimage = %d\n"
            "   colorspace = %d\n"
            "   priv = %d\n\n",
            dev_name,
            fmt.fmt.pix.width,
            fmt.fmt.pix.height,
            fmt.fmt.pix.pixelformat,
            pixel_str,
            fmt.fmt.pix.field,
            fmt.fmt.pix.bytesperline,
            fmt.fmt.pix.sizeimage,
            fmt.fmt.pix.colorspace,
            fmt.fmt.pix.priv );

    *width = fmt.fmt.pix.width;
    *height = fmt.fmt.pix.height;
    *pixel_format = fmt.fmt.pix.pixelformat;

    return RESULT_SUCCESS;
}



//alloc buffers and configure the shared memory area and start capturing
int init_mmap (int * fd, char * dev_name, FrameBuffer *framebuffer, void **fillbuf, int *buflen)
{

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(v4l2_requestbuffers));

    req.count               = BUFFER_COUNT;
    req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory              = V4L2_MEMORY_MMAP;

    if (-1 == xioctl (*fd, VIDIOC_REQBUFS, &req))
    {
        errno_exit ("VIDIOC_REQBUFS");
        return RESULT_FAILURE;
    }

    if (req.count < BUFFER_COUNT)
    {
        printf ("\nInsufficient buffer memory on %s\n",dev_name);
        return RESULT_FAILURE;
    }

    struct v4l2_buffer buf[BUFFER_COUNT];
    for(unsigned int i=0; i<BUFFER_COUNT; ++i)
    {
        memset(&buf[i], 0, sizeof(v4l2_buffer));

        buf[i].type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf[i].memory      = V4L2_MEMORY_MMAP;
        buf[i].index       = i;

        if (-1 == xioctl (*fd, VIDIOC_QUERYBUF, &buf[i]))
        {
            errno_exit ("VIDIOC_QUERYBUF");
            return RESULT_FAILURE;
        }
    }

    for(unsigned int i=0; i<BUFFER_COUNT; ++i)
    {
        framebuffer[i].mmapbuf = mmap( NULL,
                                       buf[i].length,
                                       PROT_READ | PROT_WRITE /* required */,
                                       MAP_SHARED /* recommended */,
                                       *fd,
                                       buf[i].m.offset);

        if (MAP_FAILED == framebuffer[i].mmapbuf)
        {
            errno_exit ("mmap");
            return RESULT_FAILURE;
        }

        framebuffer[i].length = buf[i].length;
    }

    *fillbuf = malloc(buf[0].length);
    *buflen = buf[0].length;

    for(unsigned int i=0; i<BUFFER_COUNT; ++i)
    {
        if (-1 == xioctl (*fd, VIDIOC_QBUF, &buf[i]))
        {
            errno_exit ("VIDIOC_QBUF");
            return RESULT_FAILURE;
        }
    }

    return RESULT_SUCCESS;
}


int start_capturing (int *fd)
{
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    //start the capture from the device
    if (-1 == xioctl (*fd, VIDIOC_STREAMON, &type))
    {
        errno_exit ("VIDIOC_STREAMON");
        return RESULT_FAILURE;
    }

    return RESULT_SUCCESS;
}


//read one frame from memory and throws the data to standard output
int read_frame  (int * fd, FrameBuffer *buffer, void **fillbuf)
{
    struct v4l2_buffer buf;//needed for memory mapping
    memset(&buf, 0, sizeof(v4l2_buffer));

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl (*fd, VIDIOC_DQBUF, &buf))
    {
        if(errno == EAGAIN)
        {
            errno_exit ("VIDIOC_DQBUF EAGAIN");
        }
        else if(errno == EINVAL)
        {
            errno_exit ("VIDIOC_DQBUF EINVAL");
        }
        else if(errno == EIO)
        {
            errno_exit ("VIDIOC_DQBUF EIO");
        }
        else
        {
            errno_exit ("VIDIOC_DQBUF NONE");
        }

        return RESULT_FAILURE;
    }

    //printf("VIDIOC_DQBUF = %d\n", buf.index);
    memcpy(*fillbuf, buffer[buf.index].mmapbuf, buffer[buf.index].length);

    if (-1 == xioctl (*fd, VIDIOC_QBUF, &buf))
    {
        errno_exit ("VIDIOC_QBUF");
        return RESULT_FAILURE;
    }


    return RESULT_SUCCESS;
}

//just the main loop of this program 
int mainloop(int * fd, FrameBuffer *buffer, void **fillbuf)
{
    int read_ok = 0;

    while (!read_ok)
	{
		fd_set fds;
		struct timeval tv;
		int r;

		FD_ZERO (&fds);
		FD_SET (*fd, &fds);

		/* Select Timeout */
        tv.tv_sec = 2;
        tv.tv_usec = 0;

		//the classic select function, who allows to wait up to 2 seconds,
		//until we have captured data,
		r = select (*fd + 1, &fds, NULL, NULL, &tv);

		if (-1 == r)
		{
			if (EINTR == errno)
            {
			    errno_exit ("select error");
				continue;
            }
		}

		if (0 == r)
		{
		    errno_exit ("select timeout");
            continue;
		}

		//read one frame from the device and put on the buffer
        if(read_frame (fd, buffer, fillbuf))
        {
            continue;
        }

        read_ok = 1;
	}

    return RESULT_SUCCESS;
}



int stop_capturing (int * fd)
{
	enum v4l2_buf_type type;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	//this call to xioctl allows to stop the stream from the capture device
	if (-1 == xioctl (*fd, VIDIOC_STREAMOFF, &type))
    {
        errno_exit ("VIDIOC_STREAMOFF");
    }

    return RESULT_SUCCESS;
}


//free the shared memory area
int uninit_device (int *fd, FrameBuffer *buffer, void **fillbuf)
{
    for(unsigned int i=0; i<BUFFER_COUNT; ++i)
    {
        if (-1 == munmap (buffer[i].mmapbuf, buffer[i].length))
            errno_exit ("munmap");

        buffer[i].mmapbuf = NULL;
        buffer[i].length = 0;
    }

    free(*fillbuf);
    *fillbuf = NULL;

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(v4l2_requestbuffers));

    req.count               = 0;
    req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory              = V4L2_MEMORY_MMAP;

    if (-1 == xioctl (*fd, VIDIOC_REQBUFS, &req))
    {
        errno_exit ("VIDIOC_REQBUFS");
    }

    return RESULT_SUCCESS;
}





int close_device (int *fd)
{
	if (-1 == close (*fd))
		errno_exit ("close fd");

	*fd = -1;

	return RESULT_SUCCESS;
}


int set_control(int *fd, __u32 id, __s32 value)
{
    struct v4l2_queryctrl queryctrl;
    struct v4l2_control control;

    memset(&queryctrl, 0, sizeof(queryctrl));
    queryctrl.id = id;

    if (-1 == ioctl(*fd, VIDIOC_QUERYCTRL, &queryctrl)) {
        if (errno != EINVAL) {
            errno_exit ("VIDIOC_QUERYCTRL");
            return RESULT_FAILURE;
        } else {
            printf("Control %i is not supported\n", (int)id);
            return RESULT_FAILURE;
        }
    } else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
        printf("Control %i is not supported\n", (int)id);
        return RESULT_FAILURE;
    } else {
        memset(&control, 0, sizeof (control));
        control.id = id;
        control.value = value;

        if (-1 == ioctl(*fd, VIDIOC_S_CTRL, &control)) {
            errno_exit ("VIDIOC_S_CTRL");
            return RESULT_FAILURE;
        }
    }
    return RESULT_SUCCESS;
}

void capturer_mmap_init_v4l2_controls()
{
    v4l2_controls_mapping["Laser Power"] = REALSENSE_LASER_CTRL;
    v4l2_controls_mapping["Accuracy"] = REALSENSE_PATTERN_CTRL;
    v4l2_controls_mapping["Motion Range Trade Off"] = REALSENSE_EXPOSURE_CTRL;
    v4l2_controls_mapping["Filter Option"] = REALSENSE_FILTER_CTRL;
    v4l2_controls_mapping["Confidence Threshold"] = REALSENSE_CONFIDENCE_CTRL;
}

int capturer_mmap_init (PVideoStream p_video_stream)
{
    if(open_device(&(p_video_stream->fd), p_video_stream->videoName))
    {
        return 1;
    }

    if(init_device(&(p_video_stream->fd),
                   p_video_stream->videoName,
                   &(p_video_stream->width),
                   &(p_video_stream->height),
                   &(p_video_stream->pixelFormat) ))
    {
        return 1;
    }

    if(init_mmap (  &(p_video_stream->fd),
                    p_video_stream->videoName,
                    p_video_stream->frameBuffer,
                    &(p_video_stream->fillbuf),
                    &p_video_stream->buflen ))
    {
        return 1;
    }

    if(start_capturing(&(p_video_stream->fd)))
    {
        return 1;
    }

    return 0;
}



int capturer_mmap_get_frame(PVideoStream p_video_stream)
{
    //printf("get frame = %s\n", p_video_stream->videoName);
    return mainloop (&(p_video_stream->fd), p_video_stream->frameBuffer, &(p_video_stream->fillbuf));
}

void capturer_mmap_exit(PVideoStream p_video_stream)
{
    //printf("\ncapturer_mmap_exit - %s\n", p_video_stream->videoName);

    stop_capturing (&(p_video_stream->fd));

    uninit_device (&(p_video_stream->fd), p_video_stream->frameBuffer, &(p_video_stream->fillbuf));

    close_device (&(p_video_stream->fd));

}

int capturer_mmap_set_control(PVideoStream p_video_stream, const std::string &control, int value)
{
    std::map<std::string, int>::iterator it = v4l2_controls_mapping.find(control);
    if (it != v4l2_controls_mapping.end())
    {
        return set_control (&(p_video_stream->fd), (__u32)it->second, value);
    }
    else
    {
        printf("Control %s does not exist", control.c_str());
        return 1;
    }
}
