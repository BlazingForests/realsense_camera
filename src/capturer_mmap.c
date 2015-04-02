
#include "capturer_mmap.h"


static void errno_exit (const char *s)
{
	fprintf (stderr, "%s error %d, %s\n",s, errno, strerror (errno));
    //exit (EXIT_FAILURE);
}

//a blocking wrapper of the ioctl function
static int xioctl (int fd, int request, void *arg)
{
	int r;

	do r = ioctl (fd, request, arg);
	while (-1 == r && EINTR == errno);

	return r;
}

void fcc2s(char *str, unsigned int val)
{

    str[0] = val & 0xff;
    str[1] = (val >> 8) & 0xff;
    str[2] = (val >> 16) & 0xff;
    str[3] = (val >> 24) & 0xff;

}

//read one frame from memory and throws the data to standard output
static int read_frame  (int * fd)
{
    struct v4l2_buffer buf;//needed for memory mapping

	CLEAR (buf);

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl (*fd, VIDIOC_DQBUF, &buf)) 
	{
		switch (errno) 
		{
			case EAGAIN:
                return EXIT_FAILURE;

			case EIO://EIO ignored

			default:
				errno_exit ("VIDIOC_DQBUF");
		}
	}
			
    assert (buf.index < 1);

	if (-1 == xioctl (*fd, VIDIOC_QBUF, &buf))
    {
		errno_exit ("VIDIOC_QBUF");
        return EXIT_FAILURE;
    }

    //printf("read frame = %d\n", buf.bytesused);

    return 0;
}

//just the main loop of this program 
static void mainloop(int * fd)
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
                fprintf (stderr, "capturer mmap select error\n");
				continue;
            }
		}

		if (0 == r) 
		{
            fprintf (stderr, "select timeout\n");
            continue;
		}

		//read one frame from the device and put on the buffer
        if(read_frame (fd))
        {
            continue;
        }

        read_ok = 1;
	}        
}

static int stop_capturing (int * fd)
{
	enum v4l2_buf_type type;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	//this call to xioctl allows to stop the stream from the capture device
	if (-1 == xioctl (*fd, VIDIOC_STREAMOFF, &type))
    {
        errno_exit ("VIDIOC_STREAMOFF");
        return EXIT_FAILURE;
    }

    return 0;
}

static int start_capturing (int * fd)
{
    struct v4l2_buffer buf;
    CLEAR (buf);

    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = 0;

    if (-1 == xioctl (*fd, VIDIOC_QBUF, &buf))
    {
        errno_exit ("VIDIOC_QBUF");
        return EXIT_FAILURE;
    }

	//start the capture from the device
    if (-1 == xioctl (*fd, VIDIOC_STREAMON, &buf.type))
    {
		errno_exit ("VIDIOC_STREAMON");
        return EXIT_FAILURE;
    }

    return 0;
}


//free the shared memory area
static void uninit_device (FrameBuffer *buffer)
{
    if (-1 == munmap (buffer->data, buffer->length))
        errno_exit ("munmap");

    buffer->data = NULL;
    buffer->length = 0;
}

//alloc buffers and configure the shared memory area
static int init_mmap (int * fd, char * dev_name, FrameBuffer *framebuffer)
{
    framebuffer->data = NULL;
    framebuffer->length = 0;

    struct v4l2_requestbuffers req;
	CLEAR (req);

    req.count               = 1;
	req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory              = V4L2_MEMORY_MMAP;

	if (-1 == xioctl (*fd, VIDIOC_REQBUFS, &req)) 
	{
		if (EINVAL == errno) 
		{
            fprintf (stderr, "%s does not support memory mapping\n", dev_name);
            return EXIT_FAILURE;
		} else {
			errno_exit ("VIDIOC_REQBUFS");
		}
	}

    if (req.count < 1)
    {
        fprintf (stderr, "Insufficient buffer memory on %s\n",dev_name);
        return EXIT_FAILURE;
    }


    struct v4l2_buffer buf;
    CLEAR (buf);

    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = 0;

    if (-1 == xioctl (*fd, VIDIOC_QUERYBUF, &buf))
    {
        fprintf (stderr, "VIDIOC_QUERYBUF\n");
        return EXIT_FAILURE;
    }

    framebuffer->length = buf.length;
    framebuffer->data = mmap (NULL /* start anywhere */,
                                buf.length,
                                PROT_READ | PROT_WRITE /* required */,
                                MAP_SHARED /* recommended */,
                                *fd, buf.m.offset);

    if (MAP_FAILED == framebuffer->data)
    {
        fprintf (stderr, "mmap");
        return EXIT_FAILURE;
    }

    return 0;
}

//configure and initialize the hardware device
static int init_device (int *fd, char *dev_name,
                        int *width, int *height, unsigned int *pixel_format,
                        FrameBuffer *framebuffer)
{
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
    struct v4l2_format fmt;
    char pixel_str[5] = {0};

	if (-1 == xioctl (*fd, VIDIOC_QUERYCAP, &cap)) 
	{
		if (EINVAL == errno) 
		{
			fprintf (stderr, "%s is no V4L2 device\n", dev_name);
            return EXIT_FAILURE;
		} else {
			errno_exit ("VIDIOC_QUERYCAP");
		}
	}

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) 
	{
		fprintf (stderr, "%s is no video capture device\n",dev_name);
        return EXIT_FAILURE;
	}

	if (!(cap.capabilities & V4L2_CAP_STREAMING)) 
	{
		fprintf (stderr, "%s does not support streaming i/o\n",dev_name);
        return EXIT_FAILURE;
	}

	/* Select video input, video standard and tune here. */
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (-1 == xioctl (*fd, VIDIOC_CROPCAP, &cropcap)) 
	{
        /* Errors ignored. */
	}

	crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	crop.c = cropcap.defrect; /* reset to default */

	if (-1 == xioctl (*fd, VIDIOC_S_CROP, &crop)) 
	{
		switch (errno) {
			case EINVAL:
				/* Cropping not supported. */
			break;
			default:
				/* Errors ignored. */
			break;
		}
	}

	CLEAR (fmt);
	//set image properties
	fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = *width;
    fmt.fmt.pix.height      = *height;
    fmt.fmt.pix.pixelformat = *pixel_format;
	//fmt.fmt.pix.colorspace  = V4L2_COLORSPACE_SRGB;
	//fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;

	if (-1 == xioctl (*fd, VIDIOC_S_FMT, &fmt))
		errno_exit ("\nError: pixel format not supported\n");

    fcc2s(pixel_str, fmt.fmt.pix.pixelformat);
    printf ("\n========================\n"
            "%s \n"
            "v4l2_pix_format\n"
            "   width = %d\n"
            "   height = %d\n"
            "   pixelformat = %s\n"
            "   field = %d\n"
            "   bytesperline = %d\n"
            "   sizeimage = %d\n"
            "   colorspace = %d\n"
            "   priv = %d\n\n",
            dev_name,
            fmt.fmt.pix.width,
            fmt.fmt.pix.height,
            pixel_str,
            fmt.fmt.pix.field,
            fmt.fmt.pix.bytesperline,
            fmt.fmt.pix.sizeimage,
            fmt.fmt.pix.colorspace,
            fmt.fmt.pix.priv );

	/* Note VIDIOC_S_FMT may change width and height. */

	//check the configuration data
//	min = fmt.fmt.pix.width * 2;
//	if (fmt.fmt.pix.bytesperline < min)
//			fmt.fmt.pix.bytesperline = min;
//	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
//	if (fmt.fmt.pix.sizeimage < min)
//			fmt.fmt.pix.sizeimage = min;

    *width = fmt.fmt.pix.width;
    *height = fmt.fmt.pix.height;
    *pixel_format = fmt.fmt.pix.pixelformat;

    int initmmap = init_mmap (fd, dev_name, framebuffer);

    return initmmap;
}

static void close_device (int * fd)
{
	if (-1 == close (*fd))
		errno_exit ("close");

	*fd = -1;
}

static int open_device (int * fd, char * dev_name)
{
	struct stat st; 

	if (-1 == stat (dev_name, &st)) 
	{
		fprintf (stderr, "Cannot identify '%s': %d, %s\n",dev_name, errno, strerror (errno));
        return EXIT_FAILURE;
	}

	if (!S_ISCHR (st.st_mode)) 
	{
		fprintf (stderr, "%s is no device\n", dev_name);
        return EXIT_FAILURE;
	}

	*fd = open (dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

	if (-1 == *fd) 
	{
		fprintf (stderr, "Cannot open '%s': %d, %s\n",dev_name, errno, strerror (errno));
        return EXIT_FAILURE;
	}

    return EXIT_SUCCESS;
}




int                 pixel_format_sizeof  = 1;


int capturer_mmap_init (PVideoStream p_video_stream)//char* video_name, int* width, int* height)
{
    if(open_device(&(p_video_stream->fd), p_video_stream->videoName))
    {
        return 1;
    }

    if(init_device(&(p_video_stream->fd),
                   p_video_stream->videoName,
                   &(p_video_stream->width),
                   &(p_video_stream->height),
                   &(p_video_stream->pixelFormat),
                   &(p_video_stream->frameBuffer)))
    {
        return 1;
    }

    if(start_capturing (&(p_video_stream->fd)))
    {
        return 1;
    }

    return 0;
}



int capturer_mmap_get_frame(PVideoStream p_video_stream)
{
    mainloop (&(p_video_stream->fd));

    return 0;
}

void capturer_mmap_exit(PVideoStream p_video_stream)
{
    stop_capturing (&(p_video_stream->fd));

    uninit_device (&(p_video_stream->frameBuffer));

    close_device (&(p_video_stream->fd));

}
