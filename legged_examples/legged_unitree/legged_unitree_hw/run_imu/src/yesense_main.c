#include     <stdio.h>      /*标准输入输出的定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*UNIX 标准函数定义*/
#include     <sys/types.h>  /**/
#include     <sys/stat.h>  
#include     <fcntl.h>	    /*文件控制定义*/
#include     <termios.h>    /*PPSIX 终端控制定义*/
#include     <errno.h>      /*错误号定义*/
#include     <sys/time.h>
#include     <string.h>
#include     <getopt.h>
#include     "analysis_data.h"
#include     "yesense_main.h"


/*----------------------------------------------------------------------*/
#define TRUE 		1
#define FALSE 		-1
#define RX_BUF_LEN	512

/*----------------------------------------------------------------------*/
unsigned char g_recv_buf[512] = {0};
unsigned short g_recv_buf_idx = 0;
protocol_info_t g_output_info = {0};

/*----------------------------------------------------------------------*/
int run_imu()
{
    int fd;
    int nread;
    char buffer[RX_BUF_LEN];
    char* dev  = NULL;
    struct termios oldtio,newtio;

    unsigned short cnt = 0;
    int pos = 0;

    speed_t speed = B460800;
    dev = "/dev/ttyACM2";	
    fd = open(dev, O_RDWR | O_NONBLOCK| O_NOCTTY | O_NDELAY); 
    if (fd < 0)	{
        printf("Can't Open Serial Port!\n");
        exit(0);	
    }
	
    printf("open serial port to decode msg!\n");

    //save to oldtio
    tcgetattr(fd, &oldtio);
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = speed | CS8 | CLOCAL | CREAD;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &= ~PARENB;
    newtio.c_iflag = IGNPAR;  
    newtio.c_oflag = 0;
    tcflush(fd,TCIFLUSH);  
    tcsetattr(fd,TCSAFLUSH,&newtio);  
    tcgetattr(fd,&oldtio);
	
    memset(buffer,0,sizeof(buffer));

    for (int i = 0; i < 1; i++)
    {
	nread = read(fd, buffer, RX_BUF_LEN);
	if(nread > 0)
	{
	    //printf("nread = %d\n", nread);
	    memcpy(g_recv_buf + g_recv_buf_idx, buffer, nread);             
	    g_recv_buf_idx += nread;
	}

        cnt = g_recv_buf_idx;
        pos = 0;
        if(cnt < YIS_OUTPUT_MIN_BYTES)
        {
            continue;
        }

        while(cnt > (unsigned int)0)
        {
            int ret = analysis_data(g_recv_buf + pos, cnt, &g_output_info);
            if(analysis_done == ret)	/*未查找到帧头*/
            {
                pos++;
                cnt--;
            }
            else if(data_len_err == ret)
            {
                break;
            }
            else if(crc_err == ret || analysis_ok == ret)	 /*删除已解析完的完整一帧*/
            {
                output_data_header_t *header = (output_data_header_t *)(g_recv_buf + pos);
                unsigned int frame_len = header->len + YIS_OUTPUT_MIN_BYTES;
                cnt -= frame_len;
                pos += frame_len;
                //memcpy(g_recv_buf, g_recv_buf + pos, cnt);

                if(analysis_ok == ret)
                {
                    printf("pitch: %f, roll: %f, yaw: %f\n", 
			  g_output_info.attitude.pitch, g_output_info.attitude.roll, g_output_info.attitude.yaw);
                }
	    }
	}

        memcpy(g_recv_buf, g_recv_buf + pos, cnt);
        g_recv_buf_idx = cnt;
	tcflush(fd,TCIFLUSH);
	usleep(10000);
    }

    close(fd);
}


