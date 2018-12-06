
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>
#include <linux/socket.h>
#include <linux/sockios.h>
//#include <linux/if.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>

//#include "can.h"


#if 0
#define PF_CAN 29
//#define AF_CAN PF_CAN

#define SIOCSCANBAUDRATE        (SIOCDEVPRIVATE + 0)
#define SIOCGCANBAUDRATE        (SIOCDEVPRIVATE + 1)

#define SOL_CAN_RAW             (SOL_CAN_BASE + CAN_RAW)
#define CAN_RAW_FILTER           1
#define CAN_RAW_RECV_OWN_MSGS    0x4 
#endif

typedef __u32 can_baudrate_t;

//extern int errno;
//static int set_baud_rate(const char* can_name, const char* baud_rate);
//static int set_can_up (const char* can_name);




#if 0

int deal(struct can_frame frame, ARS_DATA &ars_data)
{
    ars_data.object_id = frame.data[0];
    ars_data.distlat = ((0x07&(unsigned int)frame.data[2]) << 8)+(unsigned int)frame.data[3];
    ars_data.distlat = ars_data.distlat*0.2 - 204.6;
    ars_data.distlon = ((unsigned int)frame.data[1] << 5)+(((unsigned int)frame.data[2]>>3)&0x1f);
    ars_data.distlon = (ars_data.distlon*0.2) -500;
    //ars_data.vlat = (((unsigned int)frame.data[5]&0x3f) <<3) +(((unsigned int)frame.data[6]>>5)&0x70);
    //ars_data.vlat = ars_data.vlat*0.25-64;

    ars_data.vlat = (((unsigned int)frame.data[5]&0x3f) <<3) +(((unsigned int)frame.data[6]>>5)&0x07);
    ars_data.vlat = ars_data.vlat*0.25-64;

    ars_data.vlon = ((unsigned int )frame.data[4]<<2)+(((unsigned int)frame.data[5]>>6)&0x3);
    ars_data.vlon = ars_data.vlon*0.25 - 128;
    ars_data.rcs  = frame.data[7];
    ars_data.rcs = ars_data.rcs*0.5-64;
    ars_data.flag = frame.data[6]&0x7;
  //std::cout<<"object_id:"<< ars_data.object_id <<"  distance_long:"<<ars_data.distlon<<"  distance_vert:"<<ars_data.distlat<<"  speed_long:"<<ars_data.vlon<<"  speed_vert:"<<ars_data.vlat<<"  rcs"<<ars_data.rcs<<std::endl;
}

#endif

/*-------------------------命令行程序----------------------------------*/
void executeCMD(const char *cmd, char *result) {
    char buf_ps[1024];
    char ps[1024] = {0};
    FILE *ptr;
    strcpy(ps, cmd);
    if ((ptr = popen(ps, "r")) != NULL) {
        while (fgets(buf_ps, 1024, ptr) != NULL) {
            strcat(result, buf_ps);
            if (strlen(result) > 1024)
                break;
        }
        pclose(ptr);
        ptr = NULL;
    } else {
        printf("popen %s error\n", ps);
    }
}

/*-------------------------获取系统时间函数----------------------------------*/
void sysUsecTime(char (&system_time_data)[100]) {
    struct timeval tv;
    struct timezone tz;
    struct tm *t;
    gettimeofday(&tv, &tz);
    t = localtime(&tv.tv_sec);
    sprintf(system_time_data,"%d-%d-%d-%d_%d_%d_%ld", 1900+t->tm_year, 1+t->tm_mon, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, tv.tv_usec);
    cout<<system_time_data<<endl;
}


int write_can_data(int fd, int can_id, const char *str) {
    struct can_frame frame;
//	struct sockaddr_can addr;
    int send_size = 0;

    frame.can_id = can_id; //can device id
    memcpy(frame.data, str, 8);
    frame.can_dlc = 8;

//	send_size = sendto( fd, &frame, sizeof(struct can_frame),  0, (struct sockaddr*)&addr, sizeof(addr));
    send_size = write(fd, &frame, sizeof(struct can_frame));
    if (send_size < 0) {
        fprintf(stderr, "sendto error %s\n", __FUNCTION__);
    }
    fsync(fd);

    return send_size;
}


static int set_baud_rate(char *can_name, char *baud_rate) {
    pid_t pid;
    if ((pid = vfork()) < 0) {
        fprintf(stderr, "vfork error %s\n", __FUNCTION__);
        return -1;
    } else if (0 == pid) {
        char *arg[] = {"ip", "link", "set", can_name, "type", "can", "bitrate", baud_rate, NULL};
        /* ip link set can0 type can bitrate xxxx */

        if (execvp("ip", arg) < 0) {
            fprintf(stderr, "execvp error %s\n", __FUNCTION__);
            exit(1);
        }
    } else {
        waitpid(pid, NULL, 0);
    }
    return 0;
}

static int set_can_up(char *can_name) {
    pid_t pid;
    if ((pid = vfork()) < 0) {
        return -1;
    } else if (0 == pid) {
        char *arg2[] = {"ifconfig", can_name, "up", NULL};
        if (execvp("ifconfig", arg2) < 0) {
            fprintf(stderr, "execvp error %s\n", __FUNCTION__);
            exit(1);
        }
    } else {
        waitpid(pid, NULL, 0);
    }
    return 0;
}

static int set_can_down(char *can_name) {
    pid_t pid;
    if ((pid = vfork()) < 0) {
        return -1;
    } else if (0 == pid) {
        char *arg2[] = {"ifconfig", can_name, "down", NULL};
        if (execvp("ifconfig", arg2) < 0) {
            fprintf(stderr, "execvp error %s\n", __FUNCTION__);
            exit(1);
        }
    } else {
        waitpid(pid, NULL, 0);
    }
    return 0;
}


extern int init_can(char *can_name, char *baud_rate) {
    int fd_sock = 0;
    int ret_val = 0;

    int recv_own_msgs = 0;//set loop back:  1 enable 0 disable
    struct sockaddr_can addr;
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(struct ifreq));
    memset(&addr, 0, sizeof(struct sockaddr_can));

    set_can_down(can_name);
    set_baud_rate(can_name, baud_rate);
    set_can_up(can_name);
    sleep(1);

    fd_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_sock < 0) {
        fprintf(stderr, "socket error %s\n", __FUNCTION__);
        goto err1;
    }

    strcpy(ifr.ifr_name, can_name);
    ret_val = ioctl(fd_sock, SIOCGIFINDEX, &ifr);  //get index
    if (ret_val == -1 || (ifr.ifr_ifindex == 0)) {
        fprintf(stderr, "Can't get interface index for can, code= %d, can ifr_ifindex value: %d, name: %s\n", ret_val,
                ifr.ifr_ifindex, ifr.ifr_name);
        goto err2;
    }

    printf("%s can_ifindex = %x\n", ifr.ifr_name, ifr.ifr_ifindex);

#if 1
    if ((ret_val = setsockopt(fd_sock, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs))) <
        0) {
        fprintf(stderr, "setsockopt error %s\n", __FUNCTION__);
        goto err2;
    }

#endif
    addr.can_family = PF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if ((ret_val = bind(fd_sock, (struct sockaddr *) &addr, sizeof(addr))) < 0) {
        fprintf(stderr, "bind error %s   %d  fd:%d\n", __FUNCTION__, errno, fd_sock);
        goto err2;
    }


    return fd_sock;

    err2:
//	close(fd_sock);
    err1:
    return fd_sock;

//	return -1;
}


#if 0
int main(int argc, const char *argv[])
{
    init_can("can0","a")	;
    return 0;
}
#endif 
