/******************************************************************************
**
* Copyright (c)2021 SHI YANGLEI
* All Rights Reserved
*
*
******************************************************************************/

#include<stdio.h>

#include "sys_arms_supr.hpp"

#define clientudp 0

#if clientudp
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>
#include <sys/epoll.h>
#include <sys/wait.h>

#define MAXLINE 4096
#define UDPPORT 8001
#define SERVERIP "127.0.0.1"

int confd;
unsigned int addr_length;
char recvline[MAXLINE];
char sendline[MAXLINE];
struct sockaddr_in serveraddr;

int suprWorking = 1;

static void signalHandle(int mSignal)
{
  suprWorking = 0;

  printf("oops! stop!!!\n");
}

static void ininClient()
{
    if( (confd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ){
        perror("socket() error");
      }

       //通过struct sockaddr_in 结构设置服务器地址和监听端口；
     bzero(&serveraddr, sizeof(serveraddr));
     serveraddr.sin_family = AF_INET;
     serveraddr.sin_addr.s_addr = inet_addr(SERVERIP);
     serveraddr.sin_port = htons(UDPPORT);
     addr_length = sizeof(serveraddr);

     signal(SIGINT, signalHandle);
}


#endif


int main(int argc, char **argv)
{
    SUPR::dmsAppStartUp();

#if  clientudp
    ininClient();

    while (suprWorking)
    {
      // 向服务器发送数据，sendto() ；
      int send_length = 0;
      sprintf(sendline,"hello server!");
      send_length = sendto(confd, sendline, sizeof(sendline), 0, (struct sockaddr *) &serveraddr, addr_length);
      if(send_length > 0)
      {
          printf("%s\t", sendline);
      }
      usleep(10000);
    }
#endif
    return 0;
}

