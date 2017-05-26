#include <stdio.h>     
#include <stdlib.h> 
#include <unistd.h> 
#include <sys/types.h>  
#include <sys/stat.h>   
#include <fcntl.h> 
#include <termios.h> 
#include <errno.h> 
#include <sys/select.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <sys/ioctl.h>

#define D_ERR(format,...) printf("[ERR]%s#%d: "format"", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define D_INF(format,...) printf("[INF]%s#%d: "format"", __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define RS232_DEV "/dev/ttyS2"
#define RS485_DEV "/dev/ttyS0"
#define HIKIO_NAME "/dev/hikio"

#define HIK_IOC_BASE	        'H'
#define HIKIO_SEND_485			_IOW(HIK_IOC_BASE, 20, unsigned int)
#define HIKIO_REV_485			_IOW(HIK_IOC_BASE, 21, unsigned int)

/* baud rate defines */
#define S50         0
#define S75         1
#define S110        2
#define S150        3
#define S300        4
#define S600        5
#define S1200       6
#define S2400       7
#define S4800       8
#define S9600       9
#define S19200      10
#define S38400      11
#define S57600      12
#define S76800      13
#define S115200     14

/* data bits defines */
#define DATAB5      0
#define DATAB6      1
#define DATAB7      2
#define DATAB8      3

/* stop bits defines */
#define STOPB1      0
#define STOPB2      1

/* parity defines */
#define NOPARITY    0
#define ODDPARITY   1
#define EVENPARITY  2

/* flow control defines */
#define NOCTRL      0
#define SOFTCTRL    1       /* xon/xoff flow control */
#define HARDCTRL    2       /* RTS/CTS flow control */

/*************************************************
  Function: 	SetSerialBaud
  Description:	Set Serial port Baudrate
  Input:		fd	     -- file descriptor
  			speed   -- speed
  Output:
  Return:		OK(0)/ERROR(-1)
*************************************************/
int SetSerialBaud(int fd,int speed)
{
	struct termios opt;
	tcgetattr(fd,&opt);
	if (-1 == cfsetispeed(&opt, (speed_t)speed))
	{
		D_ERR("set input speed error: %d, %s\n", errno, strerror(errno));
	}
	if (-1 == cfsetospeed(&opt, (speed_t)speed))
	{
		D_ERR("set output speed error: %d, %s\n", errno, strerror(errno));
	}

	/* Enable the receiver and set local mode */
	opt.c_cflag |= (CLOCAL | CREAD);
	return tcsetattr(fd,TCSANOW,&opt);
}

/*************************************************
  Function: 	SetSerialRawMode
  Description:	Set Serial port to raw mode
  Input:		fd	     -- file descriptor
  Output:
  Return:		OK(0)/ERROR(-1)
*************************************************/
int SetSerialRawMode(int fd)
{
	struct termios opt;
	tcgetattr(fd,&opt);
	opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* input */
	opt.c_iflag &= ~(IXON|IXOFF|ICRNL|INLCR|IGNCR);	/* 不将 CR(0D) 映射成 NR(0A) */
	opt.c_oflag &= ~OPOST;	/* output */
	return tcsetattr(fd,TCSANOW,&opt);
}

/*************************************************
  Function: 	SetSerialRawMode
  Description:	Set Serial port to raw mode
  Input:		fd	     -- file descriptor
  Output:
  Return:		OK(0)/ERROR(-1)
*************************************************/
int SetSerialSysMode(int fd)
{
	struct termios opt;
	tcgetattr(fd,&opt);
	opt.c_lflag |= ICANON | ECHO | ECHOE | ISIG;
	opt.c_iflag |= ICRNL;
	opt.c_oflag |= OPOST;
	return tcsetattr(fd,TCSANOW,&opt);
}

/*************************************************
  Function: 	SetSerialParity
  Description:	Set Serial port parity check,stop bit,data bit
  Input:		fd	     -- file descriptor
  			databits --data bits
			stopbits --stop bits
			parity    --parity check
  Output:
  Return:		OK(0)/ERROR(-1)
*************************************************/
int SetSerialParity(int fd, int databits,int stopbits, int parity)
{
	struct termios opt;
	tcgetattr(fd, &opt);

	opt.c_cflag &= ~CSIZE;
	opt.c_cflag |= (unsigned int)databits;

	switch( stopbits )
	{
		case STOPB1:
			opt.c_cflag &= ~CSTOPB;
			break;
		case STOPB2:
			opt.c_cflag |= CSTOPB;
			break;
		default:
			return -1;
			break;
	}

	switch ( parity )
	{
		case NOPARITY:
			opt.c_cflag &= ~PARENB;
			opt.c_iflag &= ~INPCK;
			break;
		case ODDPARITY:
			opt.c_cflag |= (PARENB|PARODD);
			opt.c_iflag |= INPCK;
			break;
		case EVENPARITY:
			opt.c_cflag |= PARENB;
			opt.c_cflag &= ~PARODD;
			opt.c_iflag |= INPCK;
			break;
		default:
			return -1;
			break;
	}

	return tcsetattr(fd,TCSANOW,&opt);
}

/*************************************************
  Function: 	SetSerialFlowControl
  Description:	Set Serial port flow control
  Input:		fd	     -- file descriptor
			control  -- flow control
			enable  -- can contrl or not
  Output:
  Return:		OK(0)/ERROR(-1)
*************************************************/
int SetSerialFlowControl(int fd, int control, int enable)
{
	struct termios opt;
	tcgetattr(fd, &opt);

	switch( control )
	{
		case SOFTCTRL:
			if ( enable )
			{
				opt.c_iflag |= (IXON | IXOFF | IXANY);
			}
			else
			{
				opt.c_iflag &=~(IXON | IXOFF | IXANY);
			}
			break;
		case HARDCTRL:
			if ( enable )
			{
				opt.c_cflag |= CRTSCTS;
			}
			else
			{
				opt.c_cflag &=~CRTSCTS;
			}
			break;
		case NOCTRL:
			opt.c_iflag &=~(IXON | IXOFF | IXANY);
			opt.c_cflag &=~CRTSCTS;
			break;
		default:
			break;
	}

	return tcsetattr(fd,TCSANOW,&opt);
}

int initRs485(int fd, int baudrate, int data, int stop, int parity, int flowcontrol)
{
	int i;
    int iRet = -1;
	int rate[][2] = {
        {300,B300},
        {600,B600},
        {1200,B1200},
		{2400,  B2400},
		{4800,  B4800},
		{9600,  B9600},
		{19200, B19200},
		{38400, B38400},
		{57600, B57600},
		{115200,B115200}
	};
	int databits[] = {CS5, CS6, CS7, CS8};


	D_INF("baudrate = %d, data = %d, stop = %d, parity =%d , flowcontrol = %d\n", baudrate, data, stop, parity, flowcontrol);

	for (i = 0;i < (int)(sizeof(rate)/sizeof(rate[0]));i++)
	{
		if (rate[i][0] == baudrate)
		{
			break;
		}
	}

	if (i >= (int)(sizeof(rate)/sizeof(rate[0])))
	{
		D_INF("invalide baudrate and set baudrate to default 9600\n");
		baudrate = B9600;
	}
	else
	{
		baudrate = rate[i][1];
	}

	iRet = SetSerialRawMode(fd);
    if(iRet == -1)
    {
        D_ERR("SetSerialRawMode error.fd = %d\n", fd);
        return -1;
    }
	iRet = SetSerialBaud(fd, baudrate);
    if(iRet == -1)
    {
        D_ERR("SetSerialBaud error.fd = %d\n", fd);
        return -1;
    }
	iRet = SetSerialFlowControl(fd, flowcontrol, 0);
    if(iRet == -1)
    {
        D_ERR("SetSerialFlowControl error.fd = %d\n", fd);
        return -1;
    }
	iRet = SetSerialParity(fd, databits[data], stop, parity);
    if(iRet == -1)
    {
        D_ERR("SetSerialParity error.fd = %d\n", fd);
        return -1;
    }

	return 0;
}

/* Function: enable485Send
 * Description: enable 485 send
 * Input: bSend  -- send cmd or not
 * Output: none
 * Return: none
 */
static void enable485Send(int bSend)
{
     int fd = -1;
     
     fd = open(HIKIO_NAME,O_RDWR);
	 if (fd < 0)
	 {	
		  D_ERR("open HIKIO failed\n");
		  return;
	 } 
	 ioctl(fd,HIKIO_SEND_485, &bSend);
     close(fd);
}
 
/* Function: enable485Receive
 * Description: enable 485 receive
 * Input: bSend  -- receive cmd or not
 * Output: none
 * Return: none
 */
void  enable485Receive(int bReceive)
{ 
    int fd = -1;
    fd = open(HIKIO_NAME,O_RDWR);
    if (fd < 0)
    {  
         D_ERR("open HIKIO failed\n");
         return;
    } 
    ioctl(fd,HIKIO_REV_485, &bReceive);
    close(fd);
}


int readnn(int connfd, void *vptr, int n)
{
	int	nleft;
	int	nread;
	char *ptr;
	struct timeval 	select_timeout;
	fd_set rset;

	select_timeout.tv_sec = 3;
	select_timeout.tv_usec = 0;

	ptr = vptr;
	nleft = n;

	while (nleft > 0)
	{
		select_timeout.tv_sec = 10;
		select_timeout.tv_usec = 0;
		FD_ZERO(&rset);
		FD_SET(connfd, &rset);
		if(select(connfd+1, &rset, NULL, NULL, &select_timeout) <= 0) 
		{
			/*PRT_ERR(("readn: select failed, errno = 0x%x\n", errnoGet()));*/
			break;
		}
		if((nread = read(connfd, ptr, nleft)) < 0)
		{
			if(errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR)
			{
				nread = 0;
			}
			else
			{
                break;
			}
		}
		else if (nread == 0)
		{
			break;
		}
		nleft -= nread;
		ptr   += nread;
	}
	return(n - nleft);
}

int testSerialFuc(void)
{
    int fdRS485 = -1;
    int i;
    int iRet;
    int total = 0;
    int baundrate[5] = {9600, 4800, 2400, 19200, 115200};
    int iSuccess = 0;
    char databuf[255] = {0};

    sleep(1); 

    if((fdRS485 = open(RS485_DEV, O_RDWR, 0)) <= 0)
    {
        D_ERR("打开RS485失败:%s\n", strerror(errno));
        goto err;
    }
    D_INF("open %s success\n",RS485_DEV);
   
    sleep(1);    
    initRs485(fdRS485, baundrate[0], DATAB8, STOPB1, NOPARITY, NOCTRL);
    tcflush(fdRS485, TCIOFLUSH);
    enable485Send(0);
    enable485Receive(1);
    
    do
    {
        iRet = read(fdRS485, databuf, sizeof(databuf));
		if (iRet > 0);
        {
            total += iRet;        
            printf("接收了%d个字节, 总共%d个字节\n", iRet, total);
            for(i = 0; i< iRet; i++)
	        {
	            printf("0x%02x",databuf[i]);
	        }
	        printf("\n");
        }
		sleep(1);      

    }while(1);

    close(fdRS485);
    return 0;
err:
    if(fdRS485 > 0)
    {
        close(fdRS485);
    }
    return -1;
}

int main(int argc, char *argv[])
{
    testSerialFuc();	
	return 0;
}

