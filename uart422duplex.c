/**@file
 * @note    HangZhou Hikvision Digital Technology Co., Ltd. All Right Reserved.
 * @brief   test sendable model
 *
 * @author  Donggaopeng
 * @date    2014-08-05
 * @version V1.0
 *
 * @note 
 * @note
 * @warning
 */

#include <stdio.h>
#include <fcntl.h> //open
#include <sys/types.h> //open、select
#include <sys/stat.h> //open、
#include <unistd.h> //close、ioctl、read、sleep
#include <sys/ioctl.h>
#include <sys/time.h> //struct timeval、
#include <stdlib.h> //perror
#include <string.h> //memset
#include <errno.h> 
#include <pthread.h>

//#include "hal.h"

struct uart_cfg {
    int baudrate;
    int data;
    int stop;
    int parity;
    int flowcontrol;
};

#define UART_DEVICE          ("/dev/uart1")
#define UART_IOC_MAGIC       'U'
#define HAL_UART_CONFIG      _IOW(UART_IOC_MAGIC, 1, struct uart_cfg *)
#define HAL_UART_FIONREAD    _IOR(UART_IOC_MAGIC, 2, unsigned long *)
#define HAL_UART_TCFLSH      _IO (UART_IOC_MAGIC, 3)
//because of FIFO,the max value is 16
#define SEND_BUF_SIZE           (256) //(4*1024) 
#define UART232_RCV_BUF_SIZE    (2048)
#define ERROR  -1
#define OK  0
#define TEST_TIME               10
#define BUF_LEN                 64
#define UNDEF_LAST_BYTE         0xff


static int finish_flag = 0;
static int fd = -1;
static int rs485_test_ok = 0;
static unsigned char g_last_byte = UNDEF_LAST_BYTE;
static unsigned int g_total_bytes = 0;
static unsigned int g_lost_bytes = 0;
static time_t g_tm_last_lost_bytes = 0;


static int init_rs422(int baudrate, int data, int stop, int parity, int flowctrl)
{
    char devName[32];
    int ret = -1;
    struct uart_cfg cfg;

    printf("initRs485Port:baudrate = %d, data = %d, stop = %d, parity =%d , flowcontrol = %d\n", baudrate, data, stop, parity, flowctrl);

    sprintf(devName, "/dev/uart%d", 1);
    if((fd = open(devName, O_RDWR)) < 0)
    {
        printf("open %s failed %s\n", devName, strerror(errno));
        return -1;
    }
    
    if (ioctl(fd, HAL_UART_TCFLSH) < 0)
    { 
         printf ("HAL_UART_TCFLSH failed,(%s)\n",   strerror (errno));
         close(fd);
         return -1;
    } 
    
    cfg.baudrate = baudrate;
    cfg.data = data;
    cfg.stop = stop;
    cfg.parity = parity;
    cfg.flowcontrol = flowctrl;
    if(ioctl(fd, HAL_UART_CONFIG, &cfg))
    {
        printf("HAL_UART_CONFIG failed (%s)\n", strerror(errno));
        close(fd);
        return -1;
    }
    
    return fd;
}

static void save_last_byte(unsigned char byte)
{
    g_last_byte = byte;
    if(g_tm_last_lost_bytes == 0)
    {
        g_tm_last_lost_bytes = time(NULL);
    }
}

static void add_total_byte()
{
    g_total_bytes++;
}

static void test_curr_byte(unsigned char byte)
{
    if(g_last_byte == UNDEF_LAST_BYTE)
    {
        return;
    }

    unsigned int i_lost_bytes = 0;

    if(byte == g_last_byte)
    {
        i_lost_bytes = BUF_LEN - 1;
    }
    else if(byte > g_last_byte)
    {
        i_lost_bytes = (byte - g_last_byte - 1);
    }
    else
    {
        i_lost_bytes = (byte + BUF_LEN - g_last_byte - 1);
    }

    if(i_lost_bytes)
    {
        printf("当前数据[0x%02x]丢失%u字节，距离上次[0x%02x]丢失%d秒\n", byte, i_lost_bytes, g_last_byte, (int)(time(NULL)-g_tm_last_lost_bytes));
        g_tm_last_lost_bytes = time(NULL);
    }
    g_lost_bytes += i_lost_bytes;
    
}

static void prt_lost_bytes_radio()
{
    if(g_total_bytes)
    {
        printf("lost radio = %.2f(%u/%u)\n", (float)g_lost_bytes/g_total_bytes, g_lost_bytes, g_total_bytes);
    }
}


static void *rs422_rx( )
{
    int ret = 0;
    fd_set readfds;
    struct timeval timeout;
    unsigned long read_size = 0;
    //unsigned char buf_recv[BUF_LEN] = {0};
    unsigned char data_cnt[BUF_LEN] = {0};
    int i = 0, j = 0;
    unsigned int cnt = 0;
    unsigned char *buf_recv = NULL;
    FILE* fd_data = NULL;

    buf_recv = (unsigned char *)malloc(0x1000000);
    if (NULL == buf_recv) 
    {
        printf("malloc err\n");
        return NULL;
    }

    memset(buf_recv , 0 , 0x1000000);

    fd_data = fopen("/home/data.bin", "ab+");

    while (1) 
    {
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        ret = select(fd + 1, &readfds, NULL, NULL, &timeout);
        if (ret < 0) 
        {
            perror("select error\n");
            free(buf_recv);
            return NULL;
        } 
        else if (0 == ret) 
        {
            //printf("select timeout\n");
            continue;
        } 
        else if (FD_ISSET(fd, &readfds)) 
        {           
            ret = ioctl(fd, HAL_UART_FIONREAD, &read_size);
            if (ret < 0) 
            {
                perror("ioctl-1 error\n");
                free(buf_recv);
                return NULL;
            } 
            else if (UART232_RCV_BUF_SIZE == read_size) 
            {//over_write
                printf("clear RCV_BUF, when over_write\n");
                ret = ioctl(fd, HAL_UART_TCFLSH, 0); //clear RCV_BUF
                if (ret < 0) 
                {
                    perror("ioctl-2 error\n");
                    free(buf_recv);
                    return NULL;
                }       
            } 
            else 
            {
                memset(buf_recv, 0, BUF_LEN);
                if (read_size > UART232_RCV_BUF_SIZE) 
                {
                    read_size = UART232_RCV_BUF_SIZE;
                }
                ret = read(fd, buf_recv, read_size);
                if (ret != read_size) 
                {
                    printf("really read %d(%lu)", ret, read_size);
                    free(buf_recv);
                    return NULL;
                }
                #if 1
                //printf("read size: %d,  ", ret);
                for (i = 0; i < ret; i++) 
                {   
                    test_curr_byte(buf_recv[i]);
                    save_last_byte(buf_recv[i]);
                    add_total_byte();
                    //printf("0x%02x  ", (int)buf_recv[i]);
                }
                //printf("\n");
                //prt_lost_bytes_radio();
                #else
                for (i = 0; i < ret; i++) 
                {   
                    fprintf(fd_data, "0x%02x ", (int)buf_recv[i]);
                }
                
                fprintf(fd_data, "%s", "\n");
                #endif
            }
        }

        //if (finish_flag == 1) {
        //  break;
        //}
    }
    #if 0
    for(i=0;i<cnt;i++){
        for(j=0;j<BUF_LEN;j++){
            if(*(unsigned char*)(buf_recv+cnt) == j){
                data_cnt[j]++;
            }
        }
    }
    for(j=0;j<BUF_LEN;j++){
        printf("data: 0x%x, recv: %d\n",j,data_cnt[j]);
    }
    
    printf("cnt : 0x%x\n",cnt);
    for(j=0;j<cnt;j++){
        if(j % 8 ==0){
            printf("\n");
        }
        printf("0x%02x ", *(unsigned char*)(buf_recv+j));
    }
    
#endif
    free(buf_recv);
    return NULL;
    
}

static void *rs422_tx( )
{
    int ret = 0;
    int i = 0;
    unsigned char buf_send[BUF_LEN] = {0};
    unsigned char data_len = BUF_LEN; 
    unsigned char chechsum = 0;

    for (i = 0; i < BUF_LEN; i++) {
        buf_send [i] = i;
    }
    
    //for (i = 0; i < TEST_TIME; i++) {
    while (1) {
        ret = write(fd, buf_send, BUF_LEN);
        prt_lost_bytes_radio();

        sleep(1);
    }
    return NULL;
    //finish_flag = 1;
}

int main(int argc , char* argv[])
{
    int fd_uart = -1;
    int baudrate = 0;
    pthread_t thread_tx;
    pthread_t thread_rx;
    int err=0;
    if(argc != 2)
    {
        printf("please input baudrate!\n");
        return 0;
    }
    baudrate = strtoul(argv[1],0,0);
    fd_uart = init_rs422(baudrate, 3, 0, 0, 0);

    finish_flag = 0;
    
    err = pthread_create(&thread_tx, NULL, rs422_rx, &fd_uart);
    err |= pthread_create(&thread_rx, NULL, rs422_tx, &fd_uart);

    pthread_join(thread_tx, NULL);
    pthread_join(thread_rx, NULL);

    close(fd_uart);

    while(1)
    {
        pause();
    }

    return 0;
}

