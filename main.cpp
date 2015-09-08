//tutoriais comunicacao porta serial
//http://stackoverflow.com/questions/6068655/what-are-the-functions-i-need-use-for-serial-communication-in-vc


#include "MultiWiiManipulation.h"
#include <stdio.h>
//#include <windows.h>

#define MAX_THROTTLE 1850
#define MAX 2000
#define MIN 1000

#define IMU_MODE 0
#define RC_MODE 1


#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>



#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>		// inet_aton
#include <pthread.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/sockios.h>

int sockfd;
int newsockfd;
int port;

int init_socket(int argc, char *argv[])
{
    int portno = port, clilen;
//     char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    //   int n;
    //int data;
    printf( "using port #%d\n", portno );

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        perror( "ERROR opening socket");
    bzero((char *) &serv_addr, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons( portno );
    if (bind(sockfd, (struct sockaddr *) &serv_addr,
             sizeof(serv_addr)) < 0)
        perror(  "ERROR on binding" );
    listen(sockfd,5);
    clilen = sizeof(cli_addr);

    //--- infinite wait on a connection ---
    while ( 1 )
    {
        printf( "waiting for new client...\n" );
        if ( ( newsockfd = accept( sockfd, (struct sockaddr *) &cli_addr, (socklen_t*) &clilen) ) < 0 );
        else
            break;
    }
    return 0;
}



void set_mode(int want_key)
{
    static struct termios old, new_;
    if (!want_key)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old);
        return;
    }

    tcgetattr(STDIN_FILENO, &old);
    new_ = old;
    new_.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_);
}

char get_key()
{
//    int c = 0;
//    struct timeval tv;
//    fd_set fs;
//    tv.tv_usec = tv.tv_sec = 0;
//
//    FD_ZERO(&fs);
//    FD_SET(STDIN_FILENO, &fs);
//    select(STDIN_FILENO + 1, &fs, 0, 0, &tv);
//
//    if (FD_ISSET(STDIN_FILENO, &fs))
//    {
//        c = getchar();
//        set_mode(0);
//    }
//    return c;

    int charsToRead = 0;

    ioctl(newsockfd, SIOCINQ, &charsToRead);

    if(charsToRead == 0)
        return '\0';

    char ret;
    int n;

    n = recv(newsockfd,&ret,1,0);

    if (n < 0)
    {
        printf("Erro lendo do socket!\n");
        exit(1);
    }
    return ret;


}

void closeEverything(int signal)
{
    multiwii_stop_manipulation();
    close(newsockfd);
    close(sockfd);
    printf("Bye\n");
    exit(1);

}

int main(int argc, char **argv)
{
    if(argc < 2 )
    {
        printf("Uso ./leitor numero_porta\n");
        exit(1);
    }
    port = atoi(argv[1]);

    printf("\n -----------------------------------------------------\n\n");
    printf("\nPrograma para controle de quadcoptero Multiwii por porta Serial");
    printf("\nDesenvolvido por Matheus Garay Trindade\n\n");
    printf("\nUsando porta default: COM2 (Para mudar, va em MultiWiiNetwork.cpp)\n\n");

    signal(SIGINT, closeEverything);

    //set_mode(1);
    init_socket(argc, argv);
    /********************/
    multiwii_init_manipulation();
    /********************/

    int throttle = 1500;
    int pitch = 1500;
    int roll = 1500;
    int yaw = 1500;


//    int i = 0;
    while(1)
    {
        //set_mode(1);
        char c = get_key();

        switch(c)
        {
        case 'o':
            if(throttle < MAX_THROTTLE) throttle += 50;
            break;
        case 'l':
            if(throttle > MIN) throttle -= 50;
            break;
        case 'w':
            if(pitch < MAX) pitch += 50;
            break;
        case 's':
            if(pitch > MIN) pitch -= 50;
            break;
        case 'd':
            if(yaw < MAX) yaw += 50;
            break;
        case 'a':
            if(yaw > MIN) yaw -= 50;
            break;
        case 'e':
            if(roll < MAX) roll += 50;
            break;
        case 'q':
            if(roll > MIN) roll -= 50;
            break;

        }
//        gotoXY(0,10);
        control_t send;
////
        send.throttle = throttle;
        send.roll = roll;
        send.pitch = pitch;
        send.yaw = yaw;
        send.aux1 = 1500;
        send.aux2 = 1500;
        send.aux3 = 1500;
        send.aux4 = 1500;
//
//
        multiwii_set_control(send);

        control_t ctrl =
            multiwii_get_control();

        printf("Throttle -> %4d\t", ctrl.throttle);
        printf("Pitch    -> %4d\t", ctrl.pitch);
        printf("Roll     -> %4d\t", ctrl.roll);
        printf("Yaw      -> %4d\t", ctrl.yaw);

        printf("\n");

//        imu_t imu = multiwii_get_imu();
//
//        printf("Acc X    -> %4d ", imu.acc_x);
//        printf("Acc y    -> %4d ", imu.acc_y);
//        printf("Acc z    -> %4d ", imu.acc_z);
//        printf("Gyr X    -> %4d ", imu.gyro_x);
//        printf("Gyr Y    -> %4d ", imu.gyro_y);
//        printf("Gyr Z    -> %4d ", imu.gyro_z);
//        printf("Mag X    -> %4d ", imu.mag_x);
//        printf("Mag Y    -> %4d ", imu.mag_y);
//        printf("Mag Z    -> %4d ", imu.mag_z);
//        printf("\n");

    }



    return 0;
}

