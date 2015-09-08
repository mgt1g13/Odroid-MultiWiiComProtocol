#include "MultiWiiProtocol.h"
#include "MultiWiiNetwork.h"
//#include <windows.h>
#include <stdio.h>
#define FRAME_SIZE(x) (x+6)

#include <errno.h>
#include <iostream>
#include <cstdlib>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <pthread.h>

using namespace std;

#define BAUDRATE B115200 		//B230400
//#define PERIOD 20000 // 20ms --> 50Hz
#define DEVICE "/dev/ttyUSB0"
#define FALSE 0
#define TRUE 1


// FILE DESCRIPTOR DA SERIAL
int fd;
uint16_t rc_signals[8] = { 1234 };
uint8_t rc_bytes[16] = { 0 };



void initPort()
{


//	//unsigned int tempo = 0;
//    struct termios oldtio,newtio;
//	//struct timespec time1, time2, time3;
//	//char b0[128], b1[64], b2[64], b3[64], b4[64];
//
//    fd = open(DEVICE, O_RDWR | O_NOCTTY );
//
//    /*  while (fd < 0) {
//        	printf(".\n");
//		sleep(1);
//	        fd = open(DEVICE, O_RDWR | O_NOCTTY );
//	close(fd);
//	        fd = open(DEVICE, O_RDWR | O_NOCTTY );
//	}*/
//
//	tcgetattr(fd,&oldtio); 					// save current port settings
//
//	bzero(&newtio, sizeof(newtio));
//	newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
//	newtio.c_iflag = IGNPAR;
//	newtio.c_oflag = 0;
//
//	// set input mode (non-canonical, no echo,...)
//	newtio.c_lflag = 0;
//
//	newtio.c_cc[VTIME]    = 0;   // inter-character timer unused
//	newtio.c_cc[VMIN]     = 1;   // blocking read until 1 char received
//	//newtio.c_cc[VMIN]     = 64;   // blocking read until 1 char received
//
//	tcflush(fd, TCIFLUSH);
//	tcsetattr(fd,TCSANOW,&newtio);
//	//tcsetattr(fd,TCSANOW,&oldtio);

//
//    struct termios toptions;
//    //printf("I am here\n");
//    fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
//
//    if (fd < 0) {
//        perror("init_serialport: Unable to open port ");
//        exit(1);
//    }
//
//    if (tcgetattr(fd, &toptions) < 0) {
//        perror("init_serialport: Couldn't get term attributes");
//        exit(1);
//    }
//
//
//    cfsetispeed(&toptions, BAUDRATE);
//    cfsetospeed(&toptions, BAUDRATE);
//
//    toptions.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
//    toptions.c_oflag = 0;
//
//    toptions.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
//    toptions.c_cflag &= ~(CSIZE | PARENB);
//    toptions.c_cflag |= CS8;
//    toptions.c_cflag |= CLOCAL;
//    toptions.c_cflag |= CRTSCTS;
//
//    toptions.c_cc[VMIN] = 1;
//    toptions.c_cc[VTIME] = 10;
//
////    // set parity8N1
////    toptions.c_cflag &= ~PARENB;
////    toptions.c_cflag &= ~CSTOPB;
////    toptions.c_cflag &= ~CSIZE;
////    toptions.c_cflag |= CS8;
////
////    // see:
////    toptions.c_cc[VMIN] = 1;
////    toptions.c_cc[VTIME] = 5;
//
//    // apply options
//    if (tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
//        perror("init_serialport: Couldn't set term attributes");
//        exit(1);
//    }

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        /*
         * Could not open the port.
         */

        perror("open_port: Unable to open /dev/ttyf1 - ");
        exit(1);
    }
    else
        fcntl(fd, F_SETFL, 0);


    struct termios options;

    /*
     * Get the current options for the port...
     */

    //tcgetattr(fd, &options);
    bzero(&options, sizeof(options));

    /*
     * Set the baud rates to 1500...
     */

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    /*
     * Enable the receiver and set local mode...
     */

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE; /* Mask the character size bits */
    options.c_cflag |= CS8;    /* Select 8 data bits */

    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;


    options.c_cc[VTIME] = 0;  // timeout after .1s that isn't working
    options.c_cc[VMIN] = 1;


    /*
     * Set the new options for the port...
     */

    tcsetattr(fd, TCSANOW, &options);

}


//Initialise the connection
//Inicia a conexao
int multiwii_init_serial_port(void)
{

    initPort();
    //setPort();
    return 1;
}

//Send the given data frame using the serial connection.
//Envia o quadro passado por parametro por porta serial.
int multiwii_send_data(unsigned char opcode, unsigned char* data_frame, unsigned char data_size)
{


    unsigned char *toSend = get_msp(opcode, data_frame, data_size);
    ///toSend[FRAME_SIZE(data_size)-1] = 85;
    int res = write(fd, toSend, FRAME_SIZE(data_size));
    //tcdrain(fd);
    free(toSend);
    if(res < 0)
    {
        printf("I am here\n");
        return 0;
    }

    return 1;
}

//Receives data from the serial port. Expected size is the maximum number of bytes it is supposed to read
//Recebe dados pela porta serial. Expected_size eh o numero maximo de bytes que devem ser lidos
received_frame_t multiwii_get_data(int expected_size)
{

    //sleep(1);
    received_frame_t received_frame;
    received_frame.frame_size = read(fd, received_frame.received_frame, expected_size);
    if( (int) received_frame.frame_size < expected_size )
    {
        printf("Error on read! %s - %d %d\n", strerror(errno), (int)received_frame.frame_size, (int)expected_size);
        exit(1);
    }

    return received_frame;

}

//Close the connection
//Fecha a conexao
void multiwii_close_serial_port()
{
    close(fd);

}



//

//void *thread_leitura(void *arg)
//{
//
//	while(1)
//	{
//		cin >> in;
//		switch(in)
//		{
//			case 't':	yaw = 1000;		//Ligar motores
//						throttle = 1000;
//						roll = 1500;
//						pitch = 1500;
//						break;
//
//			case 'b':	yaw = 2000;		//Desligar motores
//					  	throttle = 1000;
//						roll = 1500;
//						pitch = 1500;
//						break;
//
//			case 'a':	if(yaw < 2000) //girar para esquerda  LEME(Rudder)
//							yaw += 10;
//						break;
//
//			case 'd':	if(yaw > 1000)  //girar para direta
//							yaw -= 10;
//						break;
//
//			case 'w':	if(throttle < 2000) //subir ACELERADOR(Throttle)
//							throttle += 10;
//						break;
//
//			case 's':	if(throttle > 1000) //descer
//							throttle -= 10;
//						break;
//
//			case 'j':	if(roll < 2000)  //rolar para esquerda  ROLADOR(Aileron)
//							roll += 10;
//						break;
//
//			case 'l':	if(roll > 1000) //rolar para direita
//							roll -= 10;
//						break;
//
//			case 'i':	if(pitch < 2000) //inclinar para frente APROFUNDADOR(Elevator)
//							pitch += 10;
//						break;
//
//			case 'k':	if(pitch > 1000) //inclinar para trás
//							pitch -= 10;
//						break;
//
//
//			default : roll = 1500;
//					  pitch = 1500;
//					  yaw = 1500;
//					  throttle = 1500;
//		}
//
//    	get_rc_signals(rc_signals, roll, throttle, yaw, pitch);
//
//    	get_rc_bytes(rc_signals, rc_bytes);
//
//    	send_msp(MSP_SET_RAW_RC, rc_bytes, MSP_SET_RAW_RC_LENGTH);
//
//    	usleep(PERIOD);
//    }
//
//
//	// Set the RC signal array to the default values.
//	void zero_rc_signals(uint16_t * rc)
//	{
//		for(int i = 0; i < 8; i++)
//		{
//			rc[i] = RC_MID;
//		}
//		rc[THROTTLE] = RC_MIN;
//	}
//
//	// Compute RC signals from the latest nunchuk data.
//	void get_rc_signals(uint16_t * rc, int roll, int throttle, int yaw, int pitch)
//	{
//		zero_rc_signals(rc);
//		rc[ROLL] = roll;
//		rc[THROTTLE] = throttle;
//		rc[YAW] = yaw;
//		rc[PITCH] = pitch;
//	}
//
//	// Compute the RC message data bytes based on the given RC signals.
//	void get_rc_bytes(uint16_t * rc, uint8_t * buff)
//	{
//		int j = 0;
//		for(int i = 0; i < 8; i++)
//		{
//			buff[j++] = rc[i] & 0xFF; // LSB first
//			buff[j++] = (rc[i] >> 8) & 0xFF; // MSB second
//		}
//	}
//
//
//
//	// uint8_t = unsigned char
//	// Send a message using the MultiWii serial protocol.
//	void send_msp(uint8_t opcode, uint8_t *data, uint8_t n_bytes)
//	{
//		int res;
//		uint8_t checksum = 0;
//
//		// Send the MSP header and message length
//		if ((res = write(fd, "$M<", 3)) < 0) {
//			printf("ERRO SERIAL MSP HEADER!\n");
//		}
//
//		res = write(fd, n_bytes, 1);
//		if ((res = write(fd, "$M<", 3)) < 0) {
//			printf("ERRO SERIAL MSP MESSAGE LENGTH!\n");
//		}
//		checksum ^= n_bytes;
//
//		// Send the op-code
//		res = write(fd, opcode, 1);
//		if ((res = write(fd, "$M<", 3)) < 0) {
//			printf("ERRO SERIAL MSP OP-CODE!\n");
//		}
//		checksum ^= opcode;
//
//		// Send the data bytes
//		for(int i = 0; i < n_bytes; i++) {
//
//			res = write(fd, data[i], 1);
//			if ((res = write(fd, "$M<", 3)) < 0){
//				printf("ERRO SERIAL MSP DATA BYTES!\n");
//			}
//		checksum ^= data[i];
//		}
//
//		// Send the checksum
//		//Serial3.write(checksum);
//		res = write(fd, checksum, 1);
//		if ((res = write(fd, "$M<", 3)) < 0)
//		{
//			printf("ERRO SERIAL MSP CHECKSUM!\n");
//		}
//}
//
//int main()
//{
//	pthread_t t_leitura;
//	pthread_create(&t_leitura,NULL,thread_leitura,NULL);
//	pthread_join(t_leitura, 0);
//	return 0;
//}
