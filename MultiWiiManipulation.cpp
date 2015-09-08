#include "MultiWiiNetwork.h"
#include "MultiWiiManipulation.h"

#include <stdio.h>

uint16_t mount16(uint16_t a, uint16_t b){
    return (b<<8) + a;
}

int complemento2(int a){
    return (a > 32768? a-65536 : a);
}


//Initialises the MultiWii manipulation. SHOULD ALWAYS BE CALLED BEFORE CALLING ANY OF THE OTHER FUNCTIONS
//Inicializa a manipulacao do MultiWii. DEVE SER CHAMADA ANTES DE CHAMAR QUALQUER OUTRA FUNCAO
int multiwii_init_manipulation(void){
    int success = multiwii_init_serial_port();
    return success;

}

//Get imu data
//Obtem dados dos sensores IMU
imu_t multiwii_get_imu(void){


    //Send request for imu data
    //int success = ;
    while(!multiwii_send_data(MSP_RAW_IMU, NULL, 0))
    {
        printf("Error on multiwii send!\n");
       // exit(1);
    }

    int expected_receive_size = 24;

    //printf("I am Here!\n");
    received_frame_t received_frame = multiwii_get_data(expected_receive_size);

    //We are expecting 25 bytes of data (Multiwii sends a 0 before every frame)
    //TODO: Validation of received frame -> Checksum!!!!!
    if(received_frame.frame_size < (uint16_t)expected_receive_size){
        printf("Error on receive !\n");
        exit(0);
    }


//    int j;
//    int temp;
    imu_t ret;

    //Interpret the received frame
    ret.acc_x = complemento2(mount16(received_frame.received_frame[5], received_frame.received_frame[6]));
    ret.acc_y = complemento2(mount16(received_frame.received_frame[7], received_frame.received_frame[8]));
    ret.acc_z = complemento2(mount16(received_frame.received_frame[9], received_frame.received_frame[10]));
    ret.gyro_x = complemento2(mount16(received_frame.received_frame[11], received_frame.received_frame[12]))/8;
    ret.gyro_y = complemento2(mount16(received_frame.received_frame[13], received_frame.received_frame[14]))/8;
    ret.gyro_z = complemento2(mount16(received_frame.received_frame[15], received_frame.received_frame[16]))/8;
    ret.mag_x = complemento2(mount16(received_frame.received_frame[17], received_frame.received_frame[18]))/3;
    ret.mag_y = complemento2(mount16(received_frame.received_frame[19], received_frame.received_frame[20]))/3;
    ret.mag_z = complemento2(mount16(received_frame.received_frame[21], received_frame.received_frame[22]))/3;

    return ret;

}

//Get Control data
//Obtem dados de controle
control_t multiwii_get_control(void){
    //Makes the request for RC values
    int success = multiwii_send_data(MSP_RC, NULL, 0);

    if(!success){
        printf("Error on multiwii send!\n");
        exit(0);
    }


    int expected_receive_size = 22;
    received_frame_t received_frame = multiwii_get_data(expected_receive_size); //23?

    if(received_frame.frame_size < (uint16_t)expected_receive_size){
        printf("Error on multiwii send!\n");
        exit(0);
    }


    control_t ret;
    //Interpret the received frame
    ret.roll = complemento2(mount16(received_frame.received_frame[5], received_frame.received_frame[6]));
    ret.pitch = complemento2(mount16(received_frame.received_frame[7], received_frame.received_frame[8]));
    ret.yaw = complemento2(mount16(received_frame.received_frame[9], received_frame.received_frame[10]));
    ret.throttle = complemento2(mount16(received_frame.received_frame[11], received_frame.received_frame[12]));
    ret.aux1 = complemento2(mount16(received_frame.received_frame[13], received_frame.received_frame[14]));
    ret.aux2 = complemento2(mount16(received_frame.received_frame[15], received_frame.received_frame[16]));
    ret.aux3 = complemento2(mount16(received_frame.received_frame[17], received_frame.received_frame[18]));
    ret.aux4 = complemento2(mount16(received_frame.received_frame[19], received_frame.received_frame[20]));

    return ret;
}

//Set control data on the FC
void multiwii_set_control(control_t control_data){

    uint16_t serializedData[8];

    serializedData[0] = control_data.roll;
    serializedData[1] = control_data.pitch;
    serializedData[2] = control_data.yaw;
    serializedData[3] = control_data.throttle;
    serializedData[4] = control_data.aux1;
    serializedData[5] = control_data.aux2;
    serializedData[6] = control_data.aux3;
    serializedData[7] = control_data.aux4;

    //Converts to the order Multiwii wants to receive the 16 bit integers
    unsigned char sendBytes[16];
    for(int i = 0 ; i < 16 ; i+=2){
        sendBytes[i] = serializedData[i/2] &(0xff);
        sendBytes[i+1] = (serializedData[i/2]>>8) &(0xff);

    }

    multiwii_send_data(MSP_SET_RAW_RC, sendBytes, 16);

}

int multiwii_stop_manipulation(void){

    multiwii_close_serial_port();
    return 1;

}
