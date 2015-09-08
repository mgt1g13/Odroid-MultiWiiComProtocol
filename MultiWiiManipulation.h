#ifndef ____MULTIWIIMANIPULATION_____
#define ____MULTIWIIMANIPULATION_____

#include <stdint.h>

//Basic data structure to encapsulate IMU (magnetometer, accelerometer and gyroscope) data
//Estrututa de dados para armazenar dados de IMU(acelerometro, giroscopio e magnetometro)
typedef struct {
    int gyro_x;
    int gyro_y;
    int gyro_z;
    int acc_x;
    int acc_y;
    int acc_z;
    int mag_x;
    int mag_y;
    int mag_z;
} imu_t;


//Basic data structure used to encapsulate control data
//Estrututa de dados para armazenar dados de controle(acelerometro, giroscopio e magnetometro)
typedef struct {

    uint16_t throttle;
    uint16_t pitch;
    uint16_t roll;
    uint16_t yaw;
    uint16_t aux1;
    uint16_t aux2;
    uint16_t aux3;
    uint16_t aux4;

} control_t;


//Initialises the MultiWii manipulation. SHOULD ALWAYS BE CALLED BEFORE CALLING ANY OF THE OTHER FUNCTIONS
//Inicializa a manipulacao do MultiWii. DEVE SER CHAMADA ANTES DE CHAMAR QUALQUER OUTRA FUNCAO
int multiwii_init_manipulation(void);

//Get imu data
//Obtem dados dos sensores IMU
imu_t multiwii_get_imu(void);

//Get Control data
//Obtem dados de controle
control_t multiwii_get_control(void);

//Set control data on the FC
void multiwii_set_control(control_t control_data);

int multiwii_stop_manipulation(void);

#endif
