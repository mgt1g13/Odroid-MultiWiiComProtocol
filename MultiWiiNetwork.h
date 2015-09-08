#ifndef ____MULTIWIINETWORK_____
#define ____MULTIWIINETWORK_____

#include <stdint.h>
#include <stdlib.h>

#define MSP_SET_MOTOR 214
#define MSP_MOTOR 104
#define MSP_RC 105
#define MSP_SET_RAW_RC 200
#define MSP_RAW_IMU 102



//Basic Parameters to configure the MultiWii serial communication: Check your MultiWii Build for these pieces information
//Other than the g_porta, all the values used are the default ones from MultiWii

//Parametros basicos para configurar a comunicacao serial no MultiWii: Os valores usados, tirando g_porta, sao os
//valores padroes usados pelo MultiWii.


typedef struct {

    long unsigned int frame_size;
    unsigned char received_frame[500];

} received_frame_t;



//Initialise the connection
//Inicia a conexao
int multiwii_init_serial_port(void);

//Send the given data frame using the serial connection.
//Envia o quadro passado por parametro por porta serial.
int multiwii_send_data(unsigned char opcode, unsigned char* data_frame, unsigned char data_size);

//Receives data from the serial port. Expected size is the maximum number of bytes it is supposed to read
//Recebe dados pela porta serial. Expected_size eh o numero maximo de bytes que devem ser lidos
received_frame_t multiwii_get_data(int expected_size);


//Close the connection
//Fecha a conexao
void multiwii_close_serial_port();




#endif // ____MULTIWIINETWORK_____
