#include "global.h"
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>


uint16_t mount16(uint16_t a, uint16_t b){
    return (b<<8) + a;
}

int complemento2(int a){
    return (a > 32768? a-65536 : a);
}

void limpaBuffer(){
    DWORD numBytesRead;
    unsigned char Buffer[50];
    while(!ReadFile(g_hSerial, &Buffer, 3, &numBytesRead, NULL));
}
void processaLeitura()
{
    static int j = 1;
    static int temp = 0;
    static int canRead = 0;
    int   numBytesToRead  = 1 ;
    DWORD numBytesRead;
    unsigned char Buffer[50];
    int resultado = 0;
    while(true)
    {
        if( !ReadFile(g_hSerial, &Buffer, numBytesToRead, &numBytesRead, NULL) )
        {
            printf("\nErro Leitura");
            exit(0);
        }


        if( numBytesRead > 0 )
        {
            if(Buffer[0] == '$' || Buffer[0] == 174){
              canRead = 1;
              if(Buffer[0] == 174) j = 0;

            }
            if(canRead)
            {
                if( j<= 5 || j == 22)
                    printf("%c - %d\t\n", Buffer[0], Buffer[0]);
                else
                {
                    if(j%2 == 0)
                        temp = Buffer[0];
                    else
                        printf("%d\t\n", complemento2(mount16(temp, Buffer[0])));
                }
                j++;
                if(j == 23)
                {
                    j = 1;
                    canRead = 0;
                    break;
                }
            }
        }
    }
}

////IMU Example
//void processaLeitura()
//{
//    static int j = 0;
//    static int temp = 0;
//    int   numBytesToRead  = 1 ;
//    DWORD numBytesRead;
//    unsigned char Buffer[50];
//    int resultado = 0;
//    if( !ReadFile(g_hSerial, &Buffer, numBytesToRead, &numBytesRead, NULL) )
//    {
//        printf("\nErro Leitura");
//        exit(0);
//    }
//
//
//    if( numBytesRead > 0 )
//    {
//
//
//        switch(j)
//        {
//        case 0: break;
//        case 1: printf("%c", Buffer[0]);break;
//        case 2: printf("%c", Buffer[0]);break;
//        case 3: printf("%c", Buffer[0]);break;
//        case 4: printf("\nData Length -> %d\t", Buffer[0]);break;
//        case 5: printf("\nOperation -> %d\t", Buffer[0]);break;
//        case 6: temp = Buffer[0];break;
//        case 7: printf("\nAcc x -> %d\t", complemento2(mount16(temp, Buffer[0]))); break;
//        case 8: temp = Buffer[0]; break;
//        case 9: printf("\nAcc y -> %d\t", complemento2(mount16(temp, Buffer[0]))); break;
//        case 10: temp = Buffer[0]; break;
//        case 11: printf("\nAcc z -> %d\t", complemento2(mount16(temp, Buffer[0]))); break;
//        case 12: temp = Buffer[0];break;
//        case 13: printf("\nGyro x -> %d\t",
//                        complemento2(mount16(temp, Buffer[0]))/8); break;
//        case 14:temp = Buffer[0];break;
//        case 15:printf("\nGyro y -> %d\t", complemento2(mount16(temp, Buffer[0]))/8); break;
//        case 16:temp = Buffer[0];break;
//        case 17:printf("\nGyro z -> %d\t", complemento2(mount16(temp, Buffer[0]))/8);break;
//        case 18:temp = Buffer[0];break;
//        case 19:printf("\nMag x -> %d\t", complemento2(mount16(temp, Buffer[0]))/3);break;
//        case 20:temp = Buffer[0];break;
//        case 21:printf("\nMag y -> %d\t", complemento2(mount16(temp, Buffer[0]))/3);break;
//        case 22:temp = Buffer[0];break;
//        case 23:printf("\nMag z -> %d\t", complemento2(mount16(temp, Buffer[0]))/3);break;
//        case 24:temp = printf("\nChecksum -> %d\t", Buffer[0]);break;
//        }
//       j++;
//       if(j == 25) j = 0;
//    }
//}



int pitch(){
    int ret = 1500;
    int state1 = GetAsyncKeyState(0x57);
    if( state1 )
        ret = 2000;
    int state2 = GetAsyncKeyState(0x53);
    if( state2 )
        ret = 1000;
    if(state1 && state2)
        ret = 1500;

    return ret;
}

int roll(){
    int ret = 1500;
    int state1 = GetAsyncKeyState(0x45);
    if( state1 )
        ret = 2000;
    int state2 = GetAsyncKeyState(0x51);
    if( state2 )
        ret = 1000;
    if(state1 && state2)
        ret = 1500;

    return ret;
}

int yaw(){
    int ret = 1500;
    int state1 = GetAsyncKeyState(0x44);
    if( state1 )
        ret = 2000;
    int state2 = GetAsyncKeyState(0x41);
    if( state2 )
        ret = 1000;
    if(state1 && state2)
        ret = 1500;

    return ret;
}

int throttle(){

    static int ret = 1500;
    int state1 = GetAsyncKeyState(VK_UP);
    if( state1 )
        if(ret < MAX_THROTTLE)
            ret += 10;
    int state2 = GetAsyncKeyState(VK_DOWN);
    if( state2 )
        if(ret > 1000)
            ret -= 10;
    if(state1 && state2)
        ret = 1500;

    return ret;
}

void processaEscrita()
{
    static int j = 0;

    BYTE* toSend;

    uint16_t envio[8];
    for(int i  = 1; i < 8 ; i++){
        envio[i] = 1500;
    }
    envio[0] = roll();
    envio[1] = pitch();
    envio[2] = yaw();
    envio[3] = throttle();


    BYTE bytesEnvio[16];
    for(int i = 0 ; i < 16 ; i+=2){
        bytesEnvio[i] = envio[i/2] &(0xff);
        bytesEnvio[i+1] = (envio[i/2]>>8) &(0xff);

    }

    DWORD  numBytesToWrite = 6;
    DWORD  numBytesWritten;

    if(j == 0){
          toSend = get_msp(MSP_RC, NULL, 0);
          numBytesToWrite = 6;
         // printf("Leitura\t\n");
    }
    if(j == 1){
        toSend = get_msp(MSP_SET_RAW_RC, bytesEnvio, 16);
        numBytesToWrite = 22;
        //printf("Set\t\n");
    }
    j = (j+1)%2;

    if(!WriteFile (g_hSerial, toSend, cont, &numBytesWritten, NULL))
    {
        printf("\n error writing");
    }
    free(toSend);
}


////Get_raw_imu
//void processaEscrita()
//{
//    BYTE* toSend toSend = get_msp(102, NULL, 0);
//    DWORD  numBytesToWrite = 6;
//    DWORD  numBytesWritten;
//
//    if(!WriteFile (g_hSerial, toSend, numBytesToWrite, &numBytesWritten, NULL))
//    {
//        printf("\n error writing");
//    }
//    free(toSend);
//}


