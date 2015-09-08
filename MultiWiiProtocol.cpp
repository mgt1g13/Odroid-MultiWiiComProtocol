#include "MultiWiiProtocol.h"
#include <stdlib.h>
#include <stdio.h>

//Generates a frame according to the MSP format. DATA SHOULD BE FREED
//Gera um quadro no formato aceito pelo MSP. CHAMAR FREE SOBRE O RETORNO

//Each [] is an octet
//Format -> ['$']['M']['<'][n_bytes][opcode]n_bytes*[data] [checksum]

unsigned char* get_msp(unsigned char opcode, unsigned char* data, unsigned char n_bytes)
{
    //Minimum number of bytes on a frame
    int cont = 6;
    unsigned char checksum = 0;
    unsigned char* ret =(unsigned char*)malloc(50*sizeof(unsigned char));

    //Insert the header
    ret[0] = '$';
    ret[1] = 'M';
    ret[2] = '<';

    //Insert the number of bytes sent along with the message
    ret[3] = (n_bytes&0xff);
    checksum ^= (n_bytes&0xff);

    //Insert opcode
    ret[4] = opcode;
    checksum ^= opcode;
    //insert data
    int i = 0;
    for(i = 0; i < n_bytes; i++)
    {
        ret[i+5] = (data[i]);
        checksum ^= data[i];
        cont++;
    }
    // insert the checksum
    ret[i+5] = checksum;
  //  printf("check %d\n", checksum);
    return ret;
}
