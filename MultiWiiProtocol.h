#ifndef ____MULTIWIIPROTOCOL_____
#define ____MULTIWIIPROTOCOL_____


//MSP refers to the MultiWii Serial protocol. Please go to http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
//for more information

//MSP se refere ao protocolo de comunicacao da plataforma MultiWii. Mais informacoes em
//http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol


//Macro to calculate the frame size when having x bytes
//Macro para calculo do tamanho do quadro quando enviamos x bytes
#define MULTIWII_FRAME_SIZE(X) (x+6)



//Generates a frame according to the MSP format. DATA SHOULD BE FREED
//Gera um quadro no formato aceito pelo MSP. CHAMAR FREE SOBRE O RETORNO
unsigned char * get_msp(unsigned char opcode, unsigned char* data, unsigned char n_bytes);




#endif
