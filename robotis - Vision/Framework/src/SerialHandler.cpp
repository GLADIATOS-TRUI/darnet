#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>
#include <pthread.h>

#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>

#include "MX28.h"
#include "SerialHandler.h"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

#define BUFFER_SIZE 1024
#define READ_SIZE 1024


const char*      portACM         = "/dev/ttyACM0";
const char*      portACM1[10]         = {"/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3", "/dev/ttyACM4",
                                          "/dev/ttyACM5", "/dev/ttyACM6", "/dev/ttyACM7", "/dev/ttyACM8",
                                          "/dev/ttyACM9", "/dev/ttyACM10"};

using namespace Robot;

SerialHandler* SerialHandler::m_UniqueInstance = new SerialHandler();

SerialHandler::SerialHandler()
{
	X_MOVE_AMPLITUDE = 0;
	A_MOVE_AMPLITUDE = 0;
	Y_MOVE_AMPLITUDE = 0;
	isRunning = false;
}

SerialHandler::~SerialHandler()
{
	
}
                                          
int SerialHandler::SerialStart(const char* portname, speed_t baud, int data){
    int fd;
    // Open the serial port as read/write, not as controlling terminal, and
    //   don't block the CPU if it takes too long to open the port.
    fd = open(portname, O_RDWR | O_NOCTTY);
        
    if (fd == -1) {
        int a = 0;
        while(a < 10){
            fd = open(portACM1[a], O_RDWR | O_NOCTTY);
            if(fd == -1){            
                a++;
            }
            else break;
        }
        printf("\nGa bisa buka port ye");
        usleep(1000);

        return -1;
        // fd = open(portACM1, O_RDWR | O_NOCTTY);
        // if(fd == -1){            
        //     printf("\nGa bisa buka port ye");
        //     usleep(1000000);
        //     return -1;
        
    }
    
    struct termios toptions;    // struct to hold the port settings
    
//  usleep(3500000);

    // Fetch the current port settings
    tcgetattr(fd, &toptions);
    
    // Set Input and Output BaudRate
    cfsetispeed(&toptions, baud);
    cfsetospeed(&toptions, baud);
    
    // Set up the frame information.
    toptions.c_cflag &= ~PARENB;    // no parity
    toptions.c_cflag &= ~CSTOPB;    // one stop bit
    toptions.c_cflag &= ~CSIZE;     // clear frame size info
    toptions.c_cflag |= CS8;        // 8 bit frames
    
    // c_cflag contains a few important things- CLOCAL and CREAD, to prevent
    // this program from "owning" the port and to enable receipt of data.
    // Also, it holds the settings for number of data bits, parity, stop bits,
    // and hardware flow control.
    toptions.c_cflag |= CREAD ;
    toptions.c_cflag |= CLOCAL;
    
     /* no hardware flow control */
     toptions.c_cflag &= ~CRTSCTS;
     
     /* disable input/output flow control, disable restart chars */
     toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
     
    /*
    ICRNL
    : map CR to NL (otherwise a CR input on the other computer
    will not terminate input)
    otherwise make device raw (no other input processing)
    */
    toptions.c_iflag = ICRNL;
     
     /* disable canonical input, disable echo,
     disable visually erase chars,
     disable terminal-generated signals */
     toptions.c_lflag = ICANON;
     /* disable output processing, output set to raw*/
     toptions.c_oflag =  ~OPOST;
     
    // Now that we've populated our options structure, let's push it back to the
    //   system.
    tcsetattr(fd, TCSANOW, &toptions);
    
    usleep(1000000);
    
    // Flush the input and output buffer one more time.
    tcflush(fd, TCIOFLUSH);
    printf("Berhasil\n");
    return fd;
}

void SerialHandler::Initialize(int baud)
{
	speed_t baudRate;
	if(baud == 9600)
	{
		baudRate = B9600;
	} else

	if(baud == 1000000)
	{
		baudRate = B1000000;
	}
	else baudRate = B9600;

    fprintf(stderr, "Baud rate : %d\n", baud);

    port = SerialStart(portACM, baudRate, READ_SIZE);

    WalkingStart();           
}

/*
void ambilChar(){

    int dataread, portKompas, hitungkompas, hitungreopen, datalama;
//    portKompas = serialStart(portACM, B9600, READ_SIZE);

printf("terima ga ya");
            dataread = -1;
            tcflush(fd, TCIOFLUSH);
            //fcntl(fd, F_SETFL, 0); // block until data comes in
            read(fd, bufferRead, 1);

            dataread = atoi(bufferRead);

            nser = dataread;

   // pthread_exit(NULL);
}
*/
void SerialHandler::KirimChar(char* inputIn, int fd)
{
//    printf("kirim ga ya");
//            dataread = -1;
            tcflush(fd, TCIOFLUSH);
//            fcntl(fd, F_SETFL, 0); // block until data comes in
//            read(fd, bufferRead, 8);
            int zz = sizeof(inputIn);
            write(fd, inputIn, zz);

            fprintf(stderr, "Data kirim : %s \n", inputIn);
//            dataread = atoi(bufferRead);

}


void SerialHandler::itoa(int value, int length, char output[], bool isSigned)
{
    // char output[length];
    int k = length;
    if(isSigned == true)
    {
        if(value < 0)
        {
            output[0] = '-';
            value *= -1;
        } 
        else if(value >= 0)
        {
            output[0] = '+';
        }        
        k--;
    }
    int i = 0;
    while (k) {
        output[length - i - 1] = '0' + (value % 10);
        value /= 10;
        i++;
        k--;
    }    
    // return output;                    
}



void SerialHandler::SendKinematics()
{
	char bufferWrite[11];
	bufferWrite[0] = 'K';

    char temp[3];

    int kVal[3];
    kVal[0] = X_MOVE_AMPLITUDE;
    kVal[1] = A_MOVE_AMPLITUDE;
    kVal[2] = Y_MOVE_AMPLITUDE;

    for(int z = 0; z < 3; z++)
    {

        itoa(kVal[z], sizeof(temp), temp, true);
        for(int w = 0; w < 3; w++)
        {
            bufferWrite[w + 1 + (3*z)] = temp[w];                    
	
	        if(z == 2 && w == 2)
	        {
	            bufferWrite[w + 1 + (3*z) + 1] = 'X';                   
	        }

        }

    }

//	fprintf(stderr, "hasil : %s\n", bufferWrite);
	KirimChar(bufferWrite, port);            
}

void SerialHandler::SendHead(int pan, int tilt)
{
	char bufferWrite[10];
	bufferWrite[0] = 'H';

    char temp[4];

    int hVal[2];

    hVal[0] = MX28::Angle2Value(pan);
    hVal[1] = MX28::Angle2Value(tilt);

    for(int z = 0; z < 2; z++)
    {
        itoa(hVal[z], sizeof(temp), temp, false);
        for(int w = 0; w < 4; w++)
        {
            bufferWrite[w + 1 + (4*z)] = temp[w];   
            if(z == 1 && w == 3)
            {
                bufferWrite[w + 1 + (4*z) + 1] = 'X';                   
            }

        }
    }

//	fprintf(stderr, "hasil : %s\n", bufferWrite);
	KirimChar(bufferWrite, port);            
}

void SerialHandler::SendAction(int aVal)
{
	char bufferWrite[4];
	bufferWrite[0] = 'A';

    char temp[2];

    itoa(aVal, sizeof(temp), temp, false);
    for(int z = 0; z < 2; z++)
    {
        bufferWrite[z + 1] = temp[z];
        if(z == 1)
        {
            bufferWrite[z + 2] = 'X';
        }
    }

//	fprintf(stderr, "hasil : %s\n", bufferWrite);
	KirimChar(bufferWrite, port);            

}

void SerialHandler::WalkingStop()
{
	SendKinematics();
	char bufferWrite[1];
	bufferWrite [0] = 'Q';
	KirimChar(bufferWrite, port);    
	isRunning = false;
}
void SerialHandler::WalkingStart()
{
	SendKinematics();
	char bufferWrite[1];
	bufferWrite[0] = 'S';
	KirimChar(bufferWrite, port);            	
	isRunning = true;
}

