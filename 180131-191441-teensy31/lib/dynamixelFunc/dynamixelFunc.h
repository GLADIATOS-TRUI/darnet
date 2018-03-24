#include <JointData.h>
#include <Head.h>
#include <Walking.h>

void writedata(unsigned char aID, unsigned char addr, unsigned char *data, unsigned int size);
void readData();
void regwritedata(unsigned char aID, unsigned char addr, unsigned char *data, unsigned int size);
void action(unsigned char aID);
void syncWrite(unsigned char *aID, unsigned short sizeI, unsigned char *data, int el, unsigned char addr);
void arrsyncWritepos (unsigned char *aID, unsigned short sizeI, int *data);
void arrsyncWritekick (unsigned char *aID, unsigned short sizeI, int *data, int times,int *state);
int countSpe3d (unsigned char id,unsigned char times,int tPos,int *state);
void tulisServo (Robot::JointData jd, unsigned char pilihan);
void tulisHead (Robot::Head *kepala);
void tulisBody (Robot::Walking *tubuh);
void ubah();