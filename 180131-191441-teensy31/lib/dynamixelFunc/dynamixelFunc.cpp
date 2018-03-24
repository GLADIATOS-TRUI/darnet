#include <dynamixelFunc.h>
#include <packetHandler.hpp>
#include <Arduino.h>

#define BROADCAST_ID 0xFE
#define MAX_ID              0xFC    // 252

/* Macro for Control Table Value */
#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

/* Instruction for DXL Protocol */
#define INST_PING               1
#define INST_READ               2
#define INST_WRITE              3
#define INST_REG_WRITE          4
#define INST_ACTION             5
#define INST_FACTORY_RESET      6
#define INST_SYNC_WRITE         131     // 0x83
#define INST_BULK_READ          146     // 0x92

// Communication Result
#define COMM_SUCCESS        0       // tx or rx packet communication success
#define COMM_PORT_BUSY      -1000   // Port is busy (in use)
#define COMM_TX_FAIL        -1001   // Failed transmit instruction packet
#define COMM_RX_FAIL        -1002   // Failed get status packet
#define COMM_TX_ERROR       -2000   // Incorrect instruction packet
#define COMM_RX_WAITING     -3000   // Now recieving status packet
#define COMM_RX_TIMEOUT     -3001   // There is no status packet
#define COMM_RX_CORRUPT     -3002   // Incorrect status packet
#define COMM_NOT_AVAILABLE  -9000   //

///////////////// Protocol 1.0 Error bit /////////////////
#define ERRBIT_VOLTAGE          1       // Supplied voltage is out of the range (operating volatage set in the control table)
#define ERRBIT_ANGLE            2       // Goal position is written out of the range (from CW angle limit to CCW angle limit)
#define ERRBIT_OVERHEAT         4       // Temperature is out of the range (operating temperature set in the control table)
#define ERRBIT_RANGE            8       // Command(setting value) is out of the range for use.
#define ERRBIT_CHECKSUM         16      // Instruction packet checksum is incorrect.
#define ERRBIT_OVERLOAD         32      // The current load cannot be controlled by the set torque.
#define ERRBIT_INSTRUCTION      64      // Undefined instruction or delivering the action command without the reg_write command.

int statuss = 0;

void writedata(unsigned char aID, unsigned char addr, unsigned char *data, int size){
    int starts;
    std::vector <unsigned char> param;
    std::vector <unsigned char> rxpack;
    param.push_back(addr);
    for (int i=0;i<size;i++)
    param.push_back(data[i]);
    DynamixelPacket pam (aID,INST_WRITE,param);
    pam.transaction();
    int cnt = 2;
    if (aID == BROADCAST_ID)
        return ;
    IntervalTimer berhenti;
    statuss = 0;
    berhenti.begin(ubah, 200000); //counting time to wait from 
    while (1){
        while (Serial2.available()){
            rxpack.push_back(Serial2.read());
            statuss = 1;
        }
        if (statuss == 1)
            break;
    }
    berhenti.end();
    while (1){
        if (rxpack[cnt]==0xff){
            cnt++;
            continue;
        }
        break;   
    }
    unsigned char chksum = 0;
    for(int i = cnt;i<rxpack.size()-1;i++){
        chksum += rxpack[i];
    }
    chksum = ~chksum;

    if (chksum != rxpack[rxpack.size()-1]){
        Serial.printf("Comm Error");
        return ;
    }
    uint8_t offset = rxpack[cnt+1];
    if (rxpack[cnt+2] != 0x00)
        starts =1;
    if (rxpack[cnt+2]>=ERRBIT_INSTRUCTION){
        rxpack[cnt+2] -= ERRBIT_INSTRUCTION;
        Serial.println("Instruction Error");
    }
    if (rxpack[cnt+2]>=ERRBIT_OVERLOAD){
        rxpack[cnt+2] -= ERRBIT_OVERLOAD;
        Serial.println("Current too big");
    }
    if (rxpack[cnt+2]>=ERRBIT_CHECKSUM){
        rxpack[cnt+2] -= ERRBIT_CHECKSUM;
        Serial.println("Checksum Error");
    }
    if (rxpack[cnt+2]>=ERRBIT_RANGE){
        rxpack[cnt+2] -= ERRBIT_RANGE;
        Serial.println("Range Error");
    }
    if (rxpack[cnt+2]>=ERRBIT_OVERHEAT){
        rxpack[cnt+2] -= ERRBIT_OVERHEAT;
        Serial.println("Temp too High");
    }
    if (rxpack[cnt+2]>=ERRBIT_ANGLE){
        rxpack[cnt+2] -= ERRBIT_ANGLE;
        Serial.println("Angle Error");
    }
    if (rxpack[cnt+2]>=ERRBIT_VOLTAGE){
        rxpack[cnt+2] -= ERRBIT_VOLTAGE;
        Serial.println("Voltage Error");
    }
    if (starts == 1)
    return ;
}
void readData();
void regwritedata(unsigned char aID, unsigned char addr, unsigned char *data, int size);
void action(unsigned char aID);
void syncWrite(unsigned char *aID, unsigned short sizeI, unsigned char *data, int el, unsigned char addr){
    int each_length = el-1;
    std::vector <unsigned char> param;
    param.push_back(addr);
    param.push_back(each_length); //each_length merupakan panjang data, dikurang satu karena tidak termasuk id
    int offset = 0;
    for (int i=0;i<sizeI;i++){
        param.push_back(aID[i]);
        for (int j=0+offset ; j<each_length+offset ; j++)
            param.push_back(data[j]);
        offset += each_length;
    }
    //Serial.print("gg");
    DynamixelPacket vgg (BROADCAST_ID,INST_SYNC_WRITE,param);
    vgg.transaction();
    //Serial.print("vgg");
    while(Serial2.available())
    Serial2.read(); //Make sure RX is empty
    //Serial.print("dgg");
}



void arrsyncWritepos (unsigned char *aID, unsigned short sizeI, int *data){
    uint16_t val;
    uint8_t datas[100];
    uint8_t addre = 0x1E;
    int each_length = 3;
    int cnt = 0;
    for (int i=1;i<sizeI;i++){ // i=1,i<19
        val = data[i-1];
        if (i==10)
            val -= 124;
        datas[cnt++] = DXL_LOBYTE(val);
        datas[cnt++] = DXL_HIBYTE(val);
    }
    syncWrite(aID,sizeI,datas,each_length,addre);
}


void arrsyncWritekick (unsigned char *aID, unsigned short sizeI, int *data, int times,int *state){
    uint16_t val;

    uint8_t datas[200];
    uint8_t addre = 0x1E;
    int each_length = 5;
    int cnt = 0;
    for (int i=1;i<=sizeI;i++){ // i=1,i<19
        val = data[i-1];
        //for (int j=0;j<2;j++){
        if (i==10)
            val -=124;
       //Serial.printf("Byte: %x",datas[cnt-1]);
        //Serial.printf("Byte: %x",datas[cnt-1]);
        //val = data[i];
        datas[cnt++] = DXL_LOBYTE(val);
        datas[cnt++] = DXL_HIBYTE(val);
        val = countSpe3d (aID[i-1], times, val,state);
        datas[cnt++] = DXL_LOBYTE(val);
        datas[cnt++] = DXL_HIBYTE(val);
        //}
    }
    //Serial.println("EEE");
    syncWrite(aID,sizeI,datas,each_length,addre);
}
int countSpe3d (unsigned char id,unsigned char times,int tPos,int *state){
    int delta = abs(state[id] - tPos);
    state[id] = tPos;
  //  Serial.println("CS");
   // Serial.println(id);
    //Serial.println(delta);
   // float gege = (1.43*delta) / (times*1.0);
    //Serial.printf("%f : \n", gege); 
    //Serial.print("EXT"); 1.43
    return ( (14.15*delta) / (times*1.0) );
}

// new code @24/03/2018

void tulisServo (Robot::JointData jd, unsigned char pilihan){
    //pilihan = 1 berarti badan, pilihan = 2 berarti kepala, pilihan = 3 berarti body slope
    unsigned char sID[20];
    unsigned short size_id = 0;
    int each_length; 
    unsigned char alamat;
    uint8_t minival;
    uint16_t val;
    unsigned char sData[200];
    uint16_t cnt = 0;
    if (pilihan==1){
        alamat = 0x1A;
        for (int i=0;i<18;i++){
            each_length = 1;
            sID[i] = i+1;
            size_id++;
            minival = ( (uint8_t) jd.GetDGain(sID[i]) );
            sData[cnt++] = minival;
            each_length++;

            minival = ( (uint8_t) jd.GetIGain(sID[i]) );;
            sData[cnt++] = minival;
            each_length++;

            minival = ( (uint8_t) jd.GetPGain(sID[i]) );;
            sData[cnt++] = minival;
            each_length++;
            sData[cnt++] = 0;
            each_length++;
            val = jd.GetValue(sID[i]);
            if (i==10)
                val -= 124;
            sData[cnt++] = DXL_LOBYTE(val);
            sData[cnt++] = DXL_HIBYTE(val);
            each_length+=2;
        }
        syncWrite(sID,size_id,sData,each_length,alamat);
    }else if(pilihan==2){
        alamat = 0x1E;
        for (int i=0;i<2;i++){
            each_length = 1;
            sID[i] = i+19;
            size_id++;
            val = jd.GetAngle(sID[i]);
            sData[cnt++] = DXL_LOBYTE(val);
            sData[cnt++] = DXL_HIBYTE(val);
            each_length+=2;
        }
    }
}

void tulisHead (Robot::Head *kepala){
    IntervalTimer cek;
    cek.begin(ubah, 2000000);
    statuss = 0;
    tulisServo (kepala->m_Joint, 2);
    while (statuss==0);
    cek.end();
}

void tulisBody (Robot::Walking *tubuh){
    IntervalTimer cek;
    cek.begin(ubah, 2000000);
    statuss = 0;
    tulisServo (tubuh->m_Joint, 1);
    while (statuss==0);
    cek.end();
}

void ubah(){
    statuss = 1;
}