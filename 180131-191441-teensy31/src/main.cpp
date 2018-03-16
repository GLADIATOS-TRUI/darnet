#include <Arduino.h>
#include <packetHandler.hpp>
#include <dynamixelFunc.h>
#include <JointData.h>
#include <Kinematics.h>
#include <Matrix.h>
#include <MotionManager.h>
#include <MotionModule.h>
#include <MotionStatus.h>
#include <MX28.h>
#include <Plane.h>
#include <Point.h>
#include <Vector.h>
#include <Walking.h>

#define UART_TXRTSE (2)
#define UART_TXRTSPOL (4)
#define BAUD_RATE 1000000
#define DXL_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

int state [] = {1498, 2518, 1844, 2248, 2381, 1712, 2048, 2047, 2052, 2044, 1637, 2459, 2653, 1441, 2389, 1707, 2040, 2021, 2048, 1809};
IntervalTimer mytimes;
int tendang = 0;
int mulai = 0;

void rutin(){
    tendang = 1;
}

int countSpeed (uint8_t id,uint8_t time,int tPos){
    int delta = abs(state[id] - tPos);
    state[id] = tPos;
  //  Serial.println("CS");
   // Serial.println(id);
    //Serial.println(delta);
    //float gege = (1.43*delta) / (time*1.0);
   // Serial.printf("%f : \n", gege); 
    return ( (143*delta) / (time*1.0) );
}


namespace std {
  void __throw_bad_alloc()
  {
    //Serial.println("Unable to allocate memory");
  }

  void __throw_length_error( char const*e )
  {
    //Serial.print("Length Error :");
    //Serial.println(e);
  }
}


void setup() {
    // put your setup code here, to run once:
    
    Serial.begin(BAUD_RATE);
    Serial2.begin(BAUD_RATE);
    //uint16_t val;
    // enable PIN 6 as hardware transmitter RTS with active HIGH.
    //CORE_PIN22_CONFIG = PORT_PCR_MUX(3);
    //UART1_MODEM = UART_TXRTSE | UART_TXRTSPOL; 
    Serial2.transmitterEnable(6);
    //pinMode(13, OUTPUT);
    Robot::Walking::GetInstance() -> Initialize();
    Robot::Walking::GetInstance() -> Start();
    /*
    uint8_t data[100];
    uint8_t id[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
    uint16_t duduk[] = {2324, 1772, 1804, 2371, 2050, 2046, 2046, 2050, 2071, 2025, 1323, 2792, 3513, 0574, 2815, 1295, 2060, 2036, 0512, 0643};
    unsigned short size_id = 20; //change to 18
    uint8_t addre = 0x1E;
    int each_length = 3;
    int cnt = 0;
    for (int i=1;i<=20;i++){ // i=1,i<19
        val = duduk[i];
        if (i==10)
            val -= 124;
        data[cnt++] = DXL_LOBYTE(val);
        data[cnt++] = DXL_HIBYTE(val);
    }
        syncWrite(id,size_id,data,each_length,addre);
    */
}

#define DEATHS 1
void loop() {
    // put your main code here, to run repeatedly:
        //code
    uint16_t val;
    uint8_t id[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
    int duduk[] = {2324, 1772, 1804, 2371, 2050, 2046, 2046, 2050, 2071, 2025, 1323, 2792, 3513, 574, 2815, 1295, 2060, 2036};
    //uint8_t id[] = {1,2};
    int tesi[] = {2324, 1772};
    int ses = 10;
    int tesi2[] = {2000, 2100};
    int ses2 = 10;
    int Rk1[] = {1818, 2278, 1845, 2248, 2381, 1711, 2048, 2048, 2102, 2049-124, 1731, 2161, 2594, 2036, 2332, 2009, 1928, 1944, 512, 648};
    int ss1 =  125;
    int Rk2[] = {1818, 2278, 2111, 2248, 2381, 1711, 2048, 2048, 2236, 2049-124, 1731, 2496, 2594, 1270, 2332, 1728, 1928, 1944, 512, 648};
    int ss2 =   125;
    int Rk3[] = {1818, 2278, 2111, 2248, 2381, 1711, 2048, 2048, 2236, 2049-124, 1666, 1974, 2594, 1078, 2332, 1941, 1928, 1944, 512, 648};
    int ss3 =  125;
    int Rk4[] = {1818, 2278, 2091, 2251, 2385, 1715, 2048, 2048, 2054, 1940-124, 1754, 2047, 2356, 902, 2213, 1300, 1910, 1974, 512, 648};
    int ss4 =   25;
    int Rk5[] = {1818, 2278, 2091, 2251, 2385, 1715, 2048, 2048, 2054, 2023-124, 1754, 2719, 2356, 1486, 2207, 1880, 1902, 1974, 512, 648};
    int ss5 =   10;
    int Rk6[] = {1818, 2278, 2091, 2251, 2385, 1715, 2048, 2048, 2054, 1940-124, 1754, 2949, 2356, 2110, 2207, 2156, 1902, 1974, 512, 648};
    int ss6 =   10;
    int Rk7[] = {1818, 2278, 1848, 2251, 2385, 1715, 2048, 2048, 2012, 1980-124, 1726, 2790, 2356, 1545, 2207, 2053, 1961, 1977, 512, 648};
    int ss7 =   10;
    int Rk8[] = {1818, 2278, 1848, 2251, 2385, 1715, 2048, 2048, 2012, 1980-124, 1726, 2396, 2356, 1756, 2207, 2000, 1961, 1977, 505, 716};
    int ss8 =   20;
    int Rk9[] = {1818, 2278, 1848, 2251, 2385, 1715, 2044, 2052, 2012, 2005-124, 1693, 2459, 2574, 1511, 2280, 1788, 2024, 2016, 512, 648};
    int ss9 =   20;
    
    int STDUP[20];
    for (int vg = 0; vg<20 ;vg++)
    STDUP[vg] = 2048;
    STDUP[9] -= 124;
    

    if ((Serial.available ()) & (0)){
    
    //uint8_t id[] = {1,2,3,4,5,6};    
    
    
    //arrsyncWritekick (id, 2, tesi, ses,state);
    //delay(ses*100); timing = time*100 !!
    //arrsyncWritekick (id, 2, tesi2, ses2,state);
    //delay(ses2*100);
    arrsyncWritepos(id,20,STDUP);
    delay(2);
    
    
    }
    
    if ((Serial.available())&&(DEATHS))
    {
        if (mulai==0){
            mulai = 1;
            mytimes.begin(rutin, 20000000);    
        }
        //while (Serial.available())
        //Serial.read();
        
        uint8_t idgg[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18}; //till 18only
        uint8_t data[100];
        unsigned short size_id = 18; //change to 18
       
        uint8_t addre = 0x1E;
        int each_length = 3;
        int cnt = 0;
        noInterrupts();
        int cpy = tendang;
        interrupts();
        if (tendang){

            //Serial.println("SSTOPP");
            mytimes.end();
            Robot::Walking::GetInstance() -> Stop();
            arrsyncWritepos(id,20,STDUP);
            delay(2);
            arrsyncWritekick(id,20,Rk1,ss1,state);
            delay(ss1*30);
            arrsyncWritekick(id,20,Rk2,ss2,state);
            delay(ss2*30);
            arrsyncWritekick(id,20,Rk3,ss3,state);
            delay(ss3*20);
            arrsyncWritekick(id,20,Rk4,ss4,state);
            delay(ss4*20);
            arrsyncWritekick(id,20,Rk5,ss5,state);
            delay(ss5*20);
            arrsyncWritekick(id,20,Rk6,ss6,state);
            delay(ss6*20);
            arrsyncWritekick(id,20,Rk7,ss7,state);
            delay(ss7*25);
            arrsyncWritekick(id,20,Rk8,ss8,state);
            delay(ss8*30);
            arrsyncWritekick(id,20,Rk9,ss9,state);
            delay(ss9*30);
            arrsyncWritepos(id,20,STDUP);
            delay(2);
            tendang = 0;
            delete Robot::Walking::GetInstance();
            //Walking* Walking::m_UniqueInstance = new Walking();
            Robot::Walking *m_UniqueInstance = new Robot::Walking();
            Robot::Walking::GetInstance() -> Initialize();
            Robot::Walking::GetInstance() -> Start();
            mytimes.begin(rutin,10000000);
        }else{
            Robot::Walking::GetInstance() -> Process();
        for (int i=1;i<=18;i++){ // i=1,i<19

            val = Robot::Walking::GetInstance() -> m_Joint.GetValue(i);
           // val = duduk[i-1];
            if (i==10)
                val -= 124;
            data[cnt++] = DXL_LOBYTE(val);
            data[cnt++] = DXL_HIBYTE(val);
        }
        syncWrite(idgg,size_id,data,each_length,addre);
        //Serial.print("VEGEGE: ");
        // Serial.println(Robot::Walking::GetInstance() -> m_Joint.GetValue(10));
        delay(2);
        }
        /*
        std::vector <unsigned char> test;
        test.push_back(0x18);
        test.push_back(0x01);
        DynamixelPacket *gg = new DynamixelPacket(0xFE,0x03,test);
        //eof
        starts = 0;
        gg -> transaction();
        delay(1000);
        delete gg;
        Serial.println();
        while (Serial2.available()){
            Serial.printf("%x ",Serial2.read());
        }
        
        Serial.println();
        std::vector <unsigned char> test2;
        test2.push_back(0x1E);
        test2.push_back(0x00);
        test2.push_back(0x08);
        DynamixelPacket *vgg = new DynamixelPacket(0xFE,0x03,test2);
        vgg -> transaction();
        delay(500);
        delete vgg;
        Serial.println();
          while (Serial2.available()){
            Serial.printf("%x ",Serial2.read());
        }
        std::vector <unsigned char> test3;
        test3.push_back(0x1E);
        test3.push_back(0x00);
        test3.push_back(0x08);
        vgg = new DynamixelPacket(0xFE,0x03,test3);
        vgg -> transaction();
        delay(500);
        delete vgg;
        Serial.println();
          while (Serial2.available()){
            Serial.printf("%x ",Serial2.read());
        }
        Serial.flush();
        */
        /*
        int each_length = 3;
        uint8_t id[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
        unsigned short size_id = 18;
        uint8_t addre = 0x1E;
        uint8_t data[] = {0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x08,0x00,0x08};
        delay(200);
        uint8_t addre1 = 0x18;
        uint8_t data2[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
        syncWrite(id,size_id,data2,2,addre1);
        delay(200);
        syncWrite(id,size_id,data,each_length,addre);
        delay(200);
        */

    }
}

