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
#include <Head.h>
#include <cstdlib>
#include <mpu6050.h>

#define UART_TXRTSE (2)
#define UART_TXRTSPOL (4)
#define BAUD_RATE 1000000
#define BAUD_RATE_ODROID 1000000
#define DXL_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

int state [] = {1498, 2518, 1844, 2248, 2381, 1712, 2048, 2047, 2052, 2044, 1637, 2459, 2653, 1441, 2389, 1707, 2040, 2021, 2048, 1809};
IntervalTimer mytimes;
int tendang = 0;
int mulai = 0;
unsigned char runns = 0;
const int ledPin = 13;
MPU6050 imu;


void rutin(){
    tendang = 1;
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

void rutinKinem(char *masukan, unsigned char sizemasukan);
void rutinHead(char *masukan, unsigned char sizemasukan);
void rutinAction(char *masukan, unsigned char sizemasukan);
void rutinBaca(char *masukan, unsigned char sizemasukan);

void setup() {
    // put your setup code here, to run once:
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_INT);   
    pinMode(ledPin, OUTPUT);
    Serial.begin(BAUD_RATE_ODROID);
    Serial2.begin(BAUD_RATE);
    //uint16_t val;
    // enable PIN 6 as hardware transmitter RTS with active HIGH.
    //CORE_PIN22_CONFIG = PORT_PCR_MUX(3);
    //UART1_MODEM = UART_TXRTSE | UART_TXRTSPOL; 
    Serial2.transmitterEnable(6);
    //pinMode(13, OUTPUT);
    /*
    if (!imu.begin(AFS_2G, GFS_250DPS)) {
        Serial.println("MPU6050 is online...");
    }
    else {
        Serial.println("Failed to init MPU6050");
    while (true);
    }
    */
    Robot::Walking::GetInstance() -> Initialize();
    
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
//testing

#define DEATHS 1
void loop() {
    unsigned char c = 0x00; //To receive data from Serial
    
    /* Old code
    if ((Serial.available())&&(DEATHS))
    {
        if (mulai==0){
            mulai = 1;
            mytimes.begin(rutin, 10000000);   
            tulisAction(3,state); //action 1 kick, action3 sit, action 2 STDUP
            delay(3);
            tulisAction(2,state); 
            delay(3);
        }

        
        //while (Serial.available())
        //Serial.read();
        
        //uint8_t idgg[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18}; //till 18only
        //uint8_t data[100];
        //unsigned short size_id = 18; //change to 18
       
        //uint8_t addre = 0x1E;
       // int each_length = 3;
       // int cnt = 0;
        noInterrupts();
        int cpy = tendang;
        interrupts();
        if (tendang){
            //Serial.println("SSTOPP");
            mytimes.end();
            Robot::Walking::GetInstance() -> Stop();
            tulisAction(1,state); //action 1 kick, action3 sit, action 2 STDUP
           // tendang = 0;
           // delete Robot::Walking::GetInstance();
            //Walking* Walking::m_UniqueInstance = new Walking();
           // Robot::Walking *m_UniqueInstance = new Robot::Walking();
           Robot::Walking::GetInstance() -> Initialize();
            Robot::Walking::GetInstance() -> Start();
            
            mytimes.begin(rutin,5000000);
            tendang = 0;
        }else{
            Robot::Walking::GetInstance() -> Process();
            tulisBody(Robot::Walking::GetInstance());
        }
       

    }
    End of Old code*/

    //The flow program
    // Wait to start or stop -> check if there is command from odroid. If yes then process the command, else 
    // just walk in place. Then if the command is Stop -> back to first again.
    //Beginned of new code
    //State 1
    //Serial.flush();
    digitalWrite(ledPin,LOW);
    while (!(Serial.available())); // State to wait command from Odroid
    //State 2
    digitalWrite(ledPin,HIGH);
    
    if ((Serial.available()) && (!(mulai))){ //This state happen after odroid send the command, and will see the command is start or not
        c = Serial.read();
        if (c == 'S'){
            Robot::Walking::GetInstance() -> Start();
            mulai = 1;
            //while (Serial.available())
            //    Serial.read(); //Make sure RX buffer is empty!
            //Suruh duduk berhenti 10ms
            //tulisAction(3,state); //action 1 kick, action3 sit, action 2 STDUP
            //delay(2000);
            //tulisAction(2,state); 
            //delay(2000);

            //Cuman biar diri doang
            //tulisAction(3,state);
            //mulai = 0;
            //Robot::Walking::GetInstance()->Stop();
            //end of code

            Robot::Walking::GetInstance() -> X_MOVE_AMPLITUDE = 0;
            Robot::Walking::GetInstance() -> A_MOVE_AMPLITUDE = 0;
        
        }
    }
    while (Robot::Walking::GetInstance()->IsRunning() == 1){
    //State 3 processing kinematics and waiting command from odroid
        if(mulai){
            //Command Parsing
            unsigned char hitung = 0;  //For counting purpose of bytes from receiving serial
            unsigned char besarParam = 1;
            char *param = (char*) malloc (besarParam * sizeof(char));
            if (param == NULL)
                SCB_AIRCR = 1;
            c = 0x00;
            if(Serial.available()){
                //First Check is K, H, A, or R ?
                c = Serial.read();
                if (c == 'K'){
                    rutinKinem(param, besarParam);
                    //Robot::Walking::GetInstance() -> A_MOVE_AMPLITUDE = 10;
                }else if (c== 'H'){
                    rutinHead(param, besarParam);
                    //Robot::Walking::GetInstance() -> A_MOVE_AMPLITUDE = 20;
                }else if (c== 'A'){
                    rutinAction(param, besarParam);
                    //Robot::Walking::GetInstance() -> A_MOVE_AMPLITUDE = 77;
                }else if (c== 'R'){
                    //rutinBaca(param, besarParam);
                    for (int i=1;i<21;i++){
                        state[i-1] = Robot::MotionStatus::m_CurrentJoints.GetValue(i);
                     }
                    tulisAction(1,state);
                    //Robot::Walking::GetInstance() -> A_MOVE_AMPLITUDE = 0;
                }else if (c=='Q'){
                    //mulai = 0;
                    Robot::Walking::GetInstance() -> Stop();
                    //Serial.println("STOPPING PROGRAM");
                    mulai = 0;
                }else{
                    //Serial.println("Wrong command(Maybe Comm Error)");
                    //Robot::Walking::GetInstance() -> Stop();
                    //Serial.println("STOPPING PROGRAM");
                    //mulai = 0;
                }            
            }
            free(param);
        }
    //end of newcode
        Robot::Walking::GetInstance() -> Process();
        for (int id=1;id<21;id++){
            Robot::MotionStatus::m_CurrentJoints.SetSlope(id, Robot::Walking::GetInstance()->m_Joint.GetCWSlope(id), Robot::Walking::GetInstance()->m_Joint.GetCCWSlope(id));
            Robot::MotionStatus::m_CurrentJoints.SetValue(id, Robot::Walking::GetInstance()->m_Joint.GetValue(id));
            Robot::MotionStatus::m_CurrentJoints.SetPGain(id, Robot::Walking::GetInstance()->m_Joint.GetPGain(id));
            Robot::MotionStatus::m_CurrentJoints.SetIGain(id, Robot::Walking::GetInstance()->m_Joint.GetIGain(id));
            Robot::MotionStatus::m_CurrentJoints.SetDGain(id, Robot::Walking::GetInstance()->m_Joint.GetDGain(id));
        }
        
        tulisBody(Robot::Walking::GetInstance());
    }
}

void rutinKinem(char *masukan, unsigned char sizemasukan){
    int temp;
    int temp2;
    int temp3;
    delay(2); //make sure RX buffer already filled up
    char c = 0x00;
    int cnt = 0;
    while(1){
        if (Serial.available()){
            c = Serial.read();
            if (c=='X')
                break;
            masukan[cnt++] = c;
            realloc(masukan, ++sizemasukan);
        }
    }
    char temps[2];
    for (int i=1;i<=2;i++)
        temps[i-1] = masukan[i];
    temp = atoi(temps);
    if (masukan[0]=='-')
        temp *= -1;

    for (int i=4;i<=5;i++)
        temps[i-4] = masukan[i];
    temp2 = atoi(temps);
    if (masukan[3]=='-')
        temp2 *= -1;

    for (int i=7;i<=8;i++)
        temps[i-7] = masukan[i];
    temp3 = atoi(temps);
    if (masukan[6]=='-')
        temp3 *= -1;
    Robot::Walking::GetInstance() -> A_MOVE_AMPLITUDE = temp3;
    Robot::Walking::GetInstance() -> X_MOVE_AMPLITUDE = temp;
    Robot::Walking::GetInstance() -> Y_MOVE_AMPLITUDE = temp2;
    Robot::Walking::GetInstance() -> Process();
    for (int id=1;id<21;id++){
        Robot::MotionStatus::m_CurrentJoints.SetSlope(id, Robot::Walking::GetInstance()->m_Joint.GetCWSlope(id), Robot::Walking::GetInstance()->m_Joint.GetCCWSlope(id));
        Robot::MotionStatus::m_CurrentJoints.SetValue(id, Robot::Walking::GetInstance()->m_Joint.GetValue(id));
        Robot::MotionStatus::m_CurrentJoints.SetPGain(id, Robot::Walking::GetInstance()->m_Joint.GetPGain(id));
        Robot::MotionStatus::m_CurrentJoints.SetIGain(id, Robot::Walking::GetInstance()->m_Joint.GetIGain(id));
        Robot::MotionStatus::m_CurrentJoints.SetDGain(id, Robot::Walking::GetInstance()->m_Joint.GetDGain(id));
    }
    tulisBody(Robot::Walking::GetInstance());
}
void rutinHead(char *masukan, unsigned char sizemasukan){
     int temp;
    int temp2;
    delay(2); //make sure RX buffer already filled up
    char c = 0x00;
    int cnt = 0;
    while(1){
        if (Serial.available()){
            c = Serial.read();
            if (c=='X')
                break;
            masukan[cnt++] = c;
            realloc(masukan, ++sizemasukan);
        }
    }
    char temps[4];
    for (int i=0;i<=3;i++)
        temps[i] = masukan[i];
    temp = atoi(temps);

    for (int i=4;i<=7;i++)
        temps[i-4] = masukan[i];
    temp2 = atoi(temps);
    Robot::Head::GetInstance() -> m_Joint.SetValue(19,temp);
    Robot::MotionStatus::m_CurrentJoints.SetValue(19, Robot::Head::GetInstance()->m_Joint.GetValue(19));
    Robot::Head::GetInstance() -> m_Joint.SetValue(20,temp2);
    Robot::MotionStatus::m_CurrentJoints.SetValue(20, Robot::Head::GetInstance()->m_Joint.GetValue(20));
    tulisHead(Robot::Head::GetInstance());
}
void rutinAction(char *masukan, unsigned char sizemasukan){
    char c= 0x00;
    int cnt = 0;
    delay(2);
    while(1){
        if (Serial.available()){
            c = Serial.read();
            if (c=='X')
                break;
            masukan[cnt++] = c;
            realloc(masukan, ++sizemasukan);
        }
    }
    char temps[1];
    temps[0] = c;
    int keadaan[20];
    for (int i=1;i<21;i++){
        keadaan[i-1] = Robot::MotionStatus::m_CurrentJoints.GetValue(i);
    }
    tulisAction(atoi(temps),keadaan);
    for (int i=1;i<21;i++){
        Robot::MotionStatus::m_CurrentJoints.SetValue(i,keadaan[i-1]);
    }
}
void rutinBaca(char *masukan, unsigned char sizemasukan){}
void rutinIMU(){
  int16_t ax, ay, az, gx, gy, gz;
  if (imu.getMotion6Counts(&ax, &ay, &az, &gx, &gy, &gz)) {
        
        Serial.print("AX");
        Serial.print(ax);
        Serial.print("AY");
        Serial.print(ay);
        Serial.print("AZ");
        Serial.print(az);
        Serial.print("GX");
        Serial.print(gx);
        Serial.print("GY");
        Serial.print(gy);
        Serial.print("GZ");
        Serial.print(gz);
        Serial.println();
        Serial.flush();
        
       /*
        Serial.print(ax);
        Serial.print(" ");
        Serial.print(ay);
        Serial.print(" ");
        Serial.print(az);
        Serial.print(" ");
        Serial.print(gx);
        Serial.print(" ");
        Serial.print(gy);
        Serial.print(" ");
        Serial.print(gz);
        Serial.println();
        delay(500);
        */
    }
}


/*
yang hadir:
    Fattah
    Gushandi
    Yo Panji
    Kripton
    Althof
    Fahriza
    Koku

Gushandi -> Tukang solder. Board yang sudah ada harus direvisi lagi karena salah. Plannya design lagi, coba tes ke robot;
            (128.500 pengeluarannya)
Yo Panji -> Problem jalan ga balance!, sama masih lemes. Motion udah dicatet semua. Measure jalannya ampe bener
Fattah -> Reparasi darput dan udah cakep lagi :v. Deteksi gambar dah bisa, tinggal tes demo. 
Kripton -> Ngira2 delay buat moving speed, benerin motion nendang, bikin alur komunikasi. problem belom ada. plan ngetes komunikasi.

darnet -> kompas coba ke teensy dulu 

yang kurang, darnet :
- komunikasi odroid-teensy -> 1
- interfacing ke sensor (KOMPAS KE ODROID) -> 2
- motion -> 1
- kamera (image processing) -> 2
- game controller -> 3
- power dist yang fix -> 1
- bracket kamera C922 -> 2
- servo MX64 yang baru -> 1

yang kurang, darput :
- bracket kamera C922
- power dist yang fix
- board teensy yang fix
- kompas taro luar dibelakang
- kompas ke teensy, dibikin pinout buat ke kompas di luar atau dibikin satu bus sama imu di i2c
 
yang kurang, dardroid :
- banyak

yang kurang, bioloid :
- belum tau, mau dicari tau minggu ini

gushandi bikin grup bareng anggota muda elektrik buat koordinasi kerjaan.

ojt mekanik minta tolong ajarin om dody aja dulu, kamil lama.

ojt programmer ngerjain bioloid, nyobain sensor, nanti bikin grup baru aja biar enak koordinasinya.


*/