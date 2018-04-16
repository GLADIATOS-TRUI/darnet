/*
 *   MotionManager.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
//#include <iostream>
//#include <fstream>
//#include "FSR.h"
#include "MX28.h"
#include "MotionManager.h"
//#include "kalman.h"
//#include "../SerialArduino/BacaSerialArduino.cpp"
//#include <iostream>
//#include <fstream>

using namespace Robot;

/*double last_time = 0.0;

static double getTime()
{
	struct timeval tv;
	gettimeofday(&tv, 0);

	return tv.tv_sec + tv.tv_usec / 1e6;
}
*/
MotionManager* MotionManager::m_UniqueInstance = new MotionManager();
// BacaSerialArduino RS;
//BacaSerialArduino RS = BacaSerialArduino("/dev/ttyUSB1");

MotionManager::MotionManager() :
        //m_CM730(0),
        m_ProcessEnable(false),
        m_Enabled(false),
        m_IsRunning(false),
        m_IsThreadRunning(false),
        m_IsLogging(false),
        DEBUG_PRINT(false)
{
    for(int i = 0; i < JointData::NUMBER_OF_JOINTS; i++)
        m_Offset[i] = 0;
}

MotionManager::~MotionManager()
{
}

/*bool MotionManager::Initialize(CM730 *cm730)
{
    int value, error;

    m_CM730 = cm730;
    m_Enabled = false;
    m_ProcessEnable = true;

    if(m_CM730->Connect() == false)
    {
        if(DEBUG_PRINT == true)
            fprintf(stderr, "Fail to connect CM-730\n");
        return false;
    }

    for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
    {
        if(DEBUG_PRINT == true)
            fprintf(stderr, "ID:%d initializing...", id);
        
        if(m_CM730->ReadWord(id, MX28::P_PRESENT_POSITION_L, &value, &error) == CM730::SUCCESS)
        {
            MotionStatus::m_CurrentJoints.SetValue(id, value);
            MotionStatus::m_CurrentJoints.SetEnable(id, true);

            if(DEBUG_PRINT == true)
                fprintf(stderr, "[%d] Success\n", value);
        }
        else
        {
            MotionStatus::m_CurrentJoints.SetEnable(id, false);

            if(DEBUG_PRINT == true)
                fprintf(stderr, " Fail\n");
        }
    }

   // RS = BacaSerialArduino();

    m_CalibrationStatus = 0;
    m_FBGyroCenter = 512;
    m_RLGyroCenter = 512;

    return true;
}

bool MotionManager::Reinitialize()
{
    m_ProcessEnable = false;

    m_CM730->DXLPowerOn();

    int value, error;
    for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
    {
        if(DEBUG_PRINT == true)
            fprintf(stderr, "ID:%d initializing...", id);
        
        if(m_CM730->ReadWord(id, MX28::P_PRESENT_POSITION_L, &value, &error) == CM730::SUCCESS)
        {
            MotionStatus::m_CurrentJoints.SetValue(id, value);
            MotionStatus::m_CurrentJoints.SetEnable(id, true);

            if(DEBUG_PRINT == true)
                fprintf(stderr, "[%d] Success\n", value);
        }
        else
        {
            MotionStatus::m_CurrentJoints.SetEnable(id, false);

            if(DEBUG_PRINT == true)
                fprintf(stderr, " Fail\n");
        }
    }

    m_ProcessEnable = true;
    return true;
}

void MotionManager::StartLogging()
{
    char szFile[32] = {0,};

    int count = 0;
    while(1)
    {
        sprintf(szFile, "Logs/Log%d.csv", count);
        if(0 != access(szFile, F_OK))
            break;
        count++;
        if(count > 256) return;
    }

    m_LogFileStream.open(szFile, std::ios::out);
    for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++)
        m_LogFileStream << "ID_" << id << "_GP,ID_" << id << "_PP,";
    m_LogFileStream << "GyroFB,GyroRL,AccelFB,AccelRL,L_FSR_X,L_FSR_Y,R_FSR_X,R_FSR_Y" << std::endl;

    m_IsLogging = true;
}

void MotionManager::StopLogging()
{
    m_IsLogging = false;
    // RS.closeSerial();
    m_LogFileStream.close();
}

void MotionManager::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, OFFSET_SECTION);
}
void MotionManager::LoadINISettings(minIni* ini, const std::string &section)
{
    int ivalue = INVALID_VALUE;

    for(int i = 1; i < JointData::NUMBER_OF_JOINTS; i++)
    {
        char key[10];
        sprintf(key, "ID_%.2d", i);
        if((ivalue = ini->geti(section, key, INVALID_VALUE)) != INVALID_VALUE)  m_Offset[i] = ivalue;
    }
}
void MotionManager::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, OFFSET_SECTION);
}
void MotionManager::SaveINISettings(minIni* ini, const std::string &section)
{
    for(int i = 1; i < JointData::NUMBER_OF_JOINTS; i++)
    {
        char key[10];
        sprintf(key, "ID_%.2d", i);
        ini->put(section, key, m_Offset[i]);
    }
}

#define GYRO_WINDOW_SIZE    100
#define ACCEL_WINDOW_SIZE   30
#define MARGIN_OF_SD        2.0
// BacaSerialArduino RS = BacaSerialArduino("/dev/ttyUSB0");

// double last_time = 0.0;

// static double getTime()
// {
//     struct timeval tv;
//     gettimeofday(&tv, 0);

//     return tv.tv_sec + tv.tv_usec / 1e6;
// }

ofstream outputFileFBaccel;
ofstream outputFileRLaccel;
ofstream outputFileGyroX;
ofstream outputFileGyroY;
ofstream outputFileGyroZ;
ofstream outputFileCompass;

void MotionManager::Process()
{
    // RS.serialWriteReady();
    RS.bacaSerial_Start();

	outputFileFBaccel.open("dataIMUFBaccel.txt", ios_base::app);
	outputFileRLaccel.open("dataIMURLaccel.txt", ios_base::app);
	outputFileGyroX.open("dataIMUGyroX.txt", ios_base::app);
	outputFileGyroY.open("dataIMUGyroY.txt", ios_base::app);	
	outputFileGyroZ.open("dataIMUGyroZ.txt", ios_base::app);

    if(m_ProcessEnable == false || m_IsRunning == true)
        return;

    m_IsRunning = true;

    //  Mengambil data dari IMU / arduino  //
    double FBaccel = RS.getFB_Accel();
    double RLaccel = RS.getRL_Accel();
    double gyroX = RS.getX_Gyro();
    double gyroY = RS.getY_Gyro();
    double gyroZ = RS.getZ_Gyro();
    double compass = RS.getCompass();

    MotionStatus::TOMBOL_1 = RS.getButton_1();
    MotionStatus::TOMBOL_2 = RS.getButton_2();
    MotionStatus::TOMBOL_3 = RS.getButton_3();

    Kalman kalmanX;
    kalmanX.setAngle(RLaccel);
  
    Kalman kalmanY;
    kalmanY.setAngle(FBaccel);
  
    Kalman kalmanZ;
    kalmanZ.setAngle(compass);
    
    double  time = getTime();
    double  dt = time - last_time;

    if (last_time == 0.0)
    {
        dt = 0.0;
    }
        
    last_time = time;

    double kalmanRLaccel = kalmanX.getAngle(RLaccel, gyroY, dt); // Calculate the angle using a Kalman filter
    double kalmanFBaccel = kalmanY.getAngle(FBaccel, gyroZ, dt); // Calculate the angle using a Kalman filter
    double kalmanCompass = kalmanZ.getAngle(compass, gyroX, dt); // Calculate the angle using a Kalman filter

    MotionStatus::Compass = kalmanCompass;

    // calibrate gyro sensor
    if(m_CalibrationStatus == 0 || m_CalibrationStatus == -1)
    {
        static int fb_gyro_array[GYRO_WINDOW_SIZE] = {512,};
        static int rl_gyro_array[GYRO_WINDOW_SIZE] = {512,};
        static int buf_idx = 0;

        if(buf_idx < GYRO_WINDOW_SIZE)
        {
            if(m_CM730->m_BulkReadData[CM730::ID_CM].error == 0)
            {
                // fb_gyro_array[buf_idx] = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L);
                // rl_gyro_array[buf_idx] = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L);
                fb_gyro_array[buf_idx] = gyroZ;
                rl_gyro_array[buf_idx] = gyroY;
                
                buf_idx++;
            }
        }
        else
        {
            double fb_sum = 0.0, rl_sum = 0.0;
            double fb_sd = 0.0, rl_sd = 0.0;
            double fb_diff, rl_diff;
            double fb_mean = 0.0, rl_mean = 0.0;

            buf_idx = 0;

            for(int i = 0; i < GYRO_WINDOW_SIZE; i++)
            {
                fb_sum += fb_gyro_array[i];
                rl_sum += rl_gyro_array[i];
            }
            fb_mean = fb_sum / GYRO_WINDOW_SIZE;
            rl_mean = rl_sum / GYRO_WINDOW_SIZE;

            fb_sum = 0.0; rl_sum = 0.0;
            for(int i = 0; i < GYRO_WINDOW_SIZE; i++)
            {
                fb_diff = fb_gyro_array[i] - fb_mean;
                rl_diff = rl_gyro_array[i] - rl_mean;
                fb_sum += fb_diff * fb_diff;
                rl_sum += rl_diff * rl_diff;
            }
            fb_sd = sqrt(fb_sum / GYRO_WINDOW_SIZE);
            rl_sd = sqrt(rl_sum / GYRO_WINDOW_SIZE);

            if(fb_sd < MARGIN_OF_SD && rl_sd < MARGIN_OF_SD)
            {
                m_FBGyroCenter = (int)fb_mean;
                m_RLGyroCenter = (int)rl_mean;
                m_CalibrationStatus = 1;
                if(DEBUG_PRINT == true)
                    fprintf(stderr, "FBGyroCenter:%d , RLGyroCenter:%d \n", m_FBGyroCenter, m_RLGyroCenter);
            }
            else
            {
                m_FBGyroCenter = 512;
                m_RLGyroCenter = 512;
                m_CalibrationStatus = -1;
            }
        }
    }

    if(m_CalibrationStatus == 1 && m_Enabled == true)
    {
        static int fb_array[ACCEL_WINDOW_SIZE] = {512,};
        static int buf_idx = 0;
        if(m_CM730->m_BulkReadData[CM730::ID_CM].error == 0)
        {

            // MotionStatus::FB_GYRO = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L) - m_FBGyroCenter;
            // MotionStatus::RL_GYRO = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L) - m_RLGyroCenter;
            // MotionStatus::RL_ACCEL = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L);
            // MotionStatus::FB_ACCEL = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L);

            MotionStatus::FB_GYRO = gyroZ - m_FBGyroCenter;
            MotionStatus::RL_GYRO = gyroY - m_RLGyroCenter;
            MotionStatus::RL_ACCEL = kalmanRLaccel;
            MotionStatus::FB_ACCEL = kalmanFBaccel;
			
			outputFileFBaccel << MotionStatus::FB_ACCEL << endl;
			outputFileRLaccel << MotionStatus::RL_ACCEL << endl;;
			outputFileGyroZ << MotionStatus::FB_GYRO << endl;
			outputFileGyroY << MotionStatus::RL_GYRO << endl;	
			outputFileGyroX << gyroX << endl;
			outputFileCompass << kalmanCompass << endl;
			
            fb_array[buf_idx] = MotionStatus::FB_ACCEL;
            if(++buf_idx >= ACCEL_WINDOW_SIZE) buf_idx = 0;
        }

        int sum = 0, avr = 512;
        for(int idx = 0; idx < ACCEL_WINDOW_SIZE; idx++)
            sum += fb_array[idx];
        avr = sum / ACCEL_WINDOW_SIZE;

        // if(avr < MotionStatus::FALLEN_F_LIMIT)
        
        if(MotionStatus::FB_ACCEL < MotionStatus::FALLEN_F_LIMIT && MotionStatus::FB_ACCEL > MotionStatus::FALLEN_F_LIMIT - 10)
        {   
            // int it=0;
            // while(MotionStatus::FB_ACCEL < MotionStatus::FALLEN_F_LIMIT){          
            //     it++;
            //     printf("\n it : %d\n",it);
            //     if(it > MotionStatus::FALLEN_MAX_COUNT){
                MotionStatus::FALLEN = FORWARD;
            //     break;
            //     }
            // }
        }
        // else if(avr > MotionStatus::FALLEN_B_LIMIT)
        else if(MotionStatus::FB_ACCEL > MotionStatus::FALLEN_B_LIMIT && MotionStatus::FB_ACCEL < MotionStatus::FALLEN_B_LIMIT + 10)
        {   
            // int it=0;
            // while(MotionStatus::FB_ACCEL > MotionStatus::FALLEN_B_LIMIT){
            //     it++;
            //     printf("\n it : %d\n",it);
            //     if(it > MotionStatus::FALLEN_MAX_COUNT){
                MotionStatus::FALLEN = BACKWARD;
            //     break;
            //     }
            // }        
        }
        else
            MotionStatus::FALLEN = STANDUP;

        // printf("Nilai FB ACCELL : %d \n",MotionStatus::FB_ACCEL);
        // printf("Nilai RL ACCELL : %d \n",MotionStatus::RL_ACCEL);
        // printf("Nilai FB GYRO : %d \n",MotionStatus::FB_GYRO);
        // printf("Nilai RL GYRO : %d \n",MotionStatus::RL_GYRO);
        // printf("Nilai TOMBOL 1 : %d \n",MotionStatus::TOMBOL_1);
        // printf("Nilai TOMBOL 2 : %d \n",MotionStatus::TOMBOL_2);
        // printf("Nilai TOMBOL 3 : %d \n",MotionStatus::TOMBOL_3);
        // printf("Nilai COMPASS : %d \n",MotionStatus::Compass);


        if(m_Modules.size() != 0)
        {
            for(std::list<MotionModule*>::iterator i = m_Modules.begin(); i != m_Modules.end(); i++)
            {
                (*i)->Process();
                for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
                {
                    if((*i)->m_Joint.GetEnable(id) == true)
                    {
                        MotionStatus::m_CurrentJoints.SetSlope(id, (*i)->m_Joint.GetCWSlope(id), (*i)->m_Joint.GetCCWSlope(id));
                        MotionStatus::m_CurrentJoints.SetValue(id, (*i)->m_Joint.GetValue(id));

                        MotionStatus::m_CurrentJoints.SetPGain(id, (*i)->m_Joint.GetPGain(id));
                        MotionStatus::m_CurrentJoints.SetIGain(id, (*i)->m_Joint.GetIGain(id));
                        MotionStatus::m_CurrentJoints.SetDGain(id, (*i)->m_Joint.GetDGain(id));
                    }
                }
            }
        }

//////////////////////////////////////////////////////////////weeeeeeeeeeeeeeeeeww///////////////

        int param[( JointData::NUMBER_OF_JOINTS - 18 ) * MX28::PARAM_BYTES];
        int n = 0;
        int joint_num = 0;
        for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
        {
            if(id>=1&&id<=18)continue;

            if(MotionStatus::m_CurrentJoints.GetEnable(id) == true)
            {
                param[n++] = id;
            
                param[n++] = MotionStatus::m_CurrentJoints.GetCWSlope(id);
                param[n++] = MotionStatus::m_CurrentJoints.GetCCWSlope(id);
            
                param[n++] = CM730::GetLowByte(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id]);
                param[n++] = CM730::GetHighByte(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id]);
                joint_num++;
            }

            if(DEBUG_PRINT == true)
                fprintf(stderr, "ID[%d] : %d \n", id, MotionStatus::m_CurrentJoints.GetValue(id));
        }

        m_CM730->SyncWrite(MX28::P_CW_COMPLIANCE_SLOPE, MX28::PARAM_BYTES, joint_num, param);

        //////////////////////

        int parammx[ 18 * MX28::MXPARAM_BYTES];
        n = 0;
        joint_num = 0;
        for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
        {
            if(id>=1&&id<=18)
            {

                if(MotionStatus::m_CurrentJoints.GetEnable(id) == true)
                {
                    parammx[n++] = id;

                    parammx[n++] = MotionStatus::m_CurrentJoints.GetDGain(id);
                    parammx[n++] = MotionStatus::m_CurrentJoints.GetIGain(id);
                    parammx[n++] = MotionStatus::m_CurrentJoints.GetPGain(id);
                    parammx[n++] = 0;
            
                    parammx[n++] = CM730::GetLowByte(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id]);
                    parammx[n++] = CM730::GetHighByte(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id]);
                    joint_num++;
                }

                if(DEBUG_PRINT == true)
                    fprintf(stderr, "ID[%d] : %d \n", id, MotionStatus::m_CurrentJoints.GetValue(id));
            }
            else continue;
        }

        m_CM730->SyncWrite(MX28::MXP_D_GAIN, MX28::MXPARAM_BYTES, joint_num, parammx);

//////////////////////////////////////////wweeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeew/////////

    }

    m_CM730->BulkRead();

    if(m_IsLogging)
    {
        for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++)
            m_LogFileStream << MotionStatus::m_CurrentJoints.GetValue(id) << "," << m_CM730->m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L) << ",";

        m_LogFileStream << m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L) << ",";
        m_LogFileStream << m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L) << ",";
        m_LogFileStream << m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L) << ",";
        m_LogFileStream << m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L) << ",";
        m_LogFileStream << m_CM730->m_BulkReadData[FSR::ID_L_FSR].ReadByte(FSR::P_FSR_X) << ",";
        m_LogFileStream << m_CM730->m_BulkReadData[FSR::ID_L_FSR].ReadByte(FSR::P_FSR_Y) << ",";
        m_LogFileStream << m_CM730->m_BulkReadData[FSR::ID_R_FSR].ReadByte(FSR::P_FSR_X) << ",";
        m_LogFileStream << m_CM730->m_BulkReadData[FSR::ID_R_FSR].ReadByte(FSR::P_FSR_Y) << ",";
        m_LogFileStream << std::endl;
    }

    if(m_CM730->m_BulkReadData[CM730::ID_CM].error == 0)
        MotionStatus::BUTTON = m_CM730->m_BulkReadData[CM730::ID_CM].ReadByte(CM730::P_BUTTON);

    m_IsRunning = false;
}

void MotionManager::SetEnable(bool enable)
{
    m_Enabled = enable;
    if(m_Enabled == true)
        m_CM730->WriteWord(CM730::ID_BROADCAST, MX28::P_MOVING_SPEED_L, 0, 0);
}
*/
void MotionManager::AddModule(MotionModule *module)
{
    module->Initialize();
    m_Modules.push_back(module);
}

void MotionManager::RemoveModule(MotionModule *module)
{
    m_Modules.remove(module);
}

void MotionManager::SetJointDisable(int index)
{
    if(m_Modules.size() != 0)
    {
        for(std::list<MotionModule*>::iterator i = m_Modules.begin(); i != m_Modules.end(); i++)
            (*i)->m_Joint.SetEnable(index, false);
    }
}
