/*
 *   MotionManager.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _MOTION_MANGER_H_
#define _MOTION_MANGER_H_

#include <list>
//#include <fstream>
//#include <iostream>
#include "MotionStatus.h"
//start
/*
#ifndef _MOTION_STATUS_H_
#define _MOTION_STATUS_H_

#include "JointData.h"


namespace Robot
{
    enum {
        BACKWARD    = -1,
        STANDUP     = 0,
        FORWARD     = 1
    };

	class MotionStatus
	{
	private:

	public:
	    static const int FALLEN_F_LIMIT     = 950;
	    static const int FALLEN_B_LIMIT     = 120;
	    static const int FALLEN_MAX_COUNT   = 100;

		static JointData m_CurrentJoints;
		static int FB_GYRO;
		static int RL_GYRO;
		static int FB_ACCEL;
		static int RL_ACCEL;

		static int BUTTON;
		static int FALLEN;
		
		static int TOMBOL_1;
		static int TOMBOL_2;
		static int TOMBOL_3;
		
		static int Compass;		
	};
}

#endif
//end
*/
#include "MotionModule.h"
/*
//start2
#ifndef _MOTION_MODULE_H_
#define _MOTION_MODULE_H_

#include "JointData.h"

namespace Robot
{
	class MotionModule
	{
	private:

	protected:

	public:
		JointData m_Joint;

		static const int TIME_UNIT = 8; //msec vda

		virtual void Initialize() = 0;
		virtual void Process() = 0;
	};
}

#endif
*/
//end2
//#include "CM730.h"
//#include "minIni.h"

#define OFFSET_SECTION "Offset"
#define INVALID_VALUE   -1024.0

namespace Robot
{
	class MotionManager
	{
	private:
		static MotionManager* m_UniqueInstance;
		std::list<MotionModule*> m_Modules;
		//CM730 *m_CM730;
		bool m_ProcessEnable;
		bool m_Enabled;
		int m_FBGyroCenter;
		int m_RLGyroCenter;
		int m_CalibrationStatus;

		bool m_IsRunning;
		bool m_IsThreadRunning;
		bool m_IsLogging;

		//std::ofstream m_LogFileStream;

        MotionManager();

	protected:

	public:
		bool DEBUG_PRINT;
        int m_Offset[JointData::NUMBER_OF_JOINTS];

		~MotionManager();

		static MotionManager* GetInstance() { return m_UniqueInstance; }

		//bool Initialize(CM730 *cm730);
		//bool Reinitialize();
        //void Process();
		//void SetEnable(bool enable);
		//bool GetEnable()				{ return m_Enabled; }
		void AddModule(MotionModule *module);
		void RemoveModule(MotionModule *module);

		//void ResetGyroCalibration() { m_CalibrationStatus = 0; m_FBGyroCenter = 512; m_RLGyroCenter = 512; }
		//int GetCalibrationStatus() { return m_CalibrationStatus; }
		void SetJointDisable(int index);

		//void StartLogging();
		//void StopLogging();

        //void LoadINISettings(minIni* ini);
        //void LoadINISettings(minIni* ini, const std::string &section);
        //void SaveINISettings(minIni* ini);
        //void SaveINISettings(minIni* ini, const std::string &section);
	};
}

#endif
