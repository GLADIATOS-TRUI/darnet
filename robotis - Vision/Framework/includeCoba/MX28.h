/*
 *   MX28.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _MX_28_H_
#define _MX_28_H_

//#define MX28_1024 //gak usah pake define, karena udah dimodif di file lainnya - MX dianggap 4096 dan AX dianggap 1024

namespace Robot
{
	class MX28
	{
	public:
		static const int MIN_VALUE = 0;
		static const int MXMIN_VALUE = 0;
//#ifdef MX28_1024
        static const int CENTER_VALUE = 512;
        static const int MAX_VALUE = 1023;
        static const double MIN_ANGLE = -150.0; // degree
        static const double MAX_ANGLE = 150.0; // degree
        static const double RATIO_VALUE2ANGLE = 0.293; // 300 / 1024
        static const double RATIO_ANGLE2VALUE = 3.413; // 1024 / 300

        static const int PARAM_BYTES = 5;

        static const int MXCENTER_VALUE = 2048;
        static const int MXMAX_VALUE = 4095;
        static const double MXMIN_ANGLE = -180.0; // degree
        static const double MXMAX_ANGLE = 180.0; // degree
        static const double MXRATIO_VALUE2ANGLE = 0.088; // 360 / 4096
        static const double MXRATIO_ANGLE2VALUE = 11.378; // 4096 / 360

        static const int MXPARAM_BYTES = 7;

        
//#else
		
//#endif

        static int GetMirrorValue(int value)		{ return MAX_VALUE + 1 - value; }
		static double GetMirrorAngle(double angle)	{ return -angle; }

		static int Angle2Value(double angle) { return (int)(angle*RATIO_ANGLE2VALUE)+CENTER_VALUE; }
        static double Value2Angle(int value) { return (double)(value-CENTER_VALUE)*RATIO_VALUE2ANGLE; }

        static int MXAngle2Value(double angle) { return (int)(angle*MXRATIO_ANGLE2VALUE)+MXCENTER_VALUE; }
        static double MXValue2Angle(int value) { return (double)(value-MXCENTER_VALUE)*MXRATIO_VALUE2ANGLE; }

		// Address
//#ifdef MX28_1024
        enum
        {
            P_MODEL_NUMBER_L            = 0,
            P_MODEL_NUMBER_H            = 1,
            P_VERSION                   = 2,
            P_ID                        = 3,
            P_BAUD_RATE                 = 4,
            P_RETURN_DELAY_TIME         = 5,
            P_CW_ANGLE_LIMIT_L          = 6,
            P_CW_ANGLE_LIMIT_H          = 7,
            P_CCW_ANGLE_LIMIT_L         = 8,
            P_CCW_ANGLE_LIMIT_H         = 9,
            P_HIGH_LIMIT_TEMPERATURE    = 11,
            P_LOW_LIMIT_VOLTAGE         = 12,
            P_HIGH_LIMIT_VOLTAGE        = 13,
            P_MAX_TORQUE_L              = 14,
            P_MAX_TORQUE_H              = 15,
            P_RETURN_LEVEL              = 16,
            P_ALARM_LED                 = 17,
            P_ALARM_SHUTDOWN            = 18,
            P_TORQUE_ENABLE             = 24,
            P_LED                       = 25,
            P_CW_COMPLIANCE_MARGIN      = 26,
            P_CCW_COMPLIANCE_MARGIN     = 27,
            P_CW_COMPLIANCE_SLOPE       = 28,
            P_CCW_COMPLIANCE_SLOPE      = 29,
            P_GOAL_POSITION_L           = 30,
            P_GOAL_POSITION_H           = 31,
            P_MOVING_SPEED_L            = 32,
            P_MOVING_SPEED_H            = 33,
            P_TORQUE_LIMIT_L            = 34,
            P_TORQUE_LIMIT_H            = 35,
            P_PRESENT_POSITION_L        = 36,
            P_PRESENT_POSITION_H        = 37,
            P_PRESENT_SPEED_L           = 38,
            P_PRESENT_SPEED_H           = 39,
            P_PRESENT_LOAD_L            = 40,
            P_PRESENT_LOAD_H            = 41,
            P_PRESENT_VOLTAGE           = 42,
            P_PRESENT_TEMPERATURE       = 43,
            P_REGISTERED_INSTRUCTION    = 44,
            P_MOVING                    = 46,
            P_LOCK                      = 47,
            P_PUNCH_L                   = 48,
            P_PUNCH_H                   = 49,
            MAXNUM_ADDRESS
        };
//#else
		enum
		{
			MXP_MODEL_NUMBER_L			= 0,
			MXP_MODEL_NUMBER_H			= 1,
			MXP_VERSION					= 2,
			MXP_ID						= 3,
			MXP_BAUD_RATE					= 4,
			MXP_RETURN_DELAY_TIME			= 5,			
			MXP_CW_ANGLE_LIMIT_L          = 6,
			MXP_CW_ANGLE_LIMIT_H          = 7,
			MXP_CCW_ANGLE_LIMIT_L         = 8,
			MXP_CCW_ANGLE_LIMIT_H         = 9,
			MXP_SYSTEM_DATA2              = 10, //////////
			MXP_HIGH_LIMIT_TEMPERATURE    = 11,
			MXP_LOW_LIMIT_VOLTAGE         = 12,
			MXP_HIGH_LIMIT_VOLTAGE        = 13,
			MXP_MAX_TORQUE_L              = 14,
			MXP_MAX_TORQUE_H              = 15,
			MXP_RETURN_LEVEL				= 16,
			MXP_ALARM_LED                 = 17,
			MXP_ALARM_SHUTDOWN            = 18,
			MXP_OPERATING_MODE            = 19, //
			MXP_LOW_CALIBRATION_L         = 20, //
			MXP_LOW_CALIBRATION_H         = 21, //
			MXP_HIGH_CALIBRATION_L        = 22, //
			MXP_HIGH_CALIBRATION_H        = 23, //
			MXP_TORQUE_ENABLE             = 24,
			MXP_LED                       = 25,
			MXP_D_GAIN                    = 26,
			MXP_I_GAIN                    = 27,
			MXP_P_GAIN                    = 28,
			MXP_RESERVED                  = 29,
			MXP_GOAL_POSITION_L           = 30,
			MXP_GOAL_POSITION_H           = 31,
			MXP_MOVING_SPEED_L            = 32,
			MXP_MOVING_SPEED_H            = 33,
			MXP_TORQUE_LIMIT_L            = 34,
			MXP_TORQUE_LIMIT_H            = 35,
			MXP_PRESENT_POSITION_L        = 36,
			MXP_PRESENT_POSITION_H        = 37,
			MXP_PRESENT_SPEED_L           = 38,
			MXP_PRESENT_SPEED_H           = 39,
			MXP_PRESENT_LOAD_L            = 40,
			MXP_PRESENT_LOAD_H            = 41,
			MXP_PRESENT_VOLTAGE           = 42,
			MXP_PRESENT_TEMPERATURE       = 43,
			MXP_REGISTERED_INSTRUCTION	= 44,
			MXP_PAUSE_TIME                = 45,
			MXP_MOVING					= 46,
			MXP_LOCK						= 47,
			MXP_PUNCH_L					= 48,
			MXP_PUNCH_H					= 49,
			MXP_RESERVED4                 = 50,
            MXP_RESERVED5                 = 51,
            MXP_POT_L                     = 52,
            MXP_POT_H                     = 53,
            MXP_PWM_OUT_L                 = 54,
            MXP_PWM_OUT_H                 = 55,
            MXP_P_ERROR_L                 = 56,
            MXP_P_ERROR_H                 = 57,
            MXP_I_ERROR_L                 = 58,
            MXP_I_ERROR_H                 = 59,
            MXP_D_ERROR_L                 = 60,
            MXP_D_ERROR_H                 = 61,
            MXP_P_ERROR_OUT_L             = 62,
            MXP_P_ERROR_OUT_H             = 63,
            MXP_I_ERROR_OUT_L             = 64,
            MXP_I_ERROR_OUT_H             = 65,
            MXP_D_ERROR_OUT_L             = 66,
            MXP_D_ERROR_OUT_H             = 67,
			MXMAXNUM_ADDRESS
		};
//#endif
	};
}

#endif
