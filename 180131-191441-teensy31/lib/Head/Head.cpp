/*
 *   Head.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdio.h>
#include <unistd.h>
#include "MX28.h"
#include "Kinematics.h"
#include "MotionStatus.h"
#include "Head.h"

using namespace Robot;


Head* Head::m_UniqueInstance = new Head();

Head::Head()
{
	m_Pan_p_gain = 0.1;
	m_Pan_d_gain = 0.1;

    m_Tilt_p_gain = 0.1;
	m_Tilt_d_gain = 0.1;

	m_LeftLimit = 70;
	m_RightLimit = -70;
	
	//di Kinematics.h... EYE_TILT_OFFSET_ANGLE = 40.0; //40 //degree
	m_TopLimit = Kinematics::EYE_TILT_OFFSET_ANGLE - 10.0; //30
	m_BottomLimit = Kinematics::EYE_TILT_OFFSET_ANGLE - 65.0; //65

	//posisi tengah
	m_Pan_Home = 0.0;
	m_Tilt_Home = Kinematics::EYE_TILT_OFFSET_ANGLE - 65.0; //30.0
	
	CariBolaPan = m_RightLimit;
	CariBolaTilt = m_TopLimit;

	m_Joint.SetEnableHeadOnly(true);
}

Head::~Head()
{
}

void Head::CheckLimit()
{
	if(m_PanAngle > m_LeftLimit)
		m_PanAngle = m_LeftLimit;
	else if(m_PanAngle < m_RightLimit)
		m_PanAngle = m_RightLimit;

	if(m_TiltAngle > m_TopLimit)
		m_TiltAngle = m_TopLimit;
	else if(m_TiltAngle < m_BottomLimit)
		m_TiltAngle = m_BottomLimit;	
}

void Head::Initialize()
{
	m_PanAngle = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
	m_TiltAngle = -MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
	CheckLimit();

	InitTracking();
	MoveToHome();
}
/*
void Head::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, HEAD_SECTION);
}

void Head::LoadINISettings(minIni* ini, const std::string &section)
{
    double value = INVALID_VALUE;

    if((value = ini->getd(section, "pan_p_gain", INVALID_VALUE)) != INVALID_VALUE)  m_Pan_p_gain = value;
    if((value = ini->getd(section, "pan_d_gain", INVALID_VALUE)) != INVALID_VALUE)  m_Pan_d_gain = value;
    if((value = ini->getd(section, "tilt_p_gain", INVALID_VALUE)) != INVALID_VALUE) m_Tilt_p_gain = value;
    if((value = ini->getd(section, "tilt_d_gain", INVALID_VALUE)) != INVALID_VALUE) m_Tilt_d_gain = value;
    if((value = ini->getd(section, "left_limit", INVALID_VALUE)) != INVALID_VALUE)  m_LeftLimit = value;
    if((value = ini->getd(section, "right_limit", INVALID_VALUE)) != INVALID_VALUE) m_RightLimit = value;
    if((value = ini->getd(section, "top_limit", INVALID_VALUE)) != INVALID_VALUE)   m_TopLimit = value;
    if((value = ini->getd(section, "bottom_limit", INVALID_VALUE)) != INVALID_VALUE)m_BottomLimit = value;
    if((value = ini->getd(section, "pan_home", INVALID_VALUE)) != INVALID_VALUE)    m_Pan_Home = value;
    if((value = ini->getd(section, "tilt_home", INVALID_VALUE)) != INVALID_VALUE)   m_Tilt_Home = value;
}

void Head::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, HEAD_SECTION);
}

void Head::SaveINISettings(minIni* ini, const std::string &section)
{
    ini->put(section,   "pan_p_gain",   m_Pan_p_gain);
    ini->put(section,   "pan_d_gain",   m_Pan_d_gain);
    ini->put(section,   "tilt_p_gain",  m_Tilt_p_gain);
    ini->put(section,   "tilt_d_gain",  m_Tilt_d_gain);
    ini->put(section,   "left_limit",   m_LeftLimit);
    ini->put(section,   "right_limit",  m_RightLimit);
    ini->put(section,   "top_limit",    m_TopLimit);
    ini->put(section,   "bottom_limit", m_BottomLimit);
    ini->put(section,   "pan_home",     m_Pan_Home);
    ini->put(section,   "tilt_home",    m_Tilt_Home);
}

*/
void Head::MoveToHome()
{
	MoveByAngle(m_Pan_Home, m_Tilt_Home);
}

void Head::MoveByAngle(double pan, double tilt)
{
	m_PanAngle = pan;
	m_TiltAngle = tilt;

	CheckLimit();
}

void Head::MoveByAngleOffset(double pan, double tilt)
{	
	MoveByAngle(m_PanAngle + pan, m_TiltAngle + tilt);
}

void Head::InitTracking()
{
	m_Pan_err = 0;
	m_Pan_err_diff = 0;
	m_Tilt_err = 0;
	m_Tilt_err_diff = 0;
}

void Head::MoveTracking(Point2D err)
{	
	m_Pan_err_diff = err.X - m_Pan_err;
	m_Pan_err = err.X;

	m_Tilt_err_diff = err.Y - m_Tilt_err;
	m_Tilt_err = err.Y;

	MoveTracking();
}

void Head::MoveTracking()
{
	double pOffset, dOffset;

	pOffset = m_Pan_err * m_Pan_p_gain;
	pOffset *= pOffset;
	if(m_Pan_err < 0)
		pOffset = -pOffset;
	dOffset = m_Pan_err_diff * m_Pan_d_gain;
	dOffset *= dOffset;
	if(m_Pan_err_diff < 0)
		dOffset = -dOffset;
	m_PanAngle += (pOffset + dOffset);

	pOffset = m_Tilt_err * m_Tilt_p_gain;
	pOffset *= pOffset;
	if(m_Tilt_err < 0)
		pOffset = -pOffset;
	dOffset = m_Tilt_err_diff * m_Tilt_d_gain;
	dOffset *= dOffset;
	if(m_Tilt_err_diff < 0)
		dOffset = -dOffset;
	m_TiltAngle += (pOffset + dOffset);

	CheckLimit();
}

void Head::Process()
{
	if(m_Joint.GetEnable(JointData::ID_HEAD_PAN) == true)
		m_Joint.SetAngle(JointData::ID_HEAD_PAN, m_PanAngle);

	if(m_Joint.GetEnable(JointData::ID_HEAD_TILT) == true)
		m_Joint.SetAngle(JointData::ID_HEAD_TILT, m_TiltAngle);
}

void Head::CariBola()
{
	MoveByAngle(CariBolaPan,CariBolaTilt);
	// printf("pan : %.4f tilt : %.4f \n",CariBolaPan,CariBolaTilt);
	CariBolaPan +=2.5;
	if (CariBolaPan >= m_LeftLimit){
		CariBolaPan = m_RightLimit;
		CariBolaTilt -=45;
		//printf("masuk pan");
		if(CariBolaTilt<=m_BottomLimit)CariBolaTilt = m_TopLimit;
	}
}

void Head::CariBolaAngguk()
{
	MoveByAngle(m_Pan_Home,CariBolaTilt);
	CariBolaTilt-=1.5;
	if(CariBolaTilt<=m_BottomLimit)CariBolaTilt = m_TopLimit;
}
