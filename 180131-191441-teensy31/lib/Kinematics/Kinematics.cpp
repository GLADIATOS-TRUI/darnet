/*
 *   Kinematics.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <math.h>
#include "Kinematics.h"


using namespace Robot;

const double Kinematics::CAMERA_DISTANCE = 45.0; //mm
const double Kinematics::EYE_TILT_OFFSET_ANGLE = 40.0;//45.0; //degree
const double Kinematics::LEG_SIDE_OFFSET = 48.75;//48.7; //mm
const double Kinematics::THIGH_LENGTH = 124.0; //mm
const double Kinematics::CALF_LENGTH = 120.0; //mm
const double Kinematics::ANKLE_LENGTH = 60.0; //mm
const double Kinematics::LEG_LENGTH = 304.0; //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)

Kinematics* Kinematics::m_UniqueInstance = new Kinematics();

Kinematics::Kinematics()
{
}

Kinematics::~Kinematics()
{
}
