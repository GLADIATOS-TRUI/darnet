/*
 *   MotionStatus.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include "MotionStatus.h"

using namespace Robot;

JointData MotionStatus::m_CurrentJoints;
int MotionStatus::FB_GYRO(0);
int MotionStatus::RL_GYRO(0);
int MotionStatus::FB_ACCEL(0);
int MotionStatus::RL_ACCEL(0);

int MotionStatus::BUTTON(0);
int MotionStatus::FALLEN(0);

int MotionStatus::TOMBOL_1(0);
int MotionStatus::TOMBOL_2(0);
int MotionStatus::TOMBOL_3(0);

int MotionStatus::Compass(0);
