/*
 *   BallFollower.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdio.h>
#include "ImgProcess.h"
#include "MX28.h"
#include "Head.h"
#include "Action.h"
#include "Walking.h"
#include "BallFollower.h"
#include "MotionStatus.h"
#include "SerialHandler.h"


using namespace Robot;


BallFollower::BallFollower()
{
	m_NoBallMaxCount = 10;
	m_NoBallCount = m_NoBallMaxCount;
m_KickBallMaxCount = 10;
	m_KickBallCount = 0;

	m_KickTopAngle = 0.0;
	m_KickRightAngle = -30.0;
	m_KickLeftAngle = 30.0;

	m_FollowMaxFBStep = 20.0;
    m_FollowMinFBStep = 2.0;
	m_FollowMaxRLTurn = 25.0;
	m_FitFBStep = 3.0;
	m_FitMaxRLTurn = 25.0;
	m_UnitFBStep = 0.2;
	m_UnitRLTurn = 0.8;

	m_GoalFBStep = 0;
	m_GoalRLTurn = 0;
	m_FBStep = 0;
	m_RLTurn = 0;
	DEBUG_PRINT = true;
	KickBall = 0;
}

BallFollower::~BallFollower()
{
}

void BallFollower::Process(Point2D ball_pos)
{
	if(DEBUG_PRINT == true)
		fprintf(stderr, "\r                                                                               \r");

    if(ball_pos.X == -1.0 || ball_pos.Y == -1.0)
    {
		KickBall = 0;

		if(m_NoBallCount > m_NoBallMaxCount)
		{
			// can not find a ball
			m_GoalFBStep = 0;
			m_GoalRLTurn = 0;
			Head::GetInstance()->MoveToHome();

			if(DEBUG_PRINT == true)
				fprintf(stderr, "[NO BALL]");
		}
		else
		{
			m_NoBallCount++;
			if(DEBUG_PRINT == true)
				fprintf(stderr, "[NO BALL COUNTING(%d/%d)]", m_NoBallCount, m_NoBallMaxCount);
		}
    }
    else
    {
		m_NoBallCount = 0;		

		//double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
		double pan = Head::GetInstance()->GetPanAngle();
		double pan_range = Head::GetInstance()->GetLeftLimitAngle();
		double pan_percent = pan / pan_range;

		//double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
		double tilt = Head::GetInstance()->GetTiltAngle();
		double tilt_min = Head::GetInstance()->GetBottomLimitAngle();		
		double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
		double tilt_percent = (tilt - tilt_min) / tilt_range;
		if(tilt_percent < 0)
			tilt_percent = -tilt_percent;

		if(pan > m_KickRightAngle && pan < m_KickLeftAngle)
		{
			if(tilt <= (tilt_min + MX28::RATIO_VALUE2ANGLE))
			{
				if(ball_pos.Y < m_KickTopAngle)
				{
					m_GoalFBStep = 0;
					m_GoalRLTurn = 0;

					if(m_KickBallCount >= m_KickBallMaxCount)
					{
						m_FBStep = 0;
						m_RLTurn = 0;						
						if(DEBUG_PRINT == true)
							fprintf(stderr, "[KICK]");

						if(pan > 0)
						{
							KickBall = 1; // Left
							if(DEBUG_PRINT == true)
								fprintf(stderr, " Left");
						}
						else
						{
							KickBall = -1; // Right
							if(DEBUG_PRINT == true)
								fprintf(stderr, " Right");
						}
					}
					else
					{
						KickBall = 0;
						if(DEBUG_PRINT == true)
							fprintf(stderr, "[KICK COUNTING(%d/%d)]", m_KickBallCount, m_KickBallMaxCount);
					}
				}
				else
				{
					m_KickBallCount = 0;
					KickBall = 0;
					m_GoalFBStep = m_FitFBStep;
					m_GoalRLTurn = m_FitMaxRLTurn * pan_percent;
					if(DEBUG_PRINT == true)
						fprintf(stderr, "[FIT(P:%.2f T:%.2f>%.2f)]", pan, ball_pos.Y, m_KickTopAngle);
				}
			}
			else
			{
				m_KickBallCount = 0;
				KickBall = 0;
				m_GoalFBStep = m_FollowMaxFBStep * tilt_percent;
				if(m_GoalFBStep < m_FollowMinFBStep)
				    m_GoalFBStep = m_FollowMinFBStep;
				m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
				if(DEBUG_PRINT == true)
					fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
			}
		}
		else
		{
			m_KickBallCount = 0;
			KickBall = 0;
			m_GoalFBStep = 0;
			m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
			if(DEBUG_PRINT == true)
				fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
		}		
	}

	if(m_GoalFBStep == 0 && m_GoalRLTurn == 0 && m_FBStep == 0 && m_RLTurn > -1 && m_RLTurn < 1)
	{
		if(SerialHandler::GetInstance()->IsRunning() == true)
			SerialHandler::GetInstance()->WalkingStop();
		else
		{
			if(m_KickBallCount < m_KickBallMaxCount)
				m_KickBallCount++;
		}

		if(DEBUG_PRINT == true)
			fprintf(stderr, " STOP");
	}
	else
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, " START");

		if(SerialHandler::GetInstance()->IsRunning() == false)
		{
			m_FBStep = 0;
			m_RLTurn = 0;
			m_KickBallCount = 0;
			KickBall = 0;
			SerialHandler::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
			SerialHandler::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
			SerialHandler::GetInstance()->WalkingStart();			
		}
		else
		{
			if(m_FBStep < m_GoalFBStep)
				m_FBStep += m_UnitFBStep;
			else if(m_FBStep > m_GoalFBStep)
				m_FBStep = m_GoalFBStep;
			SerialHandler::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;

			if(m_RLTurn < m_GoalRLTurn)
				m_RLTurn += m_UnitRLTurn;
			else if(m_RLTurn > m_GoalRLTurn)
				m_RLTurn -= m_UnitRLTurn;
			SerialHandler::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;

			if(DEBUG_PRINT == true)
				fprintf(stderr, " (FB:%.1f RL:%.1f)", m_FBStep, m_RLTurn);
		}
	}	
}
