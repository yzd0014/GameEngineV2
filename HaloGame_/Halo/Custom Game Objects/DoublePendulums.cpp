#include <math.h>
#include "DoublePendulums.h"

void eae6320::DoublePendulums::Tick(const float i_secondCountToIntegrate)
{
	if (m_Mode == Kinematic)
	{
		q(0) = cos(t);//1.2f;
		q(1) = cos(t) + 1.0f;//0.4f;//
	}
	else
	{
		MatrixXf qbar;
		qbar.resize(2, 1);
		qbar.setZero();
		qbar(0) = cos(t); //1.2f;
		qbar(1) = cos(t) + 1.0f; //0.4f;

		MatrixXf J;
		J.resize(4, 2);
		J.setZero();
		J(0, 0) = r * cos(q(0));
		J(1, 0) = -r * sin(q(0));
		J(2, 0) = r * cos(q(0));
		J(2, 1) = r * cos(q(1));
		J(3, 0) = -r * sin(q(0));
		J(3, 1) = -r * sin(q(1));
		MatrixXf M1;
		M1.resize(4, 4);
		M1.setZero();
		for (int i = 0; i < 4; i++) 
		{
			M1(i, i) = m;
		}
		MatrixXf M;
		M.resize(2, 2);
		M = J.transpose() * M1 * J; 

		MatrixXf Jdot;
		Jdot.resize(4, 2);
		Jdot.setZero();
		Jdot(0, 0) = -r * sin(q(0)) * qdot(0);
		Jdot(1, 0) = -r * cos(q(0)) * qdot(0);
		Jdot(2, 0) = -r * sin(q(0)) * qdot(0);
		Jdot(2, 1) = -r * sin(q(1)) * qdot(1);
		Jdot(3, 0) = -r * cos(q(0)) * qdot(0);
		Jdot(3, 1) = -r * cos(q(1)) * qdot(1);
		MatrixXf C;
		C.resize(2, 1);
		C = J.transpose() * M1 * Jdot * qdot;

		float kp = 30000000;
		float kd = kp * i_secondCountToIntegrate * 1.2f;
		MatrixXf Kp;
		Kp.resize(2, 2);
		Kp.setZero();
		for (int i = 0; i < 2; i++)
		{
			Kp(i, i) = kp;
		}
		MatrixXf Kd;
		Kd.resize(2, 2);
		Kd.setZero();
		for (int i = 0; i < 2; i++)
		{
			Kd(i, i) = kd;
		}

		MatrixXf qddot;
		qddot.resize(2, 1);
		qddot.setZero();
		if (m_Mode == PD)
		{
			qddot = M.inverse() * (-C + Kp * (qbar - q) - Kd * qdot);
		}
		else if (m_Mode == SPD)
		{
			qddot = (M + Kd * i_secondCountToIntegrate).inverse() * (-C + Kp * (qbar - q - qdot * i_secondCountToIntegrate) - Kd * qdot);
		}
		//forward Euler Integration
		q = q + i_secondCountToIntegrate * qdot;
		qdot = qdot + i_secondCountToIntegrate * qddot;
	}

	t += i_secondCountToIntegrate;

	//coordiniate system translation
	float x1 = r * sin(q(0));
	float y1 = r * cos(q(0));
	m_State.orientation = Math::cQuaternion(q(0), Math::sVector(0, 0, 1));	
	m_State.position = anchor + Math::sVector(x1/2.0f, -y1/2.0f, 0.0f);
	
	
	float x2 = r * (sin(q(0)) + sin(q(1)));
	float y2 = r * (cos(q(0)) + cos(q(1)));
	p_SecondPendulum->m_State.orientation = Math::cQuaternion(q(1), Math::sVector(0, 0, 1));
	p_SecondPendulum->m_State.position = anchor + Math::sVector((x1 + x2) / 2.0f, (-y1 - y2) / 2.0f, 0.0f);
}