#include "doublePendulumBallJoint.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>

eae6320::doublePendulumBallJoint::doublePendulumBallJoint(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, std::vector<GameCommon::GameObject *> & i_linkBodys, int i_numOfLinks):
	GameCommon::GameObject(i_pEffect, i_Mesh, i_State)
{
	numOfLinks = i_numOfLinks;
	m_linkBodys = i_linkBodys;
	u_global.resize(numOfLinks);
	u_local = Vector3f(0, 4, 0);
	R_dot.resize(numOfLinks * 3);
	R_dot.setZero();
	R.resize(numOfLinks * 3);
	R.setZero();

	//R.segment(0, 3) = Vector3f(0.0f, 0.0f, 1.0f);//initial condition
	ForwardKinematics();
}

void eae6320::doublePendulumBallJoint::Tick(const double i_secondCountToIntegrate)
{
	float h = static_cast<float>(i_secondCountToIntegrate);

	std::vector<Matrix3f> J;
	J.resize(numOfLinks);
	std::vector<Vector3f> gamma_theta;
	gamma_theta.resize(2);
	//compute J and gamma_theta
	for (int i = 0; i < numOfLinks; i++)
	{
		Vector3f r = R.segment(i * 3, 3);
		Vector3f r_dot = R_dot.segment(i * 3, 3);
		float theta = r.norm();
		
		float a;
		if (theta < 0.0001) a = 1.0f - theta * theta / 6.0f;
		else a = sin(theta) / theta;
		
		float b;
		if (theta < 0.0001) b = 0.5f - theta * theta / 24.0f;
		else b = (1.0f - cos(theta)) / (theta * theta);

		float c;
		if (theta < 0.0001) c = 1.0f / 6.0f - theta * theta / 120.0f;
		else c = (1.0f - a) / (theta * theta);

		float a_dot;
		a_dot = (c - b) * r.dot(r_dot);

		float b_dot;
		if (theta < 0.0001) b_dot = (-1.0f / 12.0f + 1.0f / 180.0f * theta * theta) * r.dot(r_dot);
		else b_dot = (a - 2.0f * b) / (theta * theta) * r.dot(r_dot);

		float c_dot;
		if (theta < 0.0001) c_dot = (-1.0f / 60.0f + 1.0f / 1260.0f * theta * theta) * r.dot(r_dot);
		else c_dot = (b - 3.0f * c) / (theta * theta) * r.dot(r_dot);

		J[i] = a * MatrixXf::Identity(3, 3) + b * Math::ToSkewSymmetricMatrix(r) + c * (r * r.transpose());
		gamma_theta[i] = (c * r.dot(r_dot) + a_dot) * r_dot - b_dot * r_dot.cross(r) + (c_dot * r.dot(r_dot) + c * r_dot.dot(r_dot)) * r;
	}

	//update Ht
	std::vector<MatrixXf> Ht;
	Ht.resize(numOfLinks);
	Ht[0].resize(3, numOfLinks * 3);
	Ht[0].setZero();
	Ht[0].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(u_global[0]) * J[0];
	std::cout << Math::ToSkewSymmetricMatrix(u_global[0]) << std::endl << std::endl;
	std::cout << J[0] << std::endl << std::endl;
	std::cout << Ht[0] << std::endl << std::endl;
	std::cout << Ht[0].determinant() << std::endl << std::endl;
 	if (numOfLinks == 2)
	{
		Ht[1].resize(3, 6);
		Ht[1].block<3, 3>(0, 0) = (Math::ToSkewSymmetricMatrix(u_global[0]) + Math::ToSkewSymmetricMatrix(u_global[1])) * J[0];
		Ht[1].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(u_global[1]) * J[1];

	}
	
	//update gamma_t
	std::vector<Vector3f> w;
	w.resize(numOfLinks);
	w[0] = J[0] * R.segment(0, 3);
	if (numOfLinks == 2) w[1] = w[0] + J[1] * R.segment(3, 3);

	std::vector<Vector3f> gamma_t;
	gamma_t.resize(numOfLinks);
	gamma_t[0] = Math::ToSkewSymmetricMatrix(u_global[0]) * gamma_theta[0] - w[0].cross(w[0].cross(u_global[0]));
	if (numOfLinks == 2)
	{
		gamma_t[1] = (Math::ToSkewSymmetricMatrix(u_global[0]) + Math::ToSkewSymmetricMatrix(u_global[1])) * gamma_theta[0] +
			Math::ToSkewSymmetricMatrix(u_global[1]) * gamma_theta[1]
			- w[0].cross(w[0].cross(u_global[0])) - w[1].cross(w[1].cross(u_global[1]));
	}

	//update M_r
	MatrixXf Mr;
	Mr.resize(3 * numOfLinks, 3 * numOfLinks);
	Mr.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
		Mr = Mr + Ht[i].transpose() * MatrixXf::Identity(3, 3) * Ht[i];
		std::cout << Ht[i].transpose() * MatrixXf::Identity(3, 3) << std::endl << std::endl;
	}
	std::cout << Mr << std::endl << std::endl;
	std::cout << Mr.determinant() << std::endl << std::endl;
	//compute Q_r
	VectorXf Qr;
	Qr.resize(3 * numOfLinks);
	Qr.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
		Vector3f Fe(0.0f, -9.81f, 0.0f);
		Qr = Qr + Ht[i].transpose() * (Fe - MatrixXf::Identity(3, 3) * gamma_t[i]);
	}
	//std::cout << Mr.determinant() << std::endl << std::endl;
	//std::cout << Mr.inverse() << std::endl << std::endl;
	//integration
	R_dot = R_dot + Mr.inverse() * Qr * h;
	R = R + R_dot * h;
	std::cout << R_dot << std::endl << std::endl;
	std::cout << R << std::endl << std::endl;

	ForwardKinematics();
}

void eae6320::doublePendulumBallJoint::ForwardKinematics()
{
	Vector3f preAnchor(0.0f, 0.0f, 0.0f);
	Matrix3f R_global;
	R_global.setIdentity();
	for (int i = 0; i < numOfLinks; i++)
	{
		Vector3f r = R.segment(i * 3, 3);

		Matrix3f R_local;
		R_local = AngleAxisf(r.norm(), r.normalized());
		R_global = R_global * R_local;
		
		//update u
		u_global[i] = R_global * u_local;
		
		//update orientation
		AngleAxisf angleAxis_global(R_global);
		m_linkBodys[i]->m_State.orientation = Math::cQuaternion((float)angleAxis_global.angle(), Math::EigenVector2nativeVector(angleAxis_global.axis()));
		m_linkBodys[i]->m_State.orientation.Normalize();

		//update position
		Vector3f particle_position = preAnchor + R_global * (0.5f * -u_local);
		m_linkBodys[i]->m_State.position = Math::EigenVector2nativeVector(particle_position);
		if (i == 0)
		{
			preAnchor = particle_position + R_global * (0.5f * -u_local);
		}
	}
}
void eae6320::doublePendulumBallJoint::UpdateGameObjectBasedOnInput()
{

}