#include "MultiBody.h"

#include "Engine/Math/sVector.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>

eae6320::MultiBody::MultiBody(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, std::vector<GameCommon::GameObject *> & i_linkBodys):
	GameCommon::GameObject(i_pEffect, i_Mesh, i_State)
{
	m_linkBodys = i_linkBodys;
	w.resize(numOfLinks);
	A.resize(numOfLinks);
	A_dot.resize(numOfLinks);
	B.resize(numOfLinks);
	B_dot.resize(numOfLinks);
	C.resize(numOfLinks);
	C_dot.resize(numOfLinks);

	for (size_t i = 0; i < numOfLinks; i++)
	{
		MatrixXf M_d;
		M_d.resize(6, 6);
		M_d.setZero();
		M_d(0, 0) = rigidBodyMass;
		M_d(1, 1) = rigidBodyMass;
		M_d(2, 2) = rigidBodyMass;
		M_ds.push_back(M_d);
		
		Matrix3f localInertiaTensor;
		localInertiaTensor.setIdentity();
		localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
		localInertiaTensors.push_back(localInertiaTensor);

		std::vector<Vector3f> uPairs;
		uPairs.resize(2);
		uPairs[0] = Vector3f(-1.0f, 1.0f, 1.0f); //0 stores u for joint connecting to parent
		if (i == numOfLinks - 1)
		{
			uPairs[1] = Vector3f(0.0f, 0.0f, 0.0f);
		}
		else
		{
			uPairs[1] = -uPairs[0]; //1 stores u for joint connecting to child
		}
		uLocals.push_back(uPairs);
		uGlobals.push_back(uPairs);
	}

	R_dot.resize(3 * numOfLinks);
	R_dot.setZero();
	R.resize(3 * numOfLinks);
	R.setZero();

	physicsStateUpdate();
}

void eae6320::MultiBody::Tick(const float i_secondCountToIntegrate)
{
	std::vector<MatrixXf> H;
	H.resize(numOfLinks);
	std::vector<MatrixXf> D;
	D.resize(numOfLinks);
	std::vector<VectorXf> gamma;
	gamma.resize(numOfLinks);

	for (size_t i = 0; i < numOfLinks; i++)
	{
		Vector3f r = R.segment(i * 3, 3);
		
		float theta = r.norm();
		float a = A[i];
		float b = B[i];
		float c = C[i];

		//compute H
		Matrix3f J;
		J.setZero();
		Matrix3f rrt = r * r.transpose();
		J = a * MatrixXf::Identity(3, 3) + b * Math::ToSkewSymmetricMatrix(r) + c * rrt;
		H[i].resize(6, 3);
		H[i].setZero();
		H[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * J;
		H[i].block<3, 3>(3, 0) = J;

		//compute D
		if (i > 0)
		{
			D[i].resize(6, 6);
			D[i].setIdentity();
			D[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) - Math::ToSkewSymmetricMatrix(uGlobals[i - 1][1]);
		}

		//compute gamma
		Vector3f r_dot = R_dot.segment(i * 3, 3);
		float a_dot = A_dot[i];
		float b_dot = B_dot[i];
		float c_dot = C_dot[i];
		
		Vector3f gamma_theta;
		gamma_theta = (c * r.dot(r_dot) + a_dot) * r_dot - (b_dot * r_dot).cross(r) + (c_dot * r.dot(r_dot) + c * r_dot.norm() * r_dot.norm()) * r;
		
		gamma[i].resize(6);
		gamma[i].setZero();
		if (i == 0)
		{	
			gamma[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * gamma_theta - w[i].cross(w[i].cross(uGlobals[i][0]));	
		}
		else
		{
			Vector3f gamma_R;
			gamma_R = -w[i].cross(w[i].cross(uGlobals[i][0])) + w[i - 1].cross(w[i - 1].cross(uGlobals[i - 1][1]));
			gamma[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * gamma_theta + gamma_R;
		}
		gamma[i].block<3, 1>(3, 0) = gamma_theta;
	}
/**********************************************************************************************************/
	std::vector<MatrixXf> H_t;
	H_t.resize(numOfLinks);
	std::vector<VectorXf> gamma_t;
	gamma_t.resize(numOfLinks);
	for (size_t i = 0; i < numOfLinks; i++)
	{
		//compose Ht
		H_t[i].resize(6, 3 * numOfLinks);
		H_t[i].setZero();
		for (size_t k = 0; k <= i; k++)
		{
			MatrixXf H_temp;
			H_temp.resize(6, 3);
			H_temp = H[k];
			for (size_t j = k + 1; j <= i; j++)
			{
				H_temp = D[j] * H_temp;
			}
			H_t[i].block<6, 3>(0, 3 * k) = H_temp;
		}

		//compose gamma_t
		gamma_t[i].resize(6);
		gamma_t[i].setZero();
		for (size_t j = 0; j <= i; j++)
		{
			VectorXf gamma_temp;
			gamma_temp.resize(6);
			gamma_temp = gamma[j];
			for (size_t k = j + 1; k <= i; k++)
			{
				gamma_temp = D[k] * gamma_temp;
			}
			gamma_t[i] = gamma_t[i] + gamma_temp;
		}
	}
/**********************************************************************************************************/
	MatrixXf M_r;
	M_r.resize(3 * numOfLinks, 3 * numOfLinks);
	M_r.setZero();
	VectorXf Q_r;
	Q_r.resize(3 * numOfLinks);
	Q_r.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
		MatrixXf M_temp = H_t[i].transpose() * M_ds[i] * H_t[i];
		M_r = M_r + M_temp;

		VectorXf Fe;
		Fe.resize(6);
		Fe.setZero();
		Fe.block<3, 1>(0, 0) = Vector3f(0.0f, -9.81f, 0.0f);
		VectorXf Fv;
		Fv.resize(6);
		Fv.setZero();
		Fv.block<3, 1>(3, 0) = -w[i].cross(M_ds[i].block<3, 3>(3, 3) * w[i]);
		VectorXf Q_temp;
		Q_temp.resize(3 * numOfLinks);
		Q_temp.setZero();
		Q_temp = H_t[i].transpose() * (Fe + Fv - M_ds[i] * gamma_t[i]);
		Q_r = Q_r + Q_temp;
	}
/**********************************************************************************************************/
	VectorXf R_ddot = M_r.inverse() * Q_r;
	R_dot = R_dot + R_ddot * i_secondCountToIntegrate;
	R = R + R_dot * i_secondCountToIntegrate;

	//post check
	for (size_t i = 0; i < numOfLinks; i++)
	{
		Vector3f r = R.segment(i * 3, 3);
		if (r.norm() > 3.1f)
		{
			//R.segment(i * 3, 3) = R.segment(i * 3, 3) - M_PI * r.normalized();
			std::cout << "large r!" << std::endl;
		}
	}
	physicsStateUpdate();
}

void eae6320::MultiBody::UpdateGameObjectBasedOnInput()
{

}

void eae6320::MultiBody::physicsStateUpdate()
{
	Vector3f preAnchor(0.0f, 0.0f, 0.0f);
	for (size_t i = 0; i < numOfLinks; i++)
	{
		Vector3f r = R.segment(i * 3, 3);
		Vector3f r_dot = R_dot.segment(i * 3, 3);
		
		//update inertia tensor
		Matrix3f Rt;
		Rt = AngleAxisf(r.norm(), r.normalized());
		Matrix3f globalInertiaTensor;
		globalInertiaTensor = Rt * localInertiaTensors[i] * Rt.transpose();
		M_ds[i].block<3, 3>(3, 3) = globalInertiaTensor;

		//update orientation
		m_linkBodys[i]->m_State.orientation = Math::cQuaternion(r.norm(), Math::EigenVector2nativeVector(r.normalized()));
		m_linkBodys[i]->m_State.orientation.Normalize();
		
		//update position
		Vector3f uGlobal0 = Rt * uLocals[i][0];
		uGlobals[i][0] = uGlobal0;
		Vector3f linkPos = preAnchor - uGlobal0;
		m_linkBodys[i]->m_State.position = Math::sVector(linkPos(0), linkPos(1), linkPos(2));

		//update angular velocity
		float theta = r.norm();
		float a;
		if (theta < 0.015) a = 1.0f - theta / 6.0f;
		else a = sin(theta) / theta;
		A[i] = a;

		float b;
		if (theta < 0.015) b = 0.5f - theta * theta / 24.0f;
		else b = (1.0f - cos(theta)) / (theta * theta);
		B[i] = b;

		float c;
		if (theta < 0.015) c = 1.0f / 6.0f - theta * theta / 120.0f;
		else c = (1.0f - a) / (theta * theta);
		C[i] = c;

		float a_dot;
		if (theta < 0.001) a_dot = (-1.0f / 3.0f + 1.0f / 30.0f * theta * theta) * r.dot(r_dot);
		else a_dot = (c - b) * r.dot(r_dot);
		A_dot[i] = a_dot;

		float b_dot;
		if (theta < 0.001) b_dot = (-1.0f / 12.0f + 1.0f / 180.0f * theta * theta) * r.dot(r_dot);
		else b_dot = (a - 2.0f * b) / (theta * theta) * r.dot(r_dot);
		B_dot[i] = b_dot;

		float c_dot;
		if (theta < 0.001) c_dot = (-1.0f / 60.0f + 1.0f / 1260.0f * theta * theta) * r.dot(r_dot);
		else c_dot = (b - 3.0f * c) / (theta * theta) * r.dot(r_dot);
		C_dot[i] = c_dot;

		w[i] = a * r_dot - (b * r_dot).cross(r) + c * r.dot(r_dot) * r;
		
		//get ready for the next iteration
		Vector3f uGlobal1 = Rt * uLocals[i][1];
		uGlobals[i][1] = uGlobal1;
		preAnchor = linkPos + uGlobal1;
	}
}