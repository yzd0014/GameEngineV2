#include "SphericalJointV2.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>

eae6320::SphericalJointV2::SphericalJointV2(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, std::vector<GameCommon::GameObject *> & i_linkBodys, int i_numOfLinks):
	GameCommon::GameObject(i_pEffect, i_Mesh, i_State)
{
	numOfLinks = i_numOfLinks;
	m_linkBodys = i_linkBodys;

	R.resize(numOfLinks);
	R_dot.resize(numOfLinks);
	R_ddot.resize(numOfLinks);
	V_dot.resize(numOfLinks);
	M.resize(numOfLinks);
	localInertiaTensors.resize(numOfLinks);
	Mr.resize(numOfLinks);
	Q.resize(numOfLinks);
	Qr.resize(numOfLinks);
	D.resize(numOfLinks);
	H.resize(numOfLinks);
	gamma.resize(numOfLinks);

	w_global.resize(numOfLinks);

	A.resize(numOfLinks);
	B.resize(numOfLinks);
	C.resize(numOfLinks);
	A_dot.resize(numOfLinks);
	B_dot.resize(numOfLinks);
	C_dot.resize(numOfLinks);

	for (int i = 0; i < numOfLinks; i++)
	{
		R[i].setZero();
		R_dot[i].setZero();
		V_dot[i].resize(6);
		V_dot[i].setZero();

		M[i].resize(6, 6);
		M[i].setZero();
		M[i](0, 0) = rigidBodyMass;
		M[i](1, 1) = rigidBodyMass;
		M[i](2, 2) = rigidBodyMass;
		localInertiaTensors[i].setIdentity();
		localInertiaTensors[i] = localInertiaTensors[i] * (1.0f / 12.0f) * rigidBodyMass * 8;
		M[i].block<3, 3>(3, 3) = localInertiaTensors[i];

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
	/*uLocals[0][1] = Vector3f(1.0f, -1.0f, 1.0f);
	uLocals[1][0] = Vector3f(-1.0f, 1.0f, -1.0f);*/
	ForwardKinematics();
}

void eae6320::SphericalJointV2::Tick(const double i_secondCountToIntegrate)
{
	float dt = (float)i_secondCountToIntegrate;
	
	for (int i = 0; i < numOfLinks; i++)
	{
		//compute D
		if (i > 0)
		{
			D[i].resize(6, 6);
			D[i].setIdentity();
			D[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) - Math::ToSkewSymmetricMatrix(uGlobals[i - 1][1]);
		}
		
		//compute H
		float a = A[i];
		float b = B[i];
		float c = C[i];
		Matrix3f J;
		J.setZero();
		Matrix3f rrt = R[i] * R[i].transpose();
		J = a * MatrixXf::Identity(3, 3) + b * Math::ToSkewSymmetricMatrix(R[i]) + c * rrt;
		//std::cout << J.determinant() << std::endl <<std::endl;
		H[i].resize(6, 3);
		H[i].setZero();
		H[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * J;
		H[i].block<3, 3>(3, 0) = J;
		
		//compute gamma
		float a_dot = A_dot[i];
		float b_dot = B_dot[i];
		float c_dot = C_dot[i];
		Vector3f gamma_theta;
		gamma_theta = (c * R[i].dot(R_dot[i]) + a_dot) * R_dot[i] - (b_dot * R_dot[i]).cross(R[i]) + (c_dot * R[i].dot(R_dot[i]) + c * R_dot[i].dot(R_dot[i])) * R[i];
		gamma[i].resize(6);
		gamma[i].setZero();
		if (i == 0)
		{
			gamma[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * gamma_theta - w_global[i].cross(w_global[i].cross(uGlobals[i][0]));
		}
		else
		{
			gamma[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * gamma_theta - w_global[i].cross(w_global[i].cross(uGlobals[i][0])) + w_global[i - 1].cross(w_global[i - 1].cross(uGlobals[i - 1][1]));
		}
		gamma[i].block<3, 1>(3, 0) = gamma_theta;

		//compute Q
		VectorXf Fe;
		Fe.resize(6);
		Fe.setZero();
		Fe.block<3, 1>(0, 0) = Vector3f(0.0f, -9.81f, 0.0f);
		VectorXf Fv;
		Fv.resize(6);
		Fv.setZero();
		Fv.block<3, 1>(3, 0) = -w_global[i].cross(M[i].block<3, 3>(3, 3) * w_global[i]);
		Q[i].resize(6);
		Q[i] = Fe + Fv;
	}

	for (int i = numOfLinks - 1; i >= 0; i--)
	{
		//compute Mr
		if (i == numOfLinks - 1)
		{
			Mr[i].resize(6, 6);
			Mr[i].setZero();
		}
		else
		{
			MatrixXf Mt = M[i + 1] + Mr[i + 1];
			Mr[i] = D[i + 1].transpose() * Mt * D[i + 1]
				- D[i + 1].transpose() * Mt * H[i + 1] * (H[i + 1].transpose() * Mt * H[i + 1]).inverse()
				* H[i + 1].transpose() * Mt * D[i + 1];
		}

		//compute Qr
		if (i == numOfLinks - 1)
		{
			Qr[i].resize(6);
			Qr[i].setZero();
		}
		else
		{
			MatrixXf Mt = M[i + 1] + Mr[i + 1];
			Qr[i] = -D[i + 1].transpose() * (Mt * gamma[i + 1] - (Q[i + 1] + Qr[i + 1]))
				+ D[i + 1].transpose() * Mt * H[i + 1] * (H[i + 1].transpose() * Mt * H[i + 1]).inverse()
				*(H[i + 1].transpose() * Mt * gamma[i + 1] - H[i + 1].transpose() * (Q[i + 1] + Qr[i + 1]));
		}
	}

	for (int i = 0; i < numOfLinks; i++)
	{
		MatrixXf Mt = M[i] + Mr[i];
		if (i == 0)
		{
			R_ddot[i] = -(H[i].transpose() * Mt * H[i]).inverse() *(H[i].transpose() * (Mt * gamma[i] - (Q[i] + Qr[i])));
		}
		else
		{
			R_ddot[i] = -(H[i].transpose() * Mt * H[i]).inverse() *(H[i].transpose() * (Mt * D[i] * V_dot[i - 1] + Mt * gamma[i] - (Q[i] + Qr[i])));
		}
		R_dot[i] = R_dot[i] + R_ddot[i] * dt;
		R[i] = R[i] + R_dot[i] * dt;

		/*if (i == 0)
		{
			V_dot[i] = H[i] * R_ddot[i] + gamma[i];
		}
		else
		{
			V_dot[i] = D[i] * V_dot[i - 1] + H[i] * R_ddot[i] + gamma[i];
		}*/
	}

	for (int i = 0; i < numOfLinks; i++)
	{
		if (i == 0)
		{
			V_dot[i] = H[i] * R_ddot[i] + gamma[i];
		}
		else
		{
			V_dot[i] = D[i] * V_dot[i - 1] + H[i] * R_ddot[i] + gamma[i];
		}
	}
	ForwardKinematics();
}

void eae6320::SphericalJointV2::ForwardKinematics()
{
	Vector3f preAnchor(0.0f, 0.0f, 0.0f);
	Matrix3f R_global;
	R_global.setIdentity();
	for (size_t i = 0; i < numOfLinks; i++)
	{
		Matrix3f R_local;
		R_local = AngleAxisf(R[i].norm(), R[i].normalized());
		R_global = R_global * R_local;
		AngleAxisf angleAxis_global(R_global);

		//update orientation
		m_linkBodys[i]->m_State.orientation = Math::cQuaternion(angleAxis_global.angle(), Math::EigenVector2nativeVector(angleAxis_global.axis()));
		m_linkBodys[i]->m_State.orientation.Normalize();

		//update position
		Vector3f uGlobal0 = R_global * uLocals[i][0];
		uGlobals[i][0] = uGlobal0;
		Vector3f linkPos = preAnchor - uGlobal0;
		m_linkBodys[i]->m_State.position = Math::sVector(linkPos(0), linkPos(1), linkPos(2));

		//get ready for the next iteration
		Vector3f uGlobal1 = R_global * uLocals[i][1];
		uGlobals[i][1] = uGlobal1;
		preAnchor = linkPos + uGlobal1;

		//update inertia tensor
		Matrix3f globalInertiaTensor;
		globalInertiaTensor = R_global * localInertiaTensors[i] * R_global.transpose();
		M[i].block<3, 3>(3, 3) = globalInertiaTensor;

		//update a b c
		float theta = R[i].norm();
		float a;
		if (theta < 0.0001) a = 1.0f - theta * theta / 6.0f;
		else a = sin(theta) / theta;
		A[i] = a;

		float b;
		if (theta < 0.0001) b = 0.5f - theta * theta / 24.0f;
		else b = (1.0f - cos(theta)) / (theta * theta);
		B[i] = b;

		float c;
		if (theta < 0.0001) c = 1.0f / 6.0f - theta * theta / 120.0f;
		else c = (1.0f - a) / (theta * theta);
		C[i] = c;

		float a_dot;
		if (theta < 0.0001) a_dot = (-1.0f / 3.0f + 1.0f / 30.0f * theta * theta) * R[i].dot(R_dot[i]);
		else a_dot = (c - b) * R[i].dot(R_dot[i]);
		A_dot[i] = a_dot;

		float b_dot;
		if (theta < 0.0001) b_dot = (-1.0f / 12.0f + 1.0f / 180.0f * theta * theta) * R[i].dot(R_dot[i]);
		else b_dot = (a - 2.0f * b) / (theta * theta) * R[i].dot(R_dot[i]);
		B_dot[i] = b_dot;

		float c_dot;
		if (theta < 0.0001) c_dot = (-1.0f / 60.0f + 1.0f / 1260.0f * theta * theta) * R[i].dot(R_dot[i]);
		else c_dot = (b - 3.0f * c) / (theta * theta) * R[i].dot(R_dot[i]);
		C_dot[i] = c_dot;

		Vector3f w_local = a * R_dot[i] - (b * R_dot[i]).cross(R[i]) + c * R[i].dot(R_dot[i]) * R[i];
		if (i == 0)
		{
			w_global[i] = w_local;
		}
		else
		{
			w_global[i] = w_global[i - 1] + w_local;
		}
	}
}