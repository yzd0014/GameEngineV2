#include "MujocoBallJoint.h"
#include "Engine/Math/sVector.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>

eae6320::MujocoBallJoint::MujocoBallJoint(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, std::vector<GameCommon::GameObject *> & i_linkBodys, int i_numOfLinks):
	GameCommon::GameObject(i_pEffect, i_Mesh, i_State)
{
	numOfLinks = i_numOfLinks;
	m_linkBodys = i_linkBodys;
	w_global.resize(numOfLinks);
	M_ds.resize(numOfLinks);
	localInertiaTensors.resize(numOfLinks);
	m_orientations.resize(numOfLinks);
	for (size_t i = 0; i < numOfLinks; i++)
	{
		w_global[i].setZero();
		m_orientations[i].setIdentity();

		M_ds[i].resize(6, 6);
		M_ds[i].setZero();
		M_ds[i](0, 0) = rigidBodyMass;
		M_ds[i](1, 1) = rigidBodyMass;
		M_ds[i](2, 2) = rigidBodyMass;

		localInertiaTensors[i].setIdentity();
		localInertiaTensors[i] = localInertiaTensors[i] * (1.0f / 12.0f) * rigidBodyMass * 8.0;
		M_ds[i].block<3, 3>(3, 3) = localInertiaTensors[i];

		std::vector<Vector3f> uPairs;
		uPairs.resize(2);
		uPairs[0] = Vector3f(-1.0f, 1.0f, 1.0f);
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
	w_r.resize(3 * numOfLinks);
	w_r.setZero();
	ForwardKinematics();
}

void eae6320::MujocoBallJoint::Tick(const double i_secondCountToIntegrate)
{
	float dt = (float)i_secondCountToIntegrate;

	std::vector<MatrixXf> H;
	H.resize(numOfLinks);
	std::vector<MatrixXf> D;
	D.resize(numOfLinks);
	std::vector<VectorXf> gamma;
	gamma.resize(numOfLinks);

	for (size_t i = 0; i < numOfLinks; i++)
	{
		//compute H
		H[i].resize(6, 3);
		H[i].setZero();
		H[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]);
		H[i].block<3, 3>(3, 0) = MatrixXf::Identity(3, 3);

		//compute D
		if (i > 0)
		{
			D[i].resize(6, 6);
			D[i].setIdentity();
			D[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) - Math::ToSkewSymmetricMatrix(uGlobals[i - 1][1]);
		}

		//compute gamma
		gamma[i].resize(6);
		gamma[i].setZero();
		if (i == 0)
		{
			gamma[i].block<3, 1>(0, 0) = -w_global[i].cross(w_global[i].cross(uGlobals[i][0]));
		}
		else
		{
			gamma[i].block<3, 1>(0, 0) = -w_global[i].cross(w_global[i].cross(uGlobals[i][0])) + w_global[i - 1].cross(w_global[i - 1].cross(uGlobals[i - 1][1]));
		}
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
		Fv.block<3, 1>(3, 0) = -w_global[i].cross(M_ds[i].block<3, 3>(3, 3) * w_global[i]);
		VectorXf Q_temp;
		Q_temp.resize(3 * numOfLinks);
		Q_temp.setZero();
		Q_temp = H_t[i].transpose() * (Fe + Fv - M_ds[i] * gamma_t[i]);
		Q_r = Q_r + Q_temp;
	}
	/**********************************************************************************************************/
	VectorXf w_rdot = M_r.inverse() * Q_r;
	w_r = w_r + w_rdot * dt;
	for (size_t i = 0; i < numOfLinks; i++)
	{
		if (i == 0)
		{
			w_global[i] = w_r.segment(i * 3, 3);
		}
		else
		{
			w_global[i] = w_global[i - 1] + w_r.segment(i * 3, 3);
		}
		Vector3f deltaRotVec = w_global[i] * dt;
		Quaternionf deltaRot(AngleAxisf(deltaRotVec.norm(), deltaRotVec.normalized()));
		m_orientations[i] = deltaRot * m_orientations[i];
		m_orientations[i].normalize();
	}
	ForwardKinematics();
}

void eae6320::MujocoBallJoint::ForwardKinematics()
{
	Vector3f preAnchor(0.0f, 0.0f, 0.0f);
	for (size_t i = 0; i < numOfLinks; i++)
	{
		Matrix3f R_global = m_orientations[i].toRotationMatrix();
		
		//update orientation
		m_linkBodys[i]->m_State.orientation = Math::ConvertEigenQuatToNativeQuat(m_orientations[i]);

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
		MatrixXf globalInertiaTensor;
		globalInertiaTensor = R_global * localInertiaTensors[i] * R_global.transpose();
		M_ds[i].block<3, 3>(3, 3) = globalInertiaTensor;
	}

	if (tickCountSimulated <= 600010)
	{
		LOG_TO_FILE << tickCountSimulated << ", " << m_linkBodys[1]->m_State.position.x << ", " << -m_linkBodys[1]->m_State.position.z << ", " << m_linkBodys[1]->m_State.position.y << std::endl;
		if (tickCountSimulated == 600000)
		{
			std::cout << "done!" << std::endl;
		}
	}
	tickCountSimulated++;
}

