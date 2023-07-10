#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>

eae6320::MultiBody::MultiBody(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, std::vector<GameCommon::GameObject *> & i_linkBodys, int i_numOfLinks):
	GameCommon::GameObject(i_pEffect, i_Mesh, i_State)
{
	numOfLinks = i_numOfLinks;
	
	m_linkBodys = i_linkBodys;
	w_global.resize(numOfLinks);
	A.resize(numOfLinks);
	B.resize(numOfLinks);
	C.resize(numOfLinks);
	D.resize(numOfLinks);
	H_t.resize(numOfLinks);
	M_r.resize(3 * numOfLinks, 3 * numOfLinks);
	
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
	/*uLocals[0][1] = Vector3f(1.0f, -1.0f, 1.0f);
	uLocals[1][0] = Vector3f(-1.0f, 1.0f, -1.0f);*/
	
	//uLocals[0][0] = Vector3f(1.0f, -1.0f, -1.0f);

	R_dot.resize(3 * numOfLinks);
	R_dot.setZero();
	R.resize(3 * numOfLinks);
	R.setZero();
	//R.block<3, 1>(0, 0) = Vector3f(0.0f, 0.0f, -1.0f);

	ForwardKinematics();
}

void eae6320::MultiBody::Tick(const float i_secondCountToIntegrate)
{	
	std::vector<MatrixXf> H;
	H.resize(numOfLinks);
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
		//std::cout << J.determinant() << std::endl <<std::endl;
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
	}
/**********************************************************************************************************/
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
	}
/**********************************************************************************************************/
	M_r.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
		MatrixXf M_temp = H_t[i].transpose() * M_ds[i] * H_t[i];
		M_r = M_r + M_temp;
	}
/**********************************************************************************************************/
	EulerIntegration(i_secondCountToIntegrate);
	ForwardKinematics();
	
	//post check
	for (size_t i = 0; i < numOfLinks; i++)
	{
		Vector3f r = R.segment(i * 3, 3);
		if (r.norm() > 2 * M_PI - 0.001)
		{
			R.segment(i * 3, 3) = R.segment(i * 3, 3) - 2 * M_PI * r.normalized();
			std::cout << "large r!" << std::endl;
		}
		//std::cout << r.norm() << ", ";
	}
	//std::cout << std::endl;
}

void eae6320::MultiBody::EulerIntegration(const float h)
{
	VectorXf Q_r = ComputeQ_r(R_dot);
	VectorXf R_ddot = M_r.inverse() * Q_r;
	//std::cout << h * M_r.inverse() * Q_r << std::endl;
	R_dot = R_dot + R_ddot * h;
	R = R + R_dot * h;
}

void eae6320::MultiBody::RK4Integration(const float h)
{
	MatrixXf M_rInverse = M_r.inverse();
	
	VectorXf k1 = h * M_rInverse * ComputeQ_r(R_dot);
	VectorXf k2 = h * M_rInverse * ComputeQ_r(R_dot + 0.5 * k1);
	VectorXf k3 = h * M_rInverse * ComputeQ_r(R_dot + 0.5 * k2);
	VectorXf k4 = h * M_rInverse * ComputeQ_r(R_dot + k3);

	R_dot = R_dot + (1.0f / 6.0f) * (k1 + 2 * k2 + 2 * k3 + k4);
	R = R + R_dot * h;
}

VectorXf eae6320::MultiBody::ComputeQ_r(VectorXf i_R_dot)
{
	ComputeAngularVelocity(i_R_dot);
	std::vector<VectorXf> gamma_t;
	ComputeGamma_t(gamma_t, i_R_dot);

	VectorXf Q_r;
	Q_r.resize(3 * numOfLinks);
	Q_r.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
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

	return Q_r;
}

void eae6320::MultiBody::ComputeGamma_t(std::vector<VectorXf>& o_gamma_t, VectorXf& i_R_dot)
{
	std::vector<float> m_A_dot;
	std::vector<float> m_B_dot;
	std::vector<float> m_C_dot;
	ComputeAngularVelocityExpressionCoefficientDerivative(m_A_dot, m_B_dot, m_C_dot, i_R_dot);
	
	std::vector<VectorXf> gamma;
	gamma.resize(numOfLinks);
	for (size_t i = 0; i < numOfLinks; i++)
	{
		Vector3f r = R.segment(i * 3, 3);
		Vector3f r_dot = i_R_dot.segment(i * 3, 3);
		float a = A[i];
		float b = B[i];
		float c = C[i];
		float a_dot = m_A_dot[i];
		float b_dot = m_B_dot[i];
		float c_dot = m_C_dot[i];

		Vector3f gamma_theta;
		gamma_theta = (c * r.dot(r_dot) + a_dot) * r_dot - (b_dot * r_dot).cross(r) + (c_dot * r.dot(r_dot) + c * r_dot.dot(r_dot)) * r;

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
	}

	o_gamma_t.resize(numOfLinks);
	for (size_t i = 0; i < numOfLinks; i++)
	{
		o_gamma_t[i].resize(6);
		o_gamma_t[i].setZero();
		for (size_t j = 0; j <= i; j++)
		{
			VectorXf gamma_temp;
			gamma_temp.resize(6);
			gamma_temp = gamma[j];
			for (size_t k = j + 1; k <= i; k++)
			{
				gamma_temp = D[k] * gamma_temp;
			}
			o_gamma_t[i] = o_gamma_t[i] + gamma_temp;
		}
	}
}

void eae6320::MultiBody::ComputeAngularVelocity(VectorXf& i_R_dot)
{
	for (size_t i = 0; i < numOfLinks; i++)
	{
		Vector3f r = R.segment(i * 3, 3);
		Vector3f r_dot = i_R_dot.segment(i * 3, 3);

		float a = A[i];
		float b = B[i];
		float c = C[i];

		Vector3f w_local = a * r_dot - (b * r_dot).cross(r) + c * r.dot(r_dot) * r;
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

void eae6320::MultiBody::ComputeAngularVelocityExpressionCoefficientDerivative(std::vector<float>& o_A_dot, std::vector<float>& o_B_dot, std::vector<float>& o_C_dot, VectorXf& i_R_dot)
{
	o_A_dot.resize(numOfLinks);
	o_B_dot.resize(numOfLinks);
	o_C_dot.resize(numOfLinks);

	for (size_t i = 0; i < numOfLinks; i++)
	{
		Vector3f r = R.segment(i * 3, 3);
		Vector3f r_dot = i_R_dot.segment(i * 3, 3);
		float theta = r.norm();
		
		float a = A[i];
		float b = B[i];
		float c = C[i];

		float a_dot;
		if (theta < 0.0001) a_dot = (-1.0f / 3.0f + 1.0f / 30.0f * theta * theta) * r.dot(r_dot);
		else a_dot = (c - b) * r.dot(r_dot);
		o_A_dot[i] = a_dot;

		float b_dot;
		if (theta < 0.0001) b_dot = (-1.0f / 12.0f + 1.0f / 180.0f * theta * theta) * r.dot(r_dot);
		else b_dot = (a - 2.0f * b) / (theta * theta) * r.dot(r_dot);
		o_B_dot[i] = b_dot;

		float c_dot;
		if (theta < 0.0001) c_dot = (-1.0f / 60.0f + 1.0f / 1260.0f * theta * theta) * r.dot(r_dot);
		else c_dot = (b - 3.0f * c) / (theta * theta) * r.dot(r_dot);
		o_C_dot[i] = c_dot;
	}
}

void eae6320::MultiBody::UpdateGameObjectBasedOnInput()
{

}

void eae6320::MultiBody::ForwardKinematics()
{
	Vector3f preAnchor(0.0f, 0.0f, 0.0f);
	Matrix3f R_global;
	R_global.setIdentity();
	for (size_t i = 0; i < numOfLinks; i++)
	{
		Vector3f r = R.segment(i * 3, 3);
		Matrix3f R_local;
		R_local = AngleAxisf(r.norm(), r.normalized());
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
		M_ds[i].block<3, 3>(3, 3) = globalInertiaTensor;

		//update a b c
		float theta = r.norm();
		float a;
		if (theta < 0.0001) a = 1.0f - theta / 6.0f;
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
	}

	//LOG_TO_FILE << eae6320::Physics::totalSimulationTime << ", " << m_linkBodys[1]->m_State.position.x << ", " << -m_linkBodys[1]->m_State.position.z << ", " << m_linkBodys[1]->m_State.position.y << std::endl;
	//std::cout << eae6320::Physics::totalSimulationTime << std::endl;
	/*if (eae6320::Physics::totalSimulationTime < 12)
	{
		LOG_TO_FILE << eae6320::Physics::totalSimulationTime << ", " << m_linkBodys[1]->m_State.position.x << ", " << -m_linkBodys[1]->m_State.position.z << ", " << m_linkBodys[1]->m_State.position.y << std::endl;
	}
	else
	{
		std::cout << "done!" << std::endl;
	}*/
}

//void eae6320::MultiBody::ComputeAngularVelocityExpressionCoefficient(std::vector<float>& o_A,
//	std::vector<float>& o_B, std::vector<float>& o_C, VectorXf& i_R)
//{
//	o_A.resize(numOfLinks);
//	o_B.resize(numOfLinks);
//	o_C.resize(numOfLinks);
//
//	for (size_t i = 0; i < numOfLinks; i++)
//	{
//		Vector3f r = i_R.segment(i * 3, 3);
//		float theta = r.norm();
//
//		float theta = r.norm();
//		float a;
//		if (theta < 0.0001) a = 1.0f - theta / 6.0f;
//		else a = sin(theta) / theta;
//		o_A[i] = a;
//
//		float b;
//		if (theta < 0.0001) b = 0.5f - theta * theta / 24.0f;
//		else b = (1.0f - cos(theta)) / (theta * theta);
//		o_B[i] = b;
//
//		float c;
//		if (theta < 0.0001) c = 1.0f / 6.0f - theta * theta / 120.0f;
//		else c = (1.0f - a) / (theta * theta);
//		o_C[i] = c;
//	}
//}