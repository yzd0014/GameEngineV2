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

	m_orientations.resize(numOfLinks);
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
	J_exp.resize(numOfLinks);
	R_global.resize(numOfLinks);

	w_global.resize(numOfLinks);

	A.resize(numOfLinks);
	B.resize(numOfLinks);
	C.resize(numOfLinks);
	A_dot.resize(numOfLinks);
	B_dot.resize(numOfLinks);
	C_dot.resize(numOfLinks);

	for (int i = 0; i < numOfLinks; i++)
	{
		m_orientations[i].setIdentity();
		w_global[i].setZero();

		R[i].setZero();
		R_dot[i].setZero();
		R_dot[i](0) = 1;
		R_dot[i](1) = 1;
		R_ddot[i].setZero();
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
	
	if(rotationMode == V2_LOCAL_MODE) Compute_abc();
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
		if (rotationMode == V2_MUJOCO_MODE)
		{
			H[i].resize(6, 3);
			H[i].setZero();
			H[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]);
			H[i].block<3, 3>(3, 0) = MatrixXf::Identity(3, 3);
		}
		else
		{
			float b = B[i];
			float c = C[i];
			J_exp[i] = MatrixXf::Identity(3, 3) + b * Math::ToSkewSymmetricMatrix(R[i]) + c * Math::ToSkewSymmetricMatrix(R[i]) * Math::ToSkewSymmetricMatrix(R[i]);
			//std::cout << J.determinant() << std::endl <<std::endl;
			Matrix3f A;
			if (i == 0) A = J_exp[i];
			else A = R_global[i - 1] * J_exp[i];
			H[i].resize(6, 3);
			H[i].setZero();
			H[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * A;
			H[i].block<3, 3>(3, 0) = A;
		}
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
	}
	if (rotationMode == V2_MUJOCO_MODE)
	{
		std::function<Vector3f(int)> integrateRule = [&](int i)->Vector3f
		{
			Vector3f output = R_dot[i] + dt * R_ddot[i];
			return output;
		};
		ComputeR_ddotAndIntegrate(R_dot, R_dot, R_ddot, integrateRule);	
		for (int i = 0; i < numOfLinks; i++)
		{
			if (i == 0)
			{
				w_global[i] = R_dot[i];
			}
			else
			{
				w_global[i] = w_global[i - 1] + R_dot[i];
			}
			Vector3f deltaRotVec = w_global[i] * dt;
			Quaternionf deltaRot(AngleAxisf(deltaRotVec.norm(), deltaRotVec.normalized()));
			m_orientations[i] = deltaRot * m_orientations[i];
			m_orientations[i].normalize();
		}
	}
	else
	{
		EulerIntegration(dt);
		//RK3Integration(dt);
	}
	ForwardKinematics();
}

void eae6320::SphericalJointV2::RK3Integration(const float h)
{
	std::vector<Vector3f> k1;
	std::vector<Vector3f> k2;
	std::vector<Vector3f> k3;
	std::vector<Vector3f> R_dot_temp;
	R_dot_temp.resize(numOfLinks);

	std::function<Vector3f(int)> integrateRule0 = [&](int i)->Vector3f
	{ 
		Vector3f output = R_dot[i] + h * 0.5 * k1[i];
		return output;
	};
	ComputeR_ddotAndIntegrate(R_dot, R_dot_temp, k1, integrateRule0);
	
	std::function<Vector3f(int)> integrateRule1 = [&](int i)->Vector3f
	{
		Vector3f output = R_dot[i] + h * (2.0 * k2[i] - k1[i]);
		return output;
	};
	ComputeR_ddotAndIntegrate(R_dot_temp, R_dot_temp, k2, integrateRule1);
	
	std::function<Vector3f(int)> integrateRule2 = [&](int i)->Vector3f
	{
		Vector3f output = R_dot[i] + h * (1.0f / 6.0f) * (k1[i] + 4 * k2[i] + k3[i]);
		return output;
	};
	ComputeR_ddotAndIntegrate(R_dot_temp, R_dot, k3, integrateRule2);

	for (int i = 0; i < numOfLinks; i++)
	{
		R_ddot[i] = (1.0f / 6.0f) * (k1[i] + 4 * k2[i] + k3[i]);//update acceleration
		R[i] = R[i] + R_dot[i] * h; //integrate position
	}
	//ComputeGamma(R_dot, gamma);
	//for (int i = 0; i < numOfLinks; i++) ComputeV_dot(i);
}

void eae6320::SphericalJointV2::EulerIntegration(const float h)
{
	std::function<Vector3f(int)> integrateRule = [&](int i)->Vector3f
	{
		Vector3f output = R_dot[i] + h * R_ddot[i];
		return output;
	};
	ComputeR_ddotAndIntegrate(R_dot, R_dot, R_ddot, integrateRule);
	for (int i = 0; i < numOfLinks; i++)
	{
		R[i] = R[i] + R_dot[i] * h;//integrate position
	}
	//ComputeGamma(R_dot, gamma);
	//for (int i = 0; i < numOfLinks; i++) ComputeV_dot(i);
}

void eae6320::SphericalJointV2::ComputeV_dot(int i)
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

void eae6320::SphericalJointV2::ComputeR_ddotAndIntegrate(std::vector<Vector3f>& i_R_dot, std::vector<Vector3f>& o_R_dot_new, std::vector<Vector3f>& o_R_ddot, std::function<Vector3f(int)> integrateRule)
{
	o_R_ddot.resize(numOfLinks);
	o_R_dot_new.resize(numOfLinks);
	for (int i = 0; i < numOfLinks; i++)
	{
		if (rotationMode == V2_LOCAL_MODE)
		{
			float a = A[i];
			float b = B[i];
			float c = C[i];
			Vector3f w_local = a * i_R_dot[i] - (b * i_R_dot[i]).cross(R[i]) + c * R[i].dot(i_R_dot[i]) * R[i];
			if (i == 0)
			{
				w_global[i] = J_exp[i] * i_R_dot[i];
			}
			else
			{
				w_global[i] = w_global[i - 1] + R_global[i - 1] * J_exp[i] * i_R_dot[i];
			}
		}

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
	ComputeGamma(i_R_dot, gamma);
	//std::cout << gamma[0] << std::endl;
	for (int i = 0; i < numOfLinks; i++) ComputeV_dot(i);

	//compute Qr
	for (int i = numOfLinks - 1; i >= 0; i--)
	{
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

	//compute R_ddot and integrate
	for (int i = 0; i < numOfLinks; i++)
	{
		MatrixXf Mt = M[i] + Mr[i];
		if (i == 0)
		{
			o_R_ddot[i] = -(H[i].transpose() * Mt * H[i]).inverse() *(H[i].transpose() * (Mt * gamma[i] - (Q[i] + Qr[i])));
		}
		else
		{
			o_R_ddot[i] = -(H[i].transpose() * Mt * H[i]).inverse() *(H[i].transpose() * (Mt * D[i] * V_dot[i - 1] + Mt * gamma[i] - (Q[i] + Qr[i])));
		}
		o_R_dot_new[i] = integrateRule(i);
		//std::cout << o_R_dot_new[i] << std::endl;
		//ComputeV_dot(i);
	}
}
void eae6320::SphericalJointV2::ComputeGamma(std::vector<Vector3f>& i_R_dot, std::vector<VectorXf>& o_gamma)
{
	o_gamma.resize(numOfLinks);
	if (rotationMode == V2_LOCAL_MODE) Compute_abc_dot(i_R_dot);
	for (int i = 0; i < numOfLinks; i++)
	{
		//compute gamma
		if (rotationMode == V2_MUJOCO_MODE)
		{
			o_gamma[i].resize(6);
			o_gamma[i].setZero();
			if (i == 0)
			{
				o_gamma[i].block<3, 1>(0, 0) = -w_global[i].cross(w_global[i].cross(uGlobals[i][0]));
			}
			else
			{
				o_gamma[i].block<3, 1>(0, 0) = -w_global[i].cross(w_global[i].cross(uGlobals[i][0])) + w_global[i - 1].cross(w_global[i - 1].cross(uGlobals[i - 1][1]));
			}
		}
		else
		{
			Vector3f Jdot_rdot;
			Jdot_rdot = (C[i] * R[i].dot(i_R_dot[i]) + A_dot[i]) * i_R_dot[i] - (B_dot[i] * i_R_dot[i]).cross(R[i]) + (C_dot[i] * R[i].dot(i_R_dot[i]) + C[i] * i_R_dot[i].dot(i_R_dot[i])) * R[i];
			Vector3f gamma_theta;
			if (i == 0)
			{
				gamma_theta = Jdot_rdot;
			}
			else
			{
				gamma_theta = Math::ToSkewSymmetricMatrix(w_global[i - 1]) * R_global[i - 1] * J_exp[i] * i_R_dot[i] + R_global[i - 1] * Jdot_rdot;
			}
			o_gamma[i].resize(6);
			o_gamma[i].setZero();
			if (i == 0)
			{
				o_gamma[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * gamma_theta - w_global[i].cross(w_global[i].cross(uGlobals[i][0]));
			}
			else
			{
				o_gamma[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * gamma_theta - w_global[i].cross(w_global[i].cross(uGlobals[i][0])) + w_global[i - 1].cross(w_global[i - 1].cross(uGlobals[i - 1][1]));
			}
			o_gamma[i].block<3, 1>(3, 0) = gamma_theta;
		}
	}
}
void eae6320::SphericalJointV2::ForwardKinematics()
{
	Vector3f preAnchor(0.0f, 0.0f, 0.0f);
	for (int i = 0; i < numOfLinks; i++)
	{
		if (rotationMode == V2_MUJOCO_MODE)
		{
			R_global[i] = m_orientations[i].toRotationMatrix();
			m_linkBodys[i]->m_State.orientation = Math::ConvertEigenQuatToNativeQuat(m_orientations[i]);//update orientation
		}
		else
		{
			Matrix3f R_local;
			R_local = AngleAxisf(R[i].norm(), R[i].normalized());
			if (i == 0) R_global[i] = R_local;
			else R_global[i] = R_global[i - 1] * R_local;
			
			AngleAxisf angleAxis_global(R_global[i]);

			//update orientation
			m_linkBodys[i]->m_State.orientation = Math::cQuaternion(angleAxis_global.angle(), Math::EigenVector2nativeVector(angleAxis_global.axis()));
			m_linkBodys[i]->m_State.orientation.Normalize();
		}
		
		//update position
		Vector3f uGlobal0 = R_global[i] * uLocals[i][0];
		uGlobals[i][0] = uGlobal0;
		Vector3f linkPos = preAnchor - uGlobal0;
		m_linkBodys[i]->m_State.position = Math::sVector(linkPos(0), linkPos(1), linkPos(2));

		//update inertia tensor
		Matrix3f globalInertiaTensor;
		globalInertiaTensor = R_global[i] * localInertiaTensors[i] * R_global[i].transpose();
		M[i].block<3, 3>(3, 3) = globalInertiaTensor;
		
		//get ready for the next iteration
		Vector3f uGlobal1 = R_global[i] * uLocals[i][1];
		uGlobals[i][1] = uGlobal1;
		preAnchor = linkPos + uGlobal1;
	}
}

void eae6320::SphericalJointV2::Compute_abc()
{
	for (int i = 0; i < numOfLinks; i++)
	{
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
	}
}

void eae6320::SphericalJointV2::Compute_abc_dot(std::vector<Vector3f>& i_R_dot)
{
	for (int i = 0; i < numOfLinks; i++)
	{
		float a = A[i];
		float b = B[i];
		float c = C[i];
		float theta = R[i].norm();

		A_dot[i] = (c - b) * R[i].dot(i_R_dot[i]);

		if (theta < 0.0001) B_dot[i] = (-1.0f / 12.0f + 1.0f / 180.0f * theta * theta) * R[i].dot(i_R_dot[i]);
		else B_dot[i] = (a - 2.0f * b) / (theta * theta) * R[i].dot(i_R_dot[i]);

		if (theta < 0.0001) C_dot[i] = (-1.0f / 60.0f + 1.0f / 1260.0f * theta * theta) * R[i].dot(i_R_dot[i]);
		else C_dot[i] = (b - 3.0f * c) / (theta * theta) * R[i].dot(i_R_dot[i]);
	}
}