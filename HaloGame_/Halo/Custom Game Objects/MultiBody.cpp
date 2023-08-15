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
	m_orientations.resize(numOfLinks);
	A.resize(numOfLinks);
	B.resize(numOfLinks);
	C.resize(numOfLinks);
	A_dot.resize(numOfLinks);
	B_dot.resize(numOfLinks);
	C_dot.resize(numOfLinks);
	D.resize(numOfLinks);
	H_t.resize(numOfLinks);
	M_r.resize(3 * numOfLinks, 3 * numOfLinks);
	for (size_t i = 0; i < numOfLinks; i++)
	{
		w_global[i].setZero();
		m_orientations[i].setIdentity();
		_Matrix M_d;
		M_d.resize(6, 6);
		M_d.setZero();
		M_d(0, 0) = rigidBodyMass;
		M_d(1, 1) = rigidBodyMass;
		M_d(2, 2) = rigidBodyMass;
		M_ds.push_back(M_d);
		
		_Matrix3 localInertiaTensor;
		localInertiaTensor.setIdentity();
		if (geometry == BOX)
		{
			localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
		}
		localInertiaTensors.push_back(localInertiaTensor);

		std::vector<_Vector3> uPairs;
		uPairs.resize(2);
		uPairs[0] = _Vector3(-1.0f, 1.0f, 1.0f); //0 stores u for joint connecting to parent
		if (i == numOfLinks - 1)
		{
			uPairs[1] = _Vector3(0.0f, 0.0f, 0.0f);
		}
		else
		{
			uPairs[1] = -uPairs[0]; //1 stores u for joint connecting to child
		}
		uLocals.push_back(uPairs);
		uGlobals.push_back(uPairs);
	}
	uLocals[0][1] = _Vector3(1.0f, -1.0f, 1.0f);
	uLocals[1][0] = _Vector3(-1.0f, 1.0f, -1.0f);
	
	//uLocals[0][0] = _Vector3(1.0f, -1.0f, -1.0f);

	R_dot.resize(3 * numOfLinks);
	R_dot.setZero();
	R.resize(3 * numOfLinks);
	R.setZero();
	//R.block<3, 1>(0, 0) = _Vector3(0.0f, 0.0f, -1.0f);

	ForwardKinematics();
}

void eae6320::MultiBody::Tick(const double i_secondCountToIntegrate)
{	
	_Scalar dt = (_Scalar)i_secondCountToIntegrate;
	
	if (rotationMode == LOCAL_MODE || rotationMode == GLOBAL_MODE) Compute_abc();
	std::vector<_Matrix> H;
	H.resize(numOfLinks);
	for (size_t i = 0; i < numOfLinks; i++)
	{
		//compute H
		if (rotationMode == MUJOCO_MODE)
		{
			H[i].resize(6, 3);
			H[i].setZero();
			H[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]);
			H[i].block<3, 3>(3, 0) = MatrixXf::Identity(3, 3);
		}
		else
		{
			_Vector3 r = R.segment(i * 3, 3);
			_Scalar a = A[i];
			_Scalar b = B[i];
			_Scalar c = C[i];

			_Matrix3 J;
			J.setZero();
			_Matrix3 rrt = r * r.transpose();
			J = a * _Matrix::Identity(3, 3) + b * Math::ToSkewSymmetricMatrix(r) + c * rrt;
			//std::cout << J.determinant() << std::endl <<std::endl;
			H[i].resize(6, 3);
			H[i].setZero();
			H[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * J;
			H[i].block<3, 3>(3, 0) = J;
		}
		
		//compute D
		if (i > 0)
		{
			if (rotationMode == LOCAL_MODE || rotationMode == MUJOCO_MODE)
			{
				D[i].resize(6, 6);
				D[i].setIdentity();
				D[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) - Math::ToSkewSymmetricMatrix(uGlobals[i - 1][1]);
			}
			else if (rotationMode == GLOBAL_MODE)
			{
				D[i].resize(6, 6);
				D[i].setZero();
				D[i].block<3, 3>(0, 0) = _Matrix::Identity(3, 3);
				D[i].block<3, 3>(0, 3) = -Math::ToSkewSymmetricMatrix(uGlobals[i - 1][1]);
			}
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
			_Matrix H_temp;
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
		_Matrix M_temp = H_t[i].transpose() * M_ds[i] * H_t[i];
		M_r = M_r + M_temp;
	}
/**********************************************************************************************************/
	if (rotationMode == MUJOCO_MODE)
	{		
		_Vector R_ddot = M_r.inverse() * ComputeQ_r(R_dot);
		R_dot = R_dot + R_ddot * dt;
		for (int i = 0; i < numOfLinks; i++)
		{
			if (i == 0)
			{
				w_global[i] = R_dot.segment(i * 3, 3);
			}
			else
			{
				w_global[i] = w_global[i - 1] + R_dot.segment(i * 3, 3);
			}
			Vector3f deltaRotVec = w_global[i] * dt;
			Quaternionf deltaRot(AngleAxisf(deltaRotVec.norm(), deltaRotVec.normalized()));
			m_orientations[i] = deltaRot * m_orientations[i];
			m_orientations[i].normalize();
			//std::cout << m_orientations[i].w() << ", " << m_orientations[i].x() << ", " << m_orientations[i].y() << ", " << m_orientations[i].z() << std::endl;
		}
	}
	else
	{
		EulerIntegration(dt);
		//RK3Integration(dt);
	}
	ForwardKinematics();
	//std::cout << ComputeTotalEnergy() << std::endl << std::endl;

	//post check
	for (size_t i = 0; i < numOfLinks; i++)
	{
		_Vector3 r = R.segment(i * 3, 3);
		if (r.norm() > 2 * M_PI - 0.1)
		{
			R.segment(i * 3, 3) = R.segment(i * 3, 3) - 2 * M_PI * r.normalized();
			std::cout << "large r!" << std::endl;
		}
		//std::cout << r.norm() << ", ";
	}
	//std::cout << std::endl;
}

void eae6320::MultiBody::EulerIntegration(const _Scalar h)
{
	_Vector Q_r = ComputeQ_r(R_dot);
	_Vector R_ddot = M_r.inverse() * Q_r;

	R_dot = R_dot + R_ddot * h;
	R = R + R_dot * h;
}

void eae6320::MultiBody::RK4Integration(const _Scalar h)
{
	_Matrix M_rInverse = M_r.inverse();
	
	_Vector k1 = h * M_rInverse * ComputeQ_r(R_dot);
	_Vector k2 = h * M_rInverse * ComputeQ_r(R_dot + 0.5 * k1);
	_Vector k3 = h * M_rInverse * ComputeQ_r(R_dot + 0.5 * k2);
	_Vector k4 = h * M_rInverse * ComputeQ_r(R_dot + k3);

	R_dot = R_dot + (1.0f / 6.0f) * (k1 + 2 * k2 + 2 * k3 + k4);
	R = R + R_dot * h;
}

void eae6320::MultiBody::RK3Integration(const _Scalar h)
{
	_Matrix M_rInverse = M_r.inverse();
	_Vector k1 = h * M_rInverse * ComputeQ_r(R_dot);
	_Vector k2 = h * M_rInverse * ComputeQ_r(R_dot + 0.5 * k1);
	_Vector k3 = h * M_rInverse * ComputeQ_r(R_dot + 2.0 * k2 - k1);

	R_dot = R_dot + (1.0f / 6.0f) * (k1 + 4 * k2 + k3);
	R = R + R_dot * h;
}

_Vector eae6320::MultiBody::ComputeQ_r(_Vector i_R_dot)
{
	if (rotationMode == LOCAL_MODE || rotationMode == GLOBAL_MODE)
	{
		ComputeAngularVelocity(i_R_dot);
	}
	
	std::vector<_Vector> gamma_t;
	ComputeGamma_t(gamma_t, i_R_dot);

	_Vector Q_r;
	Q_r.resize(3 * numOfLinks);
	Q_r.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
		_Vector Fe;
		Fe.resize(6);
		Fe.setZero();
		Fe.block<3, 1>(0, 0) = _Vector3(0.0f, -9.81f, 0.0f);
		_Vector Fv;
		Fv.resize(6);
		Fv.setZero();
		Fv.block<3, 1>(3, 0) = -w_global[i].cross(M_ds[i].block<3, 3>(3, 3) * w_global[i]);
		_Vector Q_temp;
		Q_temp.resize(3 * numOfLinks);
		Q_temp.setZero();
		Q_temp = H_t[i].transpose() * (Fe + Fv - M_ds[i] * gamma_t[i]);
		Q_r = Q_r + Q_temp;
	}

	return Q_r;
}

void eae6320::MultiBody::ComputeGamma_t(std::vector<_Vector>& o_gamma_t, _Vector& i_R_dot)
{
	if (rotationMode == LOCAL_MODE || rotationMode == GLOBAL_MODE)
	{
		Compute_abc_dot(i_R_dot);
	}
	
	std::vector<_Vector> gamma;
	gamma.resize(numOfLinks);
	for (size_t i = 0; i < numOfLinks; i++)
	{
		if (rotationMode == MUJOCO_MODE)
		{
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
		else
		{
			_Vector3 r = R.segment(i * 3, 3);
			_Vector3 r_dot = i_R_dot.segment(i * 3, 3);
			_Scalar a = A[i];
			_Scalar b = B[i];
			_Scalar c = C[i];
			_Scalar a_dot = A_dot[i];
			_Scalar b_dot = B_dot[i];
			_Scalar c_dot = C_dot[i];

			_Vector3 gamma_theta;
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
	}

	o_gamma_t.resize(numOfLinks);
	for (size_t i = 0; i < numOfLinks; i++)
	{
		o_gamma_t[i].resize(6);
		o_gamma_t[i].setZero();
		for (size_t j = 0; j <= i; j++)
		{
			_Vector gamma_temp;
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

void eae6320::MultiBody::ComputeAngularVelocity(_Vector& i_R_dot)
{
	for (size_t i = 0; i < numOfLinks; i++)
	{
		_Vector3 r = R.segment(i * 3, 3);
		_Vector3 r_dot = i_R_dot.segment(i * 3, 3);

		_Scalar a = A[i];
		_Scalar b = B[i];
		_Scalar c = C[i];

		if (rotationMode == LOCAL_MODE)
		{
			_Vector3 w_local = a * r_dot - (b * r_dot).cross(r) + c * r.dot(r_dot) * r;
			if (i == 0)
			{
				w_global[i] = w_local;
			}
			else
			{
				w_global[i] = w_global[i - 1] + w_local;
			}
		}
		else if (rotationMode == GLOBAL_MODE)
		{
			w_global[i] = a * r_dot - (b * r_dot).cross(r) + c * r.dot(r_dot) * r;
		}
	}
}

void eae6320::MultiBody::Compute_abc_dot(_Vector& i_R_dot)
{
	for (size_t i = 0; i < numOfLinks; i++)
	{
		_Vector3 r = R.segment(i * 3, 3);
		_Vector3 r_dot = i_R_dot.segment(i * 3, 3);
		_Scalar theta = r.norm();
		
		_Scalar a = A[i];
		_Scalar b = B[i];
		_Scalar c = C[i];

		A_dot[i] = (c - b) * r.dot(r_dot);

		_Scalar b_dot;
		if (theta < 0.0001) b_dot = (-1.0f / 12.0f + 1.0f / 180.0f * theta * theta) * r.dot(r_dot);
		else b_dot = (a - 2.0f * b) / (theta * theta) * r.dot(r_dot);
		B_dot[i] = b_dot;

		_Scalar c_dot;
		if (theta < 0.0001) c_dot = (-1.0f / 60.0f + 1.0f / 1260.0f * theta * theta) * r.dot(r_dot);
		else c_dot = (b - 3.0f * c) / (theta * theta) * r.dot(r_dot);
		C_dot[i] = c_dot;
	}
}

void eae6320::MultiBody::Compute_abc()
{
	for (size_t i = 0; i < numOfLinks; i++)
	{
		_Vector3 r = R.segment(i * 3, 3);
		//update a b c
		_Scalar theta = r.norm();
		_Scalar a;
		if (theta < 0.0001) a = 1.0f - theta * theta / 6.0f;
		else a = sin(theta) / theta;
		A[i] = a;

		_Scalar b;
		if (theta < 0.0001) b = 0.5f - theta * theta / 24.0f;
		else b = (1.0f - cos(theta)) / (theta * theta);
		B[i] = b;

		_Scalar c;
		if (theta < 0.0001) c = 1.0f / 6.0f - theta * theta / 120.0f;
		else c = (1.0f - a) / (theta * theta);
		C[i] = c;
	}
}

void eae6320::MultiBody::ForwardKinematics()
{
	_Vector3 preAnchor(0.0f, 0.0f, 0.0f);
	_Matrix3 R_global;
	R_global.setIdentity();
	for (size_t i = 0; i < numOfLinks; i++)
	{
		//update orientation
		if (rotationMode == MUJOCO_MODE)
		{
			R_global = m_orientations[i].toRotationMatrix();
			m_linkBodys[i]->m_State.orientation = Math::ConvertEigenQuatToNativeQuat(m_orientations[i]);
		}
		else
		{
			_Vector3 r = R.segment(i * 3, 3);
			_Matrix3 R_local;
#if defined (HIGH_PRECISION_MODE)
			if (rotationMode == LOCAL_MODE)
			{
				R_local = AngleAxisd(r.norm(), r.normalized());
				R_global = R_global * R_local;
			}
			else if (rotationMode == GLOBAL_MODE)
			{
				R_global = AngleAxisd(r.norm(), r.normalized());
			}
			AngleAxisd angleAxis_global(R_global);
#else
			if (rotationMode == LOCAL_MODE)
			{
				R_local = AngleAxisf(r.norm(), r.normalized());
				R_global = R_global * R_local;
			}
			else if (rotationMode == GLOBAL_MODE)
			{
				R_global = AngleAxisf(r.norm(), r.normalized());
			}
			AngleAxisf angleAxis_global(R_global);
#endif
			m_linkBodys[i]->m_State.orientation = Math::cQuaternion((float)angleAxis_global.angle(), Math::EigenVector2nativeVector(angleAxis_global.axis()));
			m_linkBodys[i]->m_State.orientation.Normalize();
		}
		//update position
		_Vector3 uGlobal0 = R_global * uLocals[i][0];
		uGlobals[i][0] = uGlobal0;
		_Vector3 linkPos = preAnchor - uGlobal0;
		m_linkBodys[i]->m_State.position = Math::sVector((float)linkPos(0), (float)linkPos(1), (float)linkPos(2));
		
		//get ready for the next iteration
		Vector3f uGlobal1 = R_global * uLocals[i][1];
		uGlobals[i][1] = uGlobal1;
		preAnchor = linkPos + uGlobal1;

		//update inertia tensor
		_Matrix3 globalInertiaTensor;
		globalInertiaTensor = R_global * localInertiaTensors[i] * R_global.transpose();
		M_ds[i].block<3, 3>(3, 3) = globalInertiaTensor;
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

_Scalar eae6320::MultiBody::ComputeTotalEnergy()
{
	_Scalar energy = 0;
	for (int i = 0; i < numOfLinks; i++)
	{
		_Scalar kineticEnergyRotation = 0;
		kineticEnergyRotation = 0.5 * w_global[i].transpose() * M_ds[i].block<3, 3>(3, 3) * w_global[i];
		
		_Scalar kineticEnergyTranslaion = 0;
		_Vector3 v = w_global[i].cross(-uGlobals[i][0]);
		kineticEnergyTranslaion = 0.5 * v.transpose() * M_ds[i].block<3, 3>(0, 0) * v;

		_Scalar potentialEnergy = 0;
		_Vector3 g(0.0f, 9.81f, 0.0f);
		_Vector3 x;
		x = Math::NativeVector2EigenVector(m_linkBodys[i]->m_State.position);
		potentialEnergy = g.transpose() * M_ds[i].block<3, 3>(0, 0) * x;

		energy += kineticEnergyRotation + kineticEnergyTranslaion + potentialEnergy;
	}
	return energy;
}

void eae6320::MultiBody::UpdateGameObjectBasedOnInput()
{
}