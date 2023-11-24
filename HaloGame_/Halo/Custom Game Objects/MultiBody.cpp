#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>

eae6320::MultiBody::MultiBody(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, std::vector<GameCommon::GameObject *> & i_linkBodys, int i_numOfLinks):
	GameCommon::GameObject(i_pEffect, i_Mesh, i_State)
{
	numOfLinks = i_numOfLinks;
	
	m_linkBodys = i_linkBodys;
	w_abs_world.resize(numOfLinks);
	w_rel_world.resize(numOfLinks);
	w_rel_local.resize(numOfLinks);
	vel.resize(numOfLinks);
	pos.resize(numOfLinks);
	jointPos.resize(numOfLinks);
	m_orientations.resize(numOfLinks);
	q.resize(numOfLinks);
	qbar.resize(numOfLinks);
	A.resize(numOfLinks);
	B.resize(numOfLinks);
	C.resize(numOfLinks);
	A_dot.resize(numOfLinks);
	B_dot.resize(numOfLinks);
	C_dot.resize(numOfLinks);
	R_global.resize(numOfLinks);
	J_rotation.resize(numOfLinks);
	D.resize(numOfLinks);
	Ht.resize(numOfLinks);
	Mr.resize(3 * numOfLinks, 3 * numOfLinks);
	Mbody.resize(numOfLinks);
	localInertiaTensors.resize(numOfLinks);
	g.resize(numOfLinks);
	for (size_t i = 0; i < numOfLinks; i++)
	{
		w_abs_world[i].setZero();
		w_rel_world[i].setZero();
		w_rel_local[i].setZero();
		vel[i].setZero();
		jointPos[i].setZero();
		pos[i].setZero();
		m_orientations[i].setIdentity();
		q[i].setIdentity();
		qbar[i].setIdentity();
		R_global[i].setIdentity();
		_Matrix M_d;
		M_d.resize(6, 6);
		M_d.setZero();
		M_d(0, 0) = rigidBodyMass;
		M_d(1, 1) = rigidBodyMass;
		M_d(2, 2) = rigidBodyMass;
		Mbody[i] = M_d;
		
		_Matrix3 localInertiaTensor;
		localInertiaTensor.setIdentity();
		if (geometry == BOX)
		{
			localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
		}
		localInertiaTensors[i] = localInertiaTensor;
		Mbody[i].block<3, 3>(3, 3) = localInertiaTensor;

		std::vector<_Vector3> uPairs;
		uPairs.resize(2);
		//uPairs[0] = _Vector3(-1.0f, 1.0f, 1.0f); //0 stores u for joint connecting to parent
		uPairs[0] = _Vector3(0.0f, 1.5f, 0.0f);
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
	/*uLocals[0][1] = _Vector3(1.0f, -1.0f, 1.0f);
	uLocals[1][0] = _Vector3(-1.0f, 1.0f, -1.0f);*/
	
	//uLocals[0][0] = _Vector3(1.0f, -1.0f, -1.0f);

	Rbar.resize(6);
	Rbar.setZero();
	Rdot.resize(3 * numOfLinks);
	Rdot.setZero();
	Rdot(3) = -2.0f;
	Rdot(4) = 5.0f;
	//Rdot(2) = 0;
	R.resize(3 * numOfLinks);
	R.setZero();
	R_new.resize(3 * numOfLinks);
	R_new.setZero();
	//R.block<3, 1>(0, 0) = _Vector3(0.0f, float(M_PI) * 0.25f, 0.0f);
	//R.block<3, 1>(3, 0) = _Vector3(0.0f, float(M_PI) * 0.25f, 0.0f);

	ForwardKinematics();
}

void eae6320::MultiBody::Tick(const double i_secondCountToIntegrate)
{	
	_Scalar dt = (_Scalar)i_secondCountToIntegrate;
	_Scalar t = (_Scalar)eae6320::Physics::totalSimulationTime;

	{//compute target pose
		//_Scalar c = 10;
		_Scalar c = 10;
	/*	Rbar(0) = sin(c*t);
		Rbar(1) = -sin(c*t);
		Rbar(4) = -sin(c*t);
		Rbar(5) = sin(c*t);*/
		Rbar(0) = 1;
		Rbar(1) = -1;
		Rbar(4) = -1;
		Rbar(5) = 1;
		for (int i = 0; i < numOfLinks; i++)
		{
			_Vector3 r_bar = Rbar.segment(i * 3, 3);
#if defined (HIGH_PRECISION_MODE)
			Quaterniond quat_target(AngleAxisd(r_bar.norm(), r_bar.normalized()));
#else
			Quaternionf quat_target(AngleAxisf(r_bar.norm(), r_bar.normalized()));
#endif	
			qbar[i] = quat_target;
		}
	}
	if (controlMode == KINEMATIC)
	{
		if (rotationMode == LOCAL_MODE) 
		{
			R = Rbar;
		}
		else if (rotationMode == MUJOCO_MODE)
		{
			for (int i = 0; i < numOfLinks; i++)
			{
				q[i] = qbar[i];
				q[i].normalize();
			}
		}
	}
	else
	{
		if (rotationMode == LOCAL_MODE)
		{
			Compute_abc();
		}

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
				H[i].block<3, 3>(3, 0) = _Matrix::Identity(3, 3);
			}
			else
			{
				_Vector3 r = R.segment(i * 3, 3);

				_Scalar b = B[i];
				_Scalar c = C[i];
				J_rotation[i] = _Matrix::Identity(3, 3) + b * Math::ToSkewSymmetricMatrix(r) + c * Math::ToSkewSymmetricMatrix(r) * Math::ToSkewSymmetricMatrix(r);
				_Matrix3 A;
				if (i == 0) A = J_rotation[i];
				else A = R_global[i - 1] * J_rotation[i];
				H[i].resize(6, 3);
				H[i].setZero();
				H[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * A;
				H[i].block<3, 3>(3, 0) = A;
			}

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
			Ht[i].resize(6, 3 * numOfLinks);
			Ht[i].setZero();
			for (size_t k = 0; k <= i; k++)
			{
				_Matrix H_temp;
				H_temp.resize(6, 3);
				H_temp = H[k];
				for (size_t j = k + 1; j <= i; j++)
				{
					H_temp = D[j] * H_temp;
				}
				Ht[i].block<6, 3>(0, 3 * k) = H_temp;
			}
		}
		/**********************************************************************************************************/
		Mr.setZero();
		for (int i = 0; i < numOfLinks; i++)
		{
			_Matrix M_temp = Ht[i].transpose() * Mbody[i] * Ht[i];
			Mr = Mr + M_temp;
		}
		if (Mr.determinant() < 0.00001)
		{
			std::cout << "mass matrix singluarity reached!" << Mr.determinant() << std::endl << std::endl;
		}
		if (controlMode == SPD || (rotationMode == LOCAL_MODE && (controlMode == SPD || controlMode == PD)))
		{
			Mr = Mr + _Matrix::Identity(3 * numOfLinks, 3 * numOfLinks) * dt * kd;
		}
		/**********************************************************************************************************/
		//EulerIntegration(dt);
		RK3Integration(dt);
		//RK4Integration(dt);
	}
	
	ForwardKinematics();
	//std::cout << ComputeTotalEnergy() << std::endl << std::endl;
	//LOG_TO_FILE << eae6320::Physics::totalSimulationTime << ", " << ComputeTotalEnergy() << std::endl;
}

void eae6320::MultiBody::ClampRotationVector()
{
	for (size_t i = 0; i < numOfLinks; i++)
	{
		_Vector3 r = R.segment(i * 3, 3);
		_Scalar theta = r.norm();
		if (theta > M_PI)
		{
			_Scalar eta = (_Scalar)(1.0f - 2.0f * M_PI / theta);
			
			//reparameterize position
			std::cout << "rotation vector clamped" << std::endl;
			R.segment(i * 3, 3) = eta * r;

			//reparameterize velocity
			_Vector3 r_dot = Rdot.segment(i * 3, 3);
			_Vector3 r_dot_new = eta * r_dot + 2 * M_PI * (r.dot(r_dot) / pow(theta, 3)) * r;
			Rdot.segment(i * 3, 3) = r_dot_new;
		}
	}
}

void eae6320::MultiBody::EulerIntegration(const _Scalar h)
{
	_Vector Qr = ComputeQr(Rdot, h);
	MrInverse = Mr.inverse();
	_Vector R_ddot = MrInverse * Qr;

	Rdot = Rdot + R_ddot * h;
	if (rotationMode == MUJOCO_MODE)
	{
		for (int i = 0; i < numOfLinks; i++)
		{
			if (i == 0) w_rel_local[i] = Rdot.segment(i * 3, 3);
			else  w_rel_local[i] = R_global[i - 1].transpose() * Rdot.segment(i * 3, 3);
			
			Math::QuatIntegrate(q[i], w_rel_local[i], h);

			/*_Quat quat_w(0, w_rel_local[i](0), w_rel_local[i](1), w_rel_local[i](2));
			_Quat quat_dot = 0.5f * quat_w * q[i];
			q[i] = q[i] + h * quat_dot;
			q[i].normalize();*/
		}
	}
	else
	{
		R = R + Rdot * h;
		ClampRotationVector();
	}
	
}

void eae6320::MultiBody::RK4Integration(const _Scalar h)
{
	MrInverse = Mr.inverse();
	
	_Vector k1 = h * MrInverse * ComputeQr(Rdot, h);
	_Vector k2 = h * MrInverse * ComputeQr(Rdot + 0.5 * k1, h);
	_Vector k3 = h * MrInverse * ComputeQr(Rdot + 0.5 * k2, h);
	_Vector k4 = h * MrInverse * ComputeQr(Rdot + k3, h);

	Rdot = Rdot + (1.0f / 6.0f) * (k1 + 2 * k2 + 2 * k3 + k4);
	R = R + Rdot * h;
	ClampRotationVector();
}

void eae6320::MultiBody::RK3Integration(const _Scalar h)
{
	MrInverse = Mr.inverse();
	_Vector k1 = h * MrInverse * ComputeQr(Rdot, h);
	_Vector k2 = h * MrInverse * ComputeQr(Rdot + 0.5 * k1, h);
	_Vector k3 = h * MrInverse * ComputeQr(Rdot + 2.0 * k2 - k1, h);

	Rdot = Rdot + (1.0f / 6.0f) * (k1 + 4 * k2 + k3);
	if (constraintSolverMode == IMPULSE)
	{
		JointLimitCheck();
		ResolveJointLimit(h);
	}
	R_new = R + Rdot * h;
	if (constraintSolverMode == PBD)
	{
		JointLimitCheck();
		ResolveJointLimitPBD(h);
	}
	R = R_new;
	ClampRotationVector();
}

_Vector eae6320::MultiBody::ComputeQr(_Vector i_R_dot, _Scalar h)
{
	
	ComputeAngularVelocity(i_R_dot);
	
	std::vector<_Vector> gamma_t;
	ComputeGamma_t(gamma_t, i_R_dot);

	_Vector Qr;
	Qr.resize(3 * numOfLinks);
	Qr.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
		_Vector Fe;
		Fe.resize(6);
		Fe.setZero();
		if (controlMode == PASSIVE && gravity)
		{
			Fe.block<3, 1>(0, 0) = _Vector3(0.0f, -9.81f, 0.0f);
		}
		_Vector Fv;
		Fv.resize(6);
		Fv.setZero();
		Fv.block<3, 1>(3, 0) = -w_abs_world[i].cross(Mbody[i].block<3, 3>(3, 3) * w_abs_world[i]);
		_Vector Q_temp;
		Q_temp.resize(3 * numOfLinks);
		Q_temp.setZero();
		Q_temp = Ht[i].transpose() * (Fe + Fv - Mbody[i] * gamma_t[i]);
		Qr = Qr + Q_temp;
	}
	if (rotationMode == MUJOCO_MODE && (controlMode == PD || controlMode == SPD))
	{
		for (int i = 0; i < numOfLinks; i++)
		{
			_Quat curr_q;
			if (controlMode == PD)
			{
				curr_q = q[i];
			}
			else if (controlMode == SPD)
			{
				curr_q = q[i];
				Math::QuatIntegrate(curr_q, w_rel_local[i], h);
			}
			
			_Quat delta_q = qbar[i] * curr_q.inverse();
			delta_q.normalize();
#if defined (HIGH_PRECISION_MODE)		
			AngleAxisd rotationVector(delta_q);
#else
			AngleAxisf rotationVector(delta_q);
#endif	
			_Vector3 posError = rotationVector.angle() * rotationVector.axis();
			if (i > 0)
			{
				posError = R_global[i - 1] * posError;
			}
			Qr.segment(i * 3, 3) = Qr.segment(i * 3, 3) + kp * posError - kd * Rdot.segment(i * 3, 3);
		}
	}
	else if (rotationMode == LOCAL_MODE && (controlMode == PD || controlMode == SPD))
	{
		_Vector curr_pos = R + h * Rdot;
		_Vector posError = Rbar - curr_pos;
		Qr = Qr + kp * posError - kd * Rdot;
	}

	return Qr;
}

void eae6320::MultiBody::ComputeGamma_t(std::vector<_Vector>& o_gamma_t, _Vector& i_R_dot)
{
	if (rotationMode == LOCAL_MODE)
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
				gamma[i].block<3, 1>(0, 0) = -w_abs_world[i].cross(w_abs_world[i].cross(uGlobals[i][0]));
			}
			else
			{
				gamma[i].block<3, 1>(0, 0) = -w_abs_world[i].cross(w_abs_world[i].cross(uGlobals[i][0])) + w_abs_world[i - 1].cross(w_abs_world[i - 1].cross(uGlobals[i - 1][1]));
			}
		}
		else
		{
			_Vector3 r = R.segment(i * 3, 3);
			_Vector3 r_dot = i_R_dot.segment(i * 3, 3);
			_Scalar b = B[i];
			_Scalar c = C[i];
			_Scalar a_dot = A_dot[i];
			_Scalar b_dot = B_dot[i];
			_Scalar c_dot = C_dot[i];

			_Vector3 Jdot_rdot;
			Jdot_rdot = (c * r.dot(r_dot) + a_dot) * r_dot - (b_dot * r_dot).cross(r) + (c_dot * r.dot(r_dot) + c * r_dot.dot(r_dot)) * r;
			_Vector3 gamma_theta;
			if (i == 0)
			{
				gamma_theta = Jdot_rdot;
			}
			else
			{
				gamma_theta = Math::ToSkewSymmetricMatrix(w_abs_world[i - 1]) * R_global[i - 1] * J_rotation[i] * r_dot + R_global[i - 1] * Jdot_rdot;
			}
			gamma[i].resize(6);
			gamma[i].setZero();
			if (i == 0)
			{
				gamma[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * gamma_theta - w_abs_world[i].cross(w_abs_world[i].cross(uGlobals[i][0]));
			}
			else
			{
				gamma[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * gamma_theta - w_abs_world[i].cross(w_abs_world[i].cross(uGlobals[i][0])) + w_abs_world[i - 1].cross(w_abs_world[i - 1].cross(uGlobals[i - 1][1]));
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
		if (rotationMode == LOCAL_MODE)
		{
			_Vector3 r_dot = i_R_dot.segment(i * 3, 3);
			if (i == 0)
			{
				w_rel_world[i] = J_rotation[i] * r_dot;
				w_abs_world[i] = w_rel_world[i];
			}
			else
			{
				w_rel_world[i] = R_global[i - 1] * J_rotation[i] * r_dot;
				w_abs_world[i] = w_abs_world[i - 1] + w_rel_world[i];
			}
		}
		else if (rotationMode == MUJOCO_MODE)
		{
			w_rel_world[i] = i_R_dot.segment(i * 3, 3);
			if (i == 0)
			{
				w_abs_world[i] = i_R_dot.segment(i * 3, 3);
			}
			else
			{
				w_abs_world[i] = w_abs_world[i - 1] + i_R_dot.segment(i * 3, 3);
			}
		}
	}
}

void eae6320::MultiBody::ComputeVelocity()
{
	for (int i = 0; i < numOfLinks; i++)
	{
		vel[i].setZero();
		for (int j = 0; j <= i; j++)
		{
			_Vector3 effectiveArm;
			effectiveArm = pos[i] - jointPos[j];
			vel[i] = vel[i] + w_rel_world[j].cross(effectiveArm);
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
		
		A[i] = Compute_a(theta);
		B[i] = Compute_b(theta);
		C[i] = Compute_c(theta, A[i]);
	}
}

void eae6320::MultiBody::ForwardKinematics()
{
	_Vector3 preAnchor;
	Math::NativeVector2EigenVector(m_State.position, preAnchor);
	jointPos[0] = preAnchor;
	for (size_t i = 0; i < numOfLinks; i++)
	{
		//update orientation
		if (rotationMode == MUJOCO_MODE)
		{
			if (i == 0)
			{
				m_orientations[i] = q[i];
			}
			else
			{
				m_orientations[i] = m_orientations[i - 1] * q[i];
			}
			R_global[i] = m_orientations[i].toRotationMatrix();
			m_linkBodys[i]->m_State.orientation = Math::ConvertEigenQuatToNativeQuat(m_orientations[i]);
		}
		else if (rotationMode == LOCAL_MODE)
		{
			_Vector3 r = R.segment(i * 3, 3);
			_Matrix3 R_local;
		
#if defined (HIGH_PRECISION_MODE)
			R_local = AngleAxisd(r.norm(), r.normalized());
#else
			R_local = AngleAxisf(r.norm(), r.normalized());
#endif
			if (i == 0)
			{
				R_global[i] = R_local;
			}
			else
			{
				R_global[i] = R_global[i - 1] * R_local;
			}
			
#if defined (HIGH_PRECISION_MODE)
			AngleAxisd angleAxis_global(R_global[i]);
#else
			AngleAxisf angleAxis_global(R_global[i]);
#endif

			m_linkBodys[i]->m_State.orientation = Math::cQuaternion((float)angleAxis_global.angle(), Math::EigenVector2nativeVector(angleAxis_global.axis()));
			m_linkBodys[i]->m_State.orientation.Normalize();
		}
		//update position
		_Vector3 uGlobal0 = R_global[i] * uLocals[i][0];
		uGlobals[i][0] = uGlobal0;
		_Vector3 linkPos = preAnchor - uGlobal0;
		pos[i] = linkPos;
		m_linkBodys[i]->m_State.position = Math::sVector((float)linkPos(0), (float)linkPos(1), (float)linkPos(2));
		
		//get ready for the next iteration
		_Vector3 uGlobal1 = R_global[i] * uLocals[i][1];
		uGlobals[i][1] = uGlobal1;
		preAnchor = linkPos + uGlobal1;
		if (i + 1 < numOfLinks) jointPos[i + 1] = preAnchor;

		//update inertia tensor
		if (geometry != BOX && geometry != BALL)
		{
			_Matrix3 globalInertiaTensor;
			globalInertiaTensor = R_global[i] * localInertiaTensors[i] * R_global[i].transpose();
			Mbody[i].block<3, 3>(3, 3) = globalInertiaTensor;
		}	
	}

	ComputeVelocity();

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
		kineticEnergyRotation = 0.5 * w_abs_world[i].transpose() * Mbody[i].block<3, 3>(3, 3) * w_abs_world[i];
		
		_Scalar kineticEnergyTranslaion = 0;
		kineticEnergyTranslaion = 0.5 * vel[i].transpose() * Mbody[i].block<3, 3>(0, 0) * vel[i];

		_Scalar potentialEnergy = 0;
		_Vector3 g(0.0f, 9.81f, 0.0f);
		_Vector3 x;
		Math::NativeVector2EigenVector(m_linkBodys[i]->m_State.position, x);
		potentialEnergy = g.transpose() * Mbody[i].block<3, 3>(0, 0) * x;

		if (gravity) energy += kineticEnergyRotation + kineticEnergyTranslaion + potentialEnergy;
		else energy += kineticEnergyRotation + kineticEnergyTranslaion;
	}
	return energy;
}

void eae6320::MultiBody::JointLimitCheck()
{
	jointsID.clear();
	for (int i = 0; i < numOfLinks; i++)
	{
		_Vector3 r = R.segment(i * 3, 3);
		_Scalar theta = r.norm();
		
		_Vector3 p = _Vector3(0, -1, 0);
		g[i] = cos(theta) + B[i] * (r.dot(p)) * (r.dot(p)) - cos(jointLimit);
		if (g[i] < 0)
		{
			jointsID.push_back(i);
		}
	}
}

void eae6320::MultiBody::ResolveJointLimit(const _Scalar h)
{
	size_t constraintNum = jointsID.size();
	if (constraintNum > 0)
	{
		_Matrix J;
		J.resize(constraintNum, 3 * numOfLinks);
		J.setZero();
		
		_Vector b;
		b.resize(constraintNum);
		b.setZero();
		for (int i = 0; i < constraintNum; i++)
		{			
			int joint_id = jointsID[i];
			
			_Vector3 r = R.segment(joint_id * 3, 3);
			_Vector3 rdot = Rdot.segment(joint_id * 3, 3);
			_Scalar theta = r.norm();
			_Scalar s = Compute_s(theta, A[joint_id], B[joint_id]);

			_Vector3 p = _Vector3(0, -1, 0);
			J.block<1, 3>(i, 3 * joint_id) = (2 * B[joint_id] * r.dot(p) * p - (2 * s * (cos(jointLimit) - cos(theta)) + A[joint_id]) * r).transpose();

			_Matrix JV = J.block<1, 3>(i, 3 * joint_id) * rdot;
			_Scalar beta = 0.2f;//0.4f;
			_Scalar CR = 0.4f;// 0.2f;
			_Scalar SlopP = 0.001f;
			b(i) = -beta / h * std::max(-g[joint_id], 0.0f) - CR * std::max(-JV(0, 0), 0.0f);
		}
		_Matrix lambda;
		lambda = (J * MrInverse * J.transpose()).inverse() * (-J * Rdot - b);

		for (int i = 0; i < constraintNum; i++)
		{
			if (lambda(i, 0) < 0) lambda(i, 0) = 0;
		}

		_Vector RdotCorrection = MrInverse * J.transpose() * lambda;
		Rdot = Rdot + RdotCorrection;
	}
}

void eae6320::MultiBody::ResolveJointLimitPBD(const _Scalar h)
{
	size_t constraintNum = jointsID.size();
	if (constraintNum > 0)
	{
		_Matrix J;
		J.resize(constraintNum, 3 * numOfLinks);
		J.setZero();

		int iterationNum = 1;
		for (int i = 0; i < iterationNum; i++)
		{
			_Vector C;
			C.resize(constraintNum, 1);

			//construct total Jacobian matrix
			for (size_t i = 0; i < constraintNum; i++)
			{
				size_t joint_id = jointsID[i];
				_Vector3 r = R_new.segment(joint_id * 3, 3);
				_Scalar theta = r.norm();

				_Scalar a = Compute_a(theta);
				_Scalar b = Compute_b(theta);
				_Scalar s = Compute_s(theta, a, b);
				_Vector3 p = _Vector3(0, -1, 0);
				J.block<1, 3>(i, 3 * joint_id) = (2 * b * r.dot(p) * p - (2 * s * (cos(jointLimit) - cos(theta)) + a) * r).transpose();
				C(i) = cos(theta) + b * (r.dot(p)) * (r.dot(p)) - cos(jointLimit);
			}

			_Matrix A = J * MrInverse * J.transpose();
			_Matrix lambda = A.inverse() * -C;

			_Vector R_correction = MrInverse * J.transpose() * lambda;
			R_new = R_new + R_correction;
		}
		Rdot = (R_new - R) / h;
	}
}

void eae6320::MultiBody::UpdateGameObjectBasedOnInput()
{
}