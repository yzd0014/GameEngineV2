#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>

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
	obs_ori.resize(numOfLinks);
	rel_ori.resize(numOfLinks);
	R_global.resize(numOfLinks);
	R_local.resize(numOfLinks);
	J_rotation.resize(numOfLinks);
	D.resize(numOfLinks);
	Ht.resize(numOfLinks);
	H.resize(numOfLinks);
	Mbody.resize(numOfLinks);
	localInertiaTensors.resize(numOfLinks);
	g.resize(numOfLinks);
	q.resize(numOfLinks);
	qdot.resize(numOfLinks);
	jointType.resize(numOfLinks);
	posDOF.resize(numOfLinks);
	posStartIndex.resize(numOfLinks);
	velDOF.resize(numOfLinks);
	velStartIndex.resize(numOfLinks);
	jointLimit.resize(numOfLinks);
	for (size_t i = 0; i < numOfLinks; i++)
	{
		w_abs_world[i].setZero();
		w_rel_world[i].setZero();
		w_rel_local[i].setZero();
		vel[i].setZero();
		jointPos[i].setZero();
		pos[i].setZero();
		obs_ori[i].setIdentity();
		rel_ori[i].setIdentity();
		R_global[i].setIdentity();
		R_local[i].setIdentity();
		_Matrix M_d;
		M_d.resize(6, 6);
		M_d.setZero();
		M_d(0, 0) = rigidBodyMass;
		M_d(1, 1) = rigidBodyMass;
		M_d(2, 2) = rigidBodyMass;
		Mbody[i] = M_d;
		jointLimit[i] = -1;
		
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
		uPairs[0] = _Vector3(0.0f, 1.0f, 0.0f);
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

		//jointType[i] = BALL_JOINT_4D;
		jointType[i] = BALL_JOINT_3D;
	}
	//jointType[0] = FREE_JOINT;

	for (size_t i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_3D)
		{
			velDOF[i] = 3;
			posDOF[i] = 3;
			totalVelDOF += 3;
			totalPosDOF += 3;
		}
		else if (jointType[i] == BALL_JOINT_4D)
		{	
			velDOF[i] = 3;
			posDOF[i] = 4;
			totalVelDOF += 3;
			totalPosDOF += 4;
		}
		else if (jointType[i] == FREE_JOINT)
		{
			velDOF[i] = 6;
			posDOF[i] = 7;
			totalVelDOF += 6;
			totalPosDOF += 7;
		}
		if (i == 0)
		{
			velStartIndex[i] = 0;
			posStartIndex[i] = 0;
		}
		else
		{
			velStartIndex[i] = velStartIndex[i - 1] + velDOF[i - 1];
			posStartIndex[i] = posStartIndex[i - 1] + posDOF[i - 1];
		}
	}
	Mr.resize(totalVelDOF, totalVelDOF);
	q.resize(totalPosDOF);
	q.setZero();
	qdot.resize(totalVelDOF);
	qdot.setZero();

	/*uLocals[0][1] = _Vector3(1.0f, -1.0f, 1.0f);
	uLocals[1][0] = _Vector3(-1.0f, 1.0f, -1.0f);*/
	
	//uLocals[0][0] = _Vector3(1.0f, -1.0f, -1.0f);

	/*qdot.segment(3, 3) = _Vector3(-2.0f, 5.0f, 0.0f);
	qdot.segment(6, 3) = _Vector3(4.0f, -10.0f, 0.0f);*/

	//qdot.segment(0, 3) = _Vector3(-2.0f, 5.0f, 0.0f);
	//qdot.segment(3, 3) = _Vector3(0.0, 0.0, 10.0);

	//Twist test
	q.segment(0, 3) = _Vector3(-0.785, 0.0, 0.0);
	_Vector3 target_w = _Vector3(0.0, -2.0, 2.0);
	Forward();
	qdot.segment(0, 3) = J_rotation[0].inverse() * target_w;
	Forward();
	//jointLimit[0] = 0.785f;
	jointLimit[0] = 0.5 * M_PI;
	//jointLimit[1] = 0.09f;
	
	kineticEnergy0 = ComputeKineticEnergy();
	totalEnergy0 = ComputeTotalEnergy();
	angularMomentum0 = ComputeAngularMomentum();
	linearMomentum0 = ComputeTranslationalMomentum();
	std::cout << "initial total energy: " << totalEnergy0 << std::endl;
	std::cout << "initial angular momentum: " << angularMomentum0.transpose() << std::endl;
	std::cout << "initial linear momentum: " << linearMomentum0.transpose() << std::endl;
}

void eae6320::MultiBody::Tick(const double i_secondCountToIntegrate)
{	
	_Scalar dt = (_Scalar)i_secondCountToIntegrate;
	_Scalar t = (_Scalar)eae6320::Physics::totalSimulationTime;

	//EulerIntegration(dt);
	//RK3Integration(dt);
	RK4Integration(dt);

	_Vector3 momentum = ComputeTranslationalMomentum();
	_Vector3 angularMomentum = ComputeAngularMomentum();
	//std::cout << "angluar:" << std::setw(15) << angularMomentum.transpose() << std::endl;
	_Vector3 momErr = angularMomentum - angularMomentum0;
	//std::cout << "angluar norm: " << angularMomentum.norm() << std::endl;
	//std::cout << std::left << "angular mom err: " << std::setw(15) << momErr.transpose() << std::endl << std::endl;
	//std::cout << std::left 
	//	<< "tran:" << std::setw(15) << momentum.transpose()
	//	<< "angluar:" << std::setw(15) << angularMomentum.transpose() << std::endl;
	//std::cout << ComputeTotalEnergy() << std::endl << std::endl;
	//LOG_TO_FILE << t << ", " << ComputeTotalEnergy() << std::endl;
	/*std::cout << t << std::endl;
	if (t >= 3.0)
	{
		_Scalar curr_energy = ComputeTotalEnergy();
		_Scalar err = abs(curr_energy - kineticEnergy0) / kineticEnergy0;
		LOG_TO_FILE << err << std::endl;
		Physics::simPause = true;
	}*/
}

void eae6320::MultiBody::ClampRotationVector()
{
	for (size_t i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_3D)
		{
			_Vector3 r = q.segment(posStartIndex[i], 3);
			_Scalar theta = r.norm();
			if (theta > M_PI)
			{
				_Scalar eta = (_Scalar)(1.0f - 2.0f * M_PI / theta);

				//reparameterize position
				std::cout << "rotation vector clamped" << std::endl;
				q.segment(posStartIndex[i], 3) = eta * r;

				//reparameterize velocity
				_Vector3 r_dot = qdot.segment(velStartIndex[i], 3);
				_Vector3 r_dot_new = eta * r_dot + 2 * M_PI * (r.dot(r_dot) / pow(theta, 3)) * r;
				qdot.segment(velStartIndex[i], 3) = r_dot_new;
			}
		}
	}
}

void eae6320::MultiBody::Integrate_q(_Vector& o_q, _Vector& i_q, _Vector& i_qdot, _Scalar h)
{
	for (int i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_3D)
		{
			o_q.segment(posStartIndex[i], 3) = i_q.segment(posStartIndex[i], 3) + i_qdot.segment(velStartIndex[i], 3) * h;
		}
		else if (jointType[i] == BALL_JOINT_4D)
		{
			Math::QuatIntegrate(rel_ori[i], i_qdot.segment(velStartIndex[i], 3), h);

			/*_Quat quat_w(0, w_rel_local[i](0), w_rel_local[i](1), w_rel_local[i](2));
			_Quat quat_dot = 0.5f * quat_w * rel_ori[i];
			rel_ori[i] = rel_ori[i] + h * quat_dot;
			rel_ori[i].normalize();*/
		}
		//TODO: add free joint
		else if (jointType[i] == FREE_JOINT)
		{
			o_q.segment(posStartIndex[i], 3) = i_q.segment(posStartIndex[i], 3) + i_qdot.segment(velStartIndex[i], 3) * h;

			w_rel_local[i] = i_qdot.segment(velStartIndex[i] + 3, 3);
			Math::QuatIntegrate(rel_ori[i], w_rel_local[i], h);
		}
	}
}

void eae6320::MultiBody::EulerIntegration(const _Scalar h)
{
	kineticEnergy0 = ComputeKineticEnergy();
	linearMomentum0 = ComputeTranslationalMomentum();
	angularMomentum0 = ComputeAngularMomentum();
	_Vector Qr = ComputeQr_SikpVelocityUpdate(qdot);
	_Vector qddot = MrInverse * Qr;

	qdot = qdot + qddot * h;
	//KineticEnergyProjection();
	//MomentumProjection();
	//EnergyMomentumProjection();
	Integrate_q(q, q, qdot, h);
	
	ClampRotationVector();
	Forward();
	//EnergyMomentumProjection();
	ManifoldProjection();
}

void eae6320::MultiBody::RK4Integration(const _Scalar h)
{
	_Vector k1 = MrInverse * ComputeQr_SikpVelocityUpdate(qdot);
	_Vector k2 = MrInverse * ComputeQr(qdot + 0.5 * h * k1);
	_Vector k3 = MrInverse * ComputeQr(qdot + 0.5 * h * k2);
	_Vector k4 = MrInverse * ComputeQr(qdot + h * k3);

	_Vector qddot = (1.0f / 6.0f) * (k1 + 2 * k2 + 2 * k3 + k4);
	qdot = qdot + h * qddot;
	if (constraintSolverMode == IMPULSE)
	{
		/*JointLimitCheck();
		ResolveJointLimit(h);*/
		TwistLimitCheck();
		ResolveTwistLimit(h);
	}
	
	_Vector q_new(totalPosDOF);
	Integrate_q(q_new, q, qdot, h);
	if (constraintSolverMode == PBD)
	{
		UpdateBodyRotation(q_new);
		JointLimitCheck();
		ResolveJointLimitPBD(q_new, h);
	}
	q = q_new;

	ClampRotationVector();
	Forward();
}

void eae6320::MultiBody::RK3Integration(const _Scalar h)
{
	_Vector k1 = h * MrInverse * ComputeQr_SikpVelocityUpdate(qdot);
	_Vector k2 = h * MrInverse * ComputeQr(qdot + 0.5 * k1);
	_Vector k3 = h * MrInverse * ComputeQr(qdot + 2.0 * k2 - k1);

	qdot = qdot + (1.0f / 6.0f) * (k1 + 4 * k2 + k3);
	if (constraintSolverMode == IMPULSE)
	{
		JointLimitCheck();
		ResolveJointLimit(h);
	}

	_Vector q_new(totalPosDOF);
	Integrate_q(q_new, q, qdot, h);
	if (constraintSolverMode == PBD)
	{
		UpdateBodyRotation(q_new);
		JointLimitCheck();
		ResolveJointLimitPBD(q_new, h);
	}
	q = q_new;
	ClampRotationVector();
	Forward();
}

void eae6320::MultiBody::ComputeH()
{
	for (size_t i = 0; i < numOfLinks; i++)
	{
		//compute H
		if (jointType[i] == BALL_JOINT_4D)
		{
			H[i].resize(6, 3);
			H[i].setZero();
			if (i == 0)
			{
				H[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]);
				H[i].block<3, 3>(3, 0) = _Matrix::Identity(3, 3);
			}
			else
			{
				H[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * R_global[i - 1];
				H[i].block<3, 3>(3, 0) = R_global[i - 1];
			}
		}
		else if (jointType[i] == BALL_JOINT_3D)
		{
			_Vector3 r = q.segment(posStartIndex[i], 3);
			_Scalar theta = r.norm();
			_Scalar b = Compute_b(theta);
			_Scalar a = Compute_a(theta);
			_Scalar c = Compute_c(theta, a);
			J_rotation[i] = _Matrix::Identity(3, 3) + b * Math::ToSkewSymmetricMatrix(r) + c * Math::ToSkewSymmetricMatrix(r) * Math::ToSkewSymmetricMatrix(r);
			_Matrix3 A;
			if (i == 0) A = J_rotation[i];
			else A = R_global[i - 1] * J_rotation[i];
			H[i].resize(6, 3);
			H[i].setZero();
			H[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * A;
			H[i].block<3, 3>(3, 0) = A;
		}
		//TODO: add free joint
		else if (jointType[i] == FREE_JOINT)
		{
			H[i].resize(6, 6);
			H[i].setIdentity();
		}
	}
}

void eae6320::MultiBody::ComputeD()
{
	for (size_t i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_3D || jointType[i] == BALL_JOINT_4D)
		{
			if (i > 0)
			{
				D[i].resize(6, 6);
				D[i].setIdentity();
				D[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) - Math::ToSkewSymmetricMatrix(uGlobals[i - 1][1]);
			}
		}
		else if (jointType[i] == FREE_JOINT)
		{
			D[i].resize(6, 6);
			D[i].setZero();
		}
	}
}

void eae6320::MultiBody::ComputeHt()
{
	ComputeH();
	ComputeD();
	
	for (size_t i = 0; i < numOfLinks; i++)
	{
		//compose Ht
		Ht[i].resize(6, totalVelDOF);
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
			Ht[i].block(0, velStartIndex[k], 6, velDOF[k]) = H_temp;
		}
	}
}

void eae6320::MultiBody::ComputeMr()
{
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
}

_Vector eae6320::MultiBody::ComputeQr_SikpVelocityUpdate(_Vector& i_qdot)
{
	std::vector<_Vector> gamma_t;
	ComputeGamma_t(gamma_t, i_qdot);

	_Vector Qr;
	Qr.resize(totalVelDOF);
	Qr.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
		_Vector Fe;
		Fe.resize(6);
		Fe.setZero();
		if (gravity)
		{
			Fe.block<3, 1>(0, 0) = _Vector3(0.0f, -9.81f, 0.0f);
		}
		_Vector Fv;
		Fv.resize(6);
		Fv.setZero();
		Fv.block<3, 1>(3, 0) = -w_abs_world[i].cross(Mbody[i].block<3, 3>(3, 3) * w_abs_world[i]);
		_Vector Q_temp;
		Q_temp.resize(totalVelDOF);
		Q_temp.setZero();
		Q_temp = Ht[i].transpose() * (Fe + Fv - Mbody[i] * gamma_t[i]);
		Qr = Qr + Q_temp;
	}

	return Qr;
}

_Vector eae6320::MultiBody::ComputeQr(_Vector i_qdot)
{
	ForwardAngularAndTranslationalVelocity(i_qdot);
	return ComputeQr_SikpVelocityUpdate(i_qdot);
}

void eae6320::MultiBody::ComputeGamma_t(std::vector<_Vector>& o_gamma_t, _Vector& i_qdot)
{	
	std::vector<_Vector> gamma;
	gamma.resize(numOfLinks);
	for (size_t i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_4D)
		{
			_Vector3 r_dot = i_qdot.segment(velStartIndex[i], 3);
			_Vector3 gamma_theta;
			gamma_theta.setZero();
			if (i > 0)
			{
				gamma_theta = Math::ToSkewSymmetricMatrix(w_abs_world[i - 1]) * R_global[i - 1] * r_dot;
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
		else if (jointType[i] == BALL_JOINT_3D)
		{
			_Vector3 r = q.segment(posStartIndex[i], 3);
			_Vector3 r_dot = i_qdot.segment(velStartIndex[i], 3);
			_Scalar theta = r.norm();
			_Scalar b = Compute_b(theta);
			_Scalar a = Compute_a(theta);
			_Scalar c = Compute_c(theta, a);
			_Scalar a_dot = Compute_a_dot(c, b, r, r_dot);
			_Scalar b_dot = Compute_b_dot(theta, a, b, r, r_dot);
			_Scalar c_dot = Compute_c_dot(theta, b, c, r, r_dot);

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
		//TODO: add free joint
		else if (jointType[i] == FREE_JOINT)
		{
			gamma[i].resize(6);
			gamma[i].setZero();
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

void eae6320::MultiBody::ForwardAngularAndTranslationalVelocity(_Vector& i_qdot)
{
	for (int i = 0; i < numOfLinks; i++)
	{
		_Vector tran_rot_velocity;
		tran_rot_velocity = Ht[i] * i_qdot;
		vel[i] = tran_rot_velocity.segment(0, 3);
		w_abs_world[i] = tran_rot_velocity.segment(3, 3);
	}
}

void eae6320::MultiBody::UpdateBodyRotation(_Vector& i_q)
{
	for (size_t i = 0; i < numOfLinks; i++)
	{
		//update orientation
		if (jointType[i] == BALL_JOINT_4D)
		{
			if (i == 0)
			{
				obs_ori[i] = rel_ori[i];
			}
			else
			{
				obs_ori[i] = obs_ori[i - 1] * rel_ori[i];
			}
			R_global[i] = obs_ori[i].toRotationMatrix();
			m_linkBodys[i]->m_State.orientation = Math::ConvertEigenQuatToNativeQuat(obs_ori[i]);
		}
		else if (jointType[i] == BALL_JOINT_3D)
		{
			_Vector3 r = i_q.segment(posStartIndex[i], 3);
			//_Matrix3 R_local;

#if defined (HIGH_PRECISION_MODE)
			R_local[i] = AngleAxisd(r.norm(), r.normalized());
#else
			R_local = AngleAxisf(r.norm(), r.normalized());
#endif
			if (i == 0)
			{
				R_global[i] = R_local[i];
			}
			else
			{
				R_global[i] = R_global[i - 1] * R_local[i];
			}

#if defined (HIGH_PRECISION_MODE)
			AngleAxisd angleAxis_global(R_global[i]);
#else
			AngleAxisf angleAxis_global(R_global[i]);
#endif

			m_linkBodys[i]->m_State.orientation = Math::cQuaternion((float)angleAxis_global.angle(), Math::EigenVector2nativeVector(angleAxis_global.axis()));
			m_linkBodys[i]->m_State.orientation.Normalize();
		}
		else if (jointType[i] == FREE_JOINT)
		{
			obs_ori[i] = rel_ori[i];
			R_global[i] = obs_ori[i].toRotationMatrix();
			m_linkBodys[i]->m_State.orientation = Math::ConvertEigenQuatToNativeQuat(obs_ori[i]);
		}
	}
}

void eae6320::MultiBody::ForwardKinematics()
{
	UpdateBodyRotation(q);

	_Vector3 preAnchor;
	Math::NativeVector2EigenVector(m_State.position, preAnchor);
	jointPos[0] = preAnchor;
	for (size_t i = 0; i < numOfLinks; i++)
	{
		//update position
		if (jointType[i] == BALL_JOINT_3D || jointType[i] == BALL_JOINT_4D)
		{
			uGlobals[i][0] = R_global[i] * uLocals[i][0];
			pos[i] = preAnchor - uGlobals[i][0];
			m_linkBodys[i]->m_State.position = Math::sVector((float)pos[i](0), (float)pos[i](1), (float)pos[i](2));
		}
		//TODO: add free joint
		else if (jointType[i] == FREE_JOINT)
		{
			uGlobals[i][0] = R_global[i] * uLocals[i][0];
			pos[i] = q.segment(posStartIndex[i], 3);
			m_linkBodys[i]->m_State.position = Math::sVector((float)pos[i](0), (float)pos[i](1), (float)pos[i](2));
		}
		
		//get ready for the next iteration
		uGlobals[i][1] = R_global[i] * uLocals[i][1];
		preAnchor = pos[i] + uGlobals[i][1];
		if (i + 1 < numOfLinks) jointPos[i + 1] = preAnchor;

		//update inertia tensor
		if (geometry != BOX && geometry != BALL)
		{
			_Matrix3 globalInertiaTensor;
			globalInertiaTensor = R_global[i] * localInertiaTensors[i] * R_global[i].transpose();
			Mbody[i].block<3, 3>(3, 3) = globalInertiaTensor;
		}	
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

void eae6320::MultiBody::Forward()
{
	ForwardKinematics();
	ComputeHt();
	ComputeMr();
	MrInverse = Mr.inverse();
	ForwardAngularAndTranslationalVelocity(qdot);
}

_Vector3 eae6320::MultiBody::ComputeTranslationalMomentum()
{
	_Vector3 translationalMomentum;
	translationalMomentum.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
		translationalMomentum += Mbody[i].block<3, 3>(0, 0) * vel[i];
	}
	return translationalMomentum;
}

_Vector3 eae6320::MultiBody::ComputeAngularMomentum()
{
	_Vector3 angularMomentum;
	angularMomentum.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
		angularMomentum += Mbody[i].block<3, 3>(3, 3) * w_abs_world[i] + rigidBodyMass * pos[i].cross(vel[i]);
	}
	return angularMomentum;
}

void eae6320::MultiBody::KineticEnergyProjection()
{
	_Matrix A(totalVelDOF, totalVelDOF);
	A.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
		A = A + Ht[i].transpose() * Mbody[i] * Ht[i];
	}

	_Matrix J(1, totalVelDOF);
	J = (A * qdot).transpose();

	_Scalar E = ComputeTotalEnergy();
	_Matrix T = J * J.transpose();
	_Scalar Ts = T(0, 0);
	if (Ts > 0.0000001)
	{
		_Scalar lambda = (-E + kineticEnergy0) / Ts;

		_Vector qdotCorrection = J.transpose() * lambda;
		qdot = qdot + qdotCorrection;
	}	
	else
	{
		std::cout << "Singular Constraint!" << std::endl;
	}
}

void eae6320::MultiBody::ManifoldProjection()
{
	//recompute H
	//TODO: external force, update totalEnergy0
	//if (gravity)
	//{
	//	kineticEnergy0 = totalEnergy0 - ComputePotentialEnergy();
	//}
	////recompute P
	//if (jointType[0] != FREE_JOINT || gravity)
	//{
	//	linearMomentum0 = ComputeTranslationalMomentum();
	//	//std::cout << linearMomentum0 << std::endl;
	//}
	////recompute L
	//if (gravity)
	//{
	//	angularMomentum0 = ComputeAngularMomentum();
	//}

	int numOfConstraints = 7;
	int nq = totalVelDOF + 2;
	int n = nq + numOfConstraints;
	_Matrix f(n, 1);
	_Matrix grad_f(n, n);

	_Vector x(n);
	x.setZero();
	x.segment(0, totalVelDOF) = qdot;

	_Vector qt(nq);
	qt.setZero();
	qt.segment(0, totalVelDOF) = qdot;

	_Matrix D(nq, nq);
	D.setZero();
	//D.block(0, 0, totalVelDOF, totalVelDOF) = Mr;
	D.block(0, 0, totalVelDOF, totalVelDOF) = _Matrix::Identity(totalVelDOF, totalVelDOF);
	_Scalar m_coeff = 0.001;
	D(totalVelDOF, totalVelDOF) = m_coeff;
	D(totalVelDOF + 1, totalVelDOF + 1) = m_coeff;
	//D(totalVelDOF + 2, totalVelDOF + 2) = m_coeff;

	_Matrix Kp(3, totalVelDOF);
	Kp.setZero();
	_Matrix Kl(3, totalVelDOF);
	Kl.setZero();
	_Matrix Sv(3, 6);
	Sv.setZero();
	Sv.block<3, 3>(0, 0) = _Matrix::Identity(3, 3);
	_Matrix Sw(3, 6);
	Sw.setZero();
	Sw.block<3, 3>(0, 3) = _Matrix::Identity(3, 3);
	for (int i = 0; i < numOfLinks; i++)
	{
		Kp = Kp + Mbody[i].block<3, 3>(0, 0) * Sv * Ht[i];
		Kl = Kl + Mbody[i].block<3, 3>(3, 3) * Sw * Ht[i] + rigidBodyMass * Math::ToSkewSymmetricMatrix(pos[i]) * Sv * Ht[i];
	}

	_Scalar kineticEnergy_t = ComputeKineticEnergy();
	_Vector linearMomentum_t = ComputeTranslationalMomentum();
	_Vector angularMomentum_t = ComputeAngularMomentum();
	_Matrix grad_C(7, nq);
	grad_C.setZero();
	grad_C.block(1, 0, 3, totalVelDOF) = Kp;
	grad_C.block(4, 0, 3, totalVelDOF) = Kl;
	//grad_C(0, totalVelDOF) = kineticEnergy0 - kineticEnergy_t;
	grad_C.block<3, 1>(1, totalVelDOF) = linearMomentum_t - linearMomentum0;
	grad_C.block<3, 1>(4, totalVelDOF + 1) = angularMomentum_t - angularMomentum0;
	//std::cout << grad_C << std::endl;

	_Matrix C(7, 1);
	_Matrix HessianC_lambda(nq, nq);
	HessianC_lambda.setZero();
	_Matrix HessianL(totalVelDOF, totalVelDOF);

	_Scalar energyErr = 1.0;
	int i = 0;
	while (energyErr > 0.0000001)
	{
		//compute f
		C(0, 0) = 0.5 * (x.segment(0, totalVelDOF).transpose() * Mr * x.segment(0, totalVelDOF))(0, 0) - kineticEnergy0;
		C.block<3, 1>(1, 0) = Kp * x.segment(0, totalVelDOF) - (1 - x(totalVelDOF)) * linearMomentum_t - x(totalVelDOF) * linearMomentum0;
		C.block<3, 1>(4, 0) = Kl * x.segment(0, totalVelDOF) - (1 - x(totalVelDOF + 1)) * angularMomentum_t - x(totalVelDOF + 1) * angularMomentum0;
		grad_C.block(0, 0, 1, totalVelDOF) = (Mr * x.segment(0, totalVelDOF)).transpose();
		if (i == 0)
		{
			//initialize lambda
			x.segment(nq, numOfConstraints) = (grad_C * grad_C.transpose()).inverse() * -C;
		}
		f.block(0, 0, nq, 1) = D * (x.segment(0, nq) - qt) - grad_C.transpose() * x.segment(nq, 7);
		f.block<7, 1>(nq, 0) = C;

		//compute Lagrange Hesssian
		HessianC_lambda.block(0, 0, totalVelDOF, totalVelDOF) = x(nq) * Mr;
		//HessianL = _Matrix::Identity(totalVelDOF, totalVelDOF) - HessianC_lambda;
		HessianL = D - HessianC_lambda;

		//compute gradient of f
		grad_f.setZero();
		grad_f.block(0, 0, nq, nq) = HessianL;
		grad_f.block(0, nq, nq, 7) = -grad_C.transpose();
		grad_f.block(nq, 0, 7, nq) = grad_C;

		//std::cout << grad_f << std::endl;
		//update
		if (grad_f.determinant() < 0.000001)
		{
			EAE6320_ASSERTF(false, "grad_f is not invertable");
		}
		x = x - grad_f.inverse() * f;
		//std::cout << std::endl << x << std::endl;
		_Vector qdot_new = x.segment(0, totalVelDOF);
		ForwardAngularAndTranslationalVelocity(qdot_new);
		energyErr = fabs(ComputeTotalEnergy() - kineticEnergy0);
		std::cout << energyErr << std::endl;
		i++;
	}
	qdot = x.segment(0, totalVelDOF);
	std::cout << ComputeTotalEnergy() << std::endl;
}

void eae6320::MultiBody::EnergyMomentumProjection()
{
	//recompute H
	if (gravity)
	{
		kineticEnergy0 = totalEnergy0 - ComputePotentialEnergy();
	}
	//recompute P
	if (jointType[0] != FREE_JOINT || gravity)
	{
		linearMomentum0 = ComputeTranslationalMomentum();
	}
	//recompute L
	if (gravity)
	{
		angularMomentum0 = ComputeAngularMomentum();
	}

	_Matrix f(totalVelDOF + 4, 1);
	_Matrix grad_f(totalVelDOF + 4, totalVelDOF + 4);

	_Vector x(totalVelDOF + 4, 1);
	x.setZero();
	x.segment(0, totalVelDOF) = qdot;

	_Matrix K(3, totalVelDOF);
	K.setZero();
	_Matrix Sv(3, 6);
	Sv.setZero();
	Sv.block<3, 3>(0, 0) = _Matrix::Identity(3, 3);
	_Matrix Sw(3, 6);
	Sw.setZero();
	Sw.block<3, 3>(0, 3) = _Matrix::Identity(3, 3);
	for (int i = 0; i < numOfLinks; i++)
	{
		K = K + Mbody[i].block<3, 3>(3, 3) * Sw * Ht[i] + rigidBodyMass * Math::ToSkewSymmetricMatrix(pos[i]) * Sv * Ht[i];
	}

	_Matrix grad_C(4, totalVelDOF);
	_Scalar energyErr = 1.0;
	grad_C.block(1, 0, 3, totalVelDOF) = K;

	_Matrix C(4, 1);
	_Matrix HessianC_lambda(totalVelDOF, totalVelDOF);
	_Matrix HessianL(totalVelDOF, totalVelDOF);

	int i = 0;
	while (energyErr > 0.0000001)
	{
		//compute f
		_Matrix energy_c(1, 1);
		energy_c(0, 0) = kineticEnergy0;
		C.block<1, 1>(0, 0) = 0.5 * x.segment(0, totalVelDOF).transpose() * Mr * x.segment(0, totalVelDOF) - energy_c;
		C.block<3, 1>(1, 0) = K * x.segment(0, totalVelDOF) - angularMomentum0;
		grad_C.block(0, 0, 1, totalVelDOF) = (Mr * x.segment(0, totalVelDOF)).transpose();
		if (i == 0)
		{
			//initialize lambda
			x.segment(totalVelDOF, 4) = (grad_C * grad_C.transpose()).inverse() * -C;
		}
		f.block(0, 0, totalVelDOF, 1) = Mr * (x.segment(0, totalVelDOF) - qdot) - grad_C.transpose() * x.segment(totalVelDOF, 4);
		f.block<4, 1>(totalVelDOF, 0) = C;

		//compute Lagrange Hesssian
		HessianC_lambda = x.segment(totalVelDOF, 4)(0) * Mr;
		//HessianL = _Matrix::Identity(totalVelDOF, totalVelDOF) - HessianC_lambda;
		HessianL = Mr - HessianC_lambda;

		//compute gradient of f
		grad_f.setZero();
		grad_f.block(0, 0, totalVelDOF, totalVelDOF) = HessianL;
		grad_f.block(0, totalVelDOF, totalVelDOF, 4) = -grad_C.transpose();
		grad_f.block(totalVelDOF, 0, 4, totalVelDOF) = grad_C;

		//update
		if (grad_f.determinant() < 0.00000001)
		{
			EAE6320_ASSERTF(false, "grad_f is not invertable");
		}
		x = x - grad_f.inverse() * f;
		_Vector qdot_new = x.segment(0, totalVelDOF);
		ForwardAngularAndTranslationalVelocity(qdot_new);
		energyErr = fabs(ComputeTotalEnergy() - kineticEnergy0);
		//std::cout << energyErr << std::endl;
		i++;
	}
	qdot = x.segment(0, totalVelDOF);
}

_Scalar eae6320::MultiBody::ComputeKineticEnergy()
{
	_Scalar out = 0;
	for (int i = 0; i < numOfLinks; i++)
	{
		_Scalar kineticEnergyRotation = 0;
		kineticEnergyRotation = 0.5 * w_abs_world[i].transpose() * Mbody[i].block<3, 3>(3, 3) * w_abs_world[i];

		_Scalar kineticEnergyTranslaion = 0;
		kineticEnergyTranslaion = 0.5 * vel[i].transpose() * Mbody[i].block<3, 3>(0, 0) * vel[i];

		out += kineticEnergyRotation + kineticEnergyTranslaion;
	}
	return out;
}

_Scalar eae6320::MultiBody::ComputePotentialEnergy()
{
	_Scalar out = 0;
	for (int i = 0; i < numOfLinks; i++)
	{
		_Scalar potentialEnergy = 0;
		_Vector3 g(0.0f, 9.81f, 0.0f);
		_Vector3 x;
		Math::NativeVector2EigenVector(m_linkBodys[i]->m_State.position, x);
		potentialEnergy = g.transpose() * Mbody[i].block<3, 3>(0, 0) * x;

		out += potentialEnergy;
	}
	return out;
}

_Scalar eae6320::MultiBody::ComputeTotalEnergy()
{
	_Scalar energy = ComputeKineticEnergy();
	if (gravity) energy += ComputePotentialEnergy();
	return energy;
}

void eae6320::MultiBody::JointLimitCheck()
{
	jointsID.clear();
	for (int i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_3D && jointLimit[i] > 0)
		{
			_Vector3 p = _Vector3(0, -1, 0);
			g[i] = p.dot(R_local[i] * p) - cos(jointLimit[i]);
			if (g[i] < 0)
			{
				jointsID.push_back(i);
			}
		}
	}
}

void eae6320::MultiBody::ResolveJointLimit(const _Scalar h)
{
	size_t constraintNum = jointsID.size();
	if (constraintNum > 0)
	{
		_Matrix J;
		J.resize(constraintNum, totalVelDOF);
		J.setZero();
		
		_Vector bias;
		bias.resize(constraintNum);
		bias.setZero();
		for (int i = 0; i < constraintNum; i++)
		{			
			int joint_id = jointsID[i];
			_Vector3 p = _Vector3(0, -1, 0);
			_Vector3 p_new = R_local[joint_id] * p;
			J.block<1, 3>(i, velStartIndex[joint_id]) = (J_rotation[joint_id].transpose() * Math::ToSkewSymmetricMatrix(p_new) * p).transpose();
			
			_Vector3 rdot = qdot.segment(velStartIndex[joint_id], 3);
			_Matrix JV = J.block<1, 3>(i, velStartIndex[joint_id]) * rdot;
			
			_Scalar beta = 0.2f;//0.4f;
			_Scalar CR = 0.4f;// 0.2f;
			_Scalar SlopP = 0.001f;
			bias(i) = -beta / h * std::max<_Scalar>(-g[joint_id], 0.0) - CR * std::max<_Scalar>(-JV(0, 0), 0.0);
		}
		
		_Matrix lambda;
		lambda = (J * MrInverse * J.transpose()).inverse() * (-J * qdot - bias);

		for (int i = 0; i < constraintNum; i++)
		{
			if (lambda(i, 0) < 0) lambda(i, 0) = 0;
		}

		_Vector RdotCorrection = MrInverse * J.transpose() * lambda;
		qdot = qdot + RdotCorrection;
	}
}

void eae6320::MultiBody::ResolveJointLimitPBD(_Vector& i_q, const _Scalar h)
{
	size_t constraintNum = jointsID.size();
	if (constraintNum > 0)
	{
		_Matrix J;
		J.resize(constraintNum, totalVelDOF);
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
				_Vector3 r = i_q.segment(posStartIndex[joint_id], 3);
				_Scalar theta = r.norm();

				_Scalar a = Compute_a(theta);
				_Scalar b = Compute_b(theta);
				_Scalar s = Compute_s(theta, a, b);
				_Vector3 p = _Vector3(0, -1, 0);
				J.block<1, 3>(i, velStartIndex[joint_id]) = (2 * b * r.dot(p) * p - (2 * s * (cos(jointLimit[joint_id]) - cos(theta)) + a) * r).transpose();
				C(i) = cos(theta) + b * (r.dot(p)) * (r.dot(p)) - cos(jointLimit[joint_id]);
			}

			_Matrix A = J * MrInverse * J.transpose();
			_Matrix lambda = A.inverse() * -C;

			_Vector R_correction = MrInverse * J.transpose() * lambda;
			for (int i = 0; i < numOfLinks; i++)
			{
				if (posDOF[i] == velDOF[i]) i_q.segment(posStartIndex[i], posDOF[i]) = i_q.segment(posStartIndex[i], posDOF[i]) + R_correction.segment(velStartIndex[i], velDOF[i]);
			}
		}
		qdot = (i_q - q) / h;
	}
}

void eae6320::MultiBody::TwistLimitCheck()
{
	jointsID.clear();
	_Vector3 p = _Vector3(0, -1, 0);
	for (int i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_3D && jointLimit[i] > 0)
		{
			_Vector s = p.cross(R_local[i] * p);
			g[i] = s.dot(R_local[i] * s) - s.squaredNorm() * cos(jointLimit[i]);
			if (g[i] < 0)
			{
				jointsID.push_back(i);
				//Physics::simPause = true;
			}
		}
	}
}

void eae6320::MultiBody::ResolveTwistLimit(const _Scalar h)
{
	size_t constraintNum = jointsID.size();
	_Vector3 p = _Vector3(0, -10, 0);
	if (constraintNum > 0)
	{
		_Matrix J;
		J.resize(constraintNum, totalVelDOF);
		J.setZero();
		_Vector bias;
		bias.resize(constraintNum);
		bias.setZero();
		for (int k = 0; k < constraintNum; k++)
		{
			int i = jointsID[k];
			_Vector3 T0;
			_Vector3 RP = R_local[i] * p;
			T0 = -J_rotation[i].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * R_local[i] * Math::ToSkewSymmetricMatrix(p) * RP;
			_Vector3 T1;
			_Vector3 RPRP = R_local[i] * Math::ToSkewSymmetricMatrix(p) * RP;
			T1 = J_rotation[i].transpose() * Math::ToSkewSymmetricMatrix(RPRP) * Math::ToSkewSymmetricMatrix(p) * RP;
			_Vector3 T2;
			T2 = -J_rotation[i].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * R_local[i].transpose() * Math::ToSkewSymmetricMatrix(p) * RP;
			_Vector3 T3;
			T3 = 2.0 * cos(jointLimit[i]) * J_rotation[i].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * Math::ToSkewSymmetricMatrix(p) * RP;
			
			J.block<1, 3>(k, velStartIndex[i]) = (T0 + T1 + T2 + T3).transpose();

			_Vector3 rdot = qdot.segment(velStartIndex[i], 3);
			_Matrix JV = J.block<1, 3>(k, velStartIndex[i]) * rdot;

			_Scalar beta = 0.2f;//0.4f;
			_Scalar CR = 0.4f;// 0.2f;
			_Scalar SlopP = 0.001f;
			bias(k) = -beta / h * std::max<_Scalar>(-g[i], 0.0) - CR * std::max<_Scalar>(-JV(0, 0), 0.0);
		}
		_Matrix lambda;
		lambda = (J * MrInverse * J.transpose()).inverse() * (-J * qdot - bias);

		for (int i = 0; i < constraintNum; i++)
		{
			if (lambda(i, 0) < 0) lambda(i, 0) = 0;
		}

		_Vector RdotCorrection = MrInverse * J.transpose() * lambda;
		qdot = qdot + RdotCorrection;
	}
}

void eae6320::MultiBody::UpdateGameObjectBasedOnInput()
{
}
