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
	//UnitTest1();
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
	jointType.resize(numOfLinks);
	posDOF.resize(numOfLinks);
	posStartIndex.resize(numOfLinks);
	velDOF.resize(numOfLinks);
	velStartIndex.resize(numOfLinks);
	jointLimit.resize(numOfLinks);
	jointRange.resize(numOfLinks);
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
		std::pair<_Scalar, _Scalar> defaultRange(-1, -1);
		jointRange[i] = defaultRange;
		
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

		jointType[i] = BALL_JOINT_4D;
		//jointType[i] = BALL_JOINT_3D;
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

	//qdot.segment(0, 3) = _Vector3(-2.0f, 2.0f, 0.0f);
	//qdot.segment(3, 3) = _Vector3(2.0, 2.0, 0.0);

	//UnitTest0();
	//general twist test
	//_Vector3 rot_vec(-0.25 * M_PI, 0.0, 0.0);
	//q.segment(0, 3) = rot_vec;
	//if (jointType[0] == BALL_JOINT_4D)
	//{
	//	rel_ori[0] = Math::RotationConversion_VecToQuat(rot_vec);
	//}
	//_Vector3 local_w = _Vector3(0.0, -2.0, -2.0);
	////_Vector3 local_w = _Vector3(0.0, -2.0, 0.0);
	//Forward();
	//_Vector3 world_w = R_global[0] * local_w;
	//qdot.segment(0, 3) = J_rotation[0].inverse() * world_w;
	//if (jointType[0] == BALL_JOINT_4D)
	//{
	//	//qdot.segment(0, 3) = world_w;
	//	qdot.segment(0, 3) = local_w;
	//}

	//swing test
	//_Vector3 rot_vec(0.0, 0.7 * M_PI, 0.0);
	//q.segment(0, 3) = rot_vec;
	//if (jointType[0] == BALL_JOINT_4D)
	//{
	//	rel_ori[0] = Math::RotationConversion_VecToQuat(rot_vec);
	//}
	//_Vector3 local_w = _Vector3(-2.0, 0.0, 2.0);
	//Forward();
	//qdot.segment(0, 3) = J_rotation[0].inverse() * local_w;
	//if (jointType[0] == BALL_JOINT_4D)
	//{
	//	qdot.segment(0, 3) = local_w;
	//}
	
	qdot.segment(0, 3) = _Vector3(-2.0, 2.0, 0.0);
	//q.segment(0, 3) = _Vector3(0.0, 0.5 * M_PI, 0.0);
	//qdot.segment(0, 3) = _Vector3(-2.0, 0.0, 2.0);
	
	Forward();
	//jointLimit[0] = 0.785f;
	//jointLimit[0] = 0.5 * M_PI;
	//jointLimit[1] = 0.09f;

	//jointRange[0].first = 0.5 * M_PI;//swing
	jointRange[0].second = 0.5 * M_PI;//twist
	
	GameplayUtility::DrawArrow(Vector3d(0, 0, 0), Vector3d(0, -1, 0), Math::sVector(0, 0, 1), 0.5);
	
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

void eae6320::MultiBody::Integrate_q(_Vector& o_q, std::vector<_Quat>& o_quat, _Vector& i_q, std::vector<_Quat>& i_quat, _Vector& i_qdot, _Scalar h)
{
	for (int i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_3D)
		{
			o_q.segment(posStartIndex[i], 3) = i_q.segment(posStartIndex[i], 3) + i_qdot.segment(velStartIndex[i], 3) * h;
		}
		else if (jointType[i] == BALL_JOINT_4D)
		{
			Math::QuatIntegrate(o_quat[i], i_quat[i], i_qdot.segment(velStartIndex[i], 3), h);

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
			Math::QuatIntegrate(o_quat[i], i_quat[i], w_rel_local[i], h);
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
	Integrate_q(q, rel_ori, q, rel_ori, qdot, h);
	
	ClampRotationVector();
	Forward();
	//EnergyMomentumProjection();
	//ManifoldProjection();
}

void eae6320::MultiBody::RK4Integration(const _Scalar h)
{
	_Vector k1 = MrInverse * ComputeQr_SikpVelocityUpdate(qdot);
	_Vector k2 = MrInverse * ComputeQr(qdot + 0.5 * h * k1);
	_Vector k3 = MrInverse * ComputeQr(qdot + 0.5 * h * k2);
	_Vector k4 = MrInverse * ComputeQr(qdot + h * k3);

	_Vector qddot = (1.0f / 6.0f) * (k1 + 2 * k2 + 2 * k3 + k4);
	qdot = qdot + h * qddot;
	//std::cout << qdot.transpose() << std::endl;

	if (constraintSolverMode == IMPULSE)
	{
		BallJointLimitCheck();
		ResolveJointLimit(h);
		/*_BallJointLimitCheck();
		_ResolveJointLimit(h);*/
	}

	_Vector q_new(totalPosDOF);
	Integrate_q(q_new, rel_ori, q, rel_ori, qdot, h);

	if (constraintSolverMode == PBD)
	{
		ComputeHt(q_new, rel_ori);
		/*BallJointLimitCheck();
		ResolveJointLimitPBD(q_new, h);*/
		_BallJointLimitCheck();
		_ResolveJointLimitPBD(q_new, h);
	}
	q = q_new;
	//std::cout << q.transpose() << std::endl;

	ClampRotationVector();
	Forward();
	{
		_Vector3 p = _Vector3(0, -1, 0);
		_Vector3 local_x = _Vector3(1, 0, 0);

		_Matrix3 R_swing;
		_Matrix3 R_twist;
		_Vector3 twistAxis(0, -1, 0);
		Math::TwistSwingDecompsition(R_local[0], twistAxis, R_twist, R_swing);
		_Vector3 vec_twist = Math::RotationConversion_MatrixToVec(R_twist);
		_Vector3 vec_swing = Math::RotationConversion_MatrixToVec(R_swing);

		_Vector s = p.cross(R_local[0] * p);
		//std::cout << "twist: " << vec_twist.norm() << " swing: " << vec_swing.norm() << std::endl;
		/*if (jointsID.size() > 0)
		{
			_Scalar cValue = ComputeAngularVelocityConstraint(qdot, R_local[0], limitType[0], jointRange[0].second);
			std::cout << "C dot: " << cValue << " qdot: " << qdot.transpose() << std::endl << std::endl;
		}*/
	
		//std::cout << vec_swing.transpose() << std::endl << std::endl;
		//std::cout << "twist: " << vec_twist.norm() <<" swing: "<< vec_swing.norm() << " s: " << s.norm() << std::endl;
		//_Vector3 rotVec = Math::RotationConversion_MatrixToVec(R_local[0]);
		//std::cout << rel_ori[0].w() << " " << rel_ori[0].x() << " " << rel_ori[0].y() << " " << rel_ori[0].z() << std::endl;
		
		if (s.norm() < 0.05 && Physics::totalSimulationTime > 0.2)
		{
			Physics::simPause = true;
			std::cout << "paused!" << std::endl;
			//_Matrix3 Rt = R_twist * R_swing;
			//std::cout << Rt - R_local[0] << std::endl << std::endl;
			//jointRange[0].second = -1;
		}
		if (isButtonGClicked)
		{
			isButtonGClicked = false;
			std::cout << "twist: " << vec_twist.transpose() << " twist norm: " << vec_twist.norm() << " swing norm: " << vec_swing.norm() << std::endl;
			if (twistArrow != nullptr)
			{
				twistArrow->DestroyGameObject();
				twistArrow = nullptr;
			}
			twistArrow = GameplayUtility::DrawArrow(Vector3d(0, 0, 0), vec_twist.normalized(), Math::sVector(0, 1, 0), 0.5);

			/*if (swingArrow != nullptr)
			{
				swingArrow->DestroyGameObject();
				swingArrow = nullptr;
			}*/
			swingArrow = GameplayUtility::DrawArrow(Vector3d(0, 0, 0), vec_swing.normalized(), Math::sVector(1, 0, 0), 0.5);
		}
	}
}

void eae6320::MultiBody::RK3Integration(const _Scalar h)
{
	_Vector k1 = h * MrInverse * ComputeQr_SikpVelocityUpdate(qdot);
	_Vector k2 = h * MrInverse * ComputeQr(qdot + 0.5 * k1);
	_Vector k3 = h * MrInverse * ComputeQr(qdot + 2.0 * k2 - k1);

	qdot = qdot + (1.0f / 6.0f) * (k1 + 4 * k2 + k3);
	
	Integrate_q(q, rel_ori, q, rel_ori, qdot, h);

	ClampRotationVector();
	Forward();
}

void eae6320::MultiBody::ComputeH(_Vector& i_q)
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
			_Vector3 r = i_q.segment(posStartIndex[i], 3);
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

void eae6320::MultiBody::ComputeHt(_Vector& i_q, std::vector<_Quat>& i_quat)
{
	ForwardKinematics(i_q, i_quat);
	ComputeH(i_q);
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
	if (Mr.determinant() < 0.0000001)
	{
		EAE6320_ASSERTF(false, "mass matrix singluarity reached!");
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

void eae6320::MultiBody::UpdateBodyRotation(_Vector& i_q, std::vector<_Quat>& i_quat)
{
	for (size_t i = 0; i < numOfLinks; i++)
	{
		//update orientation
		if (jointType[i] == BALL_JOINT_4D)
		{
			if (i == 0)
			{
				obs_ori[i] = i_quat[i];
			}
			else
			{
				obs_ori[i] = obs_ori[i - 1] * i_quat[i];
			}
			R_local[i] = i_quat[i].toRotationMatrix();
			R_global[i] = obs_ori[i].toRotationMatrix();
			m_linkBodys[i]->m_State.orientation = Math::ConvertEigenQuatToNativeQuat(obs_ori[i]);
		}
		else if (jointType[i] == BALL_JOINT_3D)
		{
			_Vector3 r = i_q.segment(posStartIndex[i], 3);

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
			obs_ori[i] = i_quat[i];
			R_local[i] = i_quat[i];
			R_global[i] = obs_ori[i].toRotationMatrix();
			m_linkBodys[i]->m_State.orientation = Math::ConvertEigenQuatToNativeQuat(obs_ori[i]);
		}
	}
}

void eae6320::MultiBody::ForwardKinematics(_Vector& i_q, std::vector<_Quat>& i_quat)
{
	UpdateBodyRotation(i_q, i_quat);

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
			pos[i] = i_q.segment(posStartIndex[i], 3);
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
	ComputeHt(q, rel_ori);
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

_Scalar eae6320::MultiBody::ComputeAngularVelocityConstraint(_Vector3 w, _Matrix3& Rot, int i_limitType, _Scalar phi)
{
	_Scalar C = 0;
	_Vector3 p = _Vector3(0, -1, 0);
	_Vector3 s = _Vector3(1, 0, 0);
	if (i_limitType == TWIST_WITH_SWING)
	{
		_Scalar t0, t1, t2, d0, d1;
		t0 = (Math::ToSkewSymmetricMatrix(p) * Math::ToSkewSymmetricMatrix(w) * Rot * p).dot(Rot * Math::ToSkewSymmetricMatrix(p) * Rot * p);
		t1 = (Math::ToSkewSymmetricMatrix(p) * Rot * p).dot(Math::ToSkewSymmetricMatrix(w) * Rot * Math::ToSkewSymmetricMatrix(p) * Rot * p);
		t2 = (Math::ToSkewSymmetricMatrix(p) * Rot * p).dot(Rot * Math::ToSkewSymmetricMatrix(p) * Math::ToSkewSymmetricMatrix(w) * Rot * p);
		d0 = t0 + t1 + t2;
		d1 = 2 * (Math::ToSkewSymmetricMatrix(p) * Math::ToSkewSymmetricMatrix(w) * Rot * p).dot(Math::ToSkewSymmetricMatrix(p) * Rot * p);

		_Vector s = p.cross(Rot * p);
		_Scalar t3 = s.dot(Rot * s);

		C = d0 / s.squaredNorm() - t3 * d1 / (s.squaredNorm() * s.squaredNorm());
	}
	else if (i_limitType == TWIST_WITHOUT_SWING)
	{
		C = s.dot(Math::ToSkewSymmetricMatrix(w) * Rot * s);
	}
	else if (i_limitType == SWING)
	{
		C = p.dot(Math::ToSkewSymmetricMatrix(w) * Rot * p);
	}
	return C;
}

void eae6320::MultiBody::BallJointLimitCheck()
{
	jointsID.clear();
	constraintValue.clear();
	limitType.clear();
	_Vector3 p = _Vector3(0, -1, 0);
	_Vector3 local_x = _Vector3(1, 0, 0);
	
	for (int i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_4D)
		{
			if (jointRange[i].first > 0)//check swing constraint
			{
				_Scalar c = p.dot(R_local[i] * p) - cos(jointRange[i].first);
				if (c < 0)
				{
					jointsID.push_back(i);
					constraintValue.push_back(c);
					limitType.push_back(SWING);
				}
			}

			if (jointRange[i].second > 0) //check twist constraint
			{
				_Vector s = p.cross(R_local[i] * p);
				if (s.norm() < swingEpsilon)
				{
					_Scalar c = local_x.dot(R_local[i] * local_x) - cos(jointRange[i].second);
					if (c < 0)
					{
						jointsID.push_back(i);
						constraintValue.push_back(c);
						limitType.push_back(TWIST_WITHOUT_SWING);
					}
				}
				else
				{
					_Scalar c = s.dot(R_local[i] * s) / s.squaredNorm() - cos(jointRange[i].second);
					if (c < 0)
					{
						jointsID.push_back(i);
						constraintValue.push_back(c);
						limitType.push_back(TWIST_WITH_SWING);
					}
				}
			}
		}
	}
}

void eae6320::MultiBody::ResolveJointLimit(const _Scalar h)
{
	size_t constraintNum = jointsID.size();
	_Vector3 p = _Vector3(0, -1, 0);
	_Vector3 local_x = _Vector3(1, 0, 0);
	if (constraintNum > 0)
	{
		_Matrix J;
		J.resize(constraintNum, totalVelDOF);
		J.setZero();
		_Matrix K;
		K = J;
		_Vector bias;
		bias.resize(constraintNum);
		bias.setZero();
		for (int k = 0; k < constraintNum; k++)
		{
			int i = jointsID[k];
			//compute J and K
			if (jointType[i] == BALL_JOINT_4D)
			{
				if (limitType[k] == TWIST_WITH_SWING || limitType[k] == TWIST_WITHOUT_SWING)
				{
					//compute J
					_Scalar j0 = ComputeAngularVelocityConstraint(_Vector3(1, 0, 0), R_local[i], limitType[k], jointRange[i].second);
					_Scalar j1 = ComputeAngularVelocityConstraint(_Vector3(0, 1, 0), R_local[i], limitType[k], jointRange[i].second);
					_Scalar j2 = ComputeAngularVelocityConstraint(_Vector3(0, 0, 1), R_local[i], limitType[k], jointRange[i].second);
					J.block<1, 3>(k, velStartIndex[i]) = _Vector3(j0, j1, j2);

					//compute K
					_Vector3 pRotated = R_local[i] * p;
					_Scalar cTest = ComputeAngularVelocityConstraint(pRotated, R_local[i], limitType[k], jointRange[i].second);
					if (cTest < 0)
					{
						pRotated = -pRotated;
					}
					K.block<1, 3>(k, velStartIndex[i]) = pRotated;
					//K.block<1, 3>(k, velStartIndex[i]) = J.block<1, 3>(k, velStartIndex[i]);
					_Scalar cValue = ComputeAngularVelocityConstraint(qdot, R_local[i], limitType[k], jointRange[i].second);
					//std::cout << "after velocity integration: " << cValue << " qdot: " << qdot.transpose() << std::endl;
				}
				else if (limitType[k] == SWING)
				{
					//compute J
					_Scalar j0 = ComputeAngularVelocityConstraint(_Vector3(1, 0, 0), R_local[i], limitType[k], jointRange[i].first);
					_Scalar j1 = ComputeAngularVelocityConstraint(_Vector3(0, 1, 0), R_local[i], limitType[k], jointRange[i].first);
					_Scalar j2 = ComputeAngularVelocityConstraint(_Vector3(0, 0, 1), R_local[i], limitType[k], jointRange[i].first);
					J.block<1, 3>(k, velStartIndex[i]) = _Vector3(j0, j1, j2);

					//compute K
					K.block<1, 3>(k, velStartIndex[i]) = _Vector3(j0, j1, j2);
				}
			}

			//compute bias
		/*	_Vector3 v = qdot.segment(velStartIndex[i], 3);
			_Matrix C_dot = J.block<1, 3>(k, velStartIndex[i]) * v;
			_Scalar beta = 0.1f;
			_Scalar CR = 0.4f;
			_Scalar SlopP = 0.001f;
			bias(k) = -beta / h * std::max<_Scalar>(-constraintValue[i] - SlopP, 0.0) - CR * std::max<_Scalar>(-C_dot(0, 0), 0.0);*/
		}
		_Matrix lambda;
		lambda = (J * MrInverse * K.transpose()).inverse() * (-J * qdot - bias);
		//std::cout << J * qdot << std::endl;
		for (int k = 0; k < constraintNum; k++)
		{
			if (lambda(k, 0) < 0)
			{
				lambda(k, 0) = 0;
			}
		}
		_Vector qdotCorrection = MrInverse * K.transpose() * lambda;
		qdot = qdot + qdotCorrection;
		_Scalar myC = ComputeAngularVelocityConstraint(qdot, R_local[0], limitType[0], jointRange[0].second);
		//_Matrix myC2 = J * qdot;
		//std::cout << "after: " << myC << std::endl;
	}
}

void eae6320::MultiBody::UpdateGameObjectBasedOnInput()
{
	if (UserInput::IsKeyFromReleasedToPressed('G'))
	{
		isButtonGClicked = true;
	}
}
