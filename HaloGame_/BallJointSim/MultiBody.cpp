#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>

eae6320::MultiBody::MultiBody(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State):
	GameCommon::GameObject(i_pEffect, i_Mesh, i_State)
{
	//UnitTest1(); //test swing twist decomposition with rotaion matrix
	//UnitTest2(); //test extreme case for twist with single body
	//UnitTest3();//chain mimic with two bodies
	//UnitTest4();//angular velocity of _Vector3(-2.0, 2.0, 0.0) for the 2nd body
	//UnitTest5();//test induced twist for single body
	//UnitTest6();//twist invariance for single body
	//UnitTest7();//test swing twist decomp with quat
	//UnitTest8(); //mujoco ball joint constraint test for single body
	//UnitTest9();//swing for 3d ball joint
	//UnitTest10();//twist invarience two bodies.
	//UnitTest11();//5 body
	//UnitTest12();//2 cube
	//HingeJointUnitTest0();//hinge joint with auto constraint
	UnitTest13();//vector vield switch test
	//UnitTest14();//5 body for Euler twist
	
	kineticEnergy0 = ComputeKineticEnergy();
	totalEnergy0 = ComputeTotalEnergy();
	angularMomentum0 = ComputeAngularMomentum();
	linearMomentum0 = ComputeTranslationalMomentum();
	std::cout << "initial total energy: " << totalEnergy0 << std::endl;
	std::cout << "initial angular momentum: " << angularMomentum0.transpose() << std::endl;
	std::cout << "initial linear momentum: " << linearMomentum0.transpose() << std::endl;
}

void eae6320::MultiBody::SetZeroInitialCondition()
{
	q.resize(totalPosDOF);
	q.setZero();
	qdot.resize(totalVelDOF);
	qdot.setZero();
	x.resize(totalXDOF);
}

void eae6320::MultiBody::InitializeJoints(int* i_jointType)
{
	Math::NativeVector2EigenVector(m_State.position, jointPos[0]);
	for (int i = 0; i < numOfLinks; i++)
	{
		jointType[i] = i_jointType[i];
		xJointType[i] = i_jointType[i];
		if (jointType[i] == BALL_JOINT_3D)
		{
			velDOF[i] = 3;
			posDOF[i] = 3;
			totalVelDOF += 3;
			totalPosDOF += 3;

			xDOF[i] = 3;
			totalXDOF += 3;
		}
		else if (jointType[i] == BALL_JOINT_4D)
		{
			velDOF[i] = 3;
			posDOF[i] = 4;
			totalVelDOF += 3;
			totalPosDOF += 4;
	
			xDOF[i] = 3;
			totalXDOF += 3;

			xJointType[i] = BALL_JOINT_3D;
		}
		else if (jointType[i] == FREE_JOINT)
		{
			velDOF[i] = 6;
			posDOF[i] = 7;
			totalVelDOF += 6;
			totalPosDOF += 7;

			xDOF[i] = 6;
			totalXDOF += 6;
		}
		else if (jointType[i] == HINGE_JOINT)
		{
			velDOF[i] = 1;
			posDOF[i] = 1;
			totalVelDOF += 1;
			totalPosDOF += 1;

			xDOF[i] = 1;
			totalXDOF += 1;
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
			xStartIndex[i] = xStartIndex[i - 1] + xDOF[i - 1];
		}
	}
	Mr.resize(totalVelDOF, totalVelDOF);
}

void eae6320::MultiBody::ConfigurateBallJoint(_Vector3& xAxis, _Vector3& yAxis, _Vector3& zAxis, _Scalar swingAngle, _Scalar twistAngle)
{
	for (int i = 0; i < numOfLinks; i++)
	{
		eulerX[i] = xAxis;
		eulerY[i] = yAxis;
		eulerZ[i] = zAxis;
		oldEulerZ[i] = zAxis;
		twistAxis[i] = xAxis;

		jointRange[i].first = swingAngle;//swing
		jointRange[i].second = twistAngle;//twist
	}
}

void eae6320::MultiBody::InitializeBodies(Assets::cHandle<Mesh> i_mesh, Vector3d i_meshScale, _Matrix3& i_localInertiaTensor, _Vector3 i_partentJointPosition, _Vector3 i_childJointPosition)
{
	for (int i = 0; i < numOfLinks; i++)
	{
		GameCommon::GameObject *pGameObject = new GameCommon::GameObject(defaultEffect, i_mesh, Physics::sRigidBodyState());
		pGameObject->scale = i_meshScale;
		m_linkBodys.push_back(pGameObject);
	}

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
	xDOF.resize(numOfLinks);
	xStartIndex.resize(numOfLinks);
	xJointType.resize(numOfLinks);
	jointLimit.resize(numOfLinks);
	jointRange.resize(numOfLinks);
	hingeDirLocals.resize(numOfLinks);
	hingeDirGlobals.resize(numOfLinks);
	hingeMagnitude.resize(numOfLinks);
	twistAxis.resize(numOfLinks);
	eulerX.resize(numOfLinks);
	eulerY.resize(numOfLinks);
	eulerZ.resize(numOfLinks);
	oldEulerZ.resize(numOfLinks);
	oldEulerAngle2.resize(numOfLinks);
	oldEulerAngle0.resize(numOfLinks);
	mAlpha.resize(numOfLinks);
	mBeta.resize(numOfLinks);
	mGamma.resize(numOfLinks);
	vectorFieldNum.resize(numOfLinks);
	vectorFieldSwitched.resize(numOfLinks);
	eulerDecompositionOffset.resize(numOfLinks);
	lastTwistAxis.resize(numOfLinks);
	userToLocalTransform.resize(numOfLinks);
	for (int i = 0; i < numOfLinks; i++)
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

		localInertiaTensors[i] = i_localInertiaTensor;
		Mbody[i].block<3, 3>(3, 3) = i_localInertiaTensor;

		std::vector<_Vector3> uPairs;
		uPairs.resize(2);
		uPairs[0] = i_partentJointPosition;
		uPairs[1] = i_childJointPosition;
		uLocals.push_back(uPairs);
		uGlobals.push_back(uPairs);

		vectorFieldNum[i] = 0;
		vectorFieldSwitched[i] = FALSE;
		twistAxis[i] = _Vector3(0, -1, 0);
		eulerX[i] = _Vector3(0, -1, 0);
		eulerY[i] = _Vector3(0, 0, 1);
		eulerZ[i] = _Vector3(-1, 0, 0);
		oldEulerZ[i] = _Vector3(-1, 0, 0);

		_Matrix3 deformationGradient;
		Math::ComputeDeformationGradient(eulerY[i], eulerZ[i], eulerX[i], _Vector3(0, 1, 0), _Vector3(0, 0, 1), _Vector3(1, 0, 0), deformationGradient);
		eulerDecompositionOffset[i] = Math::RotationConversion_MatToQuat(deformationGradient);

		Math::ComputeDeformationGradient(eulerX[i], eulerY[i], eulerZ[i], _Vector3(0, -1, 0), _Vector3(0, 0, 1), _Vector3(-1, 0, 0), deformationGradient);
		userToLocalTransform[i] = deformationGradient;
		lastTwistAxis[i] = eulerX[i];

		_Scalar eulerAngles[3];
		_Quat inputQuat = eulerDecompositionOffset[i] * rel_ori[i] * eulerDecompositionOffset[i].inverse();
		Math::quaternion2Euler(inputQuat, eulerAngles, Math::RotSeq::yzx);
		oldEulerAngle0[i] = eulerAngles[0];
		oldEulerAngle2[i] = eulerAngles[2];
		//std::cout << "Twsit angle: " << eulerAngles[0] << " " << eulerAngles[1] << " " << eulerAngles[2] << " rotatedX: " << rotatedX(0) << " " << rotatedX(1) << std::endl;
	}
}

void eae6320::MultiBody::Tick(const double i_secondCountToIntegrate)
{	
	dt = (_Scalar)i_secondCountToIntegrate;
	_Scalar t = (_Scalar)eae6320::Physics::totalSimulationTime;
	LOG_TO_FILE << t << " " << pos[0].transpose() << " " << mAlpha[0] << " " << mBeta[0] << " " << mGamma[0] << std::endl;
	
	EulerIntegration(dt);
	//RK3Integration(dt);
	//RK4Integration(dt);
	
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
	for (int i = 0; i < numOfLinks; i++)
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
		else if (jointType[i] == HINGE_JOINT)
		{
			o_q(posStartIndex[i]) = i_q(posStartIndex[i]) + i_qdot(velStartIndex[i]) * h;
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
	if (constraintSolverMode == IMPULSE)
	{
		BallJointLimitCheck();
		SolveVelocityJointLimit(h);
	}

	//KineticEnergyProjection();
	//MomentumProjection();
	//EnergyMomentumProjection();
	if (enablePositionSolve)
	{
		SolvePositionJointLimit();
	}
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
	//std::cout << qddot.transpose() << std::endl;

	if (constraintSolverMode == IMPULSE)
	{
		BallJointLimitCheck();
		SolveVelocityJointLimit(h);
		/*_BallJointLimitCheck();
		_ResolveJointLimit(h);*/
	}

	_Vector q_new(totalPosDOF);
	Integrate_q(q_new, rel_ori, q, rel_ori, qdot, h);

	if (constraintSolverMode == PBD)
	{
		ComputeHt(q_new, rel_ori);
		_BallJointLimitCheck();
		_ResolveJointLimitPBD(q_new, h);
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
	
	Integrate_q(q, rel_ori, q, rel_ori, qdot, h);

	ClampRotationVector();
	Forward();
}

void eae6320::MultiBody::ComputeHt(_Vector& i_q, std::vector<_Quat>& i_quat)
{
	ForwardKinematics(i_q, i_quat);
	for (int i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_4D)
		{
			//compute H
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
			//compute D
			if (i > 0)
			{
				D[i].resize(6, 6);
				D[i].setIdentity();
				D[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) - Math::ToSkewSymmetricMatrix(uGlobals[i - 1][1]);
			}
		}
		else if (jointType[i] == BALL_JOINT_3D)
		{
			//compute H
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
			//compute D
			if (i > 0)
			{
				D[i].resize(6, 6);
				D[i].setIdentity();
				D[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) - Math::ToSkewSymmetricMatrix(uGlobals[i - 1][1]);
			}
		}
		else if (jointType[i] == FREE_JOINT)
		{
			//compute H
			H[i].resize(6, 6);
			H[i].setIdentity();
			//compute D
			D[i].resize(6, 6);
			D[i].setZero();
		}
		else if (jointType[i] == HINGE_JOINT)
		{
			//compute H
			H[i].resize(6, 1);
			H[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * hingeDirGlobals[i];
			H[i].block<3, 1>(3, 0) = hingeDirGlobals[i];
			//compute D
			D[i].resize(6, 6);
			D[i].setIdentity();
			if (i > 0)
			{
				_Vector3 hingeVec = hingeMagnitude[i] * hingeDirGlobals[i];
				_Vector3 iVec = uGlobals[i][0] - uGlobals[i - 1][1] - hingeVec;
				D[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(iVec);
			}
		}
		//compose Ht
		Ht[i].resize(6, totalVelDOF);
		Ht[i].setZero();
		for (int k = 0; k <= i; k++)
		{
			_Matrix H_temp;
			H_temp.resize(6, 3);
			H_temp = H[k];
			for (int j = k + 1; j <= i; j++)
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
	for (int i = 0; i < numOfLinks; i++)
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
		else if (jointType[i] == FREE_JOINT)
		{
			gamma[i].resize(6);
			gamma[i].setZero();
		}
		else if (jointType[i] == HINGE_JOINT)
		{
			_Vector3 gamma_theta;
			gamma_theta.setZero();
			if (i > 0)
			{
				gamma_theta += Math::ToSkewSymmetricMatrix(w_abs_world[i - 1]) * hingeDirGlobals[i] * i_qdot(velStartIndex[i]);
			}
			_Vector3 gamma_r;
			gamma_r = -w_abs_world[i].cross(w_abs_world[i].cross(uGlobals[i][0]));
			_Vector3 hingeVec = hingeMagnitude[i] * hingeDirGlobals[i];
			if (i > 0)
			{
				gamma_r += w_abs_world[i - 1].cross(w_abs_world[i - 1].cross(uGlobals[i - 1][1] + hingeVec));
			}
			gamma[i].resize(6);
			gamma[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * gamma_theta + gamma_r;
			gamma[i].block<3, 1>(3, 0) = gamma_theta;
		}
	}

	o_gamma_t.resize(numOfLinks);
	for (int i = 0; i < numOfLinks; i++)
	{
		o_gamma_t[i].resize(6);
		o_gamma_t[i].setZero();
		for (int j = 0; j <= i; j++)
		{
			_Vector gamma_temp;
			gamma_temp.resize(6);
			gamma_temp = gamma[j];
			for (int k = j + 1; k <= i; k++)
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
	for (int i = 0; i < numOfLinks; i++)
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
			R_local[i] = AngleAxisf(r.norm(), r.normalized());
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
			R_local[i] = obs_ori[i].toRotationMatrix();
			R_global[i] = obs_ori[i].toRotationMatrix();
			m_linkBodys[i]->m_State.orientation = Math::ConvertEigenQuatToNativeQuat(obs_ori[i]);
		}
		else if (jointType[i] == HINGE_JOINT)
		{
			if (i > 0) hingeDirGlobals[i] = R_global[i - 1] * hingeDirLocals[i];
			_Scalar angle = i_q(posStartIndex[i]);
#if defined (HIGH_PRECISION_MODE)
			R_local[i] = AngleAxisd(angle, hingeDirGlobals[i]);
#else
			R_local[i] = AngleAxisf(angle, axis);
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
	}
}

void eae6320::MultiBody::ForwardKinematics(_Vector& i_q, std::vector<_Quat>& i_quat)
{
	UpdateBodyRotation(i_q, i_quat);

	_Vector3 preAnchor = jointPos[0];
	for (int i = 0; i < numOfLinks; i++)
	{
		//update position
		uGlobals[i][0] = R_global[i] * uLocals[i][0];
		if (jointType[i] == BALL_JOINT_3D || jointType[i] == BALL_JOINT_4D)
		{
			pos[i] = preAnchor - uGlobals[i][0];
		}
		else if (jointType[i] == FREE_JOINT)
		{
			pos[i] = i_q.segment(posStartIndex[i], 3);
		}
		else if (jointType[i] == HINGE_JOINT)
		{
			pos[i] = preAnchor + hingeMagnitude[i] * hingeDirGlobals[i] - uGlobals[i][0];
		}

		//update Euler angles
		_Scalar eulerAngles[3];
		GetEulerAngles(i, eulerAngles);
		mAlpha[i] = eulerAngles[2];
		mBeta[i] = eulerAngles[1];
		mGamma[i] = eulerAngles[0];
		
		//get ready for the next iteration
		uGlobals[i][1] = R_global[i] * uLocals[i][1];
		preAnchor = pos[i] + uGlobals[i][1];
		if (i + 1 < numOfLinks) jointPos[i + 1] = preAnchor;

		//update render position
		m_linkBodys[i]->m_State.position = Math::sVector((float)pos[i](0), (float)pos[i](1), (float)pos[i](2));
		
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

void eae6320::MultiBody::UpdateGameObjectBasedOnInput()
{
}
