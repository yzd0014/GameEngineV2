#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>

eae6320::MultiBody::MultiBody(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, Application::cbApplication* i_application):
	GameCommon::GameObject(i_pEffect, i_Mesh, i_State)
{
	RunUnitTest();
	
	UpdateInitialPosition();
	pApp = i_application;

	kineticEnergy0 = ComputeKineticEnergy();
	totalEnergy0 = ComputeTotalEnergy();
	angularMomentum0 = ComputeAngularMomentum();
	linearMomentum0 = ComputeTranslationalMomentum();
	std::cout << "initial total energy: " << totalEnergy0 << std::endl;
	//std::cout << "initial angular momentum: " << angularMomentum0.transpose() << std::endl;
	//std::cout << "initial linear momentum: " << linearMomentum0.transpose() << std::endl;
}

void eae6320::MultiBody::MultiBodyInitialization()
{
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
	xDOF.resize(numOfLinks);
	xStartIndex.resize(numOfLinks);
	//xJointType.resize(numOfLinks);
	jointLimit.resize(numOfLinks);
	jointRange.resize(numOfLinks);
	hingeDirLocals.resize(numOfLinks);
	hingeDirGlobals.resize(numOfLinks);
	hingeMagnitude.resize(numOfLinks);
	twistAxis.resize(numOfLinks);
	eulerX.resize(numOfLinks);
	eulerY.resize(numOfLinks);
	eulerZ.resize(numOfLinks);
	mAlpha.resize(numOfLinks);
	mBeta.resize(numOfLinks);
	mGamma.resize(numOfLinks);
	vectorFieldNum.resize(numOfLinks);
	eulerDecompositionOffset.resize(numOfLinks);
	lastValidOri.resize(numOfLinks);
	eulerDecompositionOffsetMat.resize(numOfLinks);
	totalTwist.resize(numOfLinks);
	old_R_local.resize(numOfLinks);
	externalForces.resize(numOfLinks);
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
		
		jointLimit[i] = -1;
		std::pair<_Scalar, _Scalar> defaultRange(-1, -1);
		jointRange[i] = defaultRange;

		externalForces[i].resize(6);
		externalForces[i].setZero();

		totalTwist[i] = 0;
		old_R_local[i].setIdentity();
		lastValidOri[i].setIdentity();

		mAlpha[i] = 0;
		mBeta[i] = 0;
		mGamma[i] = 0;

		vectorFieldNum[i] = 0;
		twistAxis[i] = _Vector3(0, -1, 0);
		eulerX[i] = _Vector3(0, -1, 0);
		eulerY[i] = _Vector3(0, 0, 1);
		eulerZ[i] = _Vector3(-1, 0, 0);

		_Matrix3 deformationGradient;
		Math::ComputeDeformationGradient(eulerY[i], eulerZ[i], eulerX[i], _Vector3(0, 1, 0), _Vector3(0, 0, 1), _Vector3(1, 0, 0), deformationGradient);
		eulerDecompositionOffsetMat[i] = deformationGradient;
		eulerDecompositionOffset[i] = Math::RotationConversion_MatToQuat(deformationGradient);
	}
	Math::NativeVector2EigenVector(m_State.position, jointPos[0]);
	Mr.resize(totalVelDOF, totalVelDOF);
	q.resize(totalPosDOF);
	q.setZero();
	qdot.resize(totalVelDOF);
	qdot.setZero();
	x.resize(totalXDOF);
}

void eae6320::MultiBody::ConfigurateBallJoint(_Vector3& xAxis, _Vector3& yAxis, _Vector3& zAxis, _Scalar swingAngle, _Scalar twistAngle)
{
	for (int i = 0; i < numOfLinks; i++)
	{
		ConfigureSingleBallJoint(i, xAxis, yAxis, zAxis, swingAngle, twistAngle);
	}
}

void eae6320::MultiBody::ConfigureSingleBallJoint(int bodyNum, _Vector3& xAxis, _Vector3& yAxis, _Vector3& zAxis, _Scalar swingAngle, _Scalar twistAngle)
{
	int i = bodyNum;
	eulerX[i] = xAxis;
	eulerY[i] = yAxis;
	eulerZ[i] = zAxis;
	twistAxis[i] = xAxis;

	jointRange[i].first = swingAngle;//swing
	jointRange[i].second = twistAngle;//twist

	_Matrix3 deformationGradient;
	Math::ComputeDeformationGradient(eulerY[i], eulerZ[i], eulerX[i], _Vector3(0, 1, 0), _Vector3(0, 0, 1), _Vector3(1, 0, 0), deformationGradient);
	eulerDecompositionOffsetMat[i] = deformationGradient;
	eulerDecompositionOffset[i] = Math::RotationConversion_MatToQuat(deformationGradient);
}

void eae6320::MultiBody::Tick(const double i_secondCountToIntegrate)
{	
	if (adaptiveTimestep) pApp->UpdateDeltaTime(pApp->GetSimulationUpdatePeriod_inSeconds());
	dt = (_Scalar)i_secondCountToIntegrate;
	//SaveDataToMatlab(6);
	//SaveDataToHoudini(animationDuration, frameNum);
	ResetExternalForces();
	m_control();
	EulerIntegration(dt);
	//RK3Integration(dt);
	//RK4Integration(dt);

	//std::cout << mGamma[0] << std::endl;
	_Vector3 momentum = ComputeTranslationalMomentum();
	_Vector3 angularMomentum = ComputeAngularMomentum();
	//std::cout << "angluar:" << std::setw(15) << angularMomentum.transpose() << std::endl;
	_Vector3 momErr = angularMomentum - angularMomentum0;
	//std::cout << "angluar norm: " << angularMomentum.norm() << std::endl;
	//std::cout << std::left << "angular mom err: " << std::setw(15) << momErr.transpose() << std::endl << std::endl;
	//std::cout << std::left 
	//	<< "tran:" << std::setw(15) << momentum.transpose()
	//	<< "angluar:" << std::setw(15) << angularMomentum.transpose() << std::endl;
	//std::cout << Physics::totalSimulationTime << " " << ComputeKineticEnergy() << std::endl << std::endl;
	//std::cout << Physics::totalSimulationTime << " " << ComputeTotalEnergy() << std::endl << std::endl;
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
	_Vector Qr = ComputeQr_SikpVelocityUpdate(qdot);
	_Vector qddot = MrInverse * Qr;

	qdot = qdot + qddot * h;
	qdot = damping * qdot;
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
	
	//EnergyConstraint();
	//EnergyConstraintV2();
	//totalEnergy0 = ComputeTotalEnergy();
	kineticEnergy0 = ComputeKineticEnergy();
	linearMomentum0 = ComputeTranslationalMomentum();
	angularMomentum0 = ComputeAngularMomentum();
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
		BallJointLimitCheck();
		SolveVelocityJointLimit(h);
	}

	if (enablePositionSolve)
	{
		SolvePositionJointLimit();
	}
	Integrate_q(q, rel_ori, q, rel_ori, qdot, h);
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
		int j = parentArr[i];
		if (jointType[i] == BALL_JOINT_4D)
		{
			//compute H
			H[i].resize(6, 3);
			H[i].setZero();
			if (i == 0)
			{
				//H[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]);
				H[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobalsParent[i]);
				H[i].block<3, 3>(3, 0) = _Matrix::Identity(3, 3);
			}
			else
			{
				//H[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * R_global[i - 1];
				H[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobalsParent[i]) * R_global[j];
				H[i].block<3, 3>(3, 0) = R_global[j];
			}
			//compute D
			if (i > 0)
			{
				D[i].resize(6, 6);
				D[i].setIdentity();
				//D[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) - Math::ToSkewSymmetricMatrix(uGlobals[i - 1][1]);
				D[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(uGlobalsParent[i]) - Math::ToSkewSymmetricMatrix(uGlobalsChild[i]);
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
			else A = R_global[j] * J_rotation[i];
			H[i].resize(6, 3);
			H[i].setZero();
			H[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * A;
			H[i].block<3, 3>(3, 0) = A;
			//compute D
			if (i > 0)
			{
				D[i].resize(6, 6);
				D[i].setIdentity();
				//D[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) - Math::ToSkewSymmetricMatrix(uGlobals[i - 1][1]);
				D[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(uGlobalsParent[i]) - Math::ToSkewSymmetricMatrix(uGlobalsChild[i]);
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
			H[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * hingeDirGlobals[i];//TODO
			H[i].block<3, 1>(3, 0) = hingeDirGlobals[i];
			//compute D
			D[i].resize(6, 6);
			D[i].setIdentity();
			if (i > 0)
			{
				_Vector3 hingeVec = hingeMagnitude[i] * hingeDirGlobals[i];
				_Vector3 iVec = uGlobals[i][0] - uGlobals[j][1] - hingeVec;//TODO
				D[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(iVec);
			}
		}
		//compose Ht
		Ht[i].resize(6, totalVelDOF);
		Ht[i].setZero();
		int k = i;
		while (k != -1)
		{
			_Matrix D_temp;
			D_temp.resize(6, 6);
			D_temp.setIdentity();
			int j = i;
			while (j > k)
			{
				D_temp = D_temp * D[j];
				j = parentArr[j];
			}
			Ht[i].block(0, velStartIndex[k], 6, velDOF[k]) = D_temp * H[k];
			k = parentArr[k];
		}
		/*for (int k = 0; k <= i; k++)
		{
			_Matrix H_temp;
			H_temp.resize(6, 3);
			H_temp = H[k];
			for (int j = k + 1; j <= i; j++)
			{
				H_temp = D[j] * H_temp;
			}
			Ht[i].block(0, velStartIndex[k], 6, velDOF[k]) = H_temp;
		}*/
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
		if (gravity)
		{
			externalForces[i].block<3, 1>(0, 0) = externalForces[i].block<3, 1>(0, 0) + _Vector3(0.0f, -9.81f, 0.0f);
		}
		_Vector Fv;
		Fv.resize(6);
		Fv.setZero();
		Fv.block<3, 1>(3, 0) = -w_abs_world[i].cross(Mbody[i].block<3, 3>(3, 3) * w_abs_world[i]);
		_Vector Q_temp;
		Q_temp.resize(totalVelDOF);
		Q_temp.setZero();
		Q_temp = Ht[i].transpose() * (externalForces[i] + Fv - Mbody[i] * gamma_t[i]);
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
		int j = parentArr[i];
		if (jointType[i] == BALL_JOINT_4D)
		{
			_Vector3 r_dot = i_qdot.segment(velStartIndex[i], 3);
			_Vector3 gamma_theta;
			gamma_theta.setZero();
			if (i > 0)
			{
				gamma_theta = Math::ToSkewSymmetricMatrix(w_abs_world[j]) * R_global[j] * r_dot;
			}
			
			gamma[i].resize(6);
			gamma[i].setZero();
			if (i == 0)
			{
				//gamma[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * gamma_theta - w_abs_world[i].cross(w_abs_world[i].cross(uGlobals[i][0]));
				gamma[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobalsParent[i]) * gamma_theta - w_abs_world[i].cross(w_abs_world[i].cross(uGlobalsParent[i]));
			}
			else
			{
				//gamma[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * gamma_theta - w_abs_world[i].cross(w_abs_world[i].cross(uGlobals[i][0])) + w_abs_world[i - 1].cross(w_abs_world[i - 1].cross(uGlobals[i - 1][1]));
				gamma[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobalsParent[i]) * gamma_theta - w_abs_world[i].cross(w_abs_world[i].cross(uGlobalsParent[i])) + w_abs_world[j].cross(w_abs_world[j].cross(uGlobalsChild[i]));
			}
			gamma[i].block<3, 1>(3, 0) = gamma_theta;
		}
		else if (jointType[i] == BALL_JOINT_3D)//TODO
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
				gamma_theta = Math::ToSkewSymmetricMatrix(w_abs_world[j]) * R_global[j] * J_rotation[i] * r_dot + R_global[j] * Jdot_rdot;
			}
			gamma[i].resize(6);
			gamma[i].setZero();
			if (i == 0)
			{
				gamma[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * gamma_theta - w_abs_world[i].cross(w_abs_world[i].cross(uGlobals[i][0]));
			}
			else
			{
				gamma[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobals[i][0]) * gamma_theta - w_abs_world[i].cross(w_abs_world[i].cross(uGlobals[i][0])) + w_abs_world[j].cross(w_abs_world[j].cross(uGlobals[j][1]));
			}
			gamma[i].block<3, 1>(3, 0) = gamma_theta;
		}
		else if (jointType[i] == FREE_JOINT)
		{
			gamma[i].resize(6);
			gamma[i].setZero();
		}
		else if (jointType[i] == HINGE_JOINT)//TODO
		{
			_Vector3 gamma_theta;
			gamma_theta.setZero();
			if (i > 0)
			{
				gamma_theta += Math::ToSkewSymmetricMatrix(w_abs_world[j]) * hingeDirGlobals[i] * i_qdot(velStartIndex[i]);
			}
			_Vector3 gamma_r;
			gamma_r = -w_abs_world[i].cross(w_abs_world[i].cross(uGlobals[i][0]));
			_Vector3 hingeVec = hingeMagnitude[i] * hingeDirGlobals[i];
			if (i > 0)
			{
				gamma_r += w_abs_world[j].cross(w_abs_world[j].cross(uGlobals[j][1] + hingeVec));
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
		int j = i;
		while (j != -1)
		{
			_Matrix D_temp;
			D_temp.resize(6, 6);
			D_temp.setIdentity();
			int k = i;
			while (k >= j + 1)
			{
				D_temp = D_temp * D[k];
				k = parentArr[k];
			}
			o_gamma_t[i] = o_gamma_t[i] + D_temp * gamma[j];
			j = parentArr[j];
		}
	/*	for (int j = 0; j <= i; j++)
		{
			_Vector gamma_temp;
			gamma_temp.resize(6);
			gamma_temp = gamma[j];
			for (int k = j + 1; k <= i; k++)
			{
				gamma_temp = D[k] * gamma_temp;
			}
			o_gamma_t[i] = o_gamma_t[i] + gamma_temp;
		}*/
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
		old_R_local[i] = R_local[i];
		int j = parentArr[i];
		//update orientation
		if (jointType[i] == BALL_JOINT_4D)
		{
			if (i == 0)
			{
				obs_ori[i] = i_quat[i];
			}
			else
			{
				obs_ori[i] = obs_ori[j] * i_quat[i];
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
				int j = parentArr[i];
				R_global[i] = R_global[j] * R_local[i];
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
			int j = parentArr[i];
			if (i > 0) hingeDirGlobals[i] = R_global[j] * hingeDirLocals[i];
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
				R_global[i] = R_global[j] * R_local[i];
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
		int j = parentArr[i];
		//update position
		//uGlobals[i][0] = R_global[i] * uLocals[i][0];
		uGlobalsParent[i] = R_global[i] * uLocalsParent[i];
		if (i > 0)
		{
			uGlobalsChild[i] = R_global[j] * uLocalsChild[i];
			jointPos[i] = pos[j] + uGlobalsChild[i];
		}
		if (jointType[i] == BALL_JOINT_3D || jointType[i] == BALL_JOINT_4D)
		{
			//pos[i] = preAnchor - uGlobals[i][0];
			pos[i] = jointPos[i] - uGlobalsParent[i];
		}
		else if (jointType[i] == FREE_JOINT)
		{
			pos[i] = i_q.segment(posStartIndex[i], 3);
		}
		else if (jointType[i] == HINGE_JOINT)
		{
			pos[i] = preAnchor + hingeMagnitude[i] * hingeDirGlobals[i] - uGlobals[i][0];//TODO
		}

		//update Euler angles
		_Scalar eulerAngles[3];
		GetEulerAngles(i, i_quat[i], eulerAngles);
		mAlpha[i] = eulerAngles[2];
		mBeta[i] = eulerAngles[1];
		mGamma[i] = eulerAngles[0];
		
		//get ready for the next iteration
	/*	uGlobals[i][1] = R_global[i] * uLocals[i][1];
		preAnchor = pos[i] + uGlobals[i][1];
		if (i + 1 < numOfLinks) jointPos[i + 1] = preAnchor;*/
		
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
}

void eae6320::MultiBody::ResetExternalForces()
{
	for (int i = 0; i < numOfLinks; i++)
	{
		externalForces[i].setZero();
	}
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

void eae6320::MultiBody::AddRigidBody(int parent, int i_jointType, _Vector3 jointPositionParent, _Vector3 jointPositionChild, Assets::cHandle<Mesh> i_mesh, Vector3d i_meshScale, _Matrix3& i_localInertiaTensor)
{
	//initialize body
	GameCommon::GameObject *pGameObject = new GameCommon::GameObject(defaultEffect, i_mesh, Physics::sRigidBodyState());
	pGameObject->scale = i_meshScale;
	m_linkBodys.push_back(pGameObject);
	localInertiaTensors.push_back(i_localInertiaTensor);
	_Matrix M_d;
	M_d.resize(6, 6);
	M_d.setZero();
	M_d(0, 0) = rigidBodyMass;
	M_d(1, 1) = rigidBodyMass;
	M_d(2, 2) = rigidBodyMass;
	M_d.block<3, 3>(3, 3) = i_localInertiaTensor;
	Mbody.push_back(M_d);

	parentArr.push_back(-1);
	parentArr[numOfLinks] = parent;
	
	//initialize joint
	jointType.push_back(i_jointType);
	if (i_jointType == BALL_JOINT_4D)
	{
		velDOF.push_back(3);
		posDOF.push_back(4);
		totalVelDOF += 3;
		totalPosDOF += 4;
	}
	else if (i_jointType == FREE_JOINT)
	{
		velDOF.push_back(6);
		posDOF.push_back(7);
		totalVelDOF += 6;
		totalPosDOF += 7;
	}
	else
	{
		std::cout << "TODO for other type of joints" << std::endl;//TODO
	}
	if (parent == -1)
	{
		velStartIndex.push_back(0);
		posStartIndex.push_back(0);
	}
	else
	{
		velStartIndex.push_back(velStartIndex[numOfLinks - 1] + velDOF[numOfLinks - 1]);
		posStartIndex.push_back(posStartIndex[numOfLinks - 1] + posDOF[numOfLinks - 1]);
	}
	uLocalsParent.push_back(jointPositionParent);
	uGlobalsParent.push_back(jointPositionParent);
	uLocalsChild.push_back(jointPositionChild);
	uGlobalsChild.push_back(jointPositionChild);
	
	numOfLinks++;
}

void eae6320::MultiBody::UpdateGameObjectBasedOnInput()
{
	if (UserInput::IsKeyFromReleasedToPressed('R'))
	{
		//save binary data to file
		FILE * pFile;
		const char* filePath = "sim_state.txt";
		pFile = fopen(filePath, "wb");
		for (int i = 0; i < 3; i++)
		{
			fwrite(&qdot(i), sizeof(double), 1, pFile);
		}
		fwrite(&rel_ori[0], sizeof(double) * 4, 1, pFile);
		fclose(pFile);
		std::cout << "data saved to file" << std::endl;
	}
	if (UserInput::IsKeyFromReleasedToPressed('K'))
	{
		//Save data to Houdini
		static int frames_saved = 1;
		LOG_TO_FILE << frames_saved << ",";
		for (int i = 0; i < numOfLinks; i++)
		{
			_Vector3 vecRot = Math::RotationConversion_QuatToVec(obs_ori[i]);
			_Scalar rotAngle = vecRot.norm();
			if (abs(rotAngle) < 1e-8)
			{
				vecRot = _Vector3(1, 0, 0);
			}
			else
			{
				vecRot = vecRot / rotAngle;
			}
			LOG_TO_FILE << pos[i](0) << "," << pos[i](1) << "," << pos[i](2) << "," << vecRot(0) << "," << vecRot(1) << "," << vecRot(2) << "," << rotAngle;
			if (i != numOfLinks - 1)
			{
				LOG_TO_FILE << ",";
			}
			else
			{
				LOG_TO_FILE << std::endl;
			}
		}
		frames_saved++;
		std::cout << "a frame saved!" << std::endl;
	}
}

void eae6320::MultiBody::SaveDataToMatlab(_Scalar totalDuration)
{
	static _Scalar oldTime = 0;
	_Scalar t = (_Scalar)eae6320::Physics::totalSimulationTime;
	if (t <= totalDuration)
	{	
		if (t - oldTime >= 0.01 - 1e-8 || t == 0)
		{
			LOG_TO_FILE << t << " ";
			for (int i = 0; i < numOfLinks; i++)
			{
				LOG_TO_FILE << pos[i].transpose() << " " << rel_ori[i];
				if (i != numOfLinks - 1)
				{
					LOG_TO_FILE << " ";
				}
				else
				{
					LOG_TO_FILE << " " << ComputeTotalEnergy() << std::endl;
				}
			}
			oldTime = t;
		}
	}
	else
	{
		eae6320::Physics::simPause = true;
	}
}

void eae6320::MultiBody::SaveDataToHoudini(_Scalar totalDuration, int numOfFrames)
{
	_Scalar interval = totalDuration / (numOfFrames - 1);
	static _Scalar oldTime = 0;
	static int frames_saved = 0;
	_Scalar t = (_Scalar)eae6320::Physics::totalSimulationTime;
	if (frames_saved < numOfFrames)
	{
		if (t == 0 || t - oldTime >= interval - 1e-8)
		{
			frames_saved++;
			LOG_TO_FILE << frames_saved << ",";
			for (int i = 0; i < numOfLinks; i++)
			{
				_Vector3 vecRot = Math::RotationConversion_QuatToVec(obs_ori[i]);
				_Scalar rotAngle = vecRot.norm();
				if (abs(rotAngle) < 1e-8)
				{
					vecRot = _Vector3(1, 0, 0);
				}
				else
				{
					vecRot = vecRot / rotAngle;
				}
				LOG_TO_FILE << pos[i](0) << "," << pos[i](1) << "," << pos[i](2) << "," << vecRot(0) << "," << vecRot(1) << "," << vecRot(2) << "," << rotAngle;
				if (i != numOfLinks - 1)
				{
					LOG_TO_FILE << ",";
				}
				else
				{
					LOG_TO_FILE << std::endl;
				}
			}
			oldTime = t;
		}
	}
	else
	{
		eae6320::Physics::simPause = true;
	}
}