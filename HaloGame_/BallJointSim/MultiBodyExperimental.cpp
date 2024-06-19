#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>

/***************************************momentum-energy correction*************************************************************/
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

/***************************************joint limit constraint*************************************************************/
void eae6320::MultiBody::SwingLimitCheck()
{
	/*_Matrix3 R_swing;
	_Matrix3 R_twist;
	_Vector3 twistAxis(0, -1, 0);
	Math::TwistSwingDecomposition(R_local[0], twistAxis, R_twist, R_swing);
	_Vector3 vec_twist = Math::RotationConversion_MatrixToVec(R_twist);
	_Vector3 vec_swing = Math::RotationConversion_MatrixToVec(R_swing);
	std::cout << "twist: " << vec_twist.norm() << ", swing: " << vec_swing.norm() << std::endl;*/
	
	jointsID.clear();
	for (int i = 0; i < numOfLinks; i++)
	{
		if ((jointType[i] == BALL_JOINT_3D || jointType[i] == BALL_JOINT_4D) && jointLimit[i] > 0)
		{
			_Vector3 p = _Vector3(0, -1, 0);
			g[i] = p.dot(R_local[i] * p) - cos(jointLimit[i]);
			if (g[i] < 0)
			{
				jointsID.push_back(i);
				limitType.push_back(SWING);
			}
		}
	}
}

void eae6320::MultiBody::ResolveSwingLimit(const _Scalar h)
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
		for (size_t i = 0; i < constraintNum; i++)
		{
			int joint_id = jointsID[i];

			if (jointType[joint_id] == BALL_JOINT_3D)
			{
				_Vector3 p = _Vector3(0, -1, 0);
				_Vector3 p_new = R_local[joint_id] * p;
				J.block<1, 3>(i, velStartIndex[joint_id]) = (J_rotation[joint_id].transpose() * Math::ToSkewSymmetricMatrix(p_new) * p).transpose();
			}
			else if (jointType[joint_id] == BALL_JOINT_4D)
			{
				_Scalar j0 = ComputeAngularVelocityConstraint(_Vector3(1, 0, 0), R_local[joint_id], limitType[i], jointLimit[joint_id]);
				_Scalar j1 = ComputeAngularVelocityConstraint(_Vector3(0, 1, 0), R_local[joint_id], limitType[i], jointLimit[joint_id]);
				_Scalar j2 = ComputeAngularVelocityConstraint(_Vector3(0, 0, 1), R_local[joint_id], limitType[i], jointLimit[joint_id]);
				J.block<1, 3>(i, velStartIndex[joint_id]) = _Vector3(j0, j1, j2);

				_Vector3 pRotated = R_local[joint_id] * p;
				_Vector3 s = p.cross(pRotated);
				if (s.norm() < 0.00001)
				{
					s = local_x;
				}
				s.normalize();
				_Scalar cTest = ComputeAngularVelocityConstraint(s, R_local[joint_id], limitType[i], jointLimit[joint_id]);
				if (cTest < 0)
				{
					s = -s;
				}
				K.block<1, 3>(i, velStartIndex[joint_id]) = s;
			}
			_Vector3 rdot = qdot.segment(velStartIndex[joint_id], 3);
			_Matrix JV = J.block<1, 3>(i, velStartIndex[joint_id]) * rdot;

			//_Scalar beta = 0.2f;//0.4f;
			//_Scalar CR = 0.4f;// 0.2f;
			//_Scalar SlopP = 0.001f;
			//bias(i) = -beta / h * std::max<_Scalar>(-g[joint_id], 0.0) - CR * std::max<_Scalar>(-JV(0, 0), 0.0);
		}

		//std::cout << J << std::endl;
		_Matrix lambda;
		lambda = (J * MrInverse * K.transpose()).inverse() * (-J * qdot - bias);
		//std::cout << lambda.transpose() << std::endl;

		for (size_t i = 0; i < constraintNum; i++)
		{
			if (lambda(i, 0) < 0) lambda(i, 0) = 0;
		}

		_Vector RdotCorrection = MrInverse * K.transpose() * lambda;
		qdot = qdot + RdotCorrection;
	}
}

void eae6320::MultiBody::ResolveSwingLimitPBD(_Vector& i_q, const _Scalar h)
{
	size_t constraintNum = jointsID.size();
	if (constraintNum > 0)
	{
		_Matrix J;
		J.resize(constraintNum, totalVelDOF);
		J.setZero();

		/*{
			_Matrix3 R_swing;
			_Matrix3 R_twist;
			_Vector3 twistAxis(0, -1, 0);
			Math::TwistSwingDecomposition(R_local[0], twistAxis, R_twist, R_swing);
			_Vector3 vec_twist = Math::RotationConversion_MatrixToVec(R_twist);
			_Vector3 vec_swing = Math::RotationConversion_MatrixToVec(R_swing);
			std::cout << "twist: " << vec_twist.norm() << ", swing: " << vec_swing.norm() << std::endl;
		}*/

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

		{
			UpdateBodyRotation(i_q, rel_ori);
			_Matrix3 R_swing;
			_Matrix3 R_twist;
			_Vector3 twistAxis(0, -1, 0);
			Math::TwistSwingDecomposition(R_local[0], twistAxis, R_twist, R_swing);
			_Vector3 vec_twist = Math::RotationConversion_MatrixToVec(R_twist);
			_Vector3 vec_swing = Math::RotationConversion_MatrixToVec(R_swing);
			std::cout << "twist: " << vec_twist.norm() << ", swing: " << vec_swing.norm() << std::endl;
		}

		qdot = (i_q - q) / h;
	}
}

void eae6320::MultiBody::TwistLimitCheck()
{
	jointsID.clear();
	limitType.clear();
	_Vector3 p = _Vector3(0, -1, 0);
	_Vector3 local_x = _Vector3(1, 0, 0);
	for (int i = 0; i < numOfLinks; i++)
	{
		if ((jointType[i] == BALL_JOINT_3D || jointType[i] == BALL_JOINT_4D) && jointLimit[i] > 0)
		{
			_Vector s = p.cross(R_local[i] * p);
			if (s.norm() < 0.0001)
			{
				g[i] = local_x.dot(R_local[i] * local_x) - cos(jointLimit[i]);
				if (g[i] < 0)
				{
					jointsID.push_back(i);
					limitType.push_back(TWIST_WITHOUT_SWING);
					//Physics::simPause = true;
				}
			}
			else
			{
				g[i] = s.dot(R_local[i] * s) / s.squaredNorm() - cos(jointLimit[i]);
				if (g[i] < 0)
				{
					jointsID.push_back(i);
					limitType.push_back(TWIST_WITH_SWING);
					//Physics::simPause = true;
				}
			}
		}
	}
	//{
	//	_Matrix3 R_swing;
	//	_Matrix3 R_twist;
	//	_Vector3 twistAxis(0, -1, 0);
	//	Math::TwistSwingDecomposition(R_local[0], twistAxis, R_twist, R_swing);
	//	_Vector3 vec_twist = Math::RotationConversion_MatrixToVec(R_twist);
	//	_Vector3 vec_swing = Math::RotationConversion_MatrixToVec(R_swing);
	//	std::cout << "twist: " << vec_twist.norm() << ", swing: " << vec_swing.norm() << std::endl;
	//	//std::cout << vec_twist.transpose() << std::endl;
	//}
}

void eae6320::MultiBody::ResolveTwistLimitPBD(_Vector& i_q, const _Scalar h)
{
	size_t constraintNum = jointsID.size();
	_Vector3 p = _Vector3(0, -10, 0);
	_Vector3 local_x = _Vector3(1, 0, 0);
	if (constraintNum > 0)
	{
		_Matrix3 R_swing;
		_Matrix3 R_twist;
		_Vector3 twistAxis(0, -1, 0);
		Math::TwistSwingDecomposition(R_local[0], twistAxis, R_twist, R_swing);
		_Vector3 vec_twist = Math::RotationConversion_MatrixToVec(R_twist);
		_Vector3 vec_swing = Math::RotationConversion_MatrixToVec(R_swing);
		std::cout << "twist: " << vec_twist.norm() << ", swing: " << vec_swing.norm() << std::endl;
		
		_Matrix J;
		J.resize(constraintNum, totalVelDOF);
		J.setZero();

		int iterationNum = 1;
		for (int iter = 0; iter < iterationNum; iter++)
		{
			_Vector C;
			C.resize(constraintNum, 1);

			//construct total Jacobian matrix
			for (size_t k = 0; k < constraintNum; k++)
			{
				size_t joint_id = jointsID[k];
				if (limitType[k] == TWIST_WITH_SWING)
				{
					_Vector3 T0;
					_Vector3 RP = R_local[joint_id] * p;
					T0 = -J_rotation[joint_id].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * R_local[joint_id] * Math::ToSkewSymmetricMatrix(p) * RP;
					_Vector3 T1;
					_Vector3 RPRP = R_local[joint_id] * Math::ToSkewSymmetricMatrix(p) * RP;
					T1 = J_rotation[joint_id].transpose() * Math::ToSkewSymmetricMatrix(RPRP) * Math::ToSkewSymmetricMatrix(p) * RP;
					_Vector3 T2;
					T2 = -J_rotation[joint_id].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * R_local[joint_id].transpose() * Math::ToSkewSymmetricMatrix(p) * RP;
					_Vector3 T3;
					//T3 = 2.0 * cos(jointLimit[i]) * J_rotation[i].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * Math::ToSkewSymmetricMatrix(p) * RP;
					T3 = -2.0 * J_rotation[joint_id].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * Math::ToSkewSymmetricMatrix(p) * RP;

					_Vector s = p.cross(RP);
					_Scalar SRS = s.dot(R_local[joint_id] * s);
					J.block<1, 3>(k, velStartIndex[joint_id]) = ((T0 + T1 + T2) / s.squaredNorm() - SRS / (s.squaredNorm() * s.squaredNorm()) * T3).transpose();

					C(k) = s.dot(R_local[joint_id] * s) / s.squaredNorm() - cos(jointLimit[joint_id]);
				}
				else if (limitType[k] == TWIST_WITHOUT_SWING)
				{
					_Vector3 local_x_new = R_local[joint_id] * local_x;
					J.block<1, 3>(k, velStartIndex[joint_id]) = (J_rotation[joint_id].transpose() * Math::ToSkewSymmetricMatrix(local_x_new) * local_x).transpose();

					C(k) = local_x.dot(R_local[joint_id] * local_x) - cos(jointLimit[joint_id]);
				}
			}

			_Matrix A = J * MrInverse * J.transpose();
			//_Matrix A = J * J.transpose();
			_Matrix lambda = A.inverse() * -C;

			_Vector R_correction = MrInverse * J.transpose() * lambda;
			/*std::cout << R_correction.normalized().transpose() << std::endl;
			_Vector R_correction_ = J.transpose() * lambda;
			std::cout << R_correction_.normalized().transpose() << std::endl;*/
			for (int j = 0; j < numOfLinks; j++)
			{
				if (posDOF[j] == velDOF[j]) i_q.segment(posStartIndex[j], posDOF[j]) = i_q.segment(posStartIndex[j], posDOF[j]) + R_correction.segment(velStartIndex[j], velDOF[j]);
			}

			ComputeHt(i_q, rel_ori);
		}

	/*	{
			_Matrix3 R_swing;
			_Matrix3 R_twist;
			_Vector3 twistAxis(0, -1, 0);
			Math::TwistSwingDecomposition(R_local[0], twistAxis, R_twist, R_swing);
			_Vector3 vec_twist = Math::RotationConversion_MatrixToVec(R_twist);
			_Vector3 vec_swing = Math::RotationConversion_MatrixToVec(R_swing);
			std::cout << "twist: " << vec_twist.norm() << ", swing: " << vec_swing.norm() << std::endl;
		}*/
		qdot = (i_q - q) / h;
	}
}

void eae6320::MultiBody::ResolveTwistLimit(const _Scalar h)
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
		for (size_t k = 0; k < constraintNum; k++)
		{
			int i = jointsID[k];

			if (jointType[i] == BALL_JOINT_3D)
			{
				if (limitType[k] == TWIST_WITH_SWING)
				{
					_Vector3 T0;
					_Vector3 RP = R_local[i] * p;
					T0 = -J_rotation[i].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * R_local[i] * Math::ToSkewSymmetricMatrix(p) * RP;
					_Vector3 T1;
					_Vector3 RPRP = R_local[i] * Math::ToSkewSymmetricMatrix(p) * RP;
					T1 = J_rotation[i].transpose() * Math::ToSkewSymmetricMatrix(RPRP) * Math::ToSkewSymmetricMatrix(p) * RP;
					_Vector3 T2;
					T2 = -J_rotation[i].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * R_local[i].transpose() * Math::ToSkewSymmetricMatrix(p) * RP;
					_Vector3 T3;
					//T3 = 2.0 * cos(jointLimit[i]) * J_rotation[i].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * Math::ToSkewSymmetricMatrix(p) * RP;
					T3 = -2.0 * J_rotation[i].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * Math::ToSkewSymmetricMatrix(p) * RP;

					_Vector s = p.cross(RP);
					_Scalar SRS = s.dot(R_local[i] * s);

					J.block<1, 3>(k, velStartIndex[i]) = ((T0 + T1 + T2) / s.squaredNorm() - SRS / (s.squaredNorm() * s.squaredNorm()) * T3).transpose();
				}
				else if (limitType[k] == TWIST_WITHOUT_SWING)
				{
					_Vector3 local_x_new = R_local[i] * local_x;
					J.block<1, 3>(k, velStartIndex[i]) = (J_rotation[i].transpose() * Math::ToSkewSymmetricMatrix(local_x_new) * local_x).transpose();
				}
				K.block<1, 3>(k, velStartIndex[i]) = J.block<1, 3>(k, velStartIndex[i]);
			}
			else if (jointType[i] == BALL_JOINT_4D)
			{
				_Scalar j0 = ComputeAngularVelocityConstraint(_Vector3(1, 0, 0), R_local[i], limitType[k], jointLimit[i]);
				_Scalar j1 = ComputeAngularVelocityConstraint(_Vector3(0, 1, 0), R_local[i], limitType[k], jointLimit[i]);
 				_Scalar j2 = ComputeAngularVelocityConstraint(_Vector3(0, 0, 1), R_local[i], limitType[k], jointLimit[i]);
				J.block<1, 3>(k, velStartIndex[i]) = _Vector3(j0, j1, j2);

				/*_Matrix3 R_swing;
				_Matrix3 R_twist;
				_Vector3 twistAxis(0, -1, 0);
				Math::TwistSwingDecomposition(R_local[i], twistAxis, R_twist, R_swing);

				_Vector3 vec_twist = Math::RotationConversion_MatrixToVec(R_twist);
				vec_twist.normalize();
				vec_twist = -vec_twist;
				K.block<1, 3>(k, velStartIndex[i]) = vec_twist.transpose();*/
				K.block<1, 3>(k, velStartIndex[i]) = J.block<1, 3>(k, velStartIndex[i]);
			}

			_Vector3 rdot = qdot.segment(velStartIndex[i], 3);
			_Matrix JV = J.block<1, 3>(k, velStartIndex[i]) * rdot;
			_Scalar vc = ComputeAngularVelocityConstraint(rdot, R_local[i], limitType[i], jointLimit[i]);
			std::cout << vc << std::endl;
			std::cout << J << std::endl;
			std::cout << qdot.transpose() << std::endl;

			//_Scalar beta = 0.2f;//0.4f;
			//_Scalar CR = 0.4f;// 0.2f;
			//_Scalar SlopP = 0.001f;
			//bias(k) = -beta / h * std::max<_Scalar>(-g[i], 0.0) - CR * std::max<_Scalar>(-JV(0, 0), 0.0);
		}
		_Matrix lambda;
		lambda = (J * MrInverse * K.transpose()).inverse() * (-J * qdot - bias);
		//lambda = (J * J.transpose()).inverse() * (-J * qdot - bias);

		for (size_t i = 0; i < constraintNum; i++)
		{
			if (lambda(i, 0) < 0) 
			{ 
				lambda(i, 0) = 0;
			}	
		}

		_Vector RdotCorrection = MrInverse * K.transpose() * lambda;
		//_Vector RdotCorrection = J.transpose() * lambda;
	/*	std::cout << qdot.norm() << std::endl;
		std::cout << RdotCorrection.norm() << std::endl; */
		/*_Scalar tNorm = 0.5 * (qdot.transpose() * MrInverse * qdot)(0, 0);
		std::cout << tNorm << std::endl;
		tNorm = 0.5 * (RdotCorrection.transpose() * MrInverse * RdotCorrection)(0, 0);
		std::cout << tNorm << std::endl;*/
		qdot = qdot + RdotCorrection;
		//std::cout << qdot << std::endl;
		//std::cout << lambda << std::endl;
	}
}

void eae6320::MultiBody::_BallJointLimitCheck()
{
	if (constraintType == SWING_C)
	{
		SwingLimitCheck();
	}
	else if (constraintType == TWIST_C)
	{
		TwistLimitCheck();
	}
}

void eae6320::MultiBody::_ResolveJointLimit(const _Scalar h)
{
	if (constraintType == SWING_C)
	{
		ResolveSwingLimit(h);
	}
	else if (constraintType == TWIST_C)
	{
		ResolveTwistLimit(h);
	}
}

void eae6320::MultiBody::_ResolveJointLimitPBD(_Vector& i_q, const _Scalar h)
{
	if (constraintType == SWING_C)
	{
		ResolveSwingLimitPBD(i_q, h);
	}
	else if (constraintType == TWIST_C)
	{
		ResolveTwistLimitPBD(i_q, h);
	}
}