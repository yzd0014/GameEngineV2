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
		ForwardAngularAndTranslationalVelocity(Ht, qdot_new);
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
		ForwardAngularAndTranslationalVelocity(Ht, qdot_new);
		energyErr = fabs(ComputeTotalEnergy() - kineticEnergy0);
		//std::cout << energyErr << std::endl;
		i++;
	}
	qdot = x.segment(0, totalVelDOF);
}

/***************************************joint limit constraint*************************************************************/
_Scalar eae6320::MultiBody::ComputeAngularVelocityConstraint(_Vector3& w, _Vector3& p, _Matrix3& Rot, int i_limitType, _Scalar phi)
{
	_Scalar C = 0;
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
		_Vector s = Math::GetOrthogonalVector(p);
		C = s.dot(Math::ToSkewSymmetricMatrix(w) * Rot * s);
	}
	else if (i_limitType == SWING)
	{
		C = p.dot(Math::ToSkewSymmetricMatrix(w) * Rot * p);
	}
	return C;
}

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
				J.block<1, 3>(i, velStartIndex[joint_id]) = (J_exp[joint_id].transpose() * Math::ToSkewSymmetricMatrix(p_new) * p).transpose();
			}
			else if (jointType[joint_id] == BALL_JOINT_4D)
			{
				_Scalar j0 = ComputeAngularVelocityConstraint(_Vector3(1, 0, 0), p, R_local[joint_id], limitType[i], jointLimit[joint_id]);
				_Scalar j1 = ComputeAngularVelocityConstraint(_Vector3(0, 1, 0), p, R_local[joint_id], limitType[i], jointLimit[joint_id]);
				_Scalar j2 = ComputeAngularVelocityConstraint(_Vector3(0, 0, 1), p, R_local[joint_id], limitType[i], jointLimit[joint_id]);
				J.block<1, 3>(i, velStartIndex[joint_id]) = _Vector3(j0, j1, j2);

				_Vector3 pRotated = R_local[joint_id] * p;
				_Vector3 s = p.cross(pRotated);
				if (s.norm() < 0.00001)
				{
					s = local_x;
				}
				s.normalize();
				_Scalar cTest = ComputeAngularVelocityConstraint(s, p, R_local[joint_id], limitType[i], jointLimit[joint_id]);
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
			UpdateBodyRotation(i_q, rel_ori, jointType, posStartIndex);
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
					T0 = -J_exp[joint_id].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * R_local[joint_id] * Math::ToSkewSymmetricMatrix(p) * RP;
					_Vector3 T1;
					_Vector3 RPRP = R_local[joint_id] * Math::ToSkewSymmetricMatrix(p) * RP;
					T1 = J_exp[joint_id].transpose() * Math::ToSkewSymmetricMatrix(RPRP) * Math::ToSkewSymmetricMatrix(p) * RP;
					_Vector3 T2;
					T2 = -J_exp[joint_id].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * R_local[joint_id].transpose() * Math::ToSkewSymmetricMatrix(p) * RP;
					_Vector3 T3;
					//T3 = 2.0 * cos(jointLimit[i]) * J_exp[i].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * Math::ToSkewSymmetricMatrix(p) * RP;
					T3 = -2.0 * J_exp[joint_id].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * Math::ToSkewSymmetricMatrix(p) * RP;

					_Vector s = p.cross(RP);
					_Scalar SRS = s.dot(R_local[joint_id] * s);
					J.block<1, 3>(k, velStartIndex[joint_id]) = ((T0 + T1 + T2) / s.squaredNorm() - SRS / (s.squaredNorm() * s.squaredNorm()) * T3).transpose();

					C(k) = s.dot(R_local[joint_id] * s) / s.squaredNorm() - cos(jointLimit[joint_id]);
				}
				else if (limitType[k] == TWIST_WITHOUT_SWING)
				{
					_Vector3 local_x_new = R_local[joint_id] * local_x;
					J.block<1, 3>(k, velStartIndex[joint_id]) = (J_exp[joint_id].transpose() * Math::ToSkewSymmetricMatrix(local_x_new) * local_x).transpose();

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

			ComputeHt(Ht, H, i_q, rel_ori, jointType, posStartIndex);
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
					T0 = -J_exp[i].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * R_local[i] * Math::ToSkewSymmetricMatrix(p) * RP;
					_Vector3 T1;
					_Vector3 RPRP = R_local[i] * Math::ToSkewSymmetricMatrix(p) * RP;
					T1 = J_exp[i].transpose() * Math::ToSkewSymmetricMatrix(RPRP) * Math::ToSkewSymmetricMatrix(p) * RP;
					_Vector3 T2;
					T2 = -J_exp[i].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * R_local[i].transpose() * Math::ToSkewSymmetricMatrix(p) * RP;
					_Vector3 T3;
					//T3 = 2.0 * cos(jointLimit[i]) * J_exp[i].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * Math::ToSkewSymmetricMatrix(p) * RP;
					T3 = -2.0 * J_exp[i].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * Math::ToSkewSymmetricMatrix(p) * RP;

					_Vector s = p.cross(RP);
					_Scalar SRS = s.dot(R_local[i] * s);

					J.block<1, 3>(k, velStartIndex[i]) = ((T0 + T1 + T2) / s.squaredNorm() - SRS / (s.squaredNorm() * s.squaredNorm()) * T3).transpose();
				}
				else if (limitType[k] == TWIST_WITHOUT_SWING)
				{
					_Vector3 local_x_new = R_local[i] * local_x;
					J.block<1, 3>(k, velStartIndex[i]) = (J_exp[i].transpose() * Math::ToSkewSymmetricMatrix(local_x_new) * local_x).transpose();
				}
				K.block<1, 3>(k, velStartIndex[i]) = J.block<1, 3>(k, velStartIndex[i]);
			}
			else if (jointType[i] == BALL_JOINT_4D)
			{
				_Scalar j0 = ComputeAngularVelocityConstraint(_Vector3(1, 0, 0), p, R_local[i], limitType[k], jointLimit[i]);
				_Scalar j1 = ComputeAngularVelocityConstraint(_Vector3(0, 1, 0), p, R_local[i], limitType[k], jointLimit[i]);
 				_Scalar j2 = ComputeAngularVelocityConstraint(_Vector3(0, 0, 1), p, R_local[i], limitType[k], jointLimit[i]);
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
			_Scalar vc = ComputeAngularVelocityConstraint(rdot, p, R_local[i], limitType[i], jointLimit[i]);
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

void eae6320::MultiBody::PrePositionSolveProccessing()
{
	for (int i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_4D)
		{
			x.segment(xStartIndex[i], 3) = Math::RotationConversion_QuatToVec(rel_ori[i]);
		}
		else if (jointType[i] == FREE_JOINT)
		{
			EAE6320_ASSERTF(false, "Position solve doesn't support free joint");
		}
		else
		{
			x.segment(xStartIndex[i], xDOF[i]) = q.segment(posStartIndex[i], xDOF[i]);
		}
	}
}

void eae6320::MultiBody::PostPositionSolveProccessing()
{
	for (int i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_4D)
		{
			_Vector3 rotVec = x.segment(xStartIndex[i], 3);
			rel_ori[i] = Math::RotationConversion_VecToQuat(rotVec);
		}
		else if (jointType[i] == FREE_JOINT)
		{
			EAE6320_ASSERTF(false, "Position solve doesn't support free joint");
		}
		else
		{
			q.segment(posStartIndex[i], xDOF[i]) = x.segment(xStartIndex[i], xDOF[i]);
		}
	}
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

void eae6320::MultiBody::InitializeBodies(Assets::cHandle<Mesh> i_mesh, Vector3d i_meshScale, _Matrix3& i_localInertiaTensor, _Vector3 i_partentJointPosition, _Vector3 i_childJointPosition)
{
	Math::NativeVector2EigenVector(m_State.position, jointPos[0]);
	Mr.resize(totalVelDOF, totalVelDOF);

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
	rel_ori.resize(numOfLinks);
	R_global.resize(numOfLinks);
	R_local.resize(numOfLinks);
	J_exp.resize(numOfLinks);
	D.resize(numOfLinks);
	Ht.resize(numOfLinks);
	H.resize(numOfLinks);
	Mbody.resize(numOfLinks);
	localInertiaTensors.resize(numOfLinks);
	g.resize(numOfLinks);
	//jointType.resize(numOfLinks);
	//posDOF.resize(numOfLinks);
	//posStartIndex.resize(numOfLinks);
	//velDOF.resize(numOfLinks);
	//velStartIndex.resize(numOfLinks);
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
	for (int i = 0; i < numOfLinks; i++)
	{
		w_abs_world[i].setZero();
		w_rel_world[i].setZero();
		w_rel_local[i].setZero();
		vel[i].setZero();
		jointPos[i].setZero();
		pos[i].setZero();
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

		/*std::vector<_Vector3> uPairs;
		uPairs.resize(2);
		uPairs[0] = i_partentJointPosition;
		uPairs[1] = i_childJointPosition;
		uLocals.push_back(uPairs);
		uGlobals.push_back(uPairs);*/

		totalTwist[i] = 0;
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
}

void eae6320::MultiBody::ComputeJacobianAndInertiaDerivativeFD(_Vector& i_bj, std::vector<_Vector>& i_bm, std::vector<_Matrix>& o_Jacobian, std::vector<_Matrix>& o_intertia, _Scalar i_delta)
{
	_Scalar delta = i_delta;

	_Matrix d0, d1;
	d0.resize(6, 1);
	d1.resize(6, 1);

	Populate_q(rel_ori, q);
	_Vector old_q;
	old_q = q;

	int dof = static_cast<int>(q.size());
	std::vector<_Vector> perturbed_q;
	perturbed_q.resize(dof);
	for (int i = 0; i < dof; i++)
	{
		perturbed_q[i].resize(dof);
		perturbed_q[i] = q;
		perturbed_q[i](i) = perturbed_q[i](i) + delta;
	}

	for (int i = 0; i < numOfLinks; i++)
	{
		q = old_q;
		Populate_quat(q, rel_ori, false);
		ComputeHt(Ht, H, q, rel_ori, jointType, posStartIndex);
		o_Jacobian[i].resize(6, dof);
		d0 = Ht[i] * i_bj;
		for (int k = 0; k < dof; k++)
		{
			q = perturbed_q[k];
			Populate_quat(q, rel_ori, false);
			ComputeHt(Ht, H, q, rel_ori, jointType, posStartIndex);
			d1 = Ht[i] * i_bj;

			std::cout << std::endl << std::setprecision(16) << d1 << std::endl;
			std::cout << std::endl << std::setprecision(16) << d0 << std::endl;
			o_Jacobian[i].block<6, 1>(0, k) = (d1 - d0) / delta;
			std::cout << std::endl << std::setprecision(16) << (d1 - d0) / delta << std::endl;
		}
	}

	for (int i = 0; i < numOfLinks; i++)
	{
		q = old_q;
		Populate_quat(q, rel_ori, false);
		ForwardKinematics(q, rel_ori, jointType, posStartIndex);
		o_intertia[i].resize(6, dof);
		d0 = Mbody[i] * i_bm[i];
		for (int k = 0; k < dof; k++)
		{
			q = perturbed_q[k];
			Populate_quat(q, rel_ori, false);
			ForwardKinematics(q, rel_ori, jointType, posStartIndex);
			d1 = Mbody[i] * i_bm[i];
			o_intertia[i].block<6, 1>(0, k) = (d1 - d0) / delta;
		}
	}

	q = old_q;
	Populate_quat(q, rel_ori, false);
	Forward();
}

void eae6320::MultiBody::GetNullSpace(_Matrix& o_nullSpace, _Matrix& i_M)
{
	FullPivLU<_Matrix> lu(i_M);
	int rank = static_cast<int>(lu.rank());
	int m = static_cast<int>(i_M.rows());
	if (rank != m)
	{
		EAE6320_ASSERTF(false, "matrix is not full rank");
	}
	
	JacobiSVD<_Matrix> svd(i_M, ComputeFullV);
	//_Vector S = svd.singularValues();
	_Matrix V = svd.matrixV();
	
	int n = static_cast<int>(i_M.cols());
	o_nullSpace = V.block(0, rank, n, n - rank);
}

void eae6320::MultiBody::ComputeMomentumMatrix(_Matrix& o_M, std::vector<_Matrix>& i_Ht, std::vector<_Matrix>& i_inertiaGlobal, std::vector<_Vector3>& i_positionOfCOM)
{
	o_M.resize(6, totalVelDOF);
	o_M.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
		o_M.block(0, 0, 3, totalVelDOF) = o_M.block(0, 0, 3, totalVelDOF) + i_inertiaGlobal[i].block<3, 3>(0, 0) * i_Ht[i].block(0, 0, 3, totalVelDOF);
		o_M.block(3, 0, 3, totalVelDOF) = o_M.block(3, 0, 3, totalVelDOF) + i_inertiaGlobal[i].block<3, 3>(3, 3) * i_Ht[i].block(3, 0, 3, totalVelDOF) + rigidBodyMass * Math::ToSkewSymmetricMatrix(i_positionOfCOM[i]) * i_Ht[i].block(0, 0, 3, totalVelDOF);
	}
}

void eae6320::MultiBody::EnergyNullSpaceCorrection()
{
	_Matrix momentumMatrix;
	ComputeMomentumMatrix(momentumMatrix, Ht, Mbody, pos);

	//compute the null space of the momentum matrix
	_Matrix Z;
	GetNullSpace(Z, momentumMatrix);
	//std::cout << (momentumMatrix * Z).transpose() << std::endl;

	int constraintDim = 1;
	int nullSpaceDim = static_cast<int>(Z.cols());
	int n = nullSpaceDim + constraintDim;
	_Matrix b(n, 1);
	_Matrix grad_F(n, n);
	
	_Matrix regularization(n, n);
	regularization.setIdentity();
	regularization = 1e-6 * regularization;

	_Vector x(n);
	x.setZero();

	_Scalar kineticEnergy0 = totalEnergy0 - ComputePotentialEnergy();
	_Matrix grad_C(constraintDim, nullSpaceDim);
	grad_C.setZero();

	_Scalar energyErr = 1.0;
	_Matrix C(constraintDim, 1);
	int iter = 0;
	while (true)
	{
		_Vector v = qdot + Z * x.segment(0, nullSpaceDim);
		C(0, 0) = 0.5 * v.transpose() * Mr * v - kineticEnergy0;
		_Scalar C_norm = C.norm();
		std::cout << "C_norm " << C_norm << std::endl;
		if (C_norm < 1e-6) break;
		if (iter >= 10)
		{
			std::cout << "doesn't converge" << std::endl;
		}

		grad_C.block(0, 0, constraintDim, nullSpaceDim) = (Z.transpose() * Mr * v).transpose();
		//std::cout << "grad_C " << grad_C << std::endl;

		b.block(0, 0, nullSpaceDim, 1) = -Z.transpose() * Mr * Z * x.segment(0, nullSpaceDim);
		b.block(nullSpaceDim, 0, constraintDim, constraintDim) = -C;
		//std::cout << "b " << b << std::endl;
		
		grad_F.setZero();
		grad_F.block(0, 0, nullSpaceDim, nullSpaceDim) = (1 + x(nullSpaceDim)) * Z.transpose() * Mr * Z;
		grad_F.block(0, nullSpaceDim, nullSpaceDim, constraintDim) = grad_C.transpose();
		grad_F.block(nullSpaceDim, 0, constraintDim, nullSpaceDim) = grad_C;
		//std::cout << grad_F << std::endl;
		if (abs(grad_F.determinant()) < 1e-7)
		{
			grad_F = grad_F + regularization;
			std::cout << "grad_F is not invertable" << std::endl;
			//EAE6320_ASSERTF(false, "grad_f is not invertable");
		}
		
		_Vector p(n);
		p.setZero();
		p = grad_F.inverse() * b;
		
		x.segment(0, nullSpaceDim) = x.segment(0, nullSpaceDim) + p.segment(0, nullSpaceDim);
		x(nullSpaceDim) = p(nullSpaceDim);//update lagrange multiplier
		iter++;
	}
	qdot = qdot + Z * x.segment(0, nullSpaceDim);
	ForwardAngularAndTranslationalVelocity(Ht, qdot);
	
	std::cout << "energy constraint iter: " << iter << std::endl;
	std::cout << std::setprecision(16) << "Time " << Physics::totalSimulationTime << " " << ComputeTotalEnergy() << std::endl << std::endl;
}


void eae6320::MultiBody::VariationalIntegration(const _Scalar h)
{
	std::vector<_Matrix> H_predicted, H_old;
	H_predicted.resize(numOfLinks);
	H_old.resize(numOfLinks);
	std::vector<_Matrix> Ht_predicted, Ht_old;
	Ht_predicted.resize(numOfLinks);
	Ht_old.resize(numOfLinks);
	
	_Vector q_predicted;
	q_predicted = q + h * qdot;
	
	_Matrix B;
	ComputeHt(Ht, H, q, rel_ori, jointType, posStartIndex);
	ComputeMr(Mr, Ht);
	_Matrix3 R_current;
	R_current = R_global[0];
	B = -Mr;
	
	ComputeHt(Ht_old, H_old, qOld, rel_ori, jointType, posStartIndex);
	_Matrix A;
	ComputeMr(A, Ht_old);
	//A = Mr;

	ComputeHt(Ht_predicted, H_predicted, q_predicted, rel_ori, jointType, posStartIndex);
	_Matrix Ht_diff;
	_Matrix T_predicted, T_current;
	T_predicted.resize(6, 6);
	T_predicted.setIdentity();
	T_predicted.block<3, 3>(3, 3) = R_global[0];
	T_current.resize(6, 6);
	T_current.setIdentity();
	T_current.block<3, 3>(3, 3) = R_current;

	Ht_diff = T_predicted.transpose() * Ht_predicted[0] - T_current.transpose() * Ht[0];
	_Matrix M_d;
	M_d.resize(6, 6);
	M_d.setZero();
	M_d(0, 0) = rigidBodyMass;
	M_d(1, 1) = rigidBodyMass;
	M_d(2, 2) = rigidBodyMass;
	M_d.block<3, 3>(3, 3) = localInertiaTensors[0];
	_Matrix C;
	C = h * Ht_diff.transpose() * M_d * T_current.transpose() * Ht[0];
	
	_Matrix K = B + C;
	_Vector q_new = K.inverse() * ((K - A) * q + A * qOld);
	qOld = q;
	q = q_new;
	qdot = (q - qOld) / h;
	//std::cout << qOld.transpose() << std::endl;
	//std::cout << q.transpose() << std::endl;

	_Vector qBefore = q;
	_Vector qdotBefore = qdot;
	bool clamped = false;
	clamped = ClampRotationVector(q, qdot, 0);
	//q = qBefore;
	//qdot = qdotBefore;
	
	if (clamped)
	{
		ComputeHt(Ht, H, qBefore, rel_ori, jointType, posStartIndex);
		_Vector oldOmega = Ht[0] * qdotBefore;
		ComputeHt(Ht, H, q, rel_ori, jointType, posStartIndex);
		qdot = Ht[0].inverse() * oldOmega;
	
		//std::cout << Math::RotationConversion_VecToQuat(qBefore) << std::endl;
		//_Vector3 temp = q.segmegnt(3, 3);
		//std::cout << Math::RotationConversion_VecToQuat(temp) << std::endl;

		_Vector3 r = qOld.segment(3, 3);
		_Scalar theta = r.norm();
		_Scalar eta = (_Scalar)(1.0f - 2.0f * M_PI / theta);

		//temp = qOld.segment(3, 3);
		//std::cout << Math::RotationConversion_VecToQuat(temp) << std::endl;
		
		//qOld.segment(3, 3) = eta * r;
		
		////temp = qOld.segment(3, 3);
		////std::cout << Math::RotationConversion_VecToQuat(temp) << std::endl;
		////qdot = (q - qOld) / h;

		Physics::simPause = true;
	}
	Forward();
	//std::cout << "w_abs_world " << w_abs_world[0].transpose() << std::endl;
	//std::cout << "vel[0] " << vel[0].transpose() << std::endl;
}