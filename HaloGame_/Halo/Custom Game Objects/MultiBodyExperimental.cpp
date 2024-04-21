#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>

void eae6320::MultiBody::SwingLimitCheck()
{
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
			}
		}
	}
}

void eae6320::MultiBody::ResolveSwingLimit(const _Scalar h)
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

			if (jointType[joint_id] == BALL_JOINT_3D)
			{
				_Vector3 p = _Vector3(0, -1, 0);
				_Vector3 p_new = R_local[joint_id] * p;
				J.block<1, 3>(i, velStartIndex[joint_id]) = (J_rotation[joint_id].transpose() * Math::ToSkewSymmetricMatrix(p_new) * p).transpose();
			}
			else if (jointType[joint_id] == BALL_JOINT_4D)
			{
				_Matrix3 R_swing;
				_Matrix3 R_twist;
				_Vector3 twistAxis(0, -1, 0);
				Math::TwistSwingDecompsition(R_local[i], twistAxis, R_twist, R_swing);

				_Vector3 vec_swing = Math::RotationConversion_MatrixToVec(R_swing);
				vec_swing.normalize();
				vec_swing = -vec_swing;
				J.block<1, 3>(i, velStartIndex[joint_id]) = vec_swing.transpose();
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
		lambda = (J * MrInverse * J.transpose()).inverse() * (-J * qdot - bias);
		//std::cout << lambda.transpose() << std::endl;

		for (int i = 0; i < constraintNum; i++)
		{
			if (lambda(i, 0) < 0) lambda(i, 0) = 0;
		}

		_Vector RdotCorrection = MrInverse * J.transpose() * lambda;
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
}

void eae6320::MultiBody::ResolveTwistLimitPBD(_Vector& i_q, const _Scalar h)
{
	size_t constraintNum = jointsID.size();
	_Vector3 p = _Vector3(0, -10, 0);
	_Vector3 local_x = _Vector3(1, 0, 0);
	if (constraintNum > 0)
	{
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
			_Matrix lambda = A.inverse() * -C;

			_Vector R_correction = MrInverse * J.transpose() * lambda;
			for (int j = 0; j < numOfLinks; j++)
			{
				if (posDOF[j] == velDOF[j]) i_q.segment(posStartIndex[j], posDOF[j]) = i_q.segment(posStartIndex[j], posDOF[j]) + R_correction.segment(velStartIndex[j], velDOF[j]);
			}

			ComputeHt(i_q, rel_ori);
		}
		qdot = (i_q - q) / h;
	}
}

void eae6320::MultiBody::ResolveAngularVelocityLimit()
{
	size_t constraintNum = jointsID.size();
	if (constraintNum > 0 && jointType[0] == BALL_JOINT_4D)
	{
		_Vector3 p = _Vector3(0, -1, 0);
		_Vector3 RP = R_local[0] * p;
		_Scalar twist_norm = RP.dot(qdot.segment(0, 3));
		_Vector3 twist_w = twist_norm * RP;
		std::cout << twist_w.transpose() << std::endl;
		std::cout << qdot.segment(0, 3).transpose() << std::endl;
		qdot.segment(0, 3) = qdot.segment(0, 3) - 1.5 * twist_w;
		std::cout << qdot.segment(0, 3).transpose() << std::endl;
	}
}

void eae6320::MultiBody::ResolveTwistLimit(const _Scalar h)
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
		for (int k = 0; k < constraintNum; k++)
		{
			int i = jointsID[k];

			if (jointType[i] == BALL_JOINT_3D)
			{
				_Vector3 p = _Vector3(0, -10, 0);
				_Vector3 local_x = _Vector3(1, 0, 0);
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
					J.block<1, 3>(i, velStartIndex[i]) = (J_rotation[i].transpose() * Math::ToSkewSymmetricMatrix(local_x_new) * local_x).transpose();
				}	
			}
			else if (jointType[i] == BALL_JOINT_4D)
			{
				_Matrix3 R_swing;
				_Matrix3 R_twist;
				_Vector3 twistAxis(0, -1, 0);
				Math::TwistSwingDecompsition(R_local[i], twistAxis, R_twist, R_swing);

			/*	_Vector3 vec_swing = Math::RotationConversion_MatrixToVec(R_swing);
				vec_swing.normalize();*/
				_Vector3 vec_twist = Math::RotationConversion_MatrixToVec(R_twist);
				vec_twist.normalize();
				vec_twist = -vec_twist;
				J.block<1, 3>(i, velStartIndex[i]) = vec_twist.transpose();
			}

			_Vector3 rdot = qdot.segment(velStartIndex[i], 3);
			_Matrix JV = J.block<1, 3>(k, velStartIndex[i]) * rdot;

			_Scalar beta = 0.2f;//0.4f;
			_Scalar CR = 0.4f;// 0.2f;
			_Scalar SlopP = 0.001f;
			bias(k) = -beta / h * std::max<_Scalar>(-g[i], 0.0) - CR * std::max<_Scalar>(-JV(0, 0), 0.0);
		}
		_Matrix lambda;
		lambda = (J * MrInverse * J.transpose()).inverse() * (-J * qdot - bias);
		//std::cout << lambda << std::endl;

		for (int i = 0; i < constraintNum; i++)
		{
			if (lambda(i, 0) < 0) lambda(i, 0) = 0;
		}

		_Vector RdotCorrection = MrInverse * J.transpose() * lambda;
		//std::cout << J.transpose() << std::endl << std::endl;
		//std::cout << MrInverse << std::endl;
		/*std::cout << qdot.transpose() << std::endl;
		std::cout << qdot.norm() << std::endl;*/
		qdot = qdot + RdotCorrection;
	/*	std::cout << qdot.transpose() << std::endl;
		std::cout << RdotCorrection.norm() << std::endl;*/
	}
}