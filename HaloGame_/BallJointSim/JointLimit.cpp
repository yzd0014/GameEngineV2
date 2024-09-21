#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>

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

_Scalar eae6320::MultiBody::ComputeSwingError(int jointNum)
{
	_Scalar out;
	out = twistAxis[jointNum].dot(R_local[jointNum] * twistAxis[jointNum]) - cos(jointRange[jointNum].first);
	return out;
}

_Scalar eae6320::MultiBody::ComputeTwistEulerError(int jointNum, bool checkVectorField)
{
	_Scalar out = 0;
	_Vector3 rotatedX = R_local[jointNum] * eulerX[jointNum];
	_Vector3 s;
	if (vectorFieldNum[jointNum] == 0)
	{
		s = rotatedX.cross(eulerY[jointNum]);
	}
	else
	{
		s = eulerY[jointNum].cross(rotatedX);
	}
	
	//integrate beta
	if (checkVectorField)
	{
		//std::cout << "----Twsit angle: " << mAlpha[jointNum] << " " << mBeta[jointNum] << " " << mGamma[jointNum] << std::endl;
		_Scalar alpha = mAlpha[jointNum];
		_Scalar beta = mBeta[jointNum];
		if (abs(abs(beta) - 0.5 * M_PI) > 0.0000001)
		{
			_Scalar betaDot;
			_Vector3 omega = qdot.segment(velStartIndex[jointNum], 3);
			omega = Math::RotationConversion_QuatToMat(eulerDecompositionOffset[jointNum]) * omega;
			_Matrix Rz = Math::RotationConversion_VecToMatrix(_Vector3(0, 0, -0.5 * M_PI));
			_Matrix Rx = Math::RotationConversion_VecToMatrix(_Vector3(-0.5 * M_PI, 0, 0));
			omega = Rx * Rz * omega;
			betaDot = cos(alpha) * omega(1) - sin(alpha) * omega(2);
			_Scalar newBeta = beta + dt * betaDot;
			if (newBeta > 0.5 * M_PI || newBeta < -0.5 * M_PI)
			{
				vectorFieldNum[jointNum] = !vectorFieldNum[jointNum];
				std::cout << "Switch (predicted beta): " << newBeta << std::endl;
			}
			//std::cout << "beta: " << beta << " beta dot: " << betaDot << " prediced beta: " << newBeta << std::endl << std::endl;
			//std::cout << "---------Predicted beta " << newBeta << std::endl;
		}
		else
		{
			std::cout << "No unique Euler decomposition" << std::endl;
		}
	}

	_Scalar sNorm = s.norm();
	if (sNorm > swingEpsilon)
	{
		//std::cout << "sNorm " << s.norm() << std::endl;
		s.normalize();
		out = s.dot(R_local[jointNum] * eulerZ[jointNum]) - cos(jointRange[jointNum].second);
		//std::cout << "Position error " << out << std::endl;
	}
	else
	{
		if (checkVectorField) std::cout << "Euler swing singluarity points are reached with zNorm: " << sNorm << std::endl;
	}
	
	return out;
}

void eae6320::MultiBody::BallJointLimitCheck()
{
	jointsID.clear();
	constraintValue.clear();
	limitType.clear();

	for (int i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_4D)
		{
			_Quat quat = Math::RotationConversion_MatToQuat(R_local[i]);
			_Quat twistComponent, swingComponent;
			_Scalar twistAngle, swingAngle;
			if ((jointRange[i].first > 0 || jointRange[i].second > 0) && swingMode == DIRECT_SWING)
			{
				_Vector3 p = twistAxis[i];
				Math::SwingTwistDecomposition(quat, p, swingComponent, twistComponent);
				_Vector3 twistVec = Math::RotationConversion_QuatToVec(twistComponent);
				twistAngle = twistVec.norm();
				_Vector3 swingVec = Math::RotationConversion_QuatToVec(swingComponent);
				swingAngle = swingVec.norm();
			}
			if (jointRange[i].first > 0)//check swing constraint
			{
				_Scalar swingConstraint = ComputeSwingError(i);
				if (swingConstraint < 0)
				{
					jointsID.push_back(i);
					constraintValue.push_back(swingConstraint);
					limitType.push_back(SWING);
					//std::cout << "SWING " << swingConstraint << std::endl;
				}
			}

			if (swingMode == DIRECT_SWING)
			{
				if (jointRange[i].second > 0 && jointRange[i].second - twistAngle < 0) //check twist constraint
				{
					if (swingAngle < swingEpsilon || abs(swingAngle - M_PI) < swingEpsilon)
					{
						jointsID.push_back(i);
						constraintValue.push_back(jointRange[i].second - twistAngle);
						limitType.push_back(TWIST_WITHOUT_SWING);
					}
					else
					{
						jointsID.push_back(i);
						constraintValue.push_back(jointRange[i].second - twistAngle);
						limitType.push_back(TWIST_WITH_SWING);
					}
				}
			}
			else if (swingMode == EULER_SWING && jointRange[i].second > 0)
			{
				_Scalar twistConstraint = ComputeTwistEulerError(i, TRUE);
				if (twistConstraint < 0)
				{
					jointsID.push_back(i);
					constraintValue.push_back(twistConstraint);
					limitType.push_back(TWIST_EULER);
					//std::cout << "Twist violation " << twistConstraint << std::endl;
				}
				else
				{
					//std::cout << "No twist violation " << twistConstraint << std::endl;
				}
			}
			else if (jointLimit[i] > 0)
			{
				_Vector3 rotVec = Math::RotationConversion_QuatToVec(rel_ori[i]);
				_Scalar rotAngle = rotVec.norm();
				if (jointLimit[i] - rotAngle < 0)
				{
					jointsID.push_back(i);
					constraintValue.push_back(jointLimit[i] - rotAngle);
					limitType.push_back(ROTATION_MAGNITUDE_LIMIT);
				}
			}
		}
	}
	constraintNum = jointsID.size();
}

void eae6320::MultiBody::ComputeSwingJacobian(int jointNum, _Matrix& o_J)
{
	o_J = ((R_local[jointNum] * twistAxis[jointNum]).cross(twistAxis[jointNum])).transpose();
}

void eae6320::MultiBody::ComputeTwistEulerJacobian(int jointNum, _Matrix& o_J)
{
	_Matrix A0;
	_Vector3 mVec;
	mVec = Math::ToSkewSymmetricMatrix(eulerY[jointNum]) * R_local[jointNum] * eulerZ[jointNum];
	A0 = eulerX[jointNum].transpose() * R_local[jointNum].transpose() * Math::ToSkewSymmetricMatrix(mVec);
	_Matrix A1;
	mVec = R_local[jointNum] * eulerZ[jointNum];
	A1 = -eulerX[jointNum].transpose() * R_local[jointNum].transpose() * Math::ToSkewSymmetricMatrix(eulerY[jointNum]) * Math::ToSkewSymmetricMatrix(mVec);
	_Matrix A2;
	_Vector3 s = -eulerY[jointNum].cross(R_local[jointNum] * eulerX[jointNum]);
	mVec = R_local[jointNum] * eulerX[jointNum];
	A2 = s.transpose() * Math::ToSkewSymmetricMatrix(eulerY[jointNum]) * Math::ToSkewSymmetricMatrix(mVec);
	o_J.resize(1, 3);
	_Scalar sNorm = s.norm();
	o_J = (A0 + A1) / sNorm - s.dot(R_local[jointNum] * eulerZ[jointNum]) / (sNorm * sNorm * sNorm) * A2;
	if (vectorFieldNum[jointNum] == 1)
	{
		o_J = -o_J;
	}
}

void eae6320::MultiBody::SolveVelocityJointLimit(const _Scalar h)
{
	if (constraintNum > 0)
	{
		_Matrix J, K;
		J.resize(constraintNum, totalVelDOF);
		J.setZero();
		J_constraint = J;
		K = J;
		_Vector bias;
		bias.resize(constraintNum);
		bias.setZero();
		for (size_t k = 0; k < constraintNum; k++)
		{
			int i = jointsID[k];
			//compute J and K
			if (jointType[i] == BALL_JOINT_4D)
			{
				if (limitType[k] == TWIST_WITH_SWING || limitType[k] == TWIST_WITHOUT_SWING)
				{
					_Vector3 p = twistAxis[i];
					//compute J
					_Scalar j0 = ComputeAngularVelocityConstraint(_Vector3(1, 0, 0), p, R_local[i], limitType[k], jointRange[i].second);
					_Scalar j1 = ComputeAngularVelocityConstraint(_Vector3(0, 1, 0), p, R_local[i], limitType[k], jointRange[i].second);
					_Scalar j2 = ComputeAngularVelocityConstraint(_Vector3(0, 0, 1), p, R_local[i], limitType[k], jointRange[i].second);
					J.block<1, 3>(k, velStartIndex[i]) = _Vector3(j0, j1, j2);

					//compute K
					_Vector3 pRotated = R_local[i] * p;
					_Scalar cTest = ComputeAngularVelocityConstraint(pRotated, p, R_local[i], limitType[k], jointRange[i].second);
					if (cTest < 0)
					{
						pRotated = -pRotated;
					}
					K.block<1, 3>(k, velStartIndex[i]) = pRotated;
					J_constraint.block<1, 3>(k, velStartIndex[i]) = K.block<1, 3>(k, velStartIndex[i]);
				}
				else if (limitType[k] == SWING)
				{
					_Matrix mJ;
					ComputeSwingJacobian(i, mJ);
					J.block<1, 3>(k, velStartIndex[i]) = mJ;
					J_constraint.block<1, 3>(k, velStartIndex[i]) = mJ;
				}
				else if (limitType[k] == ROTATION_MAGNITUDE_LIMIT)
				{
					_Vector3 r = Math::RotationConversion_QuatToVec(rel_ori[i]);
					_Scalar theta = r.norm();
					_Vector3 rNormalized = r / theta;

					_Scalar a = Compute_a(theta);
					_Scalar b = Compute_b(theta);
					_Scalar s = Compute_s(theta, a, b);
					_Matrix3 G = _Matrix::Identity(3, 3) - 0.5 * Math::ToSkewSymmetricMatrix(r) + s * Math::ToSkewSymmetricMatrix(r) * Math::ToSkewSymmetricMatrix(r);

					J.block<1, 3>(k, velStartIndex[i]) = -rNormalized.transpose() * G;
					J_constraint.block<1, 3>(k, velStartIndex[i]) = J.block<1, 3>(k, velStartIndex[i]);
				}
				else if (limitType[k] == TWIST_EULER)
				{
					_Matrix mJ;
					ComputeTwistEulerJacobian(i, mJ);
					J.block<1, 3>(k, velStartIndex[i]) = mJ;
					J_constraint.block<1, 3>(k, velStartIndex[i]) = R_local[i] * eulerX[i];
				}
			}
			//compute bias
			_Vector3 v = qdot.segment(velStartIndex[i], 3);
			_Matrix C_dot = J.block<1, 3>(k, velStartIndex[i]) * v;
			_Scalar CR = 0.2f;
			bias(k) = -CR * std::max<_Scalar>(-C_dot(0, 0), 0.0);
		}
		_Matrix lambda;
		_Matrix T;
		T = J * MrInverse;
		effectiveMass0 = (T * J.transpose()).inverse();
		effectiveMass1 = (T * J_constraint.transpose()).inverse();
		lambda = effectiveMass0 * (-J * qdot - bias);
		for (size_t k = 0; k < constraintNum; k++)
		{
			if (lambda(k, 0) < 0)
			{
				lambda(k, 0) = 0;
			}
		}
		_Vector qdotCorrection = MrInverse * J.transpose() * lambda;//requires modification for making direct swing-twist model work
		qdot = qdot + qdotCorrection;
		//std::cout << "Qdot correction norm " << qdotCorrection.norm() << std::endl;
	}
}

void eae6320::MultiBody::SolvePositionJointLimit()
{
	if (constraintNum > 0)
	{
		_Vector error;
		error.resize(constraintNum);
		error.setZero();
		for (size_t k = 0; k < constraintNum; k++)
		{
			int i = jointsID[k];
			_Scalar beta = 0.1f;
			_Scalar SlopP = 0.00001f;
			error(k) = beta * std::max<_Scalar>(-constraintValue[k] - SlopP, 0.0);
		}
		_Matrix lambda;
		lambda = effectiveMass1 * error;
		_Vector qCorrection = MrInverse * J_constraint.transpose() * lambda;
		Integrate_q(q, rel_ori, q, rel_ori, qCorrection, 1.0);
		//std::cout << "Position correction" << std::endl;
	}
}