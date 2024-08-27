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
	_Matrix3 R = Math::RotationConversion_QuatToMat(rel_ori[jointNum]);
	out = twistAxis[jointNum].dot(R * twistAxis[jointNum]) - cos(jointRange[jointNum].first);
	return out;
}

_Scalar eae6320::MultiBody::ComputeTwistEulerError(int jointNum, bool checkVectorField)
{
	_Scalar out = 0;
	_Matrix3 R = Math::RotationConversion_QuatToMat(rel_ori[jointNum]);
	_Vector3 rotatedX = R * eulerX[jointNum];
	_Vector3 rotatedZ;
	if (vectorFieldNum[jointNum] == 0)
	{
		rotatedZ = rotatedX.cross(eulerY[jointNum]);
	}
	else
	{
		rotatedZ = eulerY[jointNum].cross(rotatedX);
	}
	_Scalar zNorm = rotatedZ.norm();
	if (zNorm > swingEpsilon)
	{
		rotatedZ.normalize();
		if (checkVectorField)
		{
			_Scalar dotProduct = rotatedZ.dot(oldEulerZ[jointNum]);
			if (dotProduct < 0)//vector field switch
			{
				vectorFieldNum[jointNum] = !vectorFieldNum[jointNum];
				rotatedZ = -rotatedZ;
				std::cout << "Vector field switched " << std::endl;
			}
			oldEulerZ[jointNum] = rotatedZ;
		}
		out = rotatedZ.dot(R * eulerZ[jointNum]) - cos(jointRange[jointNum].second);
	}
	else
	{
		if (checkVectorField) std::cout << "Euler swing singluarity points are reached with zNorm: " << zNorm << std::endl;
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
					//std::cout << "SWING " << constraintValue.back() << std::endl;
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

void eae6320::MultiBody::ComputeSwingJacobian(int jointNum, _Matrix3& i_R, _Matrix& o_J)
{
	o_J = ((i_R * twistAxis[jointNum]).cross(twistAxis[jointNum])).transpose();
}

void eae6320::MultiBody::ComputeTwistEulerJacobian(int jointNum, _Matrix3& i_R, _Matrix& o_J)
{
	_Matrix A0;
	_Vector3 mVec;
	mVec = Math::ToSkewSymmetricMatrix(eulerY[jointNum]) * i_R * eulerZ[jointNum];
	A0 = eulerX[jointNum].transpose() * i_R.transpose() * Math::ToSkewSymmetricMatrix(mVec);
	_Matrix A1;
	mVec = i_R * eulerZ[jointNum];
	A1 = -eulerX[jointNum].transpose() * i_R.transpose() * Math::ToSkewSymmetricMatrix(eulerY[jointNum]) * Math::ToSkewSymmetricMatrix(mVec);
	_Matrix A2;
	_Vector3 s = -eulerY[jointNum].cross(i_R * eulerX[jointNum]);
	mVec = i_R * eulerX[jointNum];
	A2 = -cos(jointRange[jointNum].second) / s.norm() * s.transpose() * Math::ToSkewSymmetricMatrix(eulerY[jointNum]) * Math::ToSkewSymmetricMatrix(mVec);
	o_J.resize(1, 3);
	if (vectorFieldNum[jointNum] == 0)
	{
		o_J = A0 + A1 + A2;
	}
	else
	{
		o_J = -A0 - A1 + A2;
	}
}

void eae6320::MultiBody::SolveVelocityJointLimit(const _Scalar h)
{
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
					//K.block<1, 3>(k, velStartIndex[i]) = J.block<1, 3>(k, velStartIndex[i]);
				}
				else if (limitType[k] == SWING)
				{
					_Matrix mJ;
					ComputeSwingJacobian(i, R_local[i], mJ);
					J.block<1, 3>(k, velStartIndex[i]) = mJ;
					K.block<1, 3>(k, velStartIndex[i]) = mJ;
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
					K.block<1, 3>(k, velStartIndex[i]) = J.block<1, 3>(k, velStartIndex[i]);
				}
				else if (limitType[k] == TWIST_EULER)
				{
					_Matrix mJ;
					ComputeTwistEulerJacobian(i, R_local[i], mJ);
					J.block<1, 3>(k, velStartIndex[i]) = mJ;
					K.block<1, 3>(k, velStartIndex[i]) = mJ;
				}
			}

			//compute bias
			_Vector3 v = qdot.segment(velStartIndex[i], 3);
			_Matrix C_dot = J.block<1, 3>(k, velStartIndex[i]) * v;
			_Scalar CR = 0.0f;
			bias(k) = -CR * std::max<_Scalar>(-C_dot(0, 0), 0.0);
		}
		_Matrix lambda;
		lambda = (J * MrInverse * K.transpose()).inverse() * (-J * qdot - bias);
		Jc_jointLimit = J;//backup J for position solve

		for (size_t k = 0; k < constraintNum; k++)
		{
			if (lambda(k, 0) < 0)
			{
				lambda(k, 0) = 0;
			}
		}
		_Vector qdotCorrection = MrInverse * K.transpose() * lambda;
		qdot = qdot + qdotCorrection;
	}
}

void eae6320::MultiBody::PrePositionSolveProccessing()
{
	Mr.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_4D)
		{
			x.segment(xStartIndex[i], 3) = Math::RotationConversion_QuatToVec(rel_ori[i]);

			//update H for 4D ball joint
			_Vector3 r = x.segment(xStartIndex[i], 3);
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
		else if (jointType[i] == FREE_JOINT)
		{
			x.segment(xStartIndex[i], 3) = q.segment(posStartIndex[i], 3);
			x.segment(xStartIndex[i] + 3, 3) = Math::RotationConversion_QuatToVec(rel_ori[i]);
		}
		else
		{
			x.segment(xStartIndex[i], xDOF[i]) = q.segment(posStartIndex[i], xDOF[i]);
		}
		//update Ht
		Ht[i].resize(6, totalXDOF);
		Ht[i].setZero();
		for (int k = 0; k <= i; k++)//TODO
		{
			_Matrix H_temp;
			H_temp.resize(6, 3);
			H_temp = H[k];
			for (int j = k + 1; j <= i; j++)
			{
				H_temp = D[j] * H_temp;
			}
			Ht[i].block(0, xStartIndex[k], 6, xDOF[k]) = H_temp;
		}
		
		//update Mr
		_Matrix M_temp = Ht[i].transpose() * Mbody[i] * Ht[i];
		Mr = Mr + M_temp;
	}
	if (Mr.determinant() < 0.0000001)
	{
		EAE6320_ASSERTF(false, "mass matrix singluarity reached!");
	}
	else
	{
		MrInverse = Mr.inverse();
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
			q.segment(posStartIndex[i], 3) = x.segment(xStartIndex[i], 3);
			_Vector3 rotVec = x.segment(xStartIndex[i] + 3, 3);
			rel_ori[i] = Math::RotationConversion_VecToQuat(rotVec);
		}
		else
		{
			q.segment(posStartIndex[i], posDOF[i]) = x.segment(xStartIndex[i], xDOF[i]);
		}
	}
}

void eae6320::MultiBody::SolvePositionJointLimit()
{
	_Matrix J;
	J.resize(constraintNum, totalXDOF);
	J.setZero();

	_Matrix C;
	C.resize(constraintNum, 1);
	for (size_t k = 0; k < constraintNum; k++)
	{
		int i = jointsID[k];
		if (jointType[i] == BALL_JOINT_4D)
		{
			J.block<1, 3>(k, xStartIndex[i]) = Jc_jointLimit.block<1, 3>(k, velStartIndex[i]) * J_rotation[i];
			if (limitType[k] == SWING)
			{
				C(k, 0) = ComputeSwingError(i);
			}
			else if (limitType[k] == TWIST_EULER)
			{
				C(k, 0) = ComputeTwistEulerError(i, FALSE);
			}
		}
		else
		{
			J.block<1, 3>(k, xStartIndex[i]) = Jc_jointLimit.block<1, 3>(k, velStartIndex[i]);
		}
	}
	_Matrix A = J * MrInverse * J.transpose();
	_Matrix lambda = A.inverse() * -C;
	_Vector xCorrection = MrInverse * J.transpose() * lambda;
	x = x + xCorrection;
}