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
			if (jointRange[i].first > 0 || jointRange[i].second > 0)
			{
				_Vector3 p = twistAxis[i];
				Math::SwingTwistDecomposition(quat, p, swingComponent, twistComponent);
				_Vector3 twistVec = Math::RotationConversion_QuatToVec(twistComponent);
				twistAngle = twistVec.norm();
				_Vector3 swingVec = Math::RotationConversion_QuatToVec(swingComponent);
				swingAngle = swingVec.norm();
				//std::cout << "twist: " << twistAngle << ", swing: " << swingAngle << std::endl;
			}
			if (jointRange[i].first > 0 && jointRange[i].first - swingAngle < 0)//check swing constraint
			{
				jointsID.push_back(i);
				constraintValue.push_back(jointRange[i].first - swingAngle);
				limitType.push_back(SWING);
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
			else if (swingMode == EULER_SWING)
			{
				_Matrix3 R = Math::RotationConversion_QuatToMat(rel_ori[i]);
				_Vector3 rotatedX = R * eulerX[i];
				_Vector3 rotatedZ;
				if (vectorFieldNum == 0)
				{
					rotatedZ = rotatedX.cross(eulerY[i]);
				}
				else
				{
					rotatedZ = eulerY[i].cross(rotatedX);
				}
				_Scalar zNorm = rotatedZ.norm();
				if (zNorm > swingEpsilon)
				{
					rotatedZ.normalize();
					if (rotatedZ.dot(oldEulerZ[i]) < 0)//vector field switch
					{
						vectorFieldNum = !vectorFieldNum;
						rotatedZ = -rotatedZ;
						std::cout << "Vector field switched with zNorm: " << zNorm << std::endl;
					}
					
					_Scalar twistConstraint = rotatedZ.dot(R * eulerZ[i]) - cos(jointRange[i].second);
					
					if (twistConstraint < 0)
					{
						jointsID.push_back(i);
						constraintValue.push_back(twistConstraint);
						limitType.push_back(TWIST_EULER);
					}
					//std::cout << rotatedZ.dot(oldEulerZ) << std::endl << std::endl;
					oldEulerZ[i] = rotatedZ;
				}
				else
				{
					std::cout << "Euler swing singluarity points are reached with zNorm: " << zNorm << std::endl;
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
}

void eae6320::MultiBody::SolveVelocityJointLimit(const _Scalar h)
{
	size_t constraintNum = jointsID.size();

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
					_Vector3 p = twistAxis[i];
					//compute J
					_Scalar j0 = ComputeAngularVelocityConstraint(_Vector3(1, 0, 0), p, R_local[i], limitType[k], jointRange[i].first);
					_Scalar j1 = ComputeAngularVelocityConstraint(_Vector3(0, 1, 0), p, R_local[i], limitType[k], jointRange[i].first);
					_Scalar j2 = ComputeAngularVelocityConstraint(_Vector3(0, 0, 1), p, R_local[i], limitType[k], jointRange[i].first);
					J.block<1, 3>(k, velStartIndex[i]) = _Vector3(j0, j1, j2);

					//compute K
					K.block<1, 3>(k, velStartIndex[i]) = _Vector3(j0, j1, j2);
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
					//TODO
					_Matrix3 R = R_local[i];
					_Matrix A0;
					_Vector3 mVec;
					mVec = Math::ToSkewSymmetricMatrix(eulerY[i]) * R * eulerZ[i];
					A0 = eulerX[i].transpose() * R.transpose() * Math::ToSkewSymmetricMatrix(mVec);
					_Matrix A1;
					mVec = R * eulerZ[i];
					A1 = -eulerX[i].transpose() * R.transpose() * Math::ToSkewSymmetricMatrix(eulerY[i]) * Math::ToSkewSymmetricMatrix(mVec);
					_Matrix A2;
					_Vector3 s = -eulerY[i].cross(R * eulerX[i]);
					mVec = R * eulerX[i];
					A2 = -cos(jointRange[i].second) / s.norm() * s.transpose() * Math::ToSkewSymmetricMatrix(eulerY[i]) * Math::ToSkewSymmetricMatrix(mVec);
					_Matrix mJ;
					mJ.resize(1, 3);
					if (vectorFieldNum == 0)
					{
						mJ = A0 + A1 + A2;
					}
					else
					{
						mJ = -A0 - A1 + A2;
					}
					J.block<1, 3>(k, velStartIndex[i]) = mJ;
					K.block<1, 3>(k, velStartIndex[i]) = mJ;
				}
			}

			//compute bias
			_Vector3 v = qdot.segment(velStartIndex[i], 3);
			_Matrix C_dot = J.block<1, 3>(k, velStartIndex[i]) * v;
			_Scalar beta = 0.001f;
			_Scalar CR = 0.0f;
			_Scalar SlopP = 0.001f;
			bias(k) = -beta / h * std::max<_Scalar>(-constraintValue[i] - SlopP, 0.0) - CR * std::max<_Scalar>(-C_dot(0, 0), 0.0);
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
			Ht[i].block(0, velStartIndex[k], 6, velDOF[k]) = H_temp;
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
			q.segment(posStartIndex[i], 3) = x.segment(xStartIndex[i], 3);
			_Vector3 rotVec = x.segment(xStartIndex[i] + 3, 3);
			rel_ori[i] = Math::RotationConversion_VecToQuat(rotVec);
		}
		else
		{
			q.segment(posStartIndex[i], xDOF[i]) = x.segment(xStartIndex[i], xDOF[i]);
		}
	}
}

void eae6320::MultiBody::ComputeJ_rotation(_Vector& i_x)
{
	for (int i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_4D || jointType[i] == BALL_JOINT_3D)
		{
			_Vector3 r = i_x.segment(xStartIndex[i], 3);
			_Scalar theta = r.norm();
			_Scalar b = Compute_b(theta);
			_Scalar a = Compute_a(theta);
			_Scalar c = Compute_c(theta, a);
			J_rotation[i] = _Matrix::Identity(3, 3) + b * Math::ToSkewSymmetricMatrix(r) + c * Math::ToSkewSymmetricMatrix(r) * Math::ToSkewSymmetricMatrix(r);
		}
	}
}

void eae6320::MultiBody::SolvePositionJointLimit()//TODO
{
	_Matrix J;
	size_t constraintNum = jointsID.size();
	J.resize(constraintNum, totalXDOF);
	J.setZero();

	_Matrix C;
	C.resize(constraintNum, 1);

	for (size_t k = 0; k < constraintNum; k++)
	{
		int i = jointsID[k];
		J.block<1, 3>(k, xStartIndex[i]) = Jc_jointLimit.block<1, 3>(k, velStartIndex[i]) * J_rotation[i];
	}
	_Matrix A = J * MrInverse * J.transpose();
	_Matrix lambda = A.inverse() * -C;
}