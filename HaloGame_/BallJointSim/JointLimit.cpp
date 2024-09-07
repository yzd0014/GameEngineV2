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
			if (twistArrow != nullptr)
			{
				twistArrow->DestroyGameObject();
				twistArrow = nullptr;
			}
			twistArrow = GameplayUtility::DrawArrowScaled(jointPos[0], rotatedZ, Math::sVector(1, 0, 0), Vector3d(0.5, 1, 0.5));
			if (swingArrow != nullptr)
			{
				swingArrow->DestroyGameObject();
				swingArrow = nullptr;
			}
			swingArrow = GameplayUtility::DrawArrowScaled(jointPos[0], oldEulerZ[jointNum], Math::sVector(0, 0, 1), Vector3d(0.5, 1, 0.5));

			_Scalar eulerAngles[3];
			_Quat inputQuat = eulerDecompositionOffset[jointNum] * rel_ori[jointNum] * eulerDecompositionOffset[jointNum].inverse();
			Math::quaternion2Euler(inputQuat, eulerAngles, Math::RotSeq::yzx);
			std::cout << "Twsit angle: " << eulerAngles[0] << " " << eulerAngles[1] << " " << eulerAngles[2] << " x: " << rotatedX(0) << " " << rotatedX(1) << std::endl;
			//std::cout << rotatedX.transpose() << std::endl;

			bool closeToSingularity = FALSE;
			_Vector3 localX = userToLocalTransform[jointNum] * rotatedX;
			_Scalar testAngle = Math::GetAngleBetweenTwoVectors(localX, lastTwistAxis[jointNum]);
			_Scalar singularityEpsilon = 0.02;
			if (testAngle > singularityEpsilon)
			{
				_Vector3 pointPojection = Math::PointToTriangleDis(_Vector3(0, 0, 1), localX, lastTwistAxis[jointNum], _Vector3(0, 0, 0));
				_Scalar dist = (_Vector3(0, 0, 1) - pointPojection).norm();
				if (dist < singularityEpsilon)
				{
					closeToSingularity = TRUE;
				}
			}
			else
			{
				_Scalar testDist0 = (localX - _Vector3(0, 0, 1)).norm();
				_Scalar testDist1 = (localX - _Vector3(0, 0, -1)).norm();
				if (testDist0 < singularityEpsilon || testDist1 < singularityEpsilon)
				{
					closeToSingularity = TRUE;
				}
			}
			//if (dotProduct < 0)//vector field switch
			if (eulerAngles[0] * lastTwistAngle[jointNum] < 0 && eulerAngles[2] * lastEulerY[jointNum] < 0)
			{
				vectorFieldNum[jointNum] = !vectorFieldNum[jointNum];
				rotatedZ = -rotatedZ;
				std::cout << "Vector field switched " << std::endl;
			}
			oldEulerZ[jointNum] = rotatedZ;
			lastTwistAngle[jointNum] = eulerAngles[0];
			lastEulerY[jointNum] = eulerAngles[2];
			lastTwistAxis[jointNum] = localX;
		}
		out = rotatedZ.dot(R_local[jointNum] * eulerZ[jointNum]) - cos(jointRange[jointNum].second);
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
				//std::cout << "TWIST_EULER " << twistConstraint << std::endl;
				if (twistConstraint < 0)
				{
					jointsID.push_back(i);
					constraintValue.push_back(twistConstraint);
					limitType.push_back(TWIST_EULER);
					//std::cout << "TWIST_EULER " << twistConstraint << std::endl;
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
		J_constraint = J;
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
					J_constraint.block<1, 3>(k, velStartIndex[i]) = pRotated;
					//K.block<1, 3>(k, velStartIndex[i]) = J.block<1, 3>(k, velStartIndex[i]);
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
					J_constraint.block<1, 3>(k, velStartIndex[i]) = mJ;
				}
			}

			//compute bias
			_Vector3 v = qdot.segment(velStartIndex[i], 3);
			_Matrix C_dot = J.block<1, 3>(k, velStartIndex[i]) * v;
			_Scalar CR = 0.0f;
			bias(k) = -CR * std::max<_Scalar>(-C_dot(0, 0), 0.0);
		}
		_Matrix lambda;
		effectiveMass = (J * MrInverse * J_constraint.transpose()).inverse();
		lambda = effectiveMass * (-J * qdot - bias);

		for (size_t k = 0; k < constraintNum; k++)
		{
			if (lambda(k, 0) < 0)
			{
				lambda(k, 0) = 0;
			}
		}
		_Vector qdotCorrection = MrInverse * J_constraint.transpose() * lambda;
		qdot = qdot + qdotCorrection;
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
		_Matrix lambda = effectiveMass * error;
		_Vector qCorrection = Mr.inverse() * J_constraint.transpose() * lambda;
		Integrate_q(q, rel_ori, q, rel_ori, qCorrection, 1.0);
	}
}