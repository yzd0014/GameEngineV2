#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>

void eae6320::MultiBody::UpdateInitialPosition()
{
	for (int i = 0; i < numOfLinks; i++)
	{
		lastValidOri[i] = rel_ori[i];
		_Scalar eulerAngles[3];
		GetEulerAngles(i, rel_ori[i], eulerAngles);
		mAlpha[i] = eulerAngles[2];
		mBeta[i] = eulerAngles[1];
		mGamma[i] = eulerAngles[0];
	}
}

_Scalar eae6320::MultiBody::ComputeSwingError(int jointNum)
{
	_Scalar out;
	out = twistAxis[jointNum].dot(R_local[jointNum] * twistAxis[jointNum]) - cos(jointRange[jointNum].first);
	return out;
}

void eae6320::MultiBody::SwitchConstraint(int i)
{
	_Scalar eulerEpsilon = 1e-6;
	if (M_PI * 0.5 - abs(mBeta[i]) > eulerEpsilon)
	{
		//check if switch is required
		_Quat oriDiff = rel_ori[i] * lastValidOri[i].inverse();
		_Vector3 deltaRot;
		deltaRot = Math::RotationConversion_QuatToVec(oriDiff);
		deltaRot = Math::RotationConversion_QuatToMat(eulerDecompositionOffset[i]) * deltaRot;
		
		_Scalar eulerAngles[3];
		GetEulerAngles(i, lastValidOri[i], eulerAngles);
		_Scalar oldAlpha = eulerAngles[2];
		_Scalar oldBeta = eulerAngles[1];
		_Scalar betaDiff;
		_Vector3 K(sin(oldAlpha), 0, cos(oldAlpha));
		betaDiff = K.dot(deltaRot);
		_Scalar newBeta = oldBeta + betaDiff;
		//std::cout << "---quat " << rel_ori[i] << std::endl;
		//std::cout << "----alpha " << mAlpha[i] << " beta " << mBeta[i] << " prediced beta: " << newBeta << std::endl;
		
		if (newBeta > 0.5 * M_PI || newBeta < -0.5 * M_PI)
		{
			vectorFieldNum[i] = !vectorFieldNum[i];
			std::cout << "============Switch (predicted beta)========================================================================: " << newBeta << std::endl;
		}
		lastValidOri[i] = rel_ori[i];
	}
	else
	{
		std::cout << "-----Inside singularity region" << std::endl;
	}
}

_Scalar eae6320::MultiBody::ComputeTwistEulerError(int jointNum)
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
	_Scalar sNorm = s.norm();
	if (sNorm > swingEpsilon)
	{
		s.normalize();
		out = s.dot(R_local[jointNum] * eulerZ[jointNum]) - cos(jointRange[jointNum].second);
	}
	else
	{
		std::cout << "Euler swing singluarity points are reached with zNorm: " << sNorm << std::endl;
	}
	if (adaptiveTimestep)
	{
		_Scalar dtEpsilon = 0.005;
		if (sNorm < dtEpsilon)
		{
			_Scalar newDt = 0.0001;
			pApp->UpdateDeltaTime(newDt);
			std::cout << "Finner dt is used " << newDt << std::endl;
		}
	}
	
	return out;
}

void eae6320::MultiBody::BallJointLimitCheck()
{
	jointsID.clear();
	constraintValue.clear();
	limitType.clear();
	totalJointError = 0;

	for (int i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_4D)
		{
			_Quat quat = Math::RotationConversion_MatToQuat(R_local[i]);
			_Quat twistComponent, swingComponent;
			_Scalar twistAngle, swingAngle;
			if ((jointRange[i].first > 0 || jointRange[i].second > 0) && twistMode == DIRECT)
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
				}
			}

			if (twistMode == DIRECT)
			{
				if (jointRange[i].second > 0 && jointRange[i].second - twistAngle < 0) //check twist constraint
				{
					_Scalar twistError = cos(twistAngle) - cos(jointRange[i].second);
					if (swingAngle < swingEpsilon || abs(swingAngle - M_PI) < swingEpsilon)
					{
						jointsID.push_back(i);
						constraintValue.push_back(twistError);
						limitType.push_back(TWIST_WITHOUT_SWING);
					}
					else
					{
						jointsID.push_back(i);
						constraintValue.push_back(twistError);
						limitType.push_back(TWIST_WITH_SWING);
					}
				}
			}
			else if (twistMode == EULER && jointRange[i].second > 0)
			{
				SwitchConstraint(i);
				_Scalar twistConstraint = ComputeTwistEulerError(i);
				if (twistConstraint < 0)
				{
					jointsID.push_back(i);
					constraintValue.push_back(twistConstraint);
					limitType.push_back(TWIST_EULER);
				}
			}
			else if (twistMode == EULER_V2 && jointRange[i].second > 0)
			{
				SwitchConstraint(i);
				if (M_PI * 0.5 - mBeta[i] > swingEpsilon) //velocity constrain can only be solved when beta is not too close to the singularity region
				{
					_Scalar correctedGamma = mGamma[i];
					if (vectorFieldNum[i] == 1)
					{
						if (mGamma[i] >= 0) correctedGamma = mGamma[i] - M_PI;
						else correctedGamma = mGamma[i] + M_PI;
					}

					_Scalar errForUpperBound = jointRange[i].second - correctedGamma;
					_Scalar errForLowerBound = correctedGamma + jointRange[i].second;
					if (errForUpperBound < 0)
					{
						jointsID.push_back(i);
						constraintValue.push_back(errForUpperBound);
						limitType.push_back(TWIST_EULER_MAX);
					}
				
					if (errForLowerBound < 0)
					{
						jointsID.push_back(i);
						constraintValue.push_back(errForLowerBound);
						limitType.push_back(TWIST_EULER_MIN);
					}
				}
				else
				{
					std::cout << "Euler swing singluarity points are reached with beta: " << mBeta[i] << std::endl;
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
			else if (twistMode == INCREMENT && jointRange[i].second > 0)
			{
				_Vector3 p = R_local[i] * twistAxis[i];
				_Vector3 omega = qdot.segment(velStartIndex[i], 3);
				_Scalar projectedOmega = p.dot(omega);
				_Scalar deltaTwist = projectedOmega * dt;
				totalTwist[i] += deltaTwist;
				//std::cout << "Total incremental twist " << totalTwist[i] << " twist increase " << omega.transpose() << std::endl;
				if (totalTwist[i] > jointRange[i].second || totalTwist[i] < -jointRange[i].second)
				{
					if (totalTwist[i] > 0) totalTwist[i] = jointRange[i].second - 0.000001;
					else if (totalTwist[i] < 0) totalTwist[i] = -jointRange[i].second + 0.000001;
					
					jointsID.push_back(i);
					constraintValue.push_back(0);
					limitType.push_back(TWIST_INCREMENT);
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

void eae6320::MultiBody::ComputeTwistEulerJacobian(int i, bool isUpperBound, _Matrix& o_J)
{
	_Matrix3 R_yzx = eulerDecompositionOffsetMat[i] * R_local[i] * eulerDecompositionOffsetMat[i].transpose();
	_Matrix J_yzx;
	J_yzx.resize(1, 3);
	J_yzx.setZero();
	_Scalar squareTerm = R_yzx(1, 2) * R_yzx(1, 2) + R_yzx(1, 1) * R_yzx(1, 1);
	J_yzx(0, 0) = (-R_yzx(2, 2) * R_yzx(1, 1) + R_yzx(2, 1) * R_yzx(1, 2)) / squareTerm;
	J_yzx(0, 2) = (R_yzx(0, 2) * R_yzx(1, 1) - R_yzx(0, 1) * R_yzx(1, 2)) / squareTerm;
	o_J = J_yzx * eulerDecompositionOffsetMat[i];
	if (!isUpperBound)
	{
		o_J = -o_J;
	}
}

void eae6320::MultiBody::ComputeTwistDirectJacobian(int jointNum, int i_limitType, _Matrix& o_J)
{
	int i = jointNum;
	if (i_limitType == TWIST_WITH_SWING)
	{
		_Vector3 t_rotated = R_local[i] * twistAxis[i];
		_Vector3 s = twistAxis[i].cross(t_rotated);
		_Matrix M;
		M = Math::ToSkewSymmetricMatrix(twistAxis[jointNum]) * Math::ToSkewSymmetricMatrix(t_rotated);

		_Matrix T0;
		_Vector3 s_rotated = R_local[i] * s;
		T0 = -s.squaredNorm() * (s_rotated.transpose() * M + s.transpose() *  Math::ToSkewSymmetricMatrix(s_rotated) + s.transpose() * R_local[i] * M);

		_Matrix T1;
		T1 = 2 * s.transpose() * s_rotated * s.transpose() * M;

		o_J = (T0 + T1) / (s.squaredNorm() * s.squaredNorm());
	}
	else
	{
		_Vector3 p = Math::GetOrthogonalVector(twistAxis[i]);
		o_J = ((R_local[jointNum] * p).cross(p)).transpose();
	}
}

void eae6320::MultiBody::SolveVelocityJointLimit(const _Scalar h)
{
	if (constraintNum > 0)
	{
		J_constraint.resize(constraintNum, totalVelDOF);
		J_constraint.setZero();
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
					_Matrix mJ;
					ComputeTwistDirectJacobian(i, limitType[k], mJ);
					J_constraint.block<1, 3>(k, velStartIndex[i]) = mJ;
				}
				else if (limitType[k] == SWING)
				{
					_Matrix mJ;
					ComputeSwingJacobian(i, mJ);
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

					J_constraint.block<1, 3>(k, velStartIndex[i]) = -rNormalized.transpose() * G;
				}
				else if (limitType[k] == TWIST_INCREMENT)
				{
					_Vector3 p = R_local[i] * twistAxis[i];
					_Scalar dotProduct = p.dot(qdot.segment(velStartIndex[i], 3));
					if (dotProduct > 0) p = -p;
			
					J_constraint.block<1, 3>(k, velStartIndex[i]) = p;
				}
				else if (limitType[k] == TWIST_EULER)
				{
					_Matrix mJ;
					ComputeTwistEulerJacobian(i, mJ);
					J_constraint.block<1, 3>(k, velStartIndex[i]) = mJ;
				}
				else if (limitType[k] == TWIST_EULER_MAX)
				{
					_Matrix mJ;
					ComputeTwistEulerJacobian(i, true, mJ);
					J_constraint.block<1, 3>(k, velStartIndex[i]) = mJ;
				}
				else if (limitType[k] == TWIST_EULER_MIN)
				{
					_Matrix mJ;
					ComputeTwistEulerJacobian(i, false, mJ);
					J_constraint.block<1, 3>(k, velStartIndex[i]) = mJ;
				}
			}
			//compute bias
			_Vector3 v = qdot.segment(velStartIndex[i], 3);
			_Matrix C_dot = J_constraint.block<1, 3>(k, velStartIndex[i]) * v;
			_Scalar CR = 0;
			bias(k) = -CR * std::max<_Scalar>(-C_dot(0, 0), 0.0);
		}
		_Matrix lambda;
		_Matrix T;
		_Matrix mIdentity;
		mIdentity = _Matrix::Identity(constraintNum, constraintNum);
		
		T = J_constraint * MrInverse * J_constraint.transpose();
		_Scalar deltaSquared = abs(T.maxCoeff()) * 1e-6;
		effectiveMass0 = (T + deltaSquared * mIdentity).inverse();
		lambda = effectiveMass0 * (-J_constraint * qdot - bias);
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
			//_Scalar beta = 0.9f;
			_Scalar beta = 0.1;
			//_Scalar SlopP = 0.00001f;
			_Scalar SlopP = 0;
			error(k) = beta * std::max<_Scalar>(-constraintValue[k] - SlopP, 0.0);
		}
		_Matrix lambda;
		lambda = effectiveMass0 * error;
		_Vector qCorrection = MrInverse * J_constraint.transpose() * lambda;
		Integrate_q(q, rel_ori, q, rel_ori, qCorrection, 1.0);
	}
}