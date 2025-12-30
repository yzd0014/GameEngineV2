#include "MultiBody.h"

void eae6320::MultiBody::ComputeCloseLoopJacobian(_Matrix& o_J)
{
	std::vector<_Matrix> mH;
	mH.resize(numOfLinks);
	std::vector<_Matrix> mD;
	mD.resize(numOfLinks);
	_Vector3 m_uGlobalChild;
	_Vector3 m_uGlobalParent;
	for (int i = 0; i < numOfLinks; i++)
	{
		int j = parentArr[i];
		m_uGlobalChild = uGlobalsChild[i];
		m_uGlobalParent = uGlobalsParent[i];
		if (i == closeLoopLinkID)
		{
			m_uGlobalChild = R_global[i] * uPre;
		}
		else if (i == closeLoopLinkID + 1)
		{
			m_uGlobalParent = R_global[j] * uNext;
		}
		else if (i == numOfLinks - 1)
		{
			m_uGlobalChild = R_global[i] * uEnd;
		}

		mD[i].resize(6, 6);
		if (jointType[i] == BALL_JOINT_4D)
		{
			//compute H
			mH[i].resize(6, 3);
			mH[i].setZero();
			if (i == 0)
			{
				mH[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(m_uGlobalChild);
			}
			else
			{
				mH[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(m_uGlobalChild) * R_global[j];
				mH[i].block<3, 3>(3, 0) = R_global[j];
			}
			//compute D
			if (i > 0)
			{
				mD[i].setIdentity();
				mD[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(m_uGlobalChild) - Math::ToSkewSymmetricMatrix(m_uGlobalParent);
			}
		}
		else if (jointType[i] == BALL_JOINT_3D)
		{
			//compute H
			_Vector3 r = q.segment(posStartIndex[i], 3);
			mH[i] = ComputeExponentialMapJacobian(J_exp[i], r, i);
			//compute D
			if (i > 0)
			{
				mD[i].setIdentity();
				mD[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(m_uGlobalChild) - Math::ToSkewSymmetricMatrix(m_uGlobalParent);
			}
		}
		else if (jointType[i] == FREE_JOINT)
		{
			//compute H
			mH[i].resize(6, 6);
			mH[i].setIdentity();
			//compute D
			mD[i].setZero();
		}
		else if (jointType[i] == FREE_JOINT_EXPO)
		{
			mH[i].resize(6, 6);
			mH[i].setIdentity();

			_Vector3 r = q.segment(posStartIndex[i] + 3, 3);
			_Scalar theta = r.norm();
			_Scalar b = Compute_b(theta);
			_Scalar a = Compute_a(theta);
			_Scalar c = Compute_c(theta, a);
			mH[i].block<3, 3>(3, 3) = _Matrix::Identity(3, 3) + b * Math::ToSkewSymmetricMatrix(r) + c * Math::ToSkewSymmetricMatrix(r) * Math::ToSkewSymmetricMatrix(r);

			mD[i].setZero();
		}
		else if (jointType[i] == HINGE_JOINT)
		{
			//compute H
			mH[i].resize(6, 1);
			mH[i].block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(m_uGlobalChild) * hingeDirGlobals[i];
			mH[i].block<3, 1>(3, 0) = hingeDirGlobals[i];
			//G[i] = H[i];
			//compute D
			mD[i].setIdentity();
			if (i > 0)
			{
				_Vector3 hingeVec = hingeMagnitude[i] * hingeDirGlobals[i];
				_Vector3 iVec = m_uGlobalChild - m_uGlobalParent - hingeVec;
				mD[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(iVec);
			}
		}
	}
	//compute output
	_Matrix J_temp[2];
	int linkIDs[2] = { closeLoopLinkID, numOfLinks - 1 };
	for (int i = 0; i < 2; i++)
	{
		int k = linkIDs[i];
		J_temp[i].resize(6, totalVelDOF);
		J_temp[i].setZero();
		while (k != -1)
		{
			_Matrix D_temp;
			D_temp.resize(6, 6);
			D_temp.setIdentity();
			int j = linkIDs[i];
			while (j > k)
			{
				D_temp = D_temp * mD[j];
				j = parentArr[j];
			}
			J_temp[i].block(0, velStartIndex[k], 6, velDOF[k]) = D_temp * mH[k];
			k = parentArr[k];
		}
	}
	o_J.resize(3, totalVelDOF);
	o_J.block(0, 0, 3, totalVelDOF) = J_temp[0].block(0, 0, 3, totalVelDOF) - J_temp[1].block(0, 0, 3, totalVelDOF);
}

void eae6320::MultiBody::SolveCloseLoop()
{
	if (hasCloseLoop)
	{
		ComputeCloseLoopJacobian(J_constraint);

		_Matrix lambda;
		_Matrix K;
		_Matrix mIdentity;
		mIdentity = _Matrix::Identity(3, 3);
		K = J_constraint * MrInverse * J_constraint.transpose();
		_Scalar delta = abs(K.maxCoeff()) * 1e-6;
		effectiveMass0 = (K + delta * mIdentity).inverse();//3x3 matrix
		lambda = effectiveMass0 * (-J_constraint * qdot);
		_Vector qdotCorrection = MrInverse * J_constraint.transpose() * lambda;
		qdot = qdot + qdotCorrection;

		SolvePositionError();
	}
}

void eae6320::MultiBody::SolvePositionError()
{
	_Vector3 closeLoopLinkPos, endFactorPos;
	ComputeCloseLoopAnchorPositions(closeLoopLinkPos, endFactorPos, jointPos[0], R_global);

	_Vector3 error;
	error.setZero();
	_Vector3 positionViolation = closeLoopLinkPos - endFactorPos;
	_Scalar beta = 0.1;
	error = beta * -positionViolation;
	_Matrix lambda;
	lambda = effectiveMass0 * error;
	_Vector qCorrection = MrInverse * J_constraint.transpose() * lambda;
	Integrate_q(q, rel_ori, q, rel_ori, qCorrection, 1.0);
}

void eae6320::MultiBody::ComputeCloseLoopAnchorPositions(_Vector3& o_pos0, _Vector3& o_pos1, _Vector i_rootPos, std::vector<_Matrix3>& i_R)
{
	_Vector3 m_uGlobalChild;
	m_uGlobalChild.setZero();
	_Vector3 m_uGlobalParent;
	m_uGlobalParent.setZero();

	o_pos1 = i_rootPos;
	for (int i = 0; i < numOfLinks; i++)
	{
		int j = parentArr[i];
		m_uGlobalChild = i_R[i] * uLocalsChild[i];
		if (i > 0)
		{
			m_uGlobalParent = i_R[j] * uLocalsParent[i];
		}

		if (i == closeLoopLinkID)
		{
			m_uGlobalChild = i_R[i] * uPre;
		}
		else if (i == closeLoopLinkID + 1)
		{
			m_uGlobalParent = i_R[j] * uNext;
		}
		else if (i == numOfLinks - 1)
		{
			m_uGlobalChild = i_R[i] * uEnd;
		}

		o_pos1 = o_pos1 + m_uGlobalParent - m_uGlobalChild;//for hinge joints, assume that hingeMagnitude[i] * hingeDirGlobals[i] = 0
		if (i == closeLoopLinkID) o_pos0 = o_pos1;//this line only works when there is no branches. If there are branches, TODO is required
	}
}

void eae6320::MultiBody::AddCloseLoop(int i_linkID, _Vector3 i_uPre, _Vector3 i_uNext, _Vector3 i_uEnd)
{
	hasCloseLoop = true;
	
	closeLoopLinkID = i_linkID;
	uPre = i_uPre;
	uNext = i_uNext;
	uEnd = i_uEnd;

	_Vector3 closeLoopLinkPos, endFactorPos;
	ComputeCloseLoopAnchorPositions(closeLoopLinkPos, endFactorPos, jointPos[0], R_global);
	_Scalar initialError = (closeLoopLinkPos - endFactorPos).norm();
	if (initialError > 1e-3)
	{
		EAE6320_ASSERTF(false, "Initial close loop error is too large");
		std::cout << "Initial close loop error is too large" << std::endl;
	}
	//std::cout << "start " << closeLoopLinkPos.transpose() << std::endl;
	//std::cout << "end " << endFactorPos.transpose() << std::endl;
}