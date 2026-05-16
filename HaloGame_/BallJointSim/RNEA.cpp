#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#include "BallJointSim/BallJointSim.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>

void eae6320::MultiBody::ForwardBackwardRecursion(_Vector& o_tau, _Vector& i_q, std::vector<_Quat>& i_quat, _Vector& i_qdot, _Vector& i_qddot, std::vector<_Vector>& i_externalForces)
{
	ForwardKinematics(i_q, i_quat, jointType, posStartIndex);
	
	std::vector<_Vector> gamma;
	gamma.resize(numOfLinks);

	//forward pass
	for (int i = 0; i < numOfLinks; i++)
	{
		int j = parentArr[i];
		_Matrix3 A;
		A.setZero();
		if (jointType[i] == BALL_JOINT)
		{
			//compute H
			H[i].resize(6, 3);
			H[i].block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uLocalsChild[i]);
			H[i].block<3, 3>(3, 0) = _Matrix::Identity(3, 3);
			//compute D
			D[i].setZero();
			if (j != -1)
			{
				A = R_global[i].transpose() * R_global[j];
				D[i].block<3, 3>(0, 0) = A;
				D[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(uLocalsChild[i]) * A - A * Math::ToSkewSymmetricMatrix(uLocalsParent[i]);
				D[i].block<3, 3>(3, 3) = A;
			}
		}
		
		//update child velocity based on parent velocity
		_Vector vParent;
		vParent.resize(6);
		vParent.setZero();
		if (j != -1)
		{
			vParent.segment(0, 3) = COMVelLocal[j];
			vParent.segment(3, 3) = w_abs_local[j];
		}
		_Vector vChild = H[i] * i_qdot.segment(velStartIndex[i], velDOF[i]) + D[i] * vParent;
		COMVelLocal[i] = vChild.segment(0, 3);
		w_abs_local[i] = vChild.segment(3, 3);

		//compute gamma
		gamma[i].resize(6);
		gamma[i].setZero();
		if (jointType[i] == BALL_JOINT)
		{
			_Vector3 rdot = i_qdot.segment(velStartIndex[i], 3);
			_Vector3 gamma_theta;
			gamma_theta.setZero();
			gamma_theta = w_abs_local[i].cross(rdot);

			if (i == 0)
			{
				gamma[i].segment(0, 3) = Math::ToSkewSymmetricMatrix(uLocalsChild[i]) * gamma_theta - w_abs_local[i].cross(w_abs_local[i].cross(uLocalsChild[i]));
			}
			else
			{
				gamma[i].segment(0, 3) = Math::ToSkewSymmetricMatrix(uLocalsChild[i]) * gamma_theta - w_abs_local[i].cross(w_abs_local[i].cross(uLocalsChild[i])) + A * w_abs_local[j].cross(w_abs_local[j].cross(uLocalsParent[i]));
			}
			gamma[i].segment(3, 3) = gamma_theta;
		}
		
		//update child acceleration based on parent acceleration
		_Vector vDotParent;
		vDotParent.resize(6);
		vDotParent.setZero();
		if (j != -1)
		{
			vDotParent.segment(0, 3) = COMVelDotLocal[j];
			vDotParent.segment(3, 3) = wDot_abs_local[j];
		}
		_Vector vDotChild = H[i] * i_qddot.segment(velStartIndex[i], velDOF[i]) + D[i] * vDotParent + gamma[i];
		COMVelDotLocal[i] = vDotChild.segment(0, 3);
		wDot_abs_local[i] = vDotChild.segment(3, 3);
	}

	for (int i = 0; i < numOfLinks; i++)
	{
		f_fromChild[i].setZero();
		n_fromChild[i].setZero();
	}
	
	//backward pass
	for (int i = numOfLinks - 1; i >= 0; i--)
	{
		f[i] = rigidBodyMass * COMVelDotLocal[i] - f_fromChild[i] - R_global[i].transpose() * i_externalForces[i].segment(0, 3);
		n[i] = localInertiaTensors[i] * wDot_abs_local[i] + w_abs_local[i].cross(localInertiaTensors[i] * w_abs_local[i]) - n_fromChild[i] - R_global[i].transpose() * i_externalForces[i].segment(3, 3);
		
		int j = parentArr[i];
		if (j != -1)
		{
			_Vector fnChild;
			fnChild.resize(6);
			fnChild.segment(0, 3) = f[i];
			fnChild.segment(3, 3) = n[i];
			_Vector fnParent = D[i].transpose() * (-fnChild);
			f_fromChild[j] += fnParent.segment(0, 3);
			n_fromChild[j] += fnParent.segment(3, 3);
		}
	}

	//convert joint force into generalized coordinate
	_Vector fn;
	fn.resize(6);
	o_tau.resize(totalVelDOF);
	for (int i = 0; i < numOfLinks; i++)
	{
		fn.segment(0, 3) = f[i];
		fn.segment(3, 3) = n[i];
		o_tau.segment(velStartIndex[i], velDOF[i]) = H[i].transpose() * fn;
	}
}

void eae6320::MultiBody::RNEA(_Vector& o_C, _Vector& o_G, _Matrix& o_M, _Vector& i_q, std::vector<_Quat>& i_quat, _Vector& i_qdot, std::vector<_Vector>& i_externalForces)
{
	_Vector zeroVector;
	zeroVector.resize(totalVelDOF);
	zeroVector.setZero();
	
	ForwardBackwardRecursion(o_C, i_q, i_quat, i_qdot, zeroVector, zeroExternalForces);
	o_C = -o_C;

	ForwardBackwardRecursion(o_G, i_q, i_quat, zeroVector, zeroVector, i_externalForces);
	o_G = -o_G;

	o_M.resize(totalVelDOF, totalVelDOF);
	for (int i = 0; i < totalVelDOF; i++)
	{
		_Vector basisVector;
		basisVector.resize(totalVelDOF);
		basisVector.setZero();
		basisVector(i) = 1;
		_Vector mColmn;
		ForwardBackwardRecursion(mColmn, i_q, i_quat, zeroVector, basisVector, zeroExternalForces);
		o_M.block(0, i, totalVelDOF, 1) = mColmn;
	}
}