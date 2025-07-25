#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>

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
	obs_ori.resize(numOfLinks);
	rel_ori.resize(numOfLinks);
	R_global.resize(numOfLinks);
	R_local.resize(numOfLinks);
	J_rotation.resize(numOfLinks);
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
	old_R_local.resize(numOfLinks);
	for (int i = 0; i < numOfLinks; i++)
	{
		w_abs_world[i].setZero();
		w_rel_world[i].setZero();
		w_rel_local[i].setZero();
		vel[i].setZero();
		jointPos[i].setZero();
		pos[i].setZero();
		obs_ori[i].setIdentity();
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
		old_R_local[i].setIdentity();
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
