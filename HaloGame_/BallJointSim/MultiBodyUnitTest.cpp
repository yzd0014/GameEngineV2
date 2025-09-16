#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#include "Engine/GameCommon/Camera.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>

void eae6320::MultiBody::UnitTest5_6()
{
	constraintSolverMode = IMPULSE;
	gravity = false;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 0

	MultiBodyInitialization();
	_Vector3 rot_vec(-0.25 * M_PI, 0.0, 0.0);
	rel_ori[0] = Math::RotationConversion_VecToQuat(rot_vec);
	Forward();
	_Vector3 local_w = _Vector3(0.0, 0.0, -2.0);
	_Vector3 world_w = R_global[0] * local_w;
	qdot.segment(0, 3) = world_w;
	Forward();
	ConfigureSingleBallJoint(0, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 0.5 * M_PI, 1e-6);
}

void eae6320::MultiBody::UnitTest5_4a()
{
	constraintSolverMode = IMPULSE;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 0

	MultiBodyInitialization();
	_Vector3 rot_vec(-0.25 * M_PI, 0.0, 0);
	rel_ori[0] = Math::RotationConversion_VecToQuat(rot_vec);
	Forward();
	_Vector3 local_w = _Vector3(0.0, -2.0, 0.0);
	_Vector3 world_w = R_global[0] * local_w;
	qdot.segment(0, 3) = world_w;
	Forward();
	ConfigureSingleBallJoint(0, _Vector3(0, -1, 0), _Vector3(1, 0, 0), -1, 0.5 * M_PI);
}

void eae6320::MultiBody::UnitTest5_4b()
{
	constraintSolverMode = IMPULSE;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 0

	MultiBodyInitialization();
	_Vector3 rot_vec(0, 0.0, -0.25 * M_PI);
	rel_ori[0] = Math::RotationConversion_VecToQuat(rot_vec);
	Forward();
	_Vector3 local_w = _Vector3(0.0, -2.0, 0.0);
	_Vector3 world_w = R_global[0] * local_w;
	qdot.segment(0, 3) = world_w;
	Forward();
	ConfigureSingleBallJoint(0, _Vector3(0, -1, 0), _Vector3(1, 0, 0), -1, 0.5 * M_PI);

	m_HoudiniSave = [this](int frames_number)
	{
		LOG_TO_FILE << frames_number << ",";
		for (int i = 0; i < numOfLinks; i++)
		{
			_Vector3 vecRot = Math::RotationConversion_QuatToVec(obs_ori[i]);
			_Scalar rotAngle = vecRot.norm();
			if (abs(rotAngle) < 1e-8)
			{
				vecRot = _Vector3(1, 0, 0);
			}
			else
			{
				vecRot = vecRot / rotAngle;
			}
			LOG_TO_FILE << pos[i](0) << "," << pos[i](1) << "," << pos[i](2) << "," << vecRot(0) << "," << vecRot(1) << "," << vecRot(2) << "," << rotAngle;
			if (i != numOfLinks - 1)
			{
				LOG_TO_FILE << ",";
			}
			else
			{
				LOG_TO_FILE << std::endl;
			}
		}
	};
}

void eae6320::MultiBody::UnitTest5_2()
{
	constraintSolverMode = IMPULSE;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 0
	MultiBodyInitialization();
	_Vector3 local_w = _Vector3(-2.0, 0.0, 2.0);;
	qdot.segment(0, 3) = local_w;
	Forward();

	ConfigureSingleBallJoint(0, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), -1, 1e-6);//head
}

void eae6320::MultiBody::UnitTest5_5()
{
	constraintSolverMode = IMPULSE;
	gravity = false;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 0

	MultiBodyInitialization();
	_Vector3 local_w = _Vector3(-2.0, 2.0, 0.0);
	qdot.segment(0, 3) = local_w;
	Forward();
	ConfigureSingleBallJoint(0, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), -1, 0.25 * M_PI);
}

void eae6320::MultiBody::UnitTest5_7()
{
	constraintSolverMode = IMPULSE;
	gravity = true;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 0
	AddRigidBody(0, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 1
	AddRigidBody(1, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 2
	AddRigidBody(2, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 3
	AddRigidBody(3, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 4

	MultiBodyInitialization();
	//rel_ori[1] = Math::RotationConversion_VecToQuat(_Vector3(0, M_PI / 8, 0));
	for (int i = 0; i < numOfLinks; i++)
	{
		rel_ori[i] = Math::RotationConversion_VecToQuat(_Vector3(0, 0, M_PI / 4));
	}
	Forward();
	int bodyNum = 4;
	_Vector3 local_w = _Vector3(0.0, 10.0, 0.0);
	_Vector3 world_w = R_local[bodyNum] * local_w;
	qdot.segment(velStartIndex[bodyNum], 3) = world_w;
	Forward();
	for (int i = 0; i < 5; i++)
	{
		ConfigureSingleBallJoint(i, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 0.25 * M_PI, 0.1);
	}
}

void eae6320::MultiBody::UnitTest5_8a()
{
	constraintSolverMode = IMPULSE;
	gravity = true;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, BALL_JOINT_4D, _Vector3(0.0f, 0.7f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(0.7, 0.7, 0.7), localInertiaTensor);//body 0 head
	AddRigidBody(0, BALL_JOINT_4D, _Vector3(0.0f, 1.1f, 0.0f), _Vector3(0.0f, -0.8f, 0.0f), masterMeshArray[3], Vector3d(1, 1, 0.25), localInertiaTensor);//body 1 chest0
	AddRigidBody(1, BALL_JOINT_4D, _Vector3(0.0f, 0.6f, 0.0f), _Vector3(0.0f, -1.1f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.25), localInertiaTensor);//body 2 chest1
	AddRigidBody(2, BALL_JOINT_4D, _Vector3(0.0f, 1.15f, 0.0f), _Vector3(-0.52f, -0.65f, 0.0f), masterMeshArray[3], Vector3d(0.3, 1, 0.3), localInertiaTensor);//body 3 left_leg0
	AddRigidBody(3, BALL_JOINT_4D, _Vector3(0.0f, 1.15f, 0.0f), _Vector3(0, -1.15f, 0.0f), masterMeshArray[3], Vector3d(0.3, 1, 0.3), localInertiaTensor);//body 4 left_leg1
	AddRigidBody(4, BALL_JOINT_4D, _Vector3(0.0f, 0.2f, -0.5f), _Vector3(0, -1.15f, 0.0f), masterMeshArray[3], Vector3d(0.3, 0.1, 0.5), localInertiaTensor);//body 5 left_foot
	AddRigidBody(2, BALL_JOINT_4D, _Vector3(0.0f, 1.15f, 0.0f), _Vector3(0.52f, -0.65f, 0.0f), masterMeshArray[3], Vector3d(0.3, 1, 0.3), localInertiaTensor);//body 6 right_leg0
	AddRigidBody(6, BALL_JOINT_4D, _Vector3(0.0f, 1.15f, 0.0f), _Vector3(0, -1.15f, 0.0f), masterMeshArray[3], Vector3d(0.3, 1, 0.3), localInertiaTensor);//body 7 right_leg1
	AddRigidBody(7, BALL_JOINT_4D, _Vector3(0.0f, 0.2f, -0.5f), _Vector3(0, -1.15f, 0.0f), masterMeshArray[3], Vector3d(0.3, 0.1, 0.5), localInertiaTensor);//body 8 right_foot
	AddRigidBody(1, BALL_JOINT_4D, _Vector3(0.95f, 0, 0), _Vector3(-1.3f, 1, 0.0f), masterMeshArray[3], Vector3d(0.8, 0.2, 0.2), localInertiaTensor);//body 9 left_arm0
	AddRigidBody(9, BALL_JOINT_4D, _Vector3(0.95f, 0, 0), _Vector3(-0.95f, 0, 0.0f), masterMeshArray[3], Vector3d(0.8, 0.2, 0.2), localInertiaTensor);//body 10 left_arm1
	AddRigidBody(10, BALL_JOINT_4D, _Vector3(0.35f, 0, 0), _Vector3(-0.9f, 0, 0.0f), masterMeshArray[3], Vector3d(0.25, 0.15, 0.25), localInertiaTensor);//body 11 left_hand
	AddRigidBody(1, BALL_JOINT_4D, _Vector3(-0.95f, 0, 0), _Vector3(1.3f, 1, 0.0f), masterMeshArray[3], Vector3d(0.8, 0.2, 0.2), localInertiaTensor);//body 12 right_arm0
	AddRigidBody(12, BALL_JOINT_4D, _Vector3(-0.95f, 0, 0), _Vector3(0.95f, 0, 0.0f), masterMeshArray[3], Vector3d(0.8, 0.2, 0.2), localInertiaTensor);//body 13 right_arm1
	AddRigidBody(13, BALL_JOINT_4D, _Vector3(-0.35f, 0, 0), _Vector3(0.9f, 0, 0.0f), masterMeshArray[3], Vector3d(0.25, 0.15, 0.25), localInertiaTensor);//body 14 right_hand

	MultiBodyInitialization();
	Forward();

	ConfigureSingleBallJoint(0, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 0.25 * M_PI, 1e-3);//head
	ConfigureSingleBallJoint(1, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 1e-3, 1e-3);//chest0
	ConfigureSingleBallJoint(2, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 1e-3, 0.5);//chest1
	ConfigureSingleBallJoint(3, _Vector3(0, -1, 0), _Vector3(0, 0, 1), 0.5 * M_PI, 1e-3); //left_leg0
	ConfigureSingleBallJoint(4, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 1e-3, 1e-3); //left_leg1
	ConfigureSingleBallJoint(5, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 1e-3, 0.25 * M_PI); //left_foot
	ConfigureSingleBallJoint(6, _Vector3(0, -1, 0), _Vector3(0, 0, 1), 0.5 * M_PI, 1e-3); //right_leg0
	ConfigureSingleBallJoint(7, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 1e-3, 1e-3); //right_leg1
	ConfigureSingleBallJoint(8, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 1e-3, 0.25 * M_PI); //right_foot
	ConfigureSingleBallJoint(9, _Vector3(-1, 0, 0), _Vector3(0, 1, 0), 0.5 * M_PI, 0.1); //left_arm0
	ConfigureSingleBallJoint(10, _Vector3(0, 1, 0), _Vector3(0, 0, 1), 1e-3, 1e-3); //left_arm1
	ConfigureSingleBallJoint(11, _Vector3(0, 0, 1), _Vector3(0, 1, 0), 1e-3, 0.5 * M_PI); //left_hand
	ConfigureSingleBallJoint(12, _Vector3(1, 0, 0), _Vector3(0, 1, 0), 0.5 * M_PI, 0.1); //right_arm0
	ConfigureSingleBallJoint(13, _Vector3(0, 1, 0), _Vector3(0, 0, 1), 1e-3, 1e-3); //right_arm1
	ConfigureSingleBallJoint(14, _Vector3(0, 0, 1), _Vector3(0, 1, 0), 1e-3, 0.5 * M_PI); //right_hand

	m_HoudiniSave = [this](int frames_number)
	{
		std::vector<std::string> bodyNames;
		bodyNames.resize(15);
		bodyNames[0] = "head";
		bodyNames[1] = "chest0";
		bodyNames[2] = "chest1";
		bodyNames[3] = "left_leg0";
		bodyNames[4] = "left_leg1";
		bodyNames[5] = "left_foot";
		bodyNames[6] = "right_leg0";
		bodyNames[7] = "right_leg1";
		bodyNames[8] = "right_foot";
		bodyNames[9] = "left_arm0";
		bodyNames[10] = "left_arm1";
		bodyNames[11] = "left_hand";
		bodyNames[12] = "right_arm0";
		bodyNames[13] = "right_arm1";
		bodyNames[14] = "right_hand";

		LOG_TO_FILE << frames_number << ",";
		for (int i = 0; i < numOfLinks; i++)
		{
			_Vector3 vecRot = Math::RotationConversion_QuatToVec(obs_ori[i]);
			_Scalar rotAngle = vecRot.norm();
			if (abs(rotAngle) < 1e-8)
			{
				vecRot = _Vector3(1, 0, 0);
			}
			else
			{
				vecRot = vecRot / rotAngle;
			}
			LOG_TO_FILE << bodyNames[i] << "," << pos[i](0) << "," << pos[i](1) << "," << pos[i](2) << "," << vecRot(0) << "," << vecRot(1) << "," << vecRot(2) << "," << rotAngle;
			if (i != numOfLinks - 1)
			{
				LOG_TO_FILE << ",";
			}
			else
			{
				LOG_TO_FILE << std::endl;
			}
		}
	};
}

void eae6320::MultiBody::UnitTest5_8b()
{
	constraintSolverMode = IMPULSE;
	gravity = true;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, BALL_JOINT_4D, _Vector3(0.0f, 0.7f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(0.7, 0.7, 0.7), localInertiaTensor);//body 0 head
	AddRigidBody(0, BALL_JOINT_4D, _Vector3(0.0f, 1.1f, 0.0f), _Vector3(0.0f, -0.8f, 0.0f), masterMeshArray[3], Vector3d(1, 1, 0.25), localInertiaTensor);//body 1 chest0
	AddRigidBody(1, BALL_JOINT_4D, _Vector3(0.0f, 0.6f, 0.0f), _Vector3(0.0f, -1.1f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.25), localInertiaTensor);//body 2 chest1
	AddRigidBody(2, BALL_JOINT_4D, _Vector3(0.0f, 1.15f, 0.0f), _Vector3(-0.52f, -0.65f, 0.0f), masterMeshArray[3], Vector3d(0.3, 1, 0.3), localInertiaTensor);//body 3 left_leg0
	AddRigidBody(3, BALL_JOINT_4D, _Vector3(0.0f, 1.15f, 0.0f), _Vector3(0, -1.15f, 0.0f), masterMeshArray[3], Vector3d(0.3, 1, 0.3), localInertiaTensor);//body 4 left_leg1
	AddRigidBody(4, BALL_JOINT_4D, _Vector3(0.0f, 0.2f, -0.5f), _Vector3(0, -1.15f, 0.0f), masterMeshArray[3], Vector3d(0.3, 0.1, 0.5), localInertiaTensor);//body 5 left_foot
	AddRigidBody(2, BALL_JOINT_4D, _Vector3(0.0f, 1.15f, 0.0f), _Vector3(0.52f, -0.65f, 0.0f), masterMeshArray[3], Vector3d(0.3, 1, 0.3), localInertiaTensor);//body 6 right_leg0
	AddRigidBody(6, BALL_JOINT_4D, _Vector3(0.0f, 1.15f, 0.0f), _Vector3(0, -1.15f, 0.0f), masterMeshArray[3], Vector3d(0.3, 1, 0.3), localInertiaTensor);//body 7 right_leg1
	AddRigidBody(7, BALL_JOINT_4D, _Vector3(0.0f, 0.2f, -0.5f), _Vector3(0, -1.15f, 0.0f), masterMeshArray[3], Vector3d(0.3, 0.1, 0.5), localInertiaTensor);//body 8 right_foot
	AddRigidBody(1, BALL_JOINT_4D, _Vector3(0.95f, 0, 0), _Vector3(-1.3f, 1, 0.0f), masterMeshArray[3], Vector3d(0.8, 0.2, 0.2), localInertiaTensor);//body 9 left_arm0
	AddRigidBody(9, BALL_JOINT_4D, _Vector3(0.95f, 0, 0), _Vector3(-0.95f, 0, 0.0f), masterMeshArray[3], Vector3d(0.8, 0.2, 0.2), localInertiaTensor);//body 10 left_arm1
	AddRigidBody(10, BALL_JOINT_4D, _Vector3(0.35f, 0, 0), _Vector3(-0.9f, 0, 0.0f), masterMeshArray[3], Vector3d(0.25, 0.15, 0.25), localInertiaTensor);//body 11 left_hand
	AddRigidBody(1, BALL_JOINT_4D, _Vector3(-0.95f, 0, 0), _Vector3(1.3f, 1, 0.0f), masterMeshArray[3], Vector3d(0.8, 0.2, 0.2), localInertiaTensor);//body 12 right_arm0
	AddRigidBody(12, BALL_JOINT_4D, _Vector3(-0.95f, 0, 0), _Vector3(0.95f, 0, 0.0f), masterMeshArray[3], Vector3d(0.8, 0.2, 0.2), localInertiaTensor);//body 13 right_arm1
	AddRigidBody(13, BALL_JOINT_4D, _Vector3(-0.35f, 0, 0), _Vector3(0.9f, 0, 0.0f), masterMeshArray[3], Vector3d(0.25, 0.15, 0.25), localInertiaTensor);//body 14 right_hand

	MultiBodyInitialization();
	Forward();

	ConfigureSingleBallJoint(0, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 0.25 * M_PI, 1e-3);//head
	ConfigureSingleBallJoint(1, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 1e-3, 1e-3);//chest0
	ConfigureSingleBallJoint(2, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 1e-3, 0.5);//chest1
	ConfigureSingleBallJoint(3, _Vector3(0, -1, 0), _Vector3(0, 0, 1), 0.5 * M_PI, 1e-3); //left_leg0
	ConfigureSingleBallJoint(4, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 1e-3, 1e-3); //left_leg1
	ConfigureSingleBallJoint(5, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 1e-3, 0.25 * M_PI); //left_foot
	ConfigureSingleBallJoint(6, _Vector3(0, -1, 0), _Vector3(0, 0, 1), 0.5 * M_PI, 1e-3); //right_leg0
	ConfigureSingleBallJoint(7, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 1e-3, 1e-3); //right_leg1
	ConfigureSingleBallJoint(8, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 1e-3, 0.25 * M_PI); //right_foot
	ConfigureSingleBallJoint(9, _Vector3(-1, 0, 0), _Vector3(0, 1, 0), 0.5 * M_PI, 0.1); //left_arm0
	ConfigureSingleBallJoint(10, _Vector3(0, 1, 0), _Vector3(0, 0, 1), 1e-3, 1e-3); //left_arm1
	ConfigureSingleBallJoint(11, _Vector3(0, 0, 1), _Vector3(0, 1, 0), 1e-3, 0.5 * M_PI); //left_hand
	ConfigureSingleBallJoint(12, _Vector3(1, 0, 0), _Vector3(0, 1, 0), 0.5 * M_PI, 0.1); //right_arm0
	ConfigureSingleBallJoint(13, _Vector3(0, 1, 0), _Vector3(0, 0, 1), 1e-3, 1e-3); //right_arm1
	ConfigureSingleBallJoint(14, _Vector3(0, 0, 1), _Vector3(0, 1, 0), 1e-3, 0.5 * M_PI); //right_hand

	m_HoudiniSave = [this](int frames_number)
	{
		std::vector<std::string> bodyNames;
		bodyNames.resize(15);
		bodyNames[0] = "head";
		bodyNames[1] = "chest0";
		bodyNames[2] = "chest1";
		bodyNames[3] = "left_leg0";
		bodyNames[4] = "left_leg1";
		bodyNames[5] = "left_foot";
		bodyNames[6] = "right_leg0";
		bodyNames[7] = "right_leg1";
		bodyNames[8] = "right_foot";
		bodyNames[9] = "left_arm0";
		bodyNames[10] = "left_arm1";
		bodyNames[11] = "left_hand";
		bodyNames[12] = "right_arm0";
		bodyNames[13] = "right_arm1";
		bodyNames[14] = "right_hand";

		LOG_TO_FILE << frames_number << ",";
		for (int i = 0; i < numOfLinks; i++)
		{
			_Vector3 vecRot = Math::RotationConversion_QuatToVec(obs_ori[i]);
			_Scalar rotAngle = vecRot.norm();
			if (abs(rotAngle) < 1e-8)
			{
				vecRot = _Vector3(1, 0, 0);
			}
			else
			{
				vecRot = vecRot / rotAngle;
			}
			LOG_TO_FILE << bodyNames[i] << "," << pos[i](0) << "," << pos[i](1) << "," << pos[i](2) << "," << vecRot(0) << "," << vecRot(1) << "," << vecRot(2) << "," << rotAngle;
			if (i != numOfLinks - 1)
			{
				LOG_TO_FILE << ",";
			}
			else
			{
				LOG_TO_FILE << std::endl;
			}
		}
	};
	m_control = [this]()
	{
		externalForces[0].block<3, 1>(3, 0) = _Vector3(-500, 0, 0);
	};
}

void eae6320::MultiBody::UnitTest5_3a()
{
	constraintSolverMode = IMPULSE;
	gravity = false;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 0

	MultiBodyInitialization();
	_Vector3 rot_vec(-0.4 * M_PI, 0.0, 0.0);
	rel_ori[0] = Math::RotationConversion_VecToQuat(rot_vec);
	Forward();
	ConfigureSingleBallJoint(0, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), -1, 1e-4);

	m_control = [this]()
	{
		_Scalar r = 0.6;
		_Vector3 target;
		target(2) = 1.9;
		target(0) = r * sin(Physics::totalSimulationTime * 0.1);
		target(1) = -r * cos(Physics::totalSimulationTime * 0.1);
		if (xArrow != nullptr)
		{
			xArrow->DestroyGameObject();
			xArrow = nullptr;
		}
		_Vector3 endPoint(0, 0, 1.9);
		xArrow = GameplayUtility::DrawArrowScaled(endPoint, target - endPoint, Math::sVector(0, 0, 1), Vector3d(0.5, 0.5, 0.5));

		_Vector3 endFactor(0, -2, 0);
		endFactor = R_local[0] * endFactor;
		_Vector3 tau;
		_Scalar k = 200;
		tau = k * (target - endFactor);
		externalForces[0].block<3, 1>(0, 0) = tau;
	};

	m_MatlabSave = [this]()
	{
		_Vector3 z(0, 0, -1);
		static _Vector3 z0;
		_Scalar t = (_Scalar)eae6320::Physics::totalSimulationTime;
		if (t <= 1e-8)
		{
			z0 = R_local[0] * z;
		}
		_Vector3 zt = R_local[0] * z;
		_Scalar out = z0.dot(zt);
		LOG_TO_FILE << t << " " << out << std::endl;
	};
}

void eae6320::MultiBody::UnitTest5_3b()
{
	constraintSolverMode = IMPULSE;
	gravity = false;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 0

	MultiBodyInitialization();
	_Vector3 rot_vec(-0.9 * M_PI, 0.0, 0.0);
	rel_ori[0] = Math::RotationConversion_VecToQuat(rot_vec);
	Forward();
	ConfigureSingleBallJoint(0, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), -1, 0.5);

	m_control = [this]()
	{
		_Scalar r = 0.6;
		_Vector3 target;
		target(1) = 1.9;
		target(0) = r * sin(Physics::totalSimulationTime * 0.1);
		target(2) = r * cos(Physics::totalSimulationTime * 0.1);
		if (xArrow != nullptr)
		{
			xArrow->DestroyGameObject();
			xArrow = nullptr;
		}
		_Vector3 endPoint(0, 1.9, 0);
		xArrow = GameplayUtility::DrawArrowScaled(endPoint, target - endPoint, Math::sVector(0, 0, 1), Vector3d(0.5, 0.5, 0.5));

		_Vector3 endFactor(0, -2, 0);
		endFactor = R_local[0] * endFactor;
		_Vector3 tau;
		_Scalar k = 200;
		tau = k * (target - endFactor);
		externalForces[0].block<3, 1>(0, 0) = tau;
	};
	m_MatlabSave = [this]()
	{
		_Vector3 z(0, 0, -1);
		static _Vector3 z0;
		_Scalar t = (_Scalar)eae6320::Physics::totalSimulationTime;
		if (t <= 1e-8)
		{
			z0 = R_local[0] * z;
		}
		_Vector3 zt = R_local[0] * z;
		_Scalar out = z0.dot(zt);
		LOG_TO_FILE << t << " " << out << std::endl;
	};
}

void eae6320::MultiBody::UnitTest5_1()
{
	constraintSolverMode = IMPULSE;
	gravity = false;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 0

	MultiBodyInitialization();
	qdot.segment(0, 3) = _Vector3(-2, -2, 0);
	Forward();
	ConfigureSingleBallJoint(0, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 3.089, 1.5708);
}

void eae6320::MultiBody::HingeJointTest()
{
	constraintSolverMode = IMPULSE;
	gravity = true;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, HINGE_JOINT, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 0
	SetHingeJoint(0, _Vector3(0, 0, 1), 0);
	AddRigidBody(0, HINGE_JOINT, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 1
	SetHingeJoint(1, _Vector3(0, 0, 1), 0);
	//AddRigidBody(1, HINGE_JOINT, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 2
	//SetHingeJoint(2, _Vector3(0, 0, 1), 0);
	//AddRigidBody(2, HINGE_JOINT, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 3
	//SetHingeJoint(3, _Vector3(0, 0, 1), 0);
	//AddRigidBody(3, HINGE_JOINT, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 4
	//SetHingeJoint(4, _Vector3(0, 0, 1), 0);
	//AddRigidBody(1, HINGE_JOINT, _Vector3(1.0f, 0.0f, 0.0f), _Vector3(-0.5f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor);//body 5
	//SetHingeJoint(5, _Vector3(0, 0, 1), 0);
	//AddRigidBody(5, HINGE_JOINT, _Vector3(1.0f, 0.0f, 0.0f), _Vector3(-1.0f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor);//body 6
	//SetHingeJoint(6, _Vector3(0, 0, 1), 0);

	MultiBodyInitialization();
	q(0) = M_PI * 0.5;
	Forward();

	m_keyPressSave = [this](FILE * i_pFile)
	{
		int qDof = static_cast<int>(q.size());
		Populate_q(rel_ori, q);
		for (int i = 0; i < qDof; i++)
		{
			fwrite(&q(i), sizeof(double), 1, i_pFile);
		}
		for (int i = 0; i < numOfLinks; i++)
		{
			fwrite(&rel_ori[i].w(), sizeof(double), 1, i_pFile);
			fwrite(&rel_ori[i].x(), sizeof(double), 1, i_pFile);
			fwrite(&rel_ori[i].y(), sizeof(double), 1, i_pFile);
			fwrite(&rel_ori[i].z(), sizeof(double), 1, i_pFile);
		}
		int vDof = static_cast<int>(qdot.size());
		for (int i = 0; i < vDof; i++)
		{
			fwrite(&qdot(i), sizeof(double), 1, i_pFile);
		}
	};
}

void eae6320::MultiBody::BallJointTest()
{
	constraintSolverMode = IMPULSE;
	gravity = true;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, BALL_JOINT_4D, _Vector3(-1.0f, 0.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor);//body 0
	AddRigidBody(0, BALL_JOINT_4D, _Vector3(-1.0f, 0.0f, 0.0f), _Vector3(1.0f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor);//body 1
	AddRigidBody(1, BALL_JOINT_4D, _Vector3(-1.0f, 0.0f, 0.0f), _Vector3(1.0f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor);//body 2
	AddRigidBody(2, BALL_JOINT_4D, _Vector3(-1.0f, 0.0f, 0.0f), _Vector3(1.0f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor);//body 3
	AddRigidBody(3, BALL_JOINT_4D, _Vector3(-1.0f, 0.0f, 0.0f), _Vector3(1.0f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor);//body 4

	MultiBodyInitialization();
	rel_ori[1] = Math::RotationConversion_VecToQuat(_Vector3(0, M_PI / 8, 0));
	Forward();

	m_keyPressSave = [this](FILE * i_pFile)
	{
		int qDof = static_cast<int>(q.size());
		for (int i = 0; i < qDof; i++)
		{
			fwrite(&q(i), sizeof(double), 1, i_pFile);
		}
		for (int i = 0; i < numOfLinks; i++)
		{
			fwrite(&rel_ori[i].w(), sizeof(double), 1, i_pFile);
			fwrite(&rel_ori[i].x(), sizeof(double), 1, i_pFile);
			fwrite(&rel_ori[i].y(), sizeof(double), 1, i_pFile);
			fwrite(&rel_ori[i].z(), sizeof(double), 1, i_pFile);
		}
		int vDof = static_cast<int>(qdot.size());
		for (int i = 0; i < vDof; i++)
		{
			fwrite(&qdot(i), sizeof(double), 1, i_pFile);
		}

		std::vector<int> jointTypeCopy(jointType);//save original joint type
		std::vector<int> posStartIndexCopy(posStartIndex);//save original start index
		CopyFromQ2X();
		jointType = xJointType;
		posStartIndex = xStartIndex;
		UpdateXDot(xdot, x, qdot);
		int xDof = static_cast<int>(x.size());
		for (int i = 0; i < xDof; i++)
		{
			LOG_TO_FILE << x(i);
			if (i == xDof - 1)
			{
				LOG_TO_FILE << endl;
			}
			else
			{
				LOG_TO_FILE << " ";
			}
		}
		for (int i = 0; i < xDof; i++)
		{
			LOG_TO_FILE << xdot(i);
			if (i == xDof - 1)
			{
				LOG_TO_FILE << endl;
			}
			else
			{
				LOG_TO_FILE << " ";
			}
		}
		jointType = jointTypeCopy;
		posStartIndex = posStartIndexCopy;
	};
}


void eae6320::MultiBody::UnitTest0()
{
	constraintSolverMode = IMPULSE;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 0

	MultiBodyInitialization();
	_Vector3 rot_vec(-0.5 * M_PI, 0.0, 0);
	rel_ori[0] = Math::RotationConversion_VecToQuat(rot_vec);
	Forward();
	_Vector3 world_w = _Vector3(0.0, -2.0, 0.0);
	qdot.segment(0, 3) = world_w;
	Forward();
	ConfigureSingleBallJoint(0, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 0.5 * M_PI, 1e-6);

	m_HoudiniSave = [this](int frames_number)
	{
		LOG_TO_FILE << frames_number << ",";
		for (int i = 0; i < numOfLinks; i++)
		{
			_Vector3 vecRot = Math::RotationConversion_QuatToVec(obs_ori[i]);
			_Scalar rotAngle = vecRot.norm();
			if (abs(rotAngle) < 1e-8)
			{
				vecRot = _Vector3(1, 0, 0);
			}
			else
			{
				vecRot = vecRot / rotAngle;
			}
			LOG_TO_FILE << pos[i](0) << "," << pos[i](1) << "," << pos[i](2) << "," << vecRot(0) << "," << vecRot(1) << "," << vecRot(2) << "," << rotAngle;
			if (i != numOfLinks - 1)
			{
				LOG_TO_FILE << ",";
			}
			else
			{
				LOG_TO_FILE << std::endl;
			}
		}
	};
}

void eae6320::MultiBody::RunUnitTest()
{
	Application::AddApplicationParameter(&damping, Application::ApplicationParameterType::float_point, L"-damping");
	Application::AddApplicationParameter(&twistMode, Application::ApplicationParameterType::integer, L"-tm");
	if (twistMode == EULER_V2 || twistMode == EULER)
	{
		std::cout << "Euler twist constraint is being used" << std::endl;
	}
	else if (twistMode == INCREMENT)
	{
		std::cout << "increment twist constraint is being used" << std::endl;
	}
	else if (twistMode == DIRECT)
	{
		std::cout << "direct swing-twist constraint is being used" << std::endl;
	}

	Application::AddApplicationParameter(&enablePositionSolve, Application::ApplicationParameterType::integer, L"-ps");
	if (enablePositionSolve == 1)
	{
		std::cout << "position solve enabled (Baumgarte)" << std::endl;
	}
	else if (enablePositionSolve == 2)
	{
		std::cout << "position solve enabled (PBD)" << std::endl;
	}
	else
	{
		std::cout << "position solve disabled" << std::endl;
	}

	int testCaseNum = 0;
	Application::AddApplicationParameter(&testCaseNum, Application::ApplicationParameterType::integer, L"-example");
	if (testCaseNum == 1)
	{
		UnitTest5_1();
		std::cout << "compare with Unreal and Unity" << std::endl;
	}
	else if (testCaseNum == 2)
	{
		UnitTest5_2();
		std::cout << "0 twist constraint" << std::endl;
	}
	else if (testCaseNum == 3)
	{
		UnitTest5_3a();
		damping = 0.99;
		std::cout << "zero twist vector field pattern test (side)" << std::endl;
	}
	else if (testCaseNum == 4)
	{
		UnitTest5_3b();
		damping = 0.99;
		if (twistMode == EULER_V2) vectorFieldNum[0] = 1;
		std::cout << "zero twist vector field pattern test (Direct)" << std::endl;
	}
	else if (testCaseNum == 5)
	{
		UnitTest5_4a();
		//twistMode = EULER_V2;
		std::cout << "basic intial conditions to verify Euler twist constraint (induced swing)" << std::endl;
	}
	else if (testCaseNum == 6)
	{
		UnitTest5_4b();
		//twistMode = EULER_V2;
		std::cout << "basic intial conditions to verify Euler twist constraint (no induced swing)" << std::endl;
	}
	else if (testCaseNum == 7)
	{
		UnitTest5_5();
		twistMode = EULER_V2;
		std::cout << "singularity pass through test" << std::endl;
	}
	else if (testCaseNum == 8)
	{
		UnitTest5_6();
		twistMode = EULER_V2;
		std::cout << "regularization test" << std::endl;
	}
	else if (testCaseNum == 9)
	{
		UnitTest5_7();
		std::cout << "5 body for Euler twist" << std::endl;
	}
	else if (testCaseNum == 10)
	{
		UnitTest5_8a();
		twistMode = EULER_V2;
		std::cout << "ragdoll test" << std::endl;
	}
	else if (testCaseNum == 11)
	{
		UnitTest5_8b();
		twistMode = EULER_V2;
		std::cout << "ragdoll test2" << std::endl;
	}
	else if (testCaseNum == 12)
	{
		UnitTest0();
		twistMode = DIRECT;
		std::cout << "limitation of position based twist constraint" << std::endl;
	}
	else if (testCaseNum == 13)
	{
		HingeJointTest();
		std::cout << "5 hinge joints" << std::endl;
	}
	else if (testCaseNum == 14)
	{
		BallJointTest();
		std::cout << "5 ball joints" << std::endl;
	}
	else if (testCaseNum == 15)
	{
		AnalyticalVsFD();
		std::cout << "AnalyticalVsFD" << std::endl;
	}

	std::cout << std::endl;
}
