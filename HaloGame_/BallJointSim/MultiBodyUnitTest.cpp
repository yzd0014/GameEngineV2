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

//test the gradient of twist constraint
void eae6320::MultiBody::UnitTest0()
{
	_Vector3 q_old(-0.613267, -1.48335, 0.614194);
	q.segment(0, 3) = q_old;
	Forward();
	jointLimit[0] = 0.5 * M_PI;
	TwistLimitCheck();
	_Scalar c_old = g[0];
	
	_Scalar dq = 0.0000001;

	_Vector3 gradC;
	q(0) = q(0) + dq;
	Forward();
	TwistLimitCheck();
	gradC(0) = (g[0] - c_old) / dq;

	q.segment(0, 3) = q_old;
	q(1) = q(1) + dq;
	Forward();
	TwistLimitCheck();
	gradC(1) = (g[0] - c_old) / dq;

	q.segment(0, 3) = q_old;
	q(2) = q(2) + dq;
	Forward();
	TwistLimitCheck();
	gradC(2) = (g[0] - c_old) / dq;

	std::cout << gradC.transpose() << std::endl;

	q.segment(0, 3) = q_old;
	_Vector3 p = _Vector3(0, -10, 0);
	_Vector3 T0;
	_Vector3 RP = R_local[0] * p;
	T0 = -J_rotation[0].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * R_local[0] * Math::ToSkewSymmetricMatrix(p) * RP;
	_Vector3 T1;
	_Vector3 RPRP = R_local[0] * Math::ToSkewSymmetricMatrix(p) * RP;
	T1 = J_rotation[0].transpose() * Math::ToSkewSymmetricMatrix(RPRP) * Math::ToSkewSymmetricMatrix(p) * RP;
	_Vector3 T2;
	T2 = -J_rotation[0].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * R_local[0].transpose() * Math::ToSkewSymmetricMatrix(p) * RP;
	_Vector3 T3;
	//T3 = 2.0 * cos(jointLimit[i]) * J_rotation[i].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * Math::ToSkewSymmetricMatrix(p) * RP;
	T3 = -2.0 * J_rotation[0].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * Math::ToSkewSymmetricMatrix(p) * RP;

	_Vector s = p.cross(RP);
	_Scalar SRS = s.dot(R_local[0] * s);

	gradC.setZero();
	gradC = (T0 + T1 + T2) / s.squaredNorm() - SRS / (s.squaredNorm() * s.squaredNorm()) * T3;
	std::cout << gradC.transpose() << std::endl;
}

//test swing twist decomposition
void eae6320::MultiBody::UnitTest1()
{
	_Vector3 p(0, -1, 0);
	_Vector3 s(1, 0, 1);
	_Matrix3 R_s = Math::RotationConversion_VecToMatrix(s);
	_Vector3 rotated_p = R_s * p;
	_Matrix3 R_t = Math::RotationConversion_VecToMatrix(rotated_p);
	_Matrix3 R = R_t * R_s;
	
	_Matrix3 R_t_bar;
	_Matrix3 R_s_bar;
	Math::TwistSwingDecomposition(R, p, R_t_bar, R_s_bar);
	
	std::cout << s.transpose() << std::endl;
	std::cout << Math::RotationConversion_MatrixToVec(R_s_bar).transpose() << std::endl << std::endl;
	std::cout << rotated_p.transpose() << std::endl;
	std::cout << Math::RotationConversion_MatrixToVec(R_t_bar).transpose() << std::endl << std::endl;
}

void eae6320::MultiBody::UnitTest2()
{
	numOfLinks = 1;
	constraintSolverMode = IMPULSE;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8; 
	int scale = 1;
	InitializeBodies(masterMeshArray[4], Vector3d(scale, scale, scale), localInertiaTensor, _Vector3(0.0f, scale, 0.0f), _Vector3(0.0f, -scale, 0.0f));//4 is capsule, 3 is cube

	int jointTypeArray[] = { BALL_JOINT_4D };
	InitializeJoints(jointTypeArray);

	////SetZeroInitialCondition();
	int jointID = 0;
	//qdot.segment(velStartIndex[jointID], 3) = _Vector3(-2.0, 2.0, 0.0);
	qdot.segment(velStartIndex[jointID], 3) = _Vector3(2.0, 2.0, 0.0);
	
	Forward();

	jointRange[0].first = 0.25 * M_PI;//swing
	//jointRange[0].second = 0.5 * M_PI;//twist
	//jointLimit[0] = 0.5 * M_PI;
}

void eae6320::MultiBody::UnitTest5()
{
	numOfLinks = 1;
	constraintSolverMode = IMPULSE;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
	InitializeBodies(masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f));//4 is capsule, 3 is cube

	int jointTypeArray[] = { BALL_JOINT_4D };
	InitializeJoints(jointTypeArray);

	////SetZeroInitialCondition();

	_Vector3 rot_vec(-0.25 * M_PI, 0.0, 0.0);
	rel_ori[0] = Math::RotationConversion_VecToQuat(rot_vec);
	Forward();
	_Vector3 local_w = _Vector3(0.0, 0.0, -2.0);
	_Vector3 world_w = R_global[0] * local_w;
	qdot.segment(0, 3) = world_w;
	Forward();
	
	jointRange[0].first = 0.5 * M_PI;//swing
	jointRange[0].second = 1e-6;//twist
	//jointRange[0].second = 0.5 * M_PI;//twist
}

void eae6320::MultiBody::UnitTest19()
{
	numOfLinks = 1;
	constraintSolverMode = IMPULSE;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
	InitializeBodies(masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f));//4 is capsule, 3 is cube

	int jointTypeArray[] = { BALL_JOINT_4D };
	InitializeJoints(jointTypeArray);

	//SetZeroInitialCondition();

	_Vector3 rot_vec(-0.25 * M_PI, 0.0, 0.0);
	rel_ori[0] = Math::RotationConversion_VecToQuat(rot_vec);
	Forward();
	_Vector3 local_w = _Vector3(0.0, 0.0, -2.0);
	_Vector3 world_w = R_global[0] * local_w;
	qdot.segment(0, 3) = world_w;
	Forward();

	jointRange[0].first = 0.48 * M_PI;//swing
	jointRange[0].second = 1e-6;//twist
}

void eae6320::MultiBody::UnitTest20()
{
	numOfLinks = 1;
	constraintSolverMode = IMPULSE;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
	InitializeBodies(masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f));//4 is capsule, 3 is cube

	int jointTypeArray[] = { BALL_JOINT_4D };
	InitializeJoints(jointTypeArray);

	//SetZeroInitialCondition();

	_Vector3 rot_vec(-0.25 * M_PI, 0.0, 0.0);
	rel_ori[0] = Math::RotationConversion_VecToQuat(rot_vec);
	Forward();
	_Vector3 local_w = _Vector3(0.0, 0.0, -2.0);
	_Vector3 world_w = R_global[0] * local_w;
	qdot.segment(0, 3) = world_w;
	Forward();

	jointRange[0].first = 0.98 * M_PI;//swing
	jointRange[0].second = 1e-6;//twist
}

void eae6320::MultiBody::UnitTest3()
{
	constraintSolverMode = IMPULSE;
	gravity = true;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, BALL_JOINT_4D, _Vector3(-1.0f, 1.0f, 1.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(1, 1, 1), localInertiaTensor);//body 0
	AddRigidBody(0, BALL_JOINT_4D, _Vector3(-1.0f, 1.0f, -1.0f), _Vector3(1.0f, -1.0f, 1.0f), masterMeshArray[3], Vector3d(1, 1, 1), localInertiaTensor);//body 1

	MultiBodyInitialization();
	Forward();
}

void eae6320::MultiBody::UnitTest4()
{
	constraintSolverMode = IMPULSE;
	gravity = false;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, FREE_JOINT, _Vector3(0.0f, 0.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 0
	AddRigidBody(0, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 1

	MultiBodyInitialization();
	qdot.segment(3, 3) = _Vector3(-2.0f, 5.0f, 0.0f);
	qdot.segment(6, 3) = _Vector3(4.0f, -10.0f, 0.0f);
	Forward();
}

void eae6320::MultiBody::UnitTest6()
{
	numOfLinks = 1;
	constraintSolverMode = IMPULSE;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
	InitializeBodies(masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f));//4 is capsule, 3 is cube

	int jointTypeArray[] = { BALL_JOINT_4D };
	InitializeJoints(jointTypeArray);

	//SetZeroInitialCondition();

	_Vector3 rot_vec(-0.25 * M_PI, 0.0, 0);
	rel_ori[0] = Math::RotationConversion_VecToQuat(rot_vec);
	Forward();
	_Vector3 local_w = _Vector3(0.0, -2.0, 0.0);
	_Vector3 world_w = R_global[0] * local_w;
	qdot.segment(0, 3) = world_w;
	Forward();

	//jointRange[0].first = 0.75 * M_PI;//swing
	jointRange[0].second = 0.5 * M_PI;//twist
}

void eae6320::MultiBody::UnitTest17()
{
	numOfLinks = 1;
	constraintSolverMode = IMPULSE;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
	InitializeBodies(masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f));//4 is capsule, 3 is cube

	int jointTypeArray[] = { BALL_JOINT_4D };
	InitializeJoints(jointTypeArray);

	//SetZeroInitialCondition();

	_Vector3 rot_vec(0, 0.0, -0.25 * M_PI);
	rel_ori[0] = Math::RotationConversion_VecToQuat(rot_vec);
	Forward();
	_Vector3 local_w = _Vector3(0.0, -2.0, 0.0);
	_Vector3 world_w = R_global[0] * local_w;
	qdot.segment(0, 3) = world_w;
	Forward();

	//jointRange[0].first = 0.75 * M_PI;//swing
	jointRange[0].second = 0.5 * M_PI;//twist
}

void eae6320::MultiBody::UnitTest18()
{
	numOfLinks = 1;
	constraintSolverMode = IMPULSE;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
	InitializeBodies(masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f));//4 is capsule, 3 is cube

	int jointTypeArray[] = { BALL_JOINT_4D };
	InitializeJoints(jointTypeArray);

	//SetZeroInitialCondition();
	_Vector3 local_w = _Vector3(-2.0, 0.0, 2.0);;
	qdot.segment(0, 3) = local_w;
	Forward();

	jointRange[0].second = 1e-7;//twist
}

void eae6320::MultiBody::UnitTest7()
{
	_Vector3 p(0, -1, 0);
	_Vector3 s(1, 0, 1);
	//s = s.normalized() * M_PI;
	_Vector3 t(0, -0.5 * M_PI, 0);

	_Quat quatSwing = Math::RotationConversion_VecToQuat(s);
	_Quat quatTwist = Math::RotationConversion_VecToQuat(t);
	
	_Quat quat = quatSwing * quatTwist;
	_Vector3 totalR = Math::RotationConversion_QuatToVec(quat);
	/*quat.w() = 0;
	quat.x() = 1;
	quat.y() = 0;
	quat.z() = 0;*/
	std::cout << quat << std::endl;

	_Quat quatSwing_;
	_Quat quatTwist_;
	Math::SwingTwistDecomposition(quat, p, quatSwing_, quatTwist_);
	
	_Vector3 vecSwing = Math::RotationConversion_QuatToVec(quatSwing_);
	_Vector3 vecTwist = Math::RotationConversion_QuatToVec(quatTwist_);
	_Vector3 vecTwist2 = totalR.dot(t) * t.normalized();

	std::cout << s.transpose() << std::endl;
	std::cout << vecSwing.transpose() << std::endl << std::endl;
	std::cout << t.transpose() << std::endl;
	std::cout << vecTwist.transpose() << std::endl;
	std::cout << vecTwist2.transpose() << std::endl << std::endl;
}

void eae6320::MultiBody::UnitTest8()
{
	numOfLinks = 1;
	constraintSolverMode = IMPULSE;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
	InitializeBodies(masterMeshArray[3], Vector3d(1, 1, 1), localInertiaTensor, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f));//4 is capsule, 3 is cube

	int jointTypeArray[] = { BALL_JOINT_4D };
	InitializeJoints(jointTypeArray);

	//SetZeroInitialCondition();

	//_Vector3 rot_vec(-0.25 * M_PI, 0.0, 0.0);
	_Vector3 rot_vec(0.0, 0.0, 0.0);
	rel_ori[0] = Math::RotationConversion_VecToQuat(rot_vec);
	Forward();
	_Vector3 local_w = _Vector3(-2.0, 2.0, 0.0);
	_Vector3 world_w = R_global[0] * local_w;
	qdot.segment(0, 3) = world_w;
	Forward();

	jointLimit[0] = 0.5 * M_PI;
}

void eae6320::MultiBody::UnitTest9()
{
	numOfLinks = 1;
	constraintSolverMode = PBD;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
	InitializeBodies(masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f));//4 is capsule, 3 is cube

	int jointTypeArray[] = { BALL_JOINT_3D };
	InitializeJoints(jointTypeArray);

	//SetZeroInitialCondition();
	int jointID = 0;
	qdot.segment(velStartIndex[jointID], 3) = _Vector3(-2.0, 0.0, 0.0);

	Forward();

	constraintType = SWING_C;
	jointLimit[0] = 0.5 * M_PI;
}

void eae6320::MultiBody::UnitTest10()
{
	numOfLinks = 2;
	constraintSolverMode = IMPULSE;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
	InitializeBodies(masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f));//4 is capsule, 3 is cube

	int jointTypeArray[] = { BALL_JOINT_4D, BALL_JOINT_4D };
	InitializeJoints(jointTypeArray);

	//SetZeroInitialCondition();
	int jointID = 0;
	_Vector3 rotVec(-0.25 * M_PI, 0, 0);
	_Quat rotQuat = Math::RotationConversion_VecToQuat(rotVec);
	rel_ori[jointID] = rotQuat;
	jointID = 1;
	_Vector3 local_w = _Vector3(0.0, 2.0, 0.0);
	qdot.segment(velStartIndex[jointID], 3) = local_w;

	Forward();
	//jointRange[0].first = 0.5 * M_PI;//swing
	jointRange[0].second = 0.5 * M_PI;//twist
	jointRange[1].second = 0.5 * M_PI;//twist
}

void eae6320::MultiBody::UnitTest11()
{
	numOfLinks = 5;
	constraintSolverMode = IMPULSE;
	gravity = true;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
	InitializeBodies(masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor, _Vector3(-1, 0, 0), _Vector3(1, 0, 0));//4 is capsule, 3 is cube
	for (int i = 0; i < numOfLinks; i++) twistAxis[i] = _Vector3(1, 0, 0);//update the all links with a different twist axis
	
	int jointTypeArray[] = { BALL_JOINT_4D, BALL_JOINT_4D, BALL_JOINT_4D, BALL_JOINT_4D, BALL_JOINT_4D };
	InitializeJoints(jointTypeArray);
	
	//SetZeroInitialCondition();
	rel_ori[1] = Math::RotationConversion_VecToQuat(_Vector3(0, M_PI / 8, 0));


	Forward();
	for (int i = 0; i < numOfLinks; i++)
	{
		jointRange[i].first = 0.25 * M_PI;//swing
		jointRange[i].second = 0.000001;//twist
	}
}

void eae6320::MultiBody::UnitTest12()
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
		int dof = static_cast<int>(q.size());
		for (int i = 0; i < dof; i++)
		{
			fwrite(&q(i), sizeof(double), 1, i_pFile);
		}
		for (int i = 0; i < dof; i++)
		{
			fwrite(&qdot(i), sizeof(double), 1, i_pFile);
		}
	};
}

void eae6320::MultiBody::UnitTest13()
{
	numOfLinks = 1;
	constraintSolverMode = IMPULSE;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
	InitializeBodies(masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f));//4 is capsule, 3 is cube

	int jointTypeArray[] = { BALL_JOINT_4D };
	InitializeJoints(jointTypeArray);

	//SetZeroInitialCondition();

	//_Vector3 local_w = _Vector3(-2.0, 2.0, 0.0);
	//_Vector3 local_w = _Vector3(-2.0, 0.0, 2.0);
	_Vector3 local_w = _Vector3(-2.0, 0.0, 0.0);
	qdot.segment(0, 3) = local_w;
	Forward();

	//jointRange[0].second = 0.25 * M_PI;//twist
	jointRange[0].second = 0.000001;//twist
	//jointRange[0].second = 0.9  * M_PI;//twist
}

void eae6320::MultiBody::UnitTest14()
{
	numOfLinks = 5;
	constraintSolverMode = IMPULSE;
	gravity = true;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
	InitializeBodies(masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor, _Vector3(-1, 0, 0), _Vector3(1, 0, 0));//4 is capsule, 3 is cube

	int jointTypeArray[] = { BALL_JOINT_4D, BALL_JOINT_4D, BALL_JOINT_4D, BALL_JOINT_4D, BALL_JOINT_4D };
	InitializeJoints(jointTypeArray);
	ConfigurateBallJoint(_Vector3(1, 0, 0), _Vector3(0, 1, 0), _Vector3(0, 0, 1), -0.25 * M_PI, -0.000001);

	//SetZeroInitialCondition();
	rel_ori[1] = Math::RotationConversion_VecToQuat(_Vector3(0, M_PI / 8, 0));

	Forward();
}

void eae6320::MultiBody::UnitTest21()
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
}

void eae6320::MultiBody::UnitTest22()
{
	constraintSolverMode = IMPULSE;
	gravity = true;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 0
	AddRigidBody(0, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 1
	AddRigidBody(0, BALL_JOINT_4D, _Vector3(-1.0f, 0.0f, 0.0f), _Vector3(0.5f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor);//body 2
	//std::cout << parentArr[0] << parentArr[1] << parentArr[2] << std::endl;
	//std::cout << numOfLinks << std::endl;
	MultiBodyInitialization();
	Forward();
	ConfigureSingleBallJoint(2, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 0.25 * M_PI, -1);
}

void eae6320::MultiBody::UnitTest23()
{
	constraintSolverMode = IMPULSE;
	gravity = true;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, BALL_JOINT_4D, _Vector3(0.0f, 0.5f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(0.5, 0.5, 0.5), localInertiaTensor);//body 0 head
	AddRigidBody(0, BALL_JOINT_4D, _Vector3(0.0f, 0.5f, 0.0f), _Vector3(0.0f, -0.5f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor);//body 1 
	AddRigidBody(1, BALL_JOINT_4D, _Vector3(0.0f, 0.5f, 0.0f), _Vector3(0.0f, -0.5f, 0.0f), masterMeshArray[3], Vector3d(0.5, 0.5, 0.5), localInertiaTensor);//body 2
	AddRigidBody(2, BALL_JOINT_4D, _Vector3(0.0f, 0.5f, 0.0f), _Vector3(0.0f, -0.5f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor);//body 3
	AddRigidBody(3, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(-0.7f, -0.5f, 0.0f), masterMeshArray[3], Vector3d(0.5, 1, 0.5), localInertiaTensor);//body 4
	AddRigidBody(4, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f), masterMeshArray[3], Vector3d(0.5, 1, 0.5), localInertiaTensor);//body 5
	AddRigidBody(3, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.7f, -0.5f, 0.0f), masterMeshArray[3], Vector3d(0.5, 1, 0.5), localInertiaTensor);//body 6
	AddRigidBody(6, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f), masterMeshArray[3], Vector3d(0.5, 1, 0.5), localInertiaTensor);//body 7
	AddRigidBody(1, BALL_JOINT_4D, _Vector3(1.0f, 0.0f, 0.0f), _Vector3(-1.0f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor);//body 8
	AddRigidBody(8, BALL_JOINT_4D, _Vector3(1.0f, 0.0f, 0.0f), _Vector3(-1.0f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor);//body 9
	AddRigidBody(1, BALL_JOINT_4D, _Vector3(-1.0f, 0.0f, 0.0f), _Vector3(1.0f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor);//body 10
	AddRigidBody(10, BALL_JOINT_4D, _Vector3(-1.0f, 0.0f, 0.0f), _Vector3(1.0f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor);//body 11
	
	MultiBodyInitialization();
	_Vector3 world_w = _Vector3(-1.0, 0.0, -1.0);
	qdot.segment(0, 3) = world_w;
	Forward();
	
	//ConfigureSingleBallJoint(0, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 0.25 * M_PI, -1);
	//ConfigureSingleBallJoint(1, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 0.1, 0.5 * M_PI);
	ConfigureSingleBallJoint(2, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 0.1, 0.1);
	ConfigureSingleBallJoint(3, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 0.1, 0.1);
	/*ConfigureSingleBallJoint(4, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 0.25 * M_PI, 0.01);
	ConfigureSingleBallJoint(5, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 0.01, 0.01);
	ConfigureSingleBallJoint(6, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 0.25 * M_PI, 0.01);
	ConfigureSingleBallJoint(7, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 0.01, 0.01);*/
	/*ConfigureSingleBallJoint(8, _Vector3(-1, 0, 0), _Vector3(0, 0, -1), 0.3 * M_PI, 0.2);
	ConfigureSingleBallJoint(9, _Vector3(-1, 0, 0), _Vector3(0, 0, -1), 0.01, 0.2);
	ConfigureSingleBallJoint(10, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 0.3 * M_PI, 0.2);
	ConfigureSingleBallJoint(11, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 0.01, 0.2);*/
}

void eae6320::MultiBody::RagdollTest()
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
	AddRigidBody(1, BALL_JOINT_4D, _Vector3(0.95f, 0, 0), _Vector3(-1.15f, 1, 0.0f), masterMeshArray[3], Vector3d(0.8, 0.2, 0.2), localInertiaTensor);//body 9 left_arm0
	AddRigidBody(9, BALL_JOINT_4D, _Vector3(0.95f, 0, 0), _Vector3(-0.95f, 0, 0.0f), masterMeshArray[3], Vector3d(0.8, 0.2, 0.2), localInertiaTensor);//body 10 left_arm1
	AddRigidBody(10, BALL_JOINT_4D, _Vector3(0.35f, 0, 0), _Vector3(-0.9f, 0, 0.0f), masterMeshArray[3], Vector3d(0.25, 0.15, 0.25), localInertiaTensor);//body 11 left_hand
	AddRigidBody(1, BALL_JOINT_4D, _Vector3(-0.95f, 0, 0), _Vector3(1.15f, 1, 0.0f), masterMeshArray[3], Vector3d(0.8, 0.2, 0.2), localInertiaTensor);//body 12 right_arm0
	AddRigidBody(12, BALL_JOINT_4D, _Vector3(-0.95f, 0, 0), _Vector3(0.95f, 0, 0.0f), masterMeshArray[3], Vector3d(0.8, 0.2, 0.2), localInertiaTensor);//body 13 right_arm1
	AddRigidBody(13, BALL_JOINT_4D, _Vector3(-0.35f, 0, 0), _Vector3(0.9f, 0, 0.0f), masterMeshArray[3], Vector3d(0.25, 0.15, 0.25), localInertiaTensor);//body 14 right_hand

	MultiBodyInitialization();
	_Vector3 world_w = _Vector3(-1.0, 0.0, -1.0);
	qdot.segment(0, 3) = world_w;
	Forward();

	ConfigureSingleBallJoint(0, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 0.25 * M_PI, 0.1);//head
	ConfigureSingleBallJoint(1, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 0.1, 0.1);//chest0
	ConfigureSingleBallJoint(2, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 0.1, 0.5);//chest1
	ConfigureSingleBallJoint(3, _Vector3(0, -1, 0), _Vector3(0, 0, 1), 0.6 * M_PI, 0.01); //left_leg0
	ConfigureSingleBallJoint(4, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 0.1, 0.1); //left_leg1
	ConfigureSingleBallJoint(5, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 0.2, 0.1); //left_foot
	ConfigureSingleBallJoint(6, _Vector3(0, -1, 0), _Vector3(0, 0, 1), 0.6 * M_PI, 0.01); //right_leg0
	ConfigureSingleBallJoint(7, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 0.1, 0.1); //right_leg1
	ConfigureSingleBallJoint(8, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 0.2, 0.1); //right_foot
	ConfigureSingleBallJoint(9, _Vector3(-1, 0, 0), _Vector3(0, 0, 1), 0.25 * M_PI, 0.5 * M_PI); //left_arm0
	ConfigureSingleBallJoint(10, _Vector3(0, 1, 0), _Vector3(0, 0, 1), 0.1, 0.1); //left_arm1
	ConfigureSingleBallJoint(11, _Vector3(0, 0, 1), _Vector3(0, 1, 0), 0.1, 0.5 * M_PI); //left_hand
	ConfigureSingleBallJoint(12, _Vector3(1, 0, 0), _Vector3(0, 0, 1), 0.25 * M_PI, 0.5 * M_PI); //right_arm0
	ConfigureSingleBallJoint(13, _Vector3(0, 1, 0), _Vector3(0, 0, 1), 0.1, 0.1); //right_arm1
	ConfigureSingleBallJoint(14, _Vector3(0, 0, 1), _Vector3(0, 1, 0), 0.1, 0.5 * M_PI); //right_hand
}

void eae6320::MultiBody::UnitTest15()
{
	numOfLinks = 1;
	constraintSolverMode = IMPULSE;
	twistMode = INCREMENT;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
	InitializeBodies(masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f));//4 is capsule, 3 is cube

	int jointTypeArray[] = { BALL_JOINT_4D };
	InitializeJoints(jointTypeArray);

	////SetZeroInitialCondition();

	_Vector3 local_w = _Vector3(-2.0, 2.0, 0.0);
	qdot.segment(0, 3) = local_w;
	Forward();

	jointRange[0].second = 0.5 * M_PI;//twist
}

void eae6320::MultiBody::PersistentDataTest()
{
	_Vector3 vec(1.1, 1.2, 1.3);
	FILE * pFile;
	const char* filePath = "sim_state.txt";
	pFile = fopen(filePath, "wb");
	fwrite(&vec, sizeof(double) * 3, 1, pFile);
	fclose(pFile);

	pFile = fopen(filePath, "rb");
	_Vector3 out;
	out.setZero();
	fread(&out(0), sizeof(double) * 3, 1, pFile);
	fclose(pFile);
	std::cout << out.transpose() << std::endl;
}

void eae6320::MultiBody::UnitTest16()
{
	numOfLinks = 1;
	constraintSolverMode = IMPULSE;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
	InitializeBodies(masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f));//4 is capsule, 3 is cube

	int jointTypeArray[] = { BALL_JOINT_4D };
	InitializeJoints(jointTypeArray);

	////SetZeroInitialCondition();

	const char* filePath = "../../../../TestCases/sim_state3.txt";
	FILE* pFile = fopen(filePath, "rb");
	for (int i = 0; i < 3; i++)
	{
		fread(&qdot(i), sizeof(double) , 1, pFile);
	}
	fread(&rel_ori[0], sizeof(double) * 4, 1, pFile);
	fclose(pFile);
	Forward();

	//jointRange[0].second = 0.5 * M_PI;//twist
	jointRange[0].second = 1e-6;//twist
}

void eae6320::MultiBody::EulerDecompositionAccuracyTest()
{
	//for (double beta = 1.57; beta < 0.5 * M_PI + 0.00001 * 20; beta += 0.00001)
	//{
	//	_Vector3 gammaVec(0.25, 0, 0);
	//	_Quat quatGamma = Math::RotationConversion_VecToQuat(gammaVec);
	//	_Matrix3 gammaMat = Math::RotationConversion_VecToMatrix(gammaVec);

	//	_Vector3 betaVec(0, 0, beta);
	//	_Quat quatBeta = Math::RotationConversion_VecToQuat(betaVec);
	//	_Matrix3 betaMat = Math::RotationConversion_VecToMatrix(betaVec);

	//	_Vector3 alphaVec(0, 0.2 * M_PI, 0);
	//	_Quat quatAlpha = Math::RotationConversion_VecToQuat(alphaVec);
	//	_Matrix3 alphaMat = Math::RotationConversion_VecToMatrix(alphaVec);

	//	_Quat totalQuat =  quatAlpha * quatBeta * quatGamma;
	//	_Matrix3 totalMat = alphaMat * betaMat * gammaMat;
	//	/*totalQuat.x() += 0.000001;
	//	totalQuat.normalize();*/
	//	
	//	_Scalar eulerAngles[3];
	//	Math::quaternion2Euler(totalQuat, eulerAngles, Math::RotSeq::yzx);
	//	std::cout << "----true beta " << beta << std::endl;
	//	std::cout << "alpha " << eulerAngles[2] << " beta " << eulerAngles[1] << " gamma " << eulerAngles[0] << std::endl;
	//	
	//	_Scalar mAlpha, mBeta, mGamma;
	//	mAlpha = atan2(-totalMat(2, 0), totalMat(0, 0));
	//	mBeta = asin(totalMat(1, 0));
	//	mGamma = atan2(-totalMat(1, 2), totalMat(1, 1));
	//	std::cout << "alpha " << mAlpha << " beta " << mBeta << " gamma " << mGamma << std::endl;
	//}
	
	_Vector3 gammaVec(0.2, 0, 0);
	_Matrix3 gamma = Math::RotationConversion_VecToMatrix(gammaVec);
	
	_Vector3 betaVec(0, 0, 1.5708);
	_Matrix3 beta = Math::RotationConversion_VecToMatrix(betaVec);
	
	_Vector3 alphaVec(0, 0.2, 0);
	_Matrix3 alpha = Math::RotationConversion_VecToMatrix(alphaVec);

	_Matrix3 totalM = alpha * beta * gamma;
	_Scalar mGamma = atan2(-totalM(1, 2), totalM(1, 1));
	_Scalar mAlpha = atan2(-totalM(2, 0), totalM(0, 0));
	_Scalar test = totalM(0, 2) * totalM(1, 1) - totalM(0, 1) * totalM(1, 2);
	std::cout << totalM << std::endl;
	std::cout << mAlpha << std::endl;
	std::cout << test << std::endl;
}

void eae6320::MultiBody::UnitTest24()
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

void eae6320::MultiBody::UnitTest25()
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
	ConfigureSingleBallJoint(0, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), -1, 1e-4);

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

void eae6320::MultiBody::UnitTest26()
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
	std::cout << qdot.transpose() << " " << vel[0].transpose() << endl;
	ConfigureSingleBallJoint(0, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), 3.089, 1.5708);
}

void eae6320::MultiBody::UnitTest27()
{
	constraintSolverMode = IMPULSE;
	gravity = false;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, BALL_JOINT_4D, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 0

	MultiBodyInitialization();
	qdot.segment(0, 3) = _Vector3(-3, -3, 0);
	Forward();
	ConfigureSingleBallJoint(0, _Vector3(0, -1, 0), _Vector3(-1, 0, 0), -1, 1e+7);
	
	m_MatlabSave = [this]()
	{
		_Scalar t = (_Scalar)eae6320::Physics::totalSimulationTime;
		LOG_TO_FILE << t << " " << totalTwist[0] << std::endl;
	};
	m_control = [this]()
	{
		_Scalar t = (_Scalar)eae6320::Physics::totalSimulationTime;
		_Vector3 tau = sin(t) * old_R_local[0] * twistAxis[0];
		externalForces[0].block<3, 1>(3, 0) = tau;
		//std::cout << "my control " << totalTwist[0] << std::endl;
	};
}

void eae6320::MultiBody::RunUnitTest()
{
	//UnitTest1(); //test swing twist decomposition with rotaion matrix
	//UnitTest2(); //test extreme case for twist with single body
	//UnitTest3();//chain mimic with two bodies
	//UnitTest4();//angular velocity of _Vector3(-2.0, 2.0, 0.0) for the 2nd body
	//UnitTest5();//test induced twist for single body; use swing limit to enforce singularity point pass through
	//UnitTest6();//two basic intial conditions to verify Euler twist constraint
	//UnitTest7();//test swing twist decomp with quat
	//UnitTest8(); //mujoco ball joint constraint test for single body
	//UnitTest9();//swing for 3d ball joint
	//UnitTest10();//twist invarience two bodies.
	//UnitTest11();//5 body
	//UnitTest12();//2 cube
	//HingeJointUnitTest0();//hinge joint with auto constraint
	//UnitTest13();//vector vield switch test
	//UnitTest14();//5 body for Euler twist
	//UnitTest15();//incremental model single body
	//PersistentDataTest();
	//UnitTest16();//load initial condition from file
	//EulerDecompositionAccuracyTest();
	
	m_MatlabSave = [this]()
	{
		_Scalar t = (_Scalar)eae6320::Physics::totalSimulationTime;
		LOG_TO_FILE << t << " ";
		for (int i = 0; i < numOfLinks; i++)
		{
			LOG_TO_FILE << pos[i].transpose() << " " << rel_ori[i];
			if (i != numOfLinks - 1)
			{
				LOG_TO_FILE << " ";
			}
			else
			{
				LOG_TO_FILE << " " << ComputeTotalEnergy() << std::endl;
			}
		}
	};
	
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

	int testCaseNum = 0;
	Application::AddApplicationParameter(&testCaseNum, Application::ApplicationParameterType::integer, L"-example");
	if (testCaseNum == 0)
	{
		UnitTest6();
		std::cout << "basic intial conditions to verify Euler twist constraint (induced swing)" << std::endl;
	}
	else if (testCaseNum == 1)
	{
		UnitTest17();
		std::cout << "basic intial conditions to verify Euler twist constraint (no induced swing)" << std::endl;
	}
	else if (testCaseNum == 2)
	{
		UnitTest18();
		/*animationDuration = 2;
		frameNum = 30;*/
		std::cout << "0 twist constraint" << std::endl;
	}
	else if (testCaseNum == 3)
	{
		UnitTest13();
	/*	animationDuration = 2;
		frameNum = 18;*/
		std::cout << "singularity pass through test" << std::endl;
	}
	else if (testCaseNum == 4)
	{
		UnitTest5();
	/*	animationDuration = 5;

		if (twistMode == EULER_V2)
		{
			frameNum = 18;
		}
		else if (twistMode == INCREMENT)
		{
			frameNum = 30;
		}*/
		std::cout << "regularization test" << std::endl;
	}
	else if (testCaseNum == 5)
	{
		UnitTest21();
		/*animationDuration = 5;
		frameNum = 24 * 5;*/
		std::cout << "5 body for Euler twist" << std::endl;
	}
	else if (testCaseNum == 6)
	{
		UnitTest19();
		std::cout << "singularity winding test 90" << std::endl;
	}
	else if (testCaseNum == 7)
	{
		UnitTest20();
		std::cout << "singularity winding test 180" << std::endl;
	}
	else if (testCaseNum == 8)
	{
		UnitTest22();
		std::cout << "branch test" << std::endl;
	}
	else if (testCaseNum == 9)
	{
		UnitTest23();
		std::cout << "human skeleton test" << std::endl;
	}
	else if (testCaseNum == 10)
	{
		UnitTest3();
		std::cout << "double cube with fixed root" << std::endl;
	}
	else if (testCaseNum == 11)
	{
		UnitTest4();
		std::cout << "double cube without fixed root" << std::endl;
	}
	else if (testCaseNum == 12)
	{
		UnitTest24();
		std::cout << "zero twist vector field pattern test (side)" << std::endl;
	}
	else if (testCaseNum == 13)
	{
		UnitTest25();
		std::cout << "zero twist vector field pattern test (Direct)" << std::endl;
	}
	else if (testCaseNum == 14)
	{
		UnitTest26();
		std::cout << "compare with Unreal and Unity" << std::endl;
	}
	else if (testCaseNum == 15)
	{
		UnitTest27();
		std::cout << "compare two different incremental methods" << std::endl;
	}
	else if (testCaseNum == 16)
	{
		UnitTest12();
		std::cout << "hinge joints" << std::endl;
	}
	else if (testCaseNum == 17)
	{
		FDTest();
		std::cout << "FDTest" << std::endl;
	}
	else if (testCaseNum == 18)
	{
		AnalyticalTest();
		std::cout << "Analytical derivative test" << std::endl;
	}
	else if (testCaseNum == 19)
	{
		RagdollTest();
		std::cout << "ragdoll test" << std::endl;
	}
	Application::AddApplicationParameter(&enablePositionSolve, Application::ApplicationParameterType::integer, L"-ps");
	if (enablePositionSolve == 1)
	{
		std::cout << "position solve enabled" << std::endl;
	}
	else
	{
		std::cout << "position solve disabled" << std::endl;
	}
	std::cout << std::endl;
}
