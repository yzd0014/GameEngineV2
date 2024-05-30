#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
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
	Math::TwistSwingDecompsition(R, p, R_t_bar, R_s_bar);
	
	std::cout << s.transpose() << std::endl;
	std::cout << Math::RotationConversion_MatrixToVec(R_s_bar).transpose() << std::endl << std::endl;
	std::cout << rotated_p.transpose() << std::endl;
	std::cout << Math::RotationConversion_MatrixToVec(R_t_bar).transpose() << std::endl << std::endl;
}

void eae6320::MultiBody::UnitTest2()
{
	//numOfLinks = 2;
	//InitializeBodies(masterMeshArray[4]);//4 is capsule, 3 is cube
	//int jointTypeArray[] = { BALL_JOINT_4D, BALL_JOINT_4D };
	//InitializeJoints(jointTypeArray);
	//SetZeroInitialCondition();

	///*uLocals[0][1] = _Vector3(1.0f, -1.0f, 1.0f);
	//uLocals[1][0] = _Vector3(-1.0f, 1.0f, -1.0f);*/

	////uLocals[0][0] = _Vector3(1.0f, -1.0f, -1.0f);

	///*qdot.segment(3, 3) = _Vector3(-2.0f, 5.0f, 0.0f);
	//qdot.segment(6, 3) = _Vector3(4.0f, -10.0f, 0.0f);*/

	////qdot.segment(0, 3) = _Vector3(-2.0f, 2.0f, 0.0f);
	////qdot.segment(3, 3) = _Vector3(2.0, 2.0, 0.0);

	////UnitTest0();
	////general twist test
	////_Vector3 rot_vec(0.0, 0.0, 0.0);
	//_Vector3 rot_vec(-0.25 * M_PI, 0.0, 0.0);
	//if (jointType[0] == BALL_JOINT_4D)
	//{
	//	rel_ori[0] = Math::RotationConversion_VecToQuat(rot_vec);
	//}
	//else if (jointType[0] == BALL_JOINT_3D)
	//{
	//	q.segment(0, 3) = rot_vec;
	//}
	////_Vector3 local_w = _Vector3(0.0, -2.0, -2.0);
	//_Vector3 local_w = _Vector3(0.0, -2.0, 0.0);
	//Forward();
	//int jointID = 1;
	//_Vector3 world_w = R_global[jointID] * local_w;

	//if (jointType[jointID] == BALL_JOINT_4D)
	//{
	//	//qdot.segment(velStartIndex[jointID], 3) = world_w;
	//	qdot.segment(velStartIndex[jointID], 3) = local_w;
	//}
	//else if (jointType[jointID] == BALL_JOINT_3D)
	//{
	//	qdot.segment(velStartIndex[jointID], 3) = J_rotation[jointID].inverse() * world_w;
	//}

	////swing test
	////_Vector3 rot_vec(0.0, 0.7 * M_PI, 0.0);
	////q.segment(0, 3) = rot_vec;
	////if (jointType[0] == BALL_JOINT_4D)
	////{
	////	rel_ori[0] = Math::RotationConversion_VecToQuat(rot_vec);
	////}
	////_Vector3 local_w = _Vector3(-2.0, 0.0, 2.0);
	////Forward();
	////qdot.segment(0, 3) = J_rotation[0].inverse() * local_w;
	////if (jointType[0] == BALL_JOINT_4D)
	////{
	////	qdot.segment(0, 3) = local_w;
	////}

	////qdot.segment(0, 3) = _Vector3(-2.0, 2.0, 0.0);
	////q.segment(0, 3) = _Vector3(0.0, 0.5 * M_PI, 0.0);
	////qdot.segment(0, 3) = _Vector3(-2.0, 0.0, 2.0);

	//Forward();
	////jointLimit[0] = 0.785f;
	////jointLimit[0] = 0.5 * M_PI;
	////jointLimit[1] = 0.09f;

	////jointRange[0].first = 0.5 * M_PI;//swing
	//jointRange[0].second = 0.5 * M_PI;//twist
	////jointRange[1].first = 0.5 * M_PI;//swing
	//jointRange[1].second = 0.5 * M_PI;//twist
}

void eae6320::MultiBody::UnitTest3()
{
	numOfLinks = 2;
	constraintSolverMode = IMPULSE;
	
	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
	InitializeBodies(masterMeshArray[4], localInertiaTensor, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f));//4 is capsule, 3 is cube
	
	int jointTypeArray[] = { BALL_JOINT_4D, BALL_JOINT_4D };
	InitializeJoints(jointTypeArray);
	
	SetZeroInitialCondition();

	_Vector3 rot_vec(-0.25 * M_PI, 0.0, 0.0);
	rel_ori[0] = Math::RotationConversion_VecToQuat(rot_vec);

	_Vector3 local_w = _Vector3(0.0, -2.0, 0.0);
	int jointID = 1;
	qdot.segment(velStartIndex[jointID], 3) = local_w;
	
	Forward();
	
	jointRange[0].second = 0.5 * M_PI;//twist
	jointRange[1].second = 0.5 * M_PI;//twist
}