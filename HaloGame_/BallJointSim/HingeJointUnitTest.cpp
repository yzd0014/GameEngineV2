#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>

void eae6320::MultiBody::HingeJointUnitTest0()
{
	numOfLinks = 1;
	constraintSolverMode = -1;
	//gravity = true;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;
	InitializeBodies(masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f));//4 is capsule, 3 is cube

	int jointTypeArray[] = { HINGE_JOINT };
	hingeDirLocals[0] = _Vector3(0, 0, 1);
	hingeDirGlobals[0] = hingeDirLocals[0];
	hingeMagnitude[0] = 0;
	InitializeJoints(jointTypeArray);

	SetZeroInitialCondition();
	qdot.segment(velStartIndex[0], 1)(0) = 2;
	Forward();
}