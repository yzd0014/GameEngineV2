#include "MyActor.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>

eae6320::MyActor::MyActor(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State):
	GameCommon::GameObject(i_pEffect, i_Mesh, i_State)
{
	GameplayUtility::DrawArrow(Vector3d(0, 0, 0), Vector3d(1, 0, 0), Math::sVector(1, 0, 0), 0.1);
	twistAxis0 = Vector3d(0, -1, 0);
	x0 = Vector3d(1, 0, 0);
	z0 = Vector3d(0, 0, 1);
}

void eae6320::MyActor::Tick(const double i_secondCountToIntegrate)
{
	if (UserInput::KeyState::currFrameKeyState['F']) angleUpDown += 0.01;
	if (UserInput::KeyState::currFrameKeyState['R']) angleLeftRight += 0.01;
	if (angleUpDown > 2 * M_PI) angleUpDown = 0;
	if (angleLeftRight > 2 * M_PI) angleLeftRight = 0;

	Vector3d twistAxisWorld;
	
	Vector3d r(-1, 0, 0);
	r = angleUpDown * r;
	Matrix3d Ry = Math::RotationConversion_VecToMatrix(Vector3d(0, angleLeftRight, 0));
	r = Ry * r;
	Matrix3d R = Math::RotationConversion_VecToMatrix(r);
	twistAxisWorld = R * twistAxis0;
	Vector3d x0World = R * x0;
	Vector3d z0World = R * z0;
	
	if (xArrow != nullptr)
	{
		xArrow->DestroyGameObject();
		xArrow = nullptr;
	}
	if (zArrow != nullptr)
	{
		zArrow->DestroyGameObject();
		zArrow = nullptr;
	}
	xArrow = GameplayUtility::DrawArrow(twistAxisWorld * rigidBodyScale, x0World, Math::sVector(1, 0, 0), 0.5);
	zArrow = GameplayUtility::DrawArrow(twistAxisWorld * rigidBodyScale, z0World, Math::sVector(0, 0, 1), 0.5);
}

