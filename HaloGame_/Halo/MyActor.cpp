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
	GameplayUtility::DrawArrow(Vector3d(0, 0, 0), Vector3d(0, -1, 0), Math::sVector(1, 0, 0), 0.9);
	GameplayUtility::DrawArrow(Vector3d(0, 0, 0), Vector3d(1, 0, 0), Math::sVector(0, 1, 0), 0.9);
	twistAxis = Vector3d(0, -1, 0);
	x = Vector3d(1, 0, 0);
	z = Vector3d(0, 0, 1);

	lambda.resize(n);
	targetAngle.resize(n);
	targetAngle(0) = 0;
	targetAngle(1) = M_PI;
	normals.resize(n);
	normals[0] = Vector3d(1, 0, 0);
	normals[1] = Vector3d(-1, 0, 0);
	
	MatrixXd A;
	A.resize(n, n);
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			A(i, j) = normals[i].dot(normals[j]) + 1;
		}
	}
	lambda = A.inverse() * targetAngle;

	Vector3d twistAxisWorld;
	Matrix3d Ry;
	for (angleLeftRight = 0; angleLeftRight <= 2 * M_PI; angleLeftRight += M_PI * 0.1)
	{
		Ry = Math::RotationConversion_VecToMatrix(Vector3d(0, angleLeftRight, 0));
		for (angleUpDown = 0; angleUpDown <= 0.91 * M_PI; angleUpDown += M_PI * 0.1)
		{
			Vector3d r0;
			r0 = Vector3d(-angleUpDown, 0, 0);
			r0 = Ry * r0;
			Matrix3d R = Math::RotationConversion_VecToMatrix(r0);
			twistAxisWorld = R * twistAxis;
			
			double angleOffset = 0;
			for (int i = 0; i < n; i++)
			{
				angleOffset += lambda(i) * (twistAxisWorld.dot(normals[i]) + 1);
			}
			
			Vector3d xWorld = R * x;
			Vector3d zWorld = R * z;
			Vector3d newVec;
			newVec = sin(angleOffset) * zWorld + cos(angleOffset) * xWorld;

			GameplayUtility::DrawArrowScaled(twistAxisWorld * rigidBodyScale, xWorld, Math::sVector(1, 0, 0), Vector3d(0.05, 0.1, 0.05));
			//GameplayUtility::DrawArrowScaled(twistAxisWorld * rigidBodyScale, zWorld, Math::sVector(0, 0, 1), Vector3d(0.05, 0.1, 0.05));
			GameplayUtility::DrawArrowScaled(twistAxisWorld * rigidBodyScale, newVec, Math::sVector(0, 1, 1), Vector3d(0.05, 0.1, 0.05));
		}
	}
	
}

void eae6320::MyActor::Tick(const double i_secondCountToIntegrate)
{
	/*if (UserInput::KeyState::currFrameKeyState['F']) angleUpDown += i_secondCountToIntegrate;
	if (UserInput::KeyState::currFrameKeyState['R']) angleLeftRight += i_secondCountToIntegrate;
	if (angleUpDown > 2 * M_PI) angleUpDown = 0;
	if (angleLeftRight > 2 * M_PI) angleLeftRight = 0;

	Vector3d twistAxisWorld;
	Matrix3d Ry;
	Ry = Math::RotationConversion_VecToMatrix(Vector3d(0, angleLeftRight, 0));

	Vector3d r0;
	r0 = Vector3d(-angleUpDown, 0, 0);
	r0 = Ry * r0;
	Vector3d r1;
	r1= Vector3d(M_PI - angleUpDown, 0, 0);
	r1 = Ry * r1;
	
	Matrix3d R0 = Math::RotationConversion_VecToMatrix(r0);
	Matrix3d R1 = Math::RotationConversion_VecToMatrix(r1);
	
	Matrix3d R;
	if (angleUpDown < 0.5 * M_PI || angleUpDown > 1.5 * M_PI)
	{
		R = R0;
		twistAxisWorld = R * twistAxis0;
	}
	else
	{
		R = R1;
		twistAxisWorld = R * twistAxis1;
	}

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
	zArrow = GameplayUtility::DrawArrow(twistAxisWorld * rigidBodyScale, z0World, Math::sVector(0, 0, 1), 0.5);*/
}

