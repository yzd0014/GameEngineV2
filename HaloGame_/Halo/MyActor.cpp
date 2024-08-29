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
	UnitTest1();
}

void eae6320::MyActor::UnitTest1()
{
	GameplayUtility::DrawArrow(Vector3d(0, 0, 0), Vector3d(0, -1, 0), Math::sVector(1, 0, 0), 0.9);
	GameplayUtility::DrawArrow(Vector3d(0, 0, 0), Vector3d(1, 0, 0), Math::sVector(0, 1, 0), 0.9);
	twistAxis = Vector3d(0, -1, 0);
	x = Vector3d(1, 0, 0);
	z = Vector3d(0, 0, 1);

	GameplayUtility::DrawArrowScaled(twistAxis * rigidBodyScale, z, Math::sVector(1, 0, 0), Vector3d(0.05, 0.05, 0.05));
	double sRadius = rigidBodyScale;
	int numOfPointsEquator = 50;
	double totalLength = 2 * M_PI * sRadius;
	double pointsInterval = totalLength / numOfPointsEquator;
	Matrix3d yRotation;
	double rightAngle = 0;
	double currLength = 0;
	Vector3d twistAxisWorld;
	double upAngleInterval = M_PI * 0.04;
	for (double upAngle = upAngleInterval; upAngle <= 0.98 * M_PI; upAngle += upAngleInterval)
	{
		sRadius = sin(upAngle) * rigidBodyScale;
		totalLength = 2 * M_PI * sRadius;
		for (currLength = 0; currLength < totalLength; currLength += pointsInterval)
		{
			//direct swing
			rightAngle = currLength / sRadius;
			yRotation = Math::RotationConversion_VecToMatrix(Vector3d(0, rightAngle, 0));
			Vector3d rotVec = Vector3d(-upAngle, 0, 0);
			rotVec = yRotation * rotVec;
			Matrix3d R = Math::RotationConversion_VecToMatrix(rotVec);
			twistAxisWorld = R * twistAxis;
			{
				Vector3d zWorld = R * z;
				GameplayUtility::DrawArrowScaled(twistAxisWorld * rigidBodyScale, zWorld, Math::sVector(1, 0, 0), Vector3d(0.05, 0.05, 0.05));
			}

			//euler swing
			double zProjection = twistAxisWorld.dot(z);
			if (abs(zProjection - 1) > 0.01)
			{
				Vector3d twistProjection = twistAxisWorld - zProjection * z;
				twistProjection.normalize();

				double parentRotAngle = Math::GetAngleBetweenTwoVectors(twistProjection, twistAxis);
				Vector3d signVec = twistAxis.cross(twistProjection);
				if (signVec(2) < 0) parentRotAngle = -parentRotAngle;
				Vector3d parentRotVec = Vector3d(0, 0, parentRotAngle);
				Matrix3d parentR = Math::RotationConversion_VecToMatrix(parentRotVec);

				double childRotAngle = Math::GetAngleBetweenTwoVectors(twistAxisWorld, twistProjection);
				if (twistAxisWorld(2) < 0) childRotAngle = -childRotAngle;
				Vector3d childRotVec = Vector3d(-childRotAngle, 0, 0);
				Matrix3d childR = Math::RotationConversion_VecToMatrix(childRotVec);

				Matrix3d eulerR = parentR * childR;
				Vector3d zWorld = eulerR * z;
				GameplayUtility::DrawArrowScaled(twistAxisWorld * rigidBodyScale, zWorld, Math::sVector(0, 0, 1), Vector3d(0.05, 0.05, 0.05));
			}
		}
	}
}

void eae6320::MyActor::UnitTest2()
{
}

void eae6320::MyActor::Tick(const double i_secondCountToIntegrate)
{
	
}

