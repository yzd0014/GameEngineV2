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
	
	if (mode == 0)
	{
		GameplayUtility::DrawArrowScaled(twistAxis * rigidBodyScale, z, Math::sVector(1, 0, 0), Vector3d(0.05, 0.05, 0.05));
		double sRadius = rigidBodyScale;
		int numOfPointsEquator = 50;
		double totalLength = 2 * M_PI * sRadius;
		double pointsInterval = totalLength / numOfPointsEquator;
		Matrix3d yRotation;
		double rightAngle = 0;
		double currLength = 0;
		Vector3d twistAxisWorld;
		/*for (; currLength < totalLength; currLength += pointsInterval)
		{
			rightAngle = currLength / sRadius;
			yRotation = Math::RotationConversion_VecToMatrix(Vector3d(0, rightAngle, 0));
			Vector3d rotVec = Vector3d(-0.5 * M_PI, 0, 0);
			rotVec = yRotation * rotVec;
			Matrix3d R = Math::RotationConversion_VecToMatrix(rotVec);
			twistAxisWorld = R * twistAxis;

			Vector3d zWorld = R * z;
			GameplayUtility::DrawArrowScaled(twistAxisWorld * rigidBodyScale, zWorld, Math::sVector(1, 0, 0), Vector3d(0.05, 0.05, 0.05));
		}*/
	
		double upAngleInterval = M_PI * 0.04;
		for (double upAngle = upAngleInterval; upAngle <= 0.98 * M_PI; upAngle += upAngleInterval)
		{
			sRadius = sin(upAngle) * rigidBodyScale;
			totalLength = 2 * M_PI * sRadius;
			for (currLength = 0; currLength < totalLength; currLength += pointsInterval)
			{
				rightAngle = currLength / sRadius;
				yRotation = Math::RotationConversion_VecToMatrix(Vector3d(0, rightAngle, 0));
				Vector3d rotVec = Vector3d(-upAngle, 0, 0);
				rotVec = yRotation * rotVec;
				Matrix3d R = Math::RotationConversion_VecToMatrix(rotVec);
				twistAxisWorld = R * twistAxis;

				Vector3d zWorld = R * z;
				GameplayUtility::DrawArrowScaled(twistAxisWorld * rigidBodyScale, zWorld, Math::sVector(1, 0, 0), Vector3d(0.05, 0.05, 0.05));
			}
		}
	}
	else if (mode == 1)
	{
		lambda.resize(n);
		targetAngle.resize(n);
		targetAngle(0) = 0;
		targetAngle(1) = M_PI;
		//targetAngle(2) = 1.5 * M_PI;
		normals.resize(n);
		normals[0] = Vector3d(1, 0, 0);
		normals[1] = Vector3d(-1, 0, 0);
		//normals[2] = Vector3d(0, 0, -1);

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

				GameplayUtility::DrawArrowScaled(twistAxisWorld * rigidBodyScale, xWorld, Math::sVector(1, 0, 0), Vector3d(0.05, 0.05, 0.05));
				//GameplayUtility::DrawArrowScaled(twistAxisWorld * rigidBodyScale, zWorld, Math::sVector(0, 0, 1), Vector3d(0.05, 0.1, 0.05));
				GameplayUtility::DrawArrowScaled(twistAxisWorld * rigidBodyScale, newVec, Math::sVector(0, 1, 1), Vector3d(0.05, 0.05, 0.05));
			}
		}
	}
}

void eae6320::MyActor::Tick(const double i_secondCountToIntegrate)
{
}

