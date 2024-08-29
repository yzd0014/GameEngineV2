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
	x = Vector3d(1, 0, 0);
	z = Vector3d(0, 0, 1);
	twistAxis = Vector3d(0, -1, 0);
	UnitTest3();
}

void eae6320::MyActor::UnitTest1()
{
	GameplayUtility::DrawArrow(Vector3d(0, 0, 0), Vector3d(0, -1, 0), Math::sVector(1, 0, 0), 0.9);
	GameplayUtility::DrawArrow(Vector3d(0, 0, 0), Vector3d(1, 0, 0), Math::sVector(0, 1, 0), 0.9);
	
	
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
	GameplayUtility::DrawArrow(Vector3d(0, 0, 0), Vector3d(0, -1, 0), Math::sVector(1, 0, 0), 0.9);
	GameplayUtility::DrawArrow(Vector3d(0, 0, 0), Vector3d(1, 0, 0), Math::sVector(0, 1, 0), 0.9);
	
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

void eae6320::MyActor::UnitTest3()
{
	int numOfPoints = 30;
	double coneAngle = 0.49 * M_PI;
	double deltaAngle = 2 * M_PI / numOfPoints;
	Vector3d referenceVec(0, -1, 0);
	Vector3d v0(0, 0, 1);
	Vector3d rotVec(-coneAngle, 0, 0);
	v0 = Math::RotationConversion_VecToMatrix(rotVec) * v0;
	double totalTwist = 0;
	for (double currAngle = 0; currAngle < 2 * M_PI; currAngle += deltaAngle)
	{
		if (currAngle > 0)
		{
			rotVec = Vector3d(0, 0, -currAngle);
			Vector3d currV0;
			currV0 = Math::RotationConversion_VecToMatrix(rotVec) * v0;
			double rotAngle = Math::GetAngleBetweenTwoVectors(referenceVec, currV0);
			rotVec = rotAngle * referenceVec.cross(currV0).normalized();
			Matrix3d currR = Math::RotationConversion_VecToMatrix(rotVec);
			Matrix3d currEulerR = ComputeEulerSwing(currR, referenceVec);
			Quaterniond currEulerQuat = Math::RotationConversion_MatToQuat(currEulerR);
			Vector3d zWorld = currEulerR * z;
			GameplayUtility::DrawArrowScaled(rigidBodyScale * currEulerR * referenceVec , zWorld, Math::sVector(1, 0, 0), Vector3d(0.05, 0.05, 0.05));

			rotVec = Vector3d(0, 0, -(currAngle - deltaAngle));
			Vector3d lastV0;
			lastV0 = Math::RotationConversion_VecToMatrix(rotVec) * v0;
			rotAngle = Math::GetAngleBetweenTwoVectors(referenceVec, lastV0);
			rotVec = rotAngle * referenceVec.cross(lastV0).normalized();
			Matrix3d lastR = Math::RotationConversion_VecToMatrix(rotVec);
			Matrix3d lastEulerR = ComputeEulerSwing(lastR, referenceVec);
			Quaterniond lastEulerQuat = Math::RotationConversion_MatToQuat(lastEulerR);
			zWorld = lastEulerR * z;
			GameplayUtility::DrawArrowScaled(rigidBodyScale * lastEulerR * referenceVec, zWorld, Math::sVector(0, 1, 0), Vector3d(0.05, 0.05, 0.05));

			Quaterniond deltaQuat;
			deltaQuat = currEulerQuat * lastEulerQuat.inverse();
			Quaterniond twistComponent, swingComponent;
			Math::SwingTwistDecomposition(deltaQuat, lastV0, swingComponent, twistComponent);
			Vector3d twistVec = Math::RotationConversion_QuatToVec(twistComponent);
			totalTwist += twistVec.norm();
		}
	}
	std::cout << totalTwist << std::endl;
}

Matrix3d eae6320::MyActor::ComputeEulerSwing(Matrix3d i_Mat, Vector3d i_referencVec)
{
	Matrix3d out;
	out.setIdentity();
	Vector3d twistAxisWorld = i_Mat * i_referencVec;
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

		out = parentR * childR;
	}
	else
	{
		std::cout << "Cross product issue" << std::endl;
	}
	return out;
}

void eae6320::MyActor::Tick(const double i_secondCountToIntegrate)
{
	
}

