#pragma once
#include "Engine/GameCommon/GameObject.h"

#include "External/EigenLibrary/Eigen/Dense"
#include "External/EigenLibrary/Eigen/Geometry"
#include "Engine/Math/DataTypeDefine.h"
#include "Engine/Math/3DMathHelpers.h"

namespace eae6320
{
	class MyActor : public eae6320::GameCommon::GameObject
	{
	public:
		MyActor(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State);
		void Tick(const double i_secondCountToIntegrate) override;

	private:
		void UnitTest1();//display two different vector fields
		void UnitTest2();//vector field interpolation
		void UnitTest3();
		
		VectorXd lambda;
		VectorXd targetAngle;
		std::vector<Vector3d> normals;

		Vector3d twistAxis;
		Vector3d x;
		Vector3d z;
		
		double angleUpDown = 0;
		double angleLeftRight = 0;
		double rigidBodyScale = 2;
		GameObject* xArrow = nullptr;
		GameObject* zArrow = nullptr;

		int n = 2;
	};
}