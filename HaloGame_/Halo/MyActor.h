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
		Vector3d twistAxis0;
		Vector3d twistAxis1;
		Vector3d x0;
		Vector3d z0;
		
		double angleUpDown = 0;
		double angleLeftRight = 0;
		double rigidBodyScale = 2;
		GameObject* xArrow = nullptr;
		GameObject* zArrow = nullptr;
	};
}