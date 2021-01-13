#pragma once
#include "Engine/GameCommon/GameObject.h"

namespace eae6320 {
	class cHalo;
}

namespace eae6320 {
	class CubeSpawner : public eae6320::GameCommon::GameObject {
	public:
		CubeSpawner(Effect * i_pEffect, eae6320::Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State) :
			GameCommon::GameObject(i_pEffect, i_Mesh, i_State)
		{

		}
		CubeSpawner(CubeSpawner & i_other) :
			GameCommon::GameObject(i_other.GetEffect(), i_other.GetMesh(), i_other.m_State)
		{

		}
		~CubeSpawner() override;
		void UpdateGameObjectBasedOnInput() override;
		void Tick(const float i_secondCountToIntegrate) override;

	};
}
