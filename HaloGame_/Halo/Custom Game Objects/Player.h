#pragma once
#include "Engine/GameCommon/GameObject.h"
#include "Engine/GameCommon/Camera.h"
namespace eae6320 {
	class cHalo;
}

namespace eae6320 {
	class Player : public eae6320::GameCommon::GameObject {
	public:
		Player(Effect * i_pEffect, eae6320::Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, cHalo * const i_Halo):
			GameCommon::GameObject(i_pEffect, i_Mesh, i_State),
			m_Halo(i_Halo)
		{
		}
		Player(Player & i_other) :
			GameCommon::GameObject(i_other.GetEffect(), i_other.GetMesh(), i_other.m_State),
			m_Halo(i_other.m_Halo)
		{

		}
		~Player() override;
		void Tick(const float i_secondCountToIntegrate) override;
		void OnHit(GameObject * i_pObjectHit) override;
		void UpdateGameObjectBasedOnInput() override;

		GameCommon::Camera * m_Camera;
		GameCommon::GameObject * m_Gun;
		cHalo * const m_Halo;
		bool isOnGround = false;
	};
}
