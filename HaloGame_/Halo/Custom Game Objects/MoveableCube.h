#pragma once
#include "Engine/GameCommon/GameObject.h"

namespace eae6320 {
	class MoveableCube : public eae6320::GameCommon::GameObject {
	public:
		MoveableCube(Effect * i_pEffect, eae6320::Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State):
			GameCommon::GameObject(i_pEffect, i_Mesh, i_State)
		{

		}
		MoveableCube(MoveableCube & i_other):
			GameCommon::GameObject(i_other.GetEffect(), i_other.GetMesh(), i_other.m_State)
		{

		}
		~MoveableCube() override;
		void UpdateGameObjectBasedOnInput() override;
		void EventTick(const float i_secondCountToIntegrate) override;
	private:
		//float axis_Y_velocity = 0.0f;
		//float axis_X_velocity = 0.0f;
	};
}


