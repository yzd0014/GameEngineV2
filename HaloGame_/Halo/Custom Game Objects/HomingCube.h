#pragma once
#include "Engine/GameCommon/GameObject.h"
#include "Engine/Time/Time.h"
namespace eae6320 {
	class HomingCube : public eae6320::GameCommon::GameObject {
	public:
		HomingCube(Effect * i_pEffect, eae6320::Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State) :
			GameCommon::GameObject(i_pEffect, i_Mesh, i_State)
		{
			bornTime = (float)Time::ConvertTicksToSeconds(Time::GetCurrentSystemTimeTickCount());
		}
		HomingCube(HomingCube & i_other):
			GameCommon::GameObject(i_other.GetEffect(), i_other.GetMesh(), i_other.m_State)
		{
			bornTime = (float)Time::ConvertTicksToSeconds(Time::GetCurrentSystemTimeTickCount());
		}
		~HomingCube() override;
		void Tick(const float i_secondCountToIntegrate) override;
		void OnHit(GameObject * i_pObjectHit) override;
		
		GameCommon::GameObject * m_target;
		float bornTime;
	};
}
