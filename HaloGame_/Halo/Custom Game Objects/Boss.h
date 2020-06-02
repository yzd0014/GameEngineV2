#pragma once
#include "Engine/GameCommon/GameObject.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/cMatrix_transformation.h"
#include "Engine/Math/cQuaternion.h"
namespace eae6320 {
	class cHalo;
}
namespace eae6320 {
	class Boss : public eae6320::GameCommon::GameObject {
	public:
		Boss(Effect * i_pEffect, eae6320::Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, cHalo * const i_Halo) :
			GameCommon::GameObject(i_pEffect, i_Mesh, i_State),
			m_Halo(i_Halo)
		{
			shootingInterval = 2.0f;
			missleVelocity = Math::sVector(0, 5, 5);

			Math::cQuaternion rotation4Velocity(0.54f, Math::sVector(0, 0, 1));
			rotationDuringInterval = Math::cMatrix_transformation(rotation4Velocity, Math::sVector(0, 0, 0));

			timeLastShot = 0;
			numOfMissleLaunched = 0;
		}
		Boss(Boss & i_other) :
			GameCommon::GameObject(i_other.GetEffect(), i_other.GetMesh(), i_other.m_State),
			m_Halo(i_other.m_Halo)
		{
			shootingInterval = 2.0f;
			missleVelocity = Math::sVector(0, 5, 5);

			Math::cQuaternion rotation4Velocity(0.54f, Math::sVector(0, 0, 1));
			rotationDuringInterval = Math::cMatrix_transformation(rotation4Velocity, Math::sVector(0, 0, 0));

			timeLastShot = 0;
			numOfMissleLaunched = 0;
		}
		
		void Tick(const float i_secondCountToIntegrate) override;

		cHalo * const m_Halo;
		float shootingInterval;
		float timeLastShot;
		uint8_t numOfMissleLaunched;
		Math::sVector missleVelocity;
		Math::cMatrix_transformation rotationDuringInterval;
	};
}
