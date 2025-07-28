#pragma once
#include "GameObject.h"

namespace sca2025 
{
	class Ground : public sca2025::GameCommon::GameObject 
	{
	public:
		Ground(Effect * i_pEffect, sca2025::Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State) :
			GameCommon::GameObject(i_pEffect, i_Mesh, i_State)
		{
			m_State.mass = 1000000;
			m_State.localInverseInertiaTensor.m_00 = 1.0f/((1.0f / 12.0f)*m_State.mass * 128);
			m_State.localInverseInertiaTensor.m_11 = 1.0f/((1.0f / 12.0f)*m_State.mass * 128);
			m_State.localInverseInertiaTensor.m_22 = 1.0f/((1.0f / 12.0f)*m_State.mass * 128);
			Math::cMatrix_transformation local2WorldRot(m_State.orientation, Math::sVector(0, 0, 0));
			Math::cMatrix_transformation world2LocalRot = Math::cMatrix_transformation::CreateWorldToCameraTransform(local2WorldRot);
			m_State.globalInverseInertiaTensor = local2WorldRot * m_State.localInverseInertiaTensor * world2LocalRot;
			m_State.isStatic = true;
			
		}
	};
}