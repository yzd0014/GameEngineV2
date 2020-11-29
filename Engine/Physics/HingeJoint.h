#pragma once
#include "Engine/GameCommon/GameObject.h"
#include <Engine/Math/sVector.h>
#include "Engine/EigenLibrary/Eigen/Dense"

using namespace Eigen;

namespace eae6320
{
	namespace Physics
	{
		class HingeJoint
		{
		public:
			HingeJoint(GameCommon::GameObject* i_pA, GameCommon::GameObject* i_pB, Math::sVector i_anchorA, Math::sVector i_anchorB,
				Math::sVector i_globalAxis)
			{
				pActorA = i_pA;
				pActorB = i_pB;
				anchorLocalA = i_anchorA;
				anchorLocalB = i_anchorB;

				Math::cMatrix_transformation local2WorldRotA(pActorA->m_State.orientation, Math::sVector(0, 0, 0));
				Math::cMatrix_transformation world2LocalRotA = Math::cMatrix_transformation::CreateWorldToCameraTransform(local2WorldRotA);
				axisLocaA = world2LocalRotA * i_globalAxis;

				Math::cMatrix_transformation local2WorldRotB(pActorB->m_State.orientation, Math::sVector(0, 0, 0));
				Math::cMatrix_transformation world2LocalRotB = Math::cMatrix_transformation::CreateWorldToCameraTransform(local2WorldRotB);
				axisLocaB = world2LocalRotB * i_globalAxis;
			}

			GameCommon::GameObject* pActorA;
			GameCommon::GameObject* pActorB;

			Math::sVector anchorLocalA;
			Math::sVector anchorLocalB;
			Math::sVector axisLocaA;
			Math::sVector axisLocaB;

			float w_motor_B2A = 0.0f;
			bool motorEnable = false;

			void ResolveHingJoint(float i_dt);
		};

		void HingeJointsSolver(float i_dt);
	}
}