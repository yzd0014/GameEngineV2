#pragma once
#include "Engine/GameCommon/GameObject.h"
#include <Engine/Math/sVector.h>

namespace sca2025
{
	namespace Physics
	{
		void PointJointsResolver(float i_dt);
		
		class PointJoint
		{
		public:
			GameCommon::GameObject* pGameObject;
			GameCommon::GameObject* pParentObject;
			Math::sVector anchor;
			Math::sVector extend;

			void ResolvePointJointConstrain(float i_dt);
		};
	}
}