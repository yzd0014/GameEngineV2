#pragma once
#include "Engine/GameCommon/GameObject.h"
#include <Engine/Math/sVector.h>

namespace eae6320
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