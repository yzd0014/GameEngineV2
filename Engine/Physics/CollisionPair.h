#pragma once
#include "Engine/Math/sVector.h"
namespace eae6320 {
	namespace Physics {
		struct sRigidBodyState;
		
		struct CollisionPair {
			float collisionTime;
			Math::sVector collisionNormal4A;
			sRigidBodyState * collisionObjects[2];
		};
	}
}