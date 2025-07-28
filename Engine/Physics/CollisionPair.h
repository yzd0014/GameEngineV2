#pragma once
#include "Engine/Math/sVector.h"
namespace sca2025 {
	namespace Physics {
		struct sRigidBodyState;
		
		struct CollisionPair {
			float collisionTime;
			Math::sVector collisionNormal4A;
			sRigidBodyState * collisionObjects[2];
		};
	}
}