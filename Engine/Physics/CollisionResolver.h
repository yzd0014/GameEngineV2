#pragma once
#include "sRigidBodyState.h"
#include <vector>

namespace eae6320 
{
	namespace Physics 
	{
		void CollisionResolver(std::vector<ContactManifold3D>& i_allManifolds);
	}
}