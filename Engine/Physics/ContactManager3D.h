#pragma once
#include "sRigidBodyState.h"
#include <vector>

namespace eae6320
{
	namespace Physics
	{
		void MergeContact(Contact& i_contact, ContactManifold3D& o_dest);
		float SqDistPointTriangle(Math::sVector& vPoint, Math::sVector& vA, Math::sVector& vB, Math::sVector& vC);
	}
}