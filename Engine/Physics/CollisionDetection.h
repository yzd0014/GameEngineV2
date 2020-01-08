#pragma once
#include <vector>
namespace eae6320 {
	namespace GameCommon {
		class GameObject;
	}
	namespace Math {
		struct sVector;
	}
	namespace Physics {
		struct sRigidBodyState;
		struct CollisionPair;
		bool  CollisionDetection(sRigidBodyState  & i_object_A, sRigidBodyState & i_object_B, float i_dt, Math::sVector &o_normal4A, float &o_collisionTime);
		bool AxisCheck(sRigidBodyState  & i_object_A, sRigidBodyState & i_object_B, Math::sVector & i_axis, float i_dt, float & o_maxCloseTime, float & o_minOpenTime, Math::sVector &o_normal4A);
		bool FindEarliestCollision(std::vector<GameCommon::GameObject *> & i_allGameObjects, float i_dt, CollisionPair & o_earliestCollision);
		bool FindRotationCollision(std::vector<GameCommon::GameObject *> & i_allGameObjects, float i_dt, size_t indexOfObjectChecked);
	}
}