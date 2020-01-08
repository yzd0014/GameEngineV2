#pragma once
#include "Engine/GameCommon/GameObject.h"
#include <vector>

namespace eae6320 {
	struct CollisionPair;
	
	namespace Physics {
		void RunPhysics(std::vector<GameCommon::GameObject *> & i_allGameObjects, std::vector<GameCommon::GameObject *> & i_debugGraphics, Assets::cHandle<Mesh> i_debugMesh, Effect* i_pDebugEffect, float i_dt);
		void MoveObjectsForward(std::vector<GameCommon::GameObject *> & o_allGameObjects, float timeSpan);
		void PhysicsUpdate(std::vector<GameCommon::GameObject *> & o_allGameObjects, float i_dt);
		void ResolveCollision(CollisionPair & o_earliestCollision);
	}
}
