#include "Engine/GameCommon/GameObject.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/cMatrix_transformation.h"
#include "PhysicsSimulation.h"
#include "CollisionPair.h"
#include "CollisionDetection.h"
#include "CollisionDetection.h"
#include "Engine/UserOutput/UserOutput.h"

namespace eae6320 {
	namespace Physics {
		void RunPhysics(std::vector<GameCommon::GameObject *> & i_allGameObjects, std::vector<GameCommon::GameObject *> & i_debugGraphics, Assets::cHandle<Mesh> i_debugMesh, Effect* i_pDebugEffect, float i_dt)
		{
			for (size_t i = 0; i < i_debugGraphics.size(); i++)
			{
				delete i_debugGraphics[i];
			}
			i_debugGraphics.clear();
			
			//update collider transformation
			size_t count = i_allGameObjects.size();
			for (size_t i = 0; i < count; i++)
			{
				Math::cMatrix_transformation local2World(i_allGameObjects[i]->m_State.orientation, i_allGameObjects[i]->m_State.position);
				i_allGameObjects[i]->m_State.collider.UpdateTransformation(local2World);
			}
			for (size_t i = 0; i < count - 1; i++)
			{
				for (size_t j = i + 1; j < count; j++)
				{
					Contact contact;
					if (i_allGameObjects[i]->m_State.collider.IsCollided(i_allGameObjects[j]->m_State.collider, contact))
					{
						i_allGameObjects[i]->m_State.position += -contact.depth*contact.normal;
						//UserOutput::DebugPrint("%f, %f, %f", contact.contactNormal.x, contact.contactNormal.y, contact.contactNormal.z);

						{
							Physics::sRigidBodyState objState;
							objState.position = contact.globalPositionA;
							objState.euler_x = i_allGameObjects[i]->m_State.euler_x;
							objState.euler_y = i_allGameObjects[i]->m_State.euler_y;
							objState.euler_z = i_allGameObjects[i]->m_State.euler_z;
							GameCommon::GameObject * pGameObject = new GameCommon::GameObject(i_pDebugEffect, i_debugMesh, objState);
							i_debugGraphics.push_back(pGameObject);
						}
						
						{
							Physics::sRigidBodyState objState;
							objState.position = contact.globalPositionB;
							objState.euler_x = i_allGameObjects[j]->m_State.euler_x;
							objState.euler_y = i_allGameObjects[j]->m_State.euler_y;
							objState.euler_z = i_allGameObjects[j]->m_State.euler_z;
							GameCommon::GameObject * pGameObject = new GameCommon::GameObject(i_pDebugEffect, i_debugMesh, objState);
							i_debugGraphics.push_back(pGameObject);
						}
					}
				}
			}

			for (size_t i = 0; i < count; i++)
			{
				i_allGameObjects[i]->m_State.Update(i_dt);
			}
		}
		void MoveObjectsForward(std::vector<GameCommon::GameObject *> & o_allGameObjects, float timeSpan) {
			size_t numOfObjects = o_allGameObjects.size();
			for (size_t i = 0; i < numOfObjects; i++) {
				o_allGameObjects[i]->m_State.UpdatePosition(timeSpan);
			}
		}
		void ResolveCollision(CollisionPair & o_earliestCollision) {
			Math::sVector normal_A;
			Math::sVector normal_B;

			normal_A = o_earliestCollision.collisionNormal4A;
			normal_B = -1 * normal_A;

			Math::sVector v_reflectedA = abs(Math::Dot(-2 * o_earliestCollision.collisionObjects[0]->velocity, normal_A)) * normal_A - (-1 * o_earliestCollision.collisionObjects[0]->velocity);
			o_earliestCollision.collisionObjects[0]->velocity = v_reflectedA;

			Math::sVector v_reflectedB = abs(Math::Dot(-2 * o_earliestCollision.collisionObjects[1]->velocity, normal_B)) * normal_B - (-1 * o_earliestCollision.collisionObjects[1]->velocity);
			o_earliestCollision.collisionObjects[1]->velocity = v_reflectedB;

		}
		void PhysicsUpdate(std::vector<GameCommon::GameObject *> & o_allGameObjects, float i_dt) {
			float frameTime = i_dt;
			
			while (frameTime > 0) {
				CollisionPair earliestCollision;
				if (FindEarliestCollision(o_allGameObjects, frameTime, earliestCollision)) {
					MoveObjectsForward(o_allGameObjects, earliestCollision.collisionTime);
					ResolveCollision(earliestCollision);
					frameTime = frameTime - earliestCollision.collisionTime;
				}
				else {
					MoveObjectsForward(o_allGameObjects, earliestCollision.collisionTime);
					frameTime = 0;
				}
			}

			size_t numOfObjects = o_allGameObjects.size();
			for (size_t i = 0; i < numOfObjects; i++) {
				o_allGameObjects[i]->m_State.UpdateVelocity(i_dt);
				//o_allGameObjects[i]->m_State.UpdateOrientation(i_dt);
				//handle rotation collision		
				if (FindRotationCollision(o_allGameObjects, i_dt, i) == false) {
					o_allGameObjects[i]->m_State.UpdateOrientation(i_dt);
				}
			}
		}
		
	}
}