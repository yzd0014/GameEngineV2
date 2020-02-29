#include "Engine/GameCommon/GameObject.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/cMatrix_transformation.h"
#include "PhysicsSimulation.h"
#include "ContactManager3D.h"
#include "CollisionPair.h"
#include "CollisionDetection.h"
#include "Engine/UserOutput/UserOutput.h"
#include "CollisionResolver.h"

namespace eae6320 {
	namespace Physics {
		std::vector<ContactManifold3D> allManifolds;
		std::vector<PointJoint> allPointJoints;

		void RunPhysics(std::vector<GameCommon::GameObject *> & i_allGameObjects, std::vector<GameCommon::GameObject *> & i_debugGraphics, Assets::cHandle<Mesh> i_debugMesh, Effect* i_pDebugEffect, float i_dt)
		{
			for (size_t i = 2; i < i_debugGraphics.size(); i++)
			{
				delete i_debugGraphics[i];
				i_debugGraphics[i] = i_debugGraphics.back();
				i_debugGraphics.pop_back();
				i_debugGraphics.shrink_to_fit();
			}

			//update collider transformation and apply gravity
			int count = static_cast<int>(i_allGameObjects.size());
			for (int i = 0; i < count; i++)
			{
				Math::cMatrix_transformation local2World(i_allGameObjects[i]->m_State.orientation, i_allGameObjects[i]->m_State.position);
				i_allGameObjects[i]->m_State.collider.UpdateTransformation(local2World);
				if (i_allGameObjects[i]->m_State.hasGravity && !i_allGameObjects[i]->m_State.isStatic)
				{
					i_allGameObjects[i]->m_State.velocity += Math::sVector(0.0f, -6.0f, 0.0f) * i_dt;
				}
			}
			//collision detection
			for (int i = 0; i < count - 1; i++)
			{
				for (int j = i + 1; j < count; j++)
				{
					Contact contact;
					if (i_allGameObjects[i]->m_State.collider.IsCollided(i_allGameObjects[j]->m_State.collider, contact))
					{
						//add contact to correct manifold
						bool manifoldExist = false;
						for (size_t k = 0; k < allManifolds.size(); k++)
						{
							//ContactManifold3D* pManifold = i_allGameObjects[i]->m_State.collider.m_pManifolds[k];
							if ((allManifolds[k].m_contacts->colliderA == &i_allGameObjects[i]->m_State.collider && allManifolds[k].m_contacts->colliderB == &i_allGameObjects[j]->m_State.collider) ||
								(allManifolds[k].m_contacts->colliderA == &i_allGameObjects[j]->m_State.collider && allManifolds[k].m_contacts->colliderB == &i_allGameObjects[i]->m_State.collider))
							{
								manifoldExist = true;
								//merge contact
								MergeContact(contact, allManifolds[k]);
								break;
							}
						}
						if (!manifoldExist)
						{
							ContactManifold3D manifold;
							manifold.AddContact(contact);
							allManifolds.push_back(manifold);
							//i_allGameObjects[i]->m_State.collider.m_pManifolds.push_back(&allManifolds.back());
							//i_allGameObjects[j]->m_State.collider.m_pManifolds.push_back(&allManifolds.back());
						}
					}
					else
					{
						for (size_t k = 0; k < allManifolds.size(); k++)
						{
							//ContactManifold3D* pManifold = i_allGameObjects[i]->m_State.collider.m_pManifolds[k];
							if ((allManifolds[k].m_contacts->colliderA == &i_allGameObjects[i]->m_State.collider && allManifolds[k].m_contacts->colliderB == &i_allGameObjects[j]->m_State.collider) ||
								(allManifolds[k].m_contacts->colliderA == &i_allGameObjects[j]->m_State.collider && allManifolds[k].m_contacts->colliderB == &i_allGameObjects[i]->m_State.collider))
							{
								allManifolds[k] = allManifolds.back();
								allManifolds.pop_back();
								allManifolds.shrink_to_fit();
								break;
							}
						}
					}
				}
			}
			
			for (size_t i = 0; i < allManifolds.size(); i++)
			{
				for (int j = 0; j < allManifolds[i].numContacts; j++)
				{
					{
						Physics::sRigidBodyState objState;
						objState.position = allManifolds[i].m_contacts[j].globalPositionA;
						objState.orientation = allManifolds[i].m_contacts[j].colliderA->m_pParentRigidBody->orientation;
						GameCommon::GameObject * pGameObject = new GameCommon::GameObject(i_pDebugEffect, i_debugMesh, objState);
						i_debugGraphics.push_back(pGameObject);
					}

					{
						Physics::sRigidBodyState objState;
						objState.position = allManifolds[i].m_contacts[j].globalPositionB;
						objState.orientation = allManifolds[i].m_contacts[j].colliderB->m_pParentRigidBody->orientation;
						GameCommon::GameObject * pGameObject = new GameCommon::GameObject(i_pDebugEffect, i_debugMesh, objState);
						i_debugGraphics.push_back(pGameObject);
					}
				}
			}

			//resolve collision
			CollisionResolver(allManifolds, i_dt);
			PointJointsResolver(i_dt);

			//integration
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