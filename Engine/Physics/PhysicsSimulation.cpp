#include "Engine/GameCommon/GameObject.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/cMatrix_transformation.h"
#include "PhysicsSimulation.h"
#include "ContactManager3D.h"
#include "CollisionPair.h"
#include "CollisionDetection.h"
#include "Engine/UserOutput/UserOutput.h"
#include "CollisionResolver.h"
#include "Engine/Physics/HingeJoint.h"

namespace eae6320 {
	namespace Physics {
		std::vector<ContactManifold3D> allManifolds;
		std::vector<PointJoint> allPointJoints;
		std::vector<HingeJoint> allHingeJoints;

		void ConstraintResolver(float i_dt)
		{
			for (int k = 0; k < constraintMaxNum; k++)//resolve contrains for 10 iterations
			{
				CollisionResolver(i_dt, k);
				PointJointsResolver(i_dt);
				HingeJointsSolver(i_dt);
			}
		}

		void RunPhysics(std::vector<GameCommon::GameObject *> & i_colliderObjects, std::vector<GameCommon::GameObject *> & i_noColliderObjects, Assets::cHandle<Mesh> i_debugMesh, Effect* i_pDebugEffect, float i_dt)
		//void RunPhysics(std::vector<GameCommon::GameObject *> & i_colliderObjects, float i_dt)
		{
	
			for (size_t i = 2; i < i_noColliderObjects.size();)
			{
				delete i_noColliderObjects[i];
				i_noColliderObjects[i] = i_noColliderObjects.back();
				i_noColliderObjects.pop_back();
			}

			//update collider transformation and apply gravity
			int colliderCounts = static_cast<int>(i_colliderObjects.size());
			for (int i = 0; i < colliderCounts; i++)
			{
				Math::cMatrix_transformation local2World(i_colliderObjects[i]->m_State.orientation, i_colliderObjects[i]->m_State.position);
				Math::cMatrix_transformation local2WorldRot(i_colliderObjects[i]->m_State.orientation, Math::sVector(0, 0, 0));
				i_colliderObjects[i]->m_State.collider.UpdateTransformation(local2World, local2WorldRot);
				if (i_colliderObjects[i]->m_State.hasGravity && !i_colliderObjects[i]->m_State.isStatic)
				{
					i_colliderObjects[i]->m_State.velocity += Math::sVector(0.0f, -6.0f, 0.0f) * i_dt;
				}
			}
			for (size_t i = 0; i < i_noColliderObjects.size(); i++)
			{
				if (i_noColliderObjects[i]->m_State.hasGravity && !i_noColliderObjects[i]->m_State.isStatic)
				{
					i_noColliderObjects[i]->m_State.velocity += Math::sVector(0.0f, -6.0f, 0.0f) * i_dt;
				}
			}

			//collision detection
			for (int i = 0; i < colliderCounts - 1; i++)
			{
				for (int j = i + 1; j < colliderCounts; j++)
				{
					Contact contact;
					if (i_colliderObjects[i]->m_State.collider.IsCollided(i_colliderObjects[j]->m_State.collider, contact))
					{
						//add contact to correct manifold
						bool manifoldExist = false;
						for (size_t k = 0; k < allManifolds.size(); k++)
						{
							//ContactManifold3D* pManifold = i_colliderObjects[i]->m_State.collider.m_pManifolds[k];
							if ((allManifolds[k].m_contacts->colliderA == &i_colliderObjects[i]->m_State.collider && allManifolds[k].m_contacts->colliderB == &i_colliderObjects[j]->m_State.collider) ||
								(allManifolds[k].m_contacts->colliderA == &i_colliderObjects[j]->m_State.collider && allManifolds[k].m_contacts->colliderB == &i_colliderObjects[i]->m_State.collider))
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
							//i_colliderObjects[i]->m_State.collider.m_pManifolds.push_back(&allManifolds.back());
							//i_colliderObjects[j]->m_State.collider.m_pManifolds.push_back(&allManifolds.back());
						}
					}
					else
					{
						for (size_t k = 0; k < allManifolds.size(); k++)
						{
							//ContactManifold3D* pManifold = i_colliderObjects[i]->m_State.collider.m_pManifolds[k];
							if ((allManifolds[k].m_contacts->colliderA == &i_colliderObjects[i]->m_State.collider && allManifolds[k].m_contacts->colliderB == &i_colliderObjects[j]->m_State.collider) ||
								(allManifolds[k].m_contacts->colliderA == &i_colliderObjects[j]->m_State.collider && allManifolds[k].m_contacts->colliderB == &i_colliderObjects[i]->m_State.collider))
							{
								allManifolds[k] = allManifolds.back();
								allManifolds.pop_back();
								//allManifolds.shrink_to_fit();
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
						i_noColliderObjects.push_back(pGameObject);
					}

					{
						Physics::sRigidBodyState objState;
						objState.position = allManifolds[i].m_contacts[j].globalPositionB;
						objState.orientation = allManifolds[i].m_contacts[j].colliderB->m_pParentRigidBody->orientation;
						GameCommon::GameObject * pGameObject = new GameCommon::GameObject(i_pDebugEffect, i_debugMesh, objState);
						i_noColliderObjects.push_back(pGameObject);
					}
				}
			}

			//resolve collision
			ConstraintResolver(i_dt);

			//integration
			for (int i = 0; i < colliderCounts; i++)
			{
				i_colliderObjects[i]->m_State.Update(i_dt);
			}
			for (size_t i = 0; i < i_noColliderObjects.size(); i++)
			{
				if (!i_noColliderObjects[i]->m_State.isStatic)
				{
					i_noColliderObjects[i]->m_State.Update(i_dt);
				}
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