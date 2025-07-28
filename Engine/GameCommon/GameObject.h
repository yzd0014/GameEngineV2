#pragma once
#include "Engine/Graphics/Mesh.h"
#include "Engine/Graphics/Effect.h"
#include "Engine/Physics/sRigidBodyState.h"
#include "External/EigenLibrary/Eigen/Dense"

using namespace Eigen;
namespace sca2025 {
	namespace GameCommon
	{
		class GameObject;
	}

	extern std::vector<GameCommon::GameObject *> colliderObjects;//game objects with colliders
	extern std::vector<GameCommon::GameObject *> noColliderObjects;//game objects without colliders
	extern Concurrency::cMutex gameObjectArrayMutex;

	namespace GameCommon {
		class GameObject {
		public: 
			GameObject(){}
			GameObject(Effect * i_pEffect, sca2025::Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State) {//constructor
				if (i_pEffect != nullptr)
				{
					m_Mesh = i_Mesh;
					Mesh::s_manager.Get(m_Mesh)->IncrementReferenceCount();

					i_pEffect->IncrementReferenceCount();
					m_pEffect = i_pEffect;
				}
				
				m_State = i_State;
				m_State.collider.m_pParentRigidBody = &m_State;
				active = true;
				Math::cMatrix_transformation local2WorldRot(m_State.orientation, Math::sVector(0, 0, 0));
				Math::cMatrix_transformation world2LocalRot = Math::cMatrix_transformation::CreateWorldToCameraTransform(local2WorldRot);
				m_State.globalInverseInertiaTensor = local2WorldRot * m_State.localInverseInertiaTensor * world2LocalRot;
				
				gameObjectArrayMutex.Lock();
				if (i_State.collision)
				{
					colliderObjects.push_back(this);
				}
				else
				{
					noColliderObjects.push_back(this);
				}
				gameObjectArrayMutex.Unlock();
			}
			GameObject(const GameObject & i_other) {//copy constructor
				m_pEffect = i_other.GetEffect();
				m_pEffect->IncrementReferenceCount();

				m_Mesh = i_other.GetMesh();
				Mesh::s_manager.Get(m_Mesh)->IncrementReferenceCount();
				m_State = i_other.m_State;

				gameObjectArrayMutex.Lock();
				if (m_State.collision)
				{
					colliderObjects.push_back(this);
				}
				else
				{
					noColliderObjects.push_back(this);
				}
				gameObjectArrayMutex.Unlock();
			}
			GameObject & operator=(const GameObject &i_other) {//assignment operator
				Mesh::s_manager.Get(m_Mesh)->DecrementReferenceCount();
				m_pEffect->DecrementReferenceCount();

				m_pEffect = i_other.GetEffect();
				m_pEffect->IncrementReferenceCount();

				m_Mesh = i_other.GetMesh();
				Mesh::s_manager.Get(m_Mesh)->IncrementReferenceCount();
				m_State = i_other.m_State;

				return *this;
			}
			virtual ~GameObject() {
				if (m_pEffect != nullptr)
				{
					Mesh::s_manager.Get(m_Mesh)->DecrementReferenceCount();
					m_pEffect->DecrementReferenceCount();
				}
			}
			virtual void Tick(const float i_secondCountToIntegrate) {}//deprecated
			virtual void Tick(const double i_secondCountToIntegrate) {}
			virtual void OnHit(GameObject * i_pObjectHit) {}
			virtual void OnOverlap(GameObject * i_pObjectOverlapped) {}
			//In renderThreadNoWait mode, users are not allowed to add new game objects within UpdateGameObjectBasedOnInput
			virtual void UpdateGameObjectBasedOnInput() {}
			/*
			void UpdateState(const float i_secondCountToIntegrate) {
				m_State.Update(i_secondCountToIntegrate);
			}
			*/
			void SetEffect(Effect * i_pEffect) {
				m_pEffect->DecrementReferenceCount();
				i_pEffect->IncrementReferenceCount();
				m_pEffect = i_pEffect;
			}
			void SetMesh(sca2025::Assets::cHandle<Mesh> i_Mesh) {
				Mesh::s_manager.Get(m_Mesh)->DecrementReferenceCount();
				m_Mesh = i_Mesh;
				Mesh::s_manager.Get(m_Mesh)->IncrementReferenceCount();
			}
			Effect * GetEffect() const {
				return m_pEffect;
			}
			sca2025::Assets::cHandle<Mesh> GetMesh() const {
				return m_Mesh;
			}
			void DestroyGameObject() {
				active = false;
			}
			bool isActive() {
				return active;
			}
			Physics::sRigidBodyState m_State;
			Math::sVector m_color = Math::sVector(1, 1, 1);
			Vector3d scale = Vector3d(1, 1, 1);
			char objectType[20];
		private:
			sca2025::Assets::cHandle<Mesh> m_Mesh;
			Effect* m_pEffect = nullptr;
			bool active = true;
		};	
	
		class Camera;
		void RemoveInactiveGameObjects(std::vector<GameObject *> & o_allGameObjects);
		void ResetAllGameObjectsVelo(std::vector<GameObject *> & o_gameObjectsWithCollider, std::vector<GameObject *> & o_gameObjectsWithoutCollider, Camera & o_camera);
	}
}