#pragma once
#include "Engine/Graphics/Mesh.h"
#include "Engine/Graphics/Effect.h"
#include "Engine/Physics/sRigidBodyState.h"

namespace eae6320 {
	namespace GameCommon {
		class GameObject {
		public: 
			GameObject(){}
			GameObject(Effect * i_pEffect, eae6320::Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State) {//constructor
				m_Mesh = i_Mesh;
				Mesh::s_manager.Get(m_Mesh)->IncrementReferenceCount();
				
				i_pEffect->IncrementReferenceCount();
				m_pEffect = i_pEffect;
				m_State = i_State;

				active = true;
			}
			GameObject(GameObject & i_other) {//copy constructor
				m_pEffect = i_other.GetEffect();
				m_pEffect->IncrementReferenceCount();

				m_Mesh = i_other.GetMesh();
				Mesh::s_manager.Get(m_Mesh)->IncrementReferenceCount();

				m_State = i_other.m_State;
			}
			GameObject & operator=(const GameObject &i_other) {//assignment operator
				Mesh::s_manager.Get(m_Mesh)->DecrementReferenceCount();
				m_pEffect->DecrementReferenceCount();

				m_pEffect = i_other.GetEffect();
				m_pEffect->IncrementReferenceCount();

				m_Mesh = i_other.GetMesh();
				Mesh::s_manager.Get(m_Mesh)->IncrementReferenceCount();
				m_State = i_other.m_State;
			}
			virtual ~GameObject() {
				Mesh::s_manager.Get(m_Mesh)->DecrementReferenceCount();
				m_pEffect->DecrementReferenceCount();
			}
			virtual void EventTick(const float i_secondCountToIntegrate) {}
			virtual void EventHit(GameObject * i_pObjectHit) {}
			virtual void EventOverlap(GameObject * i_pObjectOverlapped) {}
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
			void SetMesh(eae6320::Assets::cHandle<Mesh> i_Mesh) {
				Mesh::s_manager.Get(m_Mesh)->DecrementReferenceCount();
				m_Mesh = i_Mesh;
				Mesh::s_manager.Get(m_Mesh)->IncrementReferenceCount();
			}
			Effect * GetEffect() const {
				return m_pEffect;
			}
			eae6320::Assets::cHandle<Mesh> GetMesh() const {
				return m_Mesh;
			}
			void DestroyGameObject() {
				active = false;
			}
			bool isActive() {
				return active;
			}
			Physics::sRigidBodyState m_State;
			char objectType[20];
			bool movementInterpolation = false;
		private:
			eae6320::Assets::cHandle<Mesh> m_Mesh;
			Effect* m_pEffect;
			bool active;
			
		};	
	
		class Camera;
		void RemoveInactiveGameObjects(std::vector<GameObject *> & o_allGameObjects);
		void ResetAllGameObjectsVelo(std::vector<GameObject *> & o_gameObjectsWithCollider, std::vector<GameObject *> & o_gameObjectsWithoutCollider, Camera & o_camera);
	}
}