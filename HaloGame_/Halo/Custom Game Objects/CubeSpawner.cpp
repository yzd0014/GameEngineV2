#include "Halo/cHalo.h"
#include "CubeSpawner.h"
#include "Engine/Math/Functions.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/Math/sVector.h"
void eae6320::CubeSpawner::UpdateGameObjectBasedOnInput() {
	//gameObject movement
	//reset to defualt velocity
	m_State.velocity = Math::sVector(0, 0, 0);
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::Right))
	{
		m_State.velocity = Math::sVector(5, 0, 0);
		//m_State.acceleration = Math::sVector(5, 0, 0);
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::Left))
	{
		m_State.velocity = Math::sVector(-5, 0, 0);
		//m_State.acceleration = Math::sVector(-5, 0, 0);
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::Up))
	{
		m_State.velocity = Math::sVector(0, 5, 0);
		//m_State.acceleration = Math::sVector(0, 5, 0);
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::Down))
	{
		m_State.velocity = Math::sVector(0, -5, 0);
		//m_State.acceleration = Math::sVector(0, -5, 0);
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::U)) {
		m_State.velocity = Math::sVector(0, 0, 5);
		//m_State.acceleration = Math::sVector(0, 0, 5);
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::J)) {
		m_State.velocity = Math::sVector(0, 0, -5);
		//m_State.acceleration = Math::sVector(0, 0, -5);
	}

	if (UserInput::IsKeyEdgeTriggered(UserInput::KeyCodes::Space)) 
	{
		{
			Physics::AABB boundingBox;
			boundingBox.center = Math::sVector(0.0f, 0.0f, 0.0f);
			boundingBox.extends = Math::sVector(1.0f, 1.0f, 1.0f);

			Physics::sRigidBodyState objState;
			objState.collider.InitializeCollider(boundingBox);
			objState.position = m_State.position;
			objState.orientation = Math::cQuaternion(Math::ConvertDegreesToRadians(5), Math::sVector(0, 0, 1));
			//objState.velocity = Math::sVector(0.0f, -4.0f, 0.0f);
			objState.hasGravity = true;
			GameCommon::GameObject * pGameObject = new GameCommon::GameObject(m_Halo->masterEffectArray[0], m_Halo->masterMeshArray[1], objState);
			m_Halo->masterGameObjectArr.push_back(pGameObject);
		}
	}
}
void eae6320::CubeSpawner::EventTick(const float i_secondCountToIntegrate)
{
	//m_State.euler_x = m_State.euler_x + axis_X_velocity * i_secondCountToIntegrate;
	//m_State.euler_y = m_State.euler_y + axis_Y_velocity * i_secondCountToIntegrate;
}
eae6320::CubeSpawner::~CubeSpawner() {}