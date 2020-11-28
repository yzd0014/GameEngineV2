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
			objState.collider.m_type = Box;
			objState.position = m_State.position;
			objState.orientation = Math::cQuaternion(Math::ConvertDegreesToRadians(5), Math::sVector(0, 0, 1));
			//objState.velocity = Math::sVector(0.0f, -4.0f, 0.0f);
			objState.hasGravity = true;
			GameCommon::GameObject * pGameObject = new GameCommon::GameObject(m_Halo->masterEffectArray[0], m_Halo->masterMeshArray[1], objState);
			m_Halo->colliderObjects.push_back(pGameObject);
		}
	}
	else if (UserInput::IsKeyEdgeTriggered(UserInput::KeyCodes::F))
	{
		Physics::sRigidBodyState objState;
		objState.position = m_State.position;
		//objState.angularVelocity = Math::sVector(0.0f, 10.0f, -10.0f);
		objState.hasGravity = true;
		objState.mass = 1;
		float r = 1.0f;
		objState.collider.m_type = Sphere;
		objState.collider.m_vertices.push_back(Math::sVector(0.0f, 0.0f, 0.0f));
		objState.collider.m_vertices.push_back(Math::sVector(r, 0.0f, 0.0f));
		objState.localInverseInertiaTensor.m_00 = 1.0f / ((2.0f / 5.0f)* r * r);
		objState.localInverseInertiaTensor.m_11 = 1.0f / ((2.0f / 5.0f)* r * r);
		objState.localInverseInertiaTensor.m_22 = 1.0f / ((2.0f / 5.0f)* r * r);
		Math::cMatrix_transformation local2WorldRot(objState.orientation, Math::sVector(0, 0, 0));
		Math::cMatrix_transformation world2LocalRot = Math::cMatrix_transformation::CreateWorldToCameraTransform(local2WorldRot);
		objState.globalInverseInertiaTensor = local2WorldRot * objState.localInverseInertiaTensor * world2LocalRot;

		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(m_Halo->masterEffectArray[0], m_Halo->masterMeshArray[3], objState);
		m_Halo->colliderObjects.push_back(pGameObject);
	}
}
void eae6320::CubeSpawner::Tick(const float i_secondCountToIntegrate)
{
	//m_State.euler_x = m_State.euler_x + axis_X_velocity * i_secondCountToIntegrate;
	//m_State.euler_y = m_State.euler_y + axis_Y_velocity * i_secondCountToIntegrate;
}
eae6320::CubeSpawner::~CubeSpawner() {}