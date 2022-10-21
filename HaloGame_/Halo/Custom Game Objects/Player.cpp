#include "Halo/cHalo.h"
#include "Player.h"
#include "Bullet.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/Math/sVector.h"
#include "Engine/UserOutput/UserOutput.h"
#include "Engine/Time/Time.h"
void eae6320::Player::Tick(const float i_secondCountToIntegrate) {
	//apply gravity
	if (!isOnGround) {
		m_State.acceleration = Math::sVector(0, -8, 0);
	}
	else {
		m_State.acceleration = Math::sVector(0, 0, 0);
	}
	if (m_State.velocity.y > 0) {
		isOnGround = false;
	}

	//read input from mouse about rotation
	float sensitvity = 0.2f;
	int mouseX, mouseY;
	UserInput::GetMouseMoveDistanceInDeltaTime(&mouseX, &mouseY);
	float axis_Y_velo = -1 * mouseX * sensitvity / 0.067f;
	float axis_X_velo = -1 * mouseY * sensitvity / 0.067f;

	//player rotation
	m_State.axis_X_velocity = 0.0f;
	m_State.axis_Y_velocity = 0.0f;
	m_State.axis_Z_velocity = 0.0f;

	m_State.axis_Y_velocity = axis_Y_velo;

	//x rotation for gun and camera
	//clamp x axis angle
	m_Gun->m_State.axis_X_velocity = 0;
	m_Camera->m_State.axis_X_velocity = 0;
	if (axis_X_velo > 0 && (m_Gun->m_State.euler_x < 90 || m_Gun->m_State.euler_x > 180)) {
		m_Gun->m_State.axis_X_velocity = axis_X_velo;
		m_Camera->m_State.axis_X_velocity = axis_X_velo;
	}
	if (axis_X_velo < 0 && (m_Gun->m_State.euler_x > 270 || m_Gun->m_State.euler_x < 180)) {
		m_Gun->m_State.axis_X_velocity = axis_X_velo;
		m_Camera->m_State.axis_X_velocity = axis_X_velo;
	}
	if (m_Gun->m_State.euler_x > 90 && m_Gun->m_State.euler_x < 180) {
		m_Gun->m_State.euler_x = 90;
		m_Camera->m_State.euler_x = 90;
	}
	if (m_Gun->m_State.euler_x < 270 && m_Gun->m_State.euler_x > 180) {
		m_Gun->m_State.euler_x = 270;
		m_Camera->m_State.euler_x = 270;
	}
	//m_Gun->m_State.UpdateOrientation(0);
	//m_Camera->m_State.UpdateOrientation(0);
}

void eae6320::Player::OnHit(GameObject * i_pObjectHit) {
	char * nameExpected_0 = { "Ground" };
	if (strcmp(i_pObjectHit->objectType, nameExpected_0) == 0) {
		m_State.velocity.y = 0;
		m_State.acceleration = Math::sVector(0, 0, 0);
		isOnGround = true;
	}
	
	char * nameExpected_1 = { "Missile" };
	if (strcmp(i_pObjectHit->objectType, nameExpected_1) == 0 && isActive() && i_pObjectHit->isActive()) {
		m_Halo->isGameOver = true;
		m_Halo->soundArray[0]->Stop();
	}
}

void eae6320::Player::UpdateGameObjectBasedOnInput() {
	//Jumping
	/*
	if (UserInput::IsKeyEdgeTriggered(UserInput::KeyCodes::Space)) {
		if (isOnGround) {
			m_State.velocity.y = 8.2f;
			isOnGround = false;
		}
	}
	*/
	
	//player movement
	m_State.velocity = Math::sVector(0, m_State.velocity.y, 0);

	Math::cMatrix_transformation localToWorldMat = Math::cMatrix_transformation::cMatrix_transformation(m_State.orientation, m_State.position);
	Math::sVector forwardVector = localToWorldMat.GetBackDirection();
	forwardVector.Normalize();
	forwardVector = forwardVector * -10;

	Math::sVector rightVector = localToWorldMat.GetRightDirection();
	rightVector.Normalize();
	rightVector = rightVector * 10;

	if (UserInput::IsKeyPressed(UserInput::KeyCodes::D))
	{
		m_State.velocity += rightVector;
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::A))
	{
		m_State.velocity += -1 * rightVector;
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::W))
	{
		m_State.velocity += forwardVector;
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::S))
	{
		m_State.velocity += forwardVector * -1;
	}
	
	//pass state to gun and camera
	m_Gun->m_State.position = m_State.position;
	m_Gun->m_State.velocity = m_State.velocity;

	m_Gun->m_State.euler_y = m_State.euler_y;
	m_Gun->m_State.euler_z = m_State.euler_z;
	m_Gun->m_State.axis_Y_velocity = m_State.axis_Y_velocity;
	m_Gun->m_State.axis_Z_velocity = m_State.axis_Z_velocity;
	
	//m_Camera->m_State = m_State;
	m_Camera->m_State.position = m_State.position;
	m_Camera->m_State.velocity = m_State.velocity;

	m_Camera->m_State.euler_y = m_State.euler_y;
	m_Camera->m_State.euler_z = m_State.euler_z;
	m_Camera->m_State.axis_Y_velocity = m_State.axis_Y_velocity;
	m_Camera->m_State.axis_Z_velocity = m_State.axis_Z_velocity;
	
	//Shooting
	if (UserInput::IsKeyFromReleasedToPressed(UserInput::KeyCodes::Space)) {

		Physics::sRigidBodyState objState;

		Math::sVector offset = Math::sVector(0.25f, -0.25f, -1.8f);
		Math::cMatrix_transformation local2World_rotation(m_Gun->m_State.orientation, Math::sVector(0.0f, 0.0f, 0.0f));
		offset = local2World_rotation * offset;

		objState.position = m_Gun->m_State.position + offset;
		objState.euler_x = m_Gun->m_State.euler_x;
		objState.euler_y = m_Gun->m_State.euler_y;
		objState.euler_z = m_Gun->m_State.euler_z;
		objState.UpdateOrientation(0);
		objState.boundingBox.center = Math::sVector(0.0f, 0.0f, 0.0f);
		objState.boundingBox.extends = Math::sVector(0.05f, 0.05f, 0.05f);

		Math::sVector velocity = Math::sVector(0.0f, 0.0f, -30.0f);
		velocity = local2World_rotation * velocity;
		objState.velocity = velocity;

		Bullet * pGameObject = new Bullet(m_Halo->masterEffectArray[1], m_Halo->masterMeshArray[4], objState, m_Halo);
		m_Halo->masterGameObjectArr.push_back(pGameObject);

	}
}

eae6320::Player::~Player() {
}