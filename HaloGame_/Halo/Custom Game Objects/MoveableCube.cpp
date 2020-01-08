#include "MoveableCube.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/Math/sVector.h"
void eae6320::MoveableCube::UpdateGameObjectBasedOnInput() {
	//gameObject movement
	//reset to defualt velocity
	m_State.velocity = Math::sVector(0, 0, 0);
	m_State.axis_X_velocity = 0.0f;
	m_State.axis_Y_velocity = 0.0f;
	m_State.axis_Z_velocity = 0.0f;
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::H))
	{
		m_State.axis_Y_velocity = 400;
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::K))
	{
		m_State.axis_Y_velocity = -400;
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::U))
	{
		m_State.axis_X_velocity = 400;
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::J))
	{
		m_State.axis_X_velocity = -400;
	}

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
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::G)) {
		m_State.velocity = Math::sVector(0, 0, 5);
		//m_State.acceleration = Math::sVector(0, 0, 5);
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::T)) {
		m_State.velocity = Math::sVector(0, 0, -5);
		//m_State.acceleration = Math::sVector(0, 0, -5);
	}
}

eae6320::MoveableCube::~MoveableCube() {}