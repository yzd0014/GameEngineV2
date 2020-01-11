#include "MoveableCube.h"
#include "Engine/Math/Functions.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/Math/sVector.h"
void eae6320::MoveableCube::UpdateGameObjectBasedOnInput() {
	//gameObject movement
	//reset to defualt velocity
	m_State.velocity = Math::sVector(0, 0, 0);
	axis_X_velocity = 0.0f;
	axis_Y_velocity = 0.0f;
	//m_State.axis_Z_velocity = 0.0f;
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::H))
	{
		axis_Y_velocity = 400;
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::K))
	{
		axis_Y_velocity = -400;
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::U))
	{
		axis_X_velocity = 400;
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::J))
	{
		axis_X_velocity = -400;
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
void eae6320::MoveableCube::EventTick(const float i_secondCountToIntegrate)
{
	m_State.euler_x = m_State.euler_x + axis_X_velocity * i_secondCountToIntegrate;
	m_State.euler_y = m_State.euler_y + axis_Y_velocity * i_secondCountToIntegrate;
}
eae6320::MoveableCube::~MoveableCube() {}