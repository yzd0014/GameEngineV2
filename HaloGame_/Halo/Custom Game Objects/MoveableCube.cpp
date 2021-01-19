#include "MoveableCube.h"
#include "Engine/Math/Functions.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/Math/sVector.h"
void eae6320::MoveableCube::UpdateGameObjectBasedOnInput() {
	//gameObject movement
	//reset to defualt velocity
	m_State.velocity = Math::sVector(0, 0, 0);
	Math::cMatrix_transformation localToWorldMat = Math::cMatrix_transformation::cMatrix_transformation(m_State.orientation, m_State.position);
	Math::sVector rightVector = localToWorldMat.GetRightDirection();
	rightVector.Normalize();
	m_State.angularVelocity = Math::sVector(0, 0, 0);
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::H))
	{
		m_State.angularVelocity = Math::sVector(0, 1, 0) * Math::ConvertDegreesToRadians(-400);
	}
	else if (UserInput::IsKeyPressed(UserInput::KeyCodes::K))
	{
		m_State.angularVelocity = Math::sVector(0, 1, 0) * Math::ConvertDegreesToRadians(400);
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::U))
	{
		m_State.angularVelocity = rightVector * Math::ConvertDegreesToRadians(400);
	}
	else if (UserInput::IsKeyPressed(UserInput::KeyCodes::J))
	{
		m_State.angularVelocity = rightVector * Math::ConvertDegreesToRadians(-400);
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
		m_State.velocity = Math::sVector(0, 0, -5);
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::Down))
	{
		m_State.velocity = Math::sVector(0, 0, 5);
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::G)) {
		m_State.velocity = Math::sVector(0, 5, 0);
		//m_State.acceleration = Math::sVector(0, 0, 5);
	}
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::T)) {
		m_State.velocity = Math::sVector(0, -5, 0);
		//m_State.acceleration = Math::sVector(0, 0, -5);
	}
}
void eae6320::MoveableCube::Tick(const float i_secondCountToIntegrate)
{
	//m_State.euler_x = m_State.euler_x + axis_X_velocity * i_secondCountToIntegrate;
	//m_State.euler_y = m_State.euler_y + axis_Y_velocity * i_secondCountToIntegrate;
}
eae6320::MoveableCube::~MoveableCube() {}