#include "Car.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/Functions.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/Math/sVector.h"
void eae6320::Car::UpdateGameObjectBasedOnInput() {
	
	Physics::allHingeJoints[backLeftJointIndex].w_motor_B2A = 0.0f;
	Physics::allHingeJoints[backRightJointIndex].w_motor_B2A = 0.0f;
	
	if (UserInput::KeyState::currFrameKeyState[UserInput::KeyCodes::Up])
	{
		Physics::allHingeJoints[backLeftJointIndex].w_motor_B2A = 10.0f;
		Physics::allHingeJoints[backRightJointIndex].w_motor_B2A = 10.0f;
	}
	else if (UserInput::KeyState::currFrameKeyState[UserInput::KeyCodes::Down])
	{
		Physics::allHingeJoints[backLeftJointIndex].w_motor_B2A = -10.0f;
		Physics::allHingeJoints[backRightJointIndex].w_motor_B2A = -10.0f;
	}
	
}
void eae6320::Car::Tick(const float i_secondCountToIntegrate)
{
	//m_State.euler_x = m_State.euler_x + axis_X_velocity * i_secondCountToIntegrate;
	//m_State.euler_y = m_State.euler_y + axis_Y_velocity * i_secondCountToIntegrate;
}
eae6320::Car::~Car() {}