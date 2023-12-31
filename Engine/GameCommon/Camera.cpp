#include "Camera.h"
#include "Engine/Math/Functions.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/UserOutput/UserOutput.h"
#include <Engine/Asserts/Asserts.h>
#include "Engine/Time/Time.h"
#include "Engine/Graphics/Graphics.h"
#include "Engine/Physics/PhysicsSimulation.h"
namespace eae6320
{
	GameCommon::Camera mainCamera;
}

eae6320::GameCommon::Camera::Camera(Math::sVector i_position, Math::sVector i_orientation, const float i_verticalFieldOfView_inRadians, const float i_aspectRatio, const float i_z_nearPlane, const float i_z_farPlane) {
	position = i_position;
	orientationEuler = i_orientation;

	m_verticalFieldOfView_inRadians = i_verticalFieldOfView_inRadians;
	m_aspectRatio = i_aspectRatio;
	m_z_nearPlane = i_z_nearPlane;
	m_z_farPlane = i_z_farPlane;
}


void eae6320::GameCommon::Camera::Initialize(Math::sVector i_position, Math::sVector i_orientation, const float i_verticalFieldOfView_inRadians, const float i_aspectRatio, const float i_z_nearPlane, const float i_z_farPlane) {
	position = i_position;
	orientationEuler = i_orientation;

	m_verticalFieldOfView_inRadians = i_verticalFieldOfView_inRadians;
	m_aspectRatio = i_aspectRatio;
	m_z_nearPlane = i_z_nearPlane;
	m_z_farPlane = i_z_farPlane;

	tickCount_keyIsDown = 0;
}

void eae6320::GameCommon::Camera::UpdateState(const float i_secondCountToIntegrate) {
	//update position
	position += velocity * i_secondCountToIntegrate;
	if (!Graphics::renderThreadNoWait)
	{
		UpdateCameraOrientation(i_secondCountToIntegrate);
	}
	//upadte orientaion based on mouse input
	int mouseX, mouseY;
	UserInput::GetCursorDisplacementSinceLastCall(&mouseX, &mouseY);
	axis_X_velocity = 0.0f;
	axis_Y_velocity = 0.0f;
	if (UserInput::KeyState::currFrameKeyState[UserInput::KeyCodes::RightMouseButton])
	{
		UserInput::ConfineCursorWithinWindow();
		//update rotation velocity
		float axis_X_velo = -1 * mouseY * mouseSensitvity / i_secondCountToIntegrate;
		float axis_Y_velo = -1 * mouseX * mouseSensitvity / i_secondCountToIntegrate;
		
		axis_Y_velocity = axis_Y_velo;
		if (axis_X_velo > 0 && orientationEuler.x < 90) {
			axis_X_velocity = axis_X_velo;
		}
		if (axis_X_velo < 0 && orientationEuler.x > -90) {
			axis_X_velocity = axis_X_velo;
		}
	}

	if (Graphics::renderThreadNoWait)
	{
		UpdateCameraOrientation(i_secondCountToIntegrate);
	}
}

void eae6320::GameCommon::Camera::UpdateCameraOrientation(const float i_secondCountToIntegrate)
{
	//update orientation
	orientationEuler.x = orientationEuler.x + axis_X_velocity * i_secondCountToIntegrate;
	orientationEuler.y = orientationEuler.y + axis_Y_velocity * i_secondCountToIntegrate;
	if (orientationEuler.x > 90) {
		orientationEuler.x = 90;
	}
	if (orientationEuler.x < -90) {
		orientationEuler.x = -90;
	}
	//m_State.Update(i_secondCountToIntegrate);
	const auto rotation_x = Math::cQuaternion(Math::ConvertDegreesToRadians(orientationEuler.x), Math::sVector(1, 0, 0));
	const auto rotation_y = Math::cQuaternion(Math::ConvertDegreesToRadians(orientationEuler.y), Math::sVector(0, 1, 0));
	//const auto rotation_z = Math::cQuaternion(Math::ConvertDegreesToRadians(orientation.z), Math::sVector(0, 0, 1));
	orientation = rotation_y * rotation_x;
	orientation.Normalize();
}

eae6320::Math::cMatrix_transformation eae6320::GameCommon::Camera::GetWorldToCameraMat() {
	return Math::cMatrix_transformation::CreateWorldToCameraTransform(orientation, position);
}

eae6320::Math::cMatrix_transformation eae6320::GameCommon::Camera::GetCameraToProjectedMat() {
	return Math::cMatrix_transformation::CreateCameraToProjectedTransform_perspective(m_verticalFieldOfView_inRadians, m_aspectRatio, m_z_nearPlane, m_z_farPlane);
}

void eae6320::GameCommon::Camera::UpdateCameraBasedOnInput() {
	//reset velocity before update velocity
	velocity = Math::sVector(0, 0, 0);
	
	if (UserInput::KeyState::currFrameKeyState[UserInput::KeyCodes::RightMouseButton])
	{
		Math::cMatrix_transformation localToWorldMat = Math::cMatrix_transformation::cMatrix_transformation(orientation, position);
		Math::sVector forwardVector = localToWorldMat.GetBackDirection();
		forwardVector.Normalize();
		forwardVector = forwardVector * -10;

		Math::sVector rightVector = localToWorldMat.GetRightDirection();
		rightVector.Normalize();
		rightVector = rightVector * 10;

		if (UserInput::KeyState::currFrameKeyState['D'])
		{
			velocity = rightVector;
		}
		if (UserInput::KeyState::currFrameKeyState['A'])
		{
			velocity = -1 * rightVector;
		}
		if (UserInput::KeyState::currFrameKeyState['W'])
		{
			velocity = forwardVector;
		}
		if (UserInput::KeyState::currFrameKeyState['S'])
		{
			velocity = forwardVector * -1;
		}
	}

	if (Graphics::renderThreadNoWait)
	{
		if (UserInput::IsKeyFromReleasedToPressed(' '))
		{
			Physics::simPause = !Physics::simPause;
			if (Physics::simPause)
			{
				Physics::nextSimStep = false;
			}
		}
		if (UserInput::IsKeyFromReleasedToPressed('G'))
		{
			Physics::nextSimStep = true;
		}
		if (UserInput::KeyState::currFrameKeyState['G'])
		{
			tickCount_keyIsDown += Time::tickCount_elapsedSinceLastLoop;
		}
		auto secondCount_keyIsDown = Time::ConvertTicksToSeconds(tickCount_keyIsDown);
		if (secondCount_keyIsDown > 0.2)
		{
			Physics::simPlay = true;
		}
		if (UserInput::IsKeyFromPressedToReleased('G'))
		{
			tickCount_keyIsDown = 0;
			Physics::simPlay = false;
		}
	}
}

eae6320::Math::cQuaternion eae6320::GameCommon::Camera::PredictFutureOrientation(const float i_secondCountToExtrapolate) const
{
	auto rot_x = orientationEuler.x + axis_X_velocity * i_secondCountToExtrapolate;
	auto rot_y = orientationEuler.y + axis_Y_velocity * i_secondCountToExtrapolate;
	if (rot_x > 90) {
		rot_x = 90;
	}
	if (rot_x < -90) {
		rot_x = -90;
	}
	//const auto rotation = Math::cQuaternion( angularSpeed * i_secondCountToExtrapolate, angularVelocity_axis_local );
	const auto rotation_x = Math::cQuaternion(Math::ConvertDegreesToRadians(rot_x), Math::sVector(1, 0, 0));
	const auto rotation_y = Math::cQuaternion(Math::ConvertDegreesToRadians(rot_y), Math::sVector(0, 1, 0));

	const auto rotation = rotation_y * rotation_x;
	
	return rotation.GetNormalized();
}
eae6320::Math::sVector eae6320::GameCommon::Camera::PredictFuturePosition(const float i_secondCountToExtrapolate) const
{
	return position + (velocity * i_secondCountToExtrapolate);
}