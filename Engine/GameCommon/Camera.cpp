#include "Camera.h"
#include "Engine/Math/Functions.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/UserOutput/UserOutput.h"
#include <Engine/Asserts/Asserts.h>

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
}

void eae6320::GameCommon::Camera::UpdateState(const float i_secondCountToIntegrate) {
	//update position
	position += velocity * i_secondCountToIntegrate;
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
	
	int mouseX, mouseY;
	UserInput::GetMouseMoveDistanceInDeltaTime(&mouseX, &mouseY);
	axis_X_velocity = 0.0f;
	axis_Y_velocity = 0.0f;
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::Space))
	{
		//update rotation velocity
		float axis_X_velo = -1 * mouseY * mouseSensitvity / i_secondCountToIntegrate;
		float axis_Y_velo = -1 * mouseX * mouseSensitvity / i_secondCountToIntegrate;

		axis_Y_velocity = axis_Y_velo;
		axis_X_velocity = axis_X_velo;

		if (axis_X_velo > 0 && orientationEuler.x < 90) {
			axis_X_velocity = axis_X_velo;
		}
		if (axis_X_velo < 0 && orientationEuler.x > -90) {
			axis_X_velocity = axis_X_velo;
		}
	}
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
	
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::Space))
	{
		Math::cMatrix_transformation localToWorldMat = Math::cMatrix_transformation::cMatrix_transformation(orientation, position);
		Math::sVector forwardVector = localToWorldMat.GetBackDirection();
		forwardVector.Normalize();
		forwardVector = forwardVector * -10;

		Math::sVector rightVector = localToWorldMat.GetRightDirection();
		rightVector.Normalize();
		rightVector = rightVector * 10;

		if (UserInput::IsKeyPressed(UserInput::KeyCodes::D))
		{
			velocity = rightVector;
		}
		if (UserInput::IsKeyPressed(UserInput::KeyCodes::A))
		{
			velocity = -1 * rightVector;
		}
		if (UserInput::IsKeyPressed(UserInput::KeyCodes::W))
		{
			velocity = forwardVector;
		}
		if (UserInput::IsKeyPressed(UserInput::KeyCodes::S))
		{
			velocity = forwardVector * -1;
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