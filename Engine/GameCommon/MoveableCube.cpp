#include "MoveableCube.h"
#include "Engine/Math/Functions.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/Math/sVector.h"
#include "Engine/GameCommon/GameplayUtility.h"
#include "Engine/Math/3DMathHelpers.h"
#include "Engine/GameCommon/Camera.h"

void sca2025::MoveableCube::UpdateGameObjectBasedOnInput() {
	//gameObject movement
	//reset to defualt velocity
	m_State.velocity = Math::sVector(0, 0, 0);
	Math::cMatrix_transformation localToWorldMat = Math::cMatrix_transformation::cMatrix_transformation(m_State.orientation, m_State.position);
	Math::sVector rightVector = localToWorldMat.GetRightDirection();
	rightVector.Normalize();
	m_State.angularVelocity = Math::sVector(0, 0, 0);
	if (UserInput::KeyState::currFrameKeyState['H'])
	{
		m_State.angularVelocity = Math::sVector(0, 1, 0) * Math::ConvertDegreesToRadians(-400);
	}
	else if (UserInput::KeyState::currFrameKeyState['K'])
	{
		m_State.angularVelocity = Math::sVector(0, 1, 0) * Math::ConvertDegreesToRadians(400);
	}
	if (UserInput::KeyState::currFrameKeyState['U'])
	{
		m_State.angularVelocity = rightVector * Math::ConvertDegreesToRadians(400);
	}
	else if (UserInput::KeyState::currFrameKeyState['J'])
	{
		m_State.angularVelocity = rightVector * Math::ConvertDegreesToRadians(-400);
	}

	if (UserInput::KeyState::currFrameKeyState[UserInput::KeyCodes::Right])
	{
		m_State.velocity = Math::sVector(5, 0, 0);
		//m_State.acceleration = Math::sVector(5, 0, 0);
	}
	if (UserInput::KeyState::currFrameKeyState[UserInput::KeyCodes::Left])
	{
		m_State.velocity = Math::sVector(-5, 0, 0);
		//m_State.acceleration = Math::sVector(-5, 0, 0);
	}
	if (UserInput::KeyState::currFrameKeyState[UserInput::KeyCodes::Up])
	{
		m_State.velocity = Math::sVector(0, 0, -5);
	}
	if (UserInput::KeyState::currFrameKeyState[UserInput::KeyCodes::Down])
	{
		m_State.velocity = Math::sVector(0, 0, 5);
	}
	if (UserInput::KeyState::currFrameKeyState['F']) {
		m_State.velocity = Math::sVector(0, -5, 0);
	}
	if (UserInput::KeyState::currFrameKeyState['R']) {
		m_State.velocity = Math::sVector(0, 5, 0);
	}
	/*
	
	if (UserInput::IsKeyEdgeTriggered(UserInput::KeyCodes::F))
	{
		int x, y;
		UserInput::GetCursorPositionInWindow(&x, &y);
		UserOutput::DebugPrint("%d, %d", x, y);
	}
	*/
	
	/*
	if (UserInput::IsKeyEdgeTriggered(UserInput::KeyCodes::Space))
	{
		Math::sVector dir_t = GameplayUtility::MouseRayCasting();
		Vector3f dir;
		Math::NativeVector2EigenVector(dir_t, dir);
		Vector3f cameraPos;
		Math::NativeVector2EigenVector(mainCamera.position, cameraPos);

		Vector3f intersect;
		intersect.setZero();
		Vector3f a(-10, -11, -10);
		Vector3f b(10, -11, -10);
		Vector3f c(10, -11, 10);
		bool found = Math::GetLineTriangleIntersection(cameraPos, dir, a, b, c, intersect);
		if (found)
		{
			Physics::sRigidBodyState objState;
			Math::sVector sIntersect = Math::EigenVector2nativeVector(intersect);
			objState.position = sIntersect;
			GameCommon::GameObject * pGameObject = new GameCommon::GameObject(masterEffectArray[0], masterMeshArray[4], objState);
			noColliderObjects.push_back(pGameObject);
		}

	}
	*/
	
}
void sca2025::MoveableCube::Tick(const float i_secondCountToIntegrate)
{
	//m_State.euler_x = m_State.euler_x + axis_X_velocity * i_secondCountToIntegrate;
	//m_State.euler_y = m_State.euler_y + axis_Y_velocity * i_secondCountToIntegrate;
}
sca2025::MoveableCube::~MoveableCube() {}