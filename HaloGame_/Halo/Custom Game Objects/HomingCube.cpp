#include "HomingCube.h"
#include "Engine/Math/sVector.h"

eae6320::HomingCube::~HomingCube(){}
void eae6320::HomingCube::Tick(const float i_secondCountToIntegrate) {
	//homing
	Math::sVector moveDir = m_target->m_State.position - m_State.position;
	if (moveDir.GetLength() > 0.01f && m_target->m_State.position.z > m_State.euler_z) {
		moveDir.Normalize();
		m_State.acceleration = 4 * moveDir;
	}

	//destroy after a certain period of time
	float currentTime = (float)Time::ConvertTicksToSeconds(Time::GetCurrentSystemTimeTickCount());
	if (currentTime - bornTime > 5.0f) {
		DestroyGameObject();
	}
}

void eae6320::HomingCube::OnHit(GameObject * i_pObjectHit) {
	//char * nameExpected_0 = { "Player" };
	char * nameExpected_1 = { "BackWall" };
	/*
	if (strcmp(i_pObjectHit->objectType, nameExpected_0) == 0 && isActive() && i_pObjectHit->isActive()) {
		DestroyGameObject();
	}
	*/
	if (strcmp(i_pObjectHit->objectType, nameExpected_1) == 0 && isActive() && i_pObjectHit->isActive()) {
		DestroyGameObject();
	}
}