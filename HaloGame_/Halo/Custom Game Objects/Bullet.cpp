#include "Bullet.h"
#include "Halo/cHalo.h"
void eae6320::Bullet::EventTick(const float i_secondCountToIntegrate) {
	float currentTime = (float)Time::ConvertTicksToSeconds(Time::GetCurrentSystemTimeTickCount());
	if (currentTime - bornTime > 2.0f) {
		DestroyGameObject();
	}
}

void eae6320::Bullet::EventHit(GameObject * i_pObjectHit) {
	char * nameExpected = { "Missile" };
	if (strcmp(i_pObjectHit->objectType, nameExpected) == 0 && isActive() && i_pObjectHit->isActive() && i_pObjectHit->m_State.position.z > -9) {
		i_pObjectHit->DestroyGameObject();
		m_Halo->score++;
		DestroyGameObject();
	}
}