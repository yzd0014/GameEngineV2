#include "Boss.h"
#include "HomingCube.h"
#include "Halo/cHalo.h"
void eae6320::Boss::EventTick(const float i_secondCountToIntegrate) {
	//UserOutput::DebugPrint("Number of objects: %d", numOfMissleLaunched);
	//Time::ConvertTicksToSeconds(Time::GetCurrentSystemTimeTickCount());
	if (timeLastShot == 0 || Time::ConvertTicksToSeconds(Time::GetCurrentSystemTimeTickCount()) - timeLastShot > shootingInterval) {
		timeLastShot = (float)Time::ConvertTicksToSeconds(Time::GetCurrentSystemTimeTickCount());

		Physics::sRigidBodyState objState;
		objState.boundingBox.center = Math::sVector(0.0f, 0.0f, 0.0f);
		objState.boundingBox.extends = Math::sVector(0.5f, 0.5f, 0.5f);
		objState.position = m_State.position;
		objState.axis_Z_velocity = 120;

		missleVelocity = rotationDuringInterval * missleVelocity;
		objState.velocity = missleVelocity;


		HomingCube * pGameObject = new HomingCube(m_Halo->masterEffectArray[4], m_Halo->masterMeshArray[8], objState);
		strcpy_s(pGameObject->objectType, "Missile");
		pGameObject->m_target = m_Halo->masterGameObjectArr[0];
		m_Halo->masterGameObjectArr.push_back(pGameObject);
		numOfMissleLaunched++;
	}
	//level 2
	if (numOfMissleLaunched > 10 && numOfMissleLaunched <= 30) {
		shootingInterval = 1.0f;
	}
	//levl 3
	else if (numOfMissleLaunched > 30 && numOfMissleLaunched <= 60) {
		shootingInterval = 0.5f;
	}
	//level 4
	else if (numOfMissleLaunched > 60) {
		shootingInterval = 0.15f;
	}
}