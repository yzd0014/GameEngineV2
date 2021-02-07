#pragma once
#include "Engine/Math/sVector.h"

namespace eae6320
{
	namespace Application
	{
		class cbApplication;
	}
	
	namespace GameplayUtility
	{
		extern Application::cbApplication* pGameApplication;

		Math::sVector MouseRayCasting();
	}
}