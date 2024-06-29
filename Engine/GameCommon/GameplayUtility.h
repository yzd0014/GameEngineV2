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
		extern eae6320::Assets::cHandle<Mesh> arrowMesh;

		Math::sVector MouseRayCasting();
		GameCommon::GameObject* DrawArrow(Vector3d startPoint, Vector3d dir, Math::sVector color, double scaling);
		GameCommon::GameObject* DrawArrowScaled(Vector3d startPoint, Vector3d dir, Math::sVector color, Vector3d scaling);
		void DrawXYZCoordinate(Vector3d pos);
	}
}