#pragma once
#include "Engine/Math/sVector.h"

namespace sca2025
{
	namespace Application
	{
		class cbApplication;
	}
	
	namespace GameplayUtility
	{
		extern Application::cbApplication* pGameApplication;
		extern sca2025::Assets::cHandle<Mesh> arrowMesh;

		Math::sVector MouseRayCasting();
		GameCommon::GameObject* DrawArrow(Vector3d startPoint, Vector3d dir, Math::sVector color, double scaling);
		GameCommon::GameObject* DrawArrowScaled(Vector3d startPoint, Vector3d dir, Math::sVector color, Vector3d scaling);
		void DrawXYZCoordinate(Vector3d pos);
	}
}