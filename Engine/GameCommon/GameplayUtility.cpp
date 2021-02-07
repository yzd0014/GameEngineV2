#include "Engine/UserInput/UserInput.h"
#include "Engine/Math/cMatrix_transformation.h"
#include "Engine/Application/cbApplication.h"
#include "Camera.h"
#include "GameplayUtility.h"
#include <cmath>

namespace eae6320
{
	namespace GameplayUtility
	{
		Application::cbApplication* pGameApplication = nullptr;
		
		Math::sVector MouseRayCasting()//return a ray direction in word sapce 
		{
			int mouse_x = 0, mouse_y = 0;
			UserInput::GetCursorPositionInWindow(&mouse_x, &mouse_y);
			uint16_t windowWidth = 0, windowHeight = 0;
			pGameApplication->GetCurrentResolution(windowWidth, windowHeight);
			
			float cameraSpace_x = 0.0f, cameraSpace_y = 0.0f;
			cameraSpace_x = mouse_x / (windowWidth * 0.5f) - 1.0f;
			cameraSpace_y = 1.0f - mouse_y / (windowHeight * 0.5f);
			cameraSpace_x = cameraSpace_x * std::tan(mainCamera.m_verticalFieldOfView_inRadians * 0.5f) * mainCamera.m_aspectRatio;
			cameraSpace_y = cameraSpace_y * std::tan(mainCamera.m_verticalFieldOfView_inRadians * 0.5f);

			Math::sVector p(cameraSpace_x, cameraSpace_y, -1.0f);
			p.Normalize();
			Math::cMatrix_transformation local2World(mainCamera.orientation, Math::sVector(0.0f, 0.0f, 0.0f));
			Math::sVector output;
			output = local2World * p;

			return output;
		}
	}
}