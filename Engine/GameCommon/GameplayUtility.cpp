#include "Engine/UserInput/UserInput.h"
#include "Engine/Math/cMatrix_transformation.h"
#include "Engine/Application/cbApplication.h"
#include "Camera.h"
#include "GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include "Engine/Math/3DMathHelpers.h"
#include "Engine/Math/EigenHelper.h"

namespace sca2025
{
	namespace GameplayUtility
	{
		Application::cbApplication* pGameApplication = nullptr;
		sca2025::Assets::cHandle<Mesh> arrowMesh;
		
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

		GameCommon::GameObject* DrawArrow(Vector3d startPoint, Vector3d dir, Math::sVector color, double scaling)
		{
			return DrawArrowScaled(startPoint, dir, color, Vector3d(scaling, scaling, scaling));
		}

		GameCommon::GameObject* DrawArrowScaled(Vector3d startPoint, Vector3d dir, Math::sVector color, Vector3d scaling)
		{
			//get arrow transform
			dir.normalize();
			Vector3d defaultDir(0, 1, 0);
			double rotAngle = Math::GetAngleBetweenTwoVectors(defaultDir, dir);
			Vector3d rotVec = defaultDir.cross(dir);
			Matrix3d rotMatEigen;
			double eps = 0.000001;
			if (rotAngle > eps && rotAngle < M_PI - eps)
			{
				rotMatEigen = AngleAxisd(rotAngle, rotVec.normalized());
			}
			else if (rotAngle > M_PI - eps)
			{
				rotMatEigen = AngleAxisd(M_PI, Vector3d(0, 0, 1));
			}
			else
			{
				rotMatEigen.setIdentity();
			}
			//creat game object
			GameCommon::GameObject *pGameObject = new GameCommon::GameObject(defaultEffect, arrowMesh, Physics::sRigidBodyState());
			pGameObject->scale = scaling;
			pGameObject->m_State.position = Math::EigenVector2nativeVector(startPoint);
			pGameObject->m_State.orientation = Math::ConvertEigenQuatToNativeQuat(Math::RotationConversion_MatToQuat(rotMatEigen));
			pGameObject->m_color = color;

			return pGameObject;
		}

		void DrawXYZCoordinate(Vector3d pos)
		{
			DrawArrow(pos, Vector3d(1, 0, 0), Math::sVector(1.0, 0.0, 0.0), 0.5);
			DrawArrow(pos, Vector3d(0, 1, 0), Math::sVector(0.0, 1.0, 0.0), 0.5);
			DrawArrow(pos, Vector3d(0, 0, 1), Math::sVector(0.0, 0.0, 1.0), 0.5);
		}
	}
}