#pragma once
#include "Engine/Physics/sRigidBodyState.h"
#include "Engine/Math/cMatrix_transformation.h"

namespace eae6320 {
	namespace GameCommon {
		class Camera {
		public:
			Camera() {}
			Camera(Math::sVector i_position, Math::sVector i_orientation, const float i_verticalFieldOfView_inRadians, const float i_aspectRatio, const float i_z_nearPlane, const float i_z_farPlane);
			void Initialize(Math::sVector i_position, Math::sVector i_orientation, const float i_verticalFieldOfView_inRadians, const float i_aspectRatio, const float i_z_nearPlane, const float i_z_farPlane);
			void UpdateState(const float i_secondCountToIntegrate);
			void UpdateCameraBasedOnInput();
			Math::cQuaternion PredictFutureOrientation(const float i_secondCountToExtrapolate) const;
			Math::sVector PredictFuturePosition(const float i_secondCountToExtrapolate) const;

			Math::cMatrix_transformation GetWorldToCameraMat();
			Math::cMatrix_transformation GetCameraToProjectedMat();

			//Physics::sRigidBodyState m_State;
			Math::sVector position;	
			Math::sVector velocity;	
			
			Math::cQuaternion orientation;
			Math::sVector orientationEuler;//Euler angle system in degrees. 
			float axis_X_velocity = 0.0f;
			float axis_Y_velocity = 0.0f;
		
			float mouseSensitvity = 0.2f;
			
			float m_verticalFieldOfView_inRadians;
			float m_aspectRatio;
			float m_z_nearPlane;
			float m_z_farPlane;
		};
	}
}


