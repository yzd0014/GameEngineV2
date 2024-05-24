/*
	This struct can be used to represent the state of a
	rigid body in 3D space
*/

#ifndef EAE6320_PHYSICS_SRIGIDBODYSTATE_H
#define EAE6320_PHYSICS_SRIGIDBODYSTATE_H

// Includes
//=========

#include <vector>
#include <cfloat>
#include <Engine/Math/cQuaternion.h>
#include <Engine/Math/cMatrix_transformation.h>
#include <Engine/Math/sVector.h>
#include "External/EigenLibrary/Eigen/Dense"
#include "CollisionHelpers.h"

using namespace Eigen;
// Struct Declaration
//===================

namespace eae6320
{	
	namespace Physics
	{
		struct sRigidBodyState
		{
			//Physics property
			float mass;
			Math::cMatrix_transformation localInverseInertiaTensor;
			Math::cMatrix_transformation globalInverseInertiaTensor;
			
			// Data
			//=====

			Math::sVector position;	// In arbitrary units determined by the applicaton's convention
			Math::sVector velocity;	// Distance per-second
			Math::sVector acceleration;	// Distance per-second^2
			Math::cQuaternion orientation;
			//Math::sVector angularVelocity_axis_local = Math::sVector( 0.0f, 1.0f, 0.0f );	// In local space (not world space)
			//float angularSpeed = 0.0f;	// Radians per-second (positive values rotate right-handed, negative rotate left-handed)
			Math::sVector angularVelocity;
			Math::cMatrix_transformation transform;

			//float euler_x = 0.0f; //in degrees;
			//float euler_y = 0.0f;
			//float euler_z = 0.0f;

			//float axis_X_velocity = 0.0f;//degrees per second
			//float axis_Y_velocity = 0.0f;
			//float axis_Z_velocity = 0.0f;

			AABB boundingBox;
			Collider collider;
			bool movementInterpolation = false;
			bool isStatic = false;
			bool hasGravity = false;
			bool useTransform = false;
			// Interface
			//==========
			sRigidBodyState();
			sRigidBodyState(Math::sVector i_position);
			void Update( const float i_secondCountToIntegrate );
			void UpdatePosition(const float i_secondCountToIntegrate);
			void UpdateVelocity(const float i_secondCountToIntegrate);
			void UpdateOrientation(const float i_secondCountToIntegrate);
			Math::sVector PredictFuturePosition( const float i_secondCountToExtrapolate ) const;
			Math::cQuaternion PredictFutureOrientation( const float i_secondCountToExtrapolate ) const;
		};
	}
}

#endif	// EAE6320_PHYSICS_SRIGIDBODYSTATE_H
