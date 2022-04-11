// Includes
//=========

#include "sRigidBodyState.h"
#include "Engine/Math/Functions.h"
#include "Engine/UserOutput/UserOutput.h"
#include <Engine/Asserts/Asserts.h>

#define EPA_TOLERANCE 0.0001
#define EPA_MAX_NUM_FACES 64
#define EPA_MAX_NUM_LOOSE_EDGES 32
#define EPA_MAX_NUM_ITERATIONS 30
// Interface
//==========
eae6320::Physics::sRigidBodyState::sRigidBodyState()
{
	mass = 1.0f;
	//default value is for a 2x2x2 cube
	localInverseInertiaTensor.m_00 = 1.0f / ((1.0f / 12.0f)* mass * 8);
	localInverseInertiaTensor.m_11 = 1.0f / ((1.0f / 12.0f)* mass * 8);
	localInverseInertiaTensor.m_22 = 1.0f / ((1.0f / 12.0f)* mass * 8);
}

eae6320::Physics::sRigidBodyState::sRigidBodyState(Math::sVector i_position)
{
	mass = 1.0f;
	//default value is for a 2x2x2 cube
	localInverseInertiaTensor.m_00 = 1.0f / ((1.0f / 12.0f)* mass * 8);
	localInverseInertiaTensor.m_11 = 1.0f / ((1.0f / 12.0f)* mass * 8);
	localInverseInertiaTensor.m_22 = 1.0f / ((1.0f / 12.0f)* mass * 8);

	position = i_position;
}

void eae6320::Physics::sRigidBodyState::Update( const float i_secondCountToIntegrate )
{
	// Update velocity
	{
		velocity += acceleration * i_secondCountToIntegrate;
	}
	
	// Update position
	{
		position += velocity * i_secondCountToIntegrate;
	}

	// Update orientation
	{
		Math::cQuaternion deltaRot;
		if (angularVelocity.GetLength() > 0.000001f)
		{
			deltaRot = Math::cQuaternion(angularVelocity.GetLength()*i_secondCountToIntegrate, angularVelocity.GetNormalized());
		}
		orientation = deltaRot * orientation;
		orientation.Normalize();
		Math::cMatrix_transformation local2WorldRot(orientation, Math::sVector(0, 0, 0));
		Math::cMatrix_transformation world2LocalRot = Math::cMatrix_transformation::CreateWorldToCameraTransform(local2WorldRot);
		globalInverseInertiaTensor = local2WorldRot * localInverseInertiaTensor * world2LocalRot;
	}
}
void eae6320::Physics::sRigidBodyState::UpdatePosition(const float i_secondCountToIntegrate) {
	position += velocity * i_secondCountToIntegrate;
}

void eae6320::Physics::sRigidBodyState::UpdateVelocity(const float i_secondCountToIntegrate) {
	velocity += acceleration * i_secondCountToIntegrate;
}
void eae6320::Physics::sRigidBodyState::UpdateOrientation(const float i_secondCountToIntegrate) {
	Math::cQuaternion deltaRot;
	if (angularVelocity.GetLength() > 0.000001f)
	{
		deltaRot = Math::cQuaternion(angularVelocity.GetLength()*i_secondCountToIntegrate, angularVelocity.GetNormalized());
	}
	orientation = deltaRot * orientation;
	orientation.Normalize();
	Math::cMatrix_transformation local2WorldRot(orientation, Math::sVector(0, 0, 0));
	Math::cMatrix_transformation world2LocalRot = Math::cMatrix_transformation::CreateWorldToCameraTransform(local2WorldRot);
	globalInverseInertiaTensor = local2WorldRot * localInverseInertiaTensor * world2LocalRot;
}


eae6320::Math::sVector eae6320::Physics::sRigidBodyState::PredictFuturePosition( const float i_secondCountToExtrapolate ) const
{
	return position + (velocity * i_secondCountToExtrapolate );
}

eae6320::Math::cQuaternion eae6320::Physics::sRigidBodyState::PredictFutureOrientation(const float i_secondCountToExtrapolate) const
{
	//const auto rotation = Math::cQuaternion( angularSpeed * i_secondCountToExtrapolate, angularVelocity_axis_local );
	Math::cQuaternion deltaRot;
	if (angularVelocity.GetLength() > 0.000001f)
	{
		deltaRot = Math::cQuaternion(angularVelocity.GetLength()*i_secondCountToExtrapolate, angularVelocity.GetNormalized());
	}
	const auto rotation = deltaRot * orientation;

	return rotation.GetNormalized();
}
