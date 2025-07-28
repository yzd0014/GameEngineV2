// Includes
//=========
//12/13/2018
#include "BallJointSim.h"

#include "Engine/Logging/Logging.h"
#include <Engine/Asserts/Asserts.h>
#include <Engine/UserInput/UserInput.h>
#include "Engine/Graphics/Graphics.h"
#include "Engine/Graphics/Mesh.h"
#include "Engine/Graphics/cRenderState.h"
#include "Engine/Math/Functions.h"
#include "Engine/Math/cMatrix_transformation.h"
#include "Engine/Math/cQuaternion.h"
#include "Engine/Math/sVector.h"
#include "Engine/UserOutput/UserOutput.h"
#include "Engine/Physics/CollisionDetection.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/GameCommon/Ground.h"
#include "Engine/Profiling/Profiling.h"
#include "Engine/GameCommon/GameplayUtility.h"
#include "BallJointSim.h"
#include "MultiBody.h"
// Inherited Implementation
//=========================

// Run
//----

void sca2025::BallJointSim::UpdateBasedOnInput()
{
	// Is the user pressing the ESC key?
	if (UserInput::KeyState::currFrameKeyState[UserInput::KeyCodes::Escape])
	{
		// Exit the application
		const auto result = Exit(EXIT_SUCCESS);
		EAE6320_ASSERT(result);
	}
}

// Initialization / Clean Up
//--------------------------

sca2025::cResult sca2025::BallJointSim::Initialize()
{
	GameplayUtility::DrawXYZCoordinate(Vector3d(0, -8, 0));
	//initialize camera 
	mainCamera.Initialize(Math::sVector(0.0f, 5.0f, 12.5f), Math::sVector(-30.0f, 0.0f, 0.0f), Math::ConvertDegreesToRadians(45), 1.0f, 0.1f, 500.0f);
	//mainCamera.Initialize(Math::sVector(5.0f, 10.0f, 15.0f), Math::sVector(-30.0f, 20.0f, 0.0f), Math::ConvertDegreesToRadians(45), 1.0f, 0.1f, 500.0f);

	//starting mesh indexing is 1
	LOAD_MESH("data/meshes/square_plane.mesh", mesh_plane)
	LOAD_MESH("data/meshes/bullet.mesh", mesh_anchor)
	LOAD_MESH("data/meshes/cube.mesh", mesh_cube)
	LOAD_MESH("data/meshes/capsule.mesh", mesh_capsule)

	{
		//cbApplication* pApp = this;
		MultiBody * pMultiBody = new MultiBody(defaultEffect, mesh_anchor, Physics::sRigidBodyState(Math::sVector(0, 0, 0)), this);
		pMultiBody->m_color = Math::sVector(1, 0, 0);
	}
	//Ground
	{
		Physics::sRigidBodyState objState(Math::sVector(0.0f, -10.0f, 0.0f));
		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(defaultEffect, mesh_plane, objState);
	}
	return Results::Success;
}

void sca2025::BallJointSim::UpdateSimulationBasedOnInput() {
	if (isGameOver == false)
	{
		cbApplication::UpdateSimulationBasedOnInput();
	}
}

void  sca2025::BallJointSim::UpdateSimulationBasedOnTime(const double i_elapsedSecondCount_sinceLastUpdate) {
	if (isGameOver == false)
	{
		cbApplication::UpdateSimulationBasedOnTime(i_elapsedSecondCount_sinceLastUpdate);
	}
	else
	{
		GameCommon::ResetAllGameObjectsVelo(colliderObjects, noColliderObjects, mainCamera);
	}
}


sca2025::cResult sca2025::BallJointSim::CleanUp()
{
	cbApplication::CleanUp();
	return Results::Success;
}