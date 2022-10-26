// Includes
//=========
//12/13/2018
#include "SoftbodySim.h"
#include "JellyCube.h"

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
// Inherited Implementation
//=========================

// Run
//----

void eae6320::SoftbodySim::UpdateBasedOnInput()
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

eae6320::cResult eae6320::SoftbodySim::Initialize()
{
	//initialize camera 
	mainCamera.Initialize(Math::sVector(0.0f, 1.0f, 10.0f), Math::sVector(0.0f, 0.0f, 0.0f), Math::ConvertDegreesToRadians(45), 1.0f, 0.1f, 500.0f);

	//create two meshes
	LOAD_MESH("data/meshes/square_plane(8x8).mesh", mesh_plane)
	LOAD_MESH("data/meshes/fem_cube_5.mesh", mesh_cube)
	//load effect
	LOAD_EFFECT("data/effects/default.effect", pDefaultEffect)
	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, 0.0f, 0.0f);
		JellyCube* pGameObject = new JellyCube(pDefaultEffect, mesh_cube, objState, GetSimulationUpdatePeriod_inSeconds());
		noColliderObjects.push_back(pGameObject);
	}
	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, -5.0f, 0.0f);
		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(pDefaultEffect, mesh_plane, objState);
		strcpy_s(pGameObject->objectType, "Ground");
		noColliderObjects.push_back(pGameObject);
	}
	//EnableConsolePrinting(true);
	return Results::Success;
}

void eae6320::SoftbodySim::UpdateSimulationBasedOnInput() {
	if (isGameOver == false)
	{
		cbApplication::UpdateSimulationBasedOnInput();
	}
}

void  eae6320::SoftbodySim::UpdateSimulationBasedOnTime(const float i_elapsedSecondCount_sinceLastUpdate) {
	if (isGameOver == false)
	{
		cbApplication::UpdateSimulationBasedOnTime(i_elapsedSecondCount_sinceLastUpdate);
	}
	else
	{
		GameCommon::ResetAllGameObjectsVelo(colliderObjects, noColliderObjects, mainCamera);
	}
	GameCommon::RemoveInactiveGameObjects(colliderObjects);
}


eae6320::cResult eae6320::SoftbodySim::CleanUp()
{
	/*
	UserOutput::DebugPrint("Cloth Simulation(%d times) : %f ticks(%f s)",
		g_Profiler.m_Allccumulators[0]->m_Count,
		g_Profiler.m_Allccumulators[0]->average(),
		g_Profiler.m_Allccumulators[0]->getAverageTime());*/
	cbApplication::CleanUp();
	return Results::Success;
}