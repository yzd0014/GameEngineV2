// Includes
//=========
//12/13/2018
#include "ClothSim.h"
#include "Cloth.h"

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
#include "Engine/Profiling/Profiling.h"
#include "Engine/GameCommon/Ground.h"
#include "Engine/GameCommon/MoveableCube.h"
// Inherited Implementation
//=========================

// Run
//----

void eae6320::ClothSim::UpdateBasedOnInput()
{
	// Is the user pressing the ESC key?
	if (UserInput::IsKeyPressed(UserInput::KeyCodes::Escape))
	{
		// Exit the application
		const auto result = Exit(EXIT_SUCCESS);
		EAE6320_ASSERT(result);
	}
}

// Initialization / Clean Up
//--------------------------

eae6320::cResult eae6320::ClothSim::Initialize()
{
	//EnableConsolePrinting(true);
	//initialize camera 
	mainCamera.Initialize(Math::sVector(0.0f, -1.0f, 15.0f), Math::sVector(0.0f, 0.0f, 0.0f), Math::ConvertDegreesToRadians(45), 1.0f, 0.1f, 500.0f);
	LOAD_MESH("data/meshes/plane.mesh", mesh_plane)
	LOAD_MESH("data/meshes/cloth10x10.mesh", mesh_cloth)
	LOAD_MESH("data/meshes/sphere4.mesh", mesh_sphere)
	//load effect
	LOAD_EFFECT("data/effects/default.effect", pDefaultEffect)
	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, -6.0f, -2.0f);
		MoveableCube* pGameObject = new MoveableCube(pDefaultEffect, mesh_sphere, objState);
		noColliderObjects.push_back(pGameObject);
	}
	//add cloth
	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, 0.0f, 0.0f);
		Cloth* pGameObject = new Cloth(pDefaultEffect, mesh_cloth, objState, GetSimulationUpdatePeriod_inSeconds());
		pGameObject->m_color = Math::sVector(1.0f, 0.0f, 0.0f);
		noColliderObjects.push_back(pGameObject);
	}

	//add ground mesh
	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, -11.0f, 0.0f);
		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(pDefaultEffect, mesh_plane, objState);
		strcpy_s(pGameObject->objectType, "Ground");
		noColliderObjects.push_back(pGameObject);
	}
	return Results::Success;
}

void eae6320::ClothSim::UpdateSimulationBasedOnInput() {
	if (isGameOver == false)
	{
		cbApplication::UpdateSimulationBasedOnInput();
	}
}

void  eae6320::ClothSim::UpdateSimulationBasedOnTime(const float i_elapsedSecondCount_sinceLastUpdate) {
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


eae6320::cResult eae6320::ClothSim::CleanUp()
{
	/*
	UserOutput::DebugPrint("Cloth Simulation(%d times) : %f ticks(%f s)",
		g_Profiler.m_Allccumulators[0]->m_Count,
		g_Profiler.m_Allccumulators[0]->average(),
		g_Profiler.m_Allccumulators[0]->getAverageTime());*/
	cbApplication::CleanUp();
	return Results::Success;
}