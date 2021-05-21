// Includes
//=========
//12/13/2018
#include "cHalo.h"

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
#include "Custom Game Objects/Ground.h"
#include "Halo/Custom Game Objects/Cloth.h"
#include "Engine/Profiling/Profiling.h"
//#include "Halo/Custom Game Objects/SoftShell.h"
#include "Halo/Custom Game Objects/MoveableCube.h"
#include "Halo/Custom Game Objects/MPM.h"

// Inherited Implementation
//=========================

// Run
//----

void eae6320::cHalo::UpdateBasedOnInput()
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

eae6320::cResult eae6320::cHalo::Initialize()
{
	//initialize camera 
	mainCamera.Initialize(Math::sVector(2.5f, 6.0f, 12.5f), Math::sVector(-30.0f, 0.0f, 0.0f), Math::ConvertDegreesToRadians(45), 1.0f, 0.1f, 500.0f);
	//mainCamera.Initialize(Math::sVector(5.0f, 10.0f, 15.0f), Math::sVector(-30.0f, 20.0f, 0.0f), Math::ConvertDegreesToRadians(45), 1.0f, 0.1f, 500.0f);

	//create two meshes 	
	eae6320::Assets::cHandle<Mesh> mesh_plane;
	eae6320::Assets::cHandle<Mesh> mesh_point;

	auto result = eae6320::Results::Success;
	if (!(result = Mesh::s_manager.Load("data/meshes/square_plane.mesh", mesh_plane))) {
		EAE6320_ASSERT(false);
	}
	if (!(result = Mesh::s_manager.Load("data/meshes/material_point.mesh", mesh_point))) {
		EAE6320_ASSERT(false);
	}

	masterMeshArray.push_back(mesh_plane);
	masterMeshArray.push_back(mesh_point);
	
	//load effect
	Effect* pDefaultEffect;
	Effect::Load("data/effects/default.effect", pDefaultEffect);
	masterEffectArray.push_back(pDefaultEffect);

	//MPM simulator
	int n_particles = 0;
	MPM *MPMSim;
	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(2.5f, 0.0f, 2.5f);
		MPMSim = new MPM(pDefaultEffect, mesh_plane, objState, GetSimulationUpdatePeriod_inSeconds());
		n_particles = MPMSim->n_particles;
		noColliderObjects.push_back(MPMSim);
	}
	
	for (int i = 0; i < n_particles; i++)
	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, 0.0f, 0.0f);
		GameCommon::GameObject *pGameObject = new GameCommon::GameObject(pDefaultEffect, mesh_point, objState);
		pGameObject->m_color = Math::sVector(0.12f, 0.56f, 1.0f);
		noColliderObjects.push_back(pGameObject);
		MPMSim->fluid[i] = pGameObject;
	}

	//EnableConsolePrinting(true);
	return Results::Success;
}

void eae6320::cHalo::UpdateSimulationBasedOnInput() {
	if (isGameOver == false)
	{
		cbApplication::UpdateSimulationBasedOnInput();
	}
}

void  eae6320::cHalo::UpdateSimulationBasedOnTime(const float i_elapsedSecondCount_sinceLastUpdate) {
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


eae6320::cResult eae6320::cHalo::CleanUp()
{
	/*
	UserOutput::DebugPrint("Cloth Simulation(%d times) : %f ticks(%f s)",
		g_Profiler.m_Allccumulators[0]->m_Count,
		g_Profiler.m_Allccumulators[0]->average(),
		g_Profiler.m_Allccumulators[0]->getAverageTime());*/
	cbApplication::CleanUp();
	return Results::Success;
}

