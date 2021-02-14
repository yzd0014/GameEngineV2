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
//#include "Halo/Custom Game Objects/Paper.h"

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
	mainCamera.Initialize(Math::sVector(0.0f, -1.0f, 15.0f), Math::sVector(0.0f, 0.0f, 0.0f), Math::ConvertDegreesToRadians(45), 1.0f, 0.1f, 500.0f);
	//mainCamera.Initialize(Math::sVector(5.0f, 10.0f, 15.0f), Math::sVector(-30.0f, 20.0f, 0.0f), Math::ConvertDegreesToRadians(45), 1.0f, 0.1f, 500.0f);

	//create two meshes 	
	eae6320::Assets::cHandle<Mesh> mesh_plane;
	eae6320::Assets::cHandle<Mesh> mesh_cloth;
	eae6320::Assets::cHandle<Mesh> mesh_shell;
	eae6320::Assets::cHandle<Mesh> mesh_sphere;
	eae6320::Assets::cHandle<Mesh> mesh_smallCube;

	auto result = eae6320::Results::Success;
	if (!(result = Mesh::s_manager.Load("data/meshes/plane.mesh", mesh_plane))) {
		EAE6320_ASSERT(false);
	}
	if (!(result = Mesh::s_manager.Load("data/meshes/cloth10x10.mesh", mesh_cloth))) {
		EAE6320_ASSERT(false);
	}
	if (!(result = Mesh::s_manager.Load("data/meshes/shell.mesh", mesh_shell))) {
		EAE6320_ASSERT(false);
	}
	if (!(result = Mesh::s_manager.Load("data/meshes/sphere4.mesh", mesh_sphere))) {
		EAE6320_ASSERT(false);
	}
	if (!(result = Mesh::s_manager.Load("data/meshes/bullet.mesh", mesh_smallCube))) {
		EAE6320_ASSERT(false);
	}

	masterMeshArray.push_back(mesh_plane);
	masterMeshArray.push_back(mesh_cloth);
	masterMeshArray.push_back(mesh_shell);
	masterMeshArray.push_back(mesh_sphere);
	masterMeshArray.push_back(mesh_smallCube);

	//create two effect
	Effect* pEffect_white;
	Effect* pEffect_red;

	Effect::Load("data/effects/white.effect", pEffect_white);
	Effect::Load("data/effects/red.effect", pEffect_red);

	masterEffectArray.push_back(pEffect_white);
	masterEffectArray.push_back(pEffect_red);

	//add cloth
	{
		
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, 0.0f, 0.0f);
		Cloth* pGameObject = new Cloth(pEffect_red, mesh_cloth, objState, GetSimulationUpdatePeriod_inSeconds());
		noColliderObjects.push_back(pGameObject);
	}
	
	//add ground mesh
	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, -11.0f, 0.0f);
		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(pEffect_white, mesh_plane, objState);
		strcpy_s(pGameObject->objectType, "Ground");
		noColliderObjects.push_back(pGameObject);
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
	UserOutput::DebugPrint("Cloth Simulation(%d times) : %f ticks(%f s)",
		g_Profiler.m_Allccumulators[0]->m_Count,
		g_Profiler.m_Allccumulators[0]->average(),
		g_Profiler.m_Allccumulators[0]->getAverageTime());
	cbApplication::CleanUp();
	return Results::Success;
}

