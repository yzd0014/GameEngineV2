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
#include "Custom Game Objects/HomingCube.h"
#include "Custom Game Objects/MoveableCube.h"
#include "Custom Game Objects/Player.h"
#include "Custom Game Objects/Boss.h"
#include "Custom Game Objects/Cloth.h"
#include "Custom Game Objects/Tri.h"
#include "Custom Game Objects/Ground.h"
#include "Custom Game Objects/CubeSpawner.h"
#include "Custom Game Objects/JellyCube.h"

// Inherited Implementation
//=========================

// Run
//----

void eae6320::cHalo::UpdateBasedOnInput()
{
	// Is the user pressing the ESC key?
	if ( UserInput::IsKeyPressed( UserInput::KeyCodes::Escape ) )
	{
		// Exit the application
		const auto result = Exit( EXIT_SUCCESS );
		EAE6320_ASSERT( result );
	}
}

// Initialization / Clean Up
//--------------------------

eae6320::cResult eae6320::cHalo::Initialize()
{
	//initialize camera 
	mainCamera.Initialize(Math::sVector(0.0f, 0.9f, 10.0f), Math::sVector(0.0f, 0.0f, 0.0f), Math::ConvertDegreesToRadians(45), 1.0f, 0.1f, 500.0f);
	
	//create two meshes 	
	eae6320::Assets::cHandle<Mesh> mesh_plane;
	//eae6320::Assets::cHandle<Mesh> mesh_cloth;
	eae6320::Assets::cHandle<Mesh> mesh_cube;
	eae6320::Assets::cHandle<Mesh> mesh_dot;

	auto result = eae6320::Results::Success;
	if (!(result = Mesh::s_manager.Load("data/meshes/square_plane.mesh", mesh_plane))) {
		EAE6320_ASSERT(false);
	}
	if (!(result = Mesh::s_manager.Load("data/meshes/fem_cube_5.mesh", mesh_cube))) {
		EAE6320_ASSERT(false);
	}
	if (!(result = Mesh::s_manager.Load("data/meshes/bullet.mesh", mesh_dot))) {
		EAE6320_ASSERT(false);
	}
	masterMeshArray.push_back(mesh_plane);
	masterMeshArray.push_back(mesh_cube);
	masterMeshArray.push_back(mesh_dot);

	//create two effect
	Effect* pEffect_white;
	Effect* pEffect_red;

	Effect::Load("data/effects/white.effect", pEffect_white);
	Effect::Load("data/effects/red.effect", pEffect_red);

	masterEffectArray.push_back(pEffect_white);
	masterEffectArray.push_back(pEffect_red);

	//create sound
	//soundArray.push_back(new Engine::Sound("data/audio/neon.wav"));
	//soundArray[0]->PlayInLoop();

	//cube
	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(-1.0f, -1.0f, -1.0f);
		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(pEffect_red, mesh_dot, objState);
		noColliderObjects.push_back(pGameObject);
	}
	//cube
	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(1.0f, 1.0f, 1.0f);
		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(pEffect_red, mesh_dot, objState);
		noColliderObjects.push_back(pGameObject);
	}
	//GetSimulationUpdatePeriod_inSeconds();
	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, 0.0f, 0.0f);
		JellyCube* pGameObject = new JellyCube(pEffect_white, mesh_cube, objState, GetSimulationUpdatePeriod_inSeconds(), noColliderObjects[1]);
		noColliderObjects.push_back(pGameObject);
	}

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
	cbApplication::CleanUp();
	return Results::Success;
}
