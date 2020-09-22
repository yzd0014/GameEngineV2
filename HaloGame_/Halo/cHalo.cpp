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
#include "HaloGame_/Halo/Custom Game Objects/DoublePendulums.h"

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
	mainCamera.Initialize(Math::sVector(0.0f, 1.0f, 10.0f), Math::sVector(0.0f, 0.0f, 0.0f), Math::ConvertDegreesToRadians(45), 1.0f, 0.1f, 500.0f);
	
	//create two meshes 	
	eae6320::Assets::cHandle<Mesh> mesh_plane;
	eae6320::Assets::cHandle<Mesh> mesh_p1;
	eae6320::Assets::cHandle<Mesh> mesh_p2;

	auto result = eae6320::Results::Success;
	if (!(result = Mesh::s_manager.Load("data/meshes/square_plane.mesh", mesh_plane))) {
		EAE6320_ASSERT(false);
	}
	if (!(result = Mesh::s_manager.Load("data/meshes/pendulum1.mesh", mesh_p1))) {
		EAE6320_ASSERT(false);
	}
	if (!(result = Mesh::s_manager.Load("data/meshes/pendulum2.mesh", mesh_p2))) {
		EAE6320_ASSERT(false);
	}
	
	masterMeshArray.push_back(mesh_plane);
	masterMeshArray.push_back(mesh_p1);
	masterMeshArray.push_back(mesh_p2);

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

	//GetSimulationUpdatePeriod_inSeconds();
	//add cloth
	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, 0.0f, 0.0f);
		GameCommon::GameObject * p2 = new GameCommon::GameObject(pEffect_white, mesh_p2, objState);
		noColliderObjects.push_back(p2);
	
		DoublePendulums* referencePendulum = new DoublePendulums(pEffect_white, mesh_p1, objState,
			Math::sVector(0.0f, 0.0f, 0.0f),
			p2, Kinematic);
		noColliderObjects.push_back(referencePendulum);
	}

	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, 0.0f, 0.0f);
		GameCommon::GameObject * p2 = new GameCommon::GameObject(pEffect_red, mesh_p2, objState);
		noColliderObjects.push_back(p2);

		DoublePendulums* referencePendulum = new DoublePendulums(pEffect_red, mesh_p1, objState,
			Math::sVector(4.0f, 0.0f, -4.0f),
			p2, SPD);
		noColliderObjects.push_back(referencePendulum);
	}

	{
		/*
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, 0.0f, 0.0f);
		GameCommon::GameObject * p2 = new GameCommon::GameObject(pEffect_red, mesh_p2, objState);
		noColliderObjects.push_back(p2);

		DoublePendulums* referencePendulum = new DoublePendulums(pEffect_red, mesh_p1, objState,
			Math::sVector(-4.0f, 0.0f, -4.0f),
			p2, PD);
		noColliderObjects.push_back(referencePendulum);
		*/
	}
	//add ground
	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, -10.0f, 0.0f);
		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(pEffect_white, mesh_plane, objState);
		strcpy_s(pGameObject->objectType, "Ground");
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
