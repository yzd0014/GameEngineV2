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
	if (!(result = Mesh::s_manager.Load("data/meshes/cube.mesh", mesh_cube))) {
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
	/*
	{
		Physics::AABB boundingBox;
		boundingBox.center = Math::sVector(0.0f, 0.0f, 0.0f);
		boundingBox.extends = Math::sVector(1.0f, 1.0f, 1.0f);

		Physics::sRigidBodyState objState;
		objState.collider.InitializeCollider(boundingBox);
		objState.position = Math::sVector(0.0f, 6.0f, 0.0f);
		objState.orientation = Math::cQuaternion(Math::ConvertDegreesToRadians(5), Math::sVector(0, 0, 1));
		//objState.velocity = Math::sVector(0.0f, -4.0f, 0.0f);
		objState.hasGravity = true;
		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(pEffect_white, mesh_cube, objState);
		masterGameObjectArr.push_back(pGameObject);
	}
	*/
	//cube
	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, 5.0f, 0.0f);
		objState.orientation = Math::cQuaternion();
		CubeSpawner * pGameObject = new CubeSpawner(pEffect_red, mesh_dot, objState, this);
		noColliderObjects.push_back(pGameObject);
	}
	
	//add ground collider
	{
		Physics::AABB boundingBox;
		boundingBox.center = Math::sVector(0.0f, 0.0f, 0.0f);
		boundingBox.extends = Math::sVector(8.0f, 1.0f, 8.0f);

		Physics::sRigidBodyState objState;
		objState.collider.InitializeCollider(boundingBox);
		objState.position = Math::sVector(0.0f, 0.0f, 0.0f);
		Ground* pGameObject = new Ground(pEffect_white, mesh_dot, objState);
		strcpy_s(pGameObject->objectType, "Ground");
		colliderObjects.push_back(pGameObject);
	}
	//add ground mesh
	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, 1.0f, 0.0f);
		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(pEffect_white, mesh_plane, objState);
		strcpy_s(pGameObject->objectType, "Ground");
		noColliderObjects.push_back(pGameObject);
	}
	/*
	{
		//add fixed point
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, 5.0f, 0.0f);
		MoveableCube * pGameObject = new MoveableCube(pEffect_red, mesh_dot, objState);
		gameOjbectsWithoutCollider.push_back(pGameObject);
	}
	{
		//contrained box
		Physics::AABB boundingBox;
		boundingBox.center = Math::sVector(0.0f, 0.0f, 0.0f);
		boundingBox.extends = Math::sVector(1.0f, 1.0f, 1.0f);

		Physics::sRigidBodyState objState;
		objState.collider.InitializeCollider(boundingBox);
		objState.position = Math::sVector(-1.0f, 4.0f, -1.0f);
		objState.hasGravity = true;
		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(pEffect_white, mesh_cube, objState);
		masterGameObjectArr.push_back(pGameObject);

		//add a point joint
		Physics::PointJoint pointJoint;
		pointJoint.pGameObject = pGameObject;
		pointJoint.pParentObject = gameOjbectsWithoutCollider[1];
		pointJoint.anchor = Math::sVector(0.0f, 5.0f, 0.0f);
		pointJoint.extend = Math::sVector(1.0f, 1.0f, 1.0f);
		Physics::allPointJoints.push_back(pointJoint);
	}
	*/
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
