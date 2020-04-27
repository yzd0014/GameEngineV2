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
		gameOjbectsWithoutCollider.push_back(pGameObject);
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
		masterGameObjectArr.push_back(pGameObject);
	}
	//add ground mesh
	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, 1.0f, 0.0f);
		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(pEffect_white, mesh_plane, objState);
		strcpy_s(pGameObject->objectType, "Ground");
		gameOjbectsWithoutCollider.push_back(pGameObject);
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
	if (isGameOver == false) {
		mainCamera.UpdateCameraBasedOnInput();
		size_t numOfObjects = masterGameObjectArr.size();
		for (size_t i = 0; i < numOfObjects; i++) {
			masterGameObjectArr[i]->UpdateGameObjectBasedOnInput();
		}
		numOfObjects = gameOjbectsWithoutCollider.size();
		for (size_t i = 0; i < numOfObjects; i++) {
			gameOjbectsWithoutCollider[i]->UpdateGameObjectBasedOnInput();
		}
	}
}

void  eae6320::cHalo::UpdateSimulationBasedOnTime(const float i_elapsedSecondCount_sinceLastUpdate) {
	if (isGameOver == false) {
		size_t size_physicsObject = masterGameObjectArr.size();
// ***********************run physics****************************************************	
		//update game objects with AABB
		//Physics::PhysicsUpdate(masterGameObjectArr, i_elapsedSecondCount_sinceLastUpdate);
		Physics::RunPhysics(masterGameObjectArr, gameOjbectsWithoutCollider, masterMeshArray[2], masterEffectArray[1], i_elapsedSecondCount_sinceLastUpdate);
		//update non-phyiscs objects
		for (size_t i = 0; i < gameOjbectsWithoutCollider.size(); i++) {
			gameOjbectsWithoutCollider[i]->m_State.Update(i_elapsedSecondCount_sinceLastUpdate);
		}
		//update camera
		mainCamera.UpdateState(i_elapsedSecondCount_sinceLastUpdate);
//run AI*********************************************************************************
		for (size_t i = 0; i < size_physicsObject; i++) {
			masterGameObjectArr[i]->EventTick(i_elapsedSecondCount_sinceLastUpdate);
		}
		for (size_t i = 0; i < gameOjbectsWithoutCollider.size(); i++) {
			gameOjbectsWithoutCollider[i]->EventTick(i_elapsedSecondCount_sinceLastUpdate);
		}
	}
	else {
		GameCommon::ResetAllGameObjectsVelo(masterGameObjectArr, gameOjbectsWithoutCollider, mainCamera);
	}
	GameCommon::RemoveInactiveGameObjects(masterGameObjectArr);
}


eae6320::cResult eae6320::cHalo::CleanUp()
{	//release all game objects first
	size_t numOfObjects = masterGameObjectArr.size();
	for (size_t i = 0; i < numOfObjects; i++) {
		delete masterGameObjectArr[i];
	}
	masterGameObjectArr.clear();
	numOfObjects = gameOjbectsWithoutCollider.size();
	for (size_t i = 0; i < numOfObjects; i++) {
		delete gameOjbectsWithoutCollider[i];
	}
	gameOjbectsWithoutCollider.clear();

	//release effect
	for (size_t i = 0; i < masterEffectArray.size(); i++) {
		masterEffectArray[i]->DecrementReferenceCount();
		masterEffectArray[i] = nullptr;
	}
	masterEffectArray.clear();

	//release mesh handle
	for (size_t i = 0; i < masterMeshArray.size(); i++) {
		Mesh::s_manager.Release(masterMeshArray[i]);
	}
	masterMeshArray.clear();

	//delete sound
	for (size_t i = 0; i < soundArray.size(); i++) {
		delete soundArray[i];
	}
	soundArray.clear();
	
	return Results::Success;
}

void eae6320::cHalo::SubmitDataToBeRendered(const float i_elapsedSecondCount_systemTime, const float i_elapsedSecondCount_sinceLastSimulationUpdate) {	
	//submit background color
	float color[] = { 0.0f, 0.7f, 1.0f , 1.0f };
	eae6320::Graphics::SubmitBGColor(color);
	
	//submit gameObject with colliders 
	for (size_t i = 0; i < masterGameObjectArr.size(); i++) {
		//smooth movement first
		Math::sVector position;
		Math::cQuaternion orientation;
		if (masterGameObjectArr[i]->m_State.movementInterpolation)
		{
			position = masterGameObjectArr[i]->m_State.PredictFuturePosition(i_elapsedSecondCount_sinceLastSimulationUpdate);
			orientation = masterGameObjectArr[i]->m_State.PredictFutureOrientation(i_elapsedSecondCount_sinceLastSimulationUpdate);
		}
		else
		{
			position = masterGameObjectArr[i]->m_State.position;
			orientation = masterGameObjectArr[i]->m_State.orientation;
		}
		//submit
		eae6320::Graphics::SubmitObject(Math::cMatrix_transformation(orientation, position),
			masterGameObjectArr[i]->GetEffect(), Mesh::s_manager.Get(masterGameObjectArr[i]->GetMesh()));

	}
	//submit gameObject without colliders
	for (size_t i = 0; i < gameOjbectsWithoutCollider.size(); i++) {
		//smooth movement first
		Math::sVector position;
		Math::cQuaternion orientation;
		if(gameOjbectsWithoutCollider[i]->m_State.movementInterpolation)
		{
			position = gameOjbectsWithoutCollider[i]->m_State.PredictFuturePosition(i_elapsedSecondCount_sinceLastSimulationUpdate);
			orientation = gameOjbectsWithoutCollider[i]->m_State.PredictFutureOrientation(i_elapsedSecondCount_sinceLastSimulationUpdate);
		}
		else
		{
			position = gameOjbectsWithoutCollider[i]->m_State.position;
			orientation = gameOjbectsWithoutCollider[i]->m_State.orientation;
		}
		//submit
		eae6320::Graphics::SubmitObject(Math::cMatrix_transformation(orientation, position),
			gameOjbectsWithoutCollider[i]->GetEffect(), Mesh::s_manager.Get(gameOjbectsWithoutCollider[i]->GetMesh()));

	}
	
	//submit camera
	{
		//smooth camera movemnt first before it's submitted
		Math::sVector predictedPosition = mainCamera.PredictFuturePosition(i_elapsedSecondCount_sinceLastSimulationUpdate);
		Math::cQuaternion predictedOrientation = mainCamera.PredictFutureOrientation(i_elapsedSecondCount_sinceLastSimulationUpdate);
		//submit
		eae6320::Graphics::SubmitCamera(Math::cMatrix_transformation::CreateWorldToCameraTransform(predictedOrientation, predictedPosition),
			mainCamera.GetCameraToProjectedMat());
	}	
}
