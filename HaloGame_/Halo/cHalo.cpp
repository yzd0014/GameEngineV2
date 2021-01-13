// Includes
//=========
//12/13/2018
#include "cHalo.h"
#include "Engine/GameCommon/Camera.h"
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
#include "Custom Game Objects/CubeSpawner.h"
#include "Custom Game Objects/MoveableCube.h"
#include "Engine/Physics/HingeJoint.h"
#include "Custom Game Objects/Car.h"

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
	mainCamera.Initialize(Math::sVector(0.0f, 10.0f, 10.0f), Math::sVector(-30.0f, 0.0f, 0.0f), Math::ConvertDegreesToRadians(45), 1.0f, 0.1f, 500.0f);
	
	//create two meshes 	
	eae6320::Assets::cHandle<Mesh> mesh_plane;
	eae6320::Assets::cHandle<Mesh> mesh_box;
	eae6320::Assets::cHandle<Mesh> mesh_dot;
	eae6320::Assets::cHandle<Mesh> mesh_wheel;
	eae6320::Assets::cHandle<Mesh> mesh_cube;

	auto result = eae6320::Results::Success;
	if (!(result = Mesh::s_manager.Load("data/meshes/plane.mesh", mesh_plane))) {
		EAE6320_ASSERT(false);
	}
	if (!(result = Mesh::s_manager.Load("data/meshes/box.mesh", mesh_box))) {
		EAE6320_ASSERT(false);
	}
	if (!(result = Mesh::s_manager.Load("data/meshes/bullet.mesh", mesh_dot))) {
		EAE6320_ASSERT(false);
	}
	if (!(result = Mesh::s_manager.Load("data/meshes/wheel.mesh", mesh_wheel))) {
		EAE6320_ASSERT(false);
	}
	if (!(result = Mesh::s_manager.Load("data/meshes/cube.mesh", mesh_cube))) {
		EAE6320_ASSERT(false);
	}

	masterMeshArray.push_back(mesh_plane);
	masterMeshArray.push_back(mesh_box);
	masterMeshArray.push_back(mesh_dot);
	masterMeshArray.push_back(mesh_wheel);
	masterMeshArray.push_back(mesh_cube);

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
	//car
	Car* pCar = nullptr;
	{
		Physics::sRigidBodyState objState;
		objState.position = Math::sVector(0.0f, 2.5f, 0.0f);
		objState.mass = 100.0f;
		//default value is for a 2x2x2 cube
		objState.localInverseInertiaTensor.m_00 = 1.0f / ((1.0f / 12.0f)* objState.mass * 65.0f);
		objState.localInverseInertiaTensor.m_11 = 1.0f / ((1.0f / 12.0f)* objState.mass * 80.0f);
		objState.localInverseInertiaTensor.m_22 = 1.0f / ((1.0f / 12.0f)* objState.mass * 5.0f);
		objState.hasGravity = true;
		pCar = new Car(pEffect_red, mesh_box, objState);
		noColliderObjects.push_back(pCar);
	}

	{//front left wheel
		Physics::sRigidBodyState objState;
		objState.hasGravity = true;
		objState.mass = 1;
		float r = 1.0f;
		objState.collider.m_type = Sphere;
		objState.collider.m_vertices.push_back(Math::sVector(0.0f, 0.0f, 0.0f));
		objState.collider.m_vertices.push_back(Math::sVector(r, 0.0f, 0.0f));
		objState.localInverseInertiaTensor.m_00 = 1.0f / ((2.0f / 5.0f)* r * r);
		objState.localInverseInertiaTensor.m_11 = 1.0f / ((2.0f / 5.0f)* r * r);
		objState.localInverseInertiaTensor.m_22 = 1.0f / ((2.0f / 5.0f)* r * r);
		objState.position = Math::sVector(-2.0f, 2.0f, -4.0f);

		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(pEffect_white, mesh_wheel, objState);
		colliderObjects.push_back(pGameObject);
	}

	{//front right wheel
		Physics::sRigidBodyState objState;
		objState.hasGravity = true;
		objState.mass = 1;
		float r = 1.0f;
		objState.collider.m_type = Sphere;
		objState.collider.m_vertices.push_back(Math::sVector(0.0f, 0.0f, 0.0f));
		objState.collider.m_vertices.push_back(Math::sVector(r, 0.0f, 0.0f));
		objState.localInverseInertiaTensor.m_00 = 1.0f / ((2.0f / 5.0f)* r * r);
		objState.localInverseInertiaTensor.m_11 = 1.0f / ((2.0f / 5.0f)* r * r);
		objState.localInverseInertiaTensor.m_22 = 1.0f / ((2.0f / 5.0f)* r * r);
		objState.position = Math::sVector(2.0f, 2.0f, -4.0f);

		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(pEffect_white, mesh_wheel, objState);
		colliderObjects.push_back(pGameObject);
	}
	{//back left wheel
		Physics::sRigidBodyState objState;
		objState.hasGravity = true;
		objState.mass = 1;
		float r = 1.0f;
		objState.collider.m_type = Sphere;
		objState.collider.m_vertices.push_back(Math::sVector(0.0f, 0.0f, 0.0f));
		objState.collider.m_vertices.push_back(Math::sVector(r, 0.0f, 0.0f));
		objState.localInverseInertiaTensor.m_00 = 1.0f / ((2.0f / 5.0f)* r * r);
		objState.localInverseInertiaTensor.m_11 = 1.0f / ((2.0f / 5.0f)* r * r);
		objState.localInverseInertiaTensor.m_22 = 1.0f / ((2.0f / 5.0f)* r * r);
		objState.position = Math::sVector(-2.0f, 2.0f, 4.0f);

		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(pEffect_white, mesh_wheel, objState);
		colliderObjects.push_back(pGameObject);
	}
	{//back right wheel
		Physics::sRigidBodyState objState;
		objState.hasGravity = true;
		objState.mass = 1;
		float r = 1.0f;
		objState.collider.m_type = Sphere;
		objState.collider.m_vertices.push_back(Math::sVector(0.0f, 0.0f, 0.0f));
		objState.collider.m_vertices.push_back(Math::sVector(r, 0.0f, 0.0f));
		objState.localInverseInertiaTensor.m_00 = 1.0f / ((2.0f / 5.0f)* r * r);
		objState.localInverseInertiaTensor.m_11 = 1.0f / ((2.0f / 5.0f)* r * r);
		objState.localInverseInertiaTensor.m_22 = 1.0f / ((2.0f / 5.0f)* r * r);
		objState.position = Math::sVector(2.0f, 2.0f, 4.0f);

		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(pEffect_white, mesh_wheel, objState);
		colliderObjects.push_back(pGameObject);
	}
	{
		Physics::HingeJoint hingeJoint0(noColliderObjects[0], colliderObjects[0], Math::sVector(-2.0f, -0.5f, -4.0f), Math::sVector(0.0f, 0.0f, 0.0f), Math::sVector(1.0f, 0.0f, 0.0f));
		Physics::allHingeJoints.push_back(hingeJoint0);
		Physics::HingeJoint hingeJoint1(noColliderObjects[0], colliderObjects[1], Math::sVector(2.0f, -0.5f, -4.0f), Math::sVector(0.0f, 0.0f, 0.0f), Math::sVector(1.0f, 0.0f, 0.0f));
		Physics::allHingeJoints.push_back(hingeJoint1);
		Physics::HingeJoint hingeJoint2(noColliderObjects[0], colliderObjects[2], Math::sVector(-2.0f, -0.5f, 4.0f), Math::sVector(0.0f, 0.0f, 0.0f), Math::sVector(1.0f, 0.0f, 0.0f));
		hingeJoint2.motorEnable = true;
		Physics::allHingeJoints.push_back(hingeJoint2);
		pCar->backLeftJointIndex = 2;
		Physics::HingeJoint hingeJoint3(noColliderObjects[0], colliderObjects[3], Math::sVector(2.0f, -0.5f, 4.0f), Math::sVector(0.0f, 0.0f, 0.0f), Math::sVector(1.0f, 0.0f, 0.0f));
		hingeJoint3.motorEnable = true;
		Physics::allHingeJoints.push_back(hingeJoint3);
		pCar->backRightJointIndex = 3;
		//colliderObjects[0]->m_State.orientation = Math::cQuaternion(Math::ConvertDegreesToRadians(20), Math::sVector(0, 1, 0));
		//colliderObjects[0]->m_State.angularVelocity = Math::sVector(0.0f, 20.0f, 0.0f);
	}
	
	//add ground collider
	{
		Physics::AABB boundingBox;
		boundingBox.center = Math::sVector(0.0f, 0.0f, 0.0f);
		boundingBox.extends = Math::sVector(10.0f, 1.0f, 10.0f);

		Physics::sRigidBodyState objState;
		objState.collider.InitializeCollider(boundingBox);
		objState.position = Math::sVector(0.0f, 0.0f, 0.0f);
		objState.collider.m_type = Box;
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
