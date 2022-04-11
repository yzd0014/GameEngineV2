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
#include "Halo/Custom Game Objects/HingeJointCube.h"
#include "Halo/Custom Game Objects/SphericalJoint.h"
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
	mainCamera.Initialize(Math::sVector(0.0f, 5.0f, 12.5f), Math::sVector(-30.0f, 0.0f, 0.0f), Math::ConvertDegreesToRadians(45), 1.0f, 0.1f, 500.0f);
	//mainCamera.Initialize(Math::sVector(5.0f, 10.0f, 15.0f), Math::sVector(-30.0f, 20.0f, 0.0f), Math::ConvertDegreesToRadians(45), 1.0f, 0.1f, 500.0f);

	//create two meshes
	LOAD_MESH("data/meshes/square_plane.mesh", mesh_plane)
	LOAD_MESH("data/meshes/cube.mesh", mesh_cube)
	LOAD_MESH("data/meshes/bullet.mesh", mesh_anchor)

	//load effect
	LOAD_EFFECT("data/effects/default.effect", pDefaultEffect)
	LOAD_EFFECT("data/effects/red.effect", pRedEffect)

	{
		Physics::sRigidBodyState objState(Math::sVector(0.0f, 0.0f, 0.0f));
		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(pRedEffect, mesh_anchor, objState);
		noColliderObjects.push_back(pGameObject);
	}
	//Ground
	{
		Physics::sRigidBodyState objState(Math::sVector(0.0f, -5.0f, 0.0f));
		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(pDefaultEffect, mesh_plane, objState);
		noColliderObjects.push_back(pGameObject);
	}
	
	//cube with ball joint
	{
		//HingeJointCube * pGameObject = new HingeJointCube(pDefaultEffect, mesh_cube, Physics::sRigidBodyState());
		SphericalJoint *pGameObject = new SphericalJoint(pDefaultEffect, mesh_cube, Physics::sRigidBodyState());
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
	/*
	UserOutput::DebugPrint("Cloth Simulation(%d times) : %f ticks(%f s)",
		g_Profiler.m_Allccumulators[0]->m_Count,
		g_Profiler.m_Allccumulators[0]->average(),
		g_Profiler.m_Allccumulators[0]->getAverageTime());*/
	cbApplication::CleanUp();
	return Results::Success;
}