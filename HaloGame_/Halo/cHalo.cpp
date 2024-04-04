// Includes
//=========
//12/13/2018
#include "cHalo.h"

#include "Engine/Logging/Logging.h"
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
#include "Halo/Custom Game Objects/HingeJointCube.h"
#include "Halo/Custom Game Objects/SphericalJoint.h"
#include "Halo/Custom Game Objects/MultiBody.h"
#include "Halo/Custom Game Objects/MujocoBallJoint.h"
#include "Halo/Custom Game Objects/SphericalJointV2.h"
// Inherited Implementation
//=========================

// Run
//----

void eae6320::cHalo::UpdateBasedOnInput()
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

eae6320::cResult eae6320::cHalo::Initialize()
{
	//initialize camera 
	mainCamera.Initialize(Math::sVector(0.0f, 5.0f, 12.5f), Math::sVector(-30.0f, 0.0f, 0.0f), Math::ConvertDegreesToRadians(45), 1.0f, 0.1f, 500.0f);
	//mainCamera.Initialize(Math::sVector(5.0f, 10.0f, 15.0f), Math::sVector(-30.0f, 20.0f, 0.0f), Math::ConvertDegreesToRadians(45), 1.0f, 0.1f, 500.0f);
	
	//create two meshes
	LOAD_MESH("data/meshes/square_plane.mesh", mesh_plane)
	LOAD_MESH("data/meshes/cube.mesh", mesh_cube)
	LOAD_MESH("data/meshes/bullet.mesh", mesh_anchor)
	LOAD_MESH("data/meshes/pendulum1.mesh", mesh_pen1)
	LOAD_MESH("data/meshes/pendulum2.mesh", mesh_pen2)

	//load effect
	LOAD_EFFECT("data/effects/default.effect", pDefaultEffect)
	LOAD_EFFECT("data/effects/red.effect", pRedEffect)
	
	{
		std::vector<GameCommon::GameObject *> links;
		int bodyNum = 1;
		for (int i = 0; i < bodyNum; i++)
		{
			GameCommon::GameObject *pGameObject = new GameCommon::GameObject(pDefaultEffect, mesh_cube, Physics::sRigidBodyState());
			links.push_back(pGameObject);
			noColliderObjects.push_back(pGameObject);
		}

		Physics::sRigidBodyState objState(Math::sVector(0.0f, 0.0f, 0.0f));
		MultiBody * pMultiBody = new MultiBody(pRedEffect, mesh_anchor, objState, links, bodyNum);
		//GameCommon::GameObject * pMultiBody = new SphericalJointV2(pRedEffect, mesh_anchor, objState, links, bodyNum);
		noColliderObjects.push_back(pMultiBody);
	}
	
	//{
	//	std::vector<GameCommon::GameObject *> links;
	//	int bodyNum = 2;
	//	for (int i = 0; i < bodyNum; i++)
	//	{
	//		GameCommon::GameObject *pGameObject = new GameCommon::GameObject(pDefaultEffect, mesh_cube, Physics::sRigidBodyState());
	//		links.push_back(pGameObject);
	//		noColliderObjects.push_back(pGameObject);
	//	}

	//	Physics::sRigidBodyState objState(Math::sVector(6.0f, 0.0f, 0.0f));
	//	MultiBody * pMultiBody = new MultiBody(pRedEffect, mesh_anchor, objState, links, bodyNum);
	//	pMultiBody->rotationMode = LOCAL_MODE;
	//	pMultiBody->controlMode = SPD;
	//	//GameCommon::GameObject * pGameObject = new MujocoBallJoint(pRedEffect, mesh_anchor, objState, links, bodyNum);
	//	//GameCommon::GameObject * pGameObject = new SphericalJointV2(pRedEffect, mesh_anchor, objState, links, bodyNum);
	//	noColliderObjects.push_back(pMultiBody);
	//}

	//{
	//	std::vector<GameCommon::GameObject *> links;
	//	int bodyNum = 2;
	//	for (int i = 0; i < bodyNum; i++)
	//	{
	//		GameCommon::GameObject *pGameObject = new GameCommon::GameObject(pDefaultEffect, mesh_cube, Physics::sRigidBodyState());
	//		links.push_back(pGameObject);
	//		noColliderObjects.push_back(pGameObject);
	//	}

	//	Physics::sRigidBodyState objState(Math::sVector(-6.0f, 0.0f, 0.0f));
	//	MultiBody * pMultiBody = new MultiBody(pRedEffect, mesh_anchor, objState, links, bodyNum);
	//	pMultiBody->rotationMode = MUJOCO_MODE;
	//	pMultiBody->controlMode = KINEMATIC;
	//	//GameCommon::GameObject * pGameObject = new MujocoBallJoint(pRedEffect, mesh_anchor, objState, links, bodyNum);
	//	//GameCommon::GameObject * pGameObject = new SphericalJointV2(pRedEffect, mesh_anchor, objState, links, bodyNum);
	//	noColliderObjects.push_back(pMultiBody);
	//}

	//Ground
	{
		Physics::sRigidBodyState objState(Math::sVector(0.0f, -8.0f, 0.0f));
		GameCommon::GameObject * pGameObject = new GameCommon::GameObject(pDefaultEffect, mesh_plane, objState);
		noColliderObjects.push_back(pGameObject);
	}
	
	//cube with ball joint
	//{
	//	//HingeJointCube * pGameObject = new HingeJointCube(pDefaultEffect, mesh_cube, Physics::sRigidBodyState());
	//	SphericalJoint *pGameObject = new SphericalJoint(pDefaultEffect, mesh_cube, Physics::sRigidBodyState());
	//	noColliderObjects.push_back(pGameObject);
	//}
	return Results::Success;
}

void eae6320::cHalo::UpdateSimulationBasedOnInput() {
	if (isGameOver == false)
	{
		cbApplication::UpdateSimulationBasedOnInput();
	}
}

void  eae6320::cHalo::UpdateSimulationBasedOnTime(const double i_elapsedSecondCount_sinceLastUpdate) {
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