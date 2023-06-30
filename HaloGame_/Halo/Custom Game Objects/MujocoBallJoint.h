#pragma once
#include "Engine/GameCommon/GameObject.h"

#include "External/EigenLibrary/Eigen/Dense"
#include "External/EigenLibrary/Eigen/Geometry"
using namespace Eigen;
namespace eae6320
{
	class MujocoBallJoint : public eae6320::GameCommon::GameObject
	{
	public:
		MujocoBallJoint(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, std::vector<GameCommon::GameObject *> & i_linkBodys, int i_numOfLinks);
		void Tick(const float i_secondCountToIntegrate) override;
	private:
		void ForwardKinematics();

		VectorXf w_r;
		std::vector<MatrixXf> M_ds;
		std::vector<Matrix3f> localInertiaTensors;
		std::vector<Vector3f> w_global;
		std::vector<std::vector<Vector3f>> uLocals;
		std::vector<std::vector<Vector3f>> uGlobals;
		std::vector<GameCommon::GameObject *> m_linkBodys;
		std::vector<Quaternionf> m_orientations;
		float rigidBodyMass = 1.0f;
		int numOfLinks = 2;
	};
}