#pragma once
#include "Engine/GameCommon/GameObject.h"

#include "External/EigenLibrary/Eigen/Dense"
#include "External/EigenLibrary/Eigen/Geometry"

using namespace Eigen;

namespace eae6320
{
	class doublePendulumBallJoint : public eae6320::GameCommon::GameObject
	{
	public:
		doublePendulumBallJoint(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, std::vector<GameCommon::GameObject *> & i_linkBodys, int i_numOfLinks);
		void Tick(const double i_secondCountToIntegrate) override;
		void UpdateGameObjectBasedOnInput() override;
	private:
		void ForwardKinematics();
		VectorXf R;
		VectorXf R_dot;

		std::vector<GameCommon::GameObject *> m_linkBodys;
		std::vector<Vector3f> u_global;
		Vector3f u_local;

		int numOfLinks;
	};
}