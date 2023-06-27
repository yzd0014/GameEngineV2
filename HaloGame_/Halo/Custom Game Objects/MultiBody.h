#pragma once
#include "Engine/GameCommon/GameObject.h"

#include "External/EigenLibrary/Eigen/Dense"
#include "External/EigenLibrary/Eigen/Geometry"
using namespace Eigen;
namespace eae6320
{
	class MultiBody : public eae6320::GameCommon::GameObject
	{
	public:
		MultiBody(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, std::vector<GameCommon::GameObject *> & i_linkBodys, int i_numOfLinks);
		void Tick(const float i_secondCountToIntegrate) override;
		void UpdateGameObjectBasedOnInput() override;
	private:
		void ForwardKinematics();

		VectorXf R; //6x1
		VectorXf R_dot; //6x1
		std::vector<MatrixXf> M_ds;
		std::vector<Matrix3f> localInertiaTensors;
		std::vector<Vector3f> w_global;
		std::vector<std::vector<Vector3f>> uLocals;
		std::vector<std::vector<Vector3f>> uGlobals;
		std::vector<float> A;
		std::vector<float> A_dot;
		std::vector<float> B;
		std::vector<float> B_dot;
		std::vector<float> C;
		std::vector<float> C_dot;
		std::vector<GameCommon::GameObject *> m_linkBodys;
		float rigidBodyMass = 1.0f;
		int tickCountSimulated = 0;
		int numOfLinks = 2;
	};
}