#pragma once
#include "Engine/GameCommon/GameObject.h"

#include "External/EigenLibrary/Eigen/Dense"
#include "External/EigenLibrary/Eigen/Geometry"
using namespace Eigen;

namespace eae6320
{
	class SphericalJointV2 : public eae6320::GameCommon::GameObject
	{
	public:
		SphericalJointV2(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, std::vector<GameCommon::GameObject *> & i_linkBodys, int i_numOfLinks);
		void Tick(const double i_secondCountToIntegrate) override;
	private:
		void ForwardKinematics();
		std::vector<Vector3f> R;
		std::vector<Vector3f> R_dot;
		std::vector<Vector3f> R_ddot;
		std::vector<VectorXf> V_dot;
		std::vector<MatrixXf> M;
		std::vector<Matrix3f> localInertiaTensors;
		std::vector<MatrixXf> Mr;
		std::vector<VectorXf> Q;
		std::vector<VectorXf> Qr;
		std::vector<MatrixXf> D;
		std::vector<MatrixXf> H;
		std::vector<VectorXf> gamma;

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
		int numOfLinks = 2;
	};
}