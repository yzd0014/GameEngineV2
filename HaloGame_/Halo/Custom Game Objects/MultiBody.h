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
		VectorXf ComputeQ_r(VectorXf i_R_dot);
		void ComputeGamma_t(std::vector<VectorXf>& o_gamma_t, VectorXf& i_R_dot);
		
		void ComputeAngularVelocity(VectorXf& i_R_dot);
		void ComputeAngularVelocityExpressionCoefficientDerivative(std::vector<float>& o_A_dot, std::vector<float>& o_B_dot, std::vector<float>& o_C_dot, VectorXf& i_R_dot);
		
		void EulerIntegration(const float h);
		void RK4Integration(const float h);

		void ForwardKinematics();

		VectorXf R; //6x1
		VectorXf R_dot; //6x1
		MatrixXf M_r;
		std::vector<MatrixXf> M_ds;
		std::vector<Matrix3f> localInertiaTensors;
		std::vector<Vector3f> w_global;
		std::vector<std::vector<Vector3f>> uLocals;
		std::vector<std::vector<Vector3f>> uGlobals;
		std::vector<float> A;
		//std::vector<float> A_dot;
		std::vector<float> B;
		//std::vector<float> B_dot;
		std::vector<float> C;
		//std::vector<float> C_dot;
		std::vector<MatrixXf> D;
		std::vector<MatrixXf> H_t;
		std::vector<GameCommon::GameObject *> m_linkBodys;
		float rigidBodyMass = 1.0f;
		int tickCountSimulated = 0;
		int numOfLinks = 2;
	};
}