#pragma once
#include "Engine/GameCommon/GameObject.h"
#include "Engine/Math/Functions.h"
#include "Engine/Math/cMatrix_transformation.h"
#include "External/EigenLibrary/Eigen/Dense"
#include "Engine/UserInput/UserInput.h"
#include "Engine/Math/sVector.h"
#include "Engine/UserOutput/UserOutput.h"

using namespace Eigen;

namespace eae6320
{
	enum PDMode {Kinematic, PD, SPD};
	
	class DoublePendulums : public eae6320::GameCommon::GameObject
	{
	public:
		DoublePendulums(Effect * i_pEffect, eae6320::Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, Math::sVector i_anchor, GameCommon::GameObject * i_pSecondPendulum, PDMode i_Mode) :
			GameCommon::GameObject(i_pEffect, i_Mesh, i_State),
			anchor(i_anchor),
			p_SecondPendulum(i_pSecondPendulum),
			m_Mode(i_Mode)
		{
			q.resize(2, 1);
			q.setZero();

			qdot.resize(2, 1);
			qdot.setZero();
		}
		void Tick(const float i_secondCountToIntegrate);
	private:
		MatrixXf q;
		MatrixXf qdot;
		Math::sVector anchor;
		float t = 0.0f;//total elapsed time
		float r = 4.0f;//length 
		float m = 1.0f;//mass for both pendulums
		PDMode m_Mode;
		GameCommon::GameObject * p_SecondPendulum;
	};
}