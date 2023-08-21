#pragma once
#include "Engine/GameCommon/GameObject.h"

#include "External/EigenLibrary/Eigen/Dense"
#include "External/EigenLibrary/Eigen/Geometry"

//#define HIGH_PRECISION_MODE 

using namespace Eigen;

#if defined (HIGH_PRECISION_MODE)
typedef double _Scalar;
typedef MatrixXd _Matrix;
typedef Matrix3d _Matrix3;
typedef VectorXd _Vector;
typedef Vector3d _Vector3;
#else
typedef float _Scalar;
typedef MatrixXf _Matrix;
typedef Matrix3f _Matrix3;
typedef VectorXf _Vector;
typedef Vector3f _Vector3;
#endif

#define LOCAL_MODE 0
#define GLOBAL_MODE 1
#define MUJOCO_MODE 2

#define BOX 0
#define BALL 1

namespace eae6320
{
	class MultiBody : public eae6320::GameCommon::GameObject
	{
	public:
		MultiBody(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, std::vector<GameCommon::GameObject *> & i_linkBodys, int i_numOfLinks);
		void Tick(const double i_secondCountToIntegrate) override;
		void UpdateGameObjectBasedOnInput() override;
	private:
		_Vector ComputeQ_r(_Vector i_R_dot);
		void ComputeGamma_t(std::vector<_Vector>& o_gamma_t, _Vector& i_R_dot);
		
		void ComputeAngularVelocity(_Vector& i_R_dot);
		void ComputeVelocity();
		void Compute_abc();
		void Compute_abc_dot(_Vector& i_R_dot);
		
		void EulerIntegration(const _Scalar h);
		void RK4Integration(const _Scalar h);
		void RK3Integration(const _Scalar h);

		void ForwardKinematics();
		_Scalar ComputeTotalEnergy();

		_Vector R; //6x1
		_Vector R_dot; //6x1
		_Matrix M_r;
		std::vector<_Matrix> M_ds;
		std::vector<_Matrix3> localInertiaTensors;
		std::vector<_Vector3> w_global;
		std::vector<_Vector3> w_local;
		std::vector<_Vector3> vel;
		std::vector<_Vector3> pos;
		std::vector<_Vector3> jointPos;
		std::vector<std::vector<_Vector3>> uLocals;
		std::vector<std::vector<_Vector3>> uGlobals;
		std::vector<_Scalar> A;
		std::vector<_Scalar> A_dot;
		std::vector<_Scalar> B;
		std::vector<_Scalar> B_dot;
		std::vector<_Scalar> C;
		std::vector<_Scalar> C_dot;
		std::vector<_Matrix> D;
		std::vector<_Matrix> H_t;
		
		std::vector<Quaternionf> m_orientations;
		std::vector<GameCommon::GameObject *> m_linkBodys;
		_Scalar rigidBodyMass = 1.0f;
		
		int tickCountSimulated = 0;
		int numOfLinks = 2;
		int rotationMode = LOCAL_MODE;
		int geometry = BALL;
	};
}