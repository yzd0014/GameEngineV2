#pragma once
#include "Engine/GameCommon/GameObject.h"

#include "External/EigenLibrary/Eigen/Dense"
#include "External/EigenLibrary/Eigen/Geometry"

using namespace Eigen;
//#define HIGH_PRECISION_MODE 
#if defined (HIGH_PRECISION_MODE)
typedef double _Scalar;
typedef MatrixXd _Matrix;
typedef Matrix3d _Matrix3;
typedef VectorXd _Vector;
typedef Vector3d _Vector3;
typedef Quaterniond _Quat;
#else
typedef float _Scalar;
typedef MatrixXf _Matrix;
typedef Matrix3f _Matrix3;
typedef VectorXf _Vector;
typedef Vector3f _Vector3;
typedef Quaternionf _Quat;
#endif

#ifndef LOCAL_MODE
#define LOCAL_MODE 0
#endif
#ifndef MUJOCO_MODE
#define MUJOCO_MODE 1
#endif

#ifndef BOX
#define BOX 0
#endif
#ifndef BALL
#define BALL 1
#endif

#ifndef KINEMATIC
#define KINEMATIC 0
#endif
#ifndef PD
#define PD 1
#endif

#ifndef SPD
#define SPD 2
#endif

#ifndef PASSIVE
#define PASSIVE 3
#endif

#ifndef PBD
#define PBD 0
#endif

#ifndef IMPULSE
#define IMPULSE 1
#endif

namespace eae6320
{
	class MultiBody : public eae6320::GameCommon::GameObject
	{
	public:
		MultiBody(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, std::vector<GameCommon::GameObject *> & i_linkBodys, int i_numOfLinks);
		void Tick(const double i_secondCountToIntegrate) override;
		void UpdateGameObjectBasedOnInput() override;

		int rotationMode = MUJOCO_MODE;
		int controlMode = KINEMATIC;
		int constraintSolverMode = PBD;
	private:
		_Vector ComputeQr(_Vector i_R_dot, _Scalar h);
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

		void JointLimitCheck();
		void ResolveJointLimit(const _Scalar h);
		void ResolveJointLimitPBD(const _Scalar h);

		_Vector Rbar;//desired pos
		_Vector R; //3nx1
		_Vector Rdot; //3nx1
		_Matrix Mr;
		_Matrix MrInverse;
		std::vector<_Matrix> Mbody;
		std::vector<_Matrix3> localInertiaTensors;
		std::vector<_Vector3> w_abs_world;//absolute 
		std::vector<_Vector3> w_rel_world;//relative
		std::vector<_Vector3> w_rel_local;
		std::vector<_Vector3> vel;
		std::vector<_Vector3> pos;
		std::vector<_Vector3> jointPos;
		std::vector<std::vector<_Vector3>> uLocals;//object
		std::vector<std::vector<_Vector3>> uGlobals;//world
		
		std::vector<_Scalar> A;
		std::vector<_Scalar> A_dot;
		std::vector<_Scalar> B;
		std::vector<_Scalar> B_dot;
		std::vector<_Scalar> C;
		std::vector<_Scalar> C_dot;
		
		std::vector<_Matrix3> R_global;//rigidbody rotation
		std::vector<_Matrix3> J_rotation;//jotation jabobian matrix
		std::vector<_Matrix> D;
		std::vector<_Matrix> Ht;
		
		std::vector<_Quat> m_orientations;
		std::vector<_Quat> q;//relative rotation to parent for each body
		std::vector<_Quat> qbar;
		std::vector<GameCommon::GameObject *> m_linkBodys;
		_Scalar rigidBodyMass = 1.0f;
		_Scalar kp = 1000000;
		_Scalar kd = 2000;

		std::vector<_Scalar> g_limit;
		std::vector<_Vector3> bodyRotationAxis;
		std::vector<bool> limitReached;
		bool nonZeroLimitJacobian = false;
		int constrainNum = 0;
		
		_Scalar jointLimit = 0.785f;

		int tickCountSimulated = 0;
		int numOfLinks = 2;
		int geometry = BOX;
	};
}