#pragma once
#include "Engine/GameCommon/GameObject.h"

#include "External/EigenLibrary/Eigen/Dense"
#include "External/EigenLibrary/Eigen/Geometry"
#include "MultiBodyTypeDefine.h"

using namespace Eigen;
#define HIGH_PRECISION_MODE 
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

namespace eae6320
{
	class MultiBody : public eae6320::GameCommon::GameObject
	{
	public:
		MultiBody(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, std::vector<GameCommon::GameObject *> & i_linkBodys, int i_numOfLinks);
		void Tick(const double i_secondCountToIntegrate) override;
		void UpdateGameObjectBasedOnInput() override;

		int constraintSolverMode = -1;
		bool gravity = FALSE ;
	private:
		void ComputeMr();
		void ComputeHt();
		void ComputeH();
		void ComputeD();
		_Vector ComputeQr(_Vector i_qdot);
		_Vector ComputeQr_SikpVelocityUpdate(_Vector& i_qdot);
		void ComputeGamma_t(std::vector<_Vector>& o_gamma_t, _Vector& i_qdot);
		
		void ForwardAngularAndTranslationalVelocity(_Vector& i_qdot);
		
		void EulerIntegration(const _Scalar h);
		void RK4Integration(const _Scalar h);
		void RK3Integration(const _Scalar h);
		void Integrate_q(_Vector& o_q, _Vector& i_q, _Vector& i_qdot, _Scalar h);

		void ForwardKinematics();
		void Forward();
		void ClampRotationVector();
		_Scalar ComputeKineticEnergy();
		_Scalar ComputePotentialEnergy();
		_Scalar ComputeTotalEnergy();
		_Vector3 ComputeTranslationalMomentum();
		_Vector3 ComputeAngularMomentum();

		void JointLimitCheck();
		void ResolveJointLimit(const _Scalar h);
		void ResolveJointLimitPBD(_Vector& i_q, const _Scalar h);

		void KineticEnergyProjection();
		void EnergyMomentumProjection();
		void ManifoldProjection();

		_Vector q;
		_Vector qdot;
		std::vector<int> jointType;
		std::vector<int> posDOF;
		std::vector<int> velDOF;
		std::vector<int> posStartIndex;
		std::vector<int> velStartIndex;
		_Matrix Mr;
		_Matrix MrInverse;
		std::vector<_Matrix> Mbody;
		std::vector<_Matrix3> localInertiaTensors;
		std::vector<_Vector3> w_abs_world;//absolute 
		std::vector<_Vector3> w_rel_world;//relative
		std::vector<_Vector3> w_rel_local;
		std::vector<_Vector3> vel;
		std::vector<_Vector3> pos;//rigid body center of mass
		std::vector<_Vector3> jointPos;
		std::vector<std::vector<_Vector3>> uLocals;//object
		std::vector<std::vector<_Vector3>> uGlobals;//world
		
		std::vector<_Matrix3> R_global;//rigidbody rotation
		std::vector<_Matrix3> J_rotation;//jotation jabobian matrix
		std::vector<_Matrix> D;
		std::vector<_Matrix> Ht;
		std::vector<_Matrix> H;
		
		std::vector<_Quat> obs_ori;
		std::vector<_Quat> rel_ori;//relative rotation to parent for each body
		std::vector<GameCommon::GameObject *> m_linkBodys;
		_Scalar rigidBodyMass = 1.0f;
		_Scalar kp = 1000000;
		_Scalar kd = 2000;

		std::vector<_Scalar> g;
		std::vector<int> jointsID;
		
		_Scalar jointLimit = 0.785f;
		_Scalar kineticEnergy0 = 0;
		_Scalar totalEnergy0 = 0;
		_Vector3 angularMomentum0;
		_Vector3 linearMomentum0;
		//_Vector conservedQuantity;

		int tickCountSimulated = 0;
		int numOfLinks = 2;
		int totalPosDOF = 0;
		int totalVelDOF = 0;
		int geometry = BOX;
/*******************************************************************************************/
		inline _Scalar Compute_a(_Scalar theta)
		{
			_Scalar alpha;
			if (theta < 0.0001) alpha = 1.0f - theta * theta / 6.0f;
			else alpha = sin(theta) / theta;

			return alpha;
		}
		inline _Scalar Compute_a_dot(_Scalar c, _Scalar b, _Vector3& r, _Vector3& rdot)
		{
			_Scalar out;
			out = (c - b) * r.dot(rdot);
			return out;
		}
		inline _Scalar Compute_b(_Scalar theta)
		{
			_Scalar beta;
			if (theta < 0.0001) beta = 0.5f - theta * theta / 24.0f;
			else beta = (1.0f - cos(theta)) / (theta * theta);

			return beta;
		}
		inline _Scalar Compute_b_dot(_Scalar theta, _Scalar a, _Scalar b, _Vector3 r, _Vector3 rdot)
		{
			_Scalar out;
			if (theta < 0.0001) out = (-1.0f / 12.0f + 1.0f / 180.0f * theta * theta) * r.dot(rdot);
			else out = (a - 2.0f * b) / (theta * theta) * r.dot(rdot);
			
			return out;
		}

		inline _Scalar Compute_c(_Scalar theta, _Scalar i_a)
		{
			_Scalar gamma;
			if (theta < 0.0001) gamma = 1.0f / 6.0f - theta * theta / 120.0f;
			else gamma = (1.0f - i_a) / (theta * theta);

			return gamma;
		}

		inline _Scalar Compute_c_dot(_Scalar theta, _Scalar b, _Scalar c, _Vector3 r, _Vector3 rdot)
		{
			_Scalar out;
			if (theta < 0.0001) out = (-1.0f / 60.0f + 1.0f / 1260.0f * theta * theta) * r.dot(rdot);
			else out = (b - 3.0f * c) / (theta * theta) * r.dot(rdot);

			return out;
		}

		inline _Scalar Compute_s(_Scalar theta, _Scalar i_a, _Scalar i_b)
		{
			_Scalar zeta;
			if (theta < 0.0001) zeta = 1.0f / 12.0f + theta * theta / 720.0f;
			else zeta = (1 - i_a / (2 * i_b)) / (theta * theta);

			return zeta;
		}
	};
}