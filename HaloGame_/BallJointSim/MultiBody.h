#pragma once
#include "Engine/GameCommon/GameObject.h"

#include "External/EigenLibrary/Eigen/Dense"
#include "External/EigenLibrary/Eigen/Geometry"
#include "MultiBodyTypeDefine.h"
#include "Engine/Math/DataTypeDefine.h"
#include "Engine/Math/3DMathHelpers.h"

namespace eae6320
{
	class MultiBody : public eae6320::GameCommon::GameObject
	{
	public:
		MultiBody(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State);
		void Tick(const double i_secondCountToIntegrate) override;
		void UpdateGameObjectBasedOnInput() override;

		int constraintSolverMode = IMPULSE;
		int constraintType = SWING_C;//only used for testing
		int swingMode = EULER_SWING;
		bool gravity = FALSE ;
		bool enablePositionSolve = FALSE;//position solve currently doesn't support free joint
	private:
		void InitializeBodies(Assets::cHandle<Mesh> i_mesh, Vector3d i_meshScale, _Matrix3& i_localInertiaTensor, _Vector3 i_partentJointPosition, _Vector3 i_childJointPosition);
		void InitializeJoints(int* i_jointType);
		void SetZeroInitialCondition();
		void ConfigurateBallJoint(_Vector3& xAxis, _Vector3& yAxis, _Vector3& zAxis, _Scalar swingAngle, _Scalar twistAngle);

		void ComputeMr();
		void ComputeHt(_Vector& i_q, std::vector<_Quat>& i_quat);
		_Vector ComputeQr(_Vector i_qdot);
		_Vector ComputeQr_SikpVelocityUpdate(_Vector& i_qdot);
		void ComputeGamma_t(std::vector<_Vector>& o_gamma_t, _Vector& i_qdot);
		
		void ForwardAngularAndTranslationalVelocity(_Vector& i_qdot);
		
		void EulerIntegration(const _Scalar h);
		void RK4Integration(const _Scalar h);
		void RK3Integration(const _Scalar h);
		void Integrate_q(_Vector& o_q, std::vector<_Quat>& o_quat, _Vector& i_q, std::vector<_Quat>& i_quat, _Vector& i_qdot, _Scalar h);

		void ForwardKinematics(_Vector& i_q, std::vector<_Quat>& i_quat);
		void Forward();
		void UpdateBodyRotation(_Vector& i_q, std::vector<_Quat>& i_quat);
		void ClampRotationVector();
		_Scalar ComputeKineticEnergy();
		_Scalar ComputePotentialEnergy();
		_Scalar ComputeTotalEnergy();
		_Vector3 ComputeTranslationalMomentum();
		_Vector3 ComputeAngularMomentum();

		_Scalar ComputeAngularVelocityConstraint(_Vector3& w, _Vector3& p, _Matrix3& Rot, int i_limitType, _Scalar phi);
		void SwingLimitCheck();
		void ResolveSwingLimit(const _Scalar h);
		void ResolveSwingLimitPBD(_Vector& i_q, const _Scalar h);
		
		void TwistLimitCheck();
		void ResolveTwistLimit(const _Scalar h);
		void ResolveTwistLimitPBD(_Vector& i_q, const _Scalar h);
		
		void _BallJointLimitCheck();
		void _ResolveJointLimit(const _Scalar h);
		void _ResolveJointLimitPBD(_Vector& i_q, const _Scalar h);
		
		void BallJointLimitCheck();
		void SolveVelocityJointLimit(const _Scalar h);
		void SolvePositionJointLimit(const _Scalar h);
		void SolvePositionJointLimit();
		_Scalar ComputeSwingError(int jointNum);
		_Scalar ComputeTwistEulerError(int jointNum, bool checkVectorField);
		void ComputeTwistEulerJacobian(int jointNum, _Matrix& o_J);
		void ComputeSwingJacobian(int jointNum, _Matrix& o_J);

		void PrePositionSolveProccessing();
		void PostPositionSolveProccessing();

		void KineticEnergyProjection();
		void EnergyMomentumProjection();
		void ManifoldProjection();

		//unit tests
		void UnitTest0();
		void UnitTest1();
		void UnitTest2();
		void UnitTest3();
		void UnitTest4();
		void UnitTest5();
		void UnitTest6();
		void UnitTest7();
		void UnitTest8();
		void UnitTest9();
		void UnitTest10();
		void UnitTest11();
		void UnitTest12();
		void UnitTest13();
		void UnitTest14();
		void HingeJointUnitTest0();

		_Vector q;
		_Vector qdot;
		_Vector x;//used for position solve
		_Vector xOld;
		std::vector<int> jointType;
		std::vector<int> posDOF;
		std::vector<int> xDOF;//used for position solve
		std::vector<int> velDOF;
		std::vector<int> posStartIndex;
		std::vector<int> velStartIndex;
		std::vector<int> xStartIndex;//used for position solve
		std::vector<int> xJointType;
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
		std::vector<_Vector3> hingeDirLocals;
		std::vector<_Vector3> hingeDirGlobals;
		std::vector<_Scalar> hingeMagnitude;//distance between the point from each body that defines the position of the hinge joint
		
		std::vector<_Matrix3> R_global;//rigidbody rotation
		std::vector<_Matrix3> R_local;
		std::vector<_Matrix3> J_rotation;//rotation jabobian matrix
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
		std::vector<_Scalar> constraintValue;
		std::vector<int> jointsID;
		std::vector<int> limitType;
		size_t constraintNum = 0;
		
		std::vector<_Scalar> jointLimit;
		std::vector<std::pair<_Scalar, _Scalar>> jointRange;
		std::vector<_Vector3> twistAxis;
		std::vector<_Vector3> eulerX;
		std::vector<_Vector3> eulerY;
		std::vector<_Vector3> eulerZ;
		std::vector<_Vector3> oldEulerZ;
		std::vector <uint8_t> vectorFieldNum;
		_Matrix J_constraint;
		_Matrix effectiveMass;
		_Scalar swingEpsilon = 0.01;
		bool vectorFieldSwitched = false;
		
		_Scalar kineticEnergy0 = 0;
		_Scalar totalEnergy0 = 0;
		_Vector3 angularMomentum0;
		_Vector3 linearMomentum0;
		//_Vector conservedQuantity;

		int tickCountSimulated = 0;
		int numOfLinks = 0;
		int totalPosDOF = 0;
		int totalVelDOF = 0;
		int totalXDOF = 0;//used for position solve
		int geometry = BOX;

		//debug related parameters
		GameObject* swingArrow = nullptr;
		GameObject* twistArrow = nullptr;
		GameObject* xArrow = nullptr;
		GameObject* yArrow = nullptr;
		GameObject* zArrow = nullptr;
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