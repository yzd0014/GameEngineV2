#pragma once
#include "Engine/GameCommon/GameObject.h"
#include "Engine/Application/cbApplication.h"
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
		MultiBody(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, Application::cbApplication* i_application);
		void Tick(const double i_secondCountToIntegrate) override;
		void UpdateGameObjectBasedOnInput() override;

		std::string integrationMode = "Euler";
		_Scalar damping = 1.0;
		int constraintSolverMode = IMPULSE;
		int constraintType = SWING_C;//only used for testing
		int twistMode = EULER_V2;
		bool gravity = false ;
		int enablePositionSolve = 0;//position solve currently doesn't support free joint
		bool adaptiveTimestep = false;//this feature is deprecated, it require update in the application class 
		Application::cbApplication* pApp = nullptr;
	private:
		void InitializeBodies(Assets::cHandle<Mesh> i_mesh, Vector3d i_meshScale, _Matrix3& i_localInertiaTensor, _Vector3 i_partentJointPosition, _Vector3 i_childJointPosition);
		void MultiBodyInitialization();
		void InitializeJoints(int* i_jointType);
		void ConfigurateBallJoint(_Vector3& xAxis, _Vector3& yAxis, _Vector3& zAxis, _Scalar swingAngle, _Scalar twistAngle);
		void ConfigureSingleBallJoint(int bodyNum, _Vector3& xAxis, _Vector3& zAxis, _Scalar swingAngle, _Scalar twistAngle);
		void AddRigidBody(int parent, int i_jointType, _Vector3 jointPositionParent, _Vector3 jointPositionChild, Assets::cHandle<Mesh> i_mesh, Vector3d i_meshScale, _Matrix3& i_localInertiaTensor);
		void SetHingeJoint(int jointNum, _Vector3 hingeDirLocal, _Scalar hingeLength);

		void ComputeMr(_Matrix& o_M, std::vector<_Matrix>& i_Ht);
		void ComputeHt(std::vector<_Matrix>& o_Ht, std::vector<_Matrix>& o_H, _Vector& i_q, std::vector<_Quat>& i_quat, 
			std::vector<int>& i_jointType, std::vector<int>& i_posStartIndex);
		_Vector ComputeQr(_Vector i_qdot);
		_Vector ComputeQr_SikpVelocityUpdate(_Vector& i_qdot);
		void ComputeGamma_t(std::vector<_Vector>& o_gamma_t, _Vector& i_qdot);
		
		void ForwardAngularAndTranslationalVelocity(std::vector<_Matrix> i_Ht, _Vector& i_qdot);
		void ResetExternalForces();
		
		void EulerIntegration(const _Scalar h);
		void RK4Integration(const _Scalar h);
		void RK3Integration(const _Scalar h);
		void VariationalIntegration(const _Scalar h);
		void Integrate_q(_Vector& o_q, std::vector<_Quat>& o_quat, _Vector& i_q, std::vector<_Quat>& i_quat, _Vector& i_qdot, _Scalar h);

		void ForwardKinematics(_Vector& i_q, std::vector<_Quat>& i_quat, std::vector<int>& i_jointType, std::vector<int>& i_posStartIndex);
		void Forward();
		void UpdateBodyRotation(_Vector& i_q, std::vector<_Quat>& i_quat, std::vector<int>& i_jointType, std::vector<int>& i_posStartIndex);
		_Matrix Compute_dHOmega_dr(int joint_id, _Vector& i_x, _Vector i_bj);
		void ComputeJacobianAndInertiaDerivative(int i_totalDOF, _Vector& i_bj, _Vector& i_x, std::vector<_Matrix>& i_Ht, std::vector<_Matrix>& i_H, 
			std::vector<_Matrix>& o_Jacobian, std::vector<_Matrix>& o_intertia);
		void ComputeJacobianAndInertiaDerivativeFD(_Vector& i_bj, std::vector<_Vector>& i_bm, std::vector<_Matrix>& o_Jacobian, std::vector<_Matrix>& o_intertia, _Scalar i_delta);
		void ComputeJacobianAndInertiaDerivativeFDV2(_Vector& i_x, _Vector& i_bj, std::vector<_Matrix>& o_Jacobian, std::vector<_Matrix>& o_intertia, _Scalar i_delta);
		void ComputeDxOverDpFD(std::vector<_Matrix>& o_derivative, _Vector& i_x, _Scalar i_delta);
		void Populate_q(std::vector<_Quat>& i_quat, _Vector& o_q);
		void Populate_quat(_Vector& i_q, std::vector<_Quat>& o_quat, bool normalization);
		void UpdateConstraintJacobian();
		void CopyFromQ2X(std::vector<int>& i_jointType);
		void CopyFromX2Q(std::vector<int>& i_jointType);
		void UpdateXdot(_Vector& o_xdot, _Vector& i_qdot, std::vector<int>& i_jointType);
		void UpdateQdot(_Vector& o_qdot, _Vector& i_qdot, std::vector<int>& i_jointType);
		
		bool ClampRotationVector(_Vector& io_q, _Vector& io_qdot, int i);
		_Scalar ComputeAngularVelocityConstraint(_Vector3& w, _Vector3& p, _Matrix3& Rot, int i_limitType, _Scalar phi);
		_Scalar ComputeKineticEnergy();
		_Scalar ComputePotentialEnergy();
		_Scalar ComputeTotalEnergy();
		_Vector3 ComputeTranslationalMomentum();
		_Vector3 ComputeAngularMomentum();
		
		void SwingLimitCheck();
		void ResolveSwingLimit(const _Scalar h);
		void ResolveSwingLimitPBD(_Vector& i_q, const _Scalar h);
		
		void TwistLimitCheck();
		void ResolveTwistLimit(const _Scalar h);
		void ResolveTwistLimitPBD(_Vector& i_q, const _Scalar h);
		
		void _BallJointLimitCheck();
		void _ResolveJointLimit(const _Scalar h);
		void _ResolveJointLimitPBD(_Vector& i_q, const _Scalar h);
		
		void ConstraintSolve(const _Scalar h);
		void BallJointLimitCheck();
		void SolveVelocityJointLimit(const _Scalar h);
		void PBDStablization();
		void SolvePositionJointLimit();
		_Scalar ComputeSwingError(int jointNum);
		_Scalar ComputeTwistEulerError(int jointNum);
		_Scalar ComputeTwistDirectError(int jointNum);
		void ComputeTwistEulerJacobian(int jointNum, _Matrix& o_J);
		void ComputeTwistEulerJacobian(int i, bool isUpperBound, _Matrix& o_J);
		void ComputeSwingJacobian(int jointNum, _Matrix& o_J);
		void SwitchConstraint(int i);
		void UpdateInitialPosition();//call this function whenever poistion is updated
		void ComputeTwistDirectJacobian(int jointNum, int i_limitType, _Matrix& o_J);
		void ComputeExponentialMapJacobian(_Vector& i_x, std::vector<int>& i_jointType, std::vector<int>& i_posStartIndex);
		

		void PrePositionSolveProccessing();
		void PostPositionSolveProccessing();

		void KineticEnergyProjection();
		void EnergyMomentumProjection();
		void ManifoldProjection();
		void SQP();//defualt FEPR
		void EnergyConstraintV2();//using energy at E0
		void EnergyConstraintV3();//energy constraint only
		void EnergyConstraintPositionVelocity();
		void AcceleratedEnergyConstraint();//quasi-newton
		void AcceleratedEnergyConstraintV2();//add momentum constraint
		
			//unit tests
		void UnitTest5_1();//section 5.1 in the paper
		void UnitTest5_2();//section 5.2 in the paper
		void UnitTest5_3a();//section 5.3 in the paper
		void UnitTest5_3b();//section 5.3 in the paper
		void UnitTest5_4a();//section 5.4 in the paper
		void UnitTest5_4b();//section 5.4 in the paper
		void UnitTest5_5();//section 5.5 in the paper
		void UnitTest5_6();//section 5.6 in the paper
		void UnitTest5_7();//section 5.7 in the paper
		void UnitTest5_8a();//section 5.8 in the paper
		void UnitTest5_8b();//section 5.8 in the paper
		void UnitTest0();
		void HingeJointTest();
		void BallJointTest();
		void GeneralTest();
		void DoubleCubeTest();
		void RunUnitTest();
		
		void FDTest();
		void AnalyticalTest();
		void AnalyticalVsFD();
		void HingeJointUnitTest0();

		void SaveDataToMatlab(_Scalar totalDuration);
		void SaveDataToHoudini(_Scalar totalDuration, _Scalar logInterval, int numOfFrames);

		_Vector q;
		_Vector qdot;
		_Vector x;//used for position solve
		_Vector xdot;
		_Vector qOld;
		_Vector qPre;
		std::vector<int> jointType;
		std::vector<int> posDOF;
		std::vector<int> xDOF;//used for position solve
		std::vector<int> velDOF;
		std::vector<int> posStartIndex;
		std::vector<int> velStartIndex;
		std::vector<int> xStartIndex;//used for position solve
		std::vector<int> xJointType;
		std::vector<int> parentArr;
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
		std::vector<_Vector3> uLocalsChild;
		std::vector<_Vector3> uGlobalsChild;
		std::vector<_Vector3> uLocalsParent;
		std::vector<_Vector3> uGlobalsParent;
		std::vector<_Vector3> hingeDirLocals;
		std::vector<_Vector3> hingeDirGlobals;
		std::vector<_Scalar> hingeMagnitude;//distance between the point from each body that defines the position of the hinge joint
		std::vector<_Vector> externalForces;//extern force in maximal coordinate
		
		std::vector<_Matrix3> R_global;//rigidbody rotation
		std::vector<_Matrix3> R_local;
		std::vector<_Matrix3> J_exp;//rotation jabobian matrix
		std::vector<_Matrix> D;
		std::vector<_Matrix> Ht;
		std::vector<_Matrix> H;
		std::vector<_Matrix> HtDerivativeTimes_b;
		std::vector<_Matrix> MassMatrixDerivativeTimes_b;
		std::vector<_Matrix> mA;
		std::vector<_Matrix> mB;
		std::vector<_Matrix> mE;
		std::vector<_Matrix> mN;
		
		
		std::vector<_Quat> obs_ori;
		std::vector<_Quat> rel_ori;//relative rotation to parent for each body
		std::vector<GameCommon::GameObject *> m_linkBodys;
		_Scalar rigidBodyMass = 1.0f;

		std::vector<_Scalar> g;
		std::vector<_Scalar> constraintValue;
		std::vector<int> jointsID;
		std::vector<int> limitType;
		size_t constraintNum = 0;
		
		std::vector<_Scalar> jointLimit;
		std::vector<std::pair<_Scalar, _Scalar>> jointRange;//first is swing, second is twist
		std::vector<_Vector3> twistAxis;
		std::vector<_Vector3> eulerX;
		std::vector<_Vector3> eulerY;
		std::vector<_Vector3> eulerZ;
		std::vector<_Quat> lastValidOri;
		std::vector<_Scalar> mAlpha;
		std::vector<_Scalar> mBeta;
		std::vector<_Scalar> mGamma;
		std::vector<uint16_t> vectorFieldNum;
		std::vector<_Quat> eulerDecompositionOffset;
		std::vector<_Matrix3> eulerDecompositionOffsetMat;
		_Matrix3 localInertiaTensor;
		_Matrix J_constraint;
		_Matrix effectiveMass0;
		_Matrix effectiveMass1;
		_Scalar swingEpsilon = 1e-6;//0.000001;
		
		std::vector<_Scalar> totalTwist;

		_Scalar kineticEnergy0 = 0;
		_Scalar totalEnergy0 = 0;
		_Vector3 angularMomentum0;
		_Vector3 linearMomentum0;
		_Vector3 gravity_coeff;
		//_Vector conservedQuantity;

		int tickCountSimulated = 0;
		int numOfLinks = 0;
		int totalPosDOF = 0;
		int totalVelDOF = 0;
		int totalXDOF = 0;//used for position solve
		int geometry = BOX;
		/*int frameNum = 12;
		_Scalar animationDuration = 5;*/
		int frameNum = 120;
		_Scalar animationDuration = 5;
		_Scalar totalJointError = 0;
		bool isT0 = true;

		std::function<void()> m_control;
		std::function<void()> m_MatlabSave;
		std::function<void(int frames_number)> m_HoudiniSave;
		std::function<void(FILE * i_pFile)> m_keyPressSave;

		//debug related parameters
		GameObject* swingArrow = nullptr;
		GameObject* twistArrow = nullptr;
		GameObject* xArrow = nullptr;
		GameObject* yArrow = nullptr;
		GameObject* zArrow = nullptr;
/*******************************************************************************************/
		void GetEulerAngles(int jointNum, _Quat i_quat, _Scalar o_eulerAngles[])
		{
			_Quat inputQuat = eulerDecompositionOffset[jointNum] * i_quat * eulerDecompositionOffset[jointNum].inverse();
			Math::quaternion2Euler(inputQuat, o_eulerAngles, Math::RotSeq::yzx);
		}
		
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
		
		inline _Matrix ComputeExponentialMapJacobian(_Matrix3& o_J, _Vector3 r, int i)
		{
			int j = parentArr[i];
			_Scalar theta = r.norm();
			_Scalar b = Compute_b(theta);
			_Scalar a = Compute_a(theta);
			_Scalar c = Compute_c(theta, a);
			o_J = _Matrix::Identity(3, 3) + b * Math::ToSkewSymmetricMatrix(r) + c * Math::ToSkewSymmetricMatrix(r) * Math::ToSkewSymmetricMatrix(r);
			_Matrix3 A;
			if (i == 0) A = o_J;
			else A = R_global[j] * o_J;

			_Matrix out;
			out.resize(6, 3);
			out.setZero();
			out.block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uGlobalsChild[i]) * A;
			out.block<3, 3>(3, 0) = A;

			return out;
		}
	};
}