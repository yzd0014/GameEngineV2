#pragma once
#include "Engine/GameCommon/GameObject.h"
#include <math.h>

#include "Engine/EigenLibrary/Eigen/Dense"
#include "Engine/EigenLibrary/Eigen/Geometry"

using namespace Eigen;
namespace eae6320
{
	class HingeJointCube : public eae6320::GameCommon::GameObject //actually this is a hinge joint
	{
	public: 
		HingeJointCube(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State):
			GameCommon::GameObject(i_pEffect, i_Mesh, i_State)
		{
			uJoinLocal = Vector3f(-1.0f, -1.0f, 0.0f);
			uJointGlobal = uJoinLocal;
			
			m_State.position = -Math::sVector(uJointGlobal(0), uJointGlobal(1), uJointGlobal(2));
			
			M_d.resize(6, 6);
			M_d.setZero();
			float mass = 1.0f;
			M_d(0, 0) = mass;
			M_d(1, 1) = mass;
			M_d(2, 2) = mass;
			localInertiaTensor.setIdentity();
			localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* mass * 8;
			globalInertiaTensor = localInertiaTensor;
			M_d.block<3, 3>(3, 3) = globalInertiaTensor;
		}
		void Tick(const float i_secondCountToIntegrate)
		{
			Vector3f v_z(0, 0, 1);
			Vector3f w = o_dot * v_z;

			MatrixXf H_t;
			H_t.resize(6, 1);
			H_t.setZero();
			H_t.block<3, 1>(0, 0) = uJointGlobal.cross(v_z);
			H_t.block<3, 1>(3, 0) = v_z;

			MatrixXf gamma_t;
			gamma_t.resize(6, 1);
			gamma_t.setZero();
			gamma_t.block<3, 1>(0, 0) =  - w.cross(w.cross(uJointGlobal));
			//cout << gamma_t << endl << endl;

			MatrixXf M_r = H_t.transpose() * M_d * H_t;

			MatrixXf Fe;
			Fe.resize(6, 1);
			Fe.setZero();
			Fe.block<3, 1>(0, 0) = Vector3f(0.0f, -9.8f, 0.0f); //gravity
			//cout << Fe << endl << endl;
			MatrixXf Fv;
			Fv.resize(6, 1);
			Fv.setZero();
			Fv.block<3, 1>(3, 0) = -w.cross(globalInertiaTensor * w);
			//cout << Fv << endl << endl;

			VectorXf Q_r = H_t.transpose() * (Fe + Fv - M_d * gamma_t);
	
			VectorXf Pr_ddot = M_r.inverse() * Q_r;
			//integration
			o_dot = o_dot + Pr_ddot(0) * i_secondCountToIntegrate;
			o_dot *= 0.99f;//damping
			o = o + o_dot * i_secondCountToIntegrate;
	
			//update inertiaTensor and others
			Matrix3f Rt;
			Rt = AngleAxisf(o, Vector3f::UnitZ());
			globalInertiaTensor = Rt * localInertiaTensor * Rt.inverse();
			M_d.block<3, 3>(3, 3) = globalInertiaTensor;
			uJointGlobal = Rt * uJoinLocal;
			
			physicsStateUpdate();
 		}
	private:
		void physicsStateUpdate()
		{
			m_State.orientation = Math::cQuaternion(o, Math::sVector(0, 0, 1));
			m_State.orientation.Normalize();

			m_State.position = -Math::sVector(uJointGlobal(0), uJointGlobal(1), uJointGlobal(2));
		}

	private:
		float o = 0.0f;
		float o_dot = 0.0f;
		MatrixXf M_d;
		Vector3f uJointGlobal;
		Vector3f uJoinLocal;
		Matrix3f localInertiaTensor;
		Matrix3f globalInertiaTensor;
	};
}
