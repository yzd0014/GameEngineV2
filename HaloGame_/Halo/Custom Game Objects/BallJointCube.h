#pragma once
#include "Engine/GameCommon/GameObject.h"
#include <math.h>

#include "Engine/EigenLibrary/Eigen/Dense"
#include "Engine/EigenLibrary/Eigen/Geometry"

using namespace Eigen;
namespace eae6320
{
	class BallJointCube : public eae6320::GameCommon::GameObject
	{
	public: 
		BallJointCube(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State):
			GameCommon::GameObject(i_pEffect, i_Mesh, i_State)
		{
			U_p = Vector3f(-1.0f, 1.0f, -1.0f);
			m_State.position = -Math::sVector(U_p(0), U_p(1), U_p(2));
			
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
			Vector3f gamma_theta = Vector3f(-o3_dot * sin(o3) * cos(o2) - o2_dot * cos(o3) * sin(o2), o2_dot * cos(o2), -o3_dot * cos(o3) * cos(o2) + o2_dot * sin(o3) * sin(o2)) * o1_dot 
				+ Vector3f(cos(o3), 0.0f, -sin(o3)) * o3_dot * o2_dot;
			//cout << gamma_theta << endl << endl;
			
			Vector3f v_x(cos(o3) * cos(o2), sin(o2), -sin(o3) * cos(o2));
			Vector3f v_z(sin(o3), 0, cos(o3));
			Vector3f v_y(0.0f, 1.0f, 0.0f);
			Vector3f w = o1_dot * v_x + o2_dot * v_z + o3_dot * v_y;

			MatrixXf H_t;
			H_t.resize(6, 3);
			H_t.setZero();
			H_t.block<3, 1>(0, 0) = U_p.cross(v_x);
			H_t.block<3, 1>(0, 1) = U_p.cross(v_z);
			H_t.block<3, 1>(0, 2) = U_p.cross(v_y);
			H_t.block<3, 1>(3, 0) = v_x;
			H_t.block<3, 1>(3, 1) = v_z;
			H_t.block<3, 1>(3, 2) = v_y;

			MatrixXf gamma_t;
			gamma_t.resize(6, 1);
			gamma_t.setZero();
			gamma_t.block<3, 1>(0, 0) = U_p.cross(gamma_theta) - w.cross(w.cross(U_p));
			gamma_t.block<3, 1>(3, 0) = gamma_theta; 
			//cout << gamma_t << endl << endl;

			Matrix3f M_r = H_t.transpose() * M_d * H_t;

			MatrixXf Fe;
			Fe.resize(6, 1);
			Fe.setZero();
			//Fe.block<3, 1>(0, 0) = Vector3f(0.0f, 1.0f, 0.0f); //gravity
			//cout << Fe << endl << endl;
			MatrixXf Fv;
			Fv.resize(6, 1);
			Fv.setZero();
			Fv.block<3, 1>(3, 0) = -w.cross(globalInertiaTensor * w);
			//cout << Fv << endl << endl;

			Vector3f Q_r = H_t.transpose() * (Fe + Fv - M_d * gamma_t);
	
			Vector3f Pr_ddot = M_r.inverse() * Q_r;
			//integration
			o1_dot = o1_dot + Pr_ddot(0) * i_secondCountToIntegrate;
			o2_dot = o2_dot + Pr_ddot(1) * i_secondCountToIntegrate;
			o3_dot = o3_dot + Pr_ddot(2) * i_secondCountToIntegrate;
			o1 = o1 + o1_dot;
			o2 = o2 + o2_dot;
			o3 = o3 + o3_dot;
	
			//update inertiaTensor and others
			Matrix3f Rt;
			Rt = AngleAxisf(o3, Vector3f::UnitY()) * AngleAxisf(o2, Vector3f::UnitZ()) * AngleAxisf(o1, Vector3f::UnitX());
			globalInertiaTensor = Rt * localInertiaTensor * Rt.inverse();
			M_d.block<3, 3>(3, 3) = globalInertiaTensor;
			U_p = Rt * Vector3f(-1.0f, 1.0f, -1.0f);
			
			physicsStateUpdate();
 		}
	private:
		void physicsStateUpdate()
		{
			Math::cQuaternion q1 = Math::cQuaternion(o1, Math::sVector(1, 0, 0));
			Math::cQuaternion q2 = Math::cQuaternion(o2, Math::sVector(0, 0, 1));
			Math::cQuaternion q3 = Math::cQuaternion(o3, Math::sVector(0, 1, 0));
			m_State.orientation = q3 * q2 * q1;
			m_State.orientation.Normalize();

			m_State.position = -Math::sVector(U_p(0), U_p(1), U_p(2));
		}

	private:
		float o1 = 0.0f;
		float o2 = 0.0f;
		float o3 = 0.0f;
		float o1_dot = 0.0f;
		float o2_dot = 0.0f;
		float o3_dot = 0.0f;
		MatrixXf M_d;
		Vector3f U_p;
		Matrix3f localInertiaTensor;
		Matrix3f globalInertiaTensor;
	};
}
