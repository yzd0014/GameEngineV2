#pragma once
#include "Engine/GameCommon/GameObject.h"
#include "Engine/Math/sVector.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#include <math.h>

#include "Engine/EigenLibrary/Eigen/Dense"
#include "Engine/EigenLibrary/Eigen/Geometry"
using namespace Eigen;
namespace eae6320
{
	class SphericalJoint : public eae6320::GameCommon::GameObject
	{
	public:
		SphericalJoint(Effect * i_pEffect, Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State):
			GameCommon::GameObject(i_pEffect, i_Mesh, i_State)
		{
			uJointLocal = Vector3f(1.0f, -1.0f, -1.0f);

			M_d.resize(6, 6);
			M_d.setZero();
			float mass = 1.0f;
			M_d(0, 0) = mass;
			M_d(1, 1) = mass;
			M_d(2, 2) = mass;
			localInertiaTensor.setIdentity();
			localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* mass * 8;

			r_dot.setZero();
			r.setZero();
			r = Vector3f(0.7f, 4.0f, 0.4f);
			//r = Vector3f(0.0f, 0.4f, 0.0f);
			physicsStateUpdate();
		}
		void Tick(const float i_secondCountToIntegrate)
		{
			float theta = r.norm();
			float a;
			if (theta < 0.015) a = 1.0f - theta / 6.0f;
			else a = sin(theta) / theta;

			float b;
			if (theta < 0.015) b = 0.5f - theta * theta / 24.0f;
			else b = (1.0f - cos(theta)) / (theta * theta);

			float c;
			if (theta < 0.015) c = 1.0f / 6.0f - theta * theta / 120.0f;
			else c = (1.0f - a) / (theta * theta);
			
			float a_dot;
			if (theta < 0.001) a_dot = (-1.0f / 3.0f + 1.0f / 30.0f * theta * theta) * r.dot(r_dot);
			else a_dot = (c - b) * r.dot(r_dot);

			float b_dot;
			if (theta < 0.001) b_dot = (-1.0f / 12.0f + 1.0f / 180.0f * theta * theta) * r.dot(r_dot);
			else b_dot = (a - 2.0f * b) / (theta * theta) * r.dot(r_dot);

			float c_dot;
			if (theta < 0.001) c_dot = (-1.0f / 60.0f + 1.0f / 1260.0f * theta * theta) * r.dot(r_dot);
			else c_dot = (b - 3.0f * c) / (theta * theta) * r.dot(r_dot);
			
			Vector3f w;
			w = a * r_dot - (b * r_dot).cross(r) + c * r.dot(r_dot) * r;

			MatrixXf T;
			T.resize(3, 3);
			T.setZero();
			MatrixXf I;
			I.resize(3, 3);
			I.setIdentity();
			Matrix3f rrt = r * r.transpose();
			T = a * I + b * Math::ToSkewSymmetricMatrix(r) + c * rrt;
			Vector3f gamma;
			gamma = (c * r.dot(r_dot) + a_dot) * r_dot - (b_dot * r_dot).cross(r) + (c_dot * r.dot(r_dot) + c * r_dot.norm() * r_dot.norm()) * r;

			MatrixXf H_t;
			H_t.resize(6, 3);
			H_t.setZero();
			H_t.block<3, 3>(0, 0) = Math::ToSkewSymmetricMatrix(uJointGlobal) * T;
			H_t.block<3, 3>(3, 0) = T;

			MatrixXf Gamma_t;
			Gamma_t.resize(6, 1);
			Gamma_t.setZero();
			Gamma_t.block<3, 1>(0, 0) = Math::ToSkewSymmetricMatrix(uJointGlobal) * gamma - w.cross(w.cross(uJointGlobal));
			Gamma_t.block<3, 1>(3, 0) = gamma;

			MatrixXf M_r = H_t.transpose() * M_d * H_t;

			MatrixXf Fe;
			Fe.resize(6, 1);
			Fe.setZero();
			Fe.block<3, 1>(0, 0) = Vector3f(0.0f, -9.8f, 0.0f) + F_user; //gravity
			//cout << Fe << endl << endl;
			MatrixXf Fv;
			Fv.resize(6, 1);
			Fv.setZero();
			Fv.block<3, 1>(3, 0) = -w.cross(globalInertiaTensor * w);

			VectorXf Q_r = H_t.transpose() * (Fe + Fv - M_d * Gamma_t);

			VectorXf Pr_ddot = M_r.inverse() * Q_r;
			//integration
			r_dot = r_dot + Pr_ddot * i_secondCountToIntegrate;
			r_dot *= 0.99f;
			r = r + r_dot * i_secondCountToIntegrate;

			physicsStateUpdate();
		}
		void UpdateGameObjectBasedOnInput() override
		{
			if (UserInput::KeyState::currFrameKeyState[UserInput::KeyCodes::LeftMouseButton])
			{
				Math::sVector dir_t = GameplayUtility::MouseRayCasting();
				F_user = 10.0f * Math::NativeVector2EigenVector(dir_t);
			}
		}
	private:
		void physicsStateUpdate()
		{
			//update inertiaTensor and others
			Matrix3f Rt;
			Rt = AngleAxisf(r.norm(), r.normalized());
			globalInertiaTensor = Rt * localInertiaTensor * Rt.transpose();
			M_d.block<3, 3>(3, 3) = globalInertiaTensor;
			uJointGlobal = Rt * uJointLocal;
			
			m_State.orientation = Math::cQuaternion(r.norm(), Math::EigenVector2nativeVector(r.normalized()));
			m_State.orientation.Normalize();
			m_State.position = -Math::sVector(uJointGlobal(0), uJointGlobal(1), uJointGlobal(2));
			F_user.setZero(); //clear external force
		}
	private:
		Vector3f r;
		Vector3f r_dot;
		Vector3f F_user;
		MatrixXf M_d;
		Vector3f uJointLocal;
		Vector3f uJointGlobal;
		Matrix3f localInertiaTensor;
		Matrix3f globalInertiaTensor;
	};
}