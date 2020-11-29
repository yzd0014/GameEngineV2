#include "HingeJoint.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/cMatrix_transformation.h"
#include "PhysicsSimulation.h"

namespace eae6320
{
	namespace Physics
	{
		void HingeJointsSolver(float i_dt)
		{
			for (size_t i = 0; i < allHingeJoints.size(); i++)
			{
				allHingeJoints[i].ResolveHingJoint(i_dt);
			}
		}
	}
}

void eae6320::Physics::HingeJoint::ResolveHingJoint(float i_dt)
{
	//ball joint solver
	{
		Vector3f v1, v2, w1, w2, x1, x2;
		Math::NativeVector2EigenVector(pActorA->m_State.velocity, v1);
		Math::NativeVector2EigenVector(pActorA->m_State.angularVelocity, w1);
		Math::NativeVector2EigenVector(pActorA->m_State.position, x1);
		Math::NativeVector2EigenVector(pActorB->m_State.velocity, v2);
		Math::NativeVector2EigenVector(pActorB->m_State.angularVelocity, w2);
		Math::NativeVector2EigenVector(pActorB->m_State.position, x2);

		Math::cMatrix_transformation local2WorldRotA(pActorA->m_State.orientation, Math::sVector(0.0f, 0.0f, 0.0f));
		Math::sVector anchorWorldA = local2WorldRotA * anchorLocalA;
		Math::cMatrix_transformation local2WorldRotB(pActorB->m_State.orientation, Math::sVector(0.0f, 0.0f, 0.0f));
		Math::sVector anchorWorldB = local2WorldRotB * anchorLocalB;

		Vector3f r1, r2;
		Math::NativeVector2EigenVector(anchorWorldA, r1);
		Math::NativeVector2EigenVector(anchorWorldB, r2);

		Matrix3f r1Skew;
		Math::GetSkewSymmetricMatrix(r1, r1Skew);
		Matrix3f r2Skew;
		Math::GetSkewSymmetricMatrix(r2, r2Skew);

		Matrix3f I;
		I.setIdentity();
		MatrixXf J(5, 12);
		J.setZero();
		J.block<3, 3>(0, 0) = -I;
		J.block<3, 3>(0, 3) = r1Skew;
		J.block<3, 3>(0, 6) = I;
		J.block<3, 3>(0, 9) = -r2Skew;

		Math::sVector a1 = local2WorldRotA * axisLocaA;
		a1.Normalize();
		Math::sVector a2 = local2WorldRotB * axisLocaB;
		a2.Normalize();
		Math::sVector b2 = Math::GetTangentVector(a2);
		b2.Normalize();
		Math::sVector c2 = Math::Cross(b2, a2).GetNormalized();
		Math::sVector b2xa1_vec = Math::Cross(b2, a1);
		Math::sVector c2xa1_vec = Math::Cross(c2, a1);
		Vector3f b2xa1, c2xa1;
		Math::NativeVector2EigenVector(b2xa1_vec, b2xa1);
		Math::NativeVector2EigenVector(c2xa1_vec, c2xa1);

		J.block<1, 3>(3, 3) = -b2xa1.transpose();
		J.block<1, 3>(3, 9) = b2xa1.transpose();
		J.block<1, 3>(4, 3) = -c2xa1.transpose();
		J.block<1, 3>(4, 9) = c2xa1.transpose();

		Matrix3f inertiaInv_1;
		Math::NativeMatrix2EigenMatrix(pActorA->m_State.globalInverseInertiaTensor, inertiaInv_1);
		Matrix3f inertiaInv_2;
		Math::NativeMatrix2EigenMatrix(pActorB->m_State.globalInverseInertiaTensor, inertiaInv_2);
		float m1 = pActorA->m_State.mass;
		float m2 = pActorB->m_State.mass;
		MatrixXf M_inverse(12, 12);
		M_inverse.setZero();
		M_inverse.block<3, 3>(0, 0) = (1.0f / m1) * I;
		M_inverse.block<3, 3>(3, 3) = inertiaInv_1;
		M_inverse.block<3, 3>(6, 6) = (1.0f / m2) * I;
		M_inverse.block<3, 3>(9, 9) = inertiaInv_2;

		MatrixXf K(5, 5);
		K = J * M_inverse * J.transpose();
		//Matrix3f K;
		//K = (1.0f / m1) * I + r1Skew * inertiaInv_1 * r1Skew.transpose() + (1.0f / m2) * I + r2Skew * inertiaInv_2 * r2Skew.transpose();

		Vector3f b_0;
		float beta_0 = 1.0f;
		Vector3f C_0 = x2 + r2 - x1 - r1;
		b_0 = (beta_0 / i_dt) * C_0;
		Vector2f b_1;
		Vector2f C_1;
		C_1(0) = Math::Dot(a1, b2);
		C_1(1) = Math::Dot(a1, c2);
		float beta_1 = 0.001f;
		b_1 = (beta_1 / i_dt) * C_1;
		VectorXf b(5);
		b.segment(0, 3) = b_0;
		b.segment(3, 2) = -b_1;

		VectorXf V(12);
		V.segment(0, 3) = v1;
		V.segment(3, 3) = w1;
		V.segment(6, 3) = v2;
		V.segment(9, 3) = w2;
		VectorXf lambda(5);
		lambda = K.inverse() * (-J * V - b);

		VectorXf delta_V(12);
		delta_V = M_inverse * J.transpose() * lambda;

		Vector3f delta_v1 = delta_V.segment(0, 3);
		Math::sVector native_delta_v1 = Math::EigenVector2nativeVector(delta_v1);
		Vector3f delta_w1 = delta_V.segment(3, 3);
		Math::sVector native_delta_w1 = Math::EigenVector2nativeVector(delta_w1);
		Vector3f delta_v2 = delta_V.segment(6, 3);
		Math::sVector native_delta_v2 = Math::EigenVector2nativeVector(delta_v2);
		Vector3f delta_w2 = delta_V.segment(9, 3);
		Math::sVector native_delta_w2 = Math::EigenVector2nativeVector(delta_w2);

		if (!pActorA->m_State.isStatic)
		{
			pActorA->m_State.velocity += native_delta_v1;
			pActorA->m_State.angularVelocity += native_delta_w1;
		}
		if (!pActorB->m_State.isStatic)
		{
			pActorB->m_State.velocity += native_delta_v2;
			pActorB->m_State.angularVelocity += native_delta_w2;
		}

	}
	//motor solver
	if (motorEnable)
	{
		Math::cMatrix_transformation local2WorldRotA(pActorA->m_State.orientation, Math::sVector(0, 0, 0));
		Math::sVector a1 = local2WorldRotA * axisLocaA;
		Math::cMatrix_transformation local2WorldRotB(pActorB->m_State.orientation, Math::sVector(0, 0, 0));
		Math::sVector a2 = local2WorldRotB * axisLocaB;
		Math::sVector a = (a1 + a2) / 2.0f;
		a.Normalize();
		Math::sVector w1 = pActorA->m_State.angularVelocity;
		Math::sVector w2 = pActorB->m_State.angularVelocity;
		float JVb = Math::Dot(a, w2 - w1) + w_motor_B2A;
		float K;
		K = Math::Dot(a, pActorA->m_State.globalInverseInertiaTensor * a) + Math::Dot(a, pActorB->m_State.globalInverseInertiaTensor * a);
		float lambda = -JVb / K;

		Math::sVector native_delta_w1 = pActorA->m_State.globalInverseInertiaTensor * -a * lambda;
		Math::sVector native_delta_w2 = pActorB->m_State.globalInverseInertiaTensor * a * lambda;
		if (!pActorA->m_State.isStatic)
		{
			pActorA->m_State.angularVelocity += native_delta_w1;
		}
		if (!pActorB->m_State.isStatic)
		{
			pActorB->m_State.angularVelocity += native_delta_w2;
		}
	}
	/*
	pActorA->m_State.velocity *= 0.9999f;
	pActorA->m_State.angularVelocity *= 0.9999f;
	pActorB->m_State.velocity *= 0.9999f;
	pActorB->m_State.angularVelocity *= 0.9999f;
	*/
}