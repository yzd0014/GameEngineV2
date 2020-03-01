#include "PhysicsSimulation.h"
#include "PointJoint.h"
#include "Engine/Math/cMatrix_transformation.h"
#include "Engine/EigenLibrary/Eigen/Dense"

using namespace Eigen;
void eae6320::Physics::PointJointsResolver(float i_dt)
{
	for (size_t i = 0; i < allPointJoints.size(); i++)
	{
		allPointJoints[i].ResolvePointJointConstrain(i_dt);
	}
}

void eae6320::Physics::PointJoint::ResolvePointJointConstrain(float i_dt)
{
	Math::cMatrix_transformation Local2World_rotation(pGameObject->m_State.orientation, Math::sVector(0.0f, 0.0f, 0.0f));
	Math::sVector worldExtend = Local2World_rotation * extend;
	anchor = pParentObject->m_State.position;
	Math::sVector cPosVector = (pGameObject->m_State.position + worldExtend) - anchor;
	Vector3f cPos(cPosVector.x, cPosVector.y, cPosVector.z);
	Math::sVector cVelVector = pGameObject->m_State.velocity + Math::Cross(pGameObject->m_State.angularVelocity, worldExtend);
	Vector3f cVel(cVelVector.x, cVelVector.y, cVelVector.z);

	// compute lamada
	Matrix3f S;
	S.setZero();
	S(0, 1) = worldExtend.z;
	S(0, 2) = -worldExtend.y;
	S(1, 0) = -worldExtend.z;
	S(1, 2) = worldExtend.x;
	S(2, 0) = worldExtend.y;
	S(2, 1) = -worldExtend.x;
	Matrix3f inertiaInv;
	inertiaInv(0, 0) = pGameObject->m_State.globalInverseInertiaTensor.m_00;
	inertiaInv(0, 1) = pGameObject->m_State.globalInverseInertiaTensor.m_01;
	inertiaInv(0, 2) = pGameObject->m_State.globalInverseInertiaTensor.m_02;
	inertiaInv(1, 0) = pGameObject->m_State.globalInverseInertiaTensor.m_10;
	inertiaInv(1, 1) = pGameObject->m_State.globalInverseInertiaTensor.m_11;
	inertiaInv(1, 2) = pGameObject->m_State.globalInverseInertiaTensor.m_12;
	inertiaInv(2, 0) = pGameObject->m_State.globalInverseInertiaTensor.m_20;
	inertiaInv(2, 1) = pGameObject->m_State.globalInverseInertiaTensor.m_21;
	inertiaInv(2, 2) = pGameObject->m_State.globalInverseInertiaTensor.m_22;

	Matrix3f I;
	I.setIdentity();
	float massInv = 1 / pGameObject->m_State.mass;
	Matrix3f effectiveMass = (massInv * I + S * inertiaInv * S.transpose()).inverse();
	float beta = 1.6f;
	Vector3f lamada = effectiveMass * (-cVel - (beta / i_dt)*cPos);

	//correct velocity
	Vector3f dV = (1 / pGameObject->m_State.mass)*lamada;
	Math::sVector dVVector;
	dVVector.x = dV(0);
	dVVector.y = dV(1);
	dVVector.z = dV(2);
	pGameObject->m_State.velocity += dVVector;
	pGameObject->m_State.velocity *= 0.999f;//damping

	Vector3f dA = inertiaInv * S.transpose() * lamada;
	Math::sVector dAVector;
	dAVector.x = dA(0);
	dAVector.y = dA(1);
	dAVector.z = dA(2);
	pGameObject->m_State.angularVelocity += dAVector;
	pGameObject->m_State.angularVelocity *= 0.999f;//damping
}