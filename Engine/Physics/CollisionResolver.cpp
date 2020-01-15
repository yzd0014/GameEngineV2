#include <algorithm> 
#include "sRigidBodyState.h"
#include <vector>
#include "Engine/EigenLibrary/Eigen/Dense"

using namespace Eigen;
namespace eae6320
{
	namespace Physics
	{
		void CollisionResolver(std::vector<ContactManifold3D>& o_allManifolds, float i_dt)
		{
			for (int k = 0; k < 10; k++)//resolve collision for 10 iterations
			{
				for (size_t i = 0; i < o_allManifolds.size(); i++)
				{
					for (int j = 0; j < o_allManifolds[i].numContacts; j++)
					{
						sRigidBodyState* rigidBodyA = o_allManifolds[i].m_contacts[j].colliderA->m_pParentRigidBody;
						sRigidBodyState* rigidBodyB = o_allManifolds[i].m_contacts[j].colliderB->m_pParentRigidBody;
						Math::sVector rA = o_allManifolds[i].m_contacts[j].globalPositionA - rigidBodyA->position;
						Math::sVector rB = o_allManifolds[i].m_contacts[j].globalPositionB - rigidBodyB->position;

						if (k == 0)
						{
							o_allManifolds[i].m_contacts[j].normalImpulseSum = 0;
							o_allManifolds[i].m_contacts[j].tangentImpulseSum1 = 0;
							o_allManifolds[i].m_contacts[j].tangentImpulseSum2 = 0;
						}
						//normal
						{	
							float JV;
							JV = Math::Dot(rigidBodyB->velocity + Math::Cross(rigidBodyB->angularVelocity, rB) - rigidBodyA->velocity - Math::Cross(rigidBodyA->angularVelocity, rA), o_allManifolds[i].m_contacts[j].normal);

							float effectiveMass;
							Math::sVector neRAxN = Math::Cross(-rA, o_allManifolds[i].m_contacts[j].normal);
							Math::sVector poRBxN = Math::Cross(rB, o_allManifolds[i].m_contacts[j].normal);

							effectiveMass = Math::Dot(-o_allManifolds[i].m_contacts[j].normal, -o_allManifolds[i].m_contacts[j].normal*(1 / rigidBodyA->mass))
								+ Math::Dot(neRAxN, rigidBodyA->globalInverseInertiaTensor * neRAxN)
								+ Math::Dot(o_allManifolds[i].m_contacts[j].normal, o_allManifolds[i].m_contacts[j].normal*(1 / rigidBodyB->mass))
								+ Math::Dot(poRBxN, rigidBodyB->globalInverseInertiaTensor * poRBxN);

							float beta = 0.3f;
							float CR = 0.7f;
							float SlopP = 0.0001f;
							float SlopR = 0.5f;
							float b = -beta / i_dt * std::max(o_allManifolds[i].m_contacts[j].depth - SlopP, 0.0f) - CR * std::max(-JV - SlopR, 0.0f);

							float lamada;
							lamada = (-JV - b) / effectiveMass;

							float oldImpulseSum = o_allManifolds[i].m_contacts[j].normalImpulseSum;
							o_allManifolds[i].m_contacts[j].normalImpulseSum = o_allManifolds[i].m_contacts[j].normalImpulseSum + lamada;
							if (o_allManifolds[i].m_contacts[j].normalImpulseSum < 0) o_allManifolds[i].m_contacts[j].normalImpulseSum = 0;
							lamada = o_allManifolds[i].m_contacts[j].normalImpulseSum - oldImpulseSum;
							
							if (!rigidBodyA->isStatic)
							{
								rigidBodyA->velocity = rigidBodyA->velocity + lamada * -o_allManifolds[i].m_contacts[j].normal*(1 / rigidBodyA->mass);
								rigidBodyA->angularVelocity = rigidBodyA->angularVelocity + rigidBodyA->globalInverseInertiaTensor * neRAxN * lamada;
							}
							if (!rigidBodyB->isStatic)
							{
								rigidBodyB->velocity = rigidBodyB->velocity + lamada * o_allManifolds[i].m_contacts[j].normal*(1 / rigidBodyB->mass);
								rigidBodyB->angularVelocity = rigidBodyB->angularVelocity + rigidBodyB->globalInverseInertiaTensor * poRBxN * lamada;
							}	
						}
						
						//fricition 1
						{
							float JV;
							JV = Math::Dot(rigidBodyB->velocity + Math::Cross(rigidBodyB->angularVelocity, rB) - rigidBodyA->velocity - Math::Cross(rigidBodyA->angularVelocity, rA), o_allManifolds[i].m_contacts[j].tangent1);

							float effectiveMass;
							Math::sVector neRAxN = Math::Cross(-rA, o_allManifolds[i].m_contacts[j].tangent1);
							Math::sVector poRBxN = Math::Cross(rB, o_allManifolds[i].m_contacts[j].tangent1);

							effectiveMass = Math::Dot(-o_allManifolds[i].m_contacts[j].tangent1, -o_allManifolds[i].m_contacts[j].tangent1*(1 / rigidBodyA->mass))
								+ Math::Dot(neRAxN, rigidBodyA->globalInverseInertiaTensor * neRAxN)
								+ Math::Dot(o_allManifolds[i].m_contacts[j].tangent1, o_allManifolds[i].m_contacts[j].tangent1*(1 / rigidBodyB->mass))
								+ Math::Dot(poRBxN, rigidBodyB->globalInverseInertiaTensor * poRBxN);

							float lamada;
							lamada = -JV / effectiveMass;

							float CF = 0.8f;
							float oldImpulseT = o_allManifolds[i].m_contacts[j].tangentImpulseSum1;
							o_allManifolds[i].m_contacts[j].tangentImpulseSum1 = o_allManifolds[i].m_contacts[j].tangentImpulseSum1 + lamada;
							if (o_allManifolds[i].m_contacts[j].tangentImpulseSum1 < -o_allManifolds[i].m_contacts[j].normalImpulseSum * CF) o_allManifolds[i].m_contacts[j].tangentImpulseSum1 = -o_allManifolds[i].m_contacts[j].normalImpulseSum * CF;
							else if (o_allManifolds[i].m_contacts[j].tangentImpulseSum1 > o_allManifolds[i].m_contacts[j].normalImpulseSum * CF) o_allManifolds[i].m_contacts[j].tangentImpulseSum1 = o_allManifolds[i].m_contacts[j].normalImpulseSum * CF;
							lamada = o_allManifolds[i].m_contacts[j].tangentImpulseSum1 - oldImpulseT;

							if (!rigidBodyA->isStatic)
							{
								rigidBodyA->velocity = rigidBodyA->velocity + lamada * -o_allManifolds[i].m_contacts[j].tangent1*(1 / rigidBodyA->mass);
								rigidBodyA->angularVelocity = rigidBodyA->angularVelocity + rigidBodyA->globalInverseInertiaTensor * neRAxN * lamada;
							}
							if (!rigidBodyB->isStatic)
							{
								rigidBodyB->velocity = rigidBodyB->velocity + lamada * o_allManifolds[i].m_contacts[j].tangent1*(1 / rigidBodyB->mass);
								rigidBodyB->angularVelocity = rigidBodyB->angularVelocity + rigidBodyB->globalInverseInertiaTensor * poRBxN * lamada;
							}
						}

						//friction 2
						{
							float JV;
							JV = Math::Dot(rigidBodyB->velocity + Math::Cross(rigidBodyB->angularVelocity, rB) - rigidBodyA->velocity - Math::Cross(rigidBodyA->angularVelocity, rA), o_allManifolds[i].m_contacts[j].tangent2);

							float effectiveMass;
							Math::sVector neRAxN = Math::Cross(-rA, o_allManifolds[i].m_contacts[j].tangent2);
							Math::sVector poRBxN = Math::Cross(rB, o_allManifolds[i].m_contacts[j].tangent2);

							effectiveMass = Math::Dot(-o_allManifolds[i].m_contacts[j].tangent2, -o_allManifolds[i].m_contacts[j].tangent2*(1 / rigidBodyA->mass))
								+ Math::Dot(neRAxN, rigidBodyA->globalInverseInertiaTensor * neRAxN)
								+ Math::Dot(o_allManifolds[i].m_contacts[j].tangent2, o_allManifolds[i].m_contacts[j].tangent2*(1 / rigidBodyB->mass))
								+ Math::Dot(poRBxN, rigidBodyB->globalInverseInertiaTensor * poRBxN);

							float lamada;
							lamada = -JV / effectiveMass;

							float CF = 0.8f;
							float oldImpulseT = o_allManifolds[i].m_contacts[j].tangentImpulseSum2;
							o_allManifolds[i].m_contacts[j].tangentImpulseSum2 = o_allManifolds[i].m_contacts[j].tangentImpulseSum2 + lamada;
							if (o_allManifolds[i].m_contacts[j].tangentImpulseSum2 < -o_allManifolds[i].m_contacts[j].normalImpulseSum * CF) o_allManifolds[i].m_contacts[j].tangentImpulseSum2 = -o_allManifolds[i].m_contacts[j].normalImpulseSum * CF;
							else if (o_allManifolds[i].m_contacts[j].tangentImpulseSum2 > o_allManifolds[i].m_contacts[j].normalImpulseSum * CF) o_allManifolds[i].m_contacts[j].tangentImpulseSum2 = o_allManifolds[i].m_contacts[j].normalImpulseSum * CF;
							lamada = o_allManifolds[i].m_contacts[j].tangentImpulseSum2 - oldImpulseT;

							if (!rigidBodyA->isStatic)
							{
								rigidBodyA->velocity = rigidBodyA->velocity + lamada * -o_allManifolds[i].m_contacts[j].tangent2*(1 / rigidBodyA->mass);
								rigidBodyA->angularVelocity = rigidBodyA->angularVelocity + rigidBodyA->globalInverseInertiaTensor * neRAxN * lamada;
							}
							if (!rigidBodyB->isStatic)
							{
								rigidBodyB->velocity = rigidBodyB->velocity + lamada * o_allManifolds[i].m_contacts[j].tangent2*(1 / rigidBodyB->mass);
								rigidBodyB->angularVelocity = rigidBodyB->angularVelocity + rigidBodyB->globalInverseInertiaTensor * poRBxN * lamada;
							}
						}
					}
				}
			}
		} 
	}
}