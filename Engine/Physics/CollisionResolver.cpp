#include <algorithm> 
#include "sRigidBodyState.h"
#include <vector>
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/EigenLibrary/Eigen/Dense"

using namespace Eigen;
namespace eae6320
{
	namespace Physics
	{
		void CollisionResolver(float i_dt, int k)
		{
			
			for (size_t i = 0; i < allManifolds.size(); i++)
			{
				for (int j = 0; j < allManifolds[i].numContacts; j++)
				{
					sRigidBodyState* rigidBodyA = allManifolds[i].m_contacts[j].colliderA->m_pParentRigidBody;
					sRigidBodyState* rigidBodyB = allManifolds[i].m_contacts[j].colliderB->m_pParentRigidBody;
					Math::sVector rA = allManifolds[i].m_contacts[j].globalPositionA - rigidBodyA->position;
					Math::sVector rB = allManifolds[i].m_contacts[j].globalPositionB - rigidBodyB->position;

					if (k == 0)
					{
						allManifolds[i].m_contacts[j].normalImpulseSum = 0;
						allManifolds[i].m_contacts[j].tangentImpulseSum1 = 0;
						allManifolds[i].m_contacts[j].tangentImpulseSum2 = 0;
					}
					//normal
					{	
						float JV;
						JV = Math::Dot(rigidBodyB->velocity + Math::Cross(rigidBodyB->angularVelocity, rB) - rigidBodyA->velocity - Math::Cross(rigidBodyA->angularVelocity, rA), allManifolds[i].m_contacts[j].normal);

						float effectiveMass;
						Math::sVector neRAxN = Math::Cross(-rA, allManifolds[i].m_contacts[j].normal);
						Math::sVector poRBxN = Math::Cross(rB, allManifolds[i].m_contacts[j].normal);

						effectiveMass = Math::Dot(-allManifolds[i].m_contacts[j].normal, -allManifolds[i].m_contacts[j].normal*(1 / rigidBodyA->mass))
							+ Math::Dot(neRAxN, rigidBodyA->globalInverseInertiaTensor * neRAxN)
							+ Math::Dot(allManifolds[i].m_contacts[j].normal, allManifolds[i].m_contacts[j].normal*(1 / rigidBodyB->mass))
							+ Math::Dot(poRBxN, rigidBodyB->globalInverseInertiaTensor * poRBxN);

						float beta = 0.1f;
						float CR = 0.7f;//0.7f
						float SlopP = 0.001f;
						float SlopR = 0.5f;
						float b = -beta / i_dt * std::max(allManifolds[i].m_contacts[j].depth - SlopP, 0.0f) - CR * std::max(-JV - SlopR, 0.0f);
						
						float lambda;
						lambda = (-JV - b) / effectiveMass;

						float oldImpulseSum = allManifolds[i].m_contacts[j].normalImpulseSum;
						allManifolds[i].m_contacts[j].normalImpulseSum = allManifolds[i].m_contacts[j].normalImpulseSum + lambda;
						if (allManifolds[i].m_contacts[j].normalImpulseSum < 0) allManifolds[i].m_contacts[j].normalImpulseSum = 0;
						lambda = allManifolds[i].m_contacts[j].normalImpulseSum - oldImpulseSum;
							
						if (allManifolds[i].m_contacts[j].lambdaCached && k == 0 && allManifolds[i].m_contacts[j].enableWarmStart)
						{
							lambda = 0.8f * allManifolds[i].m_contacts[j].oldNormalLambda + 0.2f * lambda;
						}
						if (k == constraintMaxNum - 1)
						{
							allManifolds[i].m_contacts[j].oldNormalLambda = lambda;
							allManifolds[i].m_contacts[j].lambdaCached = true;
						}
						
						if (!rigidBodyA->isStatic)
						{
							rigidBodyA->velocity = rigidBodyA->velocity + lambda * -allManifolds[i].m_contacts[j].normal*(1 / rigidBodyA->mass);
							rigidBodyA->angularVelocity = rigidBodyA->angularVelocity + rigidBodyA->globalInverseInertiaTensor * neRAxN * lambda;
						}
						if (!rigidBodyB->isStatic)
						{
							rigidBodyB->velocity = rigidBodyB->velocity + lambda * allManifolds[i].m_contacts[j].normal*(1 / rigidBodyB->mass);
							rigidBodyB->angularVelocity = rigidBodyB->angularVelocity + rigidBodyB->globalInverseInertiaTensor * poRBxN * lambda;
						}	
					}
						
					//fricition 1
					{
						float JV;
						JV = Math::Dot(rigidBodyB->velocity + Math::Cross(rigidBodyB->angularVelocity, rB) - rigidBodyA->velocity - Math::Cross(rigidBodyA->angularVelocity, rA), allManifolds[i].m_contacts[j].tangent1);

						float effectiveMass;
						Math::sVector neRAxN = Math::Cross(-rA, allManifolds[i].m_contacts[j].tangent1);
						Math::sVector poRBxN = Math::Cross(rB, allManifolds[i].m_contacts[j].tangent1);

						effectiveMass = Math::Dot(-allManifolds[i].m_contacts[j].tangent1, -allManifolds[i].m_contacts[j].tangent1*(1 / rigidBodyA->mass))
							+ Math::Dot(neRAxN, rigidBodyA->globalInverseInertiaTensor * neRAxN)
							+ Math::Dot(allManifolds[i].m_contacts[j].tangent1, allManifolds[i].m_contacts[j].tangent1*(1 / rigidBodyB->mass))
							+ Math::Dot(poRBxN, rigidBodyB->globalInverseInertiaTensor * poRBxN);

						float lambda;
						lambda = -JV / effectiveMass;

						float CF = 50.0f;
						float oldImpulseT = allManifolds[i].m_contacts[j].tangentImpulseSum1;
						allManifolds[i].m_contacts[j].tangentImpulseSum1 = allManifolds[i].m_contacts[j].tangentImpulseSum1 + lambda;
						if (allManifolds[i].m_contacts[j].tangentImpulseSum1 < -allManifolds[i].m_contacts[j].normalImpulseSum * CF) allManifolds[i].m_contacts[j].tangentImpulseSum1 = -allManifolds[i].m_contacts[j].normalImpulseSum * CF;
						else if (allManifolds[i].m_contacts[j].tangentImpulseSum1 > allManifolds[i].m_contacts[j].normalImpulseSum * CF) allManifolds[i].m_contacts[j].tangentImpulseSum1 = allManifolds[i].m_contacts[j].normalImpulseSum * CF;
						lambda = allManifolds[i].m_contacts[j].tangentImpulseSum1 - oldImpulseT;

						if (allManifolds[i].m_contacts[j].lambdaCached && k == 0 && allManifolds[i].m_contacts[j].enableWarmStart)
						{
							lambda = 0.8f * allManifolds[i].m_contacts[j].oldTangent1Lambda + 0.2f * lambda;
						}
						if (k == constraintMaxNum - 1)
						{
							allManifolds[i].m_contacts[j].oldTangent1Lambda = lambda;
						}

						if (!rigidBodyA->isStatic)
						{
							rigidBodyA->velocity = rigidBodyA->velocity + lambda * -allManifolds[i].m_contacts[j].tangent1*(1 / rigidBodyA->mass);
							rigidBodyA->angularVelocity = rigidBodyA->angularVelocity + rigidBodyA->globalInverseInertiaTensor * neRAxN * lambda;
						}
						if (!rigidBodyB->isStatic)
						{
							rigidBodyB->velocity = rigidBodyB->velocity + lambda * allManifolds[i].m_contacts[j].tangent1*(1 / rigidBodyB->mass);
							rigidBodyB->angularVelocity = rigidBodyB->angularVelocity + rigidBodyB->globalInverseInertiaTensor * poRBxN * lambda;
						}
					}

					//friction 2
					{
						float JV;
						JV = Math::Dot(rigidBodyB->velocity + Math::Cross(rigidBodyB->angularVelocity, rB) - rigidBodyA->velocity - Math::Cross(rigidBodyA->angularVelocity, rA), allManifolds[i].m_contacts[j].tangent2);

						float effectiveMass;
						Math::sVector neRAxN = Math::Cross(-rA, allManifolds[i].m_contacts[j].tangent2);
						Math::sVector poRBxN = Math::Cross(rB, allManifolds[i].m_contacts[j].tangent2);

						effectiveMass = Math::Dot(-allManifolds[i].m_contacts[j].tangent2, -allManifolds[i].m_contacts[j].tangent2*(1 / rigidBodyA->mass))
							+ Math::Dot(neRAxN, rigidBodyA->globalInverseInertiaTensor * neRAxN)
							+ Math::Dot(allManifolds[i].m_contacts[j].tangent2, allManifolds[i].m_contacts[j].tangent2*(1 / rigidBodyB->mass))
							+ Math::Dot(poRBxN, rigidBodyB->globalInverseInertiaTensor * poRBxN);

						float lambda;
						lambda = -JV / effectiveMass;

						float CF = 50.0f;
						float oldImpulseT = allManifolds[i].m_contacts[j].tangentImpulseSum2;
						allManifolds[i].m_contacts[j].tangentImpulseSum2 = allManifolds[i].m_contacts[j].tangentImpulseSum2 + lambda;
						if (allManifolds[i].m_contacts[j].tangentImpulseSum2 < -allManifolds[i].m_contacts[j].normalImpulseSum * CF) allManifolds[i].m_contacts[j].tangentImpulseSum2 = -allManifolds[i].m_contacts[j].normalImpulseSum * CF;
						else if (allManifolds[i].m_contacts[j].tangentImpulseSum2 > allManifolds[i].m_contacts[j].normalImpulseSum * CF) allManifolds[i].m_contacts[j].tangentImpulseSum2 = allManifolds[i].m_contacts[j].normalImpulseSum * CF;
						lambda = allManifolds[i].m_contacts[j].tangentImpulseSum2 - oldImpulseT;

						if (allManifolds[i].m_contacts[j].lambdaCached && k == 0 && allManifolds[i].m_contacts[j].enableWarmStart)
						{
							lambda = 0.8f * allManifolds[i].m_contacts[j].oldTangent2Lambda + 0.2f * lambda;
						}
						if (k == constraintMaxNum - 1)
						{
							allManifolds[i].m_contacts[j].oldTangent2Lambda = lambda;
						}

						if (!rigidBodyA->isStatic)
						{
							rigidBodyA->velocity = rigidBodyA->velocity + lambda * -allManifolds[i].m_contacts[j].tangent2*(1 / rigidBodyA->mass);
							rigidBodyA->angularVelocity = rigidBodyA->angularVelocity + rigidBodyA->globalInverseInertiaTensor * neRAxN * lambda;
						}
						if (!rigidBodyB->isStatic)
						{
							rigidBodyB->velocity = rigidBodyB->velocity + lambda * allManifolds[i].m_contacts[j].tangent2*(1 / rigidBodyB->mass);
							rigidBodyB->angularVelocity = rigidBodyB->angularVelocity + rigidBodyB->globalInverseInertiaTensor * poRBxN * lambda;
						}
					}
				}
			}
		} 
	}
}