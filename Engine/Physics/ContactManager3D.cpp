#include "sRigidBodyState.h"
#include <vector>
namespace eae6320
{
	namespace Physics
	{
		float SqDistPointTriangle(Math::sVector& vPoint, Math::sVector& vA, Math::sVector& vB, Math::sVector& vC)
		{
			float fU, fV, fW;
			Barycentric(vPoint, vA, vB, vC, fU, fV, fW);
			Math::sVector vClosestPoint = vA * fU + vB * fV + vC * fW;
			return (vClosestPoint - vPoint).GetLengthSQ();
		}
		void MergeContact(Contact& i_contact, ContactManifold3D& o_dest)
		{
			float persistentThresholdSQ = 0.01f;

			Collider* colliderA = i_contact.colliderA;
			Collider* colliderB = i_contact.colliderB;
		
			// check persistent contacts
			{
				for (int i = 0; i < o_dest.numContacts; i++)
				{
					Contact &contact = o_dest.m_contacts[i];
					//Math::cMatrix_transformation local2WorldA(colliderA->m_pParentRigidBody->orientation, colliderA->m_pParentRigidBody->position);
					Math::sVector localToGlobalA = colliderA->m_transformation*contact.localPositionA;
					//Math::cMatrix_transformation local2WorldB(colliderB->m_pParentRigidBody->orientation, colliderB->m_pParentRigidBody->position);
					Math::sVector localToGlobalB = colliderB->m_transformation*contact.localPositionB;
					const Math::sVector rA = contact.globalPositionA - localToGlobalA;
					const Math::sVector rB = contact.globalPositionB - localToGlobalB;
					if( Math::Dot(contact.normal, localToGlobalB - localToGlobalA) <= 0.01f
						&& rA.GetLengthSQ() < persistentThresholdSQ
						&& rB.GetLengthSQ() < persistentThresholdSQ)
					{
						// contact persistent, keep
						contact.persistent = true;
						//update penetration depth
						contact.depth = Math::Dot(localToGlobalB - localToGlobalA, contact.normal);
					}
					else
					{
						o_dest.RemoveContactAtIndex(i);
						// dispatch end contact event
					}
				}
			} // end of check persistent contacts

			// process new contacts
			// discard new contact if it's too close to cached contacts
			bool discard = false;
			for (int i = 0; i < o_dest.numContacts; i++)
			{
				if ((i_contact.globalPositionA - o_dest.m_contacts[i].globalPositionA).GetLengthSQ() < persistentThresholdSQ)
				{
					discard = true;
					break;
				}
			}
			if(!discard)
			{
				// add new contact to valid list
				o_dest.AddContact(i_contact);
			}
			
			// repopulate new manifold
			if (o_dest.numContacts > 4) // pick the best of 4
			{
				// 1) find deepest contact first
				Contact deepest = o_dest.m_contacts[0];
				for (int i = 1; i < o_dest.numContacts; ++i)
				{
					if (o_dest.m_contacts[i].depth > deepest.depth)
					{
						deepest = o_dest.m_contacts[i];
					}
				}

				// 2) find furthest contact to form 1D simplex (a line)
				float distSQ = std::numeric_limits<float>::lowest();
				Contact furthest1;
				for (int i = 0; i < o_dest.numContacts; ++i)
				{
					const float currDistSQ = (o_dest.m_contacts[i].globalPositionA - deepest.globalPositionA).GetLengthSQ();
					if (currDistSQ > distSQ)
					{
						furthest1 = o_dest.m_contacts[i];
						distSQ = currDistSQ;
					}
				}

				// 3) expand line to a triangle
				Contact furthest2;
				float distSQ2 = std::numeric_limits<float>::lowest();
				Math::sVector lineDir = furthest1.globalPositionA - deepest.globalPositionA;
				for (int i = 0; i < o_dest.numContacts; ++i)
				{
					// calculate distance from 1D simplex
					const Math::sVector posDiff = o_dest.m_contacts[i].globalPositionA - deepest.globalPositionA;
					const Math::sVector projection = lineDir.GetNormalized()*(Math::Dot(posDiff, lineDir.GetNormalized()));
					const float currDistSQ = (posDiff - projection).GetLengthSQ();
					if (currDistSQ > distSQ2)
					{
						furthest2 = o_dest.m_contacts[i];
						distSQ2 = currDistSQ;
					}
				}

				// 4) blow up manifold using furthest contact from 2D simplex (triangle)
				Contact furthest3;
				float distSQ3 = std::numeric_limits<float>::lowest();
				for (int i = 0; i < o_dest.numContacts; ++i)
				{
					// calculate distance from 1D simplex
					Math::sVector p = o_dest.m_contacts[i].globalPositionA;
					const float currDistSQ = SqDistPointTriangle(p, deepest.globalPositionA, furthest1.globalPositionA, furthest2.globalPositionA);
					if (currDistSQ > distSQ3)
					{
						furthest3 = o_dest.m_contacts[i];
						distSQ3 = currDistSQ;
					}
				}

				o_dest.Clear();
				o_dest.AddContact(deepest);
				o_dest.AddContact(furthest1);
				o_dest.AddContact(furthest2);
				o_dest.AddContact(furthest3);
			}
		}
	}
}