#pragma once
#include <vector>
#include <cfloat>
#include <Engine/Math/cQuaternion.h>
#include <Engine/Math/cMatrix_transformation.h>
#include <Engine/Math/sVector.h>
#include "External/EigenLibrary/Eigen/Dense"

using namespace Eigen;

namespace eae6320
{
	enum ColliderType { Box, Sphere };
	namespace Physics
	{
		class Collider;//forward declaration
		struct sRigidBodyState;//forward declaration
		
		struct AABB {
			Math::sVector center;
			Math::sVector extends;
		};
		
		struct Contact
		{
			Math::sVector globalPositionA;
			Math::sVector globalPositionB;
			Math::sVector localPositionA;
			Math::sVector localPositionB;
			Math::sVector normal;
			Math::sVector tangent1, tangent2;
			float depth;
			// for clamping (more on this later)
			float normalImpulseSum = 0.0f;
			float tangentImpulseSum1 = 0.0f;
			float tangentImpulseSum2 = 0.0f;

			//warm start
			float oldNormalLambda = 0.0f;
			float oldTangent1Lambda = 0.0f;
			float oldTangent2Lambda = 0.0f;

			Collider* colliderA;
			Collider* colliderB;
			bool persistent = false;
			bool lambdaCached = false;
			bool enableWarmStart = false;
		};

		class ContactManifold3D
		{
		public:
			void AddContact(Contact& i_contact)
			{
				m_contacts[numContacts] = i_contact;
				numContacts++;
			}
			void RemoveContactAtIndex(int index)
			{
				m_contacts[index] = m_contacts[numContacts - 1];
				numContacts--;
			}

			void Clear()
			{
				numContacts = 0;
			}
			Contact m_contacts[5];//maxmum is 4, 5th one is used to select the best 4
			int numContacts = 0;
		};

		//struct to cache local position of support function result
		struct SupportResult
		{
			Math::sVector globalPosition;
			Math::sVector localPositionA;
			Math::sVector localPositionB;
			Math::sVector m_vec3;//SupportResult serves the purpose of m_vec3 wrapper
		};

		class Simplex
		{
		public:
			std::vector<SupportResult> m_points;

			Simplex() { m_points.clear(); }
			size_t GetSize() { return m_points.size(); }
			void Clear() { m_points.clear(); }
			SupportResult GetA() { return m_points[0]; }
			SupportResult GetB() { return m_points[1]; }
			SupportResult GetC() { return m_points[2]; }
			SupportResult GetD() { return m_points[3]; }
			void RemoveA() { m_points.erase(m_points.begin()); }
			void RemoveB() { m_points.erase(m_points.begin() + 1); }
			void RemoveC() { m_points.erase(m_points.begin() + 2); }
			void RemoveD() { m_points.erase(m_points.begin() + 3); }
			void Add(SupportResult i_data) { m_points.push_back(i_data); }
			SupportResult GetLast() { return m_points[m_points.size() - 1]; }
			bool ContainsOrigin(Math::sVector& i_d);
		};

		class Collider
		{
		public:
			Collider();
			Collider(std::vector<Math::sVector>& i_v, ColliderType i_type);
			Collider(const Collider& i_v);

			void InitializeCollider(AABB &i_box);
			void UpdateTransformation(eae6320::Math::cMatrix_transformation i_t, eae6320::Math::cMatrix_transformation i_rot);
			Math::sVector Center();
			bool IsCollided(Collider& i_B, Contact& o_contact);
			void RemoveManifold(ContactManifold3D* i_pManifold);

			eae6320::Math::cMatrix_transformation m_transformation;
			std::vector<Math::sVector> m_vertices;
			std::vector<ContactManifold3D*> m_pManifolds;
			sRigidBodyState* m_pParentRigidBody;
			ColliderType m_type;
		private:
			Math::cMatrix_transformation m_rotMatrix;
			static SupportResult supportFunction(Collider& i_A, Collider& i_B, Math::sVector i_dir);
			SupportResult getFarthestPointInDirection(Math::sVector i_dir);
			Contact getContact(Simplex&i_simplex, Collider* coll2);
		};

		void MergeContact(Contact& i_contact, ContactManifold3D& o_dest);
	}
}