/*
	This struct can be used to represent the state of a
	rigid body in 3D space
*/

#ifndef EAE6320_PHYSICS_SRIGIDBODYSTATE_H
#define EAE6320_PHYSICS_SRIGIDBODYSTATE_H

// Includes
//=========

#include <vector>
#include <cfloat>
#include <Engine/Math/cQuaternion.h>
#include <Engine/Math/cMatrix_transformation.h>
#include <Engine/Math/sVector.h>

// Struct Declaration
//===================

namespace eae6320
{
	namespace Physics
	{
		class Collider;//forward declaration
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
			float normalImpulseSum;
			float tangentImpulseSum1;
			float tangentImpulseSUm2;
			Collider* colliderA;
			Collider* colliderB;
		};
		
		class ContactManifold3D
		{
		public:
			void AddContact(Contact& i_contact)
			{
				m_contacts[numContacts] = i_contact;
				numContacts++;
			}
			void Clear()
			{
				numContacts = 0;
			}
			Contact m_contacts[4];
			int numContacts;
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
			Collider(std::vector<Math::sVector>& i_v);
			Collider(const Collider& i_v);
			
			void InitializeCollider(AABB &i_box);
			void UpdateTransformation(eae6320::Math::cMatrix_transformation i_t);
			Math::sVector Center();
			bool IsCollided(Collider& i_B, Contact& o_contact);

			std::vector<Math::sVector> m_vertices;
		private:
			static SupportResult supportFunction(Collider& i_A, Collider& i_B, Math::sVector i_dir);
			SupportResult getFarthestPointInDirection(Math::sVector i_dir);
			Contact getContact(Simplex&i_simplex, Collider* coll2);
			void Barycentric(Math::sVector p, Math::sVector a, Math::sVector b, Math::sVector c, float &u, float &v, float &w);
			eae6320::Math::cMatrix_transformation m_transformation;
		};

		struct sRigidBodyState
		{
			// Data
			//=====

			Math::sVector position;	// In arbitrary units determined by the applicaton's convention
			Math::sVector velocity;	// Distance per-second
			Math::sVector acceleration;	// Distance per-second^2
			Math::cQuaternion orientation;
			Math::sVector angularVelocity_axis_local = Math::sVector( 0.0f, 1.0f, 0.0f );	// In local space (not world space)
			float angularSpeed = 0.0f;	// Radians per-second (positive values rotate right-handed, negative rotate left-handed)

			float euler_x = 0.0f; //in degrees;
			float euler_y = 0.0f;
			float euler_z = 0.0f;

			float axis_X_velocity = 0.0f;//degrees per second
			float axis_Y_velocity = 0.0f;
			float axis_Z_velocity = 0.0f;

			AABB boundingBox;
			Collider collider;
			// Interface
			//==========

			void Update( const float i_secondCountToIntegrate );
			void UpdatePosition(const float i_secondCountToIntegrate);
			void UpdateVelocity(const float i_secondCountToIntegrate);
			void UpdateOrientation(const float i_secondCountToIntegrate);
			Math::sVector PredictFuturePosition( const float i_secondCountToExtrapolate ) const;
			Math::cQuaternion PredictFutureOrientation( const float i_secondCountToExtrapolate ) const;
		};
		
		static Math::sVector GetTangentVector(Math::sVector n)
		{
			Math::sVector t;
			if (abs(n.x) >= 0.57735f)
			{
				t = { n.y, n.x, 0.0f };
			}
			else
			{
				t = { 0.0f, n.z, n.y };
			}
			return t;
		}

		static Math::sVector GetSurfaceNormal(Math::sVector a, Math::sVector b, Math::sVector c)
		{
			float bias = 0.000001f;
			Math::sVector faceNormal;
			faceNormal = Math::Cross(b - a, c - a);
			if (faceNormal.GetLength() > bias)
			{
				faceNormal = faceNormal.GetNormalized();
			}
			else if ((a-b).GetLength() < bias && (b-c).GetLength() < bias)
			{// handle case when surface is a point
				faceNormal = a.GetNormalized();
			}
			else
			{//handle case where surface is a line segement
				if ((a - b).GetLength() > bias)
				{
					Math::sVector ab = b - a;
					faceNormal = Math::Cross(Math::Cross(ab, a), ab);
					if (faceNormal.GetLength()>bias)
					{
						faceNormal.Normalize();
					}
					else
					{
						faceNormal = GetTangentVector(a-b).GetNormalized();
					}
				}
				else if ((b - c).GetLength() > bias)
				{
					Math::sVector bc = c - b;
					faceNormal = Math::Cross(Math::Cross(bc, b), bc);
					if (faceNormal.GetLength() > bias)
					{
						faceNormal.Normalize();
					}
					else
					{
						faceNormal = GetTangentVector(b-c).GetNormalized();
					}
				}
				else if ((c - a).GetLength() > bias)
				{
					Math::sVector ca = a - c;
					faceNormal = Math::Cross(Math::Cross(ca, c), ca);
					if (faceNormal.GetLength() > bias)
					{
						faceNormal.Normalize();
					}
					else
					{
						faceNormal = GetTangentVector(c-a).GetNormalized();
					}
				}
			}
			return faceNormal;
		}
	}
}

#endif	// EAE6320_PHYSICS_SRIGIDBODYSTATE_H
