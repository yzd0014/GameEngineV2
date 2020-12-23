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
#include "Engine/EigenLibrary/Eigen/Dense"

using namespace Eigen;
// Struct Declaration
//===================

namespace eae6320
{
	enum ColliderType {Box, Sphere};
	
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
				m_contacts[index] = m_contacts[numContacts-1];
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

		struct sRigidBodyState
		{
			//Physics property
			float mass;
			Math::cMatrix_transformation localInverseInertiaTensor;
			Math::cMatrix_transformation globalInverseInertiaTensor;
			
			// Data
			//=====

			Math::sVector position;	// In arbitrary units determined by the applicaton's convention
			Math::sVector velocity;	// Distance per-second
			Math::sVector acceleration;	// Distance per-second^2
			Math::cQuaternion orientation;
			//Math::sVector angularVelocity_axis_local = Math::sVector( 0.0f, 1.0f, 0.0f );	// In local space (not world space)
			//float angularSpeed = 0.0f;	// Radians per-second (positive values rotate right-handed, negative rotate left-handed)
			Math::sVector angularVelocity;

			//float euler_x = 0.0f; //in degrees;
			//float euler_y = 0.0f;
			//float euler_z = 0.0f;

			//float axis_X_velocity = 0.0f;//degrees per second
			//float axis_Y_velocity = 0.0f;
			//float axis_Z_velocity = 0.0f;

			AABB boundingBox;
			Collider collider;
			bool movementInterpolation = false;
			bool isStatic = false;
			bool hasGravity = false;
			// Interface
			//==========
			sRigidBodyState();
			void Update( const float i_secondCountToIntegrate );
			void UpdatePosition(const float i_secondCountToIntegrate);
			void UpdateVelocity(const float i_secondCountToIntegrate);
			void UpdateOrientation(const float i_secondCountToIntegrate);
			Math::sVector PredictFuturePosition( const float i_secondCountToExtrapolate ) const;
			Math::cQuaternion PredictFutureOrientation( const float i_secondCountToExtrapolate ) const;
		};
		
		void Barycentric(Math::sVector& p, Math::sVector& a, Math::sVector& b, Math::sVector& c, float &u, float &v, float &w);
		Math::sVector GetSurfaceNormal(Math::sVector a, Math::sVector b, Math::sVector c, bool guaranteeOutwards = false);
	}
}

#endif	// EAE6320_PHYSICS_SRIGIDBODYSTATE_H
