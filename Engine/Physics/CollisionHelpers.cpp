#include "sRigidBodyState.h"
#include "Engine/Math/Functions.h"
#include "Engine/Math/3DMathHelpers.h"
#include "Engine/UserOutput/UserOutput.h"
#include <Engine/Asserts/Asserts.h>

#define EPA_TOLERANCE 0.0001
#define EPA_MAX_NUM_FACES 64
#define EPA_MAX_NUM_LOOSE_EDGES 32
#define EPA_MAX_NUM_ITERATIONS 30
// Interface

eae6320::Physics::SupportResult eae6320::Physics::Collider::getFarthestPointInDirection(Math::sVector i_dir)
{
	SupportResult supportResult;
	if (m_type == Box)
	{
		int selection = 0;
		float maxDist = Math::Dot(m_transformation * m_vertices[0], i_dir);
		for (size_t i = 1; i < m_vertices.size(); i++)
		{
			float dist = Math::Dot(m_transformation * m_vertices[i], i_dir);//(m_transformation * m_vertices[i]).Dot(i_dir);
			if (dist > maxDist)
			{
				maxDist = dist;
				selection = (int)i;
			}
		}
		supportResult.globalPosition = m_transformation * m_vertices[selection];
		supportResult.m_vec3 = m_vertices[selection];//store local position
	}
	else if (m_type == Sphere)
	{
		Math::sVector normalizedSearchDir = i_dir.GetNormalized();
		float r = m_vertices[1].GetLength();
		supportResult.globalPosition = m_transformation * m_vertices[0] + r * normalizedSearchDir;
		Math::cMatrix_transformation world2LocalRot = Math::cMatrix_transformation::CreateWorldToCameraTransform(m_rotMatrix);
		supportResult.m_vec3 = world2LocalRot * (r * normalizedSearchDir);
	}
	return supportResult;
}

void eae6320::Physics::Collider::RemoveManifold(ContactManifold3D* i_pManifold)
{
	for (size_t i = 0; i < m_pManifolds.size(); i++)
	{
		if (m_pManifolds[i] == i_pManifold)
		{
			m_pManifolds[i] = m_pManifolds.back();
			m_pManifolds.pop_back();
			m_pManifolds.shrink_to_fit();
		}
	}
}

eae6320::Physics::SupportResult eae6320::Physics::Collider::supportFunction(Collider&i_A, Collider&i_B, Math::sVector i_dir)
{
	auto a = i_A.getFarthestPointInDirection(i_dir);
	auto b = i_B.getFarthestPointInDirection(i_dir*-1);
	SupportResult supportResult;
	supportResult.globalPosition = a.globalPosition - b.globalPosition;
	supportResult.localPositionA = a.m_vec3;
	supportResult.localPositionB = b.m_vec3;
	return supportResult;
}

eae6320::Math::sVector eae6320::Physics::Collider::Center()
{
	Math::sVector center;
	if (m_type != Sphere)
	{
		int count = 0;
		for (size_t i = 0; i < m_vertices.size(); i++)
		{
			center = center + m_transformation * m_vertices[i];
			count++;
		}
		return center / float(count);
	}
	center = m_transformation * m_vertices[0];
	return center;
}

eae6320::Physics::Contact eae6320::Physics::Collider::getContact(Simplex&i_simplex, Collider* coll2)
{
	Contact contact;//output

	SupportResult faces[EPA_MAX_NUM_FACES][4]; //Array of faces, each with 3 verts and a normal

	//Init with final simplex from GJK
	SupportResult a = i_simplex.GetA();
	SupportResult b = i_simplex.GetB();
	SupportResult c = i_simplex.GetC();
	SupportResult d = i_simplex.GetD();

	faces[0][0] = d;
	faces[0][1] = a;
	faces[0][2] = b;
	//faces[0][3].m_vec3 = (Math::Cross(a.globalPosition - d.globalPosition, b.globalPosition - d.globalPosition)).GetNormalized();
	faces[0][3].m_vec3 = Math::GetSurfaceNormal(d.globalPosition, a.globalPosition, b.globalPosition);
	faces[1][0] = d;
	faces[1][1] = b;
	faces[1][2] = c;
	//faces[1][3].m_vec3 = (Math::Cross(b.globalPosition - d.globalPosition, c.globalPosition - d.globalPosition)).GetNormalized();
	faces[1][3].m_vec3 = Math::GetSurfaceNormal(d.globalPosition, b.globalPosition, c.globalPosition);
	faces[2][0] = d;
	faces[2][1] = c;
	faces[2][2] = a;
	//faces[2][3].m_vec3 = (Math::Cross(c.globalPosition - d.globalPosition, a.globalPosition - d.globalPosition)).GetNormalized();
	faces[2][3].m_vec3 = Math::GetSurfaceNormal(d.globalPosition, c.globalPosition, a.globalPosition);
	faces[3][0] = a;
	faces[3][1] = c;
	faces[3][2] = b;
	//faces[3][3].m_vec3 = (Math::Cross(c.globalPosition - a.globalPosition, b.globalPosition - a.globalPosition)).GetNormalized(); 
	faces[3][3].m_vec3 = Math::GetSurfaceNormal(a.globalPosition, c.globalPosition, b.globalPosition);

	int num_faces = 4;
	int closest_face;

	for (int iterations = 0; iterations < EPA_MAX_NUM_ITERATIONS; iterations++) {
		//Find face that's closest to origin
		float min_dist = Math::Dot(faces[0][0].globalPosition, faces[0][3].m_vec3);
		closest_face = 0;
		for (int i = 1; i < num_faces; i++) {
			float dist = Math::Dot(faces[i][0].globalPosition, faces[i][3].m_vec3);
			if (dist < min_dist) {
				min_dist = dist;
				closest_face = i;
			}
		}

		//search normal to face that's closest to origin
		Math::sVector search_dir = faces[closest_face][3].m_vec3;
		SupportResult p = supportFunction(*this, *coll2, search_dir);

		if (abs(Math::Dot(p.globalPosition, search_dir) - min_dist) < EPA_TOLERANCE) {
			//Convergence (new point is not significantly further from origin)
			Math::sVector contactNormal = faces[closest_face][3].m_vec3 * Math::Dot(p.globalPosition, search_dir);//dot vertex with normal to resolve collision along normal!
			float u, v, w;
			Math::Barycentric(contactNormal, faces[closest_face][0].globalPosition, faces[closest_face][1].globalPosition, faces[closest_face][2].globalPosition, u, v, w);
			contact.localPositionA = u * faces[closest_face][0].localPositionA + v * faces[closest_face][1].localPositionA + w * faces[closest_face][2].localPositionA;
			contact.localPositionB = u * faces[closest_face][0].localPositionB + v * faces[closest_face][1].localPositionB + w * faces[closest_face][2].localPositionB;
			contact.globalPositionA = m_transformation * contact.localPositionA;
			contact.globalPositionB = coll2->m_transformation * contact.localPositionB;
			contact.normal = faces[closest_face][3].m_vec3;
			contact.depth = Math::Dot(p.globalPosition, search_dir);
			contact.tangent1 = Math::GetTangentVector(contact.normal);
			contact.tangent1.Normalize();
			contact.tangent2 = Math::Cross(contact.normal, contact.tangent1).GetNormalized();
			contact.colliderA = this;
			contact.colliderB = coll2;
			return contact;
		}

		SupportResult loose_edges[EPA_MAX_NUM_LOOSE_EDGES][2]; //keep track of edges we need to fix after removing faces
		int num_loose_edges = 0;

		//Find all triangles that are facing p
		for (int i = 0; i < num_faces; i++)
		{
			if (Math::Dot(faces[i][3].m_vec3, p.globalPosition - faces[i][0].globalPosition) > 0) //triangle i faces p, remove it
			{
				//Add removed triangle's edges to loose edge list.
				//If it's already there, remove it (both triangles it belonged to are gone)
				for (int j = 0; j < 3; j++) //Three edges per face
				{
					SupportResult current_edge[2] = { faces[i][j], faces[i][(j + 1) % 3] };
					bool found_edge = false;
					for (int k = 0; k < num_loose_edges; k++) //Check if current edge is already in list
					{
						if (loose_edges[k][1].globalPosition == current_edge[0].globalPosition && loose_edges[k][0].globalPosition == current_edge[1].globalPosition) {
							//Edge is already in the list, remove it
							//THIS ASSUMES EDGE CAN ONLY BE SHARED BY 2 TRIANGLES (which should be true)
							//THIS ALSO ASSUMES SHARED EDGE WILL BE REVERSED IN THE TRIANGLES (which 
							//should be true provided every triangle is wound CCW)
							loose_edges[k][0] = loose_edges[num_loose_edges - 1][0]; //Overwrite current edge
							loose_edges[k][1] = loose_edges[num_loose_edges - 1][1]; //with last edge in list
							num_loose_edges--;
							found_edge = true;
							k = num_loose_edges; //exit loop because edge can only be shared once
						}
					}//endfor loose_edges

					if (!found_edge) { //add current edge to list
						// assert(num_loose_edges<EPA_MAX_NUM_LOOSE_EDGES);
						if (num_loose_edges >= EPA_MAX_NUM_LOOSE_EDGES) break;
						loose_edges[num_loose_edges][0] = current_edge[0];
						loose_edges[num_loose_edges][1] = current_edge[1];
						num_loose_edges++;
					}
				}

				//Remove triangle i from list
				faces[i][0] = faces[num_faces - 1][0];
				faces[i][1] = faces[num_faces - 1][1];
				faces[i][2] = faces[num_faces - 1][2];
				faces[i][3] = faces[num_faces - 1][3];
				num_faces--;
				i--;
			}//endif p can see triangle i
		}//endfor num_faces

		//Reconstruct polytope with p added
		for (int i = 0; i < num_loose_edges; i++)
		{
			float bias = 0.000001f; //in case dot result is only slightly < 0 (because origin is on face)
			// assert(num_faces<EPA_MAX_NUM_FACES);
			if (num_faces >= EPA_MAX_NUM_FACES) break;
			faces[num_faces][0] = loose_edges[i][0];
			faces[num_faces][1] = loose_edges[i][1];
			faces[num_faces][2] = p;
			faces[num_faces][3].m_vec3 = Math::GetSurfaceNormal(loose_edges[i][0].globalPosition, loose_edges[i][1].globalPosition, p.globalPosition);

			//Check for wrong normal to maintain CCW winding
			if (Math::Dot(faces[num_faces][0].globalPosition, faces[num_faces][3].m_vec3) + bias < 0) {
				SupportResult temp = faces[num_faces][0];
				faces[num_faces][0] = faces[num_faces][1];
				faces[num_faces][1] = temp;
				faces[num_faces][3].m_vec3 = -faces[num_faces][3].m_vec3;
			}
			num_faces++;
		}
	} //End for iterations
	UserOutput::DebugPrint("EPA did not converge\n");
	//Return most recent closest point
	Math::sVector contactNormal = faces[closest_face][3].m_vec3 * Math::Dot(faces[closest_face][0].globalPosition, faces[closest_face][3].m_vec3);
	float u, v, w;
	Math::Barycentric(contactNormal, faces[closest_face][0].globalPosition, faces[closest_face][1].globalPosition, faces[closest_face][2].globalPosition, u, v, w);
	contact.localPositionA = u * faces[closest_face][0].localPositionA + v * faces[closest_face][1].localPositionA + w * faces[closest_face][2].localPositionA;
	contact.localPositionB = u * faces[closest_face][0].localPositionB + v * faces[closest_face][1].localPositionB + w * faces[closest_face][2].localPositionB;
	contact.globalPositionA = m_transformation * contact.localPositionA;
	contact.globalPositionB = coll2->m_transformation * contact.localPositionB;
	contact.normal = faces[closest_face][3].m_vec3;
	contact.depth = Math::Dot(faces[closest_face][0].globalPosition, faces[closest_face][3].m_vec3);
	contact.tangent1 = Math::GetTangentVector(contact.normal);
	contact.tangent1.Normalize();
	contact.tangent2 = Math::Cross(contact.normal, contact.tangent1).GetNormalized();
	contact.colliderA = this;
	contact.colliderB = coll2;
	return contact;
}

bool eae6320::Physics::Collider::IsCollided(Collider&i_B, Contact& o_contact)
{
	Math::sVector dir = i_B.Center() - this->Center();
	Simplex simplex;
	simplex.Clear();
	while (true)
	{
		simplex.Add(supportFunction(*this, i_B, dir));

		if (Math::Dot(simplex.GetLast().globalPosition, dir) < 0) {
			return false;
		}
		else {
			if (simplex.ContainsOrigin(dir)) {
				o_contact = getContact(simplex, &i_B);
				return true;
			}
		}
	}
}

bool eae6320::Physics::Simplex::ContainsOrigin(Math::sVector &t_direction)
{
	Math::sVector a, b, c, d;
	Math::sVector ab, ac, ao;
	float bias = 0.000000000001f;
	Math::sVector normal_abo;
	switch (this->GetSize())
	{
	case 1:
		if (this->GetA().globalPosition.GetLengthSQ() < bias)
		{
			t_direction = Math::sVector(1.0f, 0.0f, 0.0f);
		}
		else
		{
			t_direction = this->GetA().globalPosition * -1;
		}
		break;
	case 2:
		a = this->GetA().globalPosition;
		b = this->GetB().globalPosition;
		ab = b - a;
		ao = a * -1;
		normal_abo = Math::Cross(ab, ao);
		if (normal_abo.GetLengthSQ() < bias)
		{
			if ((a - b).GetLengthSQ() < bias) //when line segment is a point
			{
				if (a.GetLengthSQ() < bias) t_direction = Math::sVector(1.0f, 0.0f, 0.0f);
			}
			else
			{
				t_direction = GetTangentVector(a - b).GetNormalized();//when origin is on line segment
			}
		}
		else
		{
			t_direction = Math::Cross(Math::Cross(ab, ao), ab);
		}
		break;

	case 3:
		a = this->GetA().globalPosition;
		b = this->GetB().globalPosition;
		c = this->GetC().globalPosition;
		ab = b - a;
		ac = c - a;
		ao = a * -1;
		t_direction = Math::Cross(ab, ac);
		if (t_direction.GetLengthSQ() > bias)
		{
			if (Math::Dot(t_direction, ao) < 0)
			{
				t_direction = t_direction * -1;
				auto temp = m_points[1];
				m_points[1] = m_points[2];
				m_points[2] = temp;
			}
		}
		else
		{
			t_direction = Math::GetSurfaceNormal(a, b, c, true) * -1;
		}

		break;
	case 4:
		a = this->GetA().globalPosition;
		b = this->GetB().globalPosition;
		c = this->GetC().globalPosition;
		d = this->GetD().globalPosition;
		auto da = a - d;
		auto db = b - d;
		auto dc = c - d;
		auto normal_dab = Math::Cross(da, db);//da.cross(db);
		auto normal_dac = Math::Cross(dc, da);//dc.cross(da);
		auto normal_dbc = Math::Cross(db, dc);//db.cross(dc);
		auto do_ = d * -1; // (0,0,0) - point d
		auto ndab = Math::Dot(normal_dab, do_);//normal_dab.dot(do_);
		auto ndac = Math::Dot(normal_dac, do_);//normal_dac.dot(do_);
		auto ndbc = Math::Dot(normal_dbc, do_);//normal_dbc.dot(do_);
		if (ndab > 0)
		{
			this->RemoveC();
			t_direction = normal_dab;
		}
		else if (ndac > 0)
		{
			this->m_points[1] = this->GetD();
			this->RemoveD();
			t_direction = normal_dac;
		}
		else if (ndbc > 0)
		{
			this->m_points[0] = this->GetD();
			this->RemoveD();
			t_direction = normal_dbc;
		}
		else
		{
			return true;
		}
		break;
	}
	return false;
}

eae6320::Physics::Collider::Collider() { m_vertices.clear(); }

eae6320::Physics::Collider::Collider(std::vector<Math::sVector>& i_v, ColliderType i_type)
{
	m_vertices = i_v;
	m_type = i_type;
}

eae6320::Physics::Collider::Collider(const Collider& i_v)
{
	m_vertices = i_v.m_vertices;
	m_type = i_v.m_type;
}

void eae6320::Physics::Collider::InitializeCollider(AABB &i_box)
{
	m_vertices.resize(8);
	for (int i = 0; i < 8; i++)
	{
		m_vertices[i] = i_box.extends;
	}
	m_vertices[1].z = -m_vertices[1].z;
	m_vertices[2] = m_vertices[1];
	m_vertices[2].x = -m_vertices[2].x;
	m_vertices[3] = m_vertices[2];
	m_vertices[3].z = -m_vertices[3].z;

	m_vertices[4].y = -m_vertices[4].y;
	m_vertices[5] = m_vertices[4];
	m_vertices[5].z = -m_vertices[5].z;
	m_vertices[6] = m_vertices[5];
	m_vertices[6].x = -m_vertices[6].x;
	m_vertices[7] = m_vertices[6];
	m_vertices[7].z = -m_vertices[7].z;
}

void eae6320::Physics::Collider::UpdateTransformation(eae6320::Math::cMatrix_transformation i_t, eae6320::Math::cMatrix_transformation i_rot)
{
	m_transformation = i_t;
	m_rotMatrix = i_rot;

}

void eae6320::Physics::MergeContact(Contact& i_contact, ContactManifold3D& o_dest)
{
	float persistentThresholdSQ = 0.0025f;

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
			float epsilonZero = 0.01f;
			if (Math::Dot(contact.normal, localToGlobalB - localToGlobalA) <= epsilonZero
				&& rA.GetLengthSQ() < persistentThresholdSQ
				&& rB.GetLengthSQ() < persistentThresholdSQ)
			{
				// contact persistent, keep
				contact.persistent = true;
				//update penetration depth
				contact.depth = Math::Dot(localToGlobalA - localToGlobalB, contact.normal);
			}
			else
			{
				o_dest.RemoveContactAtIndex(i);
				i--;
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
	if (!discard)
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
		//rule out colinear first 
		Math::sVector edge1 = furthest1.globalPositionA - deepest.globalPositionA;
		Math::sVector edge2 = furthest2.globalPositionA - deepest.globalPositionA;
		Math::sVector temp = Math::Cross(edge1, edge2);
		bool colinear = false;
		if (temp.GetLengthSQ() < 0.0001f) colinear = true;

		float distSQ3 = std::numeric_limits<float>::lowest();
		for (int i = 0; i < o_dest.numContacts; ++i)
		{
			//handle colinear case
			Math::sVector p = o_dest.m_contacts[i].globalPositionA;
			if (colinear && p != deepest.globalPositionA && p != furthest1.globalPositionA && p != furthest2.globalPositionA)
			{
				furthest3 = o_dest.m_contacts[i];
				break;
			}

			// calculate distance from 1D simplex
			const float currDistSQ = Math::SqDistPointTriangle(p, deepest.globalPositionA, furthest1.globalPositionA, furthest2.globalPositionA);//return nan sometimes
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