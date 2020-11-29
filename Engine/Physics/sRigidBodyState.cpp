// Includes
//=========

#include "sRigidBodyState.h"
#include "Engine/Math/Functions.h"
#include "Engine/UserOutput/UserOutput.h"
#include <Engine/Asserts/Asserts.h>

#define EPA_TOLERANCE 0.0001
#define EPA_MAX_NUM_FACES 64
#define EPA_MAX_NUM_LOOSE_EDGES 32
#define EPA_MAX_NUM_ITERATIONS 64
// Interface
//==========
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

void eae6320::Physics::Barycentric(Math::sVector& p, Math::sVector& a, Math::sVector& b, Math::sVector& c, float &u, float &v, float &w)
{
	Math::sVector v0 = b - a, v1 = c - a, v2 = p - a;
	float d00 = Math::Dot(v0, v0);
	float d01 = Dot(v0, v1);
	float d11 = Dot(v1, v1);
	float d20 = Dot(v2, v0);
	float d21 = Dot(v2, v1);
	float denom = d00 * d11 - d01 * d01;
	v = (d11 * d20 - d01 * d21) / denom;
	w = (d00 * d21 - d01 * d20) / denom;
	u = 1.0f - v - w;
}

eae6320::Math::sVector eae6320::Physics::GetSurfaceNormal(Math::sVector a, Math::sVector b, Math::sVector c)
{
	float bias = 0.000001f;
	Math::sVector faceNormal;
	faceNormal = Math::Cross(b - a, c - a);
	if (faceNormal.GetLength() > bias)
	{
		faceNormal = faceNormal.GetNormalized();
	}
	else if ((a - b).GetLength() < bias && (b - c).GetLength() < bias)
	{// handle case when surface is a point
		faceNormal = a.GetNormalized();
	}
	else
	{//handle case where surface is a line segement
		if ((a - b).GetLength() > bias)
		{
			Math::sVector ab = b - a;
			faceNormal = Math::Cross(Math::Cross(ab, a), ab);
			if (faceNormal.GetLength() > bias)
			{
				faceNormal.Normalize();
			}
			else
			{
				faceNormal = Math::GetTangentVector(a - b).GetNormalized();
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
				faceNormal = Math::GetTangentVector(b - c).GetNormalized();
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
				faceNormal = Math::GetTangentVector(c - a).GetNormalized();
			}
		}
	}
	return faceNormal;
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
	faces[0][3].m_vec3 = GetSurfaceNormal(d.globalPosition,a.globalPosition,b.globalPosition);
	faces[1][0] = d;
	faces[1][1] = b;
	faces[1][2] = c;
	//faces[1][3].m_vec3 = (Math::Cross(b.globalPosition - d.globalPosition, c.globalPosition - d.globalPosition)).GetNormalized();
	faces[1][3].m_vec3 = GetSurfaceNormal(d.globalPosition, b.globalPosition, c.globalPosition);
	faces[2][0] = d;
	faces[2][1] = c;
	faces[2][2] = a;
	//faces[2][3].m_vec3 = (Math::Cross(c.globalPosition - d.globalPosition, a.globalPosition - d.globalPosition)).GetNormalized();
	faces[2][3].m_vec3 = GetSurfaceNormal(d.globalPosition, c.globalPosition, a.globalPosition);
	faces[3][0] = a;
	faces[3][1] = c;
	faces[3][2] = b;
	//faces[3][3].m_vec3 = (Math::Cross(c.globalPosition - a.globalPosition, b.globalPosition - a.globalPosition)).GetNormalized(); 
	faces[3][3].m_vec3 = GetSurfaceNormal(a.globalPosition, c.globalPosition, b.globalPosition);

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
			Barycentric(contactNormal, faces[closest_face][0].globalPosition, faces[closest_face][1].globalPosition, faces[closest_face][2].globalPosition, u, v, w);
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
			faces[num_faces][3].m_vec3 = GetSurfaceNormal(loose_edges[i][0].globalPosition, loose_edges[i][1].globalPosition, p.globalPosition);
			
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
	Barycentric(contactNormal, faces[closest_face][0].globalPosition, faces[closest_face][1].globalPosition, faces[closest_face][2].globalPosition, u, v, w);
	contact.localPositionA = u * faces[closest_face][0].localPositionA + v * faces[closest_face][1].localPositionA + w * faces[closest_face][2].localPositionA;
	contact.localPositionB = u * faces[closest_face][0].localPositionB + v * faces[closest_face][1].localPositionB + w * faces[closest_face][2].localPositionB;
	contact.globalPositionA = m_transformation * contact.localPositionA;
	contact.globalPositionB = m_transformation * contact.localPositionB;
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
	switch (this->GetSize())
	{
	case 1:
		t_direction = t_direction * -1;
		break;
	case 2:
		a = this->GetA().globalPosition;
		b = this->GetB().globalPosition;
		ab = b - a;
		ao = a*-1;
		//t_direction = ab.cross(ao).cross(ab);
		t_direction = Math::Cross(Math::Cross(ab, ao), ab);
		break;

	case 3:
		a = this->GetA().globalPosition;
		b = this->GetB().globalPosition;
		c = this->GetC().globalPosition;
		ab = b - a;
		ac = c - a;
		ao = a*-1;
		//t_direction = ac.cross(ab);
		t_direction = Math::Cross(ac, ab);
		
		if (Math::Dot(t_direction, ao) < 0) t_direction = t_direction * -1;
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
		auto do_ = d*-1; // (0,0,0) - point d
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

eae6320::Physics::sRigidBodyState::sRigidBodyState()
{
	mass = 1.0f;
	//default value is for a 2x2x2 cube
	localInverseInertiaTensor.m_00 = 1.0f / ((1.0f / 12.0f)* mass * 8);
	localInverseInertiaTensor.m_11 = 1.0f / ((1.0f / 12.0f)* mass * 8);
	localInverseInertiaTensor.m_22 = 1.0f / ((1.0f / 12.0f)* mass * 8);
}

void eae6320::Physics::sRigidBodyState::Update( const float i_secondCountToIntegrate )
{
	// Update velocity
	{
		velocity += acceleration * i_secondCountToIntegrate;
	}
	
	// Update position
	{
		position += velocity * i_secondCountToIntegrate;
	}

	// Update orientation
	{
		Math::cQuaternion deltaRot;
		if (angularVelocity.GetLength() > 0.000001f)
		{
			deltaRot = Math::cQuaternion(angularVelocity.GetLength()*i_secondCountToIntegrate, angularVelocity.GetNormalized());
		}
		orientation = deltaRot * orientation;
		orientation.Normalize();
		Math::cMatrix_transformation local2WorldRot(orientation, Math::sVector(0, 0, 0));
		Math::cMatrix_transformation world2LocalRot = Math::cMatrix_transformation::CreateWorldToCameraTransform(local2WorldRot);
		globalInverseInertiaTensor = local2WorldRot * localInverseInertiaTensor * world2LocalRot;
	}
}
void eae6320::Physics::sRigidBodyState::UpdatePosition(const float i_secondCountToIntegrate) {
	position += velocity * i_secondCountToIntegrate;
}

void eae6320::Physics::sRigidBodyState::UpdateVelocity(const float i_secondCountToIntegrate) {
	velocity += acceleration * i_secondCountToIntegrate;
}
void eae6320::Physics::sRigidBodyState::UpdateOrientation(const float i_secondCountToIntegrate) {
	Math::cQuaternion deltaRot;
	if (angularVelocity.GetLength() > 0.000001f)
	{
		deltaRot = Math::cQuaternion(angularVelocity.GetLength()*i_secondCountToIntegrate, angularVelocity.GetNormalized());
	}
	orientation = deltaRot * orientation;
	orientation.Normalize();
	Math::cMatrix_transformation local2WorldRot(orientation, Math::sVector(0, 0, 0));
	Math::cMatrix_transformation world2LocalRot = Math::cMatrix_transformation::CreateWorldToCameraTransform(local2WorldRot);
	globalInverseInertiaTensor = local2WorldRot * localInverseInertiaTensor * world2LocalRot;
}


eae6320::Math::sVector eae6320::Physics::sRigidBodyState::PredictFuturePosition( const float i_secondCountToExtrapolate ) const
{
	return position + (velocity * i_secondCountToExtrapolate );
}

eae6320::Math::cQuaternion eae6320::Physics::sRigidBodyState::PredictFutureOrientation(const float i_secondCountToExtrapolate) const
{
	//const auto rotation = Math::cQuaternion( angularSpeed * i_secondCountToExtrapolate, angularVelocity_axis_local );
	Math::cQuaternion deltaRot;
	if (angularVelocity.GetLength() > 0.000001f)
	{
		deltaRot = Math::cQuaternion(angularVelocity.GetLength()*i_secondCountToExtrapolate, angularVelocity.GetNormalized());
	}
	const auto rotation = deltaRot * orientation;

	return rotation.GetNormalized();
}
