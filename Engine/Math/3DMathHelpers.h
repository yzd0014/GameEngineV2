#pragma once
#include "Engine/EigenLibrary/Eigen/Dense"

using namespace Eigen;

namespace eae6320
{
	namespace Math
	{
		// i_contact is a point on the plane where the point will be projected
		Vector3d ProjectPointToPlane(Vector3d i_point, Vector3d i_planeNormal, Vector3d i_contact);
		double GetTriangleArea(Vector3d p0, Vector3d p1, Vector3d p2);
		Vector3d PointToTriangleDis(Vector3d p, Vector3d p1, Vector3d p2, Vector3d p3); //returns a point on the triangle
		void Barycentric(Vector3d i_p, Vector3d i_a, Vector3d i_b, Vector3d i_c, double &o_u, double &o_v, double &o_w);
		//returned normal points from edge1 to edge2
		bool EdgeToEdgeDis(Vector3d pa, Vector3d pb, Vector3d pc, Vector3d pd, Vector3d &o_normal, double &o_s, double &o_t);
		bool GetLineTriangleIntersection(Vector3f lineStart, Vector3f lineDirNormalized, Vector3f v1, Vector3f v2, Vector3f v3, Vector3f& o_intersect);
	}
}