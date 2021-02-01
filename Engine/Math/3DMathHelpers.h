#pragma once
#include "Engine/EigenLibrary/Eigen/Dense"

using namespace Eigen;

namespace eae6320
{
	namespace Math
	{
		// i_contact is a point on the plane where the point will be projected
		Vector3d ProjectPointToPlane(Vector3d i_point, Vector3d i_planeNormal, Vector3d i_contact)
		{
			double a = i_planeNormal(0);
			double b = i_planeNormal(1);
			double c = i_planeNormal(2);
			double d = i_contact(0);
			double e = i_contact(1);
			double f = i_contact(2);
			double x = i_point(0);
			double y = i_point(1);
			double z = i_point(2);

			double s = a * d - a * x + b * e - b * y + c * f - c * z;
			Vector3d projectionPoint = i_point + s * i_planeNormal;

			return projectionPoint;
		}

		double GetTriangleArea(Vector3d p0, Vector3d p1, Vector3d p2)
		{
			Vector3d edge1 = p1 - p0;
			Vector3d edge2 = p2 - p0;
			double area = edge1.cross(edge2).norm() * 0.5;

			return area;
		}

		Vector3d PointToTriangleDis(Vector3d p, Vector3d p1, Vector3d p2, Vector3d p3) //returns a point on the triangle
		{
			Vector3d v21 = p2 - p1;
			Vector3d v32 = p3 - p2;
			Vector3d v13 = p1 - p3;
			Vector3d normal = v21.cross(v13);
			normal.normalize();

			bool insideTriangle = true;
			if (v21.cross(normal).dot(p - p1) < 0) insideTriangle = false;
			if (v32.cross(normal).dot(p - p2) < 0) insideTriangle = false;
			if (v13.cross(normal).dot(p - p3) < 0) insideTriangle = false;

			Vector3d output;
			if (insideTriangle)
			{
				double scale = normal.dot(p - p1);
				output = p - scale * normal;
			}
			else
			{
				double s1 = v21.dot(p - p1) / v21.dot(v21);
				if (s1 < 0) s1 = 0;
				else if (s1 > 1) s1 = 1;
				double d1 = (p - p1 - v21 * s1).squaredNorm();

				double s2 = v32.dot(p - p2) / v32.dot(v32);
				if (s2 < 0) s2 = 0;
				else if (s2 > 1) s2 = 1;
				double d2 = (p - p2 - v32 * s2).squaredNorm();

				double s3 = v13.dot(p - p3) / v13.dot(v13);
				if (s3 < 0) s3 = 0;
				else if (s3 > 1) s3 = 1;
				double d3 = (p - p3 - v13 * s3).squaredNorm();

				if (d1 <= d2 && d1 <= d3)
				{
					output = p1 + s1 * v21;
				}
				else if (d2 <= d1 && d2 <= d3)
				{
					output = p2 + s2 * v32;
				}
				else if (d3 <= d1 && d3 <= d2)
				{
					output = p3 + s3 * v13;
				}
			}

			return output;
		}

		void Barycentric(Vector3d i_p, Vector3d i_a, Vector3d i_b, Vector3d i_c, double &o_u, double &o_v, double &o_w)
		{
			Vector3d v0 = i_b - i_a;
			Vector3d v1 = i_c - i_a;
			Vector3d v2 = i_p - i_a;

			double d00 = v0.dot(v0);
			double d01 = v0.dot(v1);
			double d11 = v1.dot(v1);
			double d20 = v2.dot(v0);
			double d21 = v2.dot(v1);

			double denom = d00 * d11 - d01 * d01;
			o_v = (d11 * d20 - d01 * d21) / denom;
			o_w = (d00 * d21 - d01 * d20) / denom;
			o_u = 1.0f - o_v - o_w;
		}

		//returned normal points from edge1 to edge2
		bool EdgeToEdgeDis(Vector3d pa, Vector3d pb, Vector3d pc, Vector3d pd, Vector3d &o_normal, double &o_s, double &o_t)
		{
			//check if two lines are parallel or not first
			MatrixXd tetra(3, 3);
			tetra.row(0) = pa.transpose() - pb.transpose();
			tetra.row(1) = pb.transpose() - pc.transpose();
			tetra.row(2) = pc.transpose() - pd.transpose();

			double tertraArea = fabs(tetra.determinant() / 6.0);
			if (tertraArea <= 0.00001)
			{
				o_s = -1;
				o_t = -1;
				return false;
			}

			Vector3d d1 = pb - pa;
			Vector3d d2 = pd - pc;
			Vector3d virtualNormal = d1.cross(d2);

			Vector3d n2 = d2.cross(virtualNormal);
			Vector3d c1;
			c1 = pa + (pc - pa).dot(n2) / d1.dot(n2) * d1;

			Vector3d n1 = d1.cross(virtualNormal);
			Vector3d c2;
			c2 = pc + (pa - pc).dot(n1) / d2.dot(n1) * d2;

			Vector3d v1Cut = c1 - pa;
			if (v1Cut.dot(d1) < 0)
			{
				o_s = 0;
			}
			else
			{
				o_s = v1Cut.norm() / d1.norm();
				if (o_s > 1) o_s = 1;
			}

			Vector3d v2Cut = c2 - pc;
			if (v2Cut.dot(d2) < 0)
			{
				o_t = 0;
			}
			else
			{
				o_t = v2Cut.norm() / d2.norm();
				if (o_t > 1) o_t = 1;
			}

			Vector3d pAlpha = pa + o_s * (pb - pa);
			Vector3d pBeta = pc + o_t * (pd - pc);
			o_normal = (pBeta - pAlpha).normalized();

			return true;
		}
	}
}