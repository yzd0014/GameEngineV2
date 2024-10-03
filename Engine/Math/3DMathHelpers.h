#pragma once
#include "External/EigenLibrary/Eigen/Geometry"
#include "External/EigenLibrary/Eigen/Dense"
#include "sVector.h"

using namespace Eigen;

namespace eae6320
{
	namespace Math
	{
		enum RotSeq { zyx, zyz, zxy, zxz, yxz, yxy, yzx, yzy, xyz, xyx, xzy, xzx };
		
		// i_contact is a point on the plane where the point will be projected
		Vector3d ProjectPointToPlane(Vector3d i_point, Vector3d i_planeNormal, Vector3d i_contact);
		double GetTriangleArea(Vector3d p0, Vector3d p1, Vector3d p2);
		Vector3d PointToTriangleDis(Vector3d p, Vector3d p1, Vector3d p2, Vector3d p3); //returns a point on the triangle
		void Barycentric(Vector3d i_p, Vector3d i_a, Vector3d i_b, Vector3d i_c, double &o_u, double &o_v, double &o_w);
		//returned normal points from edge1 to edge2
		bool EdgeToEdgeDis(Vector3d pa, Vector3d pb, Vector3d pc, Vector3d pd, Vector3d &o_normal, double &o_s, double &o_t);
		bool GetLineTriangleIntersection(Vector3f lineStart, Vector3f lineDirNormalized, Vector3f v1, Vector3f v2, Vector3f v3, Vector3f& o_intersect);
		
		Math::sVector GetSurfaceNormal(Math::sVector a, Math::sVector b, Math::sVector c, bool guaranteeOutwards = false);
		void Barycentric(Math::sVector& p, Math::sVector& a, Math::sVector& b, Math::sVector& c, float &u, float &v, float &w);
		float SqDistPointTriangle(Math::sVector& vPoint, Math::sVector& vA, Math::sVector& vB, Math::sVector& vC);
		void ComputeDeformationGradient(Vector3d& i_material0, Vector3d& i_material1, Vector3d& i_material2, Vector3d& i_world0, Vector3d& i_world1, Vector3d& i_world2, Matrix3d& o_F);

		void TwistSwingDecomposition(Matrix3d& i_Rot, Vector3d& i_twistAxis, Matrix3d& o_twist, Matrix3d& o_swing);
		void SwingTwistDecomposition(Quaterniond& i_Rot, Vector3d& i_twistAxis, Quaterniond& o_swing, Quaterniond& o_twist);

		void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]);
		void quaternion2Euler(const Quaterniond& q, double res[], RotSeq rotSeq);
		void rotationMatrix2Euler(const Matrix3d& M, double res[], RotSeq rotSeq);
		/**************************************inline functions************************************************************************/
		inline double GetAngleBetweenTwoVectors(Vector3d& vec0, Vector3d& vec1)
		{
			double dotProduct = vec0.normalized().dot(vec1.normalized());
			if (dotProduct > 1) dotProduct = 1;
			if (dotProduct < -1) dotProduct = -1;
			double angle = acos(dotProduct);

			return angle;
		}

		inline Vector3d GetOrthogonalVector(Vector3d& n)
		{
			Vector3d t;
			if (abs(n.x()) >= 0.57735f)
			{
				t = Vector3d(n.y(), n.x(), 0.0);
			}
			else
			{
				t = Vector3d(0.0, n.z(), n.y());
			}
			return t;
		}
		inline Vector3f GetOrthogonalVector(Vector3f& n)
		{
			Vector3f t;
			if (abs(n.x()) >= 0.57735f)
			{
				t = Vector3f(n.y(), n.x(), 0.0f);
			}
			else
			{
				t = Vector3f(0.0f, n.z(), n.y());
			}
			return t;
		}
	}
}