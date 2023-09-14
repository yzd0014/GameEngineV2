#pragma once
#include "sVector.h"
#include "cQuaternion.h"
#include "cMatrix_transformation.h"
#include "External/EigenLibrary/Eigen/Dense"

using namespace Eigen;

namespace eae6320
{
	namespace Math
	{
		inline void NativeVector2EigenVector(sVector i_vector, Vector3d &o_vector)
		{
			o_vector(0) = i_vector.x;
			o_vector(1) = i_vector.y;
			o_vector(2) = i_vector.z;
		}

		inline cQuaternion ConvertEigenQuatToNativeQuat(Quaterniond i_quat)
		{
			return eae6320::Math::cQuaternion((float)i_quat.w(), (float)i_quat.x(), (float)i_quat.y(), (float)i_quat.z());
		}
	}
}

inline Quaternionf operator*(const float lhs, const Quaternionf& rhs)
{
	Quaternionf out(rhs.coeffs() * lhs);
	return out;
}

inline Quaterniond operator*(const double lhs, const Quaterniond& rhs)
{
	Quaterniond out(rhs.coeffs() * lhs);
	return out;
}

inline Quaternionf operator*(const Quaternionf& lhs, const float rhs)
{
	Quaternionf out(lhs.coeffs() * rhs);
	return out;
}

inline Quaterniond operator*(const Quaterniond& lhs, const double rhs)
{
	Quaterniond out(lhs.coeffs() * rhs);
	return out;
}