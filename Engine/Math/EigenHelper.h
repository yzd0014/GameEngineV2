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

		inline Quaterniond ConertNativeQuatToEigenQuatd(cQuaternion i_quat)
		{
			return Quaterniond(i_quat.w(), i_quat.x(), i_quat.y(), i_quat.z());
		}
		inline Quaternionf ConertNativeQuatToEigenQuatf(cQuaternion i_quat)
		{
			return Quaternionf(i_quat.w(), i_quat.x(), i_quat.y(), i_quat.z());
		}

		inline void QuatIntegrate(Quaternionf& io_quat, const Vector3f& vel, float dt)
		{
			Vector3f deltaRotVec;
			deltaRotVec = vel * dt;
			Quaternionf deltaRot(AngleAxisf(deltaRotVec.norm(), deltaRotVec.normalized()));
			io_quat = deltaRot * io_quat;
			io_quat.normalize();
		}

		inline void QuatIntegrate(Quaterniond& o_quat, Quaterniond& i_quat, const Vector3d& vel, double dt)
		{
			Vector3d deltaRotVec;
			deltaRotVec = vel * dt;
			Quaterniond deltaRot(AngleAxisd(deltaRotVec.norm(), deltaRotVec.normalized()));
			o_quat = deltaRot * i_quat;
			o_quat.normalize();
		}
/*******************Rotation conversion for double***************************************/
		inline Matrix3d RotationConversion_VecToMatrix(const Vector3d& i_vec)
		{
			Matrix3d out;
			out = AngleAxisd(i_vec.norm(), i_vec.normalized());
			return out;
		}

		inline Vector3d RotationConversion_MatrixToVec(const Matrix3d& i_mat)
		{
			AngleAxisd vec;
			vec = i_mat;
			Vector3d out;
			out = vec.angle() * vec.axis();
			return out;
		}

		inline Quaterniond RotationConversion_VecToQuat(const Vector3d& i_vec)
		{
			AngleAxisd vec(i_vec.norm(), i_vec.normalized());
			Quaterniond out;
			out = Quaterniond(vec);
			return out;
		}

		inline Vector3d RotationConversion_QuatToVec(const Quaterniond& i_quat)
		{
			AngleAxisd vec;
			vec = i_quat;
			Vector3d out;
			out = vec.angle() * vec.axis();
			return out;
		}

		inline Matrix3d RotationConversion_QuatToMat(const Quaterniond& i_quat)
		{
			Matrix3d out;
			out = i_quat.toRotationMatrix();
			return out;
		}

		inline Quaterniond RotationConversion_MatToQuat(const Matrix3d& i_mat)
		{
			Quaterniond out;
			out = Quaterniond(i_mat);
			return out;
		}
/*******************Rotation conversion for float***************************************/
		inline Matrix3f RotationConversion_VecToMatrix(const Vector3f& i_vec)
		{
			Matrix3f out;
			out = AngleAxisf(i_vec.norm(), i_vec.normalized());
			return out;
		}

		inline Vector3f RotationConversion_MatrixToVec(const Matrix3f& i_mat)
		{
			AngleAxisf vec;
			vec = i_mat;
			Vector3f out;
			out = vec.angle() * vec.axis();
			return out;
		}

		inline Quaternionf RotationConversion_VecToQuat(const Vector3f& i_vec)
		{
			AngleAxisf vec(i_vec.norm(), i_vec.normalized());
			Quaternionf out;
			out = Quaternionf(vec);
			return out;
		}

		inline Vector3f RotationConversion_QuatToVec(const Quaternionf& i_quat)
		{
			AngleAxisf vec;
			vec = i_quat;
			Vector3f out;
			out = vec.angle() * vec.axis();
			return out;
		}

		inline Matrix3f RotationConversion_QuatToMat(const Quaternionf& i_quat)
		{
			Matrix3f out;
			out = i_quat.toRotationMatrix();
			return out;
		}

		inline Quaternionf RotationConversion_MatToQuat(const Matrix3f& i_mat)
		{
			Quaternionf out;
			out = Quaternionf(i_mat);
			return out;
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

inline Quaternionf operator+(const Quaternionf& lhs, const Quaternionf& rhs)
{
	Quaternionf out(lhs.w() + rhs.w(), lhs.x() + rhs.x(), lhs.y() + rhs.y(), lhs.z() + rhs.z());
	return out;
}

inline Quaterniond operator+(const Quaterniond& lhs, const Quaterniond& rhs)
{
	Quaterniond out(lhs.w() + rhs.w(), lhs.x() + rhs.x(), lhs.y() + rhs.y(), lhs.z() + rhs.z());
	return out;
}