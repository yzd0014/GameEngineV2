/*
	This struct represents a position or direction
*/

#ifndef EAE6320_MATH_SVECTOR_H
#define EAE6320_MATH_SVECTOR_H
#include "Engine/EigenLibrary/Eigen/Dense"

using namespace Eigen;
// Struct Declaration
//===================

namespace eae6320
{
	namespace Math
	{
		void GetSkewSymmetricMatrix(Vector3f &i_vecotor, Matrix3f &o_matrix);
		Matrix3f ToSkewSymmetricMatrix(Vector3f &i_vecotor);

		struct sVector
		{
			// Data
			//=====

			float x = 0.0f, y = 0.0f, z = 0.0f;

			// Interface
			//==========

			// Addition
			//---------

			sVector operator +( const sVector i_rhs ) const;
			sVector& operator +=( const sVector i_rhs );

			// Subtraction / Negation
			//-----------------------

			sVector operator -( const sVector i_rhs ) const;
			sVector& operator -=( const sVector i_rhs );
			sVector operator -() const;

			// Multiplication
			//---------------

			sVector operator *( const float i_rhs ) const;
			sVector& operator *=( const float i_rhs );
			friend sVector operator *( const float i_lhs, const sVector i_rhs );

			// Division
			//---------

			sVector operator /( const float i_rhs ) const;
			sVector& operator /=( const float i_rhs );

			// Length / Normalization
			//-----------------------

			float GetLength() const;
			float GetLengthSQ() const;
			float Normalize();
			sVector GetNormalized() const;

			// Products
			//---------

			friend float Dot( const sVector i_lhs, const sVector i_rhs );
			friend sVector Cross( const sVector i_lhs, const sVector i_rhs );

			// Comparison
			//-----------

			bool operator ==( const sVector i_rhs ) const;
			bool operator !=( const sVector i_rhs ) const;

			// Initialization / Shut Down
			//---------------------------

			sVector() = default;
			sVector( const float i_x, const float i_y, const float i_z );
		};

		// Friends
		//========

		sVector operator *( const float i_lhs, const sVector i_rhs );
		float Dot( const sVector i_lhs, const sVector i_rhs );
		sVector Cross( const sVector i_lhs, const sVector i_rhs );

		void NativeVector2EigenVector(sVector i_vector, Vector3f &o_vector);
		Vector3f NativeVector2EigenVector(const sVector &i_vector);
		sVector EigenVector2nativeVector(const Vector3f &i_vector);
		sVector GetTangentVector(sVector n);
	}
}

#endif	// EAE6320_MATH_SVECTOR_H
