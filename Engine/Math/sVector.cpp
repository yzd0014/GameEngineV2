// Includes
//=========

#include "sVector.h"

#include <cmath>
#include <Engine/Asserts/Asserts.h>

// Static Data Initialization
//===========================

namespace
{
	constexpr auto s_epsilon = 1.0e-9f;
}

// Interface
//==========

// Addition
//---------

sca2025::Math::sVector sca2025::Math::sVector::operator +( const sVector i_rhs ) const
{
	return sVector( x + i_rhs.x, y + i_rhs.y, z + i_rhs.z );
}

sca2025::Math::sVector& sca2025::Math::sVector::operator +=( const sVector i_rhs )
{
	x += i_rhs.x;
	y += i_rhs.y;
	z += i_rhs.z;
	return *this;
}

// Subtraction / Negation
//-----------------------

sca2025::Math::sVector sca2025::Math::sVector::operator -( const sVector i_rhs ) const
{
	return sVector( x - i_rhs.x, y - i_rhs.y, z - i_rhs.z );
}

sca2025::Math::sVector& sca2025::Math::sVector::operator -=( const sVector i_rhs )
{
	x -= i_rhs.x;
	y -= i_rhs.y;
	z -= i_rhs.z;
	return *this;
}

sca2025::Math::sVector sca2025::Math::sVector::operator -() const
{
	return sVector( -x, -y, -z );
}

// Multiplication
//---------------

sca2025::Math::sVector sca2025::Math::sVector::operator *( const float i_rhs ) const
{
	return sVector( x * i_rhs, y * i_rhs, z * i_rhs );
}

sca2025::Math::sVector& sca2025::Math::sVector::operator *=( const float i_rhs )
{
	x *= i_rhs;
	y *= i_rhs;
	z *= i_rhs;
	return *this;
}

sca2025::Math::sVector sca2025::Math::operator *( const float i_lhs, const sca2025::Math::sVector i_rhs )
{
	return i_rhs * i_lhs;
}

// Division
//---------

sca2025::Math::sVector sca2025::Math::sVector::operator /( const float i_rhs ) const
{
	EAE6320_ASSERTF( std::abs( i_rhs ) > s_epsilon, "Can't divide by zero" );
	return sVector( x / i_rhs, y / i_rhs, z / i_rhs );
}

sca2025::Math::sVector& sca2025::Math::sVector::operator /=( const float i_rhs )
{
	EAE6320_ASSERTF( std::abs( i_rhs ) > s_epsilon, "Can't divide by zero" );
	x /= i_rhs;
	y /= i_rhs;
	z /= i_rhs;
	return *this;
}

// Length / Normalization
//-----------------------

float sca2025::Math::sVector::GetLength() const
{
	const auto length_squared = ( x * x ) + ( y * y ) + ( z * z );
	EAE6320_ASSERTF( length_squared >= 0.0f, "Can't take a square root of a negative number" );
	return std::sqrt( length_squared );
}

float sca2025::Math::sVector::GetLengthSQ() const
{
	const auto length_squared = (x * x) + (y * y) + (z * z);
	return length_squared;
}

float sca2025::Math::sVector::Normalize()
{
	const auto length = GetLength();
	EAE6320_ASSERTF( length > s_epsilon, "Can't divide by zero" );
	operator /=( length );
	return length;
}

sca2025::Math::sVector sca2025::Math::sVector::GetNormalized() const
{
	const auto length = GetLength();
	EAE6320_ASSERTF( length > s_epsilon, "Can't divide by zero" );
	return sVector( x / length, y / length, z / length );
}

// Products
//---------

float sca2025::Math::Dot( const sVector i_lhs, const sVector i_rhs )
{
	return ( i_lhs.x * i_rhs.x ) + ( i_lhs.y * i_rhs.y ) + ( i_lhs.z * i_rhs.z );
}

sca2025::Math::sVector sca2025::Math::Cross( const sVector i_lhs, const sVector i_rhs )
{
	return sVector(
		( i_lhs.y * i_rhs.z ) - ( i_lhs.z * i_rhs.y ),
		( i_lhs.z * i_rhs.x ) - ( i_lhs.x * i_rhs.z ),
		( i_lhs.x * i_rhs.y ) - ( i_lhs.y * i_rhs.x )
	);
}

// Comparison
//-----------

bool sca2025::Math::sVector::operator ==( const sVector i_rhs ) const
{
	// Use & rather than && to prevent branches (all three comparisons will be evaluated)
	return ( x == i_rhs.x ) & ( y == i_rhs.y ) & ( z == i_rhs.z );
}

bool sca2025::Math::sVector::operator !=( const sVector i_rhs ) const
{
	// Use | rather than || to prevent branches (all three comparisons will be evaluated)
	return ( x != i_rhs.x ) | ( y != i_rhs.y ) | ( z != i_rhs.z );
}

// Initialization / Shut Down
//---------------------------

sca2025::Math::sVector::sVector( const float i_x, const float i_y, const float i_z )
	:
	x( i_x ), y( i_y ), z( i_z )
{

}

void sca2025::Math::GetSkewSymmetricMatrix(Vector3f &i_vecotor, Matrix3f &o_matrix)
{
	o_matrix.setZero();
	o_matrix(0, 1) = -i_vecotor(2);
	o_matrix(0, 2) = i_vecotor(1);
	o_matrix(1, 0) = i_vecotor(2);
	o_matrix(1, 2) = -i_vecotor(0);
	o_matrix(2, 0) = -i_vecotor(1);
	o_matrix(2, 1) = i_vecotor(0);
}

Matrix3f sca2025::Math::ToSkewSymmetricMatrix(Vector3f &i_vecotor)
{
	Matrix3f output;
	output.setZero();
	output(0, 1) = -i_vecotor(2);
	output(0, 2) = i_vecotor(1);
	output(1, 0) = i_vecotor(2);
	output(1, 2) = -i_vecotor(0);
	output(2, 0) = -i_vecotor(1);
	output(2, 1) = i_vecotor(0);

	return output;
}

Matrix3d sca2025::Math::ToSkewSymmetricMatrix(Vector3d &i_vecotor)
{
	Matrix3d output;
	output.setZero();
	output(0, 1) = -i_vecotor(2);
	output(0, 2) = i_vecotor(1);
	output(1, 0) = i_vecotor(2);
	output(1, 2) = -i_vecotor(0);
	output(2, 0) = -i_vecotor(1);
	output(2, 1) = i_vecotor(0);

	return output;
}

void sca2025::Math::NativeVector2EigenVector(sVector i_vector, Vector3f &o_vector)
{
	o_vector(0) = i_vector.x;
	o_vector(1) = i_vector.y;
	o_vector(2) = i_vector.z;
}

sca2025::Math::sVector sca2025::Math::EigenVector2nativeVector(const Vector3f &i_vector)
{
	sVector output;
	output.x = i_vector(0);
	output.y = i_vector(1);
	output.z = i_vector(2);

	return output;
}

sca2025::Math::sVector sca2025::Math::EigenVector2nativeVector(const Vector3d &i_vector)
{
	sVector output;
	output.x = static_cast<float>(i_vector(0));
	output.y = static_cast<float>(i_vector(1));
	output.z = static_cast<float>(i_vector(2));

	return output;
}

Vector3f sca2025::Math::NativeVector2EigenVector(const sVector &i_vector)
{
	Vector3f output;
	output(0) = i_vector.x;
	output(1) = i_vector.y;
	output(2) = i_vector.z;

	return output;
}

sca2025::Math::sVector sca2025::Math::GetTangentVector(sVector n)
{
	sVector t;
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