// Includes
//=========

#include "cQuaternion.h"

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

// Multiplication
//---------------

sca2025::Math::cQuaternion sca2025::Math::cQuaternion::operator *( const cQuaternion i_rhs ) const
{
	return cQuaternion(
		( m_w * i_rhs.m_w ) - ( ( m_x * i_rhs.m_x ) + ( m_y * i_rhs.m_y ) + ( m_z * i_rhs.m_z ) ),
		( m_w * i_rhs.m_x ) + ( m_x * i_rhs.m_w ) + ( ( m_y * i_rhs.m_z ) - ( m_z * i_rhs.m_y ) ),
		( m_w * i_rhs.m_y ) + ( m_y * i_rhs.m_w ) + ( ( m_z * i_rhs.m_x ) - ( m_x * i_rhs.m_z ) ),
		( m_w * i_rhs.m_z ) + ( m_z * i_rhs.m_w ) + ( ( m_x * i_rhs.m_y ) - ( m_y * i_rhs.m_x ) ) );
}

// Inversion
//----------

void sca2025::Math::cQuaternion::Invert()
{
	m_x = -m_x;
	m_y = -m_y;
	m_z = -m_z;
}

sca2025::Math::cQuaternion sca2025::Math::cQuaternion::GetInverse() const
{
	return cQuaternion( m_w, -m_x, -m_y, -m_z );
}

// Normalization
//--------------

void sca2025::Math::cQuaternion::Normalize()
{
	const auto length = std::sqrt( ( m_w * m_w ) + ( m_x * m_x ) + ( m_y * m_y ) + ( m_z * m_z ) );
	EAE6320_ASSERTF( length > s_epsilon, "Can't divide by zero" );
	const auto length_reciprocal = 1.0f / length;
	m_w *= length_reciprocal;
	m_x *= length_reciprocal;
	m_y *= length_reciprocal;
	m_z *= length_reciprocal;
}

sca2025::Math::cQuaternion sca2025::Math::cQuaternion::GetNormalized() const
{
	const auto length = std::sqrt( ( m_w * m_w ) + ( m_x * m_x ) + ( m_y * m_y ) + ( m_z * m_z ) );
	EAE6320_ASSERTF( length > s_epsilon, "Can't divide by zero" );
	const auto length_reciprocal = 1.0f / length;
	return cQuaternion( m_w * length_reciprocal, m_x * length_reciprocal, m_y * length_reciprocal, m_z * length_reciprocal );
}

// Products
//---------

float sca2025::Math::Dot( const cQuaternion i_lhs, const cQuaternion i_rhs )
{
	return ( i_lhs.m_w * i_rhs.m_w ) + ( i_lhs.m_x * i_rhs.m_x ) + ( i_lhs.m_y * i_rhs.m_y ) + ( i_lhs.m_z * i_rhs.m_z );
}

// Access
//-------

sca2025::Math::sVector sca2025::Math::cQuaternion::CalculateForwardDirection() const
{
	const auto _2x = m_x + m_x;
	const auto _2y = m_y + m_y;
	const auto _2xx = m_x * _2x;
	const auto _2xz = _2x * m_z;
	const auto _2xw = _2x * m_w;
	const auto _2yy = _2y * m_y;
	const auto _2yz = _2y * m_z;
	const auto _2yw = _2y * m_w;

	return sVector( -_2xz - _2yw, -_2yz + _2xw, -1.0f + _2xx + _2yy );
}

// Initialization / Shut Down
//---------------------------

sca2025::Math::cQuaternion::cQuaternion( const float i_angleInRadians, const sVector i_axisOfRotation_normalized )
{
	const auto theta_half = i_angleInRadians * 0.5f;
	m_w = std::cos( theta_half );
	const auto sin_theta_half = std::sin( theta_half );
	m_x = i_axisOfRotation_normalized.x * sin_theta_half;
	m_y = i_axisOfRotation_normalized.y * sin_theta_half;
	m_z = i_axisOfRotation_normalized.z * sin_theta_half;
}

// Implementation
//===============

// Initialization / Shut Down
//---------------------------

sca2025::Math::cQuaternion::cQuaternion( const float i_w, const float i_x, const float i_y, const float i_z )
	:
	m_w( i_w ), m_x( i_x ), m_y( i_y ), m_z( i_z )
{

}

sca2025::Math::sVector sca2025::Math::cQuaternion::Quaternion2Euler()
{
	float res[3];
	Math::sVector rot;
	threeaxisrot(2 * (m_x*m_z + m_w*m_y),
		m_w*m_w - m_x*m_x - m_y*m_y + m_z*m_z,
		-2 * (m_y*m_z - m_w*m_x),
		2 * (m_x*m_y + m_w*m_z),
		m_w*m_w - m_x*m_x + m_y*m_y - m_z*m_z,
		res);

	rot.z = res[0];
	rot.x = res[1];
	rot.y = res[2];
	return rot;
}

void sca2025::Math::cQuaternion::threeaxisrot(float r11, float r12, float r21, float r31, float r32, float res[]) {
	res[0] = atan2(r31, r32);
	res[1] = asin(r21);
	res[2] = atan2(r11, r12);
}

sca2025::Math::sVector sca2025::Math::cQuaternion::Quaternion2AxisAngle()
{
	if (m_w > 1) Normalize(); // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
	float angle = 2 * acos(m_w);
	float s = sqrt(1 - m_w*m_w); // assuming quaternion normalised then w is less than 1, so term always positive.

	Math::sVector rot;
	if (s < s_epsilon) { // test to avoid divide by zero, s is always positive due to sqrt
	  // if s close to zero then direction of axis not important
		rot.x = m_x; // if it is important that axis is normalised then replace with x=1; y=z=0;
		rot.y = m_y;
		rot.z = m_z;
	}
	else {
		rot.x = m_x / s; // normalise axis
		rot.y = m_y / s;
		rot.z = m_z / s;
		rot = rot.GetNormalized()*angle;
	}

	return rot;
}

sca2025::Math::cQuaternion sca2025::Math::ConvertEigenQuatToNativeQuat(Eigen::Quaternionf i_quat)
{
	return sca2025::Math::cQuaternion(i_quat.w(), i_quat.x(), i_quat.y(), i_quat.z());
}