/*
	This class represents a rotation or an orientation
*/

#ifndef EAE6320_MATH_CQUATERNION_H
#define EAE6320_MATH_CQUATERNION_H
#include "External/EigenLibrary/Eigen/Dense"

// Forward Declarations
//=====================

namespace sca2025
{
	namespace Math
	{
		struct sVector;
	}
}

// Class Declaration
//==================

namespace sca2025
{
	namespace Math
	{
		class cQuaternion
		{
			// Interface
			//==========

		public:

			// Multiplication
			//---------------

			cQuaternion operator *( const cQuaternion i_rhs ) const;

			// Inversion
			//----------

			void Invert();
			cQuaternion GetInverse() const;

			// Normalization
			//--------------

			void Normalize();
			cQuaternion GetNormalized() const;

			// Products
			//---------

			friend float Dot( const cQuaternion i_lhs, const cQuaternion i_rhs );

			// Access
			//-------

			// Calculating the forward direction involves a variation of calculating a full transformation matrix;
			// if the transform is already available or will need to be calculated in the future
			// it is more efficient to extract the forward direction from that
			sVector CalculateForwardDirection() const;

			// Initialization / Shut Down
			//---------------------------

			cQuaternion() = default;	// Identity
			cQuaternion( const float i_angleInRadians,	// A positive angle rotates counter-clockwise (right-handed) around the axis
				const sVector i_axisOfRotation_normalized );
			cQuaternion(const float i_w, const float i_x, const float i_y, const float i_z);

			// Data
			//=====
			//implementation details: https://stackoverflow.com/questions/1031005/is-there-an-algorithm-for-converting-quaternion-rotations-to-euler-angle-rotatio
			Math::sVector Quaternion2Euler();//rotSeq is yxz
			Math::sVector Quaternion2AxisAngle();
			
			float w()
			{
				return m_w;
			}
			float x()
			{
				return m_x;
			}
			float y()
			{
				return m_y;
			}
			float z()
			{
				return m_z;
			}
		private:

			float m_w = 1.0f;
			float m_x = 0.0f;
			float m_y = 0.0f;
			float m_z = 0.0f;

			// Implementation
			//===============

		private:

			// Initialization / Shut Down
			//---------------------------
			// Friends
			//========
			void threeaxisrot(float r11, float r12, float r21, float r31, float r32, float res[]);
			friend class cMatrix_transformation;
		};

		// Friends
		//========

		float Dot( const cQuaternion i_lhs, const cQuaternion i_rhs );
		cQuaternion ConvertEigenQuatToNativeQuat(Eigen::Quaternionf i_quat);
	}
}

#endif	// EAE6320_MATH_CQUATERNION_H
