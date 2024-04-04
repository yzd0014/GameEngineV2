#pragma once
#include "External/EigenLibrary/Eigen/Dense"

using namespace Eigen;

#define HIGH_PRECISION_MODE 
#if defined (HIGH_PRECISION_MODE)
typedef double _Scalar;
typedef MatrixXd _Matrix;
typedef Matrix3d _Matrix3;
typedef VectorXd _Vector;
typedef Vector3d _Vector3;
typedef Quaterniond _Quat;
#else
typedef float _Scalar;
typedef MatrixXf _Matrix;
typedef Matrix3f _Matrix3;
typedef VectorXf _Vector;
typedef Vector3f _Vector3;
typedef Quaternionf _Quat;
#endif