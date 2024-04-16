#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>

//test the gradient of twist constraint
void eae6320::MultiBody::UnitTest0()
{
	_Vector3 q_old(-0.613267, -1.48335, 0.614194);
	q.segment(0, 3) = q_old;
	Forward();
	jointLimit[0] = 0.5 * M_PI;
	TwistLimitCheck();
	_Scalar c_old = g[0];
	
	_Scalar dq = 0.0000001;

	_Vector3 gradC;
	q(0) = q(0) + dq;
	Forward();
	TwistLimitCheck();
	gradC(0) = (g[0] - c_old) / dq;

	q.segment(0, 3) = q_old;
	q(1) = q(1) + dq;
	Forward();
	TwistLimitCheck();
	gradC(1) = (g[0] - c_old) / dq;

	q.segment(0, 3) = q_old;
	q(2) = q(2) + dq;
	Forward();
	TwistLimitCheck();
	gradC(2) = (g[0] - c_old) / dq;

	std::cout << gradC.transpose() << std::endl;

	q.segment(0, 3) = q_old;
	_Vector3 p = _Vector3(0, -10, 0);
	_Vector3 T0;
	_Vector3 RP = R_local[0] * p;
	T0 = -J_rotation[0].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * R_local[0] * Math::ToSkewSymmetricMatrix(p) * RP;
	_Vector3 T1;
	_Vector3 RPRP = R_local[0] * Math::ToSkewSymmetricMatrix(p) * RP;
	T1 = J_rotation[0].transpose() * Math::ToSkewSymmetricMatrix(RPRP) * Math::ToSkewSymmetricMatrix(p) * RP;
	_Vector3 T2;
	T2 = -J_rotation[0].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * R_local[0].transpose() * Math::ToSkewSymmetricMatrix(p) * RP;
	_Vector3 T3;
	//T3 = 2.0 * cos(jointLimit[i]) * J_rotation[i].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * Math::ToSkewSymmetricMatrix(p) * RP;
	T3 = -2.0 * J_rotation[0].transpose() * Math::ToSkewSymmetricMatrix(RP) * Math::ToSkewSymmetricMatrix(p) * Math::ToSkewSymmetricMatrix(p) * RP;

	_Vector s = p.cross(RP);
	_Scalar SRS = s.dot(R_local[0] * s);

	gradC.setZero();
	gradC = (T0 + T1 + T2) / s.squaredNorm() - SRS / (s.squaredNorm() * s.squaredNorm()) * T3;
	std::cout << gradC.transpose() << std::endl;
}