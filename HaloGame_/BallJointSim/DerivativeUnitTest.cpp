#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#include "Engine/GameCommon/Camera.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>

void eae6320::MultiBody::AnalyticalVsFD()
{
	int m_jointType = BALL_JOINT_4D;
	int m_mode = 1;//0 tests Jacobian derivative, 1 tests intertia derivative, 2 tests position dervative
	constraintSolverMode = IMPULSE;
	gravity = true;

	localInertiaTensor.setZero();
	{
		_Scalar x = 1;
		_Scalar y = 2;
		_Scalar z = 3;
		localInertiaTensor(0, 0) = 1.0f / 12.0f * rigidBodyMass * (y * y + z * z);
		localInertiaTensor(1, 1) = 1.0f / 12.0f * rigidBodyMass * (x * x + z * z);
		localInertiaTensor(2, 2) = 1.0f / 12.0f * rigidBodyMass * (x * x + y * y);
	}

	if (m_jointType == HINGE_JOINT)
	{
		AddRigidBody(-1, HINGE_JOINT, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 0
		SetHingeJoint(0, _Vector3(0, 0, 1), 0);
		AddRigidBody(0, HINGE_JOINT, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 1
		SetHingeJoint(1, _Vector3(0, 0, 1), 0);
	}
	else if (m_jointType == BALL_JOINT_4D)
	{
		AddRigidBody(-1, BALL_JOINT_4D, _Vector3(-1.0f, 0.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor);//body 0
		AddRigidBody(0, BALL_JOINT_4D, _Vector3(-1.0f, 0.0f, 0.0f), _Vector3(1.0f, 0.0f, 0.0f), masterMeshArray[3], Vector3d(1, 0.5, 0.5), localInertiaTensor);//body 1
	}

	MultiBodyInitialization();
	const char* filePath = "key_press_save.txt";
	FILE* pFile = fopen(filePath, "rb");
	int qDof = static_cast<int>(q.size());
	for (int i = 0; i < qDof; i++)
	{
		fread(&q(i), sizeof(double), 1, pFile);
	}
	for (int i = 0; i < numOfLinks; i++)
	{
		fread(&rel_ori[i].w(), sizeof(double), 1, pFile);
		fread(&rel_ori[i].x(), sizeof(double), 1, pFile);
		fread(&rel_ori[i].y(), sizeof(double), 1, pFile);
		fread(&rel_ori[i].z(), sizeof(double), 1, pFile);
	}
	int vDof = static_cast<int>(qdot.size());
	for (int i = 0; i < vDof; i++)
	{
		fread(&qdot(i), sizeof(double), 1, pFile);
	}
	fclose(pFile);
	Forward();
	
	CopyFromQ2X(jointType);
	ComputeExponentialMapJacobian(x, xJointType, xStartIndex);
	UpdateXdot(xdot, qdot, jointType);

	std::vector<_Matrix> HtDerivativeAnalytical;
	HtDerivativeAnalytical.resize(numOfLinks);
	std::vector<_Matrix> MassDerivativeAnalytical;
	MassDerivativeAnalytical.resize(numOfLinks);
	std::vector<_Matrix> HtDerivativeFD;
	HtDerivativeFD.resize(numOfLinks);
	std::vector<_Matrix> MassDerivativeFD;
	MassDerivativeFD.resize(numOfLinks);
	std::vector<_Matrix> PosDerivative;
	PosDerivative.resize(numOfLinks);
	std::vector<_Matrix> PosDerivativeFD;
	PosDerivativeFD.resize(numOfLinks);
	_Matrix PosDerivative3;

	std::vector<_Vector> bm;
	bm.resize(numOfLinks);
	for (int i = 0; i < numOfLinks; i++)
	{
		_Vector vec;
		vec.resize(6);
		vec.segment(0, 3) = vel[i];
		vec.segment(3, 3) = w_abs_world[i];
		bm[i] = vec;
	}
	std::cout << std::setprecision(16) << bm[0].transpose() << std::endl;
	std::cout << std::setprecision(16) << bm[1].transpose() << std::endl;
	
	/*for (int i = 0; i < 8; i++)
	{
		ComputeJacobianAndInertiaDerivativeFDV2(x, qdot, bm, HtDerivativeFD, MassDerivativeFD, pow(10, -2 - i));
		LOG_TO_FILE << std::setprecision(std::numeric_limits<double>::max_digits10);
		LOG_TO_FILE << HtDerivativeFD[0](0, 0) << std::endl;
	}*/
	
	std::vector<_Matrix> Ht_x;
	std::vector<_Matrix> H_x;
	Ht_x.resize(numOfLinks);
	H_x.resize(numOfLinks);
	ComputeHt(Ht_x, H_x, x, rel_ori, xJointType, xStartIndex);
	if (m_mode == 0 || m_mode == 1)
	{
		ComputeJacobianAndInertiaDerivativeFDV2(x, xdot, bm, HtDerivativeFD, MassDerivativeFD, pow(10, -9));
		ComputeJacobianAndInertiaDerivative(totalVelDOF, xdot, bm, x, Ht_x, H_x, HtDerivativeAnalytical, MassDerivativeAnalytical);
	}
	else  if (m_mode == 2)
	{
		ComputeDxOverDp(PosDerivative, Ht_x, totalVelDOF);
	
		PosDerivative3.resize(3, 6);
		PosDerivative3.block<3, 3>(0, 0) = -Math::ToSkewSymmetricMatrix(pos[1]) * H_x[0].block<3, 3>(3, 0);
		_Vector3 localV = R_local[1] * uLocalsChild[1];
		//Compute J1
		_Matrix3 J1;
		int j = parentArr[1];
		_Vector3 r = x.segment(xStartIndex[1], 3);
		_Scalar theta = r.norm();
		_Scalar b = Compute_b(theta);
		_Scalar a = Compute_a(theta);
		_Scalar c = Compute_c(theta, a);
		J1 = _Matrix::Identity(3, 3) + b * Math::ToSkewSymmetricMatrix(r) + c * Math::ToSkewSymmetricMatrix(r) * Math::ToSkewSymmetricMatrix(r);
		PosDerivative3.block<3, 3>(0, 3) = R_local[0] * Math::ToSkewSymmetricMatrix(localV) * J1;
		
		ComputeDxOverDpFD(PosDerivativeFD, x, pow(10, -9));
	}
	
	if (m_mode == 0)
	{
		std::cout << "Jacobain dervative" << std::endl;
		std::cout << std::setprecision(16) << HtDerivativeAnalytical[0] << std::endl << std::endl;
		std::cout << HtDerivativeAnalytical[1] << std::endl;
		std::cout << "============================" << std::endl;
		std::cout << std::setprecision(16) << HtDerivativeFD[0] << std::endl << std::endl;
		std::cout << std::setprecision(16) << HtDerivativeFD[1] << std::endl << std::endl;
	}
	else if (m_mode == 1)
	{
		std::cout << "mass dervative" << std::endl;
		std::cout << std::setprecision(16) << MassDerivativeAnalytical[0] << std::endl << std::endl;
		std::cout << MassDerivativeAnalytical[1] << std::endl;
		std::cout << "============================" << std::endl;
		std::cout << std::setprecision(16) << MassDerivativeFD[0] << std::endl << std::endl;
		std::cout << std::setprecision(16) << MassDerivativeFD[1] << std::endl << std::endl;
		//LOG_TO_FILE << std::setprecision(std::numeric_limits<double>::max_digits10);
		//LOG_TO_FILE << HtDerivativeFD[0](0, 0) << std::endl;
	}
	else if (m_mode == 2)
	{
		std::cout << "position dervative" << std::endl;
		//std::cout << std::setprecision(16) << PosDerivative[0] << std::endl << std::endl;
		std::cout << std::setprecision(16) << PosDerivative[1] << std::endl;
		std::cout << "============================" << std::endl;
		std::cout << std::setprecision(16) << PosDerivative3 << std::endl << std::endl;
		std::cout << "============================" << std::endl;
		//std::cout << std::setprecision(16) << PosDerivativeFD[0] << std::endl << std::endl;
		std::cout << std::setprecision(16) << Ht_x[1].block<3, 6>(0, 0) << std::endl << std::endl;
		std::cout << "============================" << std::endl;
		//std::cout << std::setprecision(16) << PosDerivativeFD[0] << std::endl << std::endl;
		std::cout << std::setprecision(16) << PosDerivativeFD[1] << std::endl << std::endl;
		
	}
}