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

void eae6320::MultiBody::FDTest()
{
	gravity = true;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, HINGE_JOINT, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 0
	SetHingeJoint(0, _Vector3(0, 0, 1), 0);
	AddRigidBody(0, HINGE_JOINT, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 1
	SetHingeJoint(1, _Vector3(0, 0, 1), 0);

	MultiBodyInitialization();
	const char* filePath = "key_press_save.txt";
	FILE* pFile = fopen(filePath, "rb");
	int dof = static_cast<int>(q.size());
	for (int i = 0; i < dof; i++)
	{
		fread(&q(i), sizeof(double), 1, pFile);
	}
	for (int i = 0; i < dof; i++)
	{
		fread(&qdot(i), sizeof(double), 1, pFile);
	}
	fclose(pFile);
	Forward();

	_Scalar delta = 1e-9;
	
	std::vector<_Matrix> expectedDerivative0;
	expectedDerivative0.resize(2);
	for (int i = 0; i < numOfLinks; i++)
	{
		expectedDerivative0[i].resize(6, 2);
	}
	
	_Matrix d0, d1;
	d0.resize(6, 1);
	d1.resize(6, 1);

	std::vector<_Vector> perturbed_q;
	_Vector old_q;
	old_q = q;
	perturbed_q.resize(dof);
	for (int i = 0; i < dof; i++)
	{
		perturbed_q[i].resize(dof);
		perturbed_q[i] = q;
		perturbed_q[i](i) = perturbed_q[i](i) + delta;
	}
	//std::cout << q.transpose() << std::endl;
	//std::cout << perturbed_q[0].transpose() << std::endl;
	//std::cout << perturbed_q[1].transpose() << std::endl;

	for (int i = 0; i < numOfLinks; i++)
	{
		q = old_q;
		Forward();
		d0 = Ht[i] * qdot;
		for (int j = 0; j < dof; j++)
		{
			q = perturbed_q[j];
			Forward();
			d1 = Ht[i] * qdot;
			expectedDerivative0[i].block<6, 1>(0, j) = (d1 - d0) / delta;
		}
	}
	std::cout << expectedDerivative0[0] << std::endl << std::endl;
	std::cout << expectedDerivative0[1] << std::endl << std::endl;
	//LOG_TO_FILE << std::setprecision(std::numeric_limits<double>::max_digits10);
	//LOG_TO_FILE << expectedDerivative0[0](0, 0) << std::endl;
	//****************************************************************************
	pFile = fopen(filePath, "rb");
	for (int i = 0; i < dof; i++)
	{
		fread(&q(i), sizeof(double), 1, pFile);
	}
	for (int i = 0; i < dof; i++)
	{
		fread(&qdot(i), sizeof(double), 1, pFile);
	}
	fclose(pFile);
	Forward();

	std::vector<_Matrix> expectedDerivative1;
	expectedDerivative1.resize(2);
	for (int i = 0; i < numOfLinks; i++)
	{
		expectedDerivative1[i].resize(6, 2);
	}
	d0.setZero();
	d1.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
		q = old_q;
		ForwardKinematics(q, rel_ori);
		
		_Vector tran_rot_velocity;//TODO this one needs to be updated when it's used with velocity
		tran_rot_velocity.resize(6);
		tran_rot_velocity.segment(0, 3) = vel[i];
		tran_rot_velocity.segment(3, 3) = w_abs_world[i];
		d0 = Mbody[i] * tran_rot_velocity;

		for (int j = 0; j < dof; j++)
		{
			q = perturbed_q[j];
			ForwardKinematics(q, rel_ori);
			tran_rot_velocity.segment(0, 3) = vel[i];
			tran_rot_velocity.segment(3, 3) = w_abs_world[i];
			d1 = Mbody[i] * tran_rot_velocity;
			expectedDerivative1[i].block<6, 1>(0, j) = (d1 - d0) / delta;
		}
	}
	std::cout << expectedDerivative1[0] << std::endl << std::endl;
	std::cout << expectedDerivative1[1] << std::endl << std::endl;
	//****************************************************************************
	pFile = fopen(filePath, "rb");
	for (int i = 0; i < dof; i++)
	{
		fread(&q(i), sizeof(double), 1, pFile);
	}
	for (int i = 0; i < dof; i++)
	{
		fread(&qdot(i), sizeof(double), 1, pFile);
	}
	fclose(pFile);
	Forward();

	_Matrix M0;
	M0.resize(1, totalPosDOF);
	M0.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
		M0 = M0 + qdot.transpose() * Ht[i].transpose() * Mbody[i] * expectedDerivative0[i] + 0.5 * qdot.transpose() * Ht[i].transpose() * expectedDerivative1[i];
	}
	std::cout << "grad_c " << M0 << std::endl;
}

void eae6320::MultiBody::AnalyticalTest()
{
	gravity = true;

	_Matrix3 localInertiaTensor;
	localInertiaTensor.setIdentity();
	if (geometry == BOX) localInertiaTensor = localInertiaTensor * (1.0f / 12.0f)* rigidBodyMass * 8;

	AddRigidBody(-1, HINGE_JOINT, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, 0.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 0
	SetHingeJoint(0, _Vector3(0, 0, 1), 0);
	AddRigidBody(0, HINGE_JOINT, _Vector3(0.0f, 1.0f, 0.0f), _Vector3(0.0f, -1.0f, 0.0f), masterMeshArray[4], Vector3d(1, 1, 1), localInertiaTensor);//body 1
	SetHingeJoint(1, _Vector3(0, 0, 1), 0);

	MultiBodyInitialization();
	const char* filePath = "key_press_save.txt";
	FILE* pFile = fopen(filePath, "rb");
	int dof = static_cast<int>(q.size());
	for (int i = 0; i < dof; i++)
	{
		fread(&q(i), sizeof(double), 1, pFile);
	}
	for (int i = 0; i < dof; i++)
	{
		fread(&qdot(i), sizeof(double), 1, pFile);
	}
	fclose(pFile);
	Forward();
	//**********************************************************************
	//LOG_TO_FILE << std::setprecision(std::numeric_limits<double>::max_digits10);
	_Matrix M0;
	M0.resize(1, totalPosDOF);
	M0.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
		//ComputeN
		mN[i].setZero();
		if (i > 0)
		{
			int j = parentArr[i];
			mN[i].block(0, 0, 3, totalPosDOF) = Ht[j].block(3, 0, 3, totalVelDOF);
		}
		mN[i].block(3, 0, 3, totalPosDOF) = Ht[i].block(3, 0, 3, totalVelDOF);
		mN[i](6, i) = 1;

		//ComputeB
		mB[i].setZero();
		_Vector3 Vec3;
		Vec3 = H[i].block<3, 1>(3, 0) * qdot.segment(velStartIndex[i], velDOF[i]); //qdot.segment(velStartIndex[i], velDOF[i]) is b
		mB[i].block<3, 3>(0, 0) = -Math::ToSkewSymmetricMatrix(uGlobalsChild[i]) * Math::ToSkewSymmetricMatrix(Vec3);
		mB[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(Vec3) * Math::ToSkewSymmetricMatrix(uGlobalsChild[i]);
		mB[i].block<3, 3>(3, 0) = -Math::ToSkewSymmetricMatrix(Vec3);
		//ComputeE
		_Vector tran_rot_velocity;//TODO this one needs to be updated when it's used with velocity
		tran_rot_velocity.resize(6);
		tran_rot_velocity.segment(0, 3) = vel[i];
		tran_rot_velocity.segment(3, 3) = w_abs_world[i];
		mE[i].setZero();
		_Vector3 b1 = tran_rot_velocity.segment(0, 3);
		_Vector3 b2 = tran_rot_velocity.segment(3, 3);//tran_rot_velocity is b
		_Vector3 V0 = Mbody[i].block<3, 3>(0, 3) * b2;
		mE[i].block<3, 3>(0, 3) = -Math::ToSkewSymmetricMatrix(V0) + Mbody[i].block<3, 3>(0, 3) * Math::ToSkewSymmetricMatrix(b2);
		_Vector3 V1 = Mbody[i].block<3, 3>(3, 0) * b1;
		_Vector3 V2 = Mbody[i].block<3, 3>(3, 3) * b2;
		mE[i].block<3, 3>(3, 3) = -Math::ToSkewSymmetricMatrix(V1) + Mbody[i].block<3, 3>(3, 0) * Math::ToSkewSymmetricMatrix(b1)
			- Math::ToSkewSymmetricMatrix(V2) + Mbody[i].block<3, 3>(3, 3) * Math::ToSkewSymmetricMatrix(b2);
		//ComputeHtDerivativeTimes_b
		if (i == 0)
		{
			HtDerivativeTimes_b[0] = mB[0] * mN[0];
		}
		else
		{
			int j = parentArr[i];
			_Vector tran_rot_velocity;//TODO this one needs to be updated when it's used with velocity
			tran_rot_velocity.resize(6);
			tran_rot_velocity.segment(0, 3) = vel[j];
			tran_rot_velocity.segment(3, 3) = w_abs_world[j];

			//ComputeA
			_Vector3 b2 = tran_rot_velocity.segment(3, 3);//tran_rot_velocity is b
			mA[i].setZero();
			_Vector3 Vec3;
			Vec3 = hingeMagnitude[i] * hingeDirGlobals[i];
			mA[i].block<3, 3>(0, 0) = -Math::ToSkewSymmetricMatrix(b2) * (Math::ToSkewSymmetricMatrix(uGlobalsParent[i]) + Math::ToSkewSymmetricMatrix(Vec3));
			mA[i].block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(b2) * Math::ToSkewSymmetricMatrix(uGlobalsChild[i]);

			HtDerivativeTimes_b[i] = D[i] * HtDerivativeTimes_b[j] + mA[i] * mN[i] + mB[i] * mN[i];
		}
		//ComputeMassMatrixDerivativeTimes_b
		MassMatrixDerivativeTimes_b[i] = mE[i] * mN[i];
		//std::cout << HtDerivativeTimes_b[i] << std::endl << std::endl;
		//std::cout << MassMatrixDerivativeTimes_b[i] << std::endl << std::endl;
	}
	//LOG_TO_FILE << HtDerivativeTimes_b[0](0, 0) << std::endl;
}