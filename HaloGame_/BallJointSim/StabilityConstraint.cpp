#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>

void eae6320::MultiBody::EnergyConstraint()
{
	int energeMomentumConstraintDim = 7;
	int nq = totalVelDOF + 2;
	int n = nq + energeMomentumConstraintDim;
	_Matrix b(n, 1);
	_Matrix grad_F(n, n);

	_Vector x(n);
	x.setZero();
	x.segment(0, totalVelDOF) = qdot;

	_Vector qt(nq);
	qt.setZero();
	qt.segment(0, totalVelDOF) = qdot;

	_Matrix D(nq, nq);
	D.setZero();
	D.block(0, 0, totalVelDOF, totalVelDOF) = _Matrix::Identity(totalVelDOF, totalVelDOF);
	_Scalar m_coeff = 10;
	D(totalVelDOF, totalVelDOF) = m_coeff;
	D(totalVelDOF + 1, totalVelDOF + 1) = m_coeff;

	_Matrix Kp(3, totalVelDOF);
	Kp.setZero();
	_Matrix Kl(3, totalVelDOF);
	Kl.setZero();
	_Matrix Sv(3, 6);
	Sv.setZero();
	Sv.block<3, 3>(0, 0) = _Matrix::Identity(3, 3);
	_Matrix Sw(3, 6);
	Sw.setZero();
	Sw.block<3, 3>(0, 3) = _Matrix::Identity(3, 3);
	for (int i = 0; i < numOfLinks; i++)
	{
		Kp = Kp + Mbody[i].block<3, 3>(0, 0) * Sv * Ht[i];
		Kl = Kl + Mbody[i].block<3, 3>(3, 3) * Sw * Ht[i] + rigidBodyMass * Math::ToSkewSymmetricMatrix(pos[i]) * Sv * Ht[i];
	}

	_Scalar kineticEnergyExpected = totalEnergy0 - ComputePotentialEnergy();
	_Vector3 linearMomentumNew = ComputeTranslationalMomentum();
	_Vector3 angularMomentumNew = ComputeAngularMomentum();
	_Matrix grad_C(7, nq);
	grad_C.setZero();
	grad_C.block(1, 0, 3, totalVelDOF) = Kp;
	grad_C.block(4, 0, 3, totalVelDOF) = Kl;
	grad_C.block<3, 1>(1, totalVelDOF) = linearMomentumNew - linearMomentum0;
	grad_C.block<3, 1>(4, totalVelDOF + 1) = angularMomentumNew - angularMomentum0;

	_Scalar energyErr = 1.0;
	_Matrix C(7, 1);
	int iter = 0;
	while (energyErr > 1e-3)
	{
		if (iter >= 5)
		{
			std::cout << "limit reached!" << std::endl;
			break;
		}
			
		//C(0, 0) = 0.5 * (x.segment(0, totalVelDOF).transpose() * Mr * x.segment(0, totalVelDOF))(0, 0) - kineticEnergy0;
		C(0, 0) = 0.5 * (x.segment(0, totalVelDOF).transpose() * Mr * x.segment(0, totalVelDOF))(0, 0) - kineticEnergyExpected;
		//std::cout << C(0, 0) << std::endl;
		C.block<3, 1>(1, 0) = Kp * x.segment(0, totalVelDOF) - (1 - x(totalVelDOF)) * linearMomentumNew - x(totalVelDOF) * linearMomentum0;
		C.block<3, 1>(4, 0) = Kl * x.segment(0, totalVelDOF) - (1 - x(totalVelDOF + 1)) * angularMomentumNew - x(totalVelDOF + 1) * angularMomentum0;
		grad_C.block(0, 0, 1, totalVelDOF) = (Mr * x.segment(0, totalVelDOF)).transpose();

		b.block(0, 0, nq, 1) = -D * (x.segment(0, nq) - qt);
		b.block<7, 1>(nq, 0) = -C;

		_Matrix HessianC_lambda(nq, nq);
		HessianC_lambda.setZero();
		HessianC_lambda.block(0, 0, totalVelDOF, totalVelDOF) = x(nq) * Mr;

		grad_F.setZero();
		grad_F.block(0, 0, nq, nq) = D - HessianC_lambda;
		grad_F.block(0, nq, nq, 7) = -grad_C.transpose();
		grad_F.block(nq, 0, 7, nq) = grad_C;
		if (grad_F.determinant() < 1e-6)
		{
			EAE6320_ASSERTF(false, "grad_f is not invertable");
		}
		_Vector y(n);
		y.setZero();
		y = grad_F.inverse() * b;
		x.segment(0, nq) = x.segment(0, nq) + y.segment(0, nq);
		//std::cout << y.segment(0, nq).transpose() << std::endl;
		x.segment(nq, energeMomentumConstraintDim) = y.segment(nq, energeMomentumConstraintDim);
		_Vector qdot_new = x.segment(0, totalVelDOF);
		ForwardAngularAndTranslationalVelocity(qdot_new);
		//energyErr = fabs(ComputeKineticEnergy() - kineticEnergy0);
		energyErr = fabs(ComputeKineticEnergy() - kineticEnergyExpected);
		//energyErr = ComputeTotalEnergy();
		//std::cout << "expected momentum " << linearMomentumNew.transpose() << " actual momentum " << ComputeTranslationalMomentum().transpose() << std::endl;
		//std::cout << "expected momentum " << angularMomentum0.transpose() << " actual momentum " << ComputeAngularMomentum().transpose() << std::endl;
		//std::cout << ComputeTotalEnergy() << std::endl;
		iter++;
	}
	qdot = x.segment(0, totalVelDOF);
	//std::cout << endl;
	//std::cout << "Energy at end of solve " << ComputeTotalEnergy() << std::endl;
}

void eae6320::MultiBody::EnergyConstraintV2()
{
	int energeMomentumConstraintDim = 7;
	int nq = totalVelDOF + 1;
	int n = nq + energeMomentumConstraintDim;
	_Matrix b(n, 1);
	_Matrix grad_F(n, n);
	_Matrix regularization(n, n);
	regularization.setIdentity();
	regularization = 1e-6 * regularization;

	_Vector x(n);
	x.setZero();
	x.segment(0, totalVelDOF) = qdot;

	_Vector qt(nq);
	qt.setZero();
	qt.segment(0, totalVelDOF) = qdot;

	_Matrix D(nq, nq);
	D.setZero();
	D.block(0, 0, totalVelDOF, totalVelDOF) = Mr;
	_Scalar m_coeff = 10;
	D(totalVelDOF, totalVelDOF) = m_coeff;

	_Matrix Kp(3, totalVelDOF);
	Kp.setZero();
	_Matrix Kl(3, totalVelDOF);
	Kl.setZero();
	_Matrix Sv(3, 6);
	Sv.setZero();
	Sv.block<3, 3>(0, 0) = _Matrix::Identity(3, 3);
	_Matrix Sw(3, 6);
	Sw.setZero();
	Sw.block<3, 3>(0, 3) = _Matrix::Identity(3, 3);
	for (int i = 0; i < numOfLinks; i++)
	{
		Kp = Kp + Mbody[i].block<3, 3>(0, 0) * Sv * Ht[i];
		Kl = Kl + Mbody[i].block<3, 3>(3, 3) * Sw * Ht[i] + rigidBodyMass * Math::ToSkewSymmetricMatrix(pos[i]) * Sv * Ht[i];
	}

	_Scalar kineticEnergyExpected = totalEnergy0 - ComputePotentialEnergy();
	_Scalar kineticEnergyCurrent = ComputeKineticEnergy();
	_Vector3 linearMomentumCurrent = ComputeTranslationalMomentum();
	_Vector3 angularMomentumCurrent = ComputeAngularMomentum();
	_Matrix grad_C(7, nq);
	grad_C.setZero();
	grad_C.block(1, 0, 3, totalVelDOF) = Kp;
	grad_C.block(4, 0, 3, totalVelDOF) = Kl;

	_Scalar energyErr = 1.0;
	_Matrix C(7, 1);
	int iter = 0;
	while (energyErr > 1e-3)
	{
		if (iter >= 20)
		{
			//std::cout << "limit reached!" << std::endl;
			break;
		}

		//C(0, 0) = 0.5 * (x.segment(0, totalVelDOF).transpose() * Mr * x.segment(0, totalVelDOF))(0, 0) - kineticEnergy0;
		C(0, 0) = 0.5 * (x.segment(0, totalVelDOF).transpose() * Mr * x.segment(0, totalVelDOF))(0, 0) - (1 - x(totalVelDOF)) * kineticEnergyExpected - x(totalVelDOF) * kineticEnergyCurrent;
		C.block<3, 1>(1, 0) = Kp * x.segment(0, totalVelDOF) - linearMomentumCurrent;
		C.block<3, 1>(4, 0) = Kl * x.segment(0, totalVelDOF) - angularMomentumCurrent;
		grad_C.block(0, 0, 1, totalVelDOF) = (Mr * x.segment(0, totalVelDOF)).transpose();
		_Matrix temp;
		temp.resize(1, 1);
		temp(0, 0) = kineticEnergyExpected - kineticEnergyCurrent;
		grad_C.block<1, 1>(0, totalVelDOF) = temp;

		b.block(0, 0, nq, 1) = -D * (x.segment(0, nq) - qt);
		b.block<7, 1>(nq, 0) = -C;

		_Matrix HessianC_lambda(nq, nq);
		HessianC_lambda.setZero();
		HessianC_lambda.block(0, 0, totalVelDOF, totalVelDOF) = x(nq) * Mr;

		grad_F.setZero();
		grad_F.block(0, 0, nq, nq) = D - HessianC_lambda;
		grad_F.block(0, nq, nq, 7) = -grad_C.transpose();
		grad_F.block(nq, 0, 7, nq) = grad_C;
		grad_F = grad_F + regularization;
		if (abs(grad_F.determinant()) < 1e-6)
		{
			//std::cout << grad_F << std::endl;
			//std::cout << "grad_F " << abs(grad_F.determinant()) << std::endl;
		}
		_Vector y(n);
		y.setZero();
		y = grad_F.inverse() * b;
		//std::cout << grad_F << std::endl << std::endl;
		//std::cout << b.transpose() << std::endl << std::endl;
		//std::cout << y.transpose() << std::endl;
		x.segment(0, nq) = x.segment(0, nq) + y.segment(0, nq);
		//std::cout << y.segment(0, nq).transpose() << std::endl;
		x.segment(nq, energeMomentumConstraintDim) = y.segment(nq, energeMomentumConstraintDim);//update lagrange multiplier
		_Vector qdot_new = x.segment(0, totalVelDOF);
		//std::cout << qdot_new.transpose() << std::endl;
		ForwardAngularAndTranslationalVelocity(qdot_new);
		//energyErr = fabs(ComputeKineticEnergy() - kineticEnergy0);
		energyErr = fabs(ComputeKineticEnergy() - kineticEnergyExpected);
		//std::cout << energyErr << std::endl;
		//energyErr = ComputeTotalEnergy();
		//std::cout << "expected momentum " << linearMomentumNew.transpose() << " actual momentum " << ComputeTranslationalMomentum().transpose() << std::endl;
		//std::cout << "expected momentum " << angularMomentum0.transpose() << " actual momentum " << ComputeAngularMomentum().transpose() << std::endl;
		//std::cout << ComputeTotalEnergy() << std::endl;
		iter++;
	}
	qdot = x.segment(0, totalVelDOF);
	//std::cout << endl;
	//std::cout << "Energy at end of solve " << ComputeTotalEnergy() << std::endl;
	//std::cout << iter << std::endl;
}

void eae6320::MultiBody::EnergyConstraintV3()
{
	int energeMomentumConstraintDim = 1;
	int nq = totalVelDOF;
	int n = nq + energeMomentumConstraintDim;
	_Matrix b(n, 1);
	_Matrix grad_F(n, n);
	_Matrix regularization(n, n);
	regularization.setIdentity();
	regularization = 1e-6 * regularization;

	_Vector x(n);
	x.setZero();
	x.segment(0, totalVelDOF) = qdot;

	_Vector qt(nq);
	qt.setZero();
	qt.segment(0, totalVelDOF) = qdot;

	_Matrix D(nq, nq);
	D.setZero();
	D.block(0, 0, totalVelDOF, totalVelDOF) = Mr;

	_Scalar kineticEnergyExpected = totalEnergy0 - ComputePotentialEnergy();
	_Scalar kineticEnergyCurrent = ComputeKineticEnergy();
	_Matrix grad_C(1, nq);
	grad_C.setZero();

	_Scalar energyErr = 1.0;
	_Matrix C(1, 1);
	int iter = 0;
	while (energyErr > 1e-3)
	{
		if (iter >= 20)
		{
			//std::cout << "limit reached!" << std::endl;
			break;
		}

		C(0, 0) = 0.5 * (x.segment(0, totalVelDOF).transpose() * Mr * x.segment(0, totalVelDOF))(0, 0) - kineticEnergyExpected;
	
		grad_C.block(0, 0, 1, totalVelDOF) = (Mr * x.segment(0, totalVelDOF)).transpose();

		b.block(0, 0, nq, 1) = -D * (x.segment(0, nq) - qt);
		b.block<1, 1>(nq, 0) = -C;

		_Matrix HessianC_lambda(nq, nq);
		HessianC_lambda.setZero();
		HessianC_lambda.block(0, 0, totalVelDOF, totalVelDOF) = x(nq) * Mr;

		grad_F.setZero();
		grad_F.block(0, 0, nq, nq) = D - HessianC_lambda;
		grad_F.block(0, nq, nq, energeMomentumConstraintDim) = -grad_C.transpose();
		grad_F.block(nq, 0, energeMomentumConstraintDim, nq) = grad_C;
		grad_F = grad_F + regularization;
		if (abs(grad_F.determinant()) < 1e-6)
		{
			//std::cout << grad_F << std::endl;
			//std::cout << "grad_F " << abs(grad_F.determinant()) << std::endl;
		}
		_Vector y(n);
		y.setZero();
		y = grad_F.inverse() * b;
		//std::cout << grad_F << std::endl << std::endl;
		//std::cout << b.transpose() << std::endl << std::endl;
		//std::cout << y.transpose() << std::endl;
		x.segment(0, nq) = x.segment(0, nq) + y.segment(0, nq);
		//std::cout << y.segment(0, nq).transpose() << std::endl;
		x.segment(nq, energeMomentumConstraintDim) = y.segment(nq, energeMomentumConstraintDim);//update lagrange multiplier
		_Vector qdot_new = x.segment(0, totalVelDOF);
		//std::cout << qdot_new.transpose() << std::endl;
		ForwardAngularAndTranslationalVelocity(qdot_new);
		//energyErr = fabs(ComputeKineticEnergy() - kineticEnergy0);
		energyErr = fabs(ComputeKineticEnergy() - kineticEnergyExpected);
		//std::cout << "iteration " << iter << " energyErr " << energyErr << std::endl;
		//energyErr = ComputeTotalEnergy();
		//std::cout << "expected momentum " << linearMomentumNew.transpose() << " actual momentum " << ComputeTranslationalMomentum().transpose() << std::endl;
		//std::cout << "expected momentum " << angularMomentum0.transpose() << " actual momentum " << ComputeAngularMomentum().transpose() << std::endl;
		//std::cout << ComputeTotalEnergy() << std::endl;
		iter++;
	}
	qdot = x.segment(0, totalVelDOF);
	//std::cout << endl;
	//std::cout << "Energy at end of solve " << ComputeTotalEnergy() << std::endl;
	//std::cout << iter << std::endl;
}

void eae6320::MultiBody::AcceleratedEnergyConstraint()
{
	int energeMomentumConstraintDim = 1;
	int nq = totalVelDOF;
	int n = nq + energeMomentumConstraintDim;
	
	_Vector mq(nq);
	mq.setZero();
	mq.segment(0, totalVelDOF) = qdot;
	
	_Matrix grad_C(energeMomentumConstraintDim, nq);
	grad_C.setZero();

	_Matrix DInv;
	DInv = Mr.inverse();

	_Scalar kineticEnergyExpected = totalEnergy0 - ComputePotentialEnergy();

	_Scalar energyErr = 1.0;
	_Matrix C(energeMomentumConstraintDim, 1);
	_Matrix lambdaNew(energeMomentumConstraintDim, 1);
	int iter = 0;
	while (energyErr > 1e-3)
	{
		C(0, 0) = 0.5 * (mq.segment(0, totalVelDOF).transpose() * Mr * mq.segment(0, totalVelDOF))(0, 0) - kineticEnergyExpected;
		grad_C.block(0, 0, 1, totalVelDOF) = (Mr * mq.segment(0, totalVelDOF)).transpose();
		lambdaNew = (grad_C * DInv * grad_C.transpose()).inverse() * C;
		mq = mq - DInv * grad_C.transpose() * lambdaNew;

		ForwardAngularAndTranslationalVelocity(mq);
		energyErr = fabs(ComputeKineticEnergy() - kineticEnergyExpected);
		iter++;
	}
	qdot = mq;
	//std::cout << iter << std::endl;
}