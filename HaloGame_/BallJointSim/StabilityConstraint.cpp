#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>

void eae6320::MultiBody::SQP()//energy conservation + momentum conservation and mmentum interpolation
{
	int energeMomentumConstraintDim = 7;
	int nq = totalVelDOF + 2;
	int n = nq + energeMomentumConstraintDim;
	_Matrix b(n, 1);
	_Matrix grad_F(n, n);

	_Matrix regularization(n, n);
	regularization.setIdentity();
	regularization = 1e-6 * regularization;
	
	_Vector q0(nq);
	q0.setZero();
	q0.segment(0, totalVelDOF) = qdot;
	
	_Vector x(n);
	x.setZero();
	x.segment(0, nq) = q0;

	_Matrix D(nq, nq);
	D.setZero();
	D.block(0, 0, totalVelDOF, totalVelDOF) = Mr;
	_Scalar m_coeff = 1e-3;
	D(totalVelDOF, totalVelDOF) = m_coeff;
	D(totalVelDOF + 1, totalVelDOF + 1) = m_coeff;

	_Matrix Kp(3, totalVelDOF);
	Kp.setZero();
	_Matrix Kl(3, totalVelDOF);
	Kl.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
		Kp = Kp + Mbody[i].block<3, 3>(0, 0) * Ht[i].block(0, 0, 3, totalVelDOF);
		Kl = Kl + Mbody[i].block<3, 3>(3, 3) * Ht[i].block(3, 0, 3, totalVelDOF) + rigidBodyMass * Math::ToSkewSymmetricMatrix(pos[i]) * Ht[i].block(0, 0, 3, totalVelDOF);
	}

	_Scalar kineticEnergy0 = totalEnergy0 - ComputePotentialEnergy();
	_Vector3 linearMomentum1 = ComputeTranslationalMomentum();
	_Vector3 angularMomentum1 = ComputeAngularMomentum();
	_Matrix grad_C(energeMomentumConstraintDim, nq);
	grad_C.setZero();
	grad_C.block(1, 0, 3, totalVelDOF) = Kp;
	grad_C.block(4, 0, 3, totalVelDOF) = Kl;
	grad_C.block<3, 1>(1, totalVelDOF) = linearMomentum1 - linearMomentum0;
	grad_C.block<3, 1>(4, totalVelDOF + 1) = angularMomentum1 - angularMomentum0;

	_Scalar energyErr = 1.0;
	_Matrix C(energeMomentumConstraintDim, 1);
	int iter = 0;
	while (true)
	{
		C(0, 0) = ComputeKineticEnergy() - kineticEnergy0;
		C.block<3, 1>(1, 0) = Kp * x.segment(0, totalVelDOF) - linearMomentum1 - x(totalVelDOF) * (linearMomentum0 - linearMomentum1);
		C.block<3, 1>(4, 0) = Kl * x.segment(0, totalVelDOF) - angularMomentum1 - x(totalVelDOF + 1) * (angularMomentum0 - angularMomentum1);
		_Scalar C_norm = C.norm();
		if (C_norm < 1e-6 || iter >= 20)
		{
			std::cout << "s " << x(totalVelDOF) << " t " << x(totalVelDOF + 1) << std::endl;
			std::cout << "C_norm " << C_norm << " iter " << iter << std::endl;
			break;
		}
		
		grad_C.block(0, 0, 1, totalVelDOF) = (Mr * x.segment(0, totalVelDOF)).transpose();

		b.block(0, 0, nq, 1) = -D * (x.segment(0, nq) - q0);
		b.block<7, 1>(nq, 0) = -C;

		_Matrix HessianC_lambda(nq, nq);
		HessianC_lambda.setZero();
		HessianC_lambda.block(0, 0, totalVelDOF, totalVelDOF) = x(nq) * Mr;

		grad_F.setZero();
		grad_F.block(0, 0, nq, nq) = D + HessianC_lambda;
		grad_F.block(0, nq, nq, 7) = grad_C.transpose();
		grad_F.block(nq, 0, 7, nq) = grad_C;
		if (grad_F.determinant() < 1e-6)
		{
			//EAE6320_ASSERTF(false, "grad_f is not invertable");
			grad_F = grad_F + regularization;
		}
		_Vector p(n);
		p.setZero();
		p = grad_F.inverse() * b;
		x.segment(0, nq) = x.segment(0, nq) + p.segment(0, nq);
		x.segment(nq, energeMomentumConstraintDim) = p.segment(nq, energeMomentumConstraintDim);
		
		qdot = x.segment(0, totalVelDOF);
		ForwardAngularAndTranslationalVelocity(Ht, qdot);
		
		iter++;
	}
}

void eae6320::MultiBody::EnergyConstraintV2()//energy conservation + momentum conservation and energy interpolation
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
		ForwardAngularAndTranslationalVelocity(Ht, qdot_new);
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

void eae6320::MultiBody::EnergyConstraintV3()//energy constraint
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
		ForwardAngularAndTranslationalVelocity(Ht, qdot_new);
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

void eae6320::MultiBody::AcceleratedEnergyConstraint()//energy constraint
{
	Forward();

	int energeMomentumConstraintDim = 1;
	int nq = totalVelDOF;
	int n = nq + energeMomentumConstraintDim;
	
	_Vector mq(nq);
	mq.setZero();
	mq.segment(0, totalVelDOF) = qdot;
	
	_Matrix grad_C(energeMomentumConstraintDim, nq);
	grad_C.setZero();

	_Matrix DInv;
	DInv = MrInverse;

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

		ForwardAngularAndTranslationalVelocity(Ht, mq);
		energyErr = fabs(ComputeKineticEnergy() - kineticEnergyExpected);
		iter++;
	}
	qdot = mq;
	std::cout << "energy constraint iter: "<< iter << std::endl;
}

void eae6320::MultiBody::AcceleratedEnergyConstraintV2()//energy constraint, momentum constraint, velocity only, no alpha
{
	int energeMomentumConstraintDim = 7;
	int nq = totalVelDOF + 2;
	int n = nq + energeMomentumConstraintDim;

	_Vector mq(nq);
	mq.setZero();
	mq.segment(0, totalVelDOF) = qdot;

	_Matrix grad_C(energeMomentumConstraintDim, nq);
	grad_C.setZero();

	_Matrix DInv;
	DInv.resize(nq, nq);
	DInv.setZero();
	DInv.block(0, 0, totalVelDOF, totalVelDOF) = MrInverse;
	_Scalar coeff_s_t = 100;
	DInv(totalVelDOF, totalVelDOF) = 1.0 / coeff_s_t;
	DInv(totalVelDOF + 1, totalVelDOF + 1) = 1.0 / coeff_s_t;

	_Matrix Kp(3, totalVelDOF);
	Kp.setZero();
	_Matrix Kl(3, totalVelDOF);
	Kl.setZero();
	for (int i = 0; i < numOfLinks; i++)
	{
		Kp = Kp + Mbody[i].block<3, 3>(0, 0) * Ht[i].block(0, 0, 3, totalVelDOF);
		Kl = Kl + Mbody[i].block<3, 3>(3, 3) * Ht[i].block(3, 0, 3, totalVelDOF) + rigidBodyMass * Math::ToSkewSymmetricMatrix(pos[i]) * Ht[i].block(0, 0, 3, totalVelDOF);
	}
	grad_C.block(1, 0, 3, totalVelDOF) = Kp;
	grad_C.block(4, 0, 3, totalVelDOF) = Kl;
	
	_Scalar kineticEnergy0 = totalEnergy0 - ComputePotentialEnergy();
	_Vector3 linearMomentum1 = ComputeTranslationalMomentum();
	_Vector3 angularMomentum1 = ComputeAngularMomentum();
	grad_C.block(1, totalVelDOF, 3, 1) = linearMomentum1 - linearMomentum0;
	grad_C.block(4, totalVelDOF + 1, 3, 1) = angularMomentum1 - angularMomentum0;

	_Scalar energyErr = 1.0;
	_Matrix C(energeMomentumConstraintDim, 1);
	_Matrix lambdaNew(energeMomentumConstraintDim, 1);
	int iter = 0;
	while (true)
	{
		C(0, 0) = ComputeKineticEnergy() - kineticEnergy0;
		C.block<3, 1>(1, 0) = Kp * mq.segment(0, totalVelDOF) - linearMomentum1 - mq(totalVelDOF) * (linearMomentum0 - linearMomentum1);
		C.block<3, 1>(4, 0) = Kl * mq.segment(0, totalVelDOF) - angularMomentum1 - mq(totalVelDOF + 1) * (angularMomentum0 - angularMomentum1);
	
		_Scalar C_norm = C.norm();
		if (C_norm < 1e-6 || iter >= 20)
		{
			std::cout << "s " << mq(totalVelDOF) << " t " << mq(totalVelDOF + 1) << std::endl;
			std::cout << "C_norm " << C_norm << " iter " << iter << std::endl;
			break;
		}
		grad_C.block(0, 0, 1, totalVelDOF) = (Mr * mq.segment(0, totalVelDOF)).transpose();
	
		_Matrix K = grad_C * DInv * grad_C.transpose();
		if (K.determinant() < 1e-7)
		{
			_Matrix mI;
			mI.resize(energeMomentumConstraintDim, energeMomentumConstraintDim);
			mI.setIdentity();
			K = K + 1e-7 * mI;
		}
		lambdaNew = K.inverse() * C;
		_Vector delta_q;
		delta_q = DInv * grad_C.transpose() * lambdaNew;
		mq = mq - delta_q;
		
		qdot = mq.segment(0, totalVelDOF);
		ForwardAngularAndTranslationalVelocity(Ht, qdot);
		
		iter++;
	}
}

void eae6320::MultiBody::EnergyConstraintPositionVelocity()
{
	Forward();

	int posVelDof = 2 * totalVelDOF;
	int totalDof = posVelDof + 2;
	int energeMomentumConstraintDim = 7;
	_Matrix grad_C(energeMomentumConstraintDim, totalDof);
	grad_C.setZero();

	_Matrix D(totalDof, totalDof);
	D.setZero();
	D.block(0, 0, totalVelDOF, totalVelDOF) = Mr;
	_Scalar dt = pApp->GetSimulationUpdatePeriod_inSeconds();
	D.block(totalVelDOF, totalVelDOF, totalVelDOF, totalVelDOF) = dt * dt * Mr;
	_Scalar coeff_s_t = 1e-3;
	D(posVelDof, posVelDof) = coeff_s_t;
	D(posVelDof + 1, posVelDof + 1) = coeff_s_t;
	_Matrix DInv = D.inverse();

	_Vector mq(totalDof);
	mq.setZero();
	mq.segment(0, totalVelDOF) = q;
	mq.segment(totalVelDOF, totalVelDOF) = qdot;
	
	_Vector3 linearMomentum1 = ComputeTranslationalMomentum();
	_Vector3 angularMomentum1 = ComputeAngularMomentum();
	
	_Matrix M0(1, totalVelDOF);
	_Matrix M1(3, totalVelDOF);
	_Matrix M2(3, totalVelDOF);
	_Matrix Kp(3, totalVelDOF);
	_Matrix Kl(3, totalVelDOF);

	_Matrix C(energeMomentumConstraintDim, 1);
	_Matrix lambdaNew(energeMomentumConstraintDim, 1);
	int iter = 0;
	while (true)
	{
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
		
		//ComputeJacobianAndInertiaDerivativeFD(qdot, bm, HtDerivativeTimes_b, MassMatrixDerivativeTimes_b, 1e-6);
		ComputeJacobianAndInertiaDerivative(totalVelDOF, qdot, bm, q, Ht, H, HtDerivativeTimes_b, MassMatrixDerivativeTimes_b);
		M0.setZero();
		M1.setZero();
		M2.setZero();
		Kp.setZero();
		Kl.setZero();
		for (int i = 0; i < numOfLinks; i++)
		{
			//***********************kinetic energy derivative*****************************
			M0 = M0 + qdot.transpose() * Ht[i].transpose() * Mbody[i] * HtDerivativeTimes_b[i] + 0.5 * qdot.transpose() * Ht[i].transpose() * MassMatrixDerivativeTimes_b[i];
			//***********************potential energy derivative*****************************
			_Vector3 g(0.0f, 9.81f, 0.0f);
			M0 = M0 + g.transpose() * Mbody[i].block<3, 3>(0, 0) * Ht[i].block(0, 0, 3, totalVelDOF);

			//***********************linear monentum derivative*****************************
			M1 = M1 + Mbody[i].block<3, 3>(0, 0) * HtDerivativeTimes_b[i].block(0, 0, 3, totalVelDOF);
			Kp = Kp + Mbody[i].block<3, 3>(0, 0) * Ht[i].block(0, 0, 3, totalVelDOF);
			//***********************angular monentum derivative*****************************
			M2 = M2 + MassMatrixDerivativeTimes_b[i].block(3, 0, 3, totalVelDOF) + Mbody[i].block<3, 3>(3, 3) * HtDerivativeTimes_b[i].block(3, 0, 3, totalVelDOF)
				+ rigidBodyMass * Math::ToSkewSymmetricMatrix(pos[i]) * HtDerivativeTimes_b[i].block(0, 0, 3, totalVelDOF) - rigidBodyMass * Math::ToSkewSymmetricMatrix(vel[i]) * Ht[i].block(0, 0, 3, totalVelDOF);
			Kl = Kl + Mbody[i].block<3, 3>(3, 3) * Ht[i].block(3, 0, 3, totalVelDOF) + rigidBodyMass * Math::ToSkewSymmetricMatrix(pos[i]) * Ht[i].block(0, 0, 3, totalVelDOF);
		}
		
		C(0, 0) = ComputeTotalEnergy() - totalEnergy0;
		C.block<3, 1>(1, 0) = Kp * mq.segment(totalVelDOF, totalVelDOF) - linearMomentum1 - mq(posVelDof) * (linearMomentum0 - linearMomentum1);
		C.block<3, 1>(4, 0) = Kl * mq.segment(totalVelDOF, totalVelDOF) - angularMomentum1 - mq(posVelDof + 1) * (angularMomentum0 - angularMomentum1);
		_Scalar C_norm = C.norm();
		if (C_norm < 1e-6 || iter >= 20)
		{
			break;
		}
		
		grad_C.block(0, 0, 1, totalVelDOF) = M0;
		grad_C.block(0, totalVelDOF, 1, totalVelDOF) = (Mr * qdot).transpose();
		
		grad_C.block(1, 0, 3, totalVelDOF) = M1;
		grad_C.block(1, totalVelDOF, 3, totalVelDOF) = Kp;
		grad_C.block(1, posVelDof, 3, 1) = linearMomentum1 - linearMomentum0;

		grad_C.block(4, 0, 3, totalVelDOF) = M2;
		grad_C.block(4, totalVelDOF, 3, totalVelDOF) = Kl;
		grad_C.block(4, posVelDof + 1, 3, 1) = angularMomentum1 - angularMomentum0;

		D.block(0, 0, totalVelDOF, totalVelDOF) = Mr;
		D.block(totalVelDOF, totalVelDOF, totalVelDOF, totalVelDOF) = dt * dt * Mr;
		DInv = D.inverse();
		_Matrix K = grad_C * DInv * grad_C.transpose();
		//std::cout << std::setprecision(16) << "K " << K << std::endl << std::endl;
		if (K.determinant() < 1e-7)
		{
			_Matrix mI;
			mI.resize(energeMomentumConstraintDim, energeMomentumConstraintDim);
			mI.setIdentity();
			K = K + 1e-7 * mI;
		}
		lambdaNew = K.inverse() * C;
		_Vector delta_q = -DInv * grad_C.transpose() * lambdaNew;
		mq = mq + delta_q;

		//update D's position part
		q = mq.segment(0, totalVelDOF);
		qdot = mq.segment(totalVelDOF, totalVelDOF);
		Forward();
		iter++;
	}
	std::cout << "energy constraint iter: " << iter << " s " << mq(posVelDof) << " t " << mq(posVelDof + 1) << std::endl;
}

void eae6320::MultiBody::ComputeJacobianAndInertiaDerivativeFDV2(_Vector& i_x, _Vector& i_bj, std::vector<_Vector>& i_bm, std::vector<_Matrix>& o_Jacobian, std::vector<_Matrix>& o_intertia, _Scalar i_delta)
{
	_Scalar delta = i_delta;

	_Matrix d0, d1;
	d0.resize(6, 1);
	d1.resize(6, 1);

	_Vector old_x;
	old_x = i_x;

	int dof = static_cast<int>(x.size());
	std::vector<_Vector> perturbed_x;
	perturbed_x.resize(dof);
	for (int i = 0; i < dof; i++)
	{
		perturbed_x[i].resize(dof);
		perturbed_x[i] = x;
		perturbed_x[i](i) = perturbed_x[i](i) + delta;
	}

	for (int i = 0; i < numOfLinks; i++)
	{
		ComputeHt(Ht, H, old_x, rel_ori, xJointType, xStartIndex);
		o_Jacobian[i].resize(6, dof);
		d0 = Ht[i] * i_bj;
		for (int k = 0; k < dof; k++)
		{
			ComputeHt(Ht, H, perturbed_x[k], rel_ori, xJointType, xStartIndex);
			d1 = Ht[i] * i_bj;
			o_Jacobian[i].block<6, 1>(0, k) = (d1 - d0) / delta;
		}
	}

	for (int i = 0; i < numOfLinks; i++)
	{
		ForwardKinematics(old_x, rel_ori, xJointType, xStartIndex);
		o_intertia[i].resize(6, dof);
		d0 = Mbody[i] * i_bm[i];
		for (int k = 0; k < dof; k++)
		{
			ForwardKinematics(perturbed_x[k], rel_ori, xJointType, xStartIndex);
			d1 = Mbody[i] * i_bm[i];
			o_intertia[i].block<6, 1>(0, k) = (d1 - d0) / delta;
		}
	}
}

_Matrix eae6320::MultiBody::Compute_dHOmega_dr(int joint_id, _Vector& i_x, _Vector i_bj)
{
	_Matrix out;
	int i = joint_id;
	if (xJointType[i] == BALL_JOINT_3D)
	{
		out.resize(3, 3);

		_Vector3 r = i_x.segment(xStartIndex[i], 3);
		_Scalar th = r.norm();
		_Scalar th2 = th * th;

		_Scalar beta, gamma, cbeta, cgamma;
		if (th < 1e-4)
		{
			beta = 0.5 - th2 / 24.0;
			gamma = 1.0 / 6.0 - th2 / 120.0;
			cbeta = -1.0 / 12.0 + th2 / 180.0;
			cgamma = -1.0 / 60.0 + th2 / 1260.0;
		}
		else
		{
			beta = (1 - cos(th)) / th2;
			gamma = (th - sin(th)) / (th2 * th);
			cbeta = (th * sin(th) - 2 + 2 * cos(th)) / (th2 * th2);
			cgamma = (-th * cos(th) - 2 * th + 3 * sin(th)) / (th2 * th2 * th);
		}

		_Vector3 b = i_bj.segment(velStartIndex[i], 3);
		_Matrix3 bx = Math::ToSkewSymmetricMatrix(b);           // [b]_x
		_Matrix3 rx = Math::ToSkewSymmetricMatrix(r);           // [r]_x
		_Vector3 rxb = r.cross(b);       // r × b
		_Vector3 rx2b = r * (r.dot(b)) - b * th2; // [r]_x^2 b

		_Matrix3 term1 = -beta * bx;
		_Matrix3 term2 = rxb * r.transpose() * cbeta;
		_Matrix3 term3 = gamma * (r * b.transpose() + r.dot(b) * _Matrix3::Identity() - 2.0 * b * r.transpose());
		_Matrix3 term4 = rx2b * r.transpose() * cgamma;

		out = term1 + term2 + term3 + term4;
	}
	else if (jointType[i] == HINGE_JOINT)
	{
		out.resize(3, 1);
		out.setZero();
	}
	return out;
}


void eae6320::MultiBody::ComputeJacobianAndInertiaDerivative(int i_totalDOF, _Vector& i_bj, std::vector<_Vector>& i_bm, _Vector& i_x, std::vector<_Matrix>& i_Ht, std::vector<_Matrix>& i_H,
	std::vector<_Matrix>& o_Jacobian, std::vector<_Matrix>& o_intertia)
{
	_Matrix mN;
	_Matrix mB;
	_Matrix mE;
	_Matrix mA;
	for (int i = 0; i < numOfLinks; i++)
	{
		int sz = 6 + velDOF[i];	
		int j = parentArr[i];
		//ComputeN
		mN.resize(sz, i_totalDOF);
		mN.setZero();
		if (i > 0)
		{
			mN.block(0, 0, 3, i_totalDOF) = i_Ht[j].block(3, 0, 3, i_totalDOF);
		}
		mN.block(3, 0, 3, i_totalDOF) = i_Ht[i].block(3, 0, 3, i_totalDOF);
		for (int k = 0; k < velDOF[i]; k++)
		{
/*
This funciton will only be called when all ball joints are converted into exponential representation. If a hybrid of exponential and omega is used for ball
joints, then velStartIndex need to be replaced by posStartIndex
*/
			mN(6 + k, velStartIndex[i] + k) = 1;
		}

		//ComputeB
		mB.resize(6, sz);
		mB.setZero();
		_Vector3 Vec3;
		Vec3 = i_H[i].block(3, 0, 3, velDOF[i]) * i_bj.segment(velStartIndex[i], velDOF[i]); //qdot.segment(velStartIndex[i], velDOF[i]) is b
		mB.block<3, 3>(0, 0) = -Math::ToSkewSymmetricMatrix(uGlobalsChild[i]) * Math::ToSkewSymmetricMatrix(Vec3);
		mB.block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(Vec3) * Math::ToSkewSymmetricMatrix(uGlobalsChild[i]);
		mB.block<3, 3>(3, 0) = -Math::ToSkewSymmetricMatrix(Vec3);
		if (i == 0)
		{
			mB.block(0, 6, 3, velDOF[i]) = Math::ToSkewSymmetricMatrix(uGlobalsChild[i]) * Compute_dHOmega_dr(i, i_x, i_bj);
			mB.block(3, 6, 3, velDOF[i]) = Compute_dHOmega_dr(i, i_x, i_bj);
		}
		else
		{
			mB.block(0, 6, 3, velDOF[i]) = Math::ToSkewSymmetricMatrix(uGlobalsChild[i]) * R_global[j] * Compute_dHOmega_dr(i, i_x, i_bj);
			mB.block(3, 6, 3, velDOF[i]) = R_global[j] * Compute_dHOmega_dr(i, i_x, i_bj);
		}
		
		//ComputeHtDerivativeTimes_b
		if (i == 0)
		{
			o_Jacobian[0] = mB * mN;
		}
		else
		{
			//ComputeA
			_Vector3 b2;
			b2 = (i_Ht[j] * i_bj).segment(3, 3);
			mA.resize(6, sz);
			mA.setZero();
			_Vector3 Vec3;
			Vec3 = hingeMagnitude[i] * hingeDirGlobals[i];
			mA.block<3, 3>(0, 0) = -Math::ToSkewSymmetricMatrix(b2) * (Math::ToSkewSymmetricMatrix(uGlobalsParent[i]) + Math::ToSkewSymmetricMatrix(Vec3));
			mA.block<3, 3>(0, 3) = Math::ToSkewSymmetricMatrix(b2) * Math::ToSkewSymmetricMatrix(uGlobalsChild[i]);

			o_Jacobian[i] = D[i] * o_Jacobian[j] + mA * mN + mB * mN;
		}
		//ComputeMassMatrixDerivativeTimes_b
		//ComputeE
		mE.resize(6, sz);
		mE.setZero();
		_Vector3 b1 = i_bm[i].segment(0, 3);
		_Vector3 b2 = i_bm[i].segment(3, 3);
		_Vector3 V0 = Mbody[i].block<3, 3>(0, 3) * b2;
		mE.block<3, 3>(0, 3) = -Math::ToSkewSymmetricMatrix(V0) + Mbody[i].block<3, 3>(0, 3) * Math::ToSkewSymmetricMatrix(b2);
		_Vector3 V1 = Mbody[i].block<3, 3>(3, 0) * b1;
		_Vector3 V2 = Mbody[i].block<3, 3>(3, 3) * b2;
		mE.block<3, 3>(3, 3) = -Math::ToSkewSymmetricMatrix(V1) + Mbody[i].block<3, 3>(3, 0) * Math::ToSkewSymmetricMatrix(b1)
			- Math::ToSkewSymmetricMatrix(V2) + Mbody[i].block<3, 3>(3, 3) * Math::ToSkewSymmetricMatrix(b2);
		o_intertia[i] = mE * mN;
	}
}

void eae6320::MultiBody::ComputeDxOverDpFD(std::vector<_Matrix>& o_derivative, _Vector& i_x, _Scalar i_delta)
{
	_Scalar delta = i_delta;

	_Vector3 d0, d1;

	_Vector old_x;
	old_x = i_x;

	int dof = static_cast<int>(x.size());
	std::vector<_Vector> perturbed_x;
	perturbed_x.resize(dof);
	for (int i = 0; i < dof; i++)
	{
		perturbed_x[i].resize(dof);
		perturbed_x[i] = x;
		perturbed_x[i](i) = perturbed_x[i](i) + delta;
	}

	for (int i = 0; i < numOfLinks; i++)
	{
		o_derivative[i].resize(3, dof);
		ForwardKinematics(old_x, rel_ori, xJointType, xStartIndex);
		d0 = pos[i];
		for (int k = 0; k < dof; k++)
		{
			ForwardKinematics(perturbed_x[k], rel_ori, xJointType, xStartIndex);
			d1 = pos[i];
			o_derivative[i].block<3, 1>(0, k) = (d1 - d0) / delta;
		}
	}
}


//_Matrix eae6320::MultiBody::ComputeDhGlobalOverDp(int i)
//{
//	_Matrix output;
//	output.resize(3, totalPosDOF);
//	_Vector3 h;
//	h = hingeMagnitude[i] * hingeDirGlobals[i];
//	int j = parentArr[i];
//	//output = -Math::ToSkewSymmetricMatrix(h) * Gt[j].block(3, 0, 3, totalPosDOF);
//	output = -Math::ToSkewSymmetricMatrix(h) * Ht[j].block(3, 0, 3, totalPosDOF);//TODO: replace Ht with the input of the function
//	return output;
//}

void eae6320::MultiBody::Populate_q(std::vector<_Quat>& i_quat, _Vector& o_q)
{
	for (int i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_4D)
		{
			o_q(posStartIndex[i]) = i_quat[i].w();
			o_q(posStartIndex[i] + 1) = i_quat[i].x();
			o_q(posStartIndex[i] + 2) = i_quat[i].y();
			o_q(posStartIndex[i] + 3) = i_quat[i].z();
		}
	}
}

void eae6320::MultiBody::Populate_quat(_Vector& i_q, std::vector<_Quat>& o_quat, bool normalization)
{
	for (int i = 0; i < numOfLinks; i++)
	{
		if (jointType[i] == BALL_JOINT_4D)
		{
			_Scalar w = i_q(posStartIndex[i]);
			_Scalar x = i_q(posStartIndex[i] + 1);
			_Scalar y = i_q(posStartIndex[i] + 2);
			_Scalar z = i_q(posStartIndex[i] + 3);
			if (!normalization)
			{
				o_quat[i].w() = w;
				o_quat[i].x() = x;
				o_quat[i].y() = y;
				o_quat[i].z() = z;
			}
			else
			{
				_Scalar norm = sqrt(x * x + y * y + z * z + w * w);
				o_quat[i].w() = w / norm;
				o_quat[i].x() = x / norm;
				o_quat[i].y() = y / norm;
				o_quat[i].z() = z / norm;
				i_q.segment(posStartIndex[i], 4) = i_q.segment(posStartIndex[i], 4) / norm;
			}
		}
	}
}

void eae6320::MultiBody::ComputeExponentialMapJacobian(_Vector& i_x, std::vector<int>& i_jointType, std::vector<int>& i_posStartIndex)
{
	for (int i = 0; i < numOfLinks; i++)
	{
		if (i_jointType[i] == BALL_JOINT_3D)
		{
			_Vector3 r = i_x.segment(i_posStartIndex[i], 3);
			ComputeExponentialMapJacobian(J_exp[i], r, i);
		}
	}
}

void eae6320::MultiBody::UpdateXdot(_Vector& o_xdot, _Vector& i_qdot, std::vector<int>& i_jointType)
{
	for (int i = 0; i < numOfLinks; i++)
	{
		if (i_jointType[i] == BALL_JOINT_4D)
		{
			o_xdot.segment(xStartIndex[i], 3) = J_exp[i].inverse() * i_qdot.segment(velStartIndex[i], 3);
		}
		else
		{
			o_xdot.segment(xStartIndex[i], xDOF[i]) = i_qdot.segment(velStartIndex[i], velDOF[i]);
		}
	}
}

void eae6320::MultiBody::UpdateQdot(_Vector& o_qdot, _Vector& i_xdot, std::vector<int>& i_jointType)
{
	for (int i = 0; i < numOfLinks; i++)
	{
		if (i_jointType[i] == BALL_JOINT_4D)
		{
			o_qdot.segment(velStartIndex[i], 3) = J_exp[i] * i_xdot.segment(xStartIndex[i], 3);
		}
		else
		{
			o_qdot.segment(velStartIndex[i], velDOF[i]) = i_xdot.segment(xStartIndex[i], xDOF[i]);
		}
	}
}