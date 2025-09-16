#include "MultiBody.h"
#include "Engine/Physics/PhysicsSimulation.h"
#include "Engine/Math/sVector.h"
#include "Engine/Math/EigenHelper.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>

void eae6320::MultiBody::EnergyConstraint()//energy conservation + momentum conservation and mmentum interpolation
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

void eae6320::MultiBody::AcceleratedEnergyConstraint()//energy constraint
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
	std::cout << "energy constraint iter: "<< iter << std::endl;
}

void eae6320::MultiBody::EnergyConstraintPositionVelocity()
{
	std::vector<int> jointTypeCopy(jointType);//save original joint type
	std::vector<int> posStartIndexCopy(posStartIndex);//save original start index

	CopyFromQ2X();
	jointType = xJointType;
	posStartIndex = xStartIndex;
	UpdateXDot(xdot, x, qdot);
	
	int nq = 2 * totalVelDOF;
	int energeMomentumConstraintDim = 1;
	_Matrix grad_C(energeMomentumConstraintDim, nq);
	grad_C.setZero();

	std::vector<_Matrix> Ht_x;
	std::vector<_Matrix> H_x;
	Ht_x.resize(numOfLinks);
	H_x.resize(numOfLinks);	
	ComputeHt(Ht_x, H_x, x, rel_ori);
	_Matrix Mr_x;
	Mr_x.resize(totalVelDOF, totalVelDOF);
	ComputeMr(Mr_x, Ht_x);
	_Matrix D(nq, nq);
	D.setZero();
	D.block(0, 0, totalVelDOF, totalVelDOF) = Mr_x;
	D.block(totalVelDOF, totalVelDOF, totalVelDOF, totalVelDOF) = dt * dt * Mr;
	_Matrix DInv = D.inverse();

	_Vector mq(nq);
	mq.setZero();
	mq.segment(0, totalVelDOF) = x;
	mq.segment(totalVelDOF, totalVelDOF) = qdot;
	
	_Matrix C(energeMomentumConstraintDim, 1);
	C(0, 0) = ComputeTotalEnergy() - totalEnergy0;
	
	_Matrix lambdaNew(energeMomentumConstraintDim, 1);
	
	int iter = 0;
	_Vector mq_old = mq;//for debug
	while (abs(C(0, 0)) > 1e-3)
	{
		//if (iter >= 10)
		//{
		//	//std::cout << "limit reached!" << std::endl;
		//	break;
		//}
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
		jointType = xJointType;
		posStartIndex = xStartIndex;
		ComputeJacobianAndInertiaDerivative(totalVelDOF, xdot, bm, x, Ht_x, H_x, HtDerivativeTimes_b, MassMatrixDerivativeTimes_b);
		std::vector<_Matrix> positionDerivative;
		ComputeDxOverDp(positionDerivative, Ht_x, totalVelDOF);
		
		_Matrix M0;
		M0.resize(1, totalVelDOF);
		M0.setZero();
		for (int i = 0; i < numOfLinks; i++)
		{
			//***********************kinetic energy derivative*****************************
			M0 = M0 + qdot.transpose() * Ht_x[i].transpose() * Mbody[i] * HtDerivativeTimes_b[i] + 0.5 * qdot.transpose() * Ht_x[i].transpose() * MassMatrixDerivativeTimes_b[i];
			//***********************potential energy derivative*****************************
			_Vector3 g(0.0f, 9.81f, 0.0f);
			M0 = M0 + g.transpose() * Mbody[i].block<3, 3>(0, 0) * positionDerivative[i];
		}
		
		grad_C.block(0, 0, 1, totalVelDOF) = M0;
		grad_C.block(0, totalVelDOF, 1, totalVelDOF) = (Mr * qdot).transpose();
		_Matrix K = grad_C * DInv * grad_C.transpose();
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
		x = mq.segment(0, totalVelDOF);
		jointType = xJointType;
		posStartIndex = xStartIndex;
		ComputeHt(Ht_x, H_x, x, rel_ori);
		ComputeMr(Mr_x, Ht_x);
		D.block(0, 0, totalVelDOF, totalVelDOF) = Mr_x;
		
		//update D's velocity part
		jointType = jointTypeCopy;
		posStartIndex = posStartIndexCopy;
		CopyFromX2Q();
		ComputeHt(Ht, H, q, rel_ori);
		ComputeMr(Mr, Ht);
		D.block(totalVelDOF, totalVelDOF, totalVelDOF, totalVelDOF) = dt * dt * Mr;
		DInv = D.inverse();
		
		//update qdot
		qdot = mq.segment(totalVelDOF, totalVelDOF);
		ForwardAngularAndTranslationalVelocity(qdot);
		//update xdot
		jointType = xJointType;
		posStartIndex = xStartIndex;
		UpdateXDot(xdot, x, qdot);
		jointType = jointTypeCopy;
		posStartIndex = posStartIndexCopy;

		C(0, 0) = ComputeTotalEnergy() - totalEnergy0;
		iter++;
	}

	std::cout << "energy constraint iter: " << iter << std::endl;
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
		ComputeHt(Ht, H, old_x, rel_ori);
		o_Jacobian[i].resize(6, dof);
		d0 = Ht[i] * i_bj;
		for (int k = 0; k < dof; k++)
		{
			ComputeHt(Ht, H, perturbed_x[k], rel_ori);
			d1 = Ht[i] * i_bj;

			//std::cout << std::endl << std::setprecision(16) << d1 << std::endl;
			//std::cout << std::endl << std::setprecision(16) << d0 << std::endl;
			o_Jacobian[i].block<6, 1>(0, k) = (d1 - d0) / delta;
			//std::cout << std::endl << std::setprecision(16) << (d1 - d0) / delta << std::endl;
		}
	}

	for (int i = 0; i < numOfLinks; i++)
	{
		ForwardKinematics(old_x, rel_ori);
		o_intertia[i].resize(6, dof);
		d0 = Mbody[i] * i_bm[i];
		for (int k = 0; k < dof; k++)
		{
			ForwardKinematics(perturbed_x[k], rel_ori);
			d1 = Mbody[i] * i_bm[i];
			o_intertia[i].block<6, 1>(0, k) = (d1 - d0) / delta;
		}
	}
	Forward();
}

_Matrix eae6320::MultiBody::Compute_dHOmega_dr(int joint_id, _Vector& i_x, _Vector i_bj)
{
	_Matrix out;
	int i = joint_id;
	if (jointType[i] == BALL_JOINT_3D)
	{
		out.resize(3, 3);

		_Vector3 r = i_x.segment(posStartIndex[i], 3);
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

void eae6320::MultiBody::ComputeJacobianAndInertiaDerivative(int i_totalDOF, _Vector& i_bj, std::vector<_Vector>& i_bm,
	_Vector& i_x,
	std::vector<_Matrix>& i_Ht, std::vector<_Matrix>& i_H,
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
			mN(6 + k, posStartIndex[i] + k) = 1;
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

void eae6320::MultiBody::ComputeDxOverDp(std::vector<_Matrix>& o_derivative, std::vector<_Matrix>& i_Ht, int i_totalDOF)
{
	o_derivative.resize(numOfLinks);
	for (int i = 0; i < numOfLinks; i++)
	{
		o_derivative[i].resize(3, i_totalDOF);
		if (i == 0)
		{
			o_derivative[i] = -ComputeDuGlobalOverDp(i, uGlobalsChild[i], i_Ht, i_totalDOF);
		}
		else
		{
			o_derivative[i] = o_derivative[i - 1] - ComputeDuGlobalOverDp(i, uGlobalsChild[i], i_Ht, i_totalDOF) + ComputeDuGlobalOverDp(i, uGlobalsParent[i], i_Ht, i_totalDOF);
		}
	}
}

_Matrix eae6320::MultiBody::ComputeDuGlobalOverDp(int i, _Vector3& uGlobal, std::vector<_Matrix>& i_Ht, int i_totalDOF)
{
	_Matrix output;
	output.resize(3, i_totalDOF);
	//output = -Math::ToSkewSymmetricMatrix(uGlobal) * Gt[i].block(3, 0, 3, totalPosDOF);
	output = -Math::ToSkewSymmetricMatrix(uGlobal) * i_Ht[i].block(3, 0, 3, i_totalDOF);
	return output;
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