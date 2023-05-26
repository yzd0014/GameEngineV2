#pragma once
#include "Engine/GameCommon/GameObject.h"
#include "Engine/UserInput/UserInput.h"
#include <math.h>

#include "External/EigenLibrary/Eigen/Dense"

using namespace Eigen;
namespace eae6320
{
	class MPM : public eae6320::GameCommon::GameObject
	{
	public:
		MPM(Effect * i_pEffect, eae6320::Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, float i_h) :
			GameCommon::GameObject(i_pEffect, i_Mesh, i_State),
			dt(i_h)
		{
			n_particles = 15 * 15 * 15;
			n_grid = 16;
			dx = 5.0f / n_grid;
			inv_dx = 1.0f / dx;
			p_vol = dx * dx * dx;
			p_density = 1.0f;
			p_mass = p_vol * p_density;
			E = 400;
			
			x.resize(n_particles);
			v.resize(n_particles);
			C.resize(n_particles);
			J.resize(n_particles);
			grid_v.resize(n_grid);
			for (int i = 0; i < n_grid; i++)
			{
				grid_v[i].resize(n_grid);
				for (int j = 0; j < n_grid; j++)
				{
					grid_v[i][j].resize(n_grid);
				}
			}
			grid_m.resize(n_grid);
			for (int i = 0; i < n_grid; i++)
			{
				grid_m[i].resize(n_grid);
				for (int j = 0; j < n_grid; j++)
				{
					grid_m[i][j].resize(n_grid);
				}
			}
			fluid.resize(n_particles);
			
			//initialize pariticles positions
			float i_x = 1.5, i_y = 2, i_z = 1.5;
			int counter = 0;
			for (int i = 0; i < 15; i++)
			{
				i_y = 2;
				for (int j = 0; j < 15; j++)
				{
					i_z = 2;
					for (int k = 0; k < 15; k++)
					{
						x[counter] = Vector3f(i_x, i_y, i_z);
						counter++;
						i_z += 0.15f;
					}
					i_y += 0.15f;
				}
				i_x += 0.15f;
			}

			for (int i = 0; i < n_particles; i++)
			{
				//v[i].setZero();
				v[i] = Vector3f(0.0f, 0.0f, 0.0f);
				J[i] = 1.0f;
				C[i].setZero();
			}
		}
		void Tick(const float i_secondCountToIntegrate)
		{
			//reset grid
			for (int i = 0; i < n_grid; i++)
			{
				for (int j = 0; j < n_grid; j++)
				{
					for (int k = 0; k < n_grid; k++)
					{
						grid_v[i][j][k].setZero();
						grid_m[i][j][k] = 0;
					}
				}
			}
			//P2G
			for (int p = 0; p < n_particles; p++)
			{
				Vector3f base_f = x[p] * inv_dx - Vector3f(0.5f, 0.5f, 0.5f);
				Vector3i base;
				for (int i = 0; i < 3; i++) base(i) = (int)base_f(i);
				Vector3f fx = x[p] * inv_dx - base_f;
				Matrix3f w;
				w(0, 0) = 0.5f * pow(1.5f - fx(0), 2);
				w(0, 1) = 0.5f * pow(1.5f - fx(1), 2);
				w(0, 2) = 0.5f * pow(1.5f - fx(2), 2);
				w(1, 0) = 0.75f - pow(fx(0) - 1, 2);
				w(1, 1) = 0.75f - pow(fx(1) - 1, 2);
				w(1, 2) = 0.75f - pow(fx(2) - 1, 2);
				w(2, 0) = 0.5f * pow(fx(0) - 0.5f, 2);
				w(2, 1) = 0.5f * pow(fx(1) - 0.5f, 2);
				w(2, 2) = 0.5f * pow(fx(2) - 0.5f, 2);	
				float stress = -dt * p_vol * (J[p] - 1) * 4.0f * inv_dx * inv_dx * E;
				Matrix3f affine;
				affine = Matrix3f::Identity() * stress + p_mass * C[p];
				for (int i = 0; i < 3; i++)
				{
					for (int j = 0; j < 3; j++)
					{
						for (int k = 0; k < 3; k++)
						{
							Vector3f dpos = (Vector3f((float)i, (float)j, (float)k) - fx) * dx;
							float weight = w(i, 0) * w(j, 1) * w(k, 2);
							grid_v[base(0) + i][base(1) + j][base(2) + k] += weight * (p_mass * v[p] + affine * dpos);
							grid_m[base(0) + i][base(1) + j][base(2) + k] += weight * p_mass;
						}
					}
				}
			}
	
			//gird operation
			for (int i = 0; i < n_grid; i++)
			{
				for (int j = 0; j < n_grid; j++)
				{
					for (int k = 0; k < n_grid; k++)
					{
						if (grid_m[i][j][k] > 0)
						{
							int bound = 4;
							float inv_m = 1.0f / grid_m[i][j][k];
							grid_v[i][j][k] = inv_m * grid_v[i][j][k];
							grid_v[i][j][k](1) -= dt * gravity;
							if (i < bound && grid_v[i][j][k](0) < 0) grid_v[i][j][k](0) = 0;
							if (i > n_grid - bound && grid_v[i][j][k](0) > 0) grid_v[i][j][k](0) = 0;
							
							if (j < bound && grid_v[i][j][k](1) < 0) grid_v[i][j][k](1) = 0;
							if (j > n_grid - bound && grid_v[i][j][k](1) > 0) grid_v[i][j][k](1) = 0;

							if (k < bound && grid_v[i][j][k](2) < 0) grid_v[i][j][k](2) = 0;
							if (k > n_grid - bound && grid_v[i][j][k](2) > 0) grid_v[i][j][k](2) = 0;
						}
					}
				}
			}
	
			//G2P
			for (int p = 0; p < n_particles; p++)
			{
				Vector3f base_f = x[p] * inv_dx - Vector3f(0.5f, 0.5f, 0.5f);
				Vector3i base;
				for (int i = 0; i < 3; i++) base(i) = (int)base_f(i);
				Vector3f fx = x[p] * inv_dx - base_f;
				Matrix3f w;
				w(0, 0) = 0.5f * pow(1.5f - fx(0), 2);
				w(0, 1) = 0.5f * pow(1.5f - fx(1), 2);
				w(0, 2) = 0.5f * pow(1.5f - fx(2), 2);
				w(1, 0) = 0.75f - pow(fx(0) - 1, 2);
				w(1, 1) = 0.75f - pow(fx(1) - 1, 2);
				w(1, 2) = 0.75f - pow(fx(2) - 1, 2);
				w(2, 0) = 0.5f * pow(fx(0) - 0.5f, 2);
				w(2, 1) = 0.5f * pow(fx(1) - 0.5f, 2);
				w(2, 2) = 0.5f * pow(fx(2) - 0.5f, 2);
				Vector3f new_v;
				new_v.setZero();
				Matrix3f new_C;
				new_C.setZero();
				for (int i = 0; i < 3; i++)
				{
					for (int j = 0; j < 3; j++)
					{
						for (int k = 0; k < 3; k++)
						{
							Vector3f dpos = (Vector3f((float)i, (float)j, (float)k) - fx) * dx;
							Vector3f g_v = grid_v[base(0) + i][base(1) + j][base(2) + k];
							float weight = w(i, 0) * w(j, 1) * w(k, 2);
							new_v += weight * g_v;
							new_C += 4 * weight * g_v * dpos.transpose() * inv_dx;
						}
					}
				}
				v[p] = new_v;
				x[p] += dt * v[p];
				J[p] *= 1 + dt * new_C.trace();
				C[p] = new_C;
			}
		
			for (int i = 0; i < n_particles; i++)
			{
				fluid[i]->m_State.position.x = x[i](0);
				fluid[i]->m_State.position.y = x[i](1);
				fluid[i]->m_State.position.z = x[i](2);
			}
		}
		void UpdateGameObjectBasedOnInput()
		{
			if (UserInput::KeyState::currFrameKeyState['G'])
			{
				gravity = 9.8f;
			}
		}

		std::vector<GameCommon::GameObject *> fluid;
		std::vector<GameCommon::GameObject *> jelly;
		int n_particles;
	private:
		std::vector<Vector3f> x;
		std::vector<Vector3f> v;
		std::vector<Matrix3f> C;
		std::vector<float> J;
		std::vector<std::vector<std::vector<Vector3f>>> grid_v;
		std::vector<std::vector<std::vector<float>>> grid_m;

		float dt;
		int n_grid;
		float dx;
		float inv_dx;
		float p_vol;
		float p_density;
		float p_mass;
		float gravity = 0.0f;
		int E;
	};
}