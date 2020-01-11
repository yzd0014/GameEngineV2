#include "sRigidBodyState.h"
#include <vector>
#include "Engine/EigenLibrary/Eigen/Dense"

using namespace Eigen;
namespace eae6320
{
	namespace Physics
	{
		void CollisionResolver(std::vector<ContactManifold3D>& i_allManifolds)
		{
			for (int k = 0; k < 10; k++)
			{
				for (size_t i = 0; i < i_allManifolds.size(); i++)
				{
					for (int j = 0; j < i_allManifolds[i].numContacts; j++)
					{
						MatrixXd V(12, 1);
					}
				}
			}
		} 
	}
}