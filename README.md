# Physicis Engine

## Instructions to Run
1. Open with Visual Studio 2017
2. Go to Solution Explorer, right click "Halo"
3. Select "Properties", choose "Debugging"
4. Enter "$(GameInstallDir)$(TargetFileName)" in "Command", enter "$(GameInstallDir)" in "Working Directory"
5. Now you can compile and run
![](Images/instruction.png)
6. The executable can take two arguments, they are explained down below.
![](Images/arguments.png)

## Features
1. "HaloGame_\Halo\Custom Game Objects\Cloth.h" and its .cpp file implement cloth simulation with distance/bending constraint, self collision.
    
    <img src="Images/ClothSelfCollision.png" with="512" height="512">
    <img src="Images/ClothSim.png" width="512" height="512">




2. "HaloGame_\Halo\Custom Game Objects\JellyCube.h" implements elastic object simulation.
    
    <img src="Images/ElasticObject.png" width="512" height="512">




3. "HaloGame_\Halo\Custom Game Objects\Car.h" and its .cpp file implement car simulation by using constratint solver in maximum coordinate system.
    
    <img src="Images/MaximumConstraint.png" width="512" height="512">




4. "HaloGame_\Halo\Custom Game Objects\SphericalJoint.h" implments a spherical joint in reduced coordinate system.
    
    <img src="Images/ReducedConstraint.png" width="512" height="512">




5. "HaloGame_\Halo\Custom Game Objects\MPM.h" implements MPM.
    
    <img src="Images/MPM.png" width="512" height="512">