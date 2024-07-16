#pragma once

#ifndef LOCAL_MODE
#define LOCAL_MODE 0
#endif
#ifndef MUJOCO_MODE
#define MUJOCO_MODE 1
#endif
/*************************************/
#ifndef BOX
#define BOX 0
#endif
#ifndef BALL
#define BALL 1
#endif
/*************************************/
#ifndef PBD
#define PBD 0
#endif

#ifndef IMPULSE
#define IMPULSE 1
#endif
/*************************************/
#ifndef BALL_JOINT_3D
#define BALL_JOINT_3D 0
#endif

#ifndef BALL_JOINT_4D
#define BALL_JOINT_4D 1
#endif

#ifndef FREE_JOINT
#define FREE_JOINT 2
#endif

#ifndef HINGE_JOINT
#define HINGE_JOINT 3
#endif
/*************************************/
#ifndef TWIST_WITHOUT_SWING
#define TWIST_WITHOUT_SWING 0
#endif

#ifndef TWIST_WITH_SWING
#define TWIST_WITH_SWING 1
#endif

#ifndef SWING
#define SWING 2
#endif

#ifndef ROTATION_MAGNITUDE_LIMIT
#define ROTATION_MAGNITUDE_LIMIT 3
#endif
/*************************************/
#ifndef SWING_C
#define SWING_C 0
#endif

#ifndef TWIST_C
#define TWIST_C 1
#endif