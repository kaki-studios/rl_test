#include "rigidbody.h"
#include <raylib.h>
#include <raymath.h>

// helper functions for dealing with cuboid rigidbodies

float MassFromDensity(float, Vector3);
RigidBody CreateCuboidRB(float, Vector3, Vector3);

Matrix3 CuboidComputeInvInertiaMatrix(Vector3, float);
