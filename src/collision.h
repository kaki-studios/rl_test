#include "rigidbody.h"
#include <raylib.h>

bool OBBvsOBB(Vector3 p1, Quaternion q1, Vector3 s1, Vector3 p2, Quaternion q2,
              Vector3 s2, Vector3 *mtv_out);

void ApplyImpulse(RigidBody *a, RigidBody *b, Vector3 contactPoint,
                  Vector3 normal, float penetrationDepth);

void HandleCuboidCollisions(RigidBody *a, RigidBody *b, Vector3 aDims,
                            Vector3 bDims);
