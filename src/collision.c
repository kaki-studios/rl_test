#include "collision.h"
#include <float.h>
#include <raylib.h>
#include <raymath.h>
#include <stdio.h>

// Helper function to get axis from rotation matrix
Vector3 GetMatrixAxis(Matrix m, int axis) {
  switch (axis) {
  case 0:
    return (Vector3){m.m0, m.m1, m.m2}; // X axis
  case 1:
    return (Vector3){m.m4, m.m5, m.m6}; // Y axis
  case 2:
    return (Vector3){m.m8, m.m9, m.m10}; // Z axis
  default:
    return (Vector3){1, 0, 0};
  }
}

// Project OBB onto an axis and return min/max projection values
void ProjectOBB(Vector3 center, Vector3 halfSize, Matrix rotation, Vector3 axis,
                float *min, float *max) {
  // Get the three axes of the OBB
  Vector3 axisX = GetMatrixAxis(rotation, 0);
  Vector3 axisY = GetMatrixAxis(rotation, 1);
  Vector3 axisZ = GetMatrixAxis(rotation, 2);

  // Project center onto axis
  float centerProjection = Vector3DotProduct(center, axis);

  // Calculate the projection of the half-extents onto the axis
  float extent = fabsf(Vector3DotProduct(axisX, axis)) * halfSize.x +
                 fabsf(Vector3DotProduct(axisY, axis)) * halfSize.y +
                 fabsf(Vector3DotProduct(axisZ, axis)) * halfSize.z;

  *min = centerProjection - extent;
  *max = centerProjection + extent;
}

// Test separation along an axis and update MTV if needed
bool TestSeparatingAxis(Vector3 p1, Vector3 s1, Matrix r1, Vector3 p2,
                        Vector3 s2, Matrix r2, Vector3 axis, float *minOverlap,
                        Vector3 *mtvAxis) {
  // Normalize the axis
  float axisLength = Vector3Length(axis);
  if (axisLength < 1e-6f)
    return false; // Degenerate axis

  axis = Vector3Scale(axis, 1.0f / axisLength);

  float min1, max1, min2, max2;
  ProjectOBB(p1, s1, r1, axis, &min1, &max1);
  ProjectOBB(p2, s2, r2, axis, &min2, &max2);

  // Check for separation
  if (max1 < min2 || max2 < min1) {
    return true; // Separated
  }

  // Calculate overlap
  float overlap = fminf(max1, max2) - fmaxf(min1, min2);

  // Update MTV if this is the smallest overlap so far
  if (overlap < *minOverlap) {
    *minOverlap = overlap;
    *mtvAxis = axis;

    // Ensure MTV points from OBB1 to OBB2
    Vector3 centerDiff = Vector3Subtract(p2, p1);
    if (Vector3DotProduct(centerDiff, axis) < 0) {
      *mtvAxis = Vector3Scale(axis, -1.0f);
    }
  }

  return false; // Not separated
}

bool OBBvsOBB(Vector3 p1, Quaternion q1, Vector3 s1, Vector3 p2, Quaternion q2,
              Vector3 s2, Vector3 *mtv_out) {
  // Convert quaternions to rotation matrices
  Matrix r1 = QuaternionToMatrix(q1);
  Matrix r2 = QuaternionToMatrix(q2);

  // Half-sizes for easier calculations
  Vector3 hs1 = Vector3Scale(s1, 0.5f);
  Vector3 hs2 = Vector3Scale(s2, 0.5f);

  // Track minimum overlap and MTV
  float minOverlap = FLT_MAX;
  Vector3 mtvAxis = {1, 0, 0};

  // Test the 15 potential separating axes using SAT

  // 1-3: Face normals of OBB1
  for (int i = 0; i < 3; i++) {
    Vector3 axis = GetMatrixAxis(r1, i);
    if (TestSeparatingAxis(p1, hs1, r1, p2, hs2, r2, axis, &minOverlap,
                           &mtvAxis)) {
      return false; // Separated
    }
  }

  // 4-6: Face normals of OBB2
  for (int i = 0; i < 3; i++) {
    Vector3 axis = GetMatrixAxis(r2, i);
    if (TestSeparatingAxis(p1, hs1, r1, p2, hs2, r2, axis, &minOverlap,
                           &mtvAxis)) {
      return false; // Separated
    }
  }

  // 7-15: Cross products of edge directions
  for (int i = 0; i < 3; i++) {
    Vector3 axis1 = GetMatrixAxis(r1, i);
    for (int j = 0; j < 3; j++) {
      Vector3 axis2 = GetMatrixAxis(r2, j);
      Vector3 crossAxis = Vector3CrossProduct(axis1, axis2);

      // Skip near-parallel axes
      if (Vector3Length(crossAxis) > 1e-6f) {
        if (TestSeparatingAxis(p1, hs1, r1, p2, hs2, r2, crossAxis, &minOverlap,
                               &mtvAxis)) {
          return false; // Separated
        }
      }
    }
  }

  // No separating axis found - collision detected
  if (mtv_out) {
    *mtv_out = Vector3Scale(mtvAxis, minOverlap);
  }

  return true;
}

void ApplyImpulse(RigidBody *a, RigidBody *b, Vector3 contactPoint,
                  Vector3 normal, float penetrationDepth) {
  float restitution = (a->restitution + b->restitution) * 0.5f;
  float friction = (a->friction + b->friction) * 0.5f;

  Vector3 ra = Vector3Subtract(contactPoint, a->position);
  Vector3 rb = Vector3Subtract(contactPoint, b->position);

  // Recompute angular linearVelocity from angular momentum
  Matrix3 Ia_inv;
  Vector3 ACoM;
  float Amass;
  MeshComputeInertiaMatrix(a->mesh, a->density, &Ia_inv, &ACoM, &Amass);

  Matrix3 Ib_inv;
  Vector3 BCoM;
  float Bmass;
  MeshComputeInertiaMatrix(b->mesh, b->density, &Ib_inv, &BCoM, &Bmass);
  Vector3 wa = Vector3Transform(a->angularMomentum, Matrix3ToMatrix(Ia_inv));
  Vector3 wb = Vector3Transform(b->angularMomentum, Matrix3ToMatrix(Ib_inv));

  // Compute linearVelocity at contact point
  Vector3 va = Vector3Add(a->linearVelocity, Vector3CrossProduct(wa, ra));
  Vector3 vb = Vector3Add(b->linearVelocity, Vector3CrossProduct(wb, rb));
  Vector3 relativelinearVelocity = Vector3Subtract(vb, va);

  float contactVel = Vector3DotProduct(relativelinearVelocity, normal);

  if (contactVel > 0.0f)
    return; // No impulse needed if separating

  // Calculate impulse scalar
  Vector3 raCrossN = Vector3CrossProduct(ra, normal);
  Vector3 rbCrossN = Vector3CrossProduct(rb, normal);
  Vector3 Ia_raCrossN = Vector3Transform(raCrossN, Matrix3ToMatrix(Ia_inv));
  Vector3 Ib_rbCrossN = Vector3Transform(rbCrossN, Matrix3ToMatrix(Ib_inv));

  float denom =
      (1.0f / Amass) + (1.0f / Bmass) +
      Vector3DotProduct(Vector3CrossProduct(Ia_raCrossN, ra), normal) +
      Vector3DotProduct(Vector3CrossProduct(Ib_rbCrossN, rb), normal);

  float j = -(1.0f + restitution) * contactVel / denom;
  Vector3 impulse = Vector3Scale(normal, j);

  // Apply linear impulse
  a->linearVelocity =
      Vector3Subtract(a->linearVelocity, Vector3Scale(impulse, 1.0f / Amass));
  b->linearVelocity =
      Vector3Add(b->linearVelocity, Vector3Scale(impulse, 1.0f / Bmass));

  // Apply angular impulse to angular momentum
  a->angularMomentum =
      Vector3Subtract(a->angularMomentum, Vector3CrossProduct(ra, impulse));
  b->angularMomentum =
      Vector3Add(b->angularMomentum, Vector3CrossProduct(rb, impulse));

  // --- Friction impulse ---
  Vector3 tangent = Vector3Subtract(
      relativelinearVelocity,
      Vector3Scale(normal, Vector3DotProduct(relativelinearVelocity, normal)));
  if (Vector3LengthSqr(tangent) > EPSILON) {
    tangent = Vector3Normalize(tangent);
    float jt = -Vector3DotProduct(relativelinearVelocity, tangent) / denom;
    jt = Clamp(jt, -j * friction, j * friction);

    Vector3 frictionImpulse = Vector3Scale(tangent, jt);

    a->linearVelocity = Vector3Subtract(
        a->linearVelocity, Vector3Scale(frictionImpulse, 1.0f / Amass));
    b->linearVelocity = Vector3Add(b->linearVelocity,
                                   Vector3Scale(frictionImpulse, 1.0f / Bmass));

    a->angularMomentum = Vector3Subtract(
        a->angularMomentum, Vector3CrossProduct(ra, frictionImpulse));
    b->angularMomentum = Vector3Add(b->angularMomentum,
                                    Vector3CrossProduct(rb, frictionImpulse));
  }
}

static int collisionCount = 0;
void HandleCuboidRBCollisions(RigidBody *a, RigidBody *b, Vector3 aDims,
                              Vector3 bDims) {

  Vector3 mtv;
  if (OBBvsOBB(a->position, a->rotation, aDims, b->position, b->rotation, bDims,
               &mtv)) {
    collisionCount++;
    printf("COLLISION FOUND, COUNT: %d\n", collisionCount);
    float penetration = Vector3Length(mtv);
    Vector3 normal = Vector3Normalize(mtv);
    Vector3 contactPoint = Vector3Lerp(a->position, b->position,
                                       0.5f); // approximate contact point
    const float slop = 0.0001f;
    if (penetration > slop) {
      // Resolve position
      Vector3 correction = Vector3Scale(normal, (penetration - slop) * 0.5f);
      a->position = Vector3Subtract(a->position, correction);
      b->position = Vector3Add(b->position, correction);
      // Resolve velocity
      ApplyImpulse(a, b, contactPoint, normal, penetration);
    }
  }
}
