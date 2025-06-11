#include "collision.h"
#include "cuboid_rb.h"
#include "rigidbody.h"
#include <float.h>
#include <raylib.h>
#include <raymath.h>
#include <stdio.h>

#define MAX_DEBUG_CONTACTS 128
static DebugContact debugContacts[MAX_DEBUG_CONTACTS];
static int debugContactCount = 0;

void ClearDebugContacts(void) { debugContactCount = 0; }

void PushDebugContact(Vector3 pos, Vector3 norm, float pen) {
  if (debugContactCount < MAX_DEBUG_CONTACTS) {
    debugContacts[debugContactCount++] = (DebugContact){pos, norm, pen};
  }
}

const DebugContact *GetDebugContacts(void) { return debugContacts; }

int GetDebugContactCount(void) { return debugContactCount; }

static int collisionCount = 0;
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

// Helper function to find the closest point on an OBB to a given point
Vector3 ClosestPointOnOBB(Vector3 point, Vector3 center, Vector3 halfSize,
                          Matrix rotation) {
  // Transform point to OBB's local space
  Vector3 localPoint = Vector3Subtract(point, center);

  // Get OBB axes (normalized)
  Vector3 axisX = GetMatrixAxis(rotation, 0);
  Vector3 axisY = GetMatrixAxis(rotation, 1);
  Vector3 axisZ = GetMatrixAxis(rotation, 2);

  // Project point onto each axis and clamp to half-extents
  float projX = Vector3DotProduct(localPoint, axisX);
  float projY = Vector3DotProduct(localPoint, axisY);
  float projZ = Vector3DotProduct(localPoint, axisZ);

  projX = Clamp(projX, -halfSize.x, halfSize.x);
  projY = Clamp(projY, -halfSize.y, halfSize.y);
  projZ = Clamp(projZ, -halfSize.z, halfSize.z);

  // Transform back to world space
  Vector3 closestPoint = center;
  closestPoint = Vector3Add(closestPoint, Vector3Scale(axisX, projX));
  closestPoint = Vector3Add(closestPoint, Vector3Scale(axisY, projY));
  closestPoint = Vector3Add(closestPoint, Vector3Scale(axisZ, projZ));

  return closestPoint;
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
  // Check if axis is degenerate (should be normalized before calling this
  // function)
  float axisLength = Vector3Length(axis);
  if (axisLength < 1e-6f)
    return false; // Degenerate axis

  // Ensure axis is normalized
  if (fabsf(axisLength - 1.0f) > 1e-6f) {
    axis = Vector3Scale(axis, 1.0f / axisLength);
  }

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
              Vector3 s2, Vector3 *mtv_out, Vector3 *contactPoint_out) {
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

  // 1-3: Face normals of OBB1 (should already be normalized from rotation
  // matrix)
  for (int i = 0; i < 3; i++) {
    Vector3 axis = GetMatrixAxis(r1, i);
    axis = Vector3Normalize(axis); // Ensure normalization
    if (TestSeparatingAxis(p1, hs1, r1, p2, hs2, r2, axis, &minOverlap,
                           &mtvAxis)) {
      return false; // Separated
    }
  }

  // 4-6: Face normals of OBB2 (should already be normalized from rotation
  // matrix)
  for (int i = 0; i < 3; i++) {
    Vector3 axis = GetMatrixAxis(r2, i);
    axis = Vector3Normalize(axis); // Ensure normalization
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

      // Skip near-parallel axes (when edges are nearly parallel)
      float crossLength = Vector3Length(crossAxis);
      if (crossLength > 1e-6f) {
        // Normalize the cross axis
        crossAxis = Vector3Scale(crossAxis, 1.0f / crossLength);
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

  // Calculate contact point if requested
  if (contactPoint_out) {
    // Use the MTV to determine which object's features are in contact
    // Move along the MTV from the center of the penetrating object
    Vector3 centerDiff = Vector3Subtract(p2, p1);

    // If MTV points from A to B, then A is penetrating into B
    if (Vector3DotProduct(mtvAxis, centerDiff) > 0) {
      // A is penetrating into B, so contact point is on B's surface
      Vector3 contactOnB =
          Vector3Subtract(p2, Vector3Scale(mtvAxis, minOverlap * 0.5f));
      *contactPoint_out = contactOnB;
    } else {
      // B is penetrating into A, so contact point is on A's surface
      Vector3 contactOnA =
          Vector3Add(p1, Vector3Scale(mtvAxis, minOverlap * 0.5f));
      *contactPoint_out = contactOnA;
    }
  }

  return true;
}

void ApplyImpulseCuboidRBNew(RigidBody *rbA, RigidBody *rbB, Vector3 mtv,
                             Vector3 contactPoint, Vector3 aDims,
                             Vector3 bDims) {
  Vector3 normal = Vector3Normalize(mtv);
  float penetration = Vector3Length(mtv);

  float totalInvMass = rbA->invMass + rbB->invMass;
  Matrix IinvAWorld = ComputeWorldInertia(
      Matrix3ToMatrix(rbA->invInertiaMatrix), rbA->rotation);

  Matrix IinvBWorld = ComputeWorldInertia(
      Matrix3ToMatrix(rbB->invInertiaMatrix), rbB->rotation);

  if (totalInvMass > 0.0f) {
    float correctionPercent = 0.2f; // How much penetration to correct (80%)
    float slop = 0.001f;            // Allow small penetration to avoid jitter
    float correctionMagnitude =
        fmaxf(penetration - slop, 0.0f) * correctionPercent / totalInvMass;

    Vector3 correction = Vector3Scale(normal, correctionMagnitude);

    if (rbA->invMass != 0.0f) {

      float ratioA = rbA->invMass / totalInvMass;
      rbA->position =
          Vector3Subtract(rbA->position, Vector3Scale(correction, ratioA));
    }
    if (rbB->invMass != 0.0f) {

      float ratioB = rbB->invMass / totalInvMass;
      rbB->position =
          Vector3Add(rbB->position, Vector3Scale(correction, ratioB));
    }
  }

  rbA->linearVelocity = Vector3Add(
      rbA->linearVelocity, Vector3Scale(mtv, rbA->invMass / totalInvMass));

  Vector3 rA = Vector3Subtract(contactPoint, rbA->position);
  Vector3 rACrossNb = Vector3CrossProduct(rA, normal);
  Vector3 angAddA =
      MultiplyMatrixVector3(StripMatrixToMatrix3(IinvAWorld), rACrossNb);
  printf("angaddmag %f\n", Vector3LengthSqr(angAddA));
  rbA->angularMomentum = Vector3Add(rbA->angularMomentum, angAddA);

  // same for B
  rbB->linearVelocity = Vector3Add(
      rbB->linearVelocity, Vector3Scale(mtv, rbB->invMass / totalInvMass));

  Vector3 rB = Vector3Subtract(contactPoint, rbB->position);
  Vector3 rBCrossNb = Vector3CrossProduct(rB, normal);
  Vector3 angAddB =
      MultiplyMatrixVector3(StripMatrixToMatrix3(IinvBWorld), rBCrossNb);
  rbB->angularMomentum = Vector3Add(rbB->angularMomentum, angAddB);
}

void ApplyImpulseCuboidRB(RigidBody *rbA, RigidBody *rbB, Vector3 mtv,
                          Vector3 contactPoint, Vector3 aDims, Vector3 bDims) {

  // === STEP 1: POSITIONAL CORRECTION (SEPARATION) ===
  Vector3 normal = Vector3Normalize(mtv);
  float penetration = Vector3Length(mtv);

  // Get inertia properties
  // Matrix3 Ia_inv = CuboidComputeInvInertiaMatrix(aDims, rbA->density);
  Matrix3 Ia_inv = rbA->invInertiaMatrix;

  // Matrix3 Ib_inv = CuboidComputeInvInertiaMatrix(bDims, rbB->density);
  Matrix3 Ib_inv = rbB->invInertiaMatrix;

  // Calculate correction based on mass ratio
  float totalInvMass = rbA->invMass + rbB->invMass;

  if (totalInvMass > 0.0f) {
    float correctionPercent = 1.0f; // How much penetration to correct (80%)
    float slop = 0.01f;             // Allow small penetration to avoid jitter
    float correctionMagnitude =
        fmaxf(penetration - slop, 0.0f) * correctionPercent / totalInvMass;

    Vector3 correction = Vector3Scale(normal, correctionMagnitude);

    float ratioA = rbA->invMass / totalInvMass;
    rbA->position =
        Vector3Subtract(rbA->position, Vector3Scale(correction, ratioA));
    float ratioB = rbB->invMass / totalInvMass;
    rbB->position = Vector3Add(rbB->position, Vector3Scale(correction, ratioB));
  }

  // === STEP 2: IMPULSE RESOLUTION ===

  // Calculate relative velocity at contact point
  Vector3 rA = Vector3Subtract(contactPoint,
                               rbA->position); // Contact point relative to A
  Vector3 rB = Vector3Subtract(contactPoint,
                               rbB->position); // Contact point relative to B
  Vector3 wA =
      ComputeAngularVelocity(Ia_inv, rbA->rotation, rbA->angularMomentum);
  Vector3 wB =
      ComputeAngularVelocity(Ib_inv, rbB->rotation, rbB->angularMomentum);

  // Velocity at contact point = linear velocity + (angular velocity × radius)
  Vector3 velA = Vector3Add(rbA->linearVelocity, Vector3CrossProduct(wA, rA));
  Vector3 velB = Vector3Add(rbB->linearVelocity, Vector3CrossProduct(wA, rB));

  Vector3 relativeVel = Vector3Subtract(velB, velA);
  float velAlongNormal = Vector3DotProduct(relativeVel, normal);

  // Don't resolve if objects are separating
  if (velAlongNormal > 0)
    return;

  // Calculate restitution (bounciness)
  float restitution = fminf(rbA->restitution, rbB->restitution);

  // Calculate impulse scalar
  // float impulseScalar = -(1.0f + restitution) * velAlongNormal;
  float impulseScalar = -(restitution)*velAlongNormal;

  // Add rotational effects to impulse calculation
  Vector3 rA_cross_n = Vector3CrossProduct(rA, normal);
  Vector3 rB_cross_n = Vector3CrossProduct(rB, normal);

  float rotationalEffect = 0.0f;
  Vector3 temp = Vector3CrossProduct(rA_cross_n, rA);
  temp = (Vector3){temp.x * Ia_inv.m0, temp.y * Ia_inv.m4, temp.z * Ia_inv.m8};
  rotationalEffect += Vector3DotProduct(temp, normal);

  Vector3 tempB = Vector3CrossProduct(rB_cross_n, rB);
  tempB = (Vector3){temp.x * Ib_inv.m0, temp.y * Ib_inv.m4, temp.z * Ib_inv.m8};
  rotationalEffect += Vector3DotProduct(tempB, normal);

  impulseScalar /= totalInvMass + rotationalEffect;
  Vector3 impulse = Vector3Scale(normal, impulseScalar);

  // Apply linear impulses
  rbA->linearVelocity =
      Vector3Subtract(rbA->linearVelocity, Vector3Scale(impulse, rbA->invMass));
  Vector3 angularImpulse =
      Vector3CrossProduct(rA, Vector3Scale(impulse, -1.0f));
  rbA->angularMomentum = Vector3Add(rbA->angularMomentum, angularImpulse);

  rbB->linearVelocity =
      Vector3Add(rbB->linearVelocity, Vector3Scale(impulse, rbB->invMass));
  Vector3 angularImpulseB = Vector3CrossProduct(rB, impulse);
  rbB->angularMomentum = Vector3Add(rbB->angularMomentum, angularImpulseB);

  // === STEP 3: FRICTION ===

  // Recalculate relative velocity after normal impulse

  wA = ComputeAngularVelocity(rbA->invInertiaMatrix, rbA->rotation,
                              rbA->angularMomentum);

  wB = ComputeAngularVelocity(rbB->invInertiaMatrix, rbB->rotation,
                              rbB->angularMomentum);
  velA = Vector3Add(rbA->linearVelocity, Vector3CrossProduct(wA, rA));
  velB = Vector3Add(rbB->linearVelocity, Vector3CrossProduct(wB, rB));

  relativeVel = Vector3Subtract(velB, velA);

  // Calculate tangent vector (friction direction)
  Vector3 tangent = Vector3Subtract(
      relativeVel,
      Vector3Scale(normal, Vector3DotProduct(relativeVel, normal)));
  if (Vector3Length(tangent) > 1e-6f) {
    tangent = Vector3Normalize(tangent);

    float friction = sqrtf(rbA->friction * rbB->friction); // Geometric mean
    float frictionImpulse = -Vector3DotProduct(relativeVel, tangent);

    // Coulomb friction model
    if (fabsf(frictionImpulse) < impulseScalar * friction) {
      // Static friction
      frictionImpulse = frictionImpulse;
    } else {
      // Kinetic friction
      frictionImpulse =
          -impulseScalar * friction * (frictionImpulse > 0 ? 1.0f : -1.0f);
    }

    Vector3 frictionVector = Vector3Scale(tangent, frictionImpulse);

    // Apply friction impulses
    rbA->linearVelocity = Vector3Subtract(
        rbA->linearVelocity, Vector3Scale(frictionVector, rbA->invMass));
    Vector3 angularFriction =
        Vector3CrossProduct(rA, Vector3Scale(frictionVector, -1.0f));
    rbA->angularMomentum = Vector3Add(rbA->angularMomentum, angularFriction);

    rbB->linearVelocity = Vector3Add(
        rbB->linearVelocity, Vector3Scale(frictionVector, rbB->invMass));
    Vector3 angularFrictionB = Vector3CrossProduct(rB, frictionVector);
    rbB->angularMomentum = Vector3Add(rbB->angularMomentum, angularFrictionB);
  }

  // Optional: Debug output
  printf("Collision: %d, Impulse: %.2f, Contact: (%.2f,%.2f,%.2f)\n",
         collisionCount, impulseScalar, contactPoint.x, contactPoint.y,
         contactPoint.z);
}

void ApplyImpulseCuboidRBSB(RigidBody *rbA, RigidBody *rbB, Vector3 mtv,
                            Vector3 contactPoint, Vector3 aDims,
                            Vector3 bDims) {

  // === STEP 1: POSITIONAL CORRECTION (SEPARATION) ===
  Vector3 normal = Vector3Normalize(mtv);
  float penetration = Vector3Length(mtv);

  // Get inertia properties
  // Matrix3 Ia_inv = CuboidComputeInvInertiaMatrix(aDims, rbA->density);
  Matrix3 Ia_inv = rbA->invInertiaMatrix;

  // Calculate correction based on mass ratio
  float totalInvMass = rbA->invMass;

  if (totalInvMass > 0.0f) {
    float correctionPercent = 1.0f; // How much penetration to correct (80%)
    float slop = 0.001f;            // Allow small penetration to avoid jitter
    float correctionMagnitude =
        fmaxf(penetration - slop, 0.0f) * correctionPercent / totalInvMass;

    Vector3 correction = Vector3Scale(normal, correctionMagnitude);

    float ratioA = rbA->invMass / totalInvMass;
    rbA->position =
        Vector3Subtract(rbA->position, Vector3Scale(correction, ratioA));
  }

  // === STEP 2: IMPULSE RESOLUTION ===

  // Calculate relative velocity at contact point
  Vector3 rA = Vector3Subtract(contactPoint,
                               rbA->position); // Contact point relative to A
  Vector3 rB = Vector3Subtract(contactPoint,
                               rbB->position); // Contact point relative to B
  Vector3 wA =
      ComputeAngularVelocity(Ia_inv, rbA->rotation, rbA->angularMomentum);

  // Velocity at contact point = linear velocity + (angular velocity × radius)
  Vector3 velA = Vector3Add(rbA->linearVelocity, Vector3CrossProduct(wA, rA));

  Vector3 relativeVel = Vector3Subtract(Vector3Zero(), velA);
  float velAlongNormal = Vector3DotProduct(relativeVel, normal);

  // Don't resolve if objects are separating
  if (velAlongNormal > 0)
    return;

  // Calculate restitution (bounciness)
  float restitution = fminf(rbA->restitution, rbB->restitution);

  // Clamp restitution on resting contact
  if (velAlongNormal > -0.5f)
    restitution = 0.0f;

  // Calculate impulse scalar
  // float impulseScalar = -(1.0f + restitution)*velAlongNormal;
  float impulseScalar = -(restitution)*velAlongNormal;

  // Add rotational effects to impulse calculation
  Vector3 rA_cross_n = Vector3CrossProduct(rA, normal);
  Vector3 rB_cross_n = Vector3CrossProduct(rB, normal);

  float rotationalEffect = 0.0f;
  Vector3 temp = Vector3CrossProduct(rA_cross_n, rA);
  temp = (Vector3){temp.x * Ia_inv.m0, temp.y * Ia_inv.m4, temp.z * Ia_inv.m8};
  rotationalEffect += Vector3DotProduct(temp, normal);

  impulseScalar /= totalInvMass + rotationalEffect;
  Vector3 impulse = Vector3Scale(normal, impulseScalar);

  // Apply linear impulses
  rbA->linearVelocity =
      Vector3Subtract(rbA->linearVelocity, Vector3Scale(impulse, rbA->invMass));
  Vector3 angularImpulse =
      Vector3CrossProduct(rA, Vector3Scale(impulse, -1.0f));
  rbA->angularMomentum = Vector3Add(rbA->angularMomentum, angularImpulse);

  // === STEP 3: FRICTION ===

  // Recalculate relative velocity after normal impulse

  wA = ComputeAngularVelocity(rbA->invInertiaMatrix, rbA->rotation,
                              rbA->angularMomentum);

  velA = Vector3Add(rbA->linearVelocity, Vector3CrossProduct(wA, rA));

  relativeVel = Vector3Subtract(Vector3Zero(), velA);

  // Calculate tangent vector (friction direction)
  Vector3 tangent = Vector3Subtract(
      relativeVel,
      Vector3Scale(normal, Vector3DotProduct(relativeVel, normal)));
  if (Vector3Length(tangent) > 1e-6f) {
    tangent = Vector3Normalize(tangent);

    float friction = sqrtf(rbA->friction * rbB->friction); // Geometric mean
    float frictionImpulse = -Vector3DotProduct(relativeVel, tangent);

    // Coulomb friction model
    if (fabsf(frictionImpulse) < impulseScalar * friction) {
      // Static friction
      frictionImpulse = frictionImpulse;
    } else {
      // Kinetic friction
      frictionImpulse =
          -impulseScalar * friction * (frictionImpulse > 0 ? 1.0f : -1.0f);
    }

    Vector3 frictionVector = Vector3Scale(tangent, frictionImpulse);

    // Apply friction impulses
    rbA->linearVelocity = Vector3Subtract(
        rbA->linearVelocity, Vector3Scale(frictionVector, rbA->invMass));
    Vector3 angularFriction =
        Vector3CrossProduct(rA, Vector3Scale(frictionVector, -1.0f));
    rbA->angularMomentum = Vector3Add(rbA->angularMomentum, angularFriction);
  }

  // Optional: Debug output
  printf("Collision: %d, ImpulseScalar: %.2f, Contact: (%.2f,%.2f,%.2f), "
         "AngImpulseMagSq: %.2f\n",
         collisionCount, impulseScalar, contactPoint.x, contactPoint.y,
         contactPoint.z, Vector3LengthSqr(angularImpulse));
}

// void ApplyImpulse(RigidBody *a, RigidBody *b, Vector3 contactPoint,
//                   Vector3 normal, float penetrationDepth) {
//   float restitution = (a->restitution + b->restitution) * 0.5f;
//   float friction = (a->friction + b->friction) * 0.5f;
//
//   Vector3 ra = Vector3Subtract(contactPoint, a->position);
//   Vector3 rb = Vector3Subtract(contactPoint, b->position);
//
//   // Recompute angular linearVelocity from angular momentum
//   Matrix3 Ia_inv;
//   Vector3 ACoM;
//   float Amass;
//   MeshComputeInertiaMatrix(a->mesh, a->density, &Ia_inv, &ACoM, &Amass);
//
//   Matrix3 Ib_inv;
//   Vector3 BCoM;
//   float Bmass;
//   MeshComputeInertiaMatrix(b->mesh, b->density, &Ib_inv, &BCoM, &Bmass);
//
//   Vector3 wa = ComputeAngularVelocity(Ia_inv, a->rotation,
//   a->angularMomentum); Vector3 wb = ComputeAngularVelocity(Ib_inv,
//   b->rotation, b->angularMomentum);
//
//   // Compute linearVelocity at contact point
//   Vector3 va = Vector3Add(a->linearVelocity, Vector3CrossProduct(wa, ra));
//   Vector3 vb = Vector3Add(b->linearVelocity, Vector3CrossProduct(wb, rb));
//   Vector3 relativelinearVelocity = Vector3Subtract(vb, va);
//
//   float contactVel = Vector3DotProduct(relativelinearVelocity, normal);
//
//   if (contactVel > 0.0f)
//     return; // No impulse needed if separating
//
//   // Calculate impulse scalar
//   Vector3 raCrossN = Vector3CrossProduct(ra, normal);
//   Vector3 rbCrossN = Vector3CrossProduct(rb, normal);
//   Vector3 Ia_raCrossN = Vector3Transform(raCrossN, Matrix3ToMatrix(Ia_inv));
//   Vector3 Ib_rbCrossN = Vector3Transform(rbCrossN, Matrix3ToMatrix(Ib_inv));
//
//   float denom =
//       (1.0f / Amass) + (1.0f / Bmass) +
//       Vector3DotProduct(Vector3CrossProduct(Ia_raCrossN, ra), normal) +
//       Vector3DotProduct(Vector3CrossProduct(Ib_rbCrossN, rb), normal);
//
//   float j = -(1.0f + restitution) * contactVel / denom;
//   // NOTE j*0.5 is my own thing, normally just j
//   Vector3 impulse = Vector3Scale(normal, j * 0.5f);
//
//   // Apply linear impulse
//   a->linearVelocity =
//       Vector3Subtract(a->linearVelocity, Vector3Scale(impulse, 1.0f /
//       Amass));
//   b->linearVelocity =
//       Vector3Add(b->linearVelocity, Vector3Scale(impulse, 1.0f / Bmass));
//
//   // Apply angular impulse to angular momentum
//   a->angularMomentum =
//       Vector3Subtract(a->angularMomentum, Vector3CrossProduct(ra, impulse));
//   b->angularMomentum =
//       Vector3Add(b->angularMomentum, Vector3CrossProduct(rb, impulse));
//
//   // --- Friction impulse ---
//   Vector3 tangent = Vector3Subtract(
//       relativelinearVelocity,
//       Vector3Scale(normal, Vector3DotProduct(relativelinearVelocity,
//       normal)));
//   if (Vector3LengthSqr(tangent) > EPSILON) {
//     tangent = Vector3Normalize(tangent);
//     float jt = -Vector3DotProduct(relativelinearVelocity, tangent) / denom;
//     jt = Clamp(jt, -j * friction, j * friction);
//
//     Vector3 frictionImpulse = Vector3Scale(tangent, jt);
//
//     a->linearVelocity = Vector3Subtract(
//         a->linearVelocity, Vector3Scale(frictionImpulse, 1.0f / Amass));
//     b->linearVelocity = Vector3Add(b->linearVelocity,
//                                    Vector3Scale(frictionImpulse, 1.0f /
//                                    Bmass));
//
//     a->angularMomentum = Vector3Subtract(
//         a->angularMomentum, Vector3CrossProduct(ra, frictionImpulse));
//     b->angularMomentum = Vector3Add(b->angularMomentum,
//                                     Vector3CrossProduct(rb,
//                                     frictionImpulse));
//   }
// }
//
void ClaudeResolveCollision(RigidBody *rbA, RigidBody *rbB, Vector3 mtv,
                            Vector3 contactPoint) {

  // Skip if both bodies are static
  if (rbA->invMass == 0.0f && rbB->invMass == 0.0f)
    return;

  // === STEP 1: POSITIONAL CORRECTION (SEPARATION) ===
  Vector3 normal = Vector3Normalize(mtv);
  float penetration = Vector3Length(mtv);

  // Calculate correction based on mass ratio
  float totalInvMass = rbA->invMass + rbB->invMass;

  if (totalInvMass > 0.0f) {
    float correctionPercent = 0.8f; // How much penetration to correct (80%)
    float slop = 0.01f;             // Allow small penetration to avoid jitter
    float correctionMagnitude =
        fmaxf(penetration - slop, 0.0f) * correctionPercent / totalInvMass;

    Vector3 correction = Vector3Scale(normal, correctionMagnitude);

    if (rbA->invMass > 0.0f) {
      float ratioA = rbA->invMass / totalInvMass;
      rbA->position =
          Vector3Subtract(rbA->position, Vector3Scale(correction, ratioA));
    }
    if (rbB->invMass > 0.0f) {
      float ratioB = rbB->invMass / totalInvMass;
      rbB->position =
          Vector3Add(rbB->position, Vector3Scale(correction, ratioB));
    }
  }

  // === STEP 2: IMPULSE RESOLUTION ===

  // Calculate relative velocity at contact point
  Vector3 rA = Vector3Subtract(contactPoint, rbA->position);
  Vector3 rB = Vector3Subtract(contactPoint, rbB->position);

  // Convert angular momentum to angular velocity for velocity calculations
  Vector3 angVelA =
      (rbA->invMass == 0.0f)
          ? Vector3Zero()
          : ComputeAngularVelocity(rbA->invInertiaMatrix, rbA->rotation,
                                   rbA->angularMomentum);
  Vector3 angVelB =
      (rbB->invMass == 0.0f)
          ? Vector3Zero()
          : ComputeAngularVelocity(rbB->invInertiaMatrix, rbB->rotation,
                                   rbB->angularMomentum);

  // Velocity at contact point = linear velocity + (angular velocity × radius)
  Vector3 velA =
      (rbA->invMass == 0.0f)
          ? Vector3Zero()
          : Vector3Add(rbA->linearVelocity, Vector3CrossProduct(angVelA, rA));
  Vector3 velB =
      (rbB->invMass == 0.0f)
          ? Vector3Zero()
          : Vector3Add(rbB->linearVelocity, Vector3CrossProduct(angVelB, rB));

  Vector3 relativeVel = Vector3Subtract(velB, velA);
  float velAlongNormal = Vector3DotProduct(relativeVel, normal);

  // Don't resolve if objects are separating
  if (velAlongNormal > 0)
    return;

  // Calculate restitution (bounciness)
  float restitution = fminf(rbA->restitution, rbB->restitution);

  // Calculate impulse scalar with proper rotational inertia handling
  float impulseScalar = -(1.0f + restitution) * velAlongNormal;

  // Compute world-space inertia tensors for rotational effects
  Matrix IinvAWorld =
      (rbA->invMass == 0.0f)
          ? MatrixIdentity()
          : ComputeWorldInertia(Matrix3ToMatrix(rbA->invInertiaMatrix),
                                rbA->rotation);
  Matrix IinvBWorld =
      (rbB->invMass == 0.0f)
          ? MatrixIdentity()
          : ComputeWorldInertia(Matrix3ToMatrix(rbB->invInertiaMatrix),
                                rbB->rotation);

  // Calculate rotational contribution to effective mass
  float rotationalEffect = 0.0f;
  if (rbA->invMass > 0.0f) {
    Vector3 rA_cross_n = Vector3CrossProduct(rA, normal);
    Vector3 temp = Vector3Transform(rA_cross_n, IinvAWorld);
    rotationalEffect +=
        Vector3DotProduct(Vector3CrossProduct(temp, rA), normal);
  }
  if (rbB->invMass > 0.0f) {
    Vector3 rB_cross_n = Vector3CrossProduct(rB, normal);
    Vector3 temp = Vector3Transform(rB_cross_n, IinvBWorld);
    rotationalEffect +=
        Vector3DotProduct(Vector3CrossProduct(temp, rB), normal);
  }

  impulseScalar /= totalInvMass + rotationalEffect;
  Vector3 impulse = Vector3Scale(normal, impulseScalar);

  // Apply linear impulses
  if (rbA->invMass > 0.0f) {
    rbA->linearVelocity = Vector3Subtract(rbA->linearVelocity,
                                          Vector3Scale(impulse, rbA->invMass));
    // Apply angular impulse to angular momentum
    Vector3 angularImpulse =
        Vector3CrossProduct(rA, Vector3Scale(impulse, -1.0f));
    rbA->angularMomentum = Vector3Add(rbA->angularMomentum, angularImpulse);
  }
  if (rbB->invMass > 0.0f) {
    rbB->linearVelocity =
        Vector3Add(rbB->linearVelocity, Vector3Scale(impulse, rbB->invMass));
    // Apply angular impulse to angular momentum
    Vector3 angularImpulse = Vector3CrossProduct(rB, impulse);
    rbB->angularMomentum = Vector3Add(rbB->angularMomentum, angularImpulse);
  }

  // === STEP 3: FRICTION ===

  // Recalculate angular velocities and contact velocities after normal impulse
  angVelA = (rbA->invMass == 0.0f)
                ? Vector3Zero()
                : ComputeAngularVelocity(rbA->invInertiaMatrix, rbA->rotation,
                                         rbA->angularMomentum);
  angVelB = (rbB->invMass == 0.0f)
                ? Vector3Zero()
                : ComputeAngularVelocity(rbB->invInertiaMatrix, rbB->rotation,
                                         rbB->angularMomentum);

  velA = (rbA->invMass == 0.0f) ? Vector3Zero()
                                : Vector3Add(rbA->linearVelocity,
                                             Vector3CrossProduct(angVelA, rA));
  velB = (rbB->invMass == 0.0f) ? Vector3Zero()
                                : Vector3Add(rbB->linearVelocity,
                                             Vector3CrossProduct(angVelB, rB));

  relativeVel = Vector3Subtract(velB, velA);

  // Calculate tangent vector (friction direction)
  Vector3 tangent = Vector3Subtract(
      relativeVel,
      Vector3Scale(normal, Vector3DotProduct(relativeVel, normal)));

  if (Vector3Length(tangent) > 1e-6f) {
    tangent = Vector3Normalize(tangent);

    float friction = sqrtf(rbA->friction * rbB->friction);

    // Calculate friction impulse with rotational effects
    float frictionImpulse = -Vector3DotProduct(relativeVel, tangent);

    // Calculate rotational contribution to friction
    float frictionRotationalEffect = 0.0f;
    if (rbA->invMass > 0.0f) {
      Vector3 rA_cross_t = Vector3CrossProduct(rA, tangent);
      Vector3 temp = Vector3Transform(rA_cross_t, IinvAWorld);
      frictionRotationalEffect +=
          Vector3DotProduct(Vector3CrossProduct(temp, rA), tangent);
    }
    if (rbB->invMass > 0.0f) {
      Vector3 rB_cross_t = Vector3CrossProduct(rB, tangent);
      Vector3 temp = Vector3Transform(rB_cross_t, IinvBWorld);
      frictionRotationalEffect +=
          Vector3DotProduct(Vector3CrossProduct(temp, rB), tangent);
    }

    frictionImpulse /= totalInvMass + frictionRotationalEffect;

    // Coulomb friction model
    if (fabsf(frictionImpulse) > fabsf(impulseScalar) * friction) {
      // Kinetic friction
      frictionImpulse = -fabsf(impulseScalar) * friction *
                        (frictionImpulse > 0 ? 1.0f : -1.0f);
    }

    Vector3 frictionVector = Vector3Scale(tangent, frictionImpulse);

    // Apply friction impulses
    if (rbA->invMass > 0.0f) {
      rbA->linearVelocity = Vector3Subtract(
          rbA->linearVelocity, Vector3Scale(frictionVector, rbA->invMass));
      Vector3 angularFriction =
          Vector3CrossProduct(rA, Vector3Scale(frictionVector, -1.0f));
      rbA->angularMomentum = Vector3Add(rbA->angularMomentum, angularFriction);
    }
    if (rbB->invMass > 0.0f) {
      rbB->linearVelocity = Vector3Add(
          rbB->linearVelocity, Vector3Scale(frictionVector, rbB->invMass));
      Vector3 angularFriction = Vector3CrossProduct(rB, frictionVector);
      rbB->angularMomentum = Vector3Add(rbB->angularMomentum, angularFriction);
    }
  }
}

typedef enum CollisionType { EdgeEdge = 0, VertexFace, FaceFace } CollisionType;

void HandleCuboidRBCollisions(RigidBody *a, RigidBody *b, Vector3 aDims,
                              Vector3 bDims) {

  Vector3 mtv;
  Vector3 contactPoint;
  if (OBBvsOBB(a->position, a->rotation, aDims, b->position, b->rotation, bDims,
               &mtv, NULL)) {
    collisionCount++;
    // check what type of collision it is:

    // printf("COLLISION FOUND, COUNT: %d\n", collisionCount);
    float penetration = Vector3Length(mtv);
    Vector3 normal = Vector3Normalize(mtv);

    const float slop = 0.0001f;
    if (penetration > slop) {
      PushDebugContact(contactPoint, normal, penetration);
      printf("contactPoint: (%f, %f, %f)\n", contactPoint.x, contactPoint.y,
             contactPoint.z);
      ClaudeResolveCollision(a, b, mtv, contactPoint);
    }
  }
}
