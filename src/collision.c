#include "collision.h"
#include "cuboid_rb.h"
#include "rigidbody.h"
#include <float.h>
#include <raylib.h>
#include <raymath.h>

#define MAX_DEBUG_CONTACTS 128
static DebugContact debugContacts[MAX_DEBUG_CONTACTS];
static int debugContactCount = 0;

typedef struct {
  Vector3 position;
  Quaternion orientation;
  Vector3 halfExtents;
} OBB;
OBB CuboidToObb(Vector3 pos, Quaternion rot, Vector3 dims) {
  return (OBB){
      .position = pos,
      .orientation = rot,
      .halfExtents = Vector3Scale(dims, 0.5f),
  };
}

typedef struct {
  bool isColliding;
  Vector3 location;
  Vector3 mtv;
  float penetrationDepth;
  Vector3 normal;
} CollisionInfo;

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

// source:
// https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/physicstutorials/5collisionresponse/Physics%20-%20Collision%20Response.pdf
void ResolveCollision(RigidBody *a, RigidBody *b, CollisionInfo info) {
  float totalMass = a->invMass + b->invMass;
  if (totalMass == 0.0f) {
    return;
  }
  // Seperate positions
  a->position = Vector3Subtract(
      a->position, Vector3Scale(info.normal, info.penetrationDepth *
                                                 (a->invMass / totalMass)));

  b->position = Vector3Add(
      b->position, Vector3Scale(info.normal, info.penetrationDepth *
                                                 (b->invMass / totalMass)));
  Vector3 relativeA = Vector3Subtract(info.location, a->position);
  Vector3 relativeB = Vector3Subtract(info.location, b->position);
  // need to compute these, angVelA and angVelB are intermediaries
  Vector3 angVelA = ComputeAngularVelocity(a->invInertiaMatrix, a->rotation,
                                           a->angularMomentum);
  Vector3 angVelB = ComputeAngularVelocity(b->invInertiaMatrix, b->rotation,
                                           b->angularMomentum);
  Vector3 angVelocityA = Vector3CrossProduct(angVelA, relativeA);
  Vector3 angVelocityB = Vector3CrossProduct(angVelB, relativeB);

  Vector3 fullVelocityA = Vector3Add(a->linearVelocity, angVelocityA);
  Vector3 fullVelocityB = Vector3Add(b->linearVelocity, angVelocityB);

  Vector3 contactVelocity = Vector3Subtract(fullVelocityB, fullVelocityA);
  float impulseForce = Vector3DotProduct(contactVelocity, info.normal);

  // intermediaries
  Matrix3 Ainv = StripMatrixToMatrix3(
      ComputeWorldInertia(Matrix3ToMatrix(a->invInertiaMatrix), a->rotation));
  Matrix3 Binv = StripMatrixToMatrix3(
      ComputeWorldInertia(Matrix3ToMatrix(b->invInertiaMatrix), b->rotation));

  Vector3 inertiaA = Vector3CrossProduct(
      MultiplyMatrixVector3(InverseMatrix3(Ainv),
                            Vector3CrossProduct(relativeA, info.normal)),
      relativeA);
  Vector3 inertiaB = Vector3CrossProduct(
      MultiplyMatrixVector3(InverseMatrix3(Binv),
                            Vector3CrossProduct(relativeB, info.normal)),
      relativeB);

  float angularEffect =
      Vector3DotProduct(Vector3Add(inertiaA, inertiaB), info.normal) * 0.5;
  // float cRestitution = (a->restitution + b->restitution) * 0.5f;
  float cRestitution = a->restitution * b->restitution;
  // cRestitution = 0.66f;

  float j =
      (-(1.0f + cRestitution) * impulseForce) / (totalMass + angularEffect);

  Vector3 fullImpulse = Vector3Scale(info.normal, j);
  a->linearVelocity = Vector3Add(
      a->linearVelocity, Vector3Scale(Vector3Negate(fullImpulse), a->invMass));

  b->linearVelocity =
      Vector3Add(b->linearVelocity, Vector3Scale(fullImpulse, b->invMass));

  a->angularMomentum =
      Vector3Add(a->angularMomentum,
                 Vector3CrossProduct(relativeA, Vector3Negate(fullImpulse)));

  b->angularMomentum = Vector3Add(b->angularMomentum,
                                  Vector3CrossProduct(relativeB, fullImpulse));
}

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

// Helper function to transform a point by inverse of orientation
Vector3 InverseTransformPoint(Vector3 point, Quaternion orientation,
                              Vector3 position) {
  Quaternion invOrientation = QuaternionInvert(orientation);
  Vector3 translated = Vector3Subtract(point, position);
  return Vector3RotateByQuaternion(translated, invOrientation);
}

// Helper function to get the 8 vertices of an OBB
void GetOBBVertices(OBB obb, Vector3 vertices[8]) {
  Matrix transform = QuaternionToMatrix(obb.orientation);

  Vector3 extents[8] = {
      {obb.halfExtents.x, obb.halfExtents.y, obb.halfExtents.z},
      {obb.halfExtents.x, obb.halfExtents.y, -obb.halfExtents.z},
      {obb.halfExtents.x, -obb.halfExtents.y, obb.halfExtents.z},
      {obb.halfExtents.x, -obb.halfExtents.y, -obb.halfExtents.z},
      {-obb.halfExtents.x, obb.halfExtents.y, obb.halfExtents.z},
      {-obb.halfExtents.x, obb.halfExtents.y, -obb.halfExtents.z},
      {-obb.halfExtents.x, -obb.halfExtents.y, obb.halfExtents.z},
      {-obb.halfExtents.x, -obb.halfExtents.y, -obb.halfExtents.z}};

  for (int i = 0; i < 8; i++) {
    Vector3 transformed = Vector3Transform(extents[i], transform);
    vertices[i] = Vector3Add(transformed, obb.position);
  }
}

// Helper function to get the 15 potential separating axes for SAT
void GetSeparatingAxes(OBB a, OBB b, Vector3 axes[15]) {
  Matrix aRot = QuaternionToMatrix(a.orientation);
  Matrix bRot = QuaternionToMatrix(b.orientation);

  // Get the local axes for each OBB
  Vector3 aAxes[3] = {
      {aRot.m0, aRot.m1, aRot.m2}, // Right
      {aRot.m4, aRot.m5, aRot.m6}, // Up
      {aRot.m8, aRot.m9, aRot.m10} // Forward
  };

  Vector3 bAxes[3] = {
      {bRot.m0, bRot.m1, bRot.m2}, // Right
      {bRot.m4, bRot.m5, bRot.m6}, // Up
      {bRot.m8, bRot.m9, bRot.m10} // Forward
  };

  // Face axes from box A (3)
  axes[0] = aAxes[0];
  axes[1] = aAxes[1];
  axes[2] = aAxes[2];

  // Face axes from box B (3)
  axes[3] = bAxes[0];
  axes[4] = bAxes[1];
  axes[5] = bAxes[2];

  // Edge-edge cross products (9)
  axes[6] = Vector3CrossProduct(aAxes[0], bAxes[0]);
  axes[7] = Vector3CrossProduct(aAxes[0], bAxes[1]);
  axes[8] = Vector3CrossProduct(aAxes[0], bAxes[2]);
  axes[9] = Vector3CrossProduct(aAxes[1], bAxes[0]);
  axes[10] = Vector3CrossProduct(aAxes[1], bAxes[1]);
  axes[11] = Vector3CrossProduct(aAxes[1], bAxes[2]);
  axes[12] = Vector3CrossProduct(aAxes[2], bAxes[0]);
  axes[13] = Vector3CrossProduct(aAxes[2], bAxes[1]);
  axes[14] = Vector3CrossProduct(aAxes[2], bAxes[2]);

  // Normalize all axes
  for (int i = 0; i < 15; i++) {
    // Skip zero vectors (happens when axes are parallel)
    if (Vector3Length(axes[i]) < FLT_EPSILON) {
      axes[i] = Vector3Zero();
      continue;
    }
    axes[i] = Vector3Normalize(axes[i]);
  }
}

// Project OBB onto axis and get min and max projection values
void ProjectOBBNew(OBB obb, Vector3 axis, float *min, float *max) {
  Matrix transform = QuaternionToMatrix(obb.orientation);

  Vector3 extents = obb.halfExtents;
  Vector3 vertices[8] = {{extents.x, extents.y, extents.z},
                         {extents.x, extents.y, -extents.z},
                         {extents.x, -extents.y, extents.z},
                         {extents.x, -extents.y, -extents.z},
                         {-extents.x, extents.y, extents.z},
                         {-extents.x, extents.y, -extents.z},
                         {-extents.x, -extents.y, extents.z},
                         {-extents.x, -extents.y, -extents.z}};

  *min = FLT_MAX;
  *max = -FLT_MAX;

  for (int i = 0; i < 8; i++) {
    Vector3 vertex = Vector3Transform(vertices[i], transform);
    vertex = Vector3Add(vertex, obb.position);

    float projection = Vector3DotProduct(vertex, axis);
    if (projection < *min)
      *min = projection;
    if (projection > *max)
      *max = projection;
  }
}

// Check for collision between two OBBs using SAT
CollisionInfo CheckOBBCollision(OBB a, OBB b) {
  CollisionInfo result = {0};
  result.isColliding = false;

  Vector3 axes[15];
  GetSeparatingAxes(a, b, axes);

  float minPenetration = FLT_MAX;
  Vector3 mtvAxis = Vector3Zero();

  for (int i = 0; i < 15; i++) {
    Vector3 axis = axes[i];

    // Skip zero axes (from parallel edge cross products)
    if (Vector3Length(axis) < FLT_EPSILON)
      continue;

    // Project both OBBs onto the axis
    float aMin, aMax, bMin, bMax;
    ProjectOBBNew(a, axis, &aMin, &aMax);
    ProjectOBBNew(b, axis, &bMin, &bMax);

    // Check for overlap
    if (aMax < bMin || bMax < aMin) {
      // No overlap on this axis - no collision
      return result;
    }

    // Calculate penetration depth
    float penetration = fminf(aMax - bMin, bMax - aMin);

    // Track the axis with minimal penetration
    if (penetration < minPenetration) {
      minPenetration = penetration;
      mtvAxis = axis;
    }
  }

  // If we get here, all axes had overlap - collision occurred
  result.isColliding = true;
  result.penetrationDepth = minPenetration;

  // Ensure MTV points from A to B
  Vector3 centerA = a.position;
  Vector3 centerB = b.position;
  Vector3 centerDiff = Vector3Subtract(centerB, centerA);

  if (Vector3DotProduct(centerDiff, mtvAxis) < 0) {
    mtvAxis = Vector3Negate(mtvAxis);
  }

  result.normal = mtvAxis;
  result.mtv = Vector3Scale(mtvAxis, minPenetration);

  // Calculate approximate collision location (center of overlapping region)
  Vector3 aVertices[8], bVertices[8];
  GetOBBVertices(a, aVertices);
  GetOBBVertices(b, bVertices);

  // Find all vertices of A that are inside B and vice versa
  Vector3 insidePoints[16]; // Max possible is 8+8
  int pointCount = 0;

  for (int i = 0; i < 8; i++) {
    Vector3 localPoint =
        InverseTransformPoint(aVertices[i], b.orientation, b.position);
    if (fabsf(localPoint.x) <= b.halfExtents.x &&
        fabsf(localPoint.y) <= b.halfExtents.y &&
        fabsf(localPoint.z) <= b.halfExtents.z) {
      insidePoints[pointCount++] = aVertices[i];
    }
  }

  for (int i = 0; i < 8; i++) {
    Vector3 localPoint =
        InverseTransformPoint(bVertices[i], a.orientation, a.position);
    if (fabsf(localPoint.x) <= a.halfExtents.x &&
        fabsf(localPoint.y) <= a.halfExtents.y &&
        fabsf(localPoint.z) <= a.halfExtents.z) {
      insidePoints[pointCount++] = bVertices[i];
    }
  }

  // Calculate average of all inside points
  if (pointCount > 0) {
    Vector3 sum = Vector3Zero();
    for (int i = 0; i < pointCount; i++) {
      sum = Vector3Add(sum, insidePoints[i]);
    }
    result.location = Vector3Scale(sum, 1.0f / pointCount);
  } else {
    // Fallback if no points are inside (shouldn't happen with SAT)
    result.location = Vector3Lerp(a.position, b.position, 0.5f);
  }

  return result;
}

void HandleCuboidRBCollisions(RigidBody *a, RigidBody *b, Vector3 aDims,
                              Vector3 bDims) {

  // Vector3 mtv;
  // Vector3 contactPoint;
  OBB obbA = CuboidToObb(a->position, a->rotation, aDims);
  OBB obbB = CuboidToObb(b->position, b->rotation, bDims);
  CollisionInfo info = CheckOBBCollision(obbA, obbB);
  // info.normal = Vector3Scale(info.normal, -1.0f);
  if (info.isColliding) {

    const float slop = 0.0001f;
    if (info.penetrationDepth > slop) {
      PushDebugContact(info.location, info.normal, info.penetrationDepth);
      ResolveCollision(a, b, info);
      // ClaudeResolveCollision(a, b, info.mtv, info.location);
    }
  }
}
