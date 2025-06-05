#include "collision.h"
#include <float.h>
#include <raylib.h>
#include <raymath.h>

// TODO: double check the code since this is all from chatgpt
static float ProjectBox(Vector3 axis, Vector3 center, Vector3 axes[3],
                        Vector3 halfSize) {
  return fabsf(Vector3DotProduct(axis, axes[0]) * halfSize.x) +
         fabsf(Vector3DotProduct(axis, axes[1]) * halfSize.y) +
         fabsf(Vector3DotProduct(axis, axes[2]) * halfSize.z);
}

// Collision test with MTV
bool OBBvsOBB(Vector3 p1, Quaternion q1, Vector3 s1, Vector3 p2, Quaternion q2,
              Vector3 s2, Vector3 *mtv_out) {
  Matrix R1 = QuaternionToMatrix(q1);
  Matrix R2 = QuaternionToMatrix(q2);

  Vector3 A[3] = {(Vector3){R1.m0, R1.m4, R1.m8},
                  (Vector3){R1.m1, R1.m5, R1.m9},
                  (Vector3){R1.m2, R1.m6, R1.m10}};
  Vector3 B[3] = {(Vector3){R2.m0, R2.m4, R2.m8},
                  (Vector3){R2.m1, R2.m5, R2.m9},
                  (Vector3){R2.m2, R2.m6, R2.m10}};

  Vector3 D = Vector3Subtract(p2, p1);
  float minOverlap = FLT_MAX;
  Vector3 smallestAxis = {0};

  Vector3 half1 = Vector3Scale(s1, 0.5f);
  Vector3 half2 = Vector3Scale(s2, 0.5f);

  for (int i = 0; i < 3; i++) {
    Vector3 axis = Vector3Normalize(A[i]);
    float r1 = ProjectBox(axis, p1, A, half1);
    float r2 = ProjectBox(axis, p2, B, half2);
    float dist = fabsf(Vector3DotProduct(D, axis));
    float overlap = r1 + r2 - dist;
    if (overlap < 0)
      return false;
    if (overlap < minOverlap) {
      minOverlap = overlap;
      smallestAxis = axis;
    }
  }

  for (int i = 0; i < 3; i++) {
    Vector3 axis = Vector3Normalize(B[i]);
    float r1 = ProjectBox(axis, p1, A, half1);
    float r2 = ProjectBox(axis, p2, B, half2);
    float dist = fabsf(Vector3DotProduct(D, axis));
    float overlap = r1 + r2 - dist;
    if (overlap < 0)
      return false;
    if (overlap < minOverlap) {
      minOverlap = overlap;
      smallestAxis = axis;
    }
  }

  // Cross product axes
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Vector3 axis = Vector3CrossProduct(A[i], B[j]);
      if (Vector3LengthSqr(axis) < EPSILON)
        continue;
      axis = Vector3Normalize(axis);
      float r1 = ProjectBox(axis, p1, A, half1);
      float r2 = ProjectBox(axis, p2, B, half2);
      float dist = fabsf(Vector3DotProduct(D, axis));
      float overlap = r1 + r2 - dist;
      if (overlap < 0)
        return false;
      if (overlap < minOverlap) {
        minOverlap = overlap;
        smallestAxis = axis;
      }
    }
  }

  // Push direction
  if (Vector3DotProduct(D, smallestAxis) < 0)
    smallestAxis = Vector3Negate(smallestAxis);

  *mtv_out = Vector3Scale(smallestAxis, minOverlap);
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

void HandleCuboidCollisions(RigidBody *a, RigidBody *b, Vector3 aDims,
                            Vector3 bDims) {

  Vector3 mtv;
  if (OBBvsOBB(a->position, a->rotation, aDims, b->position, b->rotation, bDims,
               &mtv)) {
    Vector3 normal = Vector3Normalize(mtv);
    Vector3 contactPoint = Vector3Lerp(a->position, b->position,
                                       0.5f); // approximate contact point

    // Resolve position
    a->position = Vector3Subtract(
        a->position, Vector3Scale(normal, 0.5f * Vector3Length(mtv)));
    b->position = Vector3Add(b->position,
                             Vector3Scale(normal, 0.5f * Vector3Length(mtv)));

    // Resolve velocity
    ApplyImpulse(a, b, contactPoint, normal, Vector3Length(mtv));
  }
}
