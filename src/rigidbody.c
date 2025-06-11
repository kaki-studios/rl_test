#include "rigidbody.h"
#include <raylib.h>
#include <raymath.h>
#include <stdlib.h>

// helpers, maybe move into another file
//

Matrix3 Matrix3Mult(Matrix3 a, Matrix3 b) {
  Matrix3 result;

  result.m0 = a.m0 * b.m0 + a.m3 * b.m1 + a.m6 * b.m2;
  result.m1 = a.m1 * b.m0 + a.m4 * b.m1 + a.m7 * b.m2;
  result.m2 = a.m2 * b.m0 + a.m5 * b.m1 + a.m8 * b.m2;

  result.m3 = a.m0 * b.m3 + a.m3 * b.m4 + a.m6 * b.m5;
  result.m4 = a.m1 * b.m3 + a.m4 * b.m4 + a.m7 * b.m5;
  result.m5 = a.m2 * b.m3 + a.m5 * b.m4 + a.m8 * b.m5;

  result.m6 = a.m0 * b.m6 + a.m3 * b.m7 + a.m6 * b.m8;
  result.m7 = a.m1 * b.m6 + a.m4 * b.m7 + a.m7 * b.m8;
  result.m8 = a.m2 * b.m6 + a.m5 * b.m7 + a.m8 * b.m8;

  return result;
}
float Sq(float a) { return a * a; }
float IToE(Vector3 in, unsigned int I) {
  switch (I) {
  case 0:
    return in.x;
  case 1:
    return in.y;
  case 2:
    return in.z;
  }
  return 0.0f;
}

Matrix Matrix3ToMatrix(Matrix3 in) {
  Matrix out = (Matrix){in.m0, in.m3, in.m6, 0.0f, in.m1, in.m4, in.m7, 0.0f,
                        in.m2, in.m5, in.m8, 0.0f, 0.0f,  0.0f,  0.0f,  0.0f};
  return out;
}

Matrix3 StripMatrixToMatrix3(Matrix in) {
  Matrix3 out = (Matrix3){
      in.m0, in.m4, in.m8, in.m1, in.m5, in.m9, in.m2, in.m6, in.m10,
  };
  return out;
}

Matrix ComputeWorldInertia(Matrix bodyInertia, Quaternion orientation) {
  // Convert quaternion to rotation matrix
  Matrix R = QuaternionToMatrix(orientation);

  // Compute R^T (transpose)
  Matrix RT = MatrixTranspose(R);

  // Compute R * J
  Matrix temp = MatrixMultiply(R, bodyInertia);

  // Compute (R * J) * R^T = R * J * R^T
  Matrix worldInertia = MatrixMultiply(temp, RT);

  return worldInertia;
}

Vector3 MultiplyMatrixVector3(Matrix3 mat, Vector3 v) {
  Vector3 result;
  result.x = mat.m0 * v.x + mat.m3 * v.y + mat.m6 * v.z;
  result.y = mat.m1 * v.x + mat.m4 * v.y + mat.m7 * v.z;
  result.z = mat.m2 * v.x + mat.m5 * v.y + mat.m8 * v.z;
  return result;
}

Matrix3 InverseMatrix3(Matrix3 mat) {
  Matrix3 inv;

  float a00 = mat.m0, a01 = mat.m3, a02 = mat.m6;
  float a10 = mat.m1, a11 = mat.m4, a12 = mat.m7;
  float a20 = mat.m2, a21 = mat.m5, a22 = mat.m8;

  float det = a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20) +
              a02 * (a10 * a21 - a11 * a20);

  if (det == 0.0f) {
    // Singular matrix, return identity or handle error
    inv = (Matrix3){1, 0, 0, 0, 1, 0, 0, 0, 1};
    return inv;
  }

  float invDet = 1.0f / det;

  inv.m0 = (a11 * a22 - a12 * a21) * invDet;
  inv.m3 = -(a01 * a22 - a02 * a21) * invDet;
  inv.m6 = (a01 * a12 - a02 * a11) * invDet;

  inv.m1 = -(a10 * a22 - a12 * a20) * invDet;
  inv.m4 = (a00 * a22 - a02 * a20) * invDet;
  inv.m7 = -(a00 * a12 - a02 * a10) * invDet;

  inv.m2 = (a10 * a21 - a11 * a20) * invDet;
  inv.m5 = -(a00 * a21 - a01 * a20) * invDet;
  inv.m8 = (a00 * a11 - a01 * a10) * invDet;

  return inv;
}

// real functions

float ComputeInertiaProduct(Vector3 P[3], unsigned int I, unsigned int J) {
  float r1 = 2.0 * IToE(P[0], I) * IToE(P[0], J) +
             IToE(P[1], I) * IToE(P[2], J) + IToE(P[2], I) * IToE(P[1], J);

  float r2 = 2.0 * IToE(P[1], I) * IToE(P[1], J) +
             IToE(P[0], I) * IToE(P[2], J) + IToE(P[2], I) * IToE(P[0], J);

  float r3 = 2.0 * IToE(P[2], I) * IToE(P[2], J) +
             IToE(P[0], I) * IToE(P[1], J) + IToE(P[1], I) * IToE(P[0], J);
  return r1 + r2 + r3;
}

float ComputeInertiaMoment(Vector3 P[3], unsigned int I) {
  float r1 = Sq(IToE(P[0], I)) + IToE(P[1], I) * IToE(P[2], I);
  float r2 = Sq(IToE(P[1], I)) + IToE(P[0], I) * IToE(P[2], I);
  float r3 = Sq(IToE(P[2], I)) + IToE(P[0], I) * IToE(P[1], I);
  return r1 + r2 + r3;
}

void MeshComputeInertiaMatrix(Mesh *mesh, float density, Matrix3 *I,
                              Vector3 *CoM, float *massOut) {
  float mass = 0.0f;
  Vector3 massCenter = {0.0f, 0.f, 0.f};
  bool usesIndices = mesh->indices != NULL;
  float Ia = 0.f, Ib = 0.f, Ic = 0.f, Iap = 0.f, Ibp = 0.0, Icp = 0.0;
  for (unsigned int I = 0; I < mesh->triangleCount; ++I) {
    Vector3 P[3];
    for (unsigned int J = 0; J < 3; ++J) {
      unsigned int index;
      if (usesIndices) {
        index = mesh->indices[I * 3 + J];
      } else {
        index = I * 3 + J;
      }
      P[J].x = mesh->vertices[index * 3];
      P[J].y = mesh->vertices[index * 3 + 1];
      P[J].z = mesh->vertices[index * 3 + 2];
    }
    // this is signed
    float DetJ = Vector3DotProduct(P[0], Vector3CrossProduct(P[1], P[2]));
    float TetVolume = DetJ / 6.0f;
    float TetMass = density * TetVolume;
    // divide by 4 since the 4th point is the origin, get the average of their
    // points
    Vector3 TetMassCenter =
        Vector3Scale(Vector3Add(Vector3Add(P[0], P[1]), P[2]), 0.25f);

    Ia += DetJ * (ComputeInertiaMoment(P, 1) + ComputeInertiaMoment(P, 2));
    Ib += DetJ * (ComputeInertiaMoment(P, 0) + ComputeInertiaMoment(P, 2));
    Ic += DetJ * (ComputeInertiaMoment(P, 0) + ComputeInertiaMoment(P, 1));
    Iap += DetJ * (ComputeInertiaProduct(P, 1, 2));
    Ibp += DetJ * (ComputeInertiaProduct(P, 0, 1));
    Icp += DetJ * (ComputeInertiaProduct(P, 0, 2));
    massCenter = Vector3Add(Vector3Scale(TetMassCenter, TetMass), massCenter);
    mass += TetMass;
  }
  massCenter = Vector3Scale(massCenter, 1 / mass);
  Ia = density * Ia / 60.0f - mass * (Sq(massCenter.y) + Sq(massCenter.z));
  Ib = density * Ib / 60.0f - mass * (Sq(massCenter.x) + Sq(massCenter.z));
  Ic = density * Ic / 60.0f - mass * (Sq(massCenter.x) + Sq(massCenter.y));
  Iap = density * Iap / 120.0f - mass * (massCenter.y * massCenter.z);
  Ibp = density * Ibp / 120.0f - mass * (massCenter.x * massCenter.y);
  Icp = density * Icp / 120.0f - mass * (massCenter.x * massCenter.z);

  Matrix3 MatI = {
      Ia, -Ibp, -Icp, -Ibp, Ib, -Iap, -Icp, -Iap, Ic,
  };
  *I = MatI;
  *CoM = massCenter;
  *massOut = mass;
}

Matrix TransformToMatrix(Vector3 pos, Quaternion rot) {
  Matrix res = QuaternionToMatrix(rot);
  res.m12 = pos.x;
  res.m13 = pos.y;
  res.m14 = pos.z;
  return res;
}

void ApplyAngularVelocity(Quaternion *rot, Vector3 angularVelocity,
                          float deltaTime) {
  Quaternion w = {angularVelocity.x, angularVelocity.y, angularVelocity.z,
                  0.0f};

  // dq/dt = 0.5 * q * w
  Quaternion dq = QuaternionMultiply(*rot, w);
  dq.x *= 0.5f * deltaTime;
  dq.y *= 0.5f * deltaTime;
  dq.z *= 0.5f * deltaTime;
  dq.w *= 0.5f * deltaTime;

  // q += dq
  rot->x += dq.x;
  rot->y += dq.y;
  rot->z += dq.z;
  rot->w += dq.w;

  // Normalize to prevent drift
  *rot = QuaternionNormalize(*rot);
}

void ApplyLinearVelocity(Vector3 *pos, Vector3 linearVelocity,
                         float deltaTime) {

  Vector3 deltaMove = Vector3Scale(linearVelocity, deltaTime);
  *pos = Vector3Add(*pos, deltaMove);
}

Vector3 ComputeAngularVelocity(Matrix3 invInertiaMatrix, Quaternion rot,
                               Vector3 angMomentum) {
  Matrix R = QuaternionToMatrix(rot);
  Matrix3 Rt = StripMatrixToMatrix3(MatrixTranspose(R));
  Matrix3 I_inv_world =
      Matrix3Mult(StripMatrixToMatrix3(R), Matrix3Mult(invInertiaMatrix, Rt));
  return MultiplyMatrixVector3(I_inv_world, angMomentum);
  // // Step 1: Rotate angular momentum into local space
  // Vector3 localAngMom = Vector3RotateByQuaternion(angMomentum, invRot);
  // Vector3 localAngMom = Vector3RotateByQuaternion(angMomentum, rot);

  // Step 2: Multiply by I^-1
  // Vector3 localAngVel = MultiplyMatrixVector3(invInertiaMatrix, localAngMom);
  // Vector3 angVel = MultiplyMatrixVector3(invInertiaMatrix, angMomentum);

  // // Step 3: Rotate result back to world space
  // Vector3 angVel = Vector3RotateByQuaternion(localAngVel, rot);
  // Vector3 angVel = Vector3RotateByQuaternion(localAngVel, invRot);

  // return angVel;
}

void UpdateRB(RigidBody *rb, float deltaTime) {
  // steps:
  // update linear_momentum and angular_momentum if needed
  // calculate quat and pos based on them
  // normalize quat
  // calculate transform based on that

  // first: gravity
  // if (rb->invMass != 0.0f) {
  //   rb->linearVelocity.y -= deltaTime * 9.81;
  // }
  // move based on linear_momentum
  ApplyLinearVelocity(&rb->position, rb->linearVelocity, deltaTime);
  Vector3 angVel = ComputeAngularVelocity(rb->invInertiaMatrix, rb->rotation,
                                          rb->angularMomentum);
  ApplyAngularVelocity(&rb->rotation, angVel, deltaTime);
  // apply changes to transform matrix
  rb->transform = TransformToMatrix(rb->position, rb->rotation);
}

RigidBody CreateRB(Mesh *mesh, float density, Vector3 position) {
  Matrix3 inertia;
  float mass;
  Vector3 centerOfMass;
  MeshComputeInertiaMatrix(mesh, density, &inertia, &centerOfMass, &mass);
  for (unsigned long int I = 0; I < mesh->vertexCount; ++I) {
    mesh->vertices[I * 3] = mesh->vertices[I * 3] - centerOfMass.x;
    mesh->vertices[I * 3 + 1] = mesh->vertices[I * 3 + 1] - centerOfMass.y;
    mesh->vertices[I * 3 + 2] = mesh->vertices[I * 3 + 2] - centerOfMass.z;
  }

  Matrix3 invIner = InverseMatrix3(inertia);
  return (RigidBody){.mesh = mesh,
                     .density = density,
                     .invMass = 1.0f / mass,
                     .position = position,
                     .rotation = QuaternionIdentity(),
                     .transform =
                         MatrixTranslate(position.x, position.y, position.z),
                     .centerOfMass = centerOfMass,
                     .linearVelocity = {0.0f, 0.0f, 0.0f},
                     .angularMomentum = {0.0f, 0.0f, 0.0f},
                     .invInertiaMatrix = invIner,
                     .restitution = 0.1f,
                     .friction = 1.0f};
}

void DeinitRB(RigidBody *rb) { UnloadMesh(*rb->mesh); }
