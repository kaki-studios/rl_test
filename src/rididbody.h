#include <raylib.h>
#include <raymath.h>

typedef struct CuboidRigidBody {
  float mass;
  Mesh *mesh;
  Vector3 position;
  Quaternion rotation;

  Matrix transform;

  // NOTE: no need for center of mass (yet): GenMeshCube sets mesh origin =
  // center of mass
  Vector3 dimensions; // width, length, heigth

  Vector3 linearVelocity; // from center of mass
  // direction the rb "wants" to turn around
  Vector3 angularMomentum;
  Vector3 invInertiaMatrix; // note that this is a diagonal Matrix(3x3) stored
                            // as a Vector3

} CuboidRigidBody;

CuboidRigidBody CreateRB(float, Vector3, Vector3);

void UpdateRB(CuboidRigidBody *, float, Vector3 *);
void DeinitRB(CuboidRigidBody *);

Matrix TransformToMatrix(Vector3, Quaternion);
