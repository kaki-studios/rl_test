#include "rigidbody.h"
#include "staticbody.h"
#include <raylib.h>

#define MAX_DEBUG_CONTACTS 128

typedef struct {
  Vector3 position;
  Vector3 normal;
  float penetration;
} DebugContact;
int GetDebugContactCount(void);
const DebugContact *GetDebugContacts(void); // return pointer to internal buffer

void ClearDebugContacts(void);
void PushDebugContact(Vector3 position, Vector3 normal, float penetration);

bool OBBvsOBB(Vector3 p1, Quaternion q1, Vector3 s1, Vector3 p2, Quaternion q2,
              Vector3 s2, Vector3 *mtv_out, Vector3 *contactPoint_out);

void ApplyImpulse(RigidBody *a, RigidBody *b, Vector3 contactPoint,
                  Vector3 normal, float penetrationDepth);

void HandleCuboidRBCollisions(RigidBody *a, RigidBody *b, Vector3 aDims,
                              Vector3 bDims);
