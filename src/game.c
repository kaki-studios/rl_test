#include "collision.h"
#include "cuboid_rb.h"
#include "rigidbody.h"
#include <math.h>
#include <raylib.h>
#include <raymath.h>

#define CAMERA_RADIUS 15.0f
#define RB_COUNT 2
#if defined(PLATFORM_DESKTOP)
#define GLSL_VERSION 330
#else // PLATFORM_ANDROID, PLATFORM_WEB
#define GLSL_VERSION 100
#endif

Vector3 NewCamPos(Vector2, float);
void UpdateCamPos(Camera3D *, Vector2 *);

int main(void) {
  const int width = 800;
  const int height = 400;
  InitWindow(width, height, "Raylib Test");
  SetTargetFPS(60);
  // camera setup, rn just orbits center
  Camera3D camera = {0};
  Vector2 sphereCoords = {PI, 0.5f};
  camera.position = NewCamPos(sphereCoords, CAMERA_RADIUS);

  camera.target = (Vector3){0.0f, 0.0f, 0.0f}; // Looking at the origin
  camera.up = (Vector3){0.0f, 1.0f, 0.0f};     // Up vector
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;

  Vector3 rb_dims = {3.0f, 6.0f, .5f};
  Vector3 sb_dims = {50.0f, 1.0f, 50.0f};
  RigidBody rbs[RB_COUNT];
  CuboidStaticBody ground = CreateCuboidSB((Vector3){0.0f, -1.0f, 0.0f},
                                           QuaternionIdentity(), sb_dims);

  rbs[0] = CreateCuboidRB(2.0f, (Vector3){20.0f, 5.0f, 0.0f}, rb_dims);
  rbs[0].angularMomentum = (Vector3){150.0f, 0.1f, 0.1f};
  rbs[0].linearVelocity = (Vector3){-3.0f, -1.0f, 0.0f};

  rbs[1] = CreateCuboidRB(2.0f, (Vector3){-20.0f, 5.0f, 0.0f}, rb_dims);

  rbs[1].angularMomentum = (Vector3){100.f, 0.1f, 0.1f};
  rbs[1].linearVelocity = (Vector3){3.0f, -1.0f, 0.0f};

  Material mat = LoadMaterialDefault();
  Texture2D tex = LoadTexture("./assets/grassblock.png");
  mat.maps[MATERIAL_MAP_DIFFUSE].texture = tex;

  Shader shader = LoadShader("./assets/lighting.vs", "./assets/lighting.fs");
  mat.shader = shader;

  Model hammerModel = LoadModel("./assets/hammer.obj");
  RigidBody rb2 =
      CreateRB(&hammerModel.meshes[0], 2.0f, (Vector3){10.0f, 0.0f, 0.0f});
  rb2.angularMomentum = (Vector3){100.0f, 0.1f, 0.0f};
  hammerModel.materials[0].shader = shader;

  // shader uniforms setup

  // Set uniforms
  Color albedo = (Color){255, 255, 255, 255};
  Vector4 albedoVec = {albedo.r / 255.0f, albedo.g / 255.0f, albedo.b / 255.0f,
                       albedo.a / 255.0f};
  SetShaderValue(shader, GetShaderLocation(shader, "albedoColor"), &albedoVec,
                 SHADER_UNIFORM_VEC4);

  Color ambient = (Color){100, 100, 100, 255};
  Vector4 ambientVec = {ambient.r / 255.0f, ambient.g / 255.0f,
                        ambient.b / 255.0f, ambient.a / 255.0f};
  SetShaderValue(shader, GetShaderLocation(shader, "ambientColor"), &ambientVec,
                 SHADER_UNIFORM_VEC4);

  Vector3 lightPos = {10.0f, 10.0f, 10.0f};
  SetShaderValue(shader, GetShaderLocation(shader, "lightPosition"), &lightPos,
                 SHADER_UNIFORM_VEC3);

  Color light = (Color){255, 255, 255, 255};
  Vector4 lightVec = {light.r / 255.0f, light.g / 255.0f, light.b / 255.0f,
                      light.a / 255.0f};
  SetShaderValue(shader, GetShaderLocation(shader, "lightColor"), &lightVec,
                 SHADER_UNIFORM_VEC4);

  int useTex = 1;
  SetShaderValue(shader, GetShaderLocation(shader, "useTexture"), &useTex,
                 SHADER_UNIFORM_INT);

  double timeAcc = 0.0f;
  Vector3 angVel = {0.f, 0.f, 0.f};

  while (!WindowShouldClose()) // Detect window close button or ESC key
  {
    // input
    UpdateCamPos(&camera, &sphereCoords);

    // Update
    // avoid too long deltatimes:
    double H = 0.001f;
    timeAcc += GetFrameTime();
    while (timeAcc >= H) {

      UpdateRB(&rb2, H);
      for (int i = 0; i < RB_COUNT; i++) {
        UpdateRB(&rbs[i], H);
      }
      for (int i = 0; i < RB_COUNT; i++) {
        // check ground collision
        HandleCuboidRBSBCollisions(&rbs[i], &ground, rb_dims, sb_dims);
        for (int j = i + 1; j < RB_COUNT; j++) {
          HandleCuboidRBCollisions(&rbs[i], &rbs[j], rb_dims, rb_dims);
        }
      }

      timeAcc -= H;
    }

    // Draw

    BeginDrawing();
    ClearBackground(GRAY);
    DrawFPS(50, 50);

    BeginMode3D(camera);
    for (int i = 0; i < RB_COUNT; i++) {

      DrawMesh(*rbs[i].mesh, mat, rbs[i].transform);
      DrawLine3D(rbs[i].position,
                 Vector3Add(rbs[i].position, rbs[i].angularMomentum), GREEN);
      DrawLine3D(rbs[i].position,
                 Vector3Add(rbs[i].position, rbs[i].linearVelocity), RED);
    }
    DrawMesh(*ground.mesh, mat, ground.transform);
    DrawMesh(*rb2.mesh, hammerModel.materials[0], rb2.transform);
    DrawLine3D(rb2.position, Vector3Add(rb2.position, rb2.angularMomentum),
               GREEN);
    DrawLine3D(rb2.position, Vector3Add(rb2.position, rb2.linearVelocity), RED);

    // debug
    // DrawCylinderEx(rb.position, Vector3Add(rb.position, rb.angularMomentum),
    //                0.1f, 0.1f, 20, GREEN);
    // DrawCylinderEx(rb.position, Vector3Add(rb.position, rb.linearVelocity),
    //                0.1f, 0.1f, 20, RED);
    //
    // DrawCylinderEx(rb2.position, Vector3Add(rb2.position,
    // rb2.angularMomentum),
    //                0.1f, 0.1f, 20, GREEN);
    // DrawCylinderEx(rb2.position, Vector3Add(rb2.position,
    // rb2.linearVelocity),
    //                0.1f, 0.1f, 20, RED);

    DrawCylinderEx((Vector3){0.0f, 0.0f, 0.0f}, (Vector3){5.0f, 0.0f, 0.0f},
                   0.05f, 0.05f, 20, RED);
    DrawCylinderEx((Vector3){0.0f, 0.0f, 0.0f}, (Vector3){0.0f, 5.0f, 0.0f},
                   0.05f, 0.05f, 20, YELLOW);
    DrawCylinderEx((Vector3){0.0f, 0.0f, 0.0f}, (Vector3){0.0f, 0.0f, 5.0f},
                   0.05f, 0.05f, 20, BLUE);

    DrawGrid(50, 1.0f);

    EndMode3D();
    EndDrawing();
  }

  // De-initialization
  DeinitRB(&rbs[0]);
  DeinitRB(&rbs[1]);
  UnloadMaterial(mat);

  CloseWindow(); // Close window and OpenGL context

  return 0;
}

Vector3 NewCamPos(Vector2 sphereCoords, float length) {
  // find a circle at a latitude, determined by sphereCoords.y
  float height = length * sinf(sphereCoords.y);

  float radius = length * cosf(sphereCoords.y);
  // find coords within this circle
  float x = radius * sinf(sphereCoords.x);
  float y = radius * cosf(sphereCoords.x);
  // now final pos is x,height,y

  return (Vector3){x, height, y};
}

void UpdateCamPos(Camera3D *camera, Vector2 *sphereCoords) {
  if (IsKeyDown(KEY_UP)) {
    sphereCoords->y += 1 * GetFrameTime();
  }
  if (IsKeyDown(KEY_DOWN)) {
    sphereCoords->y -= 1 * GetFrameTime();
  }
  if (IsKeyDown(KEY_RIGHT)) {
    sphereCoords->x += 1 * GetFrameTime();
  }
  if (IsKeyDown(KEY_LEFT)) {
    sphereCoords->x -= 1 * GetFrameTime();
  }
  sphereCoords->y = Clamp(sphereCoords->y, -PI / 2, PI / 2);
  // instead of clamping x we can wrap instead:
  sphereCoords->x = Wrap(sphereCoords->x, -PI, PI);
  camera->position = NewCamPos(*sphereCoords, CAMERA_RADIUS);
}
