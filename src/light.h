#include <raylib.h>
// Light type
typedef enum { LIGHT_DIRECTIONAL = 0, LIGHT_POINT, LIGHT_SPOT } LightType;
#define MAX_LIGHTS 4

// Light data
typedef struct {
  int type;
  int enabled;
  Vector3 position;
  Vector3 target;
  float color[4];
  float intensity;

  // Shader light parameters locations
  int typeLoc;
  int enabledLoc;
  int positionLoc;
  int targetLoc;
  int colorLoc;
  int intensityLoc;
} Light;

//----------------------------------------------------------------------------------
// Global Variables Definition
//----------------------------------------------------------------------------------
static int lightCount =
    0; // Current number of dynamic lights that have been created

//----------------------------------------------------------------------------------
// Module specific Functions Declaration
//----------------------------------------------------------------------------------
// Create a light and get shader locations
static Light CreateLight(int type, Vector3 position, Vector3 target,
                         Color color, float intensity, Shader shader);

// Update light properties on shader
// NOTE: Light shader locations should be available
static void UpdateLight(Shader shader, Light light);
