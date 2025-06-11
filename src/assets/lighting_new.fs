#version 330
in vec2 fragTexCoord;
in vec3 fragNormal;
in vec3 fragPosition;

uniform vec4 colDiffuse;      // This is raylib's built-in material color uniform
uniform vec4 albedoColor;     // Keep this for additional tinting if needed
uniform vec4 ambientColor;
uniform vec3 lightPosition;
uniform vec4 lightColor;
uniform int useTexture;
uniform sampler2D texture0;

out vec4 finalColor;

void main()
{
    // Base albedo color - now uses material's diffuse color
    vec4 albedo = colDiffuse;  // This automatically gets the material's color
    
    if (useTexture == 1) {
        albedo *= texture(texture0, fragTexCoord);  // Multiply texture with material color
    }
    
    // Optionally multiply by albedoColor for additional tinting
    albedo *= albedoColor;
    
    // Lambert diffuse
    vec3 N = normalize(fragNormal);
    vec3 L = normalize(lightPosition - fragPosition);
    float diff = max(dot(N, L), 0.0);
    vec3 diffuse = diff * lightColor.rgb * albedo.rgb;
    vec3 ambient = ambientColor.rgb * albedo.rgb;
    
    finalColor = vec4(diffuse + ambient, albedo.a);
}
