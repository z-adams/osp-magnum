//#version 430 core

/* OSP PBR model shader
  
  There are a number of properties of the OSP universe that lend themselves to
  PBR rendering optimization:

 * Sparse environment
    - At any given time, there are likely to be only a couple significant light
      sources in the scene. The most prominent is the sun, which has a
      non-negligible angular size everywhere except for the outermost planets.
    - In general, the sky is more or less just a 4K black body with no
      significant brightness to speak of. Only the glossiest of mirrors will
      actually show any stars, and in all other cases light can reasonably be
      assumed to only come from solar system bodies and occasionally spaceship
      lights.
    - This means that in many situations, environment maps can be computed
      numerically with relative ease, using angular sizes and relative position.
 * Simple light sources
    - Most common light sources [approximate number at a time]:
        - Nearby system bodies [2-4], which present a direct circular area light
        - Distant system bodies [1], which present a direct point light
          (mostly just the sun, but perhaps a bright, approaching planet)
        - Artificial lights [0-??], e.g. ship lights, explosions, etc.
    - There is probably some way to compute the convolved irradiance map in O(N)
      given that all environment light sources are either circular, or a
      crescent shape that can be approximated analytically
*/

// Varyings
in vec3 normal;
in vec3 fragPos;
in vec2 uv;

// Outputs
layout(location = 0, index = 0) out vec4 fragColor;

// Uniforms
layout(location = 3) uniform vec3 sunDir;
layout(location = 4) uniform vec3 camPos;
layout(location = 5) uniform sampler2D albedoMap;
layout(location = 6) uniform sampler2D metallicRoughMap;
layout(location = 7) uniform sampler2D normalMap;

const float PI = 3.14159265358;

vec3 fresnel_schlick(float cosTheta, vec3 F_0)
{
    return F_0 + (1.0 - F_0) * pow(1.0 - cosTheta, 5.0);
}

float distribution_GGX(vec3 N, vec3 H, float roughness)
{
    float a = roughness*roughness;
    float NdotH = max(dot(N, H), 0.0);
    float NdotH2 = NdotH*NdotH;

    float num = a*a;
    float denom = (NdotH2 * (a*a - 1.0) + 1.0);
    denom = PI * denom * denom;

    return num / denom;
}

float geometry_schlick_GGX(float NdotV, float roughness)
{
    float r = roughness + 1.0;
    float k = (r*r) / 8.0;
    
    float num = NdotV;
    float denom = NdotV * (1.0 - k) + k;

    return num / denom;
}

float geometry_smith(vec3 N, vec3 V, vec3 L, float roughness)
{
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    float ggx2 = geometry_schlick_GGX(NdotV, roughness);
    float ggx1 = geometry_schlick_GGX(NdotL, roughness);

    return ggx1 * ggx2;
}

vec3 pbr()
{
    float roughness = texture2D(metallicRoughMap, uv).g;
    float metallic = texture2D(metallicRoughMap, uv).b;
    vec3 albedo = texture2D(albedoMap, uv).rgb;

    vec3 N = normal;
    vec3 V = normalize(camPos - fragPos);

    vec3 L_o = vec3(0.0);  // output radiance

    vec3 L = sunDir;
    vec3 H = normalize(V + L);
    vec3 sunRad = vec3(1.0);
    vec3 F_0 = vec3(0.04);
    F_0 = mix(F_0, albedo, metallic);
    vec3 F = fresnel_schlick(max(dot(H, V), 0.0), F_0);

    float NDF = distribution_GGX(N, H, roughness);
    float G = geometry_smith(N, V, L, roughness);

    vec3 numerator = NDF * G * F;
    float denominator = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0);
    vec3 specular = numerator / max(denominator, 0.001);

    vec3 kS = F;
    vec3 kD = vec3(1.0) - kS;

    kD *= 1.0 - metallic;

    float NdotL = max(dot(N, L), 0.0);
    vec3 radNL = sunRad * NdotL;
    L_o = ((kD * albedo) / PI + specular) * radNL;

    vec3 color = L_o / (L_o + vec3(1.0));
    color = pow(color, vec3(1.0/2.2));

    return color;
}

void main()
{

    fragColor = vec4(pbr(), 1.0);
    //fragColor = vec4(vec3(dot(normal, sunDir)), 1.0);
    //fragColor = vec4(normal, 1.0);
}