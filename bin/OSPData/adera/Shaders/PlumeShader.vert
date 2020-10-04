//#version 430 core

layout(location = 0) in vec4 vertPosition;
layout(location = 1) in vec2 vertTexCoords;
layout(location = 5) in vec3 vertNormal;

out vec3 normal;
out vec3 fragPos;

layout(location = 0) uniform mat4 projMat;
layout(location = 1) uniform mat4 modelTransformMat;
layout(location = 2) uniform mat3 normalMat;

void main()
{
    gl_Position = projMat * modelTransformMat * vertPosition;
    normal = normalMat * vertNormal;

    // Send vert position to frag shader for UV calculation
    fragPos = vec3(vertPosition);
}