/**
 * Open Space Program
 * Copyright © 2019-2020 Open Space Program Project
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
//#version 430 core

layout(location = 0) in vec4 vertPosition;
layout(location = 1) in vec2 vertTexCoords;
layout(location = 5) in vec3 vertNormal;

vec3 sunDirection = {1, 0, 0};

// Surface normal
out vec3 normal;

// Sphere normal (center to edge)
out vec3 radialOut;

// Model transformation
out mat3 modelTransform;

layout(location = 0) uniform mat4 projMat;
layout(location = 1) uniform mat4 modelTransformMat;
layout(location = 2) uniform mat3 normalMat;

void main()
{
    gl_Position = projMat * modelTransformMat * vertPosition;
    normal = normalMat * vertNormal;
    radialOut = vertPosition.xyz;
    modelTransform = mat3(modelTransformMat);
}
