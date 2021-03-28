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

layout(location = 0) in vec4 vertPos;
layout(location = 1) in vec2 vertTexCoord;
layout(location = 5) in vec3 vertNormal;

layout(location = 0) uniform mat4 modelViewMatrix;
layout(location = 1) uniform mat4 projMatrix;
layout(location = 2) uniform vec3 cameraPosWorld;
layout(location = 3) uniform mat3 normalMatrix;

out vec3 fragPos;
out vec3 cameraPos;
out vec3 normal;
out vec2 uv;

void main()
{
    gl_Position = projMatrix * modelViewMatrix * vertPos;
    fragPos = (modelViewMatrix * vertPos).xyz;
    cameraPos = cameraPosWorld;
    normal = normalMatrix * vertNormal;
    uv = vertTexCoord;
}
