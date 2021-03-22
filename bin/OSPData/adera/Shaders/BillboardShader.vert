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

layout(location = 0) in vec3 vertPosition;
layout(location = 1) in vec2 vertTexCoords;

layout(location = 0) uniform mat4 projMat;
layout(location = 1) uniform mat4 viewMat;
layout(location = 2) uniform vec3 worldPos;

out vec2 uv;

vec3 camera_up = {viewMat[0][1], viewMat[1][1], viewMat[2][1]};
vec3 camera_right = {viewMat[0][0], viewMat[1][0], viewMat[2][0]};

vec3 direction = {1.0, 0.0, 0.0};
float size = 100.0;

void main()
{
    vec3 pos_world = worldPos
        + camera_right * vertPosition.x * size
        + camera_up * vertPosition.y * size;

    gl_Position = projMat * viewMat * vec4(pos_world, 1.0);
    uv  = vertTexCoords;
}

