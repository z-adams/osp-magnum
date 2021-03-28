/**
 * Open Space Program
 * Copyright � 2019-2020 Open Space Program Project
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

in vec3 fragPos;
in vec3 cameraPos;
in vec3 normal;
in vec2 uv;

layout(location = 0) out vec4 gCastRay_Depth;
layout(location = 1) out vec4 gNormalXY_HitUV;

layout(pixel_center_integer) in vec4 gl_FragCoord;

// Tmp
const int width = 1280;
const int height = 720;

vec3 encode(vec3 inVal)
{
    return 0.5*normalize(inVal) + 0.5;
}

void main()
{
    vec3 ray = fragPos - cameraPos;
    vec2 normal2D = encode(normal).xy;
    float depth = gl_FragCoord.z;
    vec2 screenCoord = gl_FragCoord.xy / vec2(1280.0, 720.0);
    gCastRay_Depth = vec4(encode(ray), depth);
    gNormalXY_HitUV = vec4(encode(normal), 1.0);
}
