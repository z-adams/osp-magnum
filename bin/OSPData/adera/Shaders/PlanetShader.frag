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
//documentation here

in vec3 normal;
in vec3 radialOut;
in vec3 lightPos;

layout(location = 0, index = 0) out vec3 color;

layout(location = 3) uniform samplerCube diffuseMap;
layout(location = 4) uniform samplerCube normalMap;
layout(location = 5) uniform samplerCube displacementMap;

float depth(vec3 r)
{
    return 1.0 - texture(displacementMap, r).r;
}

float lighting(vec3 normalWorld)
{
    vec3 normalVal = texture(normalMap, radialOut).xyz;
    return clamp(dot(normalVal, lightPos), 0, 1);
}

void main()
{
    //color = texture(normalMap, radialOut).rgb;//*lighting(normalize(radialOut));
    //color = vec3(1.0)*lighting(normalize(radialOut));
    color = vec3(depth(radialOut));
}
