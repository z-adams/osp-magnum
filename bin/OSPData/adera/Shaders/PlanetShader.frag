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

const float PI = 3.14159265359;
const float PI_2 = PI/2.0;
const float PI_4 = PI/4.0;



/**
 * Converts cartesian vector XYZ to spherical surface (theta, phi) coordinates
 */
vec2 xyz_to_sphere_angles(vec3 xyz)
{
    float r = length(xyz);
    float theta = acos(xyz.z / r);
    float phi = atan(xyz.y, xyz.x);

    return vec2(theta, phi);
}

#define CM_POS_X 0
#define CM_NEG_X 1
#define CM_POS_Y 2
#define CM_NEG_Y 3
#define CM_POS_Z 4
#define CM_NEG_Z 5

uint face_from_angles(float theta, float phi)
{
    uint face = 0;
    if (theta < PI_4)
    {   // Top face (posY -> RH +Z)
        face = CM_POS_Y;
    }
    else if (theta > 3*PI_4)
    {   // Bottom face (negY -> RH -Z)
        face = CM_NEG_Y;
    }
    else
    {   // Side faces
        if ((phi > PI_4) && (phi < 3*PI_4))
        {   // RH +Y -> negZ
            face = CM_NEG_Z;
        }
        else if ((phi >= 3*PI_4) && (phi < 5*PI_4))
        {   // RH -X -> negX
            face = CM_NEG_X;
        }
        else if ((phi >= 5*PI_4) && (phi < 7*PI_4))
        {   // RH -Y -> posZ
            face = CM_POS_Z;
        }
        else
        {   // posX
            face = CM_POS_X;
        }
    }
    return face;
}

vec3 TBN()
{
    vec2 angles = xyz_to_sphere_angles(radialOut);
    float theta = angles.x;
    float phi = angles.y;

    uint face = face_from_angles(theta, phi);

    switch (face)
    {
    case CM_POS_X:
        return vec3(1, 0, 0);
    case CM_NEG_X:
        return vec3(0, 1, 1);
    case CM_POS_Y:
        return vec3(0, 1, 0);
    case CM_NEG_Y:
        return vec3(1, 0, 1);
    case CM_POS_Z:
        return vec3(0, 0, 1);
    case CM_NEG_Z:
        return vec3(1, 1, 0);
    }
}

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
    //color = vec3(depth(radialOut));
    //color = vec3(radialOut.x); //vec3(xyz_to_sphere_angles(radialOut).y);
    color = texture(diffuseMap, radialOut).rgb;
}
