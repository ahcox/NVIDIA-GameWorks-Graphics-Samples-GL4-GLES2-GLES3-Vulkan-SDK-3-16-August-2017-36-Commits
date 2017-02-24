//----------------------------------------------------------------------------------
// File:        es3aep-kepler\VisualizeHDR\assets\shaders/uniforms.h
// SDK Version: v3.00 
// Email:       gameworks@nvidia.com
// Site:        http://developer.nvidia.com/
//
// Copyright (c) 2014-2015, NVIDIA CORPORATION. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//----------------------------------------------------------------------------------
// A special hybrid header file, designed to be included by both C++ and
// GLSL compilers.  Makes it much easier to create and maintain shader
// buffers
#ifndef __cplusplus

#define sdk_bool bool
// Standard SDK defines
#define SDK_BOOL  bool
#define SDK_VEC2  vec2
#define SDK_VEC3  vec3
#define SDK_VEC4  vec4
#define SDK_MAT4  mat4

layout(std140, binding=1) uniform
#else
struct
#endif
    
ShaderParams
{
    SDK_MAT4 ModelView;
    SDK_MAT4 InvViewMatrix;
    SDK_MAT4 ModelViewProjection;
    SDK_MAT4 ProjectionMatrix;

#ifdef CPP
    ShaderParams(){}
#endif
};


#ifndef __cplusplus
// Aces Uniforms
precision highp sampler3D;
precision highp sampler2D;
uniform sampler3D uAcesLUT; 
uniform vec4 uAcesParams;

bool aces_is_hdr() {
    return uAcesParams[3] > 0.5;
}

// Sample from ACES LUT
vec3 tone_map_aces(vec3 color) {    
    if (!aces_is_hdr())
        return color;

    // log10 encode lookup
    color = log2(color + 0.00000000000001) / log2(10.0); // avoid -inf generation   
    color = color * uAcesParams.x + uAcesParams.y;
    color = color * 31.0 / 32.0 + 0.5 / 32.0;
    return texture(uAcesLUT, color.xyz).rgb;
}
#endif