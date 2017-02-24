//----------------------------------------------------------------------------------
// File:        es3aep-kepler\VisualizeHDR\assets\shaders/renderQuadFS.glsl
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
// Version ID added via C-code prefixing
#extension GL_EXT_shader_io_blocks : enable
precision highp float;
precision highp sampler3D;
precision highp sampler2D;
#UNIFORMS

uniform vec4 uSettings; // vec4(ResolutionX, ResolutionY, Time-ms, 0.0)
uniform vec4 uParams;	// vec4()
uniform sampler2D uTexture;

layout(location=0) out vec4 fragColor;

in block {
	vec4 color;
	vec2 textureCoords;
} In;
/*
vec3 colorBar() {
	vec2 fragCoord = gl_FragCoord.xy;
	vec2 iResolution = uSettings.xy;
	float time = uSettings.z;

	float timeValue = sin(time);
	float divider = iResolution.x;
    //vec2 color = fragCoord.xy/iResolution.y;
    float timeScale = 12.5;
    float colorVal = abs(timeValue) * timeScale;
    //float colorVal = 1.0;
    vec3 finalColor;
    if (fragCoord.x < divider/3.0)
    	finalColor = vec3(colorVal, 0.0,0.0);
    else if (fragCoord.x < divider/1.5)
    	finalColor = vec3(0.0, colorVal,0.0);
    else
    	finalColor = vec3(0.0, 0.0,colorVal);

    if (timeValue < 0.0)  {
   		finalColor = vec3(colorVal, colorVal,colorVal);
    }

    float radius = iResolution.y/12.0;
    vec2 center = vec2(iResolution.x / 2.0, iResolution.y / 2.0);
    float delta = 2.0;

    // Draw circle in the center at 1.0
    // vec2 p = center - fragCoord;
    // if ((abs(timeValue) * timeScale) < 1.02 && (abs(timeValue) * timeScale) > 0.98 )
    // if (p.x * p.x + p.y * p.y < (radius * radius) ) {
    // 	finalColor = vec3(1.0,1.0,1.0);
    // }

    return finalColor;

}

vec3 macBeth() {
	vec3 patches[24];
	vec2 fragCoord = gl_FragCoord.xy;
	vec2 iResolution = uSettings.xy;
	float time = uSettings.z;
	

	// MacBeth colors RGB values clamped between 0.0 and 1.0 and gamma corrected with a 2.0 gamma curve
	patches[0] = vec3(0.4362, 0.3042, 0.2472); 		// Dark Skin
	patches[1] = vec3(0.7675, 0.5853, 0.5072); 		// Light Skin
	patches[2] = vec3(0.4119, 0.4999, 0.6359);		// Blue Sky
	patches[3] = vec3(0.3454, 0.4164, 0.2215);		// Foilage
	patches[4] = vec3(0.5192, 0.5091, 0.7161);		// Blue Flower
	patches[5] = vec3(0.4593, 0.7723, 0.671);		// Bluish Green
	patches[6] = vec3(0.8391, 0.4465, 0.1816);		// Orange
	patches[7] = vec3(0.2895, 0.3709, 0.676);		// Purple Blue
	patches[8] = vec3(0.7293, 0.3142, 0.3738);		// Moderate Red
	patches[9] = vec3(0.3465, 0.222, 0.4402);		// Purple
	patches[10] = vec3(0.6563, 0.7478, 0.1849);		// Yellow Green
	patches[11] = vec3(0.9104, 0.603, 0.1288);		// Orange Yellow
	patches[12] = vec3(0.2014, 0.256, 0.6058);		// Blue
	patches[13] = vec3(0.3147, 0.5831, 0.252);		// Green
	patches[14] = vec3(0.685, 0.1908, 0.2399);		// Red
	patches[15] = vec3(0.946, 0.794, 0.0);			// Yellow
	patches[16] = vec3(0.7311, 0.3125, 0.6015);		// Mangenta
	patches[17] = vec3(0.0, 0.5483, 0.6549);		// Cyan
	patches[18] = vec3(0.9952, 1.0, 0.9923);		// White
	patches[19] = vec3(0.8093, 0.812, 0.8136); 		// Neutral 8
	patches[20] = vec3(0.6417, 0.6406, 0.6419);		// Neutral 6.5
	patches[21] = vec3(0.4724, 0.4747, 0.4726);		// Neutral 5
	patches[22] = vec3(0.3284, 0.3253, 0.3267);		// Neutral 3.5
	patches[23] = vec3(0.1969, 0.193, 0.1955);		// Black

	int valX = int(floor(fragCoord.x) / ceil(iResolution.x / 6.0));
	int modY = int(floor((iResolution.y - fragCoord.y) / ceil(iResolution.y / 4.0)));
	int block = modY * 6 + valX;
    vec3 finalColor = patches[block];

    // Removing gamma correction of 2.0
    finalColor.x = pow(finalColor.x, 2.0);
    finalColor.y = pow(finalColor.y, 2.0);
    finalColor.z = pow(finalColor.z, 2.0);


    return finalColor;
}
*/
void main() {
	// Rec.2020 Red primary
	// vec3 finalColor = vec3(1.93369, 0.19762, 0.03182);

/*	vec4 linearColor;

   	if (uParams.x > 0.5)
    	linearColor = vec4(macBeth(), 1.0);
    else
    	linearColor = vec4(colorBar(), 1.0);
	*/

    vec4 linearColor = texture(uTexture, In.textureCoords);

	vec3 finalColor = linearColor.rgb;    

	// If splitscreen
    if (uParams.y > 0.0) {
        bool is_ldr_pixel = false;
        if (uParams.y > 2.9)
            is_ldr_pixel = true;            // LDR
        else if (uParams.y > 1.9)
            is_ldr_pixel = In.textureCoords.x > 0.5;      // HDR/LDR
        else if (uParams.y > 0.9)
            is_ldr_pixel = In.textureCoords.x < 0.5;      // LDR/HDR
        if (!is_ldr_pixel) {            
            finalColor = tone_map_aces(finalColor);
        }
    } 
    else // No splitscreen
		finalColor = tone_map_aces(finalColor);

    fragColor = vec4(finalColor, 1.0);

    
}
