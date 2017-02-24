//----------------------------------------------------------------------------------
// File:        es3aep-kepler\VisualizeHDR/aces.h
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
#pragma once

#include <NV/NvPlatformGL.h>

#if !defined(ANDROID)
#define DEBUG
#endif

class Aces
{
public:
	Aces();
	~Aces();

#if 0
	// Selected tonemapping curves based on sink capabilites
	enum ToneCurve {
		Tone_Curve_LDR,
		Tone_Curve_1000_Nit_Sharpened,
		Tone_Curve_1000_Nit,
		Tone_Curve_2000_Nit,
	};
#endif

	// List of supported sink color primaries
	enum OutputColorPrimaries {
		Output_Color_Primaries_Rec709,
		Output_Color_Primaries_DCI,
		Output_Color_Primaries_Rec2020,
	};

	// Output mode from the shader to the device (not TV)
	enum OutputMode {
		Output_Mode_Linear,
		Output_Mode_Gamma_Correct,
		Output_Mode_scRGB,
		Output_Mode_Rec2020_Linear,
		Output_Mode_Rec2020_PQ,
		Output_Mode_Luminance,
	};

	enum ODTCurve
	{
		// reference curves, no parameterization
		ODT_LDR_Ref,
		ODT_1000Nit_Ref,
		ODT_2000Nit_Ref,
		ODT_4000Nit_Ref,

		// Adjustable curves, parameterized for range, level, etc
		ODT_LDR_Adj,
		ODT_1000Nit_Adj,
		ODT_2000Nit_Adj,
		ODT_4000Nit_Adj,

		ODT_Invalid = 0xffffffff
	};

	enum AcesPreset {
		Set_1000NitHDR,
		Set_1000NitHDRSharpened,
		Set_SDR,
		Set_EDR,
		Set_EDRExtreme,
		Set_TheWitness,
		Set_Nvidia,
	};

	struct AcesParameters {
		AcesParameters() :
			selectedCurve(ODT_1000Nit_Adj),
			minStops(-8.0f),
			maxStops(8.0f),
			midGrayScale(1.0f),
			adjustWP(true),
			desaturate(false),
			selectedColorMatrix(Aces::Output_Color_Primaries_Rec709),
			outputMode(Aces::Output_Mode_scRGB),
			maxLevel(-1.0f),
			surroundGamma(1.0f),
			toneCurveSaturation(1.0f),
			dimSurround(true),
			shaper(0) {	}
		float					minStops;
		float					maxStops;
		float					midGrayScale;
		float					maxLevel;
		float					surroundGamma;
		float					toneCurveSaturation;		
		int						shaper;					// Sigmoid shaper for Linear vs PQ shaping
		ODTCurve				selectedCurve;
		OutputColorPrimaries	selectedColorMatrix;
		OutputMode				outputMode;
		bool					adjustWP;
		bool					desaturate;
		bool					dimSurround;
	};

	// Create a linear to ACES look-up table which will be sampled in the tonemapper.
	void generateAcesTonemapLUT(void *data, GLfloat* aces_parameters);
	void rebuildAcesLUT(int preset = Aces::Set_Nvidia);	
	void rebuildAcesLUT(const AcesParameters &parameters);
	void updateAcesUniforms(int program, const int texUnit3D);
	GLfloat* getAcesParams() const { return &mAcesParams[0]; }
	GLuint getAcesLut() const { return mAcesLutTex; }

	// Presets
	void Apply1000nitHDR();
	void Apply1000nitHDRSharpened();
	void ApplySDR();
	void ApplyEDR();
	void ApplyEDRExtreme();
	void ApplyNvidia();

	bool getHDREnabled() const { return hdrEnabled; }
	void setHDREnabled(bool value) { hdrEnabled = value; }


private:	
	AcesParameters mCurrentParameters;
	GLfloat* mAcesParams; // Parameters used in tonemapper
	GLuint mAcesLutTex;
	bool hdrEnabled;
};