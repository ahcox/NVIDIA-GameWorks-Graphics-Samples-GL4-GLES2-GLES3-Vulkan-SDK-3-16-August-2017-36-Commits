//----------------------------------------------------------------------------------
// File:        es3aep-kepler\VisualizeHDR/VisualizeHDR.h
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
#ifndef VISUALIZE_HDR_H
#define VISUALIZE_HDR_H

#include "NvAppBase/gl/NvSampleAppGL.h"

#include "KHR/khrplatform.h"
#include "NvGamepad/NvGamepad.h"
#include "ShaderBuffer.h"

#define CPP 1
#include "NV/NvMath.h"
#include "NV/NvShaderMappings.h"
#include "assets/shaders/uniforms.h"
// For HDR
#include "aces.h"

class NvFramerateCounter;
class NvGLSLProgram;
class HDRImage;

class VisualizeHDR : public NvSampleAppGL
{
public:
	VisualizeHDR();
	~VisualizeHDR();

	void initRendering(void);
	void initUI(void);
	void draw(void);
	void reshape(int32_t width, int32_t height);

	void configurationCallback(NvGLConfiguration& config);
	void loadHDRImage(const std::string& filename);
	void updateUITweaks();

protected:
	void drawScreenAlignedQuad(const GLuint program);

private:
	ShaderParams mShaderParams;
	NvGLSLProgram* mVisualizationProg;

	GLuint mUBO;
	GLuint mVBO;

	ShaderBuffer<nv::vec4f>* mScreenQuadPos;
		
	NvStopWatch* mTimer;	// Timer to use in shader
	GLfloat* mSettings;		// Window, game engine settings to use in shader
	GLfloat* mParams;		// Other parameters to use in shader	
	uint32_t mSplitscreen;	// Splitscreen modes
	float mStep;			// Step size for movements		

	GLuint mHDRTexture;
	HDRImage* mHDRImage;
	std::vector<std::string> mImageStrings;		// Filenames of all HDR image files
	uint32_t mUIImageChoice;					// Choice of Image to view in UI
	uint32_t mCurrentImageChoice;				// Image file currently loaded

	bool mRebuildAcesLUT;     // Value is set if user has tweaked parameters in UI

	uint32_t mUIAcesColorSpaceChoice;
	float mUIAcesMidGray;
	uint32_t mUIAcesCurve;
	bool mUIAcesAdjustWP;
	bool mUIAcesDesaturate;
	uint32_t mUIAcesOutputMode;
	bool mUIAcesDimSurround;
	float mUIAcesToneCurveSaturation;
	float mUIAcesSurroundGamma;
	float mUIMaxLevel;

	// TODO: Convert to singleton or use static functions -shaveenk
	Aces acesManager;
	Aces::AcesParameters mUserAcerParameters;

};

#endif // VISUALIZE_HDR_H