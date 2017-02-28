//----------------------------------------------------------------------------------
// File:        es3aep-kepler\VisualizeHDR/VisualizeHDR.cpp
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
#include "VisualizeHDR.h"
#include "NvAppBase/NvFramerateCounter.h"
#include "NvAppBase/NvInputTransformer.h"
#include "NvAssetLoader/NvAssetLoader.h"
#include "NvGLUtils/NvGLSLProgram.h"
#include "NvGLUtils/NvImageGL.h"
#include "NvUI/NvTweakBar.h"
#include "NV/NvLogs.h"
#include <string>
#include "HDRImages.h"

extern std::string loadShaderSourceWithUniformTag(const char* uniformsFile, const char* srcFile);

VisualizeHDR::VisualizeHDR()
: 
//mReset(true),
mStep(1.0f),
//mMacBeth(true),
mSplitscreen(0),
mUIImageChoice(0),
mScreenQuadPos(NULL),
mRebuildAcesLUT(false),
mHDRTexture(0U),
mHDRImage(nullptr)
{
	m_transformer->setTranslationVec(nv::vec3f(0.0f, 0.0f, -3.0f));	

	// Settings to pass to shader
	mSettings = new GLfloat[4];		

	// Parameters to pass to shader
	mParams = new GLfloat[4];		

	// Create time data to pass to shaders
	mTimer = createStopWatch();

	// Required in all subclasses to avoid silent link issues
	forceLinkHack();
}

VisualizeHDR::~VisualizeHDR()
{
	glDeleteTextures(1, &mHDRTexture);

	if (mHDRImage)
		delete mHDRImage;

	if (mScreenQuadPos) {
		delete mScreenQuadPos;
	}

	if (mParams) {
		delete[] mParams;
	}

	if (mSettings) {
		delete[] mSettings;
	}	

	// Delete timer
	if (mTimer) {
		mTimer->stop();
		delete mTimer;
	}

	LOGI("VisualizeHDR: destroyed\n");
}

void VisualizeHDR::loadHDRImage(const std::string& filename)  {
	if (mHDRImage != nullptr)
		delete mHDRImage;

	mHDRImage = new HDRImage;
	if (!mHDRImage->loadHDRIFromFile(filename.c_str())) {
		fprintf(stderr, "Error loading image file '%s'\n", mHDRImage);
		exit(-1);
	}
	int32_t w = mHDRImage->getWidth();
	int32_t h = mHDRImage->getHeight();
	hfloat *out = new hfloat[w * h * 3];

	FP32toFP16((float*)mHDRImage->getLevel(0), out, w, h);

	glBindTexture(GL_TEXTURE_2D, mHDRTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w, h, 0, GL_RGB, GL_HALF_FLOAT, out);
	glBindTexture(GL_TEXTURE_2D, 0);

	delete (out);
	CHECK_GL_ERROR();
}


void VisualizeHDR::configurationCallback(NvGLConfiguration& config)
{
	config.depthBits = 24;
	config.stencilBits = 0;
	config.apiVer = NvGLAPIVersionES3();
}

void VisualizeHDR::initRendering(void) {
	NV_APP_BASE_SHARED_INIT();

	// Disable FPS counter
	setFPSEnable(false);

	mSettings[0] = mContext->width();		// Window width
	mSettings[1] = mContext->height();		// Window height
	mSettings[2] = 0.0f;					// Time in ms
	mSettings[3] = 1.0f;					// Unused

	mParams[0] = 0.0f;						// MacBeth Color Picker toggle 0.0 or 1.0
	mParams[1] = 0.0f;						// Splitscreen Toggle
	mParams[2] = 0.0f;						// Unused
	mParams[3] = 0.0f;						// Unused

	mImageStrings.push_back("textures/kite1.hdr");
	mImageStrings.push_back("textures/kite2.hdr");
	mImageStrings.push_back("textures/Infiltrator1.hdr");
	mImageStrings.push_back("textures/Infiltrator2.hdr");
	mImageStrings.push_back("textures/SunTemple.hdr");

	mUIAcesColorSpaceChoice = (int)mUserAcerParameters.selectedColorMatrix;
	mUIAcesCurve = (int)mUserAcerParameters.selectedCurve;
	mUIAcesMidGray = mUserAcerParameters.midGrayScale;
	mUIAcesAdjustWP = mUserAcerParameters.adjustWP;
	mUIAcesDesaturate = mUserAcerParameters.desaturate;
	mUIAcesOutputMode = (uint32_t)mUserAcerParameters.outputMode;
	mUIAcesDimSurround = mUserAcerParameters.dimSurround;
	mUIAcesToneCurveSaturation = mUserAcerParameters.toneCurveSaturation;
	mUIAcesSurroundGamma = mUserAcerParameters.surroundGamma;
	mUIMaxLevel = mUserAcerParameters.maxLevel;

	// Start timer
	mTimer->start();

	// OpenGL 4.3 is the minimum for compute shaders
	if (!requireMinAPIVersion(NvGLAPIVersionES3_1()))
		return;

	// Set Clear Color
	glClearColor(0.25f, 0.25f, 0.25f, 1.0f);

	CHECK_GL_ERROR();
	
	NvAssetLoaderAddSearchPath("es3aep-kepler/VisualizeHDR");

	const char* shaderPrefix =
		(getGLContext()->getConfiguration().apiVer.api == NvGLAPI::GL)
		? "#version 430\n" : "#version 310 es\n";

	CHECK_GL_ERROR();
	{
		// Initialize Quad Render Program
		NvScopedShaderPrefix switched(shaderPrefix);

		std::string visualizationVS = loadShaderSourceWithUniformTag("shaders/uniforms.h", "shaders/renderQuadVS.glsl");
		std::string visualizationFS = loadShaderSourceWithUniformTag("shaders/uniforms.h", "shaders/renderQuadFS.glsl");
		mVisualizationProg = new NvGLSLProgram;

		NvGLSLProgram::ShaderSourceItem sourcesQ[2];
		sourcesQ[0].type = GL_VERTEX_SHADER;
		sourcesQ[0].src = visualizationVS.c_str();
		sourcesQ[1].type = GL_FRAGMENT_SHADER;
		sourcesQ[1].src = visualizationFS.c_str();
		mVisualizationProg->setSourceFromStrings(sourcesQ, 2);
	}
	CHECK_GL_ERROR();

	// Using Radiance HDR images
	glGenTextures(1, &mHDRTexture);

	glBindTexture(GL_TEXTURE_2D, mHDRTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glBindTexture(GL_TEXTURE_2D, 0);
	CHECK_GL_ERROR();

	// Loading HDR Image
	loadHDRImage(mImageStrings[mUIImageChoice]);	

	// Initialize skybox for screen quad
	mScreenQuadPos = new ShaderBuffer<nv::vec4f>(4);
	nv::vec4f* pos = mScreenQuadPos->map();
	pos[0] = nv::vec4f(-1.0f, -1.0f, -1.0f, 1.0f);
	pos[1] = nv::vec4f(1.0f, -1.0f, -1.0f, 1.0f);
	pos[2] = nv::vec4f(-1.0f, 1.0f, -1.0f, 1.0f);
	pos[3] = nv::vec4f(1.0f, 1.0f, -1.0f, 1.0f);
	mScreenQuadPos->unmap();


	//create ubo and initialize it with the structure data
	glGenBuffers(1, &mUBO);
	glBindBuffer(GL_UNIFORM_BUFFER, mUBO);
	glBufferData(GL_UNIFORM_BUFFER, sizeof(ShaderParams), &mShaderParams, GL_STREAM_DRAW);
	CHECK_GL_ERROR();

	//create simple single-vertex VBO
	float vtx_data[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	glGenBuffers(1, &mVBO);
	glBindBuffer(GL_ARRAY_BUFFER, mVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vtx_data), vtx_data, GL_STATIC_DRAW);
	CHECK_GL_ERROR();

	//Set clockwise winding
	glFrontFace(GL_CW);

	// Texture
	const int screen_width = getAppContext()->width();
	const int screen_height = getAppContext()->height();

	acesManager.rebuildAcesLUT();

	acesManager.setHDREnabled(getHDREnable());
}

void VisualizeHDR::initUI(void) {
	if (mTweakBar) {

		mTweakBar->setCompactLayout(true);
		mTweakBar->SetNumRows(30);
		mTweakBar->addPadding();
		mTweakBar->addPadding();
		NvTweakEnum<uint32_t> uIImageChoiceVals[5];
		uIImageChoiceVals[0].m_name = "Kite Demo 1";
		uIImageChoiceVals[0].m_value = 0;
		uIImageChoiceVals[1].m_name = "Kite Demo 2";
		uIImageChoiceVals[1].m_value = 1;
		uIImageChoiceVals[2].m_name = "Infiltrator 1";
		uIImageChoiceVals[2].m_value = 2;
		uIImageChoiceVals[3].m_name = "Infiltrator 2";
		uIImageChoiceVals[3].m_value = 3;
		uIImageChoiceVals[4].m_name = "Sun Temple";
		uIImageChoiceVals[4].m_value = 4;
		mTweakBar->addEnum("Choose Image", mUIImageChoice, uIImageChoiceVals, 5);

#if defined(ANDROID) || defined(DEBUG)

#if defined(ANDROID)
		if (getHDREnable()) {
#endif
			mTweakBar->addPadding();
			// Aces Parameters
			mTweakBar->addLabel("Aces Parameters", true);		
		
			NvTweakEnum<uint32_t> uICurveChoiceVals[4];
			uICurveChoiceVals[0].m_name = "LDR - 48 nits";
			uICurveChoiceVals[0].m_value = (uint32_t)Aces::ODT_LDR_Adj;
			uICurveChoiceVals[1].m_name = "1000 nits";
			uICurveChoiceVals[1].m_value = (uint32_t)Aces::ODT_1000Nit_Adj;
			uICurveChoiceVals[2].m_name = "2000 nits";
			uICurveChoiceVals[2].m_value = (uint32_t)Aces::ODT_2000Nit_Adj;
			uICurveChoiceVals[3].m_name = "4000 nits";
			uICurveChoiceVals[3].m_value = (uint32_t)Aces::ODT_4000Nit_Adj;
			mTweakBar->addEnum("ODT Curve (Adjusted)", mUIAcesCurve, uICurveChoiceVals, 4);

			NvTweakEnum<uint32_t> uIAcesColorSpaces[3];
			uIAcesColorSpaces[0].m_name = "BT.709";
			uIAcesColorSpaces[0].m_value = (uint32_t)Aces::Output_Color_Primaries_Rec709;
			uIAcesColorSpaces[1].m_name = "DCI-P3";
			uIAcesColorSpaces[1].m_value = (uint32_t)Aces::Output_Color_Primaries_DCI;
			uIAcesColorSpaces[2].m_name = "BT.2020";
			uIAcesColorSpaces[2].m_value = (uint32_t)Aces::Output_Color_Primaries_Rec2020;
			mTweakBar->addEnum("Output Color Space", mUIAcesColorSpaceChoice, uIAcesColorSpaces, 3);

			NvTweakEnum<uint32_t> uIAcesOutputMode[6];
			uIAcesOutputMode[0].m_name = "Linear";
			uIAcesOutputMode[0].m_value = (uint32_t)Aces::Output_Mode_Linear;
			uIAcesOutputMode[1].m_name = "Gamma Corrected";
			uIAcesOutputMode[1].m_value = (uint32_t)Aces::Output_Mode_Gamma_Correct;
			uIAcesOutputMode[2].m_name = "BT.2020";
			uIAcesOutputMode[2].m_value = (uint32_t)Aces::Output_Mode_Rec2020_Linear;
			uIAcesOutputMode[3].m_name = "BT.2020 + PQ";
			uIAcesOutputMode[3].m_value = (uint32_t)Aces::Output_Mode_Rec2020_PQ;
			uIAcesOutputMode[4].m_name = "scRGB [NVIDIA SHIELD]";
			uIAcesOutputMode[4].m_value = (uint32_t)Aces::Output_Mode_scRGB;
			uIAcesOutputMode[5].m_name = "Luminance Visualization";
			uIAcesOutputMode[5].m_value = (uint32_t)Aces::Output_Mode_Luminance;
			mTweakBar->addEnum("Output Mode", mUIAcesOutputMode, uIAcesOutputMode, 6);

			mTweakBar->addValue("D60 to D65 White Adjustment", mUIAcesAdjustWP);

			mTweakBar->addValue("Desaturate", mUIAcesDesaturate);

			mTweakBar->addValue("Dim Surround", mUIAcesDimSurround);

			mTweakBar->addValue("Middle Gray", mUIAcesMidGray, 0.1f, 2.0f, 0.1f);
		
			mTweakBar->addValue("Tone Curve Saturation", mUIAcesToneCurveSaturation, 0.1f, 2.0f, 0.1f);

			mTweakBar->addValue("Surround Gamma", mUIAcesSurroundGamma, 0.1f, 2.0f, 0.1f);

			mTweakBar->addValue("Max CLL", mUIMaxLevel, -1.0f, 4000.0f, 10.0f);
		
			mTweakBar->addPadding();
			mTweakBar->addLabel("Viewing Modes", true);
			NvTweakEnum<uint32_t> splitScreenVals[4];
			splitScreenVals[0].m_name = "HDR";
			splitScreenVals[0].m_value = 0;
			splitScreenVals[1].m_name = "LDR/HDR";
			splitScreenVals[1].m_value = 1;
			splitScreenVals[2].m_name = "HDR/LDR";
			splitScreenVals[2].m_value = 2;
			splitScreenVals[3].m_name = "LDR";
			splitScreenVals[3].m_value = 3;
			mTweakBar->addEnum("Splitscreen", mSplitscreen, splitScreenVals, 4);
#if defined(ANDROID)
		}
#endif
#endif
	}
}

void VisualizeHDR::reshape(int32_t width, int32_t height)
{
	glViewport(0, 0, (GLint)width, (GLint)height);
	CHECK_GL_ERROR();
}

void VisualizeHDR::drawScreenAlignedQuad(const GLuint program)
{

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, mScreenQuadPos->getBuffer());

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void VisualizeHDR::updateUITweaks() {
	// Load new image if needed
	if (mCurrentImageChoice != mUIImageChoice) {
		loadHDRImage(mImageStrings[mUIImageChoice]);
		mCurrentImageChoice = mUIImageChoice;
	}

	// Update Aces Parameters
	if ((uint32_t)mUserAcerParameters.selectedColorMatrix != mUIAcesColorSpaceChoice){
		mUserAcerParameters.selectedColorMatrix = (Aces::OutputColorPrimaries)mUIAcesColorSpaceChoice;
		acesManager.rebuildAcesLUT(mUserAcerParameters);
	}

	if (mUserAcerParameters.midGrayScale != mUIAcesMidGray) {
		mUserAcerParameters.midGrayScale = mUIAcesMidGray;
		acesManager.rebuildAcesLUT(mUserAcerParameters);
	}

	if (mUserAcerParameters.selectedCurve != mUIAcesCurve) {
		mUserAcerParameters.selectedCurve = (Aces::ODTCurve)mUIAcesCurve;
		acesManager.rebuildAcesLUT(mUserAcerParameters);
	}

	if (mUserAcerParameters.adjustWP != mUIAcesAdjustWP) {
		mUserAcerParameters.adjustWP = mUIAcesAdjustWP;
		acesManager.rebuildAcesLUT(mUserAcerParameters);
	}

	if (mUserAcerParameters.desaturate != mUIAcesDesaturate) {
		mUserAcerParameters.desaturate = mUIAcesDesaturate;
		acesManager.rebuildAcesLUT(mUserAcerParameters);
	}

	if ((uint32_t)mUserAcerParameters.outputMode != mUIAcesOutputMode) {
		mUserAcerParameters.outputMode = (Aces::OutputMode)mUIAcesOutputMode;
		acesManager.rebuildAcesLUT(mUserAcerParameters);
	}
	if (mUserAcerParameters.dimSurround != mUIAcesDimSurround) {
		mUserAcerParameters.dimSurround = mUIAcesDimSurround;
		acesManager.rebuildAcesLUT(mUserAcerParameters);
	}
	if (mUserAcerParameters.toneCurveSaturation != mUIAcesToneCurveSaturation) {
		mUserAcerParameters.toneCurveSaturation = mUIAcesToneCurveSaturation;
		acesManager.rebuildAcesLUT(mUserAcerParameters);
	}
	if (mUserAcerParameters.surroundGamma != mUIAcesSurroundGamma) {
		mUserAcerParameters.surroundGamma = mUIAcesSurroundGamma;
		acesManager.rebuildAcesLUT(mUserAcerParameters);
	}
	if (mUserAcerParameters.maxLevel != mUIMaxLevel) {
			mUserAcerParameters.maxLevel = mUIMaxLevel;
		acesManager.rebuildAcesLUT(mUserAcerParameters);
	}
}

void VisualizeHDR::draw(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Read new values from UI and update parameters as necessary
	updateUITweaks();

	//
	// Compute matrices without the legacy matrix stack support
	//
	nv::matrix4f projectionMatrix;
	nv::perspective(projectionMatrix, 45.0f * 2.0f*3.14159f / 360.0f, (float)m_width / (float)m_height, 0.1f, 10.0f);

	nv::matrix4f viewMatrix = m_transformer->getModelViewMat();

	//
	// update struct representing UBO
	//
	mShaderParams.ModelView = viewMatrix;
	mShaderParams.InvViewMatrix = inverse(viewMatrix);
	mShaderParams.ModelViewProjection = projectionMatrix * viewMatrix;
	mShaderParams.ProjectionMatrix = projectionMatrix;

	// bind the buffer for the UBO, and update it with the latest values from the CPU-side struct
	glBindBufferBase(GL_UNIFORM_BUFFER, 1, mUBO);
	glBindBuffer(GL_UNIFORM_BUFFER, mUBO);
	glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(ShaderParams), &mShaderParams);


	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Draw visualization into FP16 Framebuffer
	{
		mVisualizationProg->enable();
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		GLfloat settings[4] = { float( mContext->width()), float(mContext->height()), mTimer->getTime(), 0.0f };
		GLuint loc = glGetUniformLocation(mVisualizationProg->getProgram(), "uSettings");
		mSettings[0] = mContext->width();
		mSettings[1] = mContext->height();
		mSettings[2] = 1.0f;
		
		// Unused
		mSettings[3] = 0.0f;		

		mParams[0] = 0.0;

		mParams[1] = (float)mSplitscreen;

		glProgramUniform4fv(mVisualizationProg->getProgram(), loc, 1, &mSettings[0]);
		loc = glGetUniformLocation(mVisualizationProg->getProgram(), "uParams");
		glProgramUniform4fv(mVisualizationProg->getProgram(), loc, 1, &mParams[0]);

		const int texUnit = 1;
		acesManager.updateAcesUniforms(mVisualizationProg->getProgram(), texUnit);		
		
		// Binding buffer with HDR image		
		loc = glGetUniformLocation(mVisualizationProg->getProgram(), "uTexture");
		glProgramUniform1i(mVisualizationProg->getProgram(), loc, 0);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, mHDRTexture);
		
		drawScreenAlignedQuad(mVisualizationProg->getProgram());
		mVisualizationProg->disable();
	}
}


// A replacement for the ARB_shader_include extension, which is not widely supported
// This loads a "header" file (uniformsFile) and a "source" file (srcFile).  It scans
// the source file for "#UNIFORMS" and replaces this tag with the contents of the
// uniforms file.  Limited, but functional for the cases used here
std::string loadShaderSourceWithUniformTag(const char* uniformsFile, const char* srcFile) {
	int32_t len;

	char *uniformsStr = NvAssetLoaderRead(uniformsFile, len);
	if (!uniformsStr)
		return "";

	char *srcStr = NvAssetLoaderRead(srcFile, len);
	if (!srcStr)
		return "";

	std::string dest = "";

	const char* uniformTag = "#UNIFORMS";

	char* uniformTagStart = strstr(srcStr, uniformTag);

	if (uniformTagStart) {
		// NULL the start of the tag
		*uniformTagStart = 0;
		dest += srcStr; // source up to tag
		dest += "\n";
		dest += uniformsStr;
		dest += "\n";
		char* uniformTagEnd = uniformTagStart + strlen(uniformTag);
		dest += uniformTagEnd;
	}
	else {
		dest += srcStr;
	}

	NvAssetLoaderFree(uniformsStr);
	NvAssetLoaderFree(srcStr);

	return dest;
}

NvAppBase* NvAppFactory() {
	NvAppBase* app = new VisualizeHDR();
	app->setHDRReady(true);
	return app;
}