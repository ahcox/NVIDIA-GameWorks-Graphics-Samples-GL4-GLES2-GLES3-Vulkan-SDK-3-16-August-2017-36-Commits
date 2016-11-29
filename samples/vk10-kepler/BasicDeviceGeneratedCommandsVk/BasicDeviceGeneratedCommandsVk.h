//----------------------------------------------------------------------------------
// File:        vk10-kepler\BasicDeviceGeneratedCommandsVk/BasicDeviceGeneratedCommandsVk.h
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
#include "NvVkUtil/NvSampleAppVK.h"
#include "NvVkUtil/NvSimpleUBO.h"
#include "NV/NvMath.h"
#define USEVULKANSDK 1
#include "vk_nvx_device_generated_commands.h"


class NvModelVK;
class NvQuadVK;
class SimpleCommandBuffer;

class BasicDeviceGeneratedCommandsVk : public NvSampleAppVK
{
public:
	BasicDeviceGeneratedCommandsVk();
	~BasicDeviceGeneratedCommandsVk();

	void initRendering(void);
	void shutdownRendering(void);
	void initUI(void);
	void draw(void);
	void reshape(int32_t width, int32_t height);

	void configurationCallback(NvVKConfiguration& config);

	NvUIEventResponse handleReaction(const NvUIReaction &react);

private:

	NvQuadVK* mQuad;
	NvVkTexture mCubeMap;

	typedef struct {
		nv::matrix4f mModelViewMatrix;
		nv::matrix4f mProjectionMatrix;
		nv::matrix4f mInvModelViewMatrix;
		nv::matrix4f mInvProjectionMatrix;
		float mModelLight[3];
	} UniformBlock;

	typedef struct {
		nv::vec4f mAmbient;
		nv::vec4f mDiffuse;
	} MaterialBlock;

	enum
	{
		FillPoint,
		FillLine,
		FillSolid,
		NumDrawModes
	};

	enum DrawMode
	{
		DrawIndexed,
		DrawIndirect,
		NumDrawModesIndirect,
		DeviceGeneratedDrawIndirect = NumDrawModesIndirect,
		DeviceGeneratedPsoDrawIndirect,
		DeviceGeneratedVboIboPsoDrawIndirect,
		NumDrawModesDeviceGenerated
	};

	struct Model
	{
		VkPipeline pipelines[NumDrawModes];
		
		uint32_t pipelineObjectTableIndices[NumDrawModes];
		uint32_t vboObjectTableIndex;
		uint32_t iboObjectTableIndex;
		
		std::string name;
		NvModelVK* model;
	};

	std::vector<Model> models;
	uint32_t mCurrentModel;

	NvSimpleUBO<UniformBlock> mUBO;

	uint32_t mMatBlockStride;
	NvVkBuffer mMatBlockBuffer;

	VkDescriptorSetLayout mDescriptorSetLayout;
	VkDescriptorSet mDescriptorSet;

	VkPipelineLayout mPipelineLayout;

	DrawMode mDrawMode;
	uint32_t mRenderModeOverride[2];
	uint32_t meshSplitRatio;

	VkPipeline mQuadPipeline;

	enum
	{
		MeshSplitRange = 100,
		NumDrawIndirectCommands = 2
	};

	// for core draw and indirect draw mode
	NvVkBuffer	drawIndirectBuffer;

	// for VK_NVX_device_generated_commands
	bool mSupportsDeviceGeneratedCommands;

#if defined (VK_NVX_device_generated_commands)

	VkCommandPool generatedCmdPool;

	VkObjectTableNVX deviceGeneratedObjectTable;
	
	VkIndirectCommandsLayoutNVX deviceGeneratedLayoutDrawIndirect;
	VkCommandBuffer	generatedCmdBuffersDrawIndirect[2];

	VkIndirectCommandsLayoutNVX deviceGeneratedLayoutPsoDrawIndirect;
	VkCommandBuffer	generatedCmdBufferPsoDrawIndirect;
	NvVkBuffer	deviceGeneratedPsoBuffer;

	VkIndirectCommandsLayoutNVX deviceGeneratedLayoutVboIboPsoDrawIndirect;
	VkCommandBuffer	generatedCmdBufferVboIboPsoDrawIndirect;
	NvVkBuffer	deviceGeneratedVboIboBuffer;

#endif

};
