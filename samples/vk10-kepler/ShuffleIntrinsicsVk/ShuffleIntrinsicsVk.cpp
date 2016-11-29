//----------------------------------------------------------------------------------
// File:        vk10-kepler\ShuffleIntrinsicsVk/ShuffleIntrinsicsVk.cpp
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
#include "ShuffleIntrinsicsVk.h"
#include "NvAppBase/NvInputTransformer.h"
#include "NvAssetLoader/NvAssetLoader.h"
#include "NvUI/NvTweakBar.h"
#include "NV/NvLogs.h"

#define ARRAY_SIZE(a) ( sizeof(a) / sizeof( (a)[0] ))

enum {
	ACTION_CHANGE_RENDER_MODE = 1,
};

ShuffleIntrinsicsVk::ShuffleIntrinsicsVk() 
    : mCmdPool(VK_NULL_HANDLE)
	, mCmd(VK_NULL_HANDLE)
	, mVertexBuffer()
	, mIndexBuffer()
	, mCurrentPipeline(0)
	, mSupportsWarpVoteShuffle(false)
{
    m_transformer->setTranslationVec(nv::vec3f(0.0f, 0.0f, -2.2f));
    m_transformer->setRotationVec(nv::vec3f(NV_PI*0.35f, 0.0f, 0.0f));

    // Required in all subclasses to avoid silent link issues
    forceLinkHack();
}

ShuffleIntrinsicsVk::~ShuffleIntrinsicsVk()
{
    LOGI("ShuffleIntrinsicsVk: destroyed\n");
}

void ShuffleIntrinsicsVk::configurationCallback(NvVKConfiguration& config)
{ 
    config.depthBits = 24; 
    config.stencilBits = 0; 

#if defined (VK_NV_glsl_shader)
	config.extensionsToEnable.push_back(VK_NV_GLSL_SHADER_EXTENSION_NAME);
#endif
}

VkPipelineLayout pipelineLayout;

void ShuffleIntrinsicsVk::initRendering(void) {
	NV_APP_BASE_SHARED_INIT();

	VkResult result = VK_ERROR_INITIALIZATION_FAILED;


	NvAssetLoaderAddSearchPath("vk10-kepler/ShuffleIntrinsicsVk");
#if defined (VK_NV_glsl_shader)
	mSupportsWarpVoteShuffle = isExtensionSupported(VK_NV_GLSL_SHADER_EXTENSION_NAME);
#else
	mSupportsWarpVoteShuffle = false;
#endif
	if (!mSupportsWarpVoteShuffle) {
		showDialog("NVIDIA GPU intrinsics Not Supported", "NVIDIA shader intrinsics extensions not supported\n"
			"You will not be able to use the warp vote and warp shuffle\n"
			" intrinsics at runtime.  Only the \"pass through \" rendermode\n"
			"will be available.", false);
	}

	VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
	pipelineLayoutCreateInfo.setLayoutCount = 0;
	pipelineLayoutCreateInfo.pSetLayouts = NULL;
	result = vkCreatePipelineLayout(device(), &pipelineLayoutCreateInfo, 0, &pipelineLayout);
	CHECK_VK_RESULT();
	
    {
        VkCommandPoolCreateInfo cmdPoolInfo = { VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
        cmdPoolInfo.queueFamilyIndex = 0;
        cmdPoolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

        result = vkCreateCommandPool(device(), &cmdPoolInfo, NULL, &mCmdPool);
        CHECK_VK_RESULT();

    }

	struct Vertex {
		float   position[2];
		uint8_t color[4];
	};
	/*
	static const Vertex vertices[3] = {
	{ { -0.5f, -0.5f }, { 0xFF, 0x00, 0x00, 0xFF }, },
	{ { 0.5f, -0.5f }, { 0x00, 0xFF, 0x00, 0xFF }, },
	{ { 0.5f, 0.5f }, { 0x00, 0x00, 0xFF, 0xFF }, },
	};
	*/

	static const Vertex vertices[3] = {
		{ { -0.8f, 0.8f },{ 0xFF, 0x00, 0x00, 0xFF }, },
		{ { 0.8f,  0.8f },{ 0x00, 0xFF, 0x00, 0xFF }, },
		{ { 0.0f, -0.8f },{ 0x00, 0x00, 0xFF, 0xFF }, },
	};

    static uint32_t indices[3] = { 0, 1, 2 };

	// Create the main command buffer.
	VkCommandBufferAllocateInfo cmdInfo = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
	cmdInfo.commandPool = mCmdPool;
	cmdInfo.level = VK_COMMAND_BUFFER_LEVEL_SECONDARY;
	cmdInfo.commandBufferCount = 1;
	result = vkAllocateCommandBuffers(device(), &cmdInfo, &mCmd);
	CHECK_VK_RESULT();

	// Create the vertex buffer
	result = vk().createAndFillBuffer(sizeof vertices,
		VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
		mVertexBuffer, vertices);
	CHECK_VK_RESULT();
    
    // Create the index buffer
	result = vk().createAndFillBuffer(sizeof indices,
		VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
		mIndexBuffer, indices);
    CHECK_VK_RESULT();

	
	
	for (uint32_t mode = 0; mode < ( mSupportsWarpVoteShuffle ? numModes : 1); ++mode)
	{
		// Create static state info for the mPipeline.
		VkVertexInputBindingDescription vertexBindings[1] = {};
		vertexBindings[0].binding = 0;
		vertexBindings[0].stride = sizeof vertices[0];
		vertexBindings[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

		VkVertexInputAttributeDescription attributes[2] = {};
		attributes[0].location = 0;
		attributes[0].binding = 0;
		attributes[0].format = VK_FORMAT_R32G32_SFLOAT;
		attributes[0].offset = offsetof(Vertex, position);
		attributes[1].location = 1;
		attributes[1].binding = 0;
		attributes[1].format = VK_FORMAT_R8G8B8A8_UNORM;
		attributes[1].offset = offsetof(Vertex, color);

		VkPipelineVertexInputStateCreateInfo viStateInfo = { VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
		viStateInfo.vertexBindingDescriptionCount = ARRAY_SIZE(vertexBindings);
		viStateInfo.pVertexBindingDescriptions = vertexBindings;
		viStateInfo.vertexAttributeDescriptionCount = ARRAY_SIZE(attributes);
		viStateInfo.pVertexAttributeDescriptions = attributes;

		VkPipelineInputAssemblyStateCreateInfo iaStateInfo = { VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
		iaStateInfo.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
		iaStateInfo.primitiveRestartEnable = VK_FALSE;

		// set dynamically
		VkPipelineViewportStateCreateInfo vpStateInfo = { VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
		vpStateInfo.pNext = 0;
		vpStateInfo.viewportCount = 1;
		vpStateInfo.scissorCount = 1;

		VkPipelineRasterizationStateCreateInfo rsStateInfo = { VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
		rsStateInfo.depthClampEnable = VK_FALSE;
		rsStateInfo.rasterizerDiscardEnable = VK_FALSE;
		rsStateInfo.polygonMode = VK_POLYGON_MODE_FILL;
		rsStateInfo.cullMode = VK_CULL_MODE_NONE;
		rsStateInfo.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
		rsStateInfo.lineWidth = 1.0f;

		VkPipelineColorBlendAttachmentState attachments[1] = {};
		attachments[0].blendEnable = VK_FALSE;
		attachments[0].colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

		VkPipelineColorBlendStateCreateInfo cbStateInfo = { VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
		cbStateInfo.logicOpEnable = VK_FALSE;
		cbStateInfo.attachmentCount = ARRAY_SIZE(attachments);
		cbStateInfo.pAttachments = attachments;

		VkPipelineMultisampleStateCreateInfo msStateInfo = { VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
		msStateInfo.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
		msStateInfo.alphaToCoverageEnable = VK_FALSE;
		msStateInfo.sampleShadingEnable = VK_FALSE;
		msStateInfo.minSampleShading = 1.0f;
		uint32_t smplMask = 0x1;
		msStateInfo.pSampleMask = &smplMask;

		VkPipelineTessellationStateCreateInfo tessStateInfo = { VK_STRUCTURE_TYPE_PIPELINE_TESSELLATION_STATE_CREATE_INFO };
		tessStateInfo.patchControlPoints = 0;

		VkPipelineDynamicStateCreateInfo dynStateInfo;
		VkDynamicState dynStates[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
		memset(&dynStateInfo, 0, sizeof(dynStateInfo));
		dynStateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
		dynStateInfo.dynamicStateCount = 2;
		dynStateInfo.pDynamicStates = dynStates;

		// Shaders

		VkPipelineShaderStageCreateInfo shaderStages[2];
		uint32_t shaderCount = 0;
		if (mSupportsWarpVoteShuffle) 
		{
			shaderCount = vk().createShadersFromSourceString(NvAssetLoadTextFile("src_shaders/simple.glsl"), shaderStages, 2);
		}
		else
		{
			int32_t length;
			char* data = NvAssetLoaderRead("shaders/simple.nvs", length);
			shaderCount = vk().createShadersFromBinaryBlob((uint32_t*)data,	length, shaderStages, 2);
		}


		VkPipelineDepthStencilStateCreateInfo noDepth;
		noDepth = { VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO };
		noDepth.depthTestEnable = VK_FALSE;
		noDepth.depthWriteEnable = VK_FALSE;
		noDepth.depthCompareOp = VK_COMPARE_OP_ALWAYS;
		noDepth.depthBoundsTestEnable = VK_FALSE;
		noDepth.stencilTestEnable = VK_FALSE;
		noDepth.minDepthBounds = 0.0f;
		noDepth.maxDepthBounds = 1.0f;

		// Create mPipeline state VI-IA-VS-VP-RS-FS-CB
		VkGraphicsPipelineCreateInfo pipelineInfo = { VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
		pipelineInfo.pVertexInputState = &viStateInfo;
		pipelineInfo.pInputAssemblyState = &iaStateInfo;
		pipelineInfo.pViewportState = &vpStateInfo;
		pipelineInfo.pRasterizationState = &rsStateInfo;
		pipelineInfo.pColorBlendState = &cbStateInfo;
		pipelineInfo.pMultisampleState = &msStateInfo;
		pipelineInfo.pTessellationState = &tessStateInfo;
		pipelineInfo.pDynamicState = &dynStateInfo;
		pipelineInfo.pDepthStencilState = &noDepth;

		pipelineInfo.stageCount = shaderCount;
		pipelineInfo.pStages = shaderStages;

		pipelineInfo.renderPass = vk().mainRenderTarget()->clearRenderPass();
		pipelineInfo.subpass = 0;

		pipelineInfo.layout = pipelineLayout;

		VkSpecializationMapEntry fs_mode;

		fs_mode.constantID = 0;
		fs_mode.offset = 0;
		fs_mode.size = 4;

		VkSpecializationInfo fs_specialization;
		fs_specialization.mapEntryCount = 1;
		fs_specialization.pMapEntries = &fs_mode;
		fs_specialization.dataSize = 4;
		fs_specialization.pData = &mode;


		for (uint32_t stage = 0; stage < pipelineInfo.stageCount; ++stage)
		{
			if (shaderStages[stage].stage == VK_SHADER_STAGE_FRAGMENT_BIT)
			{
				shaderStages[stage].pSpecializationInfo = &fs_specialization;
			}
		}

		VkPipeline pipeline = VK_NULL_HANDLE;
		result = vkCreateGraphicsPipelines(device(), VK_NULL_HANDLE, 1, &pipelineInfo, NULL, &pipeline);
		CHECK_VK_RESULT();

		mPipelines.push_back(pipeline);

	}
    
}



void ShuffleIntrinsicsVk::shutdownRendering(void) {

	vkDeviceWaitIdle(device());

	// destroy other resources here
	while (!mPipelines.empty())
	{
		vkDestroyPipeline(device(), mPipelines.back(), NULL);
		mPipelines.pop_back();
	}

    if (mVertexBuffer() != VK_NULL_HANDLE)
		vkDestroyBuffer(device(), mVertexBuffer(), NULL);
    if (mIndexBuffer()  != VK_NULL_HANDLE)
		vkDestroyBuffer(device(), mIndexBuffer(), NULL);

	vkDestroyCommandPool(device(), mCmdPool, NULL);

    if (mVertexBuffer.mem != VK_NULL_HANDLE)
		vkFreeMemory(device(), mVertexBuffer.mem, NULL);

	if (mIndexBuffer.mem != VK_NULL_HANDLE)
		vkFreeMemory(device(), mIndexBuffer.mem, NULL);
}

void ShuffleIntrinsicsVk::initUI(void) {
    if (mTweakBar) {
        NvTweakVarBase *var;

		NvTweakEnum<uint32_t> renderModes[numModes] = {
			{ "pass through color", 0 },
			{ "lane id", 1 },
			{ "tag first lane", 5 },
			{ "tag first + last lane", 6 },
			{ "ratio of active lanes", 4 },
			{ "maximum across warp", 7 },
			{ "minimum across warp", 8 },
			{ "gl_WarpIDNV", 2 },
			{ "gl_SMIDNV", 3 },

		};
		var = mTweakBar->addMenu("Render Mode", mCurrentPipeline, renderModes, std::min(mPipelines.size(),  TWEAKENUM_ARRAYSIZE(renderModes)), ACTION_CHANGE_RENDER_MODE);
		addTweakKeyBind(var, NvKey::K_M);
		addTweakButtonBind(var, NvGamepad::BUTTON_Y);

        mTweakBar->syncValues();
    }
}

NvUIEventResponse ShuffleIntrinsicsVk::handleReaction(const NvUIReaction &react)
{
	switch (react.code) {
	case ACTION_CHANGE_RENDER_MODE:
		updateRenderCommands();
		return nvuiEventHandled;
	}
	return nvuiEventNotHandled;
}


void ShuffleIntrinsicsVk::updateRenderCommands() {
	VkResult result = VK_ERROR_INITIALIZATION_FAILED;

	result = vkDeviceWaitIdle(device());
	CHECK_VK_RESULT();

	VkCommandBufferInheritanceInfo inherit = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_INHERITANCE_INFO };
	inherit.framebuffer = vk().mainRenderTarget()->frameBuffer();
	inherit.renderPass = vk().mainRenderTarget()->clearRenderPass();

	// Record the commands (resets the buffer)
	VkCommandBufferBeginInfo beginInfo = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_RENDER_PASS_CONTINUE_BIT;
	beginInfo.pInheritanceInfo = &inherit;

	result = vkBeginCommandBuffer(mCmd, &beginInfo);
	CHECK_VK_RESULT();

	// Bind the mPipeline state
	vkCmdBindPipeline(mCmd, VK_PIPELINE_BIND_POINT_GRAPHICS, mPipelines[mCurrentPipeline]);

	{
		VkViewport vp;
		VkRect2D sc;
		vp.x = 0;
		vp.y = 0;
		vp.height = (float)(m_height);
		vp.width = (float)(m_width);
		vp.minDepth = 0.0f;
		vp.maxDepth = 1.0f;

		sc.offset.x = 0;
		sc.offset.y = 0;
		sc.extent.width = vp.width;
		sc.extent.height = vp.height;

		vkCmdSetViewport(mCmd, 0, 1, &vp);
		vkCmdSetScissor(mCmd, 0, 1, &sc);

		// Bind the vertex and index buffers
		VkDeviceSize offsets[] = { 0 };
		vkCmdBindVertexBuffers(mCmd, 0, 1, &mVertexBuffer(), offsets);
		vkCmdBindIndexBuffer(mCmd, mIndexBuffer(), 0, VK_INDEX_TYPE_UINT32);

		// Draw the triangle
		vkCmdDrawIndexed(mCmd, 3, 1, 0, 0, 0);

	}

	result = vkEndCommandBuffer(mCmd);
	CHECK_VK_RESULT();
}


void ShuffleIntrinsicsVk::reshape(int32_t width, int32_t height)
{
	updateRenderCommands();
}

void ShuffleIntrinsicsVk::draw(void)
{
	VkResult result = VK_ERROR_INITIALIZATION_FAILED;

	VkCommandBuffer cmd = vk().getMainCommandBuffer();

	VkRenderPassBeginInfo renderPassBeginInfo = { VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };

	renderPassBeginInfo.renderPass = vk().mainRenderTarget()->clearRenderPass();
	renderPassBeginInfo.framebuffer = vk().mainRenderTarget()->frameBuffer();
	renderPassBeginInfo.renderArea.offset.x = 0;
	renderPassBeginInfo.renderArea.offset.y = 0;
	renderPassBeginInfo.renderArea.extent.width = m_width;
	renderPassBeginInfo.renderArea.extent.height = m_height;

	VkClearValue clearValues[2];
	clearValues[0].color.float32[0] = 0.66f;
	clearValues[0].color.float32[1] = 0.33f;
	clearValues[0].color.float32[2] = 0.44f;
	clearValues[0].color.float32[3] = 1.0f;
	clearValues[1].depthStencil.depth = 1.0f;
	clearValues[1].depthStencil.stencil = 0;


	renderPassBeginInfo.pClearValues = clearValues;
	renderPassBeginInfo.clearValueCount = 2;

	vkCmdBeginRenderPass(cmd, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_SECONDARY_COMMAND_BUFFERS);

	vkCmdExecuteCommands(cmd, 1, &mCmd);

	vkCmdEndRenderPass(cmd);
	

	vk().submitMainCommandBuffer();
}

NvAppBase* NvAppFactory() {
    return new ShuffleIntrinsicsVk();
}



