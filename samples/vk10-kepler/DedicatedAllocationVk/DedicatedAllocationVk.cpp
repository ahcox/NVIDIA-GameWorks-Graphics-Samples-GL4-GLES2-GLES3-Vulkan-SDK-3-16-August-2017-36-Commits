//----------------------------------------------------------------------------------
// File:        vk10-kepler\DedicatedAllocationVk/DedicatedAllocationVk.cpp
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
#include "DedicatedAllocationVk.h"
#include "NvAppBase/NvInputTransformer.h"
#include "NvAssetLoader/NvAssetLoader.h"
#include "NvUI/NvTweakBar.h"
#include "NV/NvLogs.h"

#define ARRAY_SIZE(a) ( sizeof(a) / sizeof( (a)[0] ))


DedicatedAllocationVk::DedicatedAllocationVk()
	: mPipeline(VK_NULL_HANDLE)
	, mPipelineLayout(VK_NULL_HANDLE)
	, mVertexBuffer()
	, mIndexBuffer()
	, mDrawGeometry(true)
	, mRenderMode(Regular)
	, mTime(0)
	, mSupportsDedicatedAllocation (false)
{
	m_transformer->setTranslationVec(nv::vec3f(0.0f, 0.0f, -2.2f));
	m_transformer->setRotationVec(nv::vec3f(NV_PI*0.35f, 0.0f, 0.0f));

	// Required in all subclasses to avoid silent link issues
	forceLinkHack();
}

DedicatedAllocationVk::~DedicatedAllocationVk()
{
	LOGI("DedicatedAllocationVk: destroyed\n");
}

void DedicatedAllocationVk::configurationCallback(NvVKConfiguration& config)
{
	config.depthBits = 24;
	config.stencilBits = 0;
	config.mainTargetUsageFlags |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;

#if defined (VK_NV_dedicated_allocation)
	config.extensionsToEnable.push_back(VK_NV_DEDICATED_ALLOCATION_EXTENSION_NAME);
#endif

}

void DedicatedAllocationVk::initTile(Tile & tile, uint32_t dimension, nv::vec3f clear_color, bool dedicated = false) {
	VkResult result = VK_ERROR_INITIALIZATION_FAILED;

	tile.clear_color = clear_color;
	tile.dimension = dimension;

	// image
	{
		VkImageCreateInfo imageInfo = { VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
		imageInfo.imageType = VK_IMAGE_TYPE_2D;
		imageInfo.format = mOffScreenTargetFormat;
		imageInfo.extent.width = tile.dimension;
		imageInfo.extent.height = tile.dimension;
		imageInfo.extent.depth = 1;
		imageInfo.mipLevels = 1;
		imageInfo.flags = 0;
		imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
		imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
		imageInfo.usage = VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
		imageInfo.pQueueFamilyIndices = NULL;

		imageInfo.arrayLayers = 1;

		imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

		// setting up VK_NV_dedicated_allocation part 1: image

#if defined (VK_NV_dedicated_allocation)
		VkDedicatedAllocationImageCreateInfoNV dedicatedImageInfo = { VK_STRUCTURE_TYPE_DEDICATED_ALLOCATION_IMAGE_CREATE_INFO_NV };

		if (dedicated)
		{
			dedicatedImageInfo.dedicatedAllocation = VK_TRUE;
			imageInfo.pNext = &dedicatedImageInfo;
		}
#endif

		result = vkCreateImage(device(), &imageInfo, NULL, &tile.image);
		CHECK_VK_RESULT();
	}

	// memory
	{
		VkMemoryAllocateInfo allocateInfo = { VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };

		VkMemoryRequirements memRequirements = { 0 };
		VkPhysicalDeviceMemoryProperties& memoryProperties = vk().physicalDeviceMemoryProperties();

		vkGetImageMemoryRequirements(device(), tile.image, &memRequirements);

		VkFlags memFlags = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
		uint32_t memoryTypeIndex;

		for (memoryTypeIndex = 0; memoryTypeIndex < memoryProperties.memoryTypeCount; ++memoryTypeIndex)
		{
			if (((memoryProperties.memoryTypes[memoryTypeIndex].propertyFlags & memFlags) == memFlags) &&
				((memRequirements.memoryTypeBits >> memoryTypeIndex) & 1))
			{
				break;
			}
		}
		NV_ASSERT(memoryTypeIndex < memoryProperties.memoryTypeCount);

		allocateInfo.allocationSize = memRequirements.size;
		allocateInfo.memoryTypeIndex = memoryTypeIndex;
		
		// setting up VK_NV_dedicated_allocationn part 2: memory
#if defined (VK_NV_dedicated_allocation)

		VkDedicatedAllocationMemoryAllocateInfoNV dedicatedAllocationInfo = { VK_STRUCTURE_TYPE_DEDICATED_ALLOCATION_MEMORY_ALLOCATE_INFO_NV };
		if (dedicated)
		{
			dedicatedAllocationInfo.image = tile.image;
			allocateInfo.pNext = &dedicatedAllocationInfo;
		}

#endif
		result = vkAllocateMemory(device(), &allocateInfo, NULL, &tile.memory);
		CHECK_VK_RESULT();
	}

	// binding image to memory
	{
		result = vkBindImageMemory(device(), tile.image, tile.memory, 0);
		CHECK_VK_RESULT();
	}

	// image view
	{
		VkImageViewCreateInfo imageViewInfo = { VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };

		imageViewInfo.format = mOffScreenTargetFormat;
		imageViewInfo.components.r = VK_COMPONENT_SWIZZLE_R;
		imageViewInfo.components.g = VK_COMPONENT_SWIZZLE_G;
		imageViewInfo.components.b = VK_COMPONENT_SWIZZLE_B;
		imageViewInfo.components.a = VK_COMPONENT_SWIZZLE_A;

		imageViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		imageViewInfo.subresourceRange.baseMipLevel = 0;
		imageViewInfo.subresourceRange.levelCount = 1;
		imageViewInfo.subresourceRange.baseArrayLayer = 0;
		imageViewInfo.subresourceRange.layerCount = 1;
		imageViewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
		imageViewInfo.flags = 0;

		imageViewInfo.image = tile.image;

		result = vkCreateImageView(device(), &imageViewInfo, NULL, &tile.imageView);
		CHECK_VK_RESULT();
	}

	// framebuffer
	{
		VkFramebufferCreateInfo frameBufferInfo = { VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };

		frameBufferInfo.attachmentCount = 1;
		frameBufferInfo.pAttachments = &tile.imageView;
		frameBufferInfo.width = tile.dimension;
		frameBufferInfo.height = tile.dimension;
		frameBufferInfo.layers = 1;
		frameBufferInfo.renderPass = mOffScreenRenderPass;

		result = vkCreateFramebuffer(device(), &frameBufferInfo, NULL, &tile.framebuffer);
		CHECK_VK_RESULT();
	}
}



void DedicatedAllocationVk::initRendering(void) {
	NV_APP_BASE_SHARED_INIT();

	VkResult result = VK_ERROR_INITIALIZATION_FAILED;

	NvAssetLoaderAddSearchPath("vk10-kepler/DedicatedAllocationVk");

#if defined ( VK_NV_dedicated_allocation )
	mSupportsDedicatedAllocation = isExtensionSupported(VK_NV_DEDICATED_ALLOCATION_EXTENSION_NAME);
	if (mSupportsDedicatedAllocation)
	{
		LOGI("VK_NV_dedicated_allocation not supported at run time. Some functionality of this sample will be unavailable.");
	}
#else
	mSupportsDedicatedAllocation = false;
    LOGI("VK_NV_dedicated_allocation not supported at compile time. Please update to a more recent version of Vulkan SDK. Some functionality of this sample will be unavailable.");
#endif

	VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
	pipelineLayoutCreateInfo.setLayoutCount = 0;
	pipelineLayoutCreateInfo.pSetLayouts = NULL;
	result = vkCreatePipelineLayout(device(), &pipelineLayoutCreateInfo, 0, &mPipelineLayout);
	CHECK_VK_RESULT();

	static struct Vertex {
		float   position[2];
		uint8_t color[4];
	} const vertices[3] = {
		{ { -0.5f, -0.5f },{ 0xFF, 0x00, 0x00, 0xFF }, },
		{ { 0.5f, -0.5f },{ 0x00, 0xFF, 0x00, 0xFF }, },
		{ { 0.5f, 0.5f },{ 0x00, 0x00, 0xFF, 0xFF }, },
	};
	static uint32_t indices[3] = { 0, 1, 2 };

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

	mOffScreenTargetFormat = VK_FORMAT_R8G8B8A8_UNORM;
	
	// render pass for off screen tiles
	{
		VkAttachmentDescription attachments[1] = {};
		attachments[0].format = mOffScreenTargetFormat;
		attachments[0].samples = VK_SAMPLE_COUNT_1_BIT;
		attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
		attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
		attachments[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		attachments[0].finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL; // we'll blit from the off screen targets to the main render target
		attachments[0].flags = 0;

		VkSubpassDescription subPass;
		subPass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
		subPass.inputAttachmentCount = 0;
		subPass.pInputAttachments = NULL;

		VkAttachmentReference colorRefs[1] = { { 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL } };
		subPass.colorAttachmentCount = ARRAY_SIZE(colorRefs);
		subPass.pColorAttachments = colorRefs;
		subPass.flags = 0;

		subPass.pDepthStencilAttachment = 0;
		subPass.preserveAttachmentCount = 0;
		subPass.pPreserveAttachments = NULL;
		subPass.pResolveAttachments = NULL;

		VkRenderPassCreateInfo renderPassInfo = { VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO };
		renderPassInfo.pAttachments = attachments;
		renderPassInfo.attachmentCount = ARRAY_SIZE(attachments);
		renderPassInfo.pSubpasses = &subPass;
		renderPassInfo.subpassCount = 1;
		renderPassInfo.pDependencies = NULL;
		renderPassInfo.dependencyCount = 0;

		result = vkCreateRenderPass(device(), &renderPassInfo, NULL, &mOffScreenRenderPass);
		CHECK_VK_RESULT();

	}

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

#ifdef SOURCE_SHADERS
	shaderCount = vk().createShadersFromSourceString(
		NvAssetLoadTextFile("src_shaders/simple.glsl"), shaderStages, 2);
#else
	{
		int32_t length;
		char* data = NvAssetLoaderRead("shaders/simple.nvs", length);
		shaderCount = vk().createShadersFromBinaryBlob((uint32_t*)data,
			length, shaderStages, 2);
	}
#endif

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

	pipelineInfo.renderPass = mOffScreenRenderPass;
	pipelineInfo.subpass = 0;

	pipelineInfo.layout = mPipelineLayout;

	result = vkCreateGraphicsPipelines(device(), VK_NULL_HANDLE, 1, &pipelineInfo, NULL, &mPipeline);
	CHECK_VK_RESULT();

	// create off screen resources and also "draw" into them once in order to clear & transition
	{
		VkCommandBuffer initCmd = vk().beginTempCmdBuffer();

		nv::vec3f clear_color(1.0f, 0.0f, 0.0f);
		uint32_t dimension = TileDimension;

		initTile(mRegularAllocation, dimension, nv::vec3f(164, 30, 34) / 255.0f);
		drawTile(initCmd, mRegularAllocation, true);

		if (mSupportsDedicatedAllocation)
		{
			initTile(mDedicatedAllocation, dimension, nv::vec3f(185, 231, 0) / 255.0f, true);
			drawTile(initCmd, mDedicatedAllocation, true);
		}

		vk().doneWithTempCmdBufferSubmit(initCmd);
	}
}

void DedicatedAllocationVk::destroyTile(Tile & tile)
{
	vkDestroyImage(device(), tile.image, NULL);
	vkDestroyImageView(device(), tile.imageView, NULL);
	vkFreeMemory(device(), tile.memory, NULL);
	vkDestroyFramebuffer(device(), tile.framebuffer, NULL);
}

void DedicatedAllocationVk::shutdownRendering(void) {

	vkDeviceWaitIdle(device());

	// destroy other resources here

	destroyTile(mRegularAllocation);

	if (mSupportsDedicatedAllocation)
		destroyTile(mDedicatedAllocation);

	vkDestroyPipeline(device(), mPipeline, NULL);

	if (mVertexBuffer() != VK_NULL_HANDLE)
		vkDestroyBuffer(device(), mVertexBuffer(), NULL);
	if (mIndexBuffer() != VK_NULL_HANDLE)
		vkDestroyBuffer(device(), mIndexBuffer(), NULL);

	if (mVertexBuffer.mem != VK_NULL_HANDLE)
		vkFreeMemory(device(), mVertexBuffer.mem, NULL);

	if (mIndexBuffer.mem != VK_NULL_HANDLE)
		vkFreeMemory(device(), mIndexBuffer.mem, NULL);
}

void DedicatedAllocationVk::initUI(void) {
	if (mTweakBar) {
		NvTweakVarBase *var;

		NvTweakEnum<uint32_t> renderModes[NumRenderModes] = {
			{ "regular allocations ", Regular},
			{ "dedicated allocations", Dedicated },
		};

		var = mTweakBar->addMenu("Render Mode", mRenderMode, renderModes, mSupportsDedicatedAllocation ? 2 : 1);
		addTweakKeyBind(var, NvKey::K_M);
		addTweakButtonBind(var, NvGamepad::BUTTON_Y);

		var = mTweakBar->addValue("Draw Geometry", mDrawGeometry);
		addTweakKeyBind(var, NvKey::K_D);

		mTweakBar->syncValues();
	}
}

NvUIEventResponse DedicatedAllocationVk::handleReaction(const NvUIReaction &react)
{
	return nvuiEventNotHandled;
}


void DedicatedAllocationVk::reshape(int32_t width, int32_t height)
{

}

// use render pass to clear & transition render target, optionally drawing geometry
void DedicatedAllocationVk::drawTile(VkCommandBuffer cmd, Tile & tile, bool forceClearOnly)
{
	VkRenderPassBeginInfo renderPassBeginInfo = { VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };

	renderPassBeginInfo.renderPass = mOffScreenRenderPass;
	renderPassBeginInfo.framebuffer = tile.framebuffer;
	renderPassBeginInfo.renderArea.offset.x = 0;
	renderPassBeginInfo.renderArea.offset.y = 0;
	renderPassBeginInfo.renderArea.extent.width = tile.dimension;
	renderPassBeginInfo.renderArea.extent.height = tile.dimension;

	float s = forceClearOnly ? 0.0f : (0.6f + 0.4f * sin(5.0f * mTime));

	VkClearValue clearValues[1];
	clearValues[0].color.float32[0] = tile.clear_color[0] * s;
	clearValues[0].color.float32[1] = tile.clear_color[1] * s;
	clearValues[0].color.float32[2] = tile.clear_color[2] * s;
	clearValues[0].color.float32[3] = 1.0f;

	renderPassBeginInfo.pClearValues = clearValues;
	renderPassBeginInfo.clearValueCount = 1;

	{
		vkCmdBeginRenderPass(cmd, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

		if (mDrawGeometry && !forceClearOnly)
		{
			vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, mPipeline);
			VkViewport vp;
			VkRect2D sc;
			vp.x = 0;
			vp.y = 0;
			vp.height = (float)(tile.dimension);
			vp.width = (float)(tile.dimension);
			vp.minDepth = 0.0f;
			vp.maxDepth = 1.0f;

			sc.offset.x = 0;
			sc.offset.y = 0;
			sc.extent.width = vp.width;
			sc.extent.height = vp.height;

			vkCmdSetViewport(cmd, 0, 1, &vp);
			vkCmdSetScissor(cmd, 0, 1, &sc);

			// Bind the vertex and index buffers
			VkDeviceSize offsets[] = { 0 };
			vkCmdBindVertexBuffers(cmd, 0, 1, &mVertexBuffer(), offsets);
			vkCmdBindIndexBuffer(cmd, mIndexBuffer(), 0, VK_INDEX_TYPE_UINT32);

			// Draw the triangle
			vkCmdDrawIndexed(cmd, 3, 1, 0, 0, 0);
		}
		vkCmdEndRenderPass(cmd);
	}
}

void DedicatedAllocationVk::blitTile(Tile & tile, int32_t x, int32_t y)
{
	VkCommandBuffer mainCmd = vk().getMainCommandBuffer();
	
	VkImageBlit region;

	region.srcOffsets[0].x = 0;
	region.srcOffsets[0].y = 0;
	region.srcOffsets[0].z = 0;

	region.srcOffsets[1].x = tile.dimension;
	region.srcOffsets[1].y = tile.dimension;
	region.srcOffsets[1].z = 1;


	region.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	region.srcSubresource.mipLevel = 0;
	region.srcSubresource.baseArrayLayer = 0;
	region.srcSubresource.layerCount = 1;

	region.dstOffsets[0].x = x;
	region.dstOffsets[0].y = y;
	region.dstOffsets[0].z = 0;

	region.dstOffsets[1].x = x + tile.dimension;
	region.dstOffsets[1].y = y + tile.dimension;
	region.dstOffsets[1].z = 1;

	// clamp to be within the destination render target. This will not preserve the aspect ratio, but that's ok
	for (int i = 0; i < 2; ++i)
	{
		region.dstOffsets[i].x = std::max(region.dstOffsets[i].x, 0);
		region.dstOffsets[i].y = std::max(region.dstOffsets[i].y, 0);

		region.dstOffsets[i].x = std::min(region.dstOffsets[i].x, vk().mainRenderTarget()->width());
		region.dstOffsets[i].y = std::min(region.dstOffsets[i].y, vk().mainRenderTarget()->height());
	}

	region.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	region.dstSubresource.mipLevel = 0;
	region.dstSubresource.baseArrayLayer = 0;
	region.dstSubresource.layerCount = 1;

	vkCmdBlitImage(mainCmd, tile.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, vk().mainRenderTarget()->image(), VK_IMAGE_LAYOUT_GENERAL, 1, &region, VK_FILTER_NEAREST);
}


void DedicatedAllocationVk::draw(void)
{
	VkResult result = VK_ERROR_INITIALIZATION_FAILED;

	VkCommandBuffer mainCmd = vk().getMainCommandBuffer();

	NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "draw");

	mTime += getFrameDeltaTime();

	{
		NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "clear main render target");

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

		vkCmdBeginRenderPass(mainCmd, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);
		vkCmdEndRenderPass(mainCmd);
	}

	Tile* tiles[NumRenderModes] = { &mRegularAllocation, &mDedicatedAllocation };

	{
		NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "update tile");
		drawTile(mainCmd, *tiles[mRenderMode]);
	}

	{
		NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "blit tile");

		const int x_offset = 400;
		const int y_offset = 50;

		blitTile(*tiles[mRenderMode], x_offset, y_offset);
	}

	vk().submitMainCommandBuffer();
}

NvAppBase* NvAppFactory() {
	return new DedicatedAllocationVk();
}



