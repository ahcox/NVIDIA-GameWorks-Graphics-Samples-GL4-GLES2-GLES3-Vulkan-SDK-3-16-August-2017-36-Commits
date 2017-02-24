//----------------------------------------------------------------------------------
// File:        vk10-kepler\BasicDeviceGeneratedCommandsVk/BasicDeviceGeneratedCommandsVk.cpp
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
#include "BasicDeviceGeneratedCommandsVk.h"
#include "NvAppBase/NvInputTransformer.h"
#include "NvAssetLoader/NvAssetLoader.h"
#include "NvModel/NvModelExt.h"
#include "NvModel/NvModel.h"
#include "NvVkUtil/NvModelVK.h"
#include "NvVkUtil/NvModelExtVK.h"
#include "NvVkUtil/NvQuadVK.h"
#include "NvUI/NvTweakBar.h"
#include "NV/NvLogs.h"


#define ARRAY_SIZE(a) ( sizeof(a) / sizeof( (a)[0] ))

using namespace Nv;


// Simple loader implementation for NvModelExt
class ModelTestVkLoader : public NvModelFileLoader
{
public:
	ModelTestVkLoader() {}
	virtual ~ModelTestVkLoader() {}
	virtual char* LoadDataFromFile(const char* fileName)
	{
		int32_t length;
		return NvAssetLoaderRead(fileName, length);
	}

	virtual void ReleaseData(char* pData)
	{
		NvAssetLoaderFree(pData);
	}
};


BasicDeviceGeneratedCommandsVk::BasicDeviceGeneratedCommandsVk()
	: mCurrentModel(0)
	, mDrawMode(DrawIndexed)
	, meshSplitRatio (MeshSplitRange / 3)
	, mSupportsDeviceGeneratedCommands (false)
{
	mRenderModeOverride[0] = FillSolid;
	mRenderModeOverride[1] = FillLine;

	m_transformer->setTranslationVec(nv::vec3f(0.0f, 0.0f, -3.0f));
	m_transformer->setRotationVec(nv::vec3f(NV_PI*0.35f, 0.0f, 0.0f));

	// Required in all subclasses to avoid silent link issues
	forceLinkHack();
}

BasicDeviceGeneratedCommandsVk::~BasicDeviceGeneratedCommandsVk()
{
	LOGI("BasicDeviceGeneratedCommandsVk: destroyed\n");
}

void BasicDeviceGeneratedCommandsVk::configurationCallback(NvVKConfiguration& config)
{
	config.depthBits = 24;
	config.stencilBits = 0;
	config.mainTargetUsageFlags |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;

#if defined (VK_NVX_device_generated_commands)
	config.extensionsToEnable.push_back(VK_NVX_DEVICE_GENERATED_COMMANDS_EXTENSION_NAME);
#endif

}

void BasicDeviceGeneratedCommandsVk::initRendering(void) {
	NV_APP_BASE_SHARED_INIT();

	VkResult result;
	NvAssetLoaderAddSearchPath("vk10-kepler/BasicDeviceGeneratedCommandsVk");


#if defined ( VK_NVX_device_generated_commands )
	mSupportsDeviceGeneratedCommands = isExtensionSupported(VK_NVX_DEVICE_GENERATED_COMMANDS_EXTENSION_NAME);
	if (mSupportsDeviceGeneratedCommands)
	{
		load_VK_NVX_device_generated_commands(vk().instance(), vkGetInstanceProcAddr);
	}
	else
	{
		LOGI("VK_NVX_device_generated_commands not supported at run time. Some functionality of this sample will be unavailable.");
	}
#else
	mSupportsDeviceGeneratedCommands = false;
	LOGI("VK_NVX_device_generated_commands not supported at compile time. Please update to a more recent version of Vulkan SDK. Some functionality of this sample will be unavailable.");
#endif

	/////////////////
	// Setup textures
	NvVkTexture tex;
	
	vk().uploadTextureFromDDSFile("textures/sky_cube.dds", tex);
	
	// Create the sampler
	VkSamplerCreateInfo samplerCreateInfo = { VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
	samplerCreateInfo.magFilter = VK_FILTER_LINEAR;
	samplerCreateInfo.minFilter = VK_FILTER_LINEAR;
	samplerCreateInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
	samplerCreateInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
	samplerCreateInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
	samplerCreateInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
	samplerCreateInfo.mipLodBias = 0.0;
	samplerCreateInfo.maxAnisotropy = 1;
	samplerCreateInfo.compareOp = VK_COMPARE_OP_NEVER;
	samplerCreateInfo.minLod = 0.0;
	samplerCreateInfo.maxLod = 16.0;
	samplerCreateInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;

	VkSampler sampler;
	result = vkCreateSampler(device(), &samplerCreateInfo, 0, &sampler);
	CHECK_VK_RESULT();

	//////////////
	// Load Models

	const char* nvmNames[2] = {
		"models/dragon.nvm",
		"models/cow.nvm",
	};

	for (uint32_t i = 0; i < ARRAY_SIZE(nvmNames); i++) {

		Model m;
		int32_t length;
		char *modelData = NvAssetLoaderRead(nvmNames[i], length);
		if (modelData) {
	
			m.model = NvModelVK::CreateFromPreprocessed(vk(), (uint8_t *)modelData);
			NvAssetLoaderFree(modelData);

			m.name = nvmNames[i];
			models.push_back(m);

			LOGI("model %s index count %u ",  m.name.c_str(), m.model->getIndexCount());
		}
	}

	uint32_t matCount = 1;

	mMatBlockStride = sizeof(MaterialBlock);
	mMatBlockStride = (mMatBlockStride + 255) & ~255;
	uint32_t matBlockSize = matCount * mMatBlockStride;

	result = vk().createAndFillBuffer(matBlockSize,
		VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT, mMatBlockBuffer);
	CHECK_VK_RESULT();

	uint8_t* matPtr;
	result = vkMapMemory(device(), mMatBlockBuffer.mem, 0, matBlockSize, 0, (void**)&matPtr);
	CHECK_VK_RESULT();

	MaterialBlock& matBlock = *((MaterialBlock*)(matPtr));

	matBlock.mAmbient = nv::vec4f(0.2f, 0.2f, 0.2f, 1.0f);
	matBlock.mDiffuse = nv::vec4f(0.7f, 0.2f, 0.2f, 1.0f);

	
	// Create descriptor layout to match the shader resources
	VkDescriptorSetLayoutBinding binding[3];
	binding[0].binding = 0;
	binding[0].descriptorCount = 1;
	binding[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
	binding[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC;
	binding[0].pImmutableSamplers = NULL;
	binding[1].binding = 1;
	binding[1].descriptorCount = 1;
	binding[1].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
	binding[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC;
	binding[1].pImmutableSamplers = NULL;
	binding[2].binding = 2;
	binding[2].descriptorCount = 1;
	binding[2].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
	binding[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
	binding[2].pImmutableSamplers = NULL;

	VkDescriptorSetLayoutCreateInfo descriptorSetEntry = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
	descriptorSetEntry.bindingCount = 3;
	descriptorSetEntry.pBindings = binding;

	result = vkCreateDescriptorSetLayout(device(), &descriptorSetEntry, 0, &mDescriptorSetLayout);
	CHECK_VK_RESULT();

	// Create descriptor region and set
	VkDescriptorPoolSize descriptorPoolInfo[2];

	descriptorPoolInfo[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC;
	descriptorPoolInfo[0].descriptorCount = 3;
	descriptorPoolInfo[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
	descriptorPoolInfo[1].descriptorCount = 1;

	VkDescriptorPoolCreateInfo descriptorRegionInfo = { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
	descriptorRegionInfo.maxSets = 4;
	descriptorRegionInfo.poolSizeCount = 2;
	descriptorRegionInfo.pPoolSizes = descriptorPoolInfo;
	VkDescriptorPool descriptorRegion;
	result = vkCreateDescriptorPool(device(), &descriptorRegionInfo, NULL, &descriptorRegion);
	CHECK_VK_RESULT();

	VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
	descriptorSetAllocateInfo.descriptorPool = descriptorRegion;
	descriptorSetAllocateInfo.descriptorSetCount = 1;
	descriptorSetAllocateInfo.pSetLayouts = &mDescriptorSetLayout;
	result = vkAllocateDescriptorSets(device(), &descriptorSetAllocateInfo, &mDescriptorSet);
	CHECK_VK_RESULT();


	VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
	pipelineLayoutCreateInfo.setLayoutCount = 1;
	pipelineLayoutCreateInfo.pSetLayouts = &mDescriptorSetLayout;
	result = vkCreatePipelineLayout(device(), &pipelineLayoutCreateInfo, 0, &mPipelineLayout);
	CHECK_VK_RESULT();

	// Create static state info for the mPipeline.

	// set dynamically
	VkPipelineViewportStateCreateInfo vpStateInfo = { VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
	vpStateInfo.pNext = 0;
	vpStateInfo.viewportCount = 1;
	vpStateInfo.scissorCount = 1;

	VkPipelineRasterizationStateCreateInfo rsStateInfoSolid = { VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
	rsStateInfoSolid.depthClampEnable = VK_FALSE;
	rsStateInfoSolid.rasterizerDiscardEnable = VK_FALSE;
	rsStateInfoSolid.polygonMode = VK_POLYGON_MODE_FILL;
	rsStateInfoSolid.cullMode = VK_CULL_MODE_NONE;
	rsStateInfoSolid.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
	rsStateInfoSolid.lineWidth = 1.0f;

	VkPipelineRasterizationStateCreateInfo rsStateInfoWireFrame = { VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
	rsStateInfoWireFrame.depthClampEnable = VK_FALSE;
	rsStateInfoWireFrame.rasterizerDiscardEnable = VK_FALSE;
	rsStateInfoWireFrame.polygonMode = VK_POLYGON_MODE_LINE;
	rsStateInfoWireFrame.cullMode = VK_CULL_MODE_NONE;
	rsStateInfoWireFrame.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
	rsStateInfoWireFrame.lineWidth = 1.0f;

	VkPipelineRasterizationStateCreateInfo rsStateInfoPoint = { VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
	rsStateInfoPoint.depthClampEnable = VK_FALSE;
	rsStateInfoPoint.rasterizerDiscardEnable = VK_FALSE;
	rsStateInfoPoint.polygonMode = VK_POLYGON_MODE_POINT;
	rsStateInfoPoint.cullMode = VK_CULL_MODE_NONE;
	rsStateInfoPoint.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
	rsStateInfoPoint.lineWidth = 1.0f;


	VkPipelineColorBlendAttachmentState attachments[1] = {};
	attachments[0].blendEnable = VK_FALSE;
	attachments[0].colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

	VkPipelineColorBlendStateCreateInfo cbStateInfo = { VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
	cbStateInfo.logicOpEnable = VK_FALSE;
	cbStateInfo.attachmentCount = ARRAY_SIZE(attachments);
	cbStateInfo.pAttachments = attachments;

	VkPipelineDepthStencilStateCreateInfo dsStateInfo = { VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO };
	dsStateInfo.depthTestEnable = VK_TRUE;
	dsStateInfo.depthWriteEnable = VK_TRUE;
	dsStateInfo.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
	dsStateInfo.depthBoundsTestEnable = VK_FALSE;
	dsStateInfo.stencilTestEnable = VK_FALSE;
	dsStateInfo.minDepthBounds = 0.0f;
	dsStateInfo.maxDepthBounds = 1.0f;

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
	{
		int32_t length;
		char* data = NvAssetLoaderRead("shaders/base_model.nvs", length);
		shaderCount = vk().createShadersFromBinaryBlob((uint32_t*)data,
			length, shaderStages, 2);
	}

	for (uint32_t i = 0; i < models.size(); i++) {
		// Create mPipeline state VI-IA-VS-VP-RS-FS-CB
		VkGraphicsPipelineCreateInfo pipelineInfo = { VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
		
		// shared state
		pipelineInfo.pVertexInputState =   &models[i].model->getVIInfo();
		pipelineInfo.pInputAssemblyState = &models[i].model->getIAInfo();
		pipelineInfo.pViewportState = &vpStateInfo;
		
		pipelineInfo.pColorBlendState = &cbStateInfo;
		pipelineInfo.pDepthStencilState = &dsStateInfo;
		pipelineInfo.pMultisampleState = &msStateInfo;
		pipelineInfo.pTessellationState = &tessStateInfo;
		pipelineInfo.pDynamicState = &dynStateInfo;

		pipelineInfo.stageCount = shaderCount;
		pipelineInfo.pStages = shaderStages;

		pipelineInfo.renderPass = vk().mainRenderTarget()->clearRenderPass();
		pipelineInfo.subpass = 0;

		pipelineInfo.layout = mPipelineLayout;

		// point
		pipelineInfo.pRasterizationState = &rsStateInfoPoint;
		result = vkCreateGraphicsPipelines(device(), VK_NULL_HANDLE, 1, &pipelineInfo, NULL, &models[i].pipelines[FillPoint]);

		// wireframe
		pipelineInfo.pRasterizationState = &rsStateInfoWireFrame;
		result = vkCreateGraphicsPipelines(device(), VK_NULL_HANDLE, 1, &pipelineInfo, NULL, &models[i].pipelines[FillLine]);

		// solid 
		pipelineInfo.pRasterizationState = &rsStateInfoSolid;
		result = vkCreateGraphicsPipelines(device(), VK_NULL_HANDLE, 1, &pipelineInfo, NULL, &models[i].pipelines[FillSolid]);

	
		CHECK_VK_RESULT();
	}
	
	mUBO.Initialize(vk());

	m_transformer->update(0.1f);

	mUBO->mModelViewMatrix = m_transformer->getModelViewMat();

	nv::perspectiveVk(mUBO->mProjectionMatrix, 45.0f * (NV_PI / 180.0f), 16.0f / 9.0f, 1.0f, 100.0f);

	mUBO->mInvProjectionMatrix = nv::inverse(mUBO->mProjectionMatrix);

	mUBO.Update();

	mQuad = NvQuadVK::Create(vk());

	// Shaders
	{
		int32_t length;
		char* data = NvAssetLoaderRead("shaders/cube_map.nvs", length);
		uint32_t shaderCount = vk().createShadersFromBinaryBlob((uint32_t*)data,
			length, shaderStages, 2);
	}

	// Create mPipeline state VI-IA-VS-VP-RS-FS-CB
	VkGraphicsPipelineCreateInfo pipelineInfo = { VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
	// Assuming that all sub-meshes in an ModelExt have the same layout...
	pipelineInfo.pVertexInputState = &mQuad->getVIInfo();
	pipelineInfo.pInputAssemblyState = &mQuad->getIAInfo();
	pipelineInfo.pViewportState = &vpStateInfo;
	pipelineInfo.pRasterizationState = &rsStateInfoSolid;
	pipelineInfo.pColorBlendState = &cbStateInfo;
	pipelineInfo.pDepthStencilState = &dsStateInfo;
	pipelineInfo.pMultisampleState = &msStateInfo;
	pipelineInfo.pTessellationState = &tessStateInfo;
	pipelineInfo.pDynamicState = &dynStateInfo;

	pipelineInfo.stageCount = shaderCount;
	pipelineInfo.pStages = shaderStages;

	pipelineInfo.renderPass = vk().mainRenderTarget()->clearRenderPass();
	pipelineInfo.subpass = 0;

	pipelineInfo.layout = mPipelineLayout;

	result = vkCreateGraphicsPipelines(device(), VK_NULL_HANDLE, 1, &pipelineInfo, NULL,
		&mQuadPipeline);
	CHECK_VK_RESULT();

	VkDescriptorBufferInfo uboDescriptorInfo[2] = {};
	mUBO.GetDesc(uboDescriptorInfo[0]);

	uboDescriptorInfo[1].buffer = mMatBlockBuffer();
	uboDescriptorInfo[1].offset = 0;
	uboDescriptorInfo[1].range = mMatBlockStride;

	VkDescriptorImageInfo texture = {};
	texture.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	texture.imageView = tex.view;
	texture.sampler = sampler;

	VkWriteDescriptorSet writeDescriptorSets[3];
	writeDescriptorSets[0] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
	writeDescriptorSets[0].dstSet = mDescriptorSet;
	writeDescriptorSets[0].dstBinding = 0;
	writeDescriptorSets[0].dstArrayElement = 0;
	writeDescriptorSets[0].descriptorCount = 1;
	writeDescriptorSets[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC;
	writeDescriptorSets[0].pBufferInfo = uboDescriptorInfo;
	writeDescriptorSets[1] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
	writeDescriptorSets[1].dstSet = mDescriptorSet;
	writeDescriptorSets[1].dstBinding = 1;
	writeDescriptorSets[1].dstArrayElement = 0;
	writeDescriptorSets[1].descriptorCount = 1;
	writeDescriptorSets[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC;
	writeDescriptorSets[1].pBufferInfo = uboDescriptorInfo+1;
	writeDescriptorSets[2] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
	writeDescriptorSets[2].dstSet = mDescriptorSet;
	writeDescriptorSets[2].dstBinding = 2;
	writeDescriptorSets[2].dstArrayElement = 0;
	writeDescriptorSets[2].descriptorCount = 1;
	writeDescriptorSets[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
	writeDescriptorSets[2].pImageInfo = &texture;
	vkUpdateDescriptorSets(device(), 3, writeDescriptorSets, 0, 0);


	// buffer to hold the arguments for the indirect draws
	// used by VK core and the VK_NVX_device_generated_commands modes
	{
		result = vk().createAndFillBuffer(NumDrawIndirectCommands * sizeof(VkDrawIndexedIndirectCommand), VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, drawIndirectBuffer);
		CHECK_VK_RESULT();
	}

	// device generated commands 
#if defined (VK_NVX_device_generated_commands)
	if(mSupportsDeviceGeneratedCommands)
	{
		VkCommandBuffer initCmd = vk().beginTempCmdBuffer();

		// create the command layouts for DeviceGeneratedDrawIndirect
		{
			VkIndirectCommandsLayoutCreateInfoNVX commandsLayoutInfo = {/* remove cast when proper header*/ (VkStructureType)VK_STRUCTURE_TYPE_INDIRECT_COMMANDS_LAYOUT_CREATE_INFO_NVX };

			VkIndirectCommandsLayoutTokenNVX commandTokenLayouts[1];

			commandTokenLayouts[0].tokenType = VK_INDIRECT_COMMANDS_TOKEN_DRAW_INDEXED_NVX;
			commandTokenLayouts[0].divisor = 1;
			commandTokenLayouts[0].bindingUnit  = 0; /* unused for VK_INDIRECT_COMMANDS_TOKEN_DRAW_INDEXED_NVX*/
			commandTokenLayouts[0].dynamicCount = 0; /* unused for VK_INDIRECT_COMMANDS_TOKEN_DRAW_INDEXED_NVX*/

			commandsLayoutInfo.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
			commandsLayoutInfo.pTokens = commandTokenLayouts;
			commandsLayoutInfo.tokenCount = ARRAY_SIZE(commandTokenLayouts);

			result = vkCreateIndirectCommandsLayoutNVX(device(), &commandsLayoutInfo, NULL, &deviceGeneratedLayoutDrawIndirect);
			CHECK_VK_RESULT();
		}

		// create the command layouts for DeviceGeneratedPsoDrawIndirect
		{
			VkIndirectCommandsLayoutCreateInfoNVX commandsLayoutInfo = {/* remove cast when proper header*/ (VkStructureType)VK_STRUCTURE_TYPE_INDIRECT_COMMANDS_LAYOUT_CREATE_INFO_NVX };

			VkIndirectCommandsLayoutTokenNVX commandTokenLayouts[2];

			commandTokenLayouts[0].tokenType = VK_INDIRECT_COMMANDS_TOKEN_PIPELINE_NVX;
			commandTokenLayouts[0].divisor = 1;
			commandTokenLayouts[0].bindingUnit = 0; /* unused for VK_INDIRECT_COMMANDS_TOKEN_PIPELINE_NVX*/
			commandTokenLayouts[0].dynamicCount = 0; /* unused for VK_INDIRECT_COMMANDS_TOKEN_PIPELINE_NVX*/

			commandTokenLayouts[1].tokenType = VK_INDIRECT_COMMANDS_TOKEN_DRAW_INDEXED_NVX;
			commandTokenLayouts[1].divisor = 1;
			commandTokenLayouts[1].bindingUnit = 0; /* unused for VK_INDIRECT_COMMANDS_TOKEN_DRAW_INDEXED_NVX*/
			commandTokenLayouts[1].dynamicCount = 0; /* unused for VK_INDIRECT_COMMANDS_TOKEN_DRAW_INDEXED_NVX*/

			commandsLayoutInfo.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
			commandsLayoutInfo.pTokens = commandTokenLayouts;
			commandsLayoutInfo.tokenCount = ARRAY_SIZE(commandTokenLayouts);

			result = vkCreateIndirectCommandsLayoutNVX(device(), &commandsLayoutInfo, NULL, &deviceGeneratedLayoutPsoDrawIndirect);
			CHECK_VK_RESULT();
		}
		
		// create the command layouts for DeviceGeneratedVboIboPsoDrawIndirect
		{
			VkIndirectCommandsLayoutCreateInfoNVX commandsLayoutInfo = {/* remove cast when proper header*/ (VkStructureType)VK_STRUCTURE_TYPE_INDIRECT_COMMANDS_LAYOUT_CREATE_INFO_NVX };

			VkIndirectCommandsLayoutTokenNVX commandTokenLayouts[4];
			commandTokenLayouts[0].tokenType = VK_INDIRECT_COMMANDS_TOKEN_VERTEX_BUFFER_NVX;
			commandTokenLayouts[0].divisor = 1;
			commandTokenLayouts[0].bindingUnit = 0; 
			commandTokenLayouts[0].dynamicCount = 0;

			commandTokenLayouts[1].tokenType = VK_INDIRECT_COMMANDS_TOKEN_INDEX_BUFFER_NVX;
			commandTokenLayouts[1].divisor = 1;
			commandTokenLayouts[1].bindingUnit = 0; 
			commandTokenLayouts[1].dynamicCount = 0; 

			commandTokenLayouts[2].tokenType = VK_INDIRECT_COMMANDS_TOKEN_PIPELINE_NVX;
			commandTokenLayouts[2].divisor = 1;
			commandTokenLayouts[2].bindingUnit = 0; /* unused for VK_INDIRECT_COMMANDS_TOKEN_PIPELINE_NVX*/
			commandTokenLayouts[2].dynamicCount = 0; /* unused for VK_INDIRECT_COMMANDS_TOKEN_PIPELINE_NVX*/

			commandTokenLayouts[3].tokenType = VK_INDIRECT_COMMANDS_TOKEN_DRAW_INDEXED_NVX;
			commandTokenLayouts[3].divisor = 1;
			commandTokenLayouts[3].bindingUnit = 0; /* unused for VK_INDIRECT_COMMANDS_TOKEN_DRAW_INDEXED_NVX*/
			commandTokenLayouts[3].dynamicCount = 0; /* unused for VK_INDIRECT_COMMANDS_TOKEN_DRAW_INDEXED_NVX*/

			commandsLayoutInfo.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
			commandsLayoutInfo.pTokens = commandTokenLayouts;
			commandsLayoutInfo.tokenCount = ARRAY_SIZE(commandTokenLayouts);

			result = vkCreateIndirectCommandsLayoutNVX(device(), &commandsLayoutInfo, NULL, &deviceGeneratedLayoutVboIboPsoDrawIndirect);
			CHECK_VK_RESULT();
		}

		// buffer with PSO indices for DeviceGeneratedPsoDrawIndirect
		{
			result = vk().createAndFillBuffer(NumDrawIndirectCommands * sizeof(uint32_t),
				VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, deviceGeneratedPsoBuffer);
			CHECK_VK_RESULT();
		}

		// buffer with VBO and IBO indices for DeviceGeneratedVboIboPsoDrawIndirect
		{
			result = vk().createAndFillBuffer(NumDrawIndirectCommands * 2 * sizeof(uint32_t), VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, deviceGeneratedVboIboBuffer);
			CHECK_VK_RESULT();
		}

		// create object table register VBOS, IBOS and PSOs for DeviceGeneratedPsoDrawIndirect and DeviceGeneratedVboIboPsoDrawIndirect
		{
			VkObjectTableCreateInfoNVX objectTableInfo = {/* remove cast when proper header*/ (VkStructureType)VK_STRUCTURE_TYPE_OBJECT_TABLE_CREATE_INFO_NVX };

			std::vector<VkObjectEntryTypeNVX> objectTableEntries;
			std::vector<uint32_t> objectTableEntryCounts;
			std::vector<VkObjectEntryUsageFlagsNVX> objectTableEntryFlags;
			
			uint32_t numPSOs = models.size() * NumDrawModes;

			uint32_t numvVBOsIBOS = models.size() ;
			
			LOGI("registering %u PSOs to object table", numPSOs);
			LOGI("registering %u VBOs & IBOs to object table", numvVBOsIBOS);

			objectTableEntries.push_back(VK_OBJECT_ENTRY_PIPELINE_NVX);
			objectTableEntryCounts.push_back(numPSOs);
			objectTableEntryFlags.push_back(VK_OBJECT_ENTRY_USAGE_GRAPHICS_BIT_NVX);

			objectTableEntries.push_back(VK_OBJECT_ENTRY_INDEX_BUFFER_NVX);
			objectTableEntryCounts.push_back(numvVBOsIBOS);
			objectTableEntryFlags.push_back(VK_OBJECT_ENTRY_USAGE_GRAPHICS_BIT_NVX);

			objectTableEntries.push_back(VK_OBJECT_ENTRY_VERTEX_BUFFER_NVX);
			objectTableEntryCounts.push_back(numvVBOsIBOS);
			objectTableEntryFlags.push_back(VK_OBJECT_ENTRY_USAGE_GRAPHICS_BIT_NVX);
			objectTableInfo.objectCount = objectTableEntries.size();

			objectTableInfo.pObjectEntryTypes = objectTableEntries.data();
			objectTableInfo.pObjectEntryCounts = objectTableEntryCounts.data();
			objectTableInfo.pObjectEntryUsageFlags = objectTableEntryFlags.data();

			objectTableInfo.maxUniformBuffersPerDescriptor = 0;
			objectTableInfo.maxStorageBuffersPerDescriptor = 0;
			objectTableInfo.maxStorageImagesPerDescriptor = 0;
			objectTableInfo.maxSampledImagesPerDescriptor = 0;
			objectTableInfo.maxPipelineLayouts = 1; // all PSOs share the same pipelinelayout

			result = vkCreateObjectTableNVX(device(), &objectTableInfo, NULL, &deviceGeneratedObjectTable);
			CHECK_VK_RESULT();
		}

		// register the objects for DeviceGeneratedPsoDrawIndirect and DeviceGeneratedVboIboPsoDrawIndirect
		{
			uint32_t availablePSOEntry = 0;
			uint32_t availableVBOIBOEntry = 0;

			for (uint32_t i = 0; i < models.size(); i++) {
				VkObjectTableEntryNVX* entryArg;

				// register each model's VBO and IBO
				{
					VkBuffer vbo = models[i].model->VBO().buffer;
					VkBuffer ibo = models[i].model->IBO().buffer;

					models[i].vboObjectTableIndex = availableVBOIBOEntry;
					models[i].iboObjectTableIndex = availableVBOIBOEntry;

					LOGI("    registering VBO %p and IBO %p to object table entry %u", vbo, ibo, availableVBOIBOEntry);

					VkObjectTableIndexBufferEntryNVX iboEntry  = { VK_OBJECT_ENTRY_INDEX_BUFFER_NVX, VK_OBJECT_ENTRY_USAGE_GRAPHICS_BIT_NVX };
					iboEntry.buffer = ibo;
					iboEntry.indexType = models[i].model->getIndexType();
					iboEntry.flags = 0;
					
					entryArg = reinterpret_cast<VkObjectTableEntryNVX*> (&iboEntry);
					result = vkRegisterObjectsNVX(device(), deviceGeneratedObjectTable,
						1, &entryArg, &models[i].iboObjectTableIndex
					);

					CHECK_VK_RESULT();

					VkObjectTableVertexBufferEntryNVX vboEntry{ VK_OBJECT_ENTRY_VERTEX_BUFFER_NVX, VK_OBJECT_ENTRY_USAGE_GRAPHICS_BIT_NVX };
					vboEntry.buffer = vbo;
					vboEntry.flags = 0;
					
					entryArg = reinterpret_cast<VkObjectTableEntryNVX*> (&vboEntry);
					result = vkRegisterObjectsNVX(device(), deviceGeneratedObjectTable,
						1, &entryArg, &models[i].vboObjectTableIndex
					);

					CHECK_VK_RESULT();
					
					++availableVBOIBOEntry;
				}

				// register all PSOs
				for (uint32_t m = 0; m < NumDrawModes; m++) {

					LOGI("    registering PSO %p to object table entry %u", models[i].pipelines[m], availablePSOEntry);

					VkPipeline pipeline = models[i].pipelines[m];
					models[i].pipelineObjectTableIndices[m] = availablePSOEntry;

					VkObjectTablePipelineEntryNVX psoEntry{ VK_OBJECT_ENTRY_PIPELINE_NVX };
					psoEntry.pipeline = pipeline;

					entryArg = reinterpret_cast<VkObjectTableEntryNVX*> (&psoEntry);
					result = vkRegisterObjectsNVX(device(), deviceGeneratedObjectTable,
						1, &entryArg, &models[i].pipelineObjectTableIndices[m]
					);
					CHECK_VK_RESULT();
					++availablePSOEntry;
				}
			}
		}
		vk().doneWithTempCmdBufferSubmit(initCmd);

		// create secondary command buffers for DeviceGeneratedDraw*
		{
			{
				VkCommandPoolCreateInfo poolInfo = { VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
				poolInfo.queueFamilyIndex = 0;
				poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

				vkCreateCommandPool(device(), &poolInfo, NULL, &generatedCmdPool);
			}

			VkCommandBufferAllocateInfo allocateInfo = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };

			allocateInfo.commandPool = generatedCmdPool;
			allocateInfo.level = VK_COMMAND_BUFFER_LEVEL_SECONDARY;

			// DeviceGenerateDrawIndirect
			allocateInfo.commandBufferCount = 2;
			result = vkAllocateCommandBuffers(device(), &allocateInfo, generatedCmdBuffersDrawIndirect);
			CHECK_VK_RESULT();

			// DeviceGeneratedPsoDrawIndirect
			allocateInfo.commandBufferCount = 1;
			result = vkAllocateCommandBuffers(device(), &allocateInfo, &generatedCmdBufferPsoDrawIndirect);
			CHECK_VK_RESULT();

			// DeviceGeneratedVboIboPsoDrawIndirect
			allocateInfo.commandBufferCount = 1;
			result = vkAllocateCommandBuffers(device(), &allocateInfo, &generatedCmdBufferVboIboPsoDrawIndirect);
			CHECK_VK_RESULT();
		}
	}
#endif


	result = vkQueueWaitIdle(queue());
	CHECK_VK_RESULT();



}

void BasicDeviceGeneratedCommandsVk::shutdownRendering(void) {

	// destroy other resources here
	
}

void BasicDeviceGeneratedCommandsVk::initUI(void) {

	if (mTweakBar) {
		NvTweakVarBase *var;
	
		std::vector<NvTweakEnum<uint32_t>>	modelUiItems;
		modelUiItems.resize(models.size());

		for (size_t i = 0; i < models.size(); ++i)
		{
			modelUiItems[i].m_name = models[i].name.c_str();
			modelUiItems[i].m_value = i ;
		}

		var = mTweakBar->addEnum("Model ", mCurrentModel, modelUiItems.data(), modelUiItems.size());
		addTweakKeyBind(var, NvKey::K_M);
		addTweakButtonBind(var, NvGamepad::BUTTON_Y);

		static NvTweakEnum<uint32_t> renderModes[NumDrawModes] =
		{
			{ "points", FillPoint },
			{ "lines" , FillLine },
			{ "solid" , FillSolid }
		};

		var = mTweakBar->addEnum("Render mode override 0", mRenderModeOverride[0],renderModes, NumDrawModes);
		addTweakKeyBind(var, NvKey::K_Q);
		addTweakButtonBind(var, NvGamepad::BUTTON_X);

		var = mTweakBar->addEnum("Render mode override 1", mRenderModeOverride[1], renderModes, NumDrawModes);
		addTweakKeyBind(var, NvKey::K_W);

		static NvTweakEnum<uint32_t> drawmodes[NumDrawModesDeviceGenerated] =
		{
			{ "core: DrawIndexed", DrawIndexed },
			{ "core: DrawIndirect" , DrawIndirect },
			{ "NVX_DGC: draw" , DeviceGeneratedDrawIndirect },
			{ "NVX_DGC: PSO+draw" , DeviceGeneratedPsoDrawIndirect },
			{ "NVX_DGC: VBO+IBO+PSO+draw" , DeviceGeneratedVboIboPsoDrawIndirect },
		};

		var = mTweakBar->addEnum("Draw mode override", (uint32_t&) mDrawMode, drawmodes, mSupportsDeviceGeneratedCommands ? NumDrawModesDeviceGenerated : NumDrawModesIndirect);
		addTweakKeyBind(var, NvKey::K_R);
		
		var = mTweakBar->addValue("split ratio", meshSplitRatio, 0, MeshSplitRange);
		addTweakKeyBind(var, NvKey::K_KP_ADD, NvKey::K_KP_SUBTRACT);

		mTweakBar->syncValues();
	}
}

void BasicDeviceGeneratedCommandsVk::reshape(int32_t width, int32_t height)
{
}

void BasicDeviceGeneratedCommandsVk::draw(void)
{
	// Single model case. Model->World transform is identity, so use transformer's matrix directly
	mUBO->mModelViewMatrix = m_transformer->getModelViewMat();
	mUBO->mInvModelViewMatrix = nv::inverse(mUBO->mModelViewMatrix);

	nv::vec4f cameraLight(0.57f, 0.57f, 0.57f, 0.0f);

	nv::vec4f eyeLight = normalize(cameraLight * m_transformer->getRotationMat());

	mUBO->mModelLight[0] = eyeLight.x;
	mUBO->mModelLight[1] = eyeLight.y;
	mUBO->mModelLight[2] = eyeLight.z;

	mUBO.Update();

	// Can't bake these clears into the system at init time, as the 
	// screen/targets can be resized.
	int32_t width = getAppContext()->width();
	int32_t height = getAppContext()->height();

	VkResult result = VK_ERROR_INITIALIZATION_FAILED;

	VkCommandBuffer mainCmd = vk().getMainCommandBuffer();

	NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "draw ");

	VkRenderPassBeginInfo renderPassBeginInfo = { VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };

	renderPassBeginInfo.renderPass = vk().mainRenderTarget()->clearRenderPass();
	renderPassBeginInfo.framebuffer = vk().mainRenderTarget()->frameBuffer();
	renderPassBeginInfo.renderArea.offset.x = 0;
	renderPassBeginInfo.renderArea.offset.y = 0;
	renderPassBeginInfo.renderArea.extent.width = m_width;
	renderPassBeginInfo.renderArea.extent.height = m_height;

	VkClearValue clearValues[2];
	clearValues[0].color.float32[0] = 0.33f;
	clearValues[0].color.float32[1] = 0.44f;
	clearValues[0].color.float32[2] = 0.66f;
	clearValues[0].color.float32[3] = 1.0f;
	clearValues[1].depthStencil.depth = 1.0f;
	clearValues[1].depthStencil.stencil = 0;

	renderPassBeginInfo.pClearValues = clearValues;
	renderPassBeginInfo.clearValueCount = 2;

	NvModelVK & model = *models[mCurrentModel].model;

	VkDrawIndexedIndirectCommand drawIndirectArgs[NumDrawIndirectCommands] = { 0 };

	// compute indirect draw arguments
	// will also be uses as arguments for regular draw calls
	{
		const float r = meshSplitRatio / float(MeshSplitRange);
		const int32_t triangleCount = model.getIndexCount() / 3;
		const int32_t splitTriangleCount = triangleCount *r;
		const int32_t splitIndex = splitTriangleCount * 3;
		const int32_t remaindingIndices = 3 * triangleCount - splitIndex;

		// subset of mesh to be rendered with the first PSO
		drawIndirectArgs[0].firstIndex = 0;
		drawIndirectArgs[0].firstInstance = 0;
		drawIndirectArgs[0].indexCount = splitIndex;
		drawIndirectArgs[0].vertexOffset = 0;
		drawIndirectArgs[0].instanceCount = 1;

		// subset of the mesh rendered with the second PSO
		drawIndirectArgs[1].firstIndex = splitIndex;
		drawIndirectArgs[1].firstInstance = 0;
		drawIndirectArgs[1].indexCount = remaindingIndices;
		drawIndirectArgs[1].vertexOffset = 0;
		drawIndirectArgs[1].instanceCount = 1;
	}

	// used by:
	//   DrawIndirect
	{
		NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "UpdateDrawIndirectBuffer");

		vkCmdUpdateBuffer(mainCmd, drawIndirectBuffer.buffer, 0, NumDrawIndirectCommands * sizeof(drawIndirectArgs), &drawIndirectArgs);

		VkBufferMemoryBarrier bufferBarrier = { VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER };

		bufferBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		bufferBarrier.dstAccessMask = VK_ACCESS_INDIRECT_COMMAND_READ_BIT;

		bufferBarrier.buffer = drawIndirectBuffer.buffer;
		bufferBarrier.offset = 0;
		bufferBarrier.size = VK_WHOLE_SIZE;

		vkCmdPipelineBarrier(mainCmd,
			VK_PIPELINE_STAGE_TRANSFER_BIT,
			VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT,
			0,
			0, VK_NULL_HANDLE,
			1, &bufferBarrier,
			0, VK_NULL_HANDLE
		);
	}

	
	// needs to be outside of render pass
#if defined (VK_NVX_device_generated_commands)

	std::vector<VkBufferMemoryBarrier> commandProcessToExecuteBarriers;

	if(mSupportsDeviceGeneratedCommands)
	{
		NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "UpdateDeviceGenertedBuffer");

		// update the GPU buffers for the device generated commands
		// record barriers

		std::vector<VkBufferMemoryBarrier> transferToCommandProcessBarriers;

		// barrier templates, will be "instanced" with actual buffers
		VkBufferMemoryBarrier transferToCommandProcess = { VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER };
		transferToCommandProcess.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		transferToCommandProcess.dstAccessMask = VK_ACCESS_COMMAND_PROCESS_READ_BIT_NVX;
		transferToCommandProcess.offset = 0;
		transferToCommandProcess.size = VK_WHOLE_SIZE;

		VkBufferMemoryBarrier commandProcessToExecute = { VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER };
		commandProcessToExecute.srcAccessMask = VK_ACCESS_COMMAND_PROCESS_WRITE_BIT_NVX;
		commandProcessToExecute.dstAccessMask = VK_ACCESS_INDIRECT_COMMAND_READ_BIT;
		commandProcessToExecute.offset = 0;
		commandProcessToExecute.size = VK_WHOLE_SIZE;

		// used by:
		//   DeviceGeneratedDrawIndirect
		//   DeviceGeneratedPsoDrawIndirect
		//   DeviceGeneratedVboIboPsoDrawIndirect
		{
			vkCmdUpdateBuffer(mainCmd, drawIndirectBuffer.buffer, 0, NumDrawIndirectCommands * sizeof(drawIndirectArgs), &drawIndirectArgs);

			transferToCommandProcess.buffer = drawIndirectBuffer.buffer;
			transferToCommandProcessBarriers.push_back(transferToCommandProcess);

			commandProcessToExecute.buffer = drawIndirectBuffer.buffer;
			commandProcessToExecuteBarriers.push_back(commandProcessToExecute);
		}

		// used by
		//   DeviceGeneratedPsoDrawIndirect
		//   DeviceGeneratedVboIboPsoDrawIndirect
		{
			uint32_t psoIndices[NumDrawIndirectCommands]
			{
				models[mCurrentModel].pipelineObjectTableIndices[mRenderModeOverride[0]],
				models[mCurrentModel].pipelineObjectTableIndices[mRenderModeOverride[1]]
			};

			NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "UpdateDeviceGeneratedPSOBuffer");

			vkCmdUpdateBuffer(mainCmd, deviceGeneratedPsoBuffer.buffer, 0, sizeof(psoIndices), psoIndices);

			VkBufferMemoryBarrier bufferBarrier = { VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER };

			transferToCommandProcess.buffer = deviceGeneratedPsoBuffer.buffer;
			transferToCommandProcessBarriers.push_back(transferToCommandProcess);

			commandProcessToExecute.buffer = deviceGeneratedPsoBuffer.buffer;
			commandProcessToExecuteBarriers.push_back(commandProcessToExecute);
		}

		// used by 
		//   DeviceGeneratedVboIboPsoDrawIndirect
		{
			// NOTE: we have 2 draw calls
			uint32_t vboIboIndices[NumDrawIndirectCommands * 2]
			{
				models[mCurrentModel].vboObjectTableIndex,
				models[mCurrentModel].vboObjectTableIndex,
				models[mCurrentModel].iboObjectTableIndex,
				models[mCurrentModel].iboObjectTableIndex
			};

			vkCmdUpdateBuffer(mainCmd, deviceGeneratedVboIboBuffer.buffer, 0, sizeof(vboIboIndices), vboIboIndices);

			VkBufferMemoryBarrier bufferBarrier = { VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER };

			transferToCommandProcess.buffer = deviceGeneratedVboIboBuffer.buffer;
			transferToCommandProcessBarriers.push_back(transferToCommandProcess);

			commandProcessToExecute.buffer = deviceGeneratedVboIboBuffer.buffer;
			commandProcessToExecuteBarriers.push_back(commandProcessToExecute);
		}

		// execute first part of the barriers to sync from host to command processing
		{
			vkCmdPipelineBarrier(mainCmd,
				VK_PIPELINE_STAGE_TRANSFER_BIT,
				VK_PIPELINE_STAGE_COMMAND_PROCESS_BIT_NVX,
				0,
				0, VK_NULL_HANDLE,
				transferToCommandProcessBarriers.size(), transferToCommandProcessBarriers.data(),
				0, VK_NULL_HANDLE
			);
		}
	}
#endif

	// device generated commands update a secondary command buffer
	bool secondaryCommandBuffers = mDrawMode >= NumDrawModesIndirect;

	vkCmdBeginRenderPass(mainCmd, &renderPassBeginInfo, secondaryCommandBuffers? VK_SUBPASS_CONTENTS_SECONDARY_COMMAND_BUFFERS : VK_SUBPASS_CONTENTS_INLINE);
	{
		NvVkContext::DebugMarkerScope marker(vk(), mainCmd , "draw model");
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

		vkCmdSetViewport(mainCmd, 0, 1, &vp);
		vkCmdSetScissor(mainCmd, 0, 1, &sc);

	
		{
			NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "sky box");
			vkCmdBindPipeline(mainCmd, VK_PIPELINE_BIND_POINT_GRAPHICS, mQuadPipeline);
			
			uint32_t offset[3];
			offset[0] = mUBO.getDynamicOffset();
			offset[1] = 0;
			offset[2] = 0;
			vkCmdBindDescriptorSets(mainCmd, VK_PIPELINE_BIND_POINT_GRAPHICS, mPipelineLayout, 0, 1, &mDescriptorSet, 1, offset);

			mQuad->Draw(mainCmd);
		}

		switch (mDrawMode)
		{
			// regular draw calls
			case DrawIndexed:
			{
				NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "DrawIndexed");

				// VBO, IBO, UBO
				{
					VkDeviceSize vertexoffsets[] = { 0 };
					vkCmdBindVertexBuffers(mainCmd, 0, 1, &model.VBO().buffer, vertexoffsets);
					vkCmdBindIndexBuffer(mainCmd, model.IBO().buffer, 0, model.getIndexType());

					uint32_t dynamicOffsets[2] = { mUBO.getDynamicOffset(), 0 };
					vkCmdBindDescriptorSets(mainCmd, VK_PIPELINE_BIND_POINT_GRAPHICS, mPipelineLayout, 0, 1, &mDescriptorSet, 1, dynamicOffsets);
				}

				// PSO
				{
					NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "Part0");
					vkCmdBindPipeline(mainCmd, VK_PIPELINE_BIND_POINT_GRAPHICS, models[mCurrentModel].pipelines[mRenderModeOverride[0]]);
					vkCmdDrawIndexed(mainCmd, drawIndirectArgs[0].indexCount, drawIndirectArgs[0].instanceCount, drawIndirectArgs[0].firstIndex, drawIndirectArgs[0].vertexOffset, drawIndirectArgs[0].firstInstance);
				}

				// PSO
				{
					NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "Part1");
					vkCmdBindPipeline(mainCmd, VK_PIPELINE_BIND_POINT_GRAPHICS, models[mCurrentModel].pipelines[mRenderModeOverride[1]]);
					vkCmdDrawIndexed(mainCmd, drawIndirectArgs[1].indexCount, drawIndirectArgs[1].instanceCount, drawIndirectArgs[1].firstIndex, drawIndirectArgs[1].vertexOffset, drawIndirectArgs[1].firstInstance);
				}

				break;
			}

			// indirect draw calls
			case DrawIndirect:
			{
				NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "DrawIndirect");

				// VBO, IBO, UBO
				{
					VkDeviceSize vertexoffsets[] = { 0 };
					vkCmdBindVertexBuffers(mainCmd, 0, 1, &model.VBO().buffer, vertexoffsets);
					vkCmdBindIndexBuffer(mainCmd, model.IBO().buffer, 0, model.getIndexType());

					uint32_t dynamicOffsets[2] = { mUBO.getDynamicOffset(), 0 };
					vkCmdBindDescriptorSets(mainCmd, VK_PIPELINE_BIND_POINT_GRAPHICS, mPipelineLayout, 0, 1, &mDescriptorSet, 1, dynamicOffsets);
				}

				// PSO
				{
					NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "Part0");
					vkCmdBindPipeline(mainCmd, VK_PIPELINE_BIND_POINT_GRAPHICS, models[mCurrentModel].pipelines[mRenderModeOverride[0]]);
					vkCmdDrawIndexedIndirect(mainCmd, drawIndirectBuffer.buffer, 0, 1, 0);
				}

				// PSO
				{
					NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "Part1");
					vkCmdBindPipeline(mainCmd, VK_PIPELINE_BIND_POINT_GRAPHICS, models[mCurrentModel].pipelines[mRenderModeOverride[1]]);
					vkCmdDrawIndexedIndirect(mainCmd, drawIndirectBuffer.buffer, sizeof(VkDrawIndexedIndirectCommand), 1, sizeof(VkDrawIndexedIndirectCommand));
				}
				break;
			}

#if defined (VK_NVX_device_generated_commands)
			
			// minimal VK_NVX_device_generated_commands usage: moral equivalent of the DrawIndirect mode
			case DeviceGeneratedDrawIndirect:
			{
				NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "DeviceGenerateDrawIndirect");

				for (int32_t part = 0; part < 2; ++part)
				{
					VkCommandBufferBeginInfo beginInfo = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };

					// we need to reserve space in the secondary command buffers
					vkBeginCommandBuffer(generatedCmdBuffersDrawIndirect[part], &beginInfo);
					{
						// secondary command buffers don't inherit state, so setup all the things
						// IBO, VBO, UBO
						{
							VkDeviceSize vertexoffsets[] = { 0 };
							vkCmdBindVertexBuffers(generatedCmdBuffersDrawIndirect[part], 0, 1, &model.VBO().buffer, vertexoffsets);
							vkCmdBindIndexBuffer(generatedCmdBuffersDrawIndirect[part], model.IBO().buffer, 0, model.getIndexType());

							uint32_t dynamicOffsets[2] = { mUBO.getDynamicOffset(), 0 };
							vkCmdBindDescriptorSets(generatedCmdBuffersDrawIndirect[part], VK_PIPELINE_BIND_POINT_GRAPHICS, mPipelineLayout, 0, 1, &mDescriptorSet, 1, dynamicOffsets);
						}

						// PSO
						{
							vkCmdBindPipeline(generatedCmdBuffersDrawIndirect[part], VK_PIPELINE_BIND_POINT_GRAPHICS, models[mCurrentModel].pipelines[mRenderModeOverride[part]]);
						}

						// allocate space where the commands will be generated into
						// this is conceptually where  vkCmdDrawIndexedIndirect would appear
						{
							VkCmdReserveSpaceForCommandsInfoNVX reserveInfo = { /* remove cast when proper header*/ (VkStructureType)VK_STRUCTURE_TYPE_CMD_RESERVE_SPACE_FOR_COMMANDS_INFO_NVX };

							reserveInfo.objectTable = deviceGeneratedObjectTable;
							reserveInfo.indirectCommandsLayout = deviceGeneratedLayoutDrawIndirect;
							reserveInfo.maxSequencesCount = 1;

							vkCmdReserveSpaceForCommandsNVX(generatedCmdBuffersDrawIndirect[part], &reserveInfo);
						}
					}
					vkEndCommandBuffer(generatedCmdBuffersDrawIndirect[part]);
					
					// now we actually generate the draw commands for the secondary command buffers
					{
						VkIndirectCommandsTokenNVX commandTokens[1];

						commandTokens[0].tokenType = VK_INDIRECT_COMMANDS_TOKEN_DRAW_INDEXED_NVX;
						commandTokens[0].buffer = drawIndirectBuffer.buffer;
						commandTokens[0].offset = part * sizeof(VkDrawIndexedIndirectCommand); // essentially the stride in vkCmdDrawIndexedIndirect

						VkCmdProcessCommandsInfoNVX processInfo = {/* remove cast when proper header*/ (VkStructureType)VK_STRUCTURE_TYPE_CMD_PROCESS_COMMANDS_INFO_NVX };

						processInfo.pIndirectCommandsTokens = commandTokens;
						processInfo.indirectCommandsTokenCount = ARRAY_SIZE(commandTokens);

						processInfo.objectTable = deviceGeneratedObjectTable;
						processInfo.indirectCommandsLayout = deviceGeneratedLayoutDrawIndirect;

						processInfo.maxSequencesCount = 1;
						processInfo.sequencesCountBuffer = VK_NULL_HANDLE;
						processInfo.sequencesCountOffset = 0;

						processInfo.targetCommandBuffer = generatedCmdBuffersDrawIndirect[part];

						vkCmdProcessCommandsNVX(mainCmd, &processInfo);
					}
				}

				// execute second part of the barriers to sync command processing to executing
				{
					vkCmdPipelineBarrier(mainCmd,
						VK_PIPELINE_STAGE_COMMAND_PROCESS_BIT_NVX,
						VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT,
						0,
						0, VK_NULL_HANDLE,
						commandProcessToExecuteBarriers.size(), commandProcessToExecuteBarriers.data(),
						0, VK_NULL_HANDLE
						);
				}

				// now execute the generated secondary command buffers
				{
					vkCmdExecuteCommands(mainCmd, 2, generatedCmdBuffersDrawIndirect);
				}
				break;
			}

			// interesting VK_NVX_device_generated_commands usage: changing the PSO within the token buffer
			case DeviceGeneratedPsoDrawIndirect:
			{
				NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "DeviceGeneratePSODrawIndirect");

				VkCommandBufferBeginInfo beginInfo = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };

				// we need to reserve space in the secondary command buffer
				vkBeginCommandBuffer(generatedCmdBufferPsoDrawIndirect, &beginInfo);
				{
					// secondary command buffers don't inherit state, so setup all the things
					// IBO, VBO, UBO
					{
						VkDeviceSize vertexoffsets[] = { 0 };
						vkCmdBindVertexBuffers(generatedCmdBufferPsoDrawIndirect, 0, 1, &model.VBO().buffer, vertexoffsets);
						vkCmdBindIndexBuffer(generatedCmdBufferPsoDrawIndirect, model.IBO().buffer, 0, model.getIndexType());

						uint32_t dynamicOffsets[2] = { mUBO.getDynamicOffset(), 0 };
						vkCmdBindDescriptorSets(generatedCmdBufferPsoDrawIndirect, VK_PIPELINE_BIND_POINT_GRAPHICS, mPipelineLayout, 0, 1, &mDescriptorSet, 1, dynamicOffsets);
					}

					// allocate space where the commands will be generated into
					// this is conceptually where  vkCmdDrawIndexedIndirect would appear
					{
						VkCmdReserveSpaceForCommandsInfoNVX reserveInfo = { /* remove cast when proper header*/ (VkStructureType)VK_STRUCTURE_TYPE_CMD_RESERVE_SPACE_FOR_COMMANDS_INFO_NVX };

						reserveInfo.objectTable = deviceGeneratedObjectTable;
						reserveInfo.indirectCommandsLayout = deviceGeneratedLayoutDrawIndirect;
						reserveInfo.maxSequencesCount = NumDrawIndirectCommands;

						vkCmdReserveSpaceForCommandsNVX(generatedCmdBufferPsoDrawIndirect, &reserveInfo);
					}
				}
				vkEndCommandBuffer(generatedCmdBufferPsoDrawIndirect);

				// now we actually generate the draw commands for the secondary command buffer
				{
					VkIndirectCommandsTokenNVX commandTokens[2];

					commandTokens[0].tokenType = VK_INDIRECT_COMMANDS_TOKEN_PIPELINE_NVX;
					commandTokens[0].buffer = deviceGeneratedPsoBuffer.buffer;
					commandTokens[0].offset = 0;  

					commandTokens[1].tokenType = VK_INDIRECT_COMMANDS_TOKEN_DRAW_INDEXED_NVX;
					commandTokens[1].buffer = drawIndirectBuffer.buffer;
					commandTokens[1].offset = 0; 

					VkCmdProcessCommandsInfoNVX processInfo = {/* remove cast when proper header*/ (VkStructureType)VK_STRUCTURE_TYPE_CMD_PROCESS_COMMANDS_INFO_NVX };

					processInfo.pIndirectCommandsTokens = commandTokens;
					processInfo.indirectCommandsTokenCount = ARRAY_SIZE(commandTokens);

					processInfo.objectTable = deviceGeneratedObjectTable;
					processInfo.indirectCommandsLayout = deviceGeneratedLayoutPsoDrawIndirect;

					processInfo.maxSequencesCount = 2;
					processInfo.sequencesCountBuffer = VK_NULL_HANDLE;
					processInfo.sequencesCountOffset = 0;

					processInfo.targetCommandBuffer = generatedCmdBufferPsoDrawIndirect;

					vkCmdProcessCommandsNVX(mainCmd, &processInfo); 
				}

				// execute second part of the barriers to sync command processing to executing
				{
					vkCmdPipelineBarrier(mainCmd,
						VK_PIPELINE_STAGE_COMMAND_PROCESS_BIT_NVX,
						VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT,
						0,
						0, VK_NULL_HANDLE,
						commandProcessToExecuteBarriers.size(), commandProcessToExecuteBarriers.data(),
						0, VK_NULL_HANDLE
						);
				}

				// now execute the generated secondary command buffer
				{
					vkCmdExecuteCommands(mainCmd, 1, &generatedCmdBufferPsoDrawIndirect);
				}
				break;
			}

			// interesting VK_NVX_device_generated_commands usage : changing the IBO, VBO and PSO within the token buffer
			case DeviceGeneratedVboIboPsoDrawIndirect:
			{
				NvVkContext::DebugMarkerScope marker(vk(), mainCmd, "DeviceGeneratedVboIboPsoDrawIndirect");

				VkCommandBufferBeginInfo beginInfo = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };

				// we need to reserve space in the secondary command buffer
				vkBeginCommandBuffer(generatedCmdBufferVboIboPsoDrawIndirect, &beginInfo);
				{
					// secondary command buffers don't inherit state, so setup all the things

					VkDeviceSize vertexoffsets[] = { 0 };
					vkCmdBindVertexBuffers(generatedCmdBufferVboIboPsoDrawIndirect, 0, 1, &model.VBO().buffer, vertexoffsets);
					vkCmdBindIndexBuffer(generatedCmdBufferVboIboPsoDrawIndirect, model.IBO().buffer, 0, model.getIndexType());

					// UBO
					{

						uint32_t dynamicOffsets[2] = { mUBO.getDynamicOffset(), 0 };
						vkCmdBindDescriptorSets(generatedCmdBufferVboIboPsoDrawIndirect, VK_PIPELINE_BIND_POINT_GRAPHICS, mPipelineLayout, 0, 1, &mDescriptorSet, 1, dynamicOffsets);
					}

					// allocate space where the commands will be generated into
					// this is conceptually where  vkCmdDrawIndexedIndirect would appear
					{
						VkCmdReserveSpaceForCommandsInfoNVX reserveInfo = { /* remove cast when proper header*/ (VkStructureType)VK_STRUCTURE_TYPE_CMD_RESERVE_SPACE_FOR_COMMANDS_INFO_NVX };

						reserveInfo.objectTable = deviceGeneratedObjectTable;
						reserveInfo.indirectCommandsLayout = deviceGeneratedLayoutDrawIndirect;
						reserveInfo.maxSequencesCount = NumDrawIndirectCommands;

						vkCmdReserveSpaceForCommandsNVX(generatedCmdBufferVboIboPsoDrawIndirect, &reserveInfo);
					}
				}
				vkEndCommandBuffer(generatedCmdBufferVboIboPsoDrawIndirect);

				// now we actually generate the draw commands for the secondary command buffers
				{
					VkIndirectCommandsTokenNVX commandTokens[4];

					// we use the same buffer for the VBO entries in the first half and IBO entries in the second half
					commandTokens[0].tokenType = VK_INDIRECT_COMMANDS_TOKEN_VERTEX_BUFFER_NVX;
					commandTokens[0].buffer = deviceGeneratedVboIboBuffer.buffer;
					commandTokens[0].offset = 0;

					commandTokens[1].tokenType = VK_INDIRECT_COMMANDS_TOKEN_INDEX_BUFFER_NVX;
					commandTokens[1].buffer = deviceGeneratedVboIboBuffer.buffer;
					commandTokens[1].offset = NumDrawIndirectCommands * sizeof(uint32_t); 

					commandTokens[2].tokenType = VK_INDIRECT_COMMANDS_TOKEN_PIPELINE_NVX;
					commandTokens[2].buffer = deviceGeneratedPsoBuffer.buffer;
					commandTokens[2].offset = 0;

					commandTokens[3].tokenType = VK_INDIRECT_COMMANDS_TOKEN_DRAW_INDEXED_NVX;
					commandTokens[3].buffer = drawIndirectBuffer.buffer;
					commandTokens[3].offset = 0;

					VkCmdProcessCommandsInfoNVX processInfo = {/* remove cast when proper header*/ (VkStructureType)VK_STRUCTURE_TYPE_CMD_PROCESS_COMMANDS_INFO_NVX };

					processInfo.pIndirectCommandsTokens = commandTokens;
					processInfo.indirectCommandsTokenCount = ARRAY_SIZE(commandTokens);

					processInfo.objectTable = deviceGeneratedObjectTable;
					processInfo.indirectCommandsLayout = deviceGeneratedLayoutVboIboPsoDrawIndirect;

					processInfo.maxSequencesCount = 2;
					processInfo.sequencesCountBuffer = VK_NULL_HANDLE;
					processInfo.sequencesCountOffset = 0;

					processInfo.targetCommandBuffer = generatedCmdBufferVboIboPsoDrawIndirect;

					vkCmdProcessCommandsNVX(mainCmd, &processInfo);
				}

				// execute second part of the barriers to sync command processing to executing
				{
					vkCmdPipelineBarrier(mainCmd,
						VK_PIPELINE_STAGE_COMMAND_PROCESS_BIT_NVX,
						VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT,
						0,
						0, VK_NULL_HANDLE,
						commandProcessToExecuteBarriers.size(), commandProcessToExecuteBarriers.data(),
						0, VK_NULL_HANDLE
						);
				}

				// now execute the generated secondary command buffer
				{
					vkCmdExecuteCommands(mainCmd, 1, &generatedCmdBufferVboIboPsoDrawIndirect);
				}
				break;
			}
#endif
		} // switch
	}

	vkCmdEndRenderPass(mainCmd);

	vk().submitMainCommandBuffer();
}

NvUIEventResponse BasicDeviceGeneratedCommandsVk::handleReaction(const NvUIReaction &react)
{
	return nvuiEventNotHandled;
}

NvAppBase* NvAppFactory() {
	return new BasicDeviceGeneratedCommandsVk();
}
