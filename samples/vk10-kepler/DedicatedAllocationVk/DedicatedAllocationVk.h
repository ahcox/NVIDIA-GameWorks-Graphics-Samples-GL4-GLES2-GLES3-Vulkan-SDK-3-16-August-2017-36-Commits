//----------------------------------------------------------------------------------
// File:        vk10-kepler\DedicatedAllocationVk/DedicatedAllocationVk.h
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

#include "KHR/khrplatform.h"
#include "NvGamepad/NvGamepad.h"
#include "NV/NvMath.h"

class DedicatedAllocationVk : public NvSampleAppVK
{
public:
    DedicatedAllocationVk();
    ~DedicatedAllocationVk();
    
	void initRendering(void);


    void shutdownRendering(void);

    void initUI(void);
    void draw(void);
    void reshape(int32_t width, int32_t height);

    void configurationCallback(NvVKConfiguration& config);

	NvUIEventResponse handleReaction(const NvUIReaction &react);

private:

    // window size dependent

    VkPipeline mPipeline;
	VkPipelineLayout mPipelineLayout;

    // buffer with geometry
    NvVkBuffer mVertexBuffer;
    NvVkBuffer mIndexBuffer;

	VkFormat mOffScreenTargetFormat;
	VkRenderPass mOffScreenRenderPass;

	struct Tile
	{
		VkDeviceMemory memory;
		VkImage 	image;
		VkImageView imageView;
		VkFramebuffer framebuffer;

		nv::vec3f clear_color;
		uint32_t dimension;
	};

	void initTile(Tile& tile, uint32_t dimension, nv::vec3f clear_color, bool dedicated);
	void destroyTile(Tile& tile);
	
	void drawTile(VkCommandBuffer cmd, Tile& tile, bool forceClearOnly = false);

	void blitTile(Tile& tile, int32_t x, int32_t y);
	
	enum
	{
		TileDimension = 512
	};

	Tile mRegularAllocation;
	Tile mDedicatedAllocation;

	// application state

	float mTime;

	enum RenderMode
	{
		Regular,
		Dedicated,
		NumRenderModes
	};

	uint32_t mRenderMode;

	bool mSupportsDedicatedAllocation;
   
    bool mDrawGeometry;
};
