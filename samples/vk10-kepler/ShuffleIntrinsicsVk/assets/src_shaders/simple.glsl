//----------------------------------------------------------------------------------
// File:        vk10-kepler\ShuffleIntrinsicsVk\assets\src_shaders/simple.glsl
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
#GLSL_VS
#version 450 core
layout(location = 0) in vec2 aVertex;
layout(location = 1) in vec4 aColor;

layout(location = 0) out vec4 vColor;

void main()
{
    vColor = aColor;
    gl_Position = vec4(aVertex, 0, 1);
}

#GLSL_FS
#version 450 core
#define HAS_INTRINSICS ( defined (GL_NV_shader_thread_shuffle) && defined (GL_NV_shader_thread_group) && defined(GL_NV_gpu_shader5) && defined(GL_ARB_shader_ballot) && defined(GL_ARB_shader_group_vote) )
#if HAS_INTRINSICS
#extension GL_NV_shader_thread_shuffle: require
#extension GL_NV_shader_thread_group : require
#extension GL_NV_gpu_shader5 : require
#extension GL_ARB_shader_ballot : require
#extension GL_ARB_shader_group_vote : require
#endif

layout(constant_id = 0) const int mode = 0;

layout(location = 0) in vec4 vColor;
layout(location = 0) out vec4 oFrag;

void main()
{
	oFrag = vec4(0, 0, 0, 0);

	float wave = 0.6f + 0.4f * sin(length(vec2(450, 250) - gl_FragCoord.xy));

	vec4 color = vColor * wave ;


	switch (mode)
	{

		// pass through vertex color
		case 0:
		{
			oFrag = color;
			break;
		}

#if HAS_INTRINSICS

		// map lane id to grey
		case 1:
		{
			float s = float(gl_SubGroupInvocationARB) / float(32);
			oFrag = vec4(s, s, s, s);
			break;
		}
		
		// map warp id to grey
		case 2:
		{
			float s = float(gl_WarpIDNV % 32) / float(32);
			oFrag = vec4(s, s, s, s);
			break;
		}

		// map SM id to grey
		case 3:
		{
			float s = float(gl_SMIDNV % 32) / float(32);
			oFrag = vec4(s, s, s, s);
			break;
		}

		// map number of active threads to color
		case 4:
		{
			uint activeThreads = ballotThreadNV(true);

			float s = float(bitCount(activeThreads) ) / float(32);
			oFrag = vec4(s, s, s, s);
			break;
		}

		// mark first invocation as yellow, using ARB intrinsics
		case 5:
		{
			uint firstLaneId = readFirstInvocationARB(gl_SubGroupInvocationARB);

			if (firstLaneId == gl_SubGroupInvocationARB)
			{
				oFrag = vec4(1, 1, 0, 0);
			}
			else
			{
				oFrag = color;
			}

			break;
		}

		// mark first invocation as yellow and last as magenta, using NV intrinsics
		case 6:
		{
			uint activeThreads = ballotThreadNV(true);

			uint firstLaneId = findLSB(activeThreads);
			uint lastLaneId = findMSB(activeThreads);

			if (firstLaneId == gl_ThreadInWarpNV)
			{
				oFrag = vec4(1, 1, 0, 0);
			}
			else if (lastLaneId == gl_ThreadInWarpNV)
			{
				oFrag = vec4(1, 0, 1, 0);
			}
			else
			{
				oFrag = color;
			}

			break;
		}
		
		// compute max of color within warp
		case 7:
		{
			vec4 warpMax = color;
			warpMax = max(warpMax, shuffleXorNV(warpMax, 16,32));
			warpMax = max(warpMax, shuffleXorNV(warpMax, 8, 32));
			warpMax = max(warpMax, shuffleXorNV(warpMax, 4, 32));
			warpMax = max(warpMax, shuffleXorNV(warpMax, 2, 32));
			warpMax = max(warpMax, shuffleXorNV(warpMax, 1, 32));

			oFrag = warpMax;
			break;
		}

		// compute min of color within warp
		case 8:
		{
			vec4 warpMin = color;
			warpMin = min(warpMin, shuffleXorNV(warpMin, 16, 32));
			warpMin = min(warpMin, shuffleXorNV(warpMin, 8, 32));
			warpMin = min(warpMin, shuffleXorNV(warpMin, 4, 32));
			warpMin = min(warpMin, shuffleXorNV(warpMin, 2, 32));
			warpMin = min(warpMin, shuffleXorNV(warpMin, 1, 32));

			oFrag = warpMin;
			break;
		}
#endif

		default:
		{
			break;
		}
	}
	


} 
