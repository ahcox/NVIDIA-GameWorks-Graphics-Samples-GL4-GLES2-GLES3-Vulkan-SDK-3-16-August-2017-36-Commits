//----------------------------------------------------------------------------------
// File:        es3aep-kepler\VisualizeHDR/aces.cpp
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
#include "aces.h"
#include "NV/NvMath.h"
#include <algorithm>
using std::max;
using std::min;

#ifndef M_PI
#define M_PI 3.1415927
#endif

// Missing utility functions
struct matrix3
{
	float m[9];
};

inline nv::vec3f mul(const matrix3& m, const nv::vec3f& v)
{
	nv::vec3f res;

	res.x = v.x * m.m[0] + v.y * m.m[1] + v.z * m.m[2];
	res.y = v.x * m.m[3] + v.y * m.m[4] + v.z * m.m[5];
	res.z = v.x * m.m[6] + v.y * m.m[7] + v.z * m.m[8];

	return res;
}

template <typename T>
T lerp(const T& a, const T& b, const float& t)
{
	return a + t*(b - a);
}

nv::vec3f max(const nv::vec3f &a, const float &b)
{
	nv::vec3f c;
	c.x = std::max(a.x, b);
	c.y = std::max(a.y, b);
	c.z = std::max(a.z, b);

	return c;
}

nv::vec3f min(const nv::vec3f& a, const float& b)
{
	nv::vec3f c;
	c.x = std::min(a.x, b);
	c.y = std::min(a.y, b);
	c.z = std::min(a.z, b);

	return c;
}

namespace {
	

	/**********************************************************************************************
	*
	* ACES utility functions
	*
	**********************************************************************************************/

	// Transformations between CIE XYZ tristimulus values and CIE x,y 
	// chromaticity coordinates
	inline nv::vec3f XYZ_2_xyY(const nv::vec3f& XYZ)
	{
		nv::vec3f xyY;
		float divisor = (XYZ[0] + XYZ[1] + XYZ[2]);
		if (divisor == 0.) divisor = 1e-10f;
		xyY[0] = XYZ[0] / divisor;
		xyY[1] = XYZ[1] / divisor;
		xyY[2] = XYZ[1];

		return xyY;
	}

	inline nv::vec3f xyY_2_XYZ(const nv::vec3f& xyY)
	{
		nv::vec3f XYZ;
		XYZ[0] = xyY[0] * xyY[2] / std::max(xyY[1], 1e-10f);
		XYZ[1] = xyY[2];
		XYZ[2] = (1.0f - xyY[0] - xyY[1]) * xyY[2] / std::max(xyY[1], 1e-10f);

		return XYZ;
	}

	inline unsigned short float2half(float f)
	{
		union fasi { float f; unsigned int i; };
		fasi v;
		v.f = f;

		unsigned short sign = (v.i >> 31) & 0x1;
		short exp = ((v.i >> 23) & 0xff) - 127;
		unsigned short mant = (v.i >> 13) & 0x3ff;

		if (exp < -14)
		{
			exp = 0;
			mant = (0x400 & mant) >> (-exp - 14);
		}
		else if (exp < 16)
		{
			exp += 15;
		}
		else
		{
			// just make it inf, ignore NaN
			exp = 31;
			mant = 0;
		}

		return (sign << 15) | (exp << 10) | mant;
	}


	/*
	* Struct with ODT spline parameters, configured for easy DX constant buffer compatibility
	*/
	struct SegmentedSplineParams_c9
	{
		nv::vec4f coefs[10];
		nv::vec2f minPoint; // {luminance, luminance} linear extension below this
		nv::vec2f midPoint; // {luminance, luminance} 
		nv::vec2f maxPoint; // {luminance, luminance} linear extension above this
		nv::vec2f slope;
		nv::vec2f limits;   // limits in ODT curve prior to RRT adjustment
	};

	SegmentedSplineParams_c9 GetAcesData(int BaseCurve, float MaxStop, float MaxLevel, float MidGrayScale);

	struct ACESparams
	{
		SegmentedSplineParams_c9 C;
		matrix3 XYZ_2_DISPLAY_PRI_MAT;
		matrix3 DISPLAY_PRI_MAT_2_XYZ;
		nv::vec2f CinemaLimits;
		int OutputMode;
		float surroundGamma;
		bool desaturate;
		bool surroundAdjust;
		bool applyCAT;
		bool tonemapLuminance;
		float saturationLevel;
	};

	/////////////////////////////////////////////////////////////////////////////////////////
	//
	//  ACES code
	//
	////////////////////////////////////////////////////////////////////////////////////////




	/*
	* Struct with RRT spline parameters
	*/
	struct SegmentedSplineParams_c5
	{
		float coefsLow[6];    // coefs for B-spline between minPoint and midPoint (units of log luminance)
		float coefsHigh[6];   // coefs for B-spline between midPoint and maxPoint (units of log luminance)
		nv::vec2f minPoint; // {luminance, luminance} linear extension below this
		nv::vec2f midPoint; // {luminance, luminance} 
		nv::vec2f maxPoint; // {luminance, luminance} linear extension above this
		nv::vec2f logMinPoint; // {luminance, luminance} linear extension below this
		nv::vec2f logMidPoint; // {luminance, luminance} 
		nv::vec2f logMaxPoint; // {luminance, luminance} linear extension above this
		float slopeLow;       // log-log slope of low linear extension
		float slopeHigh;      // log-log slope of high linear extension
	};

	/*
	* Struct with ODT spline parameters
	*/
	struct SegmentedSplineParams_c9_internal
	{
		float coefsLow[10];    // coefs for B-spline between minPoint and midPoint (units of log luminance)
		float coefsHigh[10];   // coefs for B-spline between midPoint and maxPoint (units of log luminance)
		nv::vec2f minPoint; // {luminance, luminance} linear extension below this
		nv::vec2f midPoint; // {luminance, luminance} 
		nv::vec2f maxPoint; // {luminance, luminance} linear extension above this
		nv::vec2f slope;
		nv::vec2f limits; // min and max prior to RRT
	};

	float pow10(float x)
	{
		return pow(10.0f, x);
	}


	//
	//  Spline function used by RRT
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	static float segmented_spline_c5_fwd(float x)
	{
		// RRT_PARAMS
		static const SegmentedSplineParams_c5 C =
		{
			// coefsLow[6]
			{ -4.0000000000f, -4.0000000000f, -3.1573765773f, -0.4852499958f, 1.8477324706f, 1.8477324706f },
			// coefsHigh[6]
			{ -0.7185482425f, 2.0810307172f, 3.6681241237f, 4.0000000000f, 4.0000000000f, 4.0000000000f },
			{ 0.18f*exp2(-15.0f), 0.0001f },    // minPoint
			{ 0.18f, 4.8f },    // midPoint
			{ 0.18f*exp2(18.0f), 10000.f },    // maxPoint
			{ log10f(0.18f*exp2(-15.0f)), -4.0f },    // minPoint
			{ -0.7447274949f, 0.681241237f },    // midPoint
			{ log10f(0.18f*exp2(18.0f)), 4.0f },    // maxPoint
			0.0f,  // slopeLow
			0.0f   // slopeHigh
		};

		// Textbook monomial to basis-function conversion matrix.

		static const nv::vec3f M[3] =
		{
			{ 0.5f, -1.0f, 0.5f },
			{ -1.0f, 1.0f, 0.5f },
			{ 0.5f, 0.0f, 0.0f }
		};

		const int N_KNOTS_LOW = 4;
		const int N_KNOTS_HIGH = 4;

		// Check for negatives or zero before taking the log. If negative or zero,
		// set to ACESMIN.1
		float xCheck = x <= 0 ? exp2(-14.0f) : x;

		float logx = log10(xCheck);
		float logy;

		if (logx <= C.logMinPoint.x)
		{
			logy = logx * C.slopeLow + (C.logMinPoint.y - C.slopeLow * C.logMinPoint.y);
		}
		else if ((logx > C.logMinPoint.x) && (logx < C.logMidPoint.x))
		{
			float knot_coord = (N_KNOTS_LOW - 1) * (logx - C.logMinPoint.x) / (C.logMidPoint.x - C.logMinPoint.x);
			int j = int(knot_coord);
			float t = knot_coord - j;

			nv::vec3f cf = { C.coefsLow[j], C.coefsLow[j + 1], C.coefsLow[j + 2] };

			nv::vec3f monomials = { t * t, t, 1 };
			nv::vec3f basis = cf.x * M[0] + cf.y * M[1] + cf.z * M[2];
			logy = nv::dot(monomials, basis);
		}
		else if ((logx >= C.logMidPoint.x) && (logx < C.logMaxPoint.x))
		{
			float knot_coord = (N_KNOTS_HIGH - 1) * (logx - C.logMidPoint.x) / (C.logMaxPoint.x - C.logMidPoint.x);
			int j = int(knot_coord);
			float t = knot_coord - j;

			nv::vec3f cf = { C.coefsHigh[j], C.coefsHigh[j + 1], C.coefsHigh[j + 2] };

			nv::vec3f monomials = { t * t, t, 1 };
			nv::vec3f basis = cf.x * M[0] + cf.y * M[1] + cf.z * M[2];
			logy = nv::dot(monomials, basis);
		}
		else
		{ //if ( logIn >= log10(C.maxPoint.x) ) { 
			logy = logx * C.slopeHigh + (C.logMaxPoint.y - C.slopeHigh * C.logMaxPoint.x);
		}

		return pow10(logy);
	}


	inline void mul3(nv::vec3f& res, const nv::vec3f &a, const nv::vec3f* M)
	{
		res = a.x * M[0] + a.y * M[1] + a.z * M[2];
	}

	//
	//  AdaptSpline
	//
	//    Using a reference ODT spline, adjust middle gray and max levels
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	static SegmentedSplineParams_c9_internal AdaptSpline(const SegmentedSplineParams_c9_internal C, float newMin, float newMax, float outMax, float outMidScale)
	{
		// Monomial and inverse monomial matrices
		static const nv::vec3f M[3] =
		{
			{ 0.5f, -1.0f, 0.5f },
			{ -1.0f, 1.0f, 0.5f },
			{ 0.5f, 0.0f, 0.0f }
		};

		static const nv::vec3f iM[3] =
		{
			{ 0.0f, 0.0f, 2.0f },
			{ -0.5f, 0.5f, 1.5f },
			{ 1.0f, 1.0f, 1.0f }
		};

		const int N_KNOTS_LOW = 8;
		const int N_KNOTS_HIGH = 8;

		SegmentedSplineParams_c9_internal C2 = C;

		// Set the new max input and output levels
		C2.maxPoint.x = segmented_spline_c5_fwd(newMax);
		C2.maxPoint.y = outMax;

		C2.limits.y = newMax;

		// Set new minimum levels
		C2.minPoint.x = segmented_spline_c5_fwd(newMin);

		C2.limits.x = newMin;

		// scale the middle gray output level
		C2.midPoint.y *= outMidScale;

		// compute and apply scale used to bring bottom segment of the transform to the level desired for middle gray
		float scale = (log10(C.midPoint[1]) - log10(C.minPoint[1])) / (log10(C2.midPoint[1]) - log10(C2.minPoint[1]));

		for (int j = 0; j < N_KNOTS_LOW + 2; j++)
		{
			C2.coefsLow[j] = (C2.coefsLow[j] - log10(C2.minPoint[1])) / scale + log10(C2.minPoint[1]);
		}

		// compute and apply scale to top segment of the transform to the level matching the new max and middle gray
		scale = (log10(C.maxPoint[1]) - log10(C.midPoint[1])) / (log10(C2.maxPoint[1]) - log10(C2.midPoint[1]));

		float target[10]; // saves the "target" values, as we need to match/relax the spline to properly join the low segment

		for (int j = 0; j < N_KNOTS_HIGH + 2; j++)
		{
			C2.coefsHigh[j] = (C2.coefsHigh[j] - log10(C.midPoint[1])) / scale + log10(C2.midPoint[1]);
			target[j] = C2.coefsHigh[j];
		}

		//
		// Adjust the high spline to properly meet the low spline, then relax the high spline
		//

		// Coefficients for the last segment of the low range
		nv::vec3f cfl = { C2.coefsLow[7], C2.coefsLow[8], C2.coefsLow[9] };

		// Coeffiecients for the first segment of the high range
		nv::vec3f cfh = { C2.coefsHigh[0], C2.coefsHigh[1], C2.coefsHigh[2] };

		nv::vec3f cflt, cfht;

		// transform the coefficients by the monomial matrix
		mul3(cflt, cfl, M);
		mul3(cfht, cfh, M);

		// low and high curves cover different ranges, so compute scaling factor needed to match slopes at the join point
		float scaleLow = 1.0f / (log10f(C2.midPoint[0]) - log10f(C2.minPoint[0]));
		float scaleHigh = 1.0f / (log10f(C2.maxPoint[0]) - log10f(C2.midPoint[0]));


		// compute the targeted exit point for the segment 
		float outRef = cfht[0] * 2.0f + cfht[1]; //slope at t == 1

		// match slopes and intersection
		cfht[2] = cflt[2];
		cfht[1] = (scaleLow * cflt[1]) / scaleHigh;

		// compute the exit point the segment has after the adjustment
		float out = cfht[0] * 2.0f + cfht[1]; //result at t == 1

		// ease spline toward target and adjust
		float outTarget = (outRef*7.0f + out*1.0f) / 8.0f;
		cfht[0] = (outTarget - cfht[1]) / 2.0f;

		// back-transform  the adjustments and save them
		nv::vec3f acfh;

		mul3(acfh, cfht, iM);

		C2.coefsHigh[0] = acfh[0];
		C2.coefsHigh[1] = acfh[1];
		C2.coefsHigh[2] = acfh[2];

		// now correct the rest of the spline
		for (int j = 1; j < N_KNOTS_HIGH; j++)
		{
			//  Original rescaled spline values for the segment (ideal "target")
			nv::vec3f cfoh = { target[j], target[j + 1], target[j + 2] };

			//  New spline values for the segment based on alterations to prior ranges
			nv::vec3f cfh = { C2.coefsHigh[j], C2.coefsHigh[j + 1], C2.coefsHigh[j + 2] };

			nv::vec3f cfht, cfoht;

			mul3(cfht, cfh, M);
			mul3(cfoht, cfoh, M);

			//Compute exit slope for segments
			float out = cfht[0] * 2.0f + cfht[1]; //slope at t == 1
			float outRef = cfoht[0] * 2.0f + cfoht[1]; //slope at t == 1

			//Ease spline toward targetted slope
			float outTarget = (outRef*(7.0f - j) + out*(1.0f + j)) / 8.0f;
			cfht[0] = (outTarget - cfht[1]) / 2.0f;

			// Back transform and save
			nv::vec3f acfh;

			mul3(acfh, cfht, iM);

			C2.coefsHigh[j] = acfh[0];
			C2.coefsHigh[j + 1] = acfh[1];
			C2.coefsHigh[j + 2] = acfh[2];
		}

		return C2;
	}

	//
	//  GetACESData
	//
	//    Select a curve used as part of the ACES ODT. Optionally, derive a modified
	//  version of the base curve with an altered middle gray and maximum number of
	//  stops.
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	SegmentedSplineParams_c9 GetAcesData(Aces::ODTCurve BaseCurve, float MinStop, float MaxStop, float MaxLevel, float MidGrayScale)
	{
		//
		// Standard ACES ODT curves
		//

		// ODT_48nits
		static const SegmentedSplineParams_c9_internal ODT_48nits =
		{
			// coefs[10]
			{ -1.6989700043f, -1.6989700043f, -1.4779000000f, -1.2291000000f, -0.8648000000f, -0.4480000000f, 0.0051800000f, 0.4511080334f, 0.9113744414f, 0.9113744414f },
			// coefsHigh[10]
			{ 0.5154386965f, 0.8470437783f, 1.1358000000f, 1.3802000000f, 1.5197000000f, 1.5985000000f, 1.6467000000f, 1.6746091357f, 1.6878733390f, 1.6878733390f },
			{ segmented_spline_c5_fwd(0.18f*exp2(-6.5f)), 0.02f },    // minPoint
			{ segmented_spline_c5_fwd(0.18f), 4.8f },    // midPoint  
			{ segmented_spline_c5_fwd(0.18f*exp2(6.5f)), 48.0f },    // maxPoint
			{ 0.0f, 0.04f },  // slope
			{ 0.18f*exp2f(-6.5f), 0.18f*exp2f(6.5f) } // limits
		};


		// ODT_1000nits
		static const SegmentedSplineParams_c9_internal ODT_1000nits =
		{
			// coefsLow[10]
			{ -2.3010299957f, -2.3010299957f, -1.9312000000f, -1.5205000000f, -1.0578000000f, -0.4668000000f, 0.1193800000f, 0.7088134201f, 1.2911865799f, 1.2911865799f },
			// coefsHigh[10]
			{ 0.8089132070f, 1.1910867930f, 1.5683000000f, 1.9483000000f, 2.3083000000f, 2.6384000000f, 2.8595000000f, 2.9872608805f, 3.0127391195f, 3.0127391195f },
			{ segmented_spline_c5_fwd(0.18f*pow(2.f, -12.f)), 0.005f },    // minPoint
			{ segmented_spline_c5_fwd(0.18f), 10.0f },    // midPoint  
			{ segmented_spline_c5_fwd(0.18f*pow(2.f, 10.f)), 1000.0f },    // maxPoint
			{ 0.0f, 0.06f },  // slope
			{ 0.18f*exp2f(-12.f), 0.18f*exp2f(10.f) } // limits
		};

		// ODT_2000nits
		static const SegmentedSplineParams_c9_internal ODT_2000nits =
		{
			// coefsLow[10]
			{ -2.3010299957f, -2.3010299957f, -1.9312000000f, -1.5205000000f, -1.0578000000f, -0.4668000000f, 0.1193800000f, 0.7088134201f, 1.2911865799f, 1.2911865799f },
			// coefsHigh[10]
			{ 0.8019952042f, 1.1980047958f, 1.5943000000f, 1.9973000000f, 2.3783000000f, 2.7684000000f, 3.0515000000f, 3.2746293562f, 3.3274306351f, 3.3274306351f },
			{ segmented_spline_c5_fwd(0.18f*pow(2.f, -12.f)), 0.005f },    // minPoint
			{ segmented_spline_c5_fwd(0.18f), 10.0f },    // midPoint  
			{ segmented_spline_c5_fwd(0.18f*pow(2.f, 11.f)), 2000.0f },    // maxPoint
			{ 0.0f, 0.12f },  // slope
			{ 0.18f*exp2f(-12.f), 0.18f*exp2f(11.f) } // limits
		};

		// ODT_4000nits
		static const SegmentedSplineParams_c9_internal ODT_4000nits =
		{
			// coefsLow[10]
			{ -2.3010299957f, -2.3010299957f, -1.9312000000f, -1.5205000000f, -1.0578000000f, -0.4668000000f, 0.1193800000f, 0.7088134201f, 1.2911865799f, 1.2911865799f },
			// coefsHigh[10]
			{ 0.7973186613f, 1.2026813387f, 1.6093000000f, 2.0108000000f, 2.4148000000f, 2.8179000000f, 3.1725000000f, 3.5344995451f, 3.6696204376f, 3.6696204376f },
			{ segmented_spline_c5_fwd(0.18f*pow(2.f, -12.f)), 0.005f },    // minPoint
			{ segmented_spline_c5_fwd(0.18f), 10.0 },    // midPoint  
			{ segmented_spline_c5_fwd(0.18f*pow(2.f, 12.f)), 4000.0f },    // maxPoint
			{ 0.0f, 0.3f },   // slope
			{ 0.18f*exp2f(-12.f), 0.18f*exp2f(12.f) } // limits
		};


		// convert, defaulting to 48 nits
		const SegmentedSplineParams_c9_internal *Src = &ODT_48nits;
		SegmentedSplineParams_c9_internal Generated;
		SegmentedSplineParams_c9 C;


		switch (BaseCurve)
		{
		case Aces::ODT_LDR_Ref: Src = &ODT_48nits; break;
		case Aces::ODT_1000Nit_Ref: Src = &ODT_1000nits; break;
		case Aces::ODT_2000Nit_Ref: Src = &ODT_2000nits; break;
		case Aces::ODT_4000Nit_Ref: Src = &ODT_4000nits; break;

		case Aces::ODT_LDR_Adj:
			Src = &Generated;
			MaxLevel = MaxLevel > 0.0f ? MaxLevel : 48.0f;
			MaxStop = MaxStop > 0 ? MaxStop : 6.5f;
			MinStop = MinStop < 0 ? MinStop : -6.5f;
			Generated = AdaptSpline(ODT_48nits, 0.18f*pow(2.f, MinStop), 0.18f*pow(2.f, MaxStop), MaxLevel, MidGrayScale);
			break;
		case Aces::ODT_1000Nit_Adj:
			Src = &Generated;
			MaxLevel = MaxLevel > 0.0f ? MaxLevel : 1000.0f;
			MaxStop = MaxStop > 0 ? MaxStop : 10.0f;
			MinStop = MinStop < 0 ? MinStop : -12.0f;
			Generated = AdaptSpline(ODT_1000nits, 0.18f*pow(2.f, MinStop), 0.18f*pow(2.f, MaxStop), MaxLevel, MidGrayScale);
			break;
		case Aces::ODT_2000Nit_Adj:
			Src = &Generated;
			MaxLevel = MaxLevel > 0.0f ? MaxLevel : 2000.0f;
			MaxStop = MaxStop > 0 ? MaxStop : 11.0f;
			MinStop = MinStop < 0 ? MinStop : -12.0f;
			Generated = AdaptSpline(ODT_2000nits, 0.18f*pow(2.f, MinStop), 0.18f*pow(2.f, MaxStop), MaxLevel, MidGrayScale);
			break;
		case Aces::ODT_4000Nit_Adj:
			Src = &Generated;
			MaxLevel = MaxLevel > 0.0f ? MaxLevel : 4000.0f;
			MaxStop = MaxStop > 0 ? MaxStop : 12.0f;
			MinStop = MinStop < 0 ? MinStop : -12.0f;
			Generated = AdaptSpline(ODT_4000nits, 0.18f*pow(2.f, MinStop), 0.18f*pow(2.f, MaxStop), MaxLevel, MidGrayScale);
			break;

		};

		{
			const SegmentedSplineParams_c9_internal &Curve = *Src;

			for (int Index = 0; Index < 10; Index++)
			{
				C.coefs[Index] = nv::vec4f(Curve.coefsLow[Index], Curve.coefsHigh[Index], 0.0f, 0.0f);
			}
			C.minPoint = Curve.minPoint;
			C.midPoint = Curve.midPoint;
			C.maxPoint = Curve.maxPoint;
			C.slope = Curve.slope;
			C.limits = Curve.limits;
		}


		return C;
	}

	/*****************************************************************************************************************/



	nv::vec3f clamp(const nv::vec3f& a, const float b, const float c)
	{
		nv::vec3f d;
		d.x = std::max(b, std::min(a.x, c));
		d.y = std::max(b, std::min(a.y, c));
		d.z = std::max(b, std::min(a.z, c));

		return d;
	}

	float clamp(const float& a, const float b, const float c)
	{
		return std::max(b, std::min(a, c));
	}

	float saturate(const float& a)
	{
		return clamp(a, 0.0f, 1.0f);
	}

	static const float TINY = 1e-10f;

	static const float Half_Max = 65504.0f;

	static const matrix3 AP0_2_XYZ_MAT =
	{
		0.95255238f, 0.00000000f, 0.00009368f,
		0.34396642f, 0.72816616f, -0.07213254f,
		-0.00000004f, 0.00000000f, 1.00882506f
	};

	static const matrix3 XYZ_2_AP0_MAT =
	{
		1.04981101f, -0.00000000f, -0.00009748f,
		-0.49590296f, 1.37331295f, 0.09824003f,
		0.00000004f, -0.00000000f, 0.99125212f
	};

	static const matrix3 AP1_2_XYZ_MAT =
	{
		0.66245413f, 0.13400421f, 0.15618768f,
		0.27222872f, 0.67408168f, 0.05368952f,
		-0.00557466f, 0.00406073f, 1.01033902f
	};

	static const matrix3 XYZ_2_AP1_MAT =
	{
		1.64102352f, -0.32480335f, -0.23642471f,
		-0.66366309f, 1.61533189f, 0.01675635f,
		0.01172191f, -0.00828444f, 0.98839492f
	};

	static const matrix3 AP0_2_AP1_MAT =
	{
		1.45143950f, -0.23651081f, -0.21492855f,
		-0.07655388f, 1.17623007f, -0.09967594f,
		0.00831613f, -0.00603245f, 0.99771625f
	};

	static const matrix3 AP1_2_AP0_MAT =
	{
		0.69545215f, 0.14067869f, 0.16386905f,
		0.04479461f, 0.85967094f, 0.09553432f,
		-0.00552587f, 0.00402521f, 1.00150073f
	};

	// EHart - need to check this, might be a transpose issue with CTL
	static const nv::vec3f AP1_RGB2Y = { 0.27222872f, 0.67408168f, 0.05368952f };


	float max_f3(const nv::vec3f& In)
	{
		return std::max(In.x, std::max(In.y, In.z));
	}

	float min_f3(const nv::vec3f& In)
	{
		return std::min(In.x, std::min(In.y, In.z));
	}

	float rgb_2_saturation(const nv::vec3f& rgb)
	{
		return (std::max(max_f3(rgb), TINY) - std::max(min_f3(rgb), TINY)) / std::max(max_f3(rgb), 1e-2f);
	}

	/* ---- Conversion Functions ---- */
	// Various transformations between color encodings and data representations
	//




	// Transformations from RGB to other color representations
	float rgb_2_hue(const nv::vec3f& rgb)
	{
		// Returns a geometric hue angle in degrees (0-360) based on RGB values.
		// For neutral colors, hue is undefined and the function will return a quiet NaN value.
		float hue;
		if (rgb[0] == rgb[1] && rgb[1] == rgb[2]) {
			// RGB triplets where RGB are equal have an undefined hue
			// EHart - reference code uses NaN, use 0 instead to prevent propagation of NaN
			hue = 0.0f;
		}
		else {
			hue = (180.f / M_PI) * atan2(sqrt(3.f)*(rgb[1] - rgb[2]), 2.f * rgb[0] - rgb[1] - rgb[2]);
		}

		if (hue < 0.f) hue = hue + 360.f;

		return hue;
	}

	float rgb_2_yc(const nv::vec3f& rgb, float ycRadiusWeight = 1.75)
	{
		// Converts RGB to a luminance proxy, here called YC
		// YC is ~ Y + K * Chroma
		// Constant YC is a cone-shaped surface in RGB space, with the tip on the 
		// neutral axis, towards white.
		// YC is normalized: RGB 1 1 1 maps to YC = 1
		//
		// ycRadiusWeight defaults to 1.75, although can be overridden in function 
		// call to rgb_2_yc
		// ycRadiusWeight = 1 -> YC for pure cyan, magenta, yellow == YC for neutral 
		// of same value
		// ycRadiusWeight = 2 -> YC for pure red, green, blue  == YC for  neutral of 
		// same value.

		float r = rgb[0];
		float g = rgb[1];
		float b = rgb[2];

		float chroma = sqrt(b*(b - g) + g*(g - r) + r*(r - b));

		return (b + g + r + ycRadiusWeight * chroma) / 3.f;
	}

	/* ODT utility functions */
	float Y_2_linCV(float Y, float Ymax, float Ymin)
	{
		return (Y - Ymin) / (Ymax - Ymin);
	}

	float linCV_2_Y(float linCV, float Ymax, float Ymin)
	{
		return linCV * (Ymax - Ymin) + Ymin;
	}

	// Gamma compensation factor
	static const float DIM_SURROUND_GAMMA = 0.9811f;

	nv::vec3f darkSurround_to_dimSurround(const nv::vec3f& linearCV)
	{
		nv::vec3f XYZ = mul(AP1_2_XYZ_MAT, linearCV);

		nv::vec3f xyY = XYZ_2_xyY(XYZ);
		xyY[2] = std::max(xyY[2], 0.f);
		xyY[2] = pow(xyY[2], DIM_SURROUND_GAMMA);
		XYZ = xyY_2_XYZ(xyY);

		return mul(XYZ_2_AP1_MAT, XYZ);
	}

	nv::vec3f dimSurround_to_darkSurround(const nv::vec3f& linearCV)
	{
		nv::vec3f XYZ = mul(AP1_2_XYZ_MAT, linearCV);

		nv::vec3f xyY = XYZ_2_xyY(XYZ);
		xyY[2] = std::max(xyY[2], 0.f);
		xyY[2] = pow(xyY[2], 1.f / DIM_SURROUND_GAMMA);
		XYZ = xyY_2_XYZ(xyY);

		return mul(XYZ_2_AP1_MAT, XYZ);
	}

	nv::vec3f alter_surround(const nv::vec3f& linearCV, float gamma)
	{
		nv::vec3f XYZ = mul(AP1_2_XYZ_MAT, linearCV);

		nv::vec3f xyY = XYZ_2_xyY(XYZ);
		xyY[2] = std::max(xyY[2], 0.0f);
		xyY[2] = pow(xyY[2], gamma);
		XYZ = xyY_2_XYZ(xyY);

		return mul(XYZ_2_AP1_MAT, XYZ);
	}



	matrix3 calc_sat_adjust_matrix(float sat, const nv::vec3f& rgb2Y)
	{
		//
		// This function determines the terms for a 3x3 saturation matrix that is
		// based on the luminance of the input.
		//
		matrix3 M;

		M.m[0] = (1.0f - sat) * rgb2Y[0] + sat;
		M.m[3] = (1.0f - sat) * rgb2Y[0];
		M.m[6] = (1.0f - sat) * rgb2Y[0];

		M.m[1] = (1.0f - sat) * rgb2Y[1];
		M.m[4] = (1.0f - sat) * rgb2Y[1] + sat;
		M.m[7] = (1.0f - sat) * rgb2Y[1];

		M.m[2] = (1.0f - sat) * rgb2Y[2];
		M.m[5] = (1.0f - sat) * rgb2Y[2];
		M.m[8] = (1.0f - sat) * rgb2Y[2] + sat;

		// EHart - removed transpose, as the indexing in CTL is transposed

		return M;
	}

	/* ---- Signal encode/decode functions ---- */

	float moncurve_f(float x, float gamma, float offs)
	{
		// Forward monitor curve
		float y;
		const float fs = ((gamma - 1.0f) / offs) * pow(offs * gamma / ((gamma - 1.0f) * (1.0f + offs)), gamma);
		const float xb = offs / (gamma - 1.0f);
		if (x >= xb)
			y = pow((x + offs) / (1.0f + offs), gamma);
		else
			y = x * fs;
		return y;
	}

	inline float moncurve_r(float y, float gamma, float offs)
	{
		// Reverse monitor curve
		float x;
		const float yb = pow(offs * gamma / ((gamma - 1.0f) * (1.0f + offs)), gamma);
		const float rs = pow((gamma - 1.0f) / offs, gamma - 1.0f) * pow((1.0f + offs) / gamma, gamma);
		if (y >= yb)
			x = (1.0f + offs) * pow(y, 1.0f / gamma) - offs;
		else
			x = y * rs;
		return x;
	}

	// Base functions from SMPTE ST 2084-2014

	// Constants from SMPTE ST 2084-2014
	static const float pq_m1 = 0.1593017578125f; // ( 2610.0 / 4096.0 ) / 4.0;
	static const float pq_m2 = 78.84375f; // ( 2523.0 / 4096.0 ) * 128.0;
	static const float pq_c1 = 0.8359375f; // 3424.0 / 4096.0 or pq_c3 - pq_c2 + 1.0;
	static const float pq_c2 = 18.8515625f; // ( 2413.0 / 4096.0 ) * 32.0;
	static const float pq_c3 = 18.6875f; // ( 2392.0 / 4096.0 ) * 32.0;

	static const float pq_C = 10000.0f;

	// Converts from the non-linear perceptually quantized space to linear cd/m^2
	// Note that this is in float, and assumes normalization from 0 - 1
	// (0 - pq_C for linear) and does not handle the integer coding in the Annex 
	// sections of SMPTE ST 2084-2014
	float pq_f(float N)
	{
		// Note that this does NOT handle any of the signal range
		// considerations from 2084 - this assumes full range (0 - 1)
		float Np = pow(N, 1.0f / pq_m2);
		float L = Np - pq_c1;
		if (L < 0.0f)
			L = 0.0f;
		L = L / (pq_c2 - pq_c3 * Np);
		L = pow(L, 1.0f / pq_m1);
		return L * pq_C; // returns cd/m^2
	}

	// Converts from linear cd/m^2 to the non-linear perceptually quantized space
	// Note that this is in float, and assumes normalization from 0 - 1
	// (0 - pq_C for linear) and does not handle the integer coding in the Annex 
	// sections of SMPTE ST 2084-2014
	float pq_r(float C)
	{
		// Note that this does NOT handle any of the signal range
		// considerations from 2084 - this returns full range (0 - 1)
		float L = C / pq_C;
		float Lm = pow(L, pq_m1);
		float N = (pq_c1 + pq_c2 * Lm) / (1.0f + pq_c3 * Lm);
		N = pow(N, pq_m2);
		return N;
	}

	float pq_r_generalized(float C, float _pq_C)
	{
		float L = C / _pq_C;
		float Lm = pow(L, pq_m1);
		float N = (pq_c1 + pq_c2 * Lm) / (1.0 + pq_c3 * Lm);
		N = pow(N, pq_m2);
		return N;
	}

	nv::vec3f pq_r_f3(const nv::vec3f& In)
	{
		// converts from linear cd/m^2 to PQ code values

		nv::vec3f Out;
		Out[0] = pq_r(In[0]);
		Out[1] = pq_r(In[1]);
		Out[2] = pq_r(In[2]);

		return Out;
	}

	nv::vec3f pq_r_f3_generalized(nv::vec3f iVal, float _pq_C)
	{
		// converts from linear cd/m^2 to PQ code values

		nv::vec3f oVal;
		oVal[0] = pq_r_generalized(iVal[0], _pq_C);
		oVal[1] = pq_r_generalized(iVal[1], _pq_C);
		oVal[2] = pq_r_generalized(iVal[2], _pq_C);

		return oVal;
	}

	nv::vec3f pq_f_f3(const nv::vec3f& In)
	{
		// converts from PQ code values to linear cd/m^2

		nv::vec3f Out;
		Out[0] = pq_f(In[0]);
		Out[1] = pq_f(In[1]);
		Out[2] = pq_f(In[2]);

		return Out;
	}



	float glow_fwd(float ycIn, float glowGainIn, float glowMid)
	{
		float glowGainOut;

		if (ycIn <= 2.f / 3.f * glowMid) {
			glowGainOut = glowGainIn;
		}
		else if (ycIn >= 2.f * glowMid) {
			glowGainOut = 0.f;
		}
		else {
			glowGainOut = glowGainIn * (glowMid / ycIn - 1.f / 2.f);
		}

		return glowGainOut;
	}

	float cubic_basis_shaper(float x, float w   /* full base width of the shaper function (in degrees)*/)
	{
		const float M[4][4] =
		{
			{ -1.f / 6, 3.f / 6, -3.f / 6, 1.f / 6 },
			{ 3.f / 6, -6.f / 6, 3.f / 6, 0.f / 6 },
			{ -3.f / 6, 0.f / 6, 3.f / 6, 0.f / 6 },
			{ 1.f / 6, 4.f / 6, 1.f / 6, 0.f / 6 }
		};

		float knots[5] =
		{
			-w / 2.f,
			-w / 4.f,
			0.f,
			w / 4.f,
			w / 2.f
		};

		// EHart - init y, because CTL does by default
		float y = 0;
		if ((x > knots[0]) && (x < knots[4])) {
			float knot_coord = (x - knots[0]) * 4.f / w;
			int j = int(knot_coord);
			float t = knot_coord - j;

			float monomials[4] = { t*t*t, t*t, t, 1. };

			// (if/else structure required for compatibility with CTL < v1.5.)
			if (j == 3) {
				y = monomials[0] * M[0][0] + monomials[1] * M[1][0] +
					monomials[2] * M[2][0] + monomials[3] * M[3][0];
			}
			else if (j == 2) {
				y = monomials[0] * M[0][1] + monomials[1] * M[1][1] +
					monomials[2] * M[2][1] + monomials[3] * M[3][1];
			}
			else if (j == 1) {
				y = monomials[0] * M[0][2] + monomials[1] * M[1][2] +
					monomials[2] * M[2][2] + monomials[3] * M[3][2];
			}
			else if (j == 0) {
				y = monomials[0] * M[0][3] + monomials[1] * M[1][3] +
					monomials[2] * M[2][3] + monomials[3] * M[3][3];
			}
			else {
				y = 0.0f;
			}
		}

		return y * 3 / 2.f;
	}

	float sign(float x)
	{
		if (x < 0.0f)
			return -1.0f;
		if (x > 0.0f)
			return 1.0f;
		return 0.0f;
	}


	float sigmoid_shaper(float x)
	{
		// Sigmoid function in the range 0 to 1 spanning -2 to +2.

		float t = std::max(1.f - abs(x / 2.f), 0.f);
		float y = 1.f + sign(x) * (1.f - t * t);

		return y / 2.f;
	}

	float center_hue(float hue, float centerH)
	{
		float hueCentered = hue - centerH;
		if (hueCentered < -180.f) hueCentered = hueCentered + 360.f;
		else if (hueCentered > 180.f) hueCentered = hueCentered - 360.f;
		return hueCentered;
	}
	
	// Reference Rendering Transform
	nv::vec3f rrt(const nv::vec3f &rgbIn)
	{

		// "Glow" module constants
		const float RRT_GLOW_GAIN = 0.05f;
		const float RRT_GLOW_MID = 0.08f;
		// --- Glow module --- //
		float saturation = rgb_2_saturation(rgbIn);
		float ycIn = rgb_2_yc(rgbIn);
		float s = sigmoid_shaper((saturation - 0.4f) / 0.2f);
		float addedGlow = 1.f + glow_fwd(ycIn, RRT_GLOW_GAIN * s, RRT_GLOW_MID);

		nv::vec3f aces = addedGlow * rgbIn;


		// Red modifier constants
		const float RRT_RED_SCALE = 0.82f;
		const float RRT_RED_PIVOT = 0.03f;
		const float RRT_RED_HUE = 0.f;
		const float RRT_RED_WIDTH = 135.f;
		// --- Red modifier --- //
		float hue = rgb_2_hue(aces);
		float centeredHue = center_hue(hue, RRT_RED_HUE);
		float hueWeight = cubic_basis_shaper(centeredHue, RRT_RED_WIDTH);

		aces[0] = aces[0] + hueWeight * saturation *(RRT_RED_PIVOT - aces[0]) * (1.f - RRT_RED_SCALE);


		// --- ACES to RGB rendering space --- //
		aces = max(aces, 0.0f);  // avoids saturated negative colors from becoming positive in the matrix



		nv::vec3f rgbPre = mul(AP0_2_AP1_MAT, aces);

		rgbPre = clamp(rgbPre, 0.f, Half_Max);

		// Desaturation contants
		const float RRT_SAT_FACTOR = 0.96f;
		const matrix3 RRT_SAT_MAT = calc_sat_adjust_matrix(RRT_SAT_FACTOR, AP1_RGB2Y);
		// --- Global desaturation --- //
		rgbPre = mul(RRT_SAT_MAT, rgbPre);


		// --- Apply the tonescale independently in rendering-space RGB --- //
		nv::vec3f rgbPost;
		rgbPost[0] = segmented_spline_c5_fwd(rgbPre[0]);
		rgbPost[1] = segmented_spline_c5_fwd(rgbPre[1]);
		rgbPost[2] = segmented_spline_c5_fwd(rgbPre[2]);

		// --- RGB rendering space to OCES --- //
		nv::vec3f rgbOces = mul(AP1_2_AP0_MAT, rgbPost);

		return rgbOces;
	}



	float segmented_spline_c9_fwd(float x, SegmentedSplineParams_c9 C)
	{
		static const nv::vec3f M[3] =
		{
			{ 0.5f, -1.0f, 0.5f },
			{ -1.0f, 1.0f, 0.5f },
			{ 0.5f, 0.0f, 0.0f }
		};

		const int N_KNOTS_LOW = 8;
		const int N_KNOTS_HIGH = 8;

		// Check for negatives or zero before taking the log. If negative or zero,
		// set to OCESMIN.
		float xCheck = x;
		if (xCheck <= 0.0) xCheck = 1e-4f;

		float logx = log10(xCheck);

		float logy;

		if (logx <= log10(C.minPoint.x)) {

			logy = logx * C.slope.x + (log10(C.minPoint.y) - C.slope.x * log10(C.minPoint.x));

		}
		else if ((logx > log10(C.minPoint.x)) && (logx < log10(C.midPoint.x))) {

			float knot_coord = (N_KNOTS_LOW - 1) * (logx - log10(C.minPoint.x)) / (log10(C.midPoint.x) - log10(C.minPoint.x));
			int j = int(knot_coord);
			float t = knot_coord - j;

			nv::vec3f cf = { C.coefs[j].x, C.coefs[j + 1].x, C.coefs[j + 2].x };

			nv::vec3f monomials = { t * t, t, 1. };

			nv::vec3f basis = cf.x * M[0] + cf.y * M[1] + cf.z * M[2];
			logy = nv::dot(monomials, basis);

		}
		else if ((logx >= log10(C.midPoint.x)) && (logx < log10(C.maxPoint.x))) {

			float knot_coord = (N_KNOTS_HIGH - 1) * (logx - log10(C.midPoint.x)) / (log10(C.maxPoint.x) - log10(C.midPoint.x));
			int j = int(knot_coord);
			float t = knot_coord - j;

			nv::vec3f cf = { C.coefs[j].y, C.coefs[j + 1].y, C.coefs[j + 2].y };

			nv::vec3f monomials = { t * t, t, 1. };

			nv::vec3f basis = cf.x * M[0] + cf.y * M[1] + cf.z * M[2];
			logy = nv::dot(monomials, basis);

		}
		else { //if ( logIn >= log10(C.maxPoint.X) ) { 

			logy = logx * C.slope.y + (log10(C.maxPoint.y) - C.slope.y * log10(C.maxPoint.x));

		}

		return pow10(logy);
	}



	static const matrix3 D65_2_D60_CAT =
	{
		1.01303f, 0.00610531f, -0.014971f,
		0.00769823f, 0.998165f, -0.00503203f,
		-0.00284131f, 0.00468516f, 0.924507f,
	};
	static const matrix3 sRGB_2_XYZ_MAT =
	{
		0.41239089f, 0.35758430f, 0.18048084f,
		0.21263906f, 0.71516860f, 0.07219233f,
		0.01933082f, 0.11919472f, 0.95053232f
	};
	const matrix3 XYZ_2_sRGB_MAT =
	{
		3.24096942f, -1.53738296f, -0.49861076f,
		-0.96924388f, 1.87596786f, 0.04155510f,
		0.05563002f, -0.20397684f, 1.05697131f,
	};
	const matrix3 XYZ_2_DCI_MAT =
	{
		2.493181f, -0.931265f, -0.402660f,
		-0.829503f, 1.762694f, 0.023625f,
		0.035854f, -0.076189f, 0.957093f,
	};
	const matrix3 XYZ_2_bt2020_MAT =
	{
		1.716511f, -0.355642f, -0.253346f,
		-0.666693f, 1.616502f, 0.015769f,
		0.017644f, -0.042780f, 0.942305f,
	};

	const nv::vec3f Colors[] = {
		{ 0.0f, 0.0f, 0.0f },
		{ 0.0f, 0.0f, 1.0f },
		{ 0.0f, 1.0f, 1.0f },
		{ 0.0f, 1.0f, 0.0f },
		{ 1.0f, 1.0f, 0.0f },
		{ 1.0f, 0.0f, 0.0f },
		{ 1.0f, 0.0f, 1.0f },
		{ 1.0f, 1.0f, 1.0f },
		{ 1.0f, 1.0f, 1.0f },
	};

	static const float DISPGAMMA = 2.4f;
	static const float OFFSET = 0.055f;

	inline float moncurve_baked_r(float y)
	{
		// Reverse monitor curve
		float x;
		static const float yb = pow(OFFSET * DISPGAMMA / ((DISPGAMMA - 1.0f) * (1.0f + OFFSET)), DISPGAMMA);
		static const float rs = pow((DISPGAMMA - 1.0f) / OFFSET, DISPGAMMA - 1.0f) * pow((1.0f + OFFSET) / DISPGAMMA, DISPGAMMA);
		if (y >= yb)
			x = (1.0f + OFFSET) * pow(y, 1.0f / DISPGAMMA) - OFFSET;
		else
			x = y * rs;
		return x;
	}

#pragma warning(disable : 4702)

	nv::vec3f ODT_generalized_uniform(nv::vec3f oces, const ACESparams Params)
	{
		// OCES to RGB rendering space
		nv::vec3f rgbPre = mul(AP0_2_AP1_MAT, oces);

		nv::vec3f rgbPost;

		if (Params.tonemapLuminance)
		{
			// luminance only path, for content that has been mastered for an expectation of an oversaturated tonemap operator
			float y = nv::dot(rgbPre, AP1_RGB2Y);
			float scale = segmented_spline_c9_fwd(y, Params.C) / y;

			// compute the more desaturated per-channel version
			rgbPost[0] = segmented_spline_c9_fwd(rgbPre[0], Params.C);
			rgbPost[1] = segmented_spline_c9_fwd(rgbPre[1], Params.C);
			rgbPost[2] = segmented_spline_c9_fwd(rgbPre[2], Params.C);

			// lerp between values
			rgbPost = max(lerp(rgbPost, rgbPre * scale, Params.saturationLevel), Params.CinemaLimits.x); // clamp to min to prevent the generation of negative values
		}
		else
		{
			// Apply the tonescale independently in rendering-space RGB
			rgbPost[0] = segmented_spline_c9_fwd(rgbPre[0], Params.C);
			rgbPost[1] = segmented_spline_c9_fwd(rgbPre[1], Params.C);
			rgbPost[2] = segmented_spline_c9_fwd(rgbPre[2], Params.C);
		}

		// Scale luminance to linear code value
		nv::vec3f linearCV;
		const float CINEMA_WHITE = Params.CinemaLimits.y;
		const float CINEMA_BLACK = Params.CinemaLimits.x;

		// Transform from CINEMA_WHITE - CINEMA_BLACK range to 0-1
		linearCV[0] = Y_2_linCV(rgbPost[0], CINEMA_WHITE, CINEMA_BLACK);
		linearCV[1] = Y_2_linCV(rgbPost[1], CINEMA_WHITE, CINEMA_BLACK);
		linearCV[2] = Y_2_linCV(rgbPost[2], CINEMA_WHITE, CINEMA_BLACK);

		if (Params.surroundAdjust)
		{
			// Apply gamma adjustment to compensate for surround
			linearCV = alter_surround(linearCV, Params.surroundGamma);
		}

		if (Params.desaturate)
		{
			// Apply desaturation to compensate for luminance difference
			// Saturation compensation factor
			const float ODT_SAT_FACTOR = 0.93f;
			const matrix3 ODT_SAT_MAT = calc_sat_adjust_matrix(ODT_SAT_FACTOR, AP1_RGB2Y);
			linearCV = mul(ODT_SAT_MAT, linearCV);
		}

		// Convert to display primary encoding
		// Rendering space RGB to XYZ
		nv::vec3f XYZ = mul(AP1_2_XYZ_MAT, linearCV);

		if (Params.applyCAT)
		{
			// Apply CAT from ACES white point to assumed observer adapted white point
			// EHart - should recompute this matrix
			const matrix3 D60_2_D65_CAT =
			{
				0.987224f, -0.00611327f, 0.0159533f,
				-0.00759836f, 1.00186f, 0.00533002f,
				0.00307257f, -0.00509595f, 1.08168f,
			};
			XYZ = mul(D60_2_D65_CAT, XYZ);
		}

		// CIE XYZ to display primaries
		linearCV = mul(Params.XYZ_2_DISPLAY_PRI_MAT, XYZ);

		return linearCV;
	}

	//
	//  EvalACES
	//
	/////////////////////////////////////////////////////////////////////////////////////////
	nv::vec3f EvalACES(nv::vec3f InColor, const ACESparams& Params)
	{
		// Convert from sRGB to ACES color space while applying D60 -> D65 white point
		nv::vec3f aces = mul(XYZ_2_AP0_MAT, mul(D65_2_D60_CAT, mul(sRGB_2_XYZ_MAT, InColor)));

		// Transform to reference display space
		nv::vec3f oces = rrt(aces);

		// Encode linear code values with transfer function (also brings values to 0-1 range)
		nv::vec3f linearCV = ODT_generalized_uniform(oces, Params);
		nv::vec3f outputCV = linearCV;

		if (Params.OutputMode == Aces::Output_Mode_Gamma_Correct)
		{
			// LDR mode, clamp 0/1 and encode 
			linearCV = clamp(linearCV, 0., 1.);

			outputCV[0] = moncurve_r(linearCV[0], DISPGAMMA, OFFSET);
			outputCV[1] = moncurve_r(linearCV[1], DISPGAMMA, OFFSET);
			outputCV[2] = moncurve_r(linearCV[2], DISPGAMMA, OFFSET);
		}
		else if (Params.OutputMode == Aces::OutputMode::Output_Mode_scRGB)
		{
			// scRGB
			const float CINEMA_WHITE = Params.CinemaLimits.y;
			const float CINEMA_BLACK = Params.CinemaLimits.x;
			// convert values from range 0-1 to CINEMA_BLACK - CINEMA_WHITE
			linearCV[0] = linCV_2_Y(linearCV[0], CINEMA_WHITE, CINEMA_BLACK);
			linearCV[1] = linCV_2_Y(linearCV[1], CINEMA_WHITE, CINEMA_BLACK);
			linearCV[2] = linCV_2_Y(linearCV[2], CINEMA_WHITE, CINEMA_BLACK);

			linearCV = max(linearCV, 0.0f);

			linearCV = mul(XYZ_2_sRGB_MAT, mul(Params.DISPLAY_PRI_MAT_2_XYZ, linearCV));
			// map 1.0 to 80 nits (or max nit level if it is lower)
			outputCV = linearCV * (1.0f / std::min(80.0f, CINEMA_WHITE));
		}
		else if (Params.OutputMode == Aces::Output_Mode_Linear)
		{
			// LDR mode, clamp 0/1 and encode 
			outputCV = clamp(linearCV, 0., 1.);
		}
		else if (Params.OutputMode == Aces::Output_Mode_Rec2020_Linear)
		{
			//Rec2020
			const float CINEMA_WHITE = Params.CinemaLimits.y;
			const float CINEMA_BLACK = Params.CinemaLimits.x;
			// convert values from range 0-1 to CINEMA_BLACK - CINEMA_WHITE
			linearCV[0] = linCV_2_Y(linearCV[0], CINEMA_WHITE, CINEMA_BLACK);
			linearCV[1] = linCV_2_Y(linearCV[1], CINEMA_WHITE, CINEMA_BLACK);
			linearCV[2] = linCV_2_Y(linearCV[2], CINEMA_WHITE, CINEMA_BLACK);

			linearCV = max(linearCV, 0.0f);

			linearCV = mul(XYZ_2_bt2020_MAT, mul(Params.DISPLAY_PRI_MAT_2_XYZ, linearCV));
			// map 1.0 to 80 nits (or max nit level if it is lower)
			outputCV = linearCV * (1.0f / std::min(80.0f, CINEMA_WHITE));
		}
		else if (Params.OutputMode == Aces::Output_Mode_Rec2020_PQ) {
			//Rec2020 + PQ			
			linearCV = max(linearCV, 0.0f);

			const float CINEMA_WHITE = Params.CinemaLimits.y;

			const float ACES_max = Params.C.maxPoint.y;
			const float pqScale = std::max(CINEMA_WHITE, 80.0f) / 1000.0f;

			linearCV = mul(XYZ_2_bt2020_MAT, mul(Params.DISPLAY_PRI_MAT_2_XYZ, linearCV));
			linearCV = clamp(linearCV * pqScale, 0.0f, 1.0f);

			// map 1.0 to 80 nits (or max nit level if it is lower)
			outputCV = pq_r_f3_generalized(linearCV, 1.0f);
			outputCV = clamp(outputCV, 0.0f, 1.0f);
		}
		else if (Params.OutputMode == Aces::Output_Mode_Luminance) {
			// Luminance Visualization
			const float CINEMA_WHITE = Params.CinemaLimits.y;
			const float CINEMA_BLACK = Params.CinemaLimits.x;
			// convert values from range 0-1 to CINEMA_BLACK - CINEMA_WHITE
			linearCV[0] = linCV_2_Y(linearCV[0], CINEMA_WHITE, CINEMA_BLACK);
			linearCV[1] = linCV_2_Y(linearCV[1], CINEMA_WHITE, CINEMA_BLACK);
			linearCV[2] = linCV_2_Y(linearCV[2], CINEMA_WHITE, CINEMA_BLACK);

			float LogLum = log2(max(max(linearCV[0], linearCV[1]), linearCV[2]));
			float Scale = LogLum * 0.5f + 2.0f;

			Scale = std::min(Scale, 7.0f);
			Scale = std::max(Scale, 0.0f);

			int index = int(Scale);

			linearCV = lerp(Colors[index], Colors[index + 1], Scale - index);
			
			outputCV = linearCV;
		}

		return outputCV;
	}

};

static const float ColorMatrices[12 * 3] =
{
	// rec 709
	3.24096942f, -1.53738296f, -0.49861076f, 0.0f,
	-0.96924388f, 1.87596786f, 0.04155510f, 0.0f,
	0.05563002f, -0.20397684f, 1.05697131f, 0.0f,
	// DCI-P3
	2.72539496f, -1.01800334f, -0.44016343f, 0.0f,
	-0.79516816f, 1.68973231f, 0.02264720f, 0.0f,
	0.04124193f, -0.08763910f, 1.10092998f, 0.0f,
	// BT2020
	1.71665096f, -0.35567081f, -0.25336623f, 0.0f,
	-0.66668433f, 1.61648130f, 0.01576854f, 0.0f,
	0.01763985f, -0.04277061f, 0.94210327f, 0.0f,
};

static const float ColorMatricesInv[12 * 3] =
{
	//rec709 to XYZ
	0.41239089f, 0.35758430f, 0.18048084f, 0.0f,
	0.21263906f, 0.71516860f, 0.07219233f, 0.0f,
	0.01933082f, 0.11919472f, 0.95053232f, 0.0f,
	//DCI - P3 2 XYZ
	0.44516969f, 0.27713439f, 0.17228261f, 0.0f,
	0.20949161f, 0.72159523f, 0.06891304f, 0.0f,
	0.00000000f, 0.04706058f, 0.90735501f, 0.0f,
	//bt2020 2 XYZ
	0.63695812f, 0.14461692f, 0.16888094f, 0.0f,
	0.26270023f, 0.67799807f, 0.05930171f, 0.0f,
	0.00000000f, 0.02807269f, 1.06098485f, 0.0f
};

float shaper_func_log2(float val, float bias, float scale) {
	return pow(2.0f, ((val - bias) / scale));
}

float shaper_func_log10(float val, float bias, float scale) {
	return pow(10.0f, ((val - bias) / scale));
}

void Aces::Apply1000nitHDR()
{
	mCurrentParameters.selectedCurve = ODT_1000Nit_Adj;
	mCurrentParameters.minStops = -12.0f;
	mCurrentParameters.maxStops = 10.0f;
	mCurrentParameters.midGrayScale = 1.0f;
	mCurrentParameters.adjustWP = true;
	mCurrentParameters.desaturate = false;
	mCurrentParameters.selectedColorMatrix = Aces::Output_Color_Primaries_Rec2020;
	mCurrentParameters.outputMode = Aces::Output_Mode_scRGB; // scRGB
}

void Aces::Apply1000nitHDRSharpened()
{
	mCurrentParameters.selectedCurve = ODT_1000Nit_Adj;
	mCurrentParameters.minStops = -8.0f;
	mCurrentParameters.maxStops = 8.0f;
	mCurrentParameters.midGrayScale = 1.0f;
	mCurrentParameters.adjustWP = true;
	mCurrentParameters.desaturate = false;
	mCurrentParameters.selectedColorMatrix = Aces::Output_Color_Primaries_Rec2020;
	mCurrentParameters.outputMode = Aces::Output_Mode_scRGB; // scRGB
}

void Aces::ApplySDR()
{
	mCurrentParameters.selectedCurve = ODT_LDR_Adj;
	mCurrentParameters.minStops = -6.5f;
	mCurrentParameters.maxStops = 6.5f;
	mCurrentParameters.midGrayScale = 1.0f;
	mCurrentParameters.adjustWP = true;
	mCurrentParameters.desaturate = true;
	mCurrentParameters.selectedColorMatrix = Aces::Output_Color_Primaries_Rec709;
	mCurrentParameters.outputMode = Aces::Output_Mode_Gamma_Correct; // sRGB
}

void Aces::ApplyEDRExtreme()
{
	mCurrentParameters.selectedCurve = ODT_1000Nit_Adj;
	mCurrentParameters.minStops = -12.0f;
	mCurrentParameters.maxStops = 9.0f;
	mCurrentParameters.midGrayScale = 1.0f;
	mCurrentParameters.adjustWP = true;
	mCurrentParameters.desaturate = false;
	mCurrentParameters.selectedColorMatrix = Aces::Output_Color_Primaries_Rec709;
	mCurrentParameters.outputMode = Aces::Output_Mode_Gamma_Correct; // sRGB
}

void Aces::ApplyEDR()
{
	mCurrentParameters.selectedCurve = ODT_1000Nit_Adj;
	mCurrentParameters.minStops = -8.0f;
	mCurrentParameters.maxStops = 8.0f;
	mCurrentParameters.midGrayScale = 3.0f;
	mCurrentParameters.adjustWP = true;
	mCurrentParameters.desaturate = false;
	mCurrentParameters.selectedColorMatrix = Aces::Output_Color_Primaries_Rec709;
	mCurrentParameters.outputMode = Aces::Output_Mode_Gamma_Correct; // sRGB
}

void Aces::ApplyNvidia()
{
	mCurrentParameters.selectedCurve = ODT_1000Nit_Adj;
	mCurrentParameters.minStops = -8.0f;
	mCurrentParameters.maxStops = 8.0f;
	mCurrentParameters.midGrayScale = 1.0f; //1.94
	mCurrentParameters.adjustWP = true;
	mCurrentParameters.desaturate = false;
	mCurrentParameters.selectedColorMatrix = Aces::Output_Color_Primaries_Rec2020;
	mCurrentParameters.outputMode = Aces::Output_Mode_scRGB; // scRGB

	mCurrentParameters.maxLevel = -1.0f;
	mCurrentParameters.surroundGamma = 1.0f;
	mCurrentParameters.toneCurveSaturation = 1.0f;
	mCurrentParameters.dimSurround = true;
}

Aces::Aces() : hdrEnabled(false) {

	// Parameters for tonemapper
	mAcesParams = new GLfloat[4];
	mAcesParams[0] = 0.0f;						// Aces lut scale
	mAcesParams[1] = 0.0f;						// Aces lut bias
	mAcesParams[2] = 0.0f;						// Aces sigmoid shaper function
	mAcesParams[3] = 0.0f;						// HDR compatiblity

	// Apply default preset
	Apply1000nitHDRSharpened();
}

Aces::~Aces() {
	delete[] mAcesParams;
}

void Aces::generateAcesTonemapLUT(void *data, GLfloat* aces_parameters)
{
	int selectedColorMatrix = 0;

	ACESparams params;
	params.C = GetAcesData(mCurrentParameters.selectedCurve, mCurrentParameters.minStops, mCurrentParameters.maxStops,
		mCurrentParameters.maxLevel, mCurrentParameters.midGrayScale);

	selectedColorMatrix = selectedColorMatrix;

	params.OutputMode = mCurrentParameters.outputMode;
	params.applyCAT = mCurrentParameters.adjustWP;
	params.surroundAdjust = mCurrentParameters.dimSurround;
	params.CinemaLimits.x = params.C.minPoint.y;
	params.CinemaLimits.y = params.C.maxPoint.y;
	params.desaturate = mCurrentParameters.desaturate;
	params.surroundGamma = mCurrentParameters.surroundGamma;
	params.saturationLevel = mCurrentParameters.toneCurveSaturation;

	memcpy(&params.XYZ_2_DISPLAY_PRI_MAT, &ColorMatrices[selectedColorMatrix * 12], sizeof(float)* 3);
	memcpy(&(params.XYZ_2_DISPLAY_PRI_MAT.m[3]), &ColorMatrices[selectedColorMatrix * 12 + 4], sizeof(float)* 3);
	memcpy(&(params.XYZ_2_DISPLAY_PRI_MAT.m[6]), &ColorMatrices[selectedColorMatrix * 12 + 8], sizeof(float)* 3);
	memcpy(&params.DISPLAY_PRI_MAT_2_XYZ, &ColorMatricesInv[selectedColorMatrix * 12], sizeof(float)* 3);
	memcpy(&(params.DISPLAY_PRI_MAT_2_XYZ.m[3]), &ColorMatricesInv[selectedColorMatrix * 12 + 4], sizeof(float)* 3);
	memcpy(&(params.DISPLAY_PRI_MAT_2_XYZ.m[6]), &ColorMatricesInv[selectedColorMatrix * 12 + 8], sizeof(float)* 3);

	float scale, bias;
	float log_min = log10(std::max(params.C.limits.x, 0.0000001f));
	float log_max = log10(params.C.limits.y);

	scale = 1.0f / (log_max - log_min);
	bias = -(scale * log_min);

	aces_parameters[0] = scale;
	aces_parameters[1] = bias;
	aces_parameters[2] = mCurrentParameters.shaper; // Used for log encoding vs PQ. We use only log. Default 0.

	const int size_x = 32;
	const int size_y = 32;
	const int size_z = 32;
	unsigned short* walk = (unsigned short*)data;

	for (int i = 0; i < size_z; i++) {
		for (int j = 0; j < size_y; j++) {
			for (int k = 0; k < size_x; k++) {
				float x = k / float(size_x);
				float y = j / float(size_y);
				float z = i / float(size_z);

				x = shaper_func_log10(x, bias, scale);
				y = shaper_func_log10(y, bias, scale);
				z = shaper_func_log10(z, bias, scale);

				nv::vec3f color = { x, y, z };
				nv::vec3f temp = EvalACES(color, params);

				walk[0] = float2half(temp.x);
				walk[1] = float2half(temp.y);
				walk[2] = float2half(temp.z);
				walk[3] = 0x3b00; // 1.0 half

				walk += 4;
			}
		}
	}
}

void Aces::rebuildAcesLUT(int preset) {

	switch (preset) {
	case Aces::Set_1000NitHDR:
		Apply1000nitHDR();
		break;
	case Aces::Set_1000NitHDRSharpened:
		Apply1000nitHDRSharpened();
		break;
	case Aces::Set_EDR:
		ApplyEDR();
		break;
	case Aces::Set_EDRExtreme:
		ApplyEDRExtreme();
		break;
	case Aces::Set_Nvidia:
		ApplyNvidia();
		break;
	case Aces::Set_SDR:
		ApplySDR();
		break;
	default:
		ApplyNvidia();
	}

	unsigned char * data = new unsigned char[32 * 32 * 32 * 4 * 2];	

	const float log_min_val = -3.45399745587f; //2^-9 *0.18
	const float log_max_val = 2.26557246174f; //2^10 *0.18
	float aces_lut_scale = 1.0f / (log_max_val - log_min_val);
	mAcesParams[0] = aces_lut_scale;
	float aces_lut_bias = -log_min_val * aces_lut_scale;
	mAcesParams[1] = aces_lut_bias;
	Aces::generateAcesTonemapLUT(data, mAcesParams);
	glGenTextures(1, &mAcesLutTex);
	glBindTexture(GL_TEXTURE_3D, mAcesLutTex);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAX_LEVEL, 0);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA16F, 32, 32, 32, 0, GL_RGBA, GL_HALF_FLOAT, data);
	glBindTexture(GL_TEXTURE_3D, 0);

	delete[] data;
}

void Aces::rebuildAcesLUT(const AcesParameters &parameters) {

	mCurrentParameters = parameters;

	unsigned char * data = new unsigned char[32 * 32 * 32 * 4 * 2];

	const float log_min_val = -3.45399745587f; //2^-9 *0.18
	const float log_max_val = 2.26557246174f; //2^10 *0.18
	float aces_lut_scale = 1.0f / (log_max_val - log_min_val);
	mAcesParams[0] = aces_lut_scale;
	float aces_lut_bias = -log_min_val * aces_lut_scale;
	mAcesParams[1] = aces_lut_bias;
	Aces::generateAcesTonemapLUT(data, mAcesParams);
	glGenTextures(1, &mAcesLutTex);
	glBindTexture(GL_TEXTURE_3D, mAcesLutTex);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAX_LEVEL, 0);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA16F, 32, 32, 32, 0, GL_RGBA, GL_HALF_FLOAT, data);
	glBindTexture(GL_TEXTURE_3D, 0);

	delete[] data;
}

void Aces::updateAcesUniforms(int program, const int texUnit3D) {

	//TODO: Remove all references to DEBUG -shaveenk
#if defined(DEBUG)
	mAcesParams[3] = 1.0f;
#endif

#if defined(ANDROID) 
	mAcesParams[3] = getHDREnabled() ? 1.0f : 0.0f;
#endif



	int loc = glGetUniformLocation(program, "uAcesParams");
	glProgramUniform4fv(program, loc, 1, mAcesParams);

	glActiveTexture(GL_TEXTURE0 + texUnit3D);
	glBindTexture(GL_TEXTURE_3D, mAcesLutTex);
	loc = glGetUniformLocation(program, "uAcesLUT");
	glProgramUniform1i(program, loc, texUnit3D);
}