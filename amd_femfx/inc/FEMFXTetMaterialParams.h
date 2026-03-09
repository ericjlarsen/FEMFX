/*
MIT License

Copyright (c) 2019 Advanced Micro Devices, Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

//---------------------------------------------------------------------------------------
// Tetrahedron material parameters
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommon.h"
#include "FEMFXVectorMath.h"

namespace AMD
{
    // Parameters that define the material of a tetrahedron
    struct FmTetMaterialParams
    {
        float restDensity = 50.0f;               // Density at rest
        float youngsModulus = 1.0e5f;            // Greater values will increase stiffness of material
        float poissonsRatio = 0.25f;             // Value must be < 0.5.  Defines how much material bulges when compressed, with 0 causing none.  Values closer to 0.5 worsen conditioning and require more iterations.
        float plasticYieldThreshold = 0.0f;      // Threshold for stress magnitude where plastic deformation starts.
        float plasticCreep = 0.0f;               // Value >= 0 and <=1.  Portion of elastic deformation converted to plastic is creep * (stress_mag - yield)/stress_mag
        float plasticMin = 0.5f;                 // Value > 0 and <= 1.  Minimum scale of compression from plastic deformation.  Smaller values allow greater plastic deformation but may worsen conditioning.
        float plasticMax = 2.0f;                 // Value >= 1.  Maximum scale of stretch from plastic deformation.   Larger values allow greater plastic deformation but may worsen conditioning.
        float fractureStressThreshold = 5.0e3f;  // Threshold for stress max eigenvalue where fracture occurs
        uint  maxUnconstrainedSolveIterations = FM_DEFAULT_MAX_CG_ITERATIONS;  // Maximum number of CG iterations to use with this material

        float lowerDeformationLimit = 0.0f;    // Value > 0 and <= 1, or unlimited if = 0.  Constrains minimum scale of deformation.
        float upperDeformationLimit = 0.0f;    // Value >= 1, or unlimited if = 0.  Constrains maximum scale of deformation.
        bool  adjustShapeParams = true;        // Adjusts shape parameters of narrow tetrahedra to improve convergence.

        inline FmTetMaterialParams() {}

        inline FmTetMaterialParams(
            float inRestDensity, float inYoungsModulus, float inPoissonsRatio,
            float inPlasticYieldThreshold, float inPlasticStrainCreep,
            float inPlasticStrainMin, float inPlasticStrainMax,
            float inFractureStressThreshold, uint inMaxCGIterations,
            float inLowerDeformationLimit = 0.0f, float inUpperDeformationLimit = 0.0f, float inAdjustShapeParams = false)
        {
            restDensity = inRestDensity;
            youngsModulus = inYoungsModulus;
            poissonsRatio = inPoissonsRatio;
            plasticYieldThreshold = inPlasticYieldThreshold;
            plasticCreep = inPlasticStrainCreep;
            plasticMin = inPlasticStrainMin;
            plasticMax = inPlasticStrainMax;
            fractureStressThreshold = inFractureStressThreshold;
            maxUnconstrainedSolveIterations = inMaxCGIterations;
            lowerDeformationLimit = inLowerDeformationLimit;
            upperDeformationLimit = inUpperDeformationLimit;
            adjustShapeParams = inAdjustShapeParams;
        }
    };
}

