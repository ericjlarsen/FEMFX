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
// Tetrahedron-specific types and operations for the FEM model
//---------------------------------------------------------------------------------------

#include "FEMFXTypes.h"
#include "FEMFXTetMath.h"
#include "FEMFXSvd3x3.h"

namespace AMD
{
    void FmComputeShapeParams(
        FmTetShapeParams* resultShapeParams,
        const FmVector3& p0,
        const FmVector3& p1,
        const FmVector3& p2,
        const FmVector3& p3,
        float minSingularValueRatio, float minSingularValue)
    {
        FmTetShapeParams shapeParams;

        FmMatrix3 Dm;
        Dm.col0 = (p1 - p0);
        Dm.col1 = (p2 - p0);
        Dm.col2 = (p3 - p0);

        FmMatrix3 DmInv;
        bool clamped = false;
        if (minSingularValueRatio > 0.0f || minSingularValue > 0.0f)
        {
            FmMatrix3 U, V;
            FmVector3 sigma;
            FmSvd3x3(&U, &sigma, &V, Dm);

            minSingularValue = FmMaxFloat(sigma.x * minSingularValueRatio, minSingularValue);

            if (sigma.x < minSingularValue)
            {
                sigma.x = minSingularValue;
                clamped = true;
            }
            if (sigma.y < minSingularValue)
            {
                sigma.y = minSingularValue;
                clamped = true;
            }
            if (sigma.z < minSingularValue)
            {
                sigma.z = minSingularValue;
                clamped = true;
            }

            DmInv = V * FmMatrix3::scale(inv(sigma)) * transpose(U);
        }
        else
        {
            DmInv = inverse(Dm);
        }

        shapeParams.DmInv = DmInv;
        shapeParams.clamped = clamped;

        FmVector4 baryCol0(-sum(DmInv.col0), DmInv.col0.x, DmInv.col0.y, DmInv.col0.z);
        FmVector4 baryCol1(-sum(DmInv.col1), DmInv.col1.x, DmInv.col1.y, DmInv.col1.z);
        FmVector4 baryCol2(-sum(DmInv.col2), DmInv.col2.x, DmInv.col2.y, DmInv.col2.z);
        shapeParams.baryCol3 = -(baryCol0 * p0.x + baryCol1 * p0.y + baryCol2 * p0.z);
        shapeParams.baryCol3.x += 1.0f;

        shapeParams.det = fabsf(determinant(Dm));

        *resultShapeParams = shapeParams;
    }

    // Ratio of maximum edge length to minimum height of vertex from plane containing other vertices.
    // Measure of tetrahedron quality.
    float FmComputeTetAspectRatio(const FmVector3& position0, const FmVector3& position1, const FmVector3& position2, const FmVector3& position3)
    {
        FmVector3 edge0 = position0 - position1;
        FmVector3 edge1 = position0 - position2;
        FmVector3 edge2 = position0 - position3;
        FmVector3 edge3 = position1 - position2;
        FmVector3 edge4 = position1 - position3;
        FmVector3 edge5 = position2 - position3;

        float planeDist0 = dot(edge0, normalize(cross(edge5, edge4)));
        float planeDist1 = dot(edge4, normalize(cross(edge2, edge5)));
        float planeDist2 = dot(edge5, normalize(cross(edge4, edge2)));
        float planeDist3 = dot(edge2, normalize(cross(edge3, edge1)));

        float maxEdgeLength =
            FmMaxFloat(FmMaxFloat(FmMaxFloat(FmMaxFloat(FmMaxFloat(length(edge0), length(edge1)), length(edge2)), length(edge3)), length(edge4)), length(edge5));

        float minPlaneDist =
            FmMinFloat(FmMinFloat(FmMinFloat(planeDist0, planeDist1), planeDist2), planeDist3);

        return maxEdgeLength / minPlaneDist;
    }

    // Ratio of maximum edge length to minimum height of vertex from plane containing other vertices.
    // Measure of tetrahedron quality.
    float FmComputeTetAspectRatio(const FmVector3 positions[4])
    {
        return FmComputeTetAspectRatio(positions[0], positions[1], positions[2], positions[3]);
    }
        
    void FmComputeTetStrainMatrix(FmTetStrainMatrix* strainMat, const FmTetShapeParams& shapeParams)
    {
        float a0 = shapeParams.GetA0();
        float a1 = shapeParams.GetA1();
        float a2 = shapeParams.GetA2();
        float a3 = shapeParams.GetA3();
        float b0 = shapeParams.GetB0();
        float b1 = shapeParams.GetB1();
        float b2 = shapeParams.GetB2();
        float b3 = shapeParams.GetB3();
        float c0 = shapeParams.GetC0();
        float c1 = shapeParams.GetC1();
        float c2 = shapeParams.GetC2();
        float c3 = shapeParams.GetC3();

        strainMat->Be00 = FmMatrix3(FmVector3(a0, 0.0f, 0.0f), FmVector3(0.0f, b0, 0.0f), FmVector3(0.0f, 0.0f, c0));
        strainMat->Be01 = FmMatrix3(FmVector3(a1, 0.0f, 0.0f), FmVector3(0.0f, b1, 0.0f), FmVector3(0.0f, 0.0f, c1));
        strainMat->Be02 = FmMatrix3(FmVector3(a2, 0.0f, 0.0f), FmVector3(0.0f, b2, 0.0f), FmVector3(0.0f, 0.0f, c2));
        strainMat->Be03 = FmMatrix3(FmVector3(a3, 0.0f, 0.0f), FmVector3(0.0f, b3, 0.0f), FmVector3(0.0f, 0.0f, c3));
        strainMat->Be10 = FmMatrix3(FmVector3(b0, 0.0f, c0), FmVector3(a0, c0, 0.0f), FmVector3(0.0f, b0, a0));
        strainMat->Be11 = FmMatrix3(FmVector3(b1, 0.0f, c1), FmVector3(a1, c1, 0.0f), FmVector3(0.0f, b1, a1));
        strainMat->Be12 = FmMatrix3(FmVector3(b2, 0.0f, c2), FmVector3(a2, c2, 0.0f), FmVector3(0.0f, b2, a2));
        strainMat->Be13 = FmMatrix3(FmVector3(b3, 0.0f, c3), FmVector3(a3, c3, 0.0f), FmVector3(0.0f, b3, a3));
    }

    void FmComputeTetStrain(
        FmVector3 strain[2],
        const FmTetShapeParams& shapeParams,
        const FmVector3& unrotatedOffset1,
        const FmVector3& unrotatedOffset2,
        const FmVector3& unrotatedOffset3)
    {
        //float a0 = shapeParams.GetA0();
        float a1 = shapeParams.GetA1();
        float a2 = shapeParams.GetA2();
        float a3 = shapeParams.GetA3();
        //float b0 = shapeParams.GetB0();
        float b1 = shapeParams.GetB1();
        float b2 = shapeParams.GetB2();
        float b3 = shapeParams.GetB3();
        //float c0 = shapeParams.GetC0();
        float c1 = shapeParams.GetC1();
        float c2 = shapeParams.GetC2();
        float c3 = shapeParams.GetC3();

        // unrotatedOffset0 is 0
        //float x0 = unrotatedOffset0.x;
        //float y0 = unrotatedOffset0.y;
        //float z0 = unrotatedOffset0.z;
        float x1 = unrotatedOffset1.x;
        float y1 = unrotatedOffset1.y;
        float z1 = unrotatedOffset1.z;
        float x2 = unrotatedOffset2.x;
        float y2 = unrotatedOffset2.y;
        float z2 = unrotatedOffset2.z;
        float x3 = unrotatedOffset3.x;
        float y3 = unrotatedOffset3.y;
        float z3 = unrotatedOffset3.z;        
        
        strain[0].x = a3*x3 + a2*x2 + a1*x1 /* + a0*x0 */;
        strain[0].y = b3*y3 + b2*y2 + b1*y1 /* + b0*y0 */;
        strain[0].z = c3*z3 + c2*z2 + c1*z1 /* + c0*z0 */;
        strain[1].x = a3*y3 + a2*y2 + a1*y1 /* + a0*y0 */ + b3*x3 + b2*x2 + b1*x1 /* + b0*x0 */;
        strain[1].y = b3*z3 + b2*z2 + b1*z1 /* + b0*z0 */ + c3*y3 + c2*y2 + c1*y1 /* + c0*y0 */;
        strain[1].z = a3*z3 + a2*z2 + a1*z1 /* + a0*z0 */ + c3*x3 + c2*x2 + c1*x1 /* + c0*x0 */;
    }

    void FmComputeTetStrainMatrixTransposedSubmats(FmTetStrainMatrix* strainMat, const FmTetShapeParams& shapeParams)
    {
        float a0 = shapeParams.GetA0();
        float a1 = shapeParams.GetA1();
        float a2 = shapeParams.GetA2();
        float a3 = shapeParams.GetA3();
        float b0 = shapeParams.GetB0();
        float b1 = shapeParams.GetB1();
        float b2 = shapeParams.GetB2();
        float b3 = shapeParams.GetB3();
        float c0 = shapeParams.GetC0();
        float c1 = shapeParams.GetC1();
        float c2 = shapeParams.GetC2();
        float c3 = shapeParams.GetC3();

        strainMat->Be00 = transpose(FmMatrix3(FmVector3(a0, 0.0f, 0.0f), FmVector3(0.0f, b0, 0.0f), FmVector3(0.0f, 0.0f, c0)));
        strainMat->Be01 = transpose(FmMatrix3(FmVector3(a1, 0.0f, 0.0f), FmVector3(0.0f, b1, 0.0f), FmVector3(0.0f, 0.0f, c1)));
        strainMat->Be02 = transpose(FmMatrix3(FmVector3(a2, 0.0f, 0.0f), FmVector3(0.0f, b2, 0.0f), FmVector3(0.0f, 0.0f, c2)));
        strainMat->Be03 = transpose(FmMatrix3(FmVector3(a3, 0.0f, 0.0f), FmVector3(0.0f, b3, 0.0f), FmVector3(0.0f, 0.0f, c3)));
        strainMat->Be10 = transpose(FmMatrix3(FmVector3(b0, 0.0f, c0), FmVector3(a0, c0, 0.0f), FmVector3(0.0f, b0, a0)));
        strainMat->Be11 = transpose(FmMatrix3(FmVector3(b1, 0.0f, c1), FmVector3(a1, c1, 0.0f), FmVector3(0.0f, b1, a1)));
        strainMat->Be12 = transpose(FmMatrix3(FmVector3(b2, 0.0f, c2), FmVector3(a2, c2, 0.0f), FmVector3(0.0f, b2, a2)));
        strainMat->Be13 = transpose(FmMatrix3(FmVector3(b3, 0.0f, c3), FmVector3(a3, c3, 0.0f), FmVector3(0.0f, b3, a3)));
    }

    void FmComputeTetStressMatrix(
        FmTetStressMatrix* stressMat,
        const FmMatrix3& DmInv,
        float youngsModulus,
        float poissonsRatio)
    {
        float a0 = -sum(DmInv.col0);
        float a1 = DmInv.col0.x;
        float a2 = DmInv.col0.y;
        float a3 = DmInv.col0.z;
        float b0 = -sum(DmInv.col1);
        float b1 = DmInv.col1.x;
        float b2 = DmInv.col1.y;
        float b3 = DmInv.col1.z;
        float c0 = -sum(DmInv.col2);
        float c1 = DmInv.col2.x;
        float c2 = DmInv.col2.y;
        float c3 = DmInv.col2.z;

        float scale = (youngsModulus / ((1.0f + poissonsRatio)*(1.0f - 2.0f * poissonsRatio)));

        float v0 = scale * (1.0f - poissonsRatio);
        float v1 = scale * poissonsRatio;
        float v2 = scale * (0.5f - poissonsRatio);

        float a0_v0 = a0 * v0;
        float a0_v1 = a0 * v1;
        float a0_v2 = a0 * v2;
        float a1_v0 = a1 * v0;
        float a1_v1 = a1 * v1;
        float a1_v2 = a1 * v2;
        float a2_v0 = a2 * v0;
        float a2_v1 = a2 * v1;
        float a2_v2 = a2 * v2;
        float a3_v0 = a3 * v0;
        float a3_v1 = a3 * v1;
        float a3_v2 = a3 * v2;
        float b0_v0 = b0 * v0;
        float b0_v1 = b0 * v1;
        float b0_v2 = b0 * v2;
        float b1_v0 = b1 * v0;
        float b1_v1 = b1 * v1;
        float b1_v2 = b1 * v2;
        float b2_v0 = b2 * v0;
        float b2_v1 = b2 * v1;
        float b2_v2 = b2 * v2;
        float b3_v0 = b3 * v0;
        float b3_v1 = b3 * v1;
        float b3_v2 = b3 * v2;
        float c0_v0 = c0 * v0;
        float c0_v1 = c0 * v1;
        float c0_v2 = c0 * v2;
        float c1_v0 = c1 * v0;
        float c1_v1 = c1 * v1;
        float c1_v2 = c1 * v2;
        float c2_v0 = c2 * v0;
        float c2_v1 = c2 * v1;
        float c2_v2 = c2 * v2;
        float c3_v0 = c3 * v0;
        float c3_v1 = c3 * v1;
        float c3_v2 = c3 * v2;

        stressMat->EBe00 = FmMatrix3(FmVector3(a0_v0, a0_v1, a0_v1), FmVector3(b0_v1, b0_v0, b0_v1), FmVector3(c0_v1, c0_v1, c0_v0));
        stressMat->EBe01 = FmMatrix3(FmVector3(a1_v0, a1_v1, a1_v1), FmVector3(b1_v1, b1_v0, b1_v1), FmVector3(c1_v1, c1_v1, c1_v0));
        stressMat->EBe02 = FmMatrix3(FmVector3(a2_v0, a2_v1, a2_v1), FmVector3(b2_v1, b2_v0, b2_v1), FmVector3(c2_v1, c2_v1, c2_v0));
        stressMat->EBe03 = FmMatrix3(FmVector3(a3_v0, a3_v1, a3_v1), FmVector3(b3_v1, b3_v0, b3_v1), FmVector3(c3_v1, c3_v1, c3_v0));
        stressMat->EBe10 = FmMatrix3(FmVector3(b0_v2, 0.0f, c0_v2), FmVector3(a0_v2, c0_v2, 0.0f), FmVector3(0.0f, b0_v2, a0_v2));
        stressMat->EBe11 = FmMatrix3(FmVector3(b1_v2, 0.0f, c1_v2), FmVector3(a1_v2, c1_v2, 0.0f), FmVector3(0.0f, b1_v2, a1_v2));
        stressMat->EBe12 = FmMatrix3(FmVector3(b2_v2, 0.0f, c2_v2), FmVector3(a2_v2, c2_v2, 0.0f), FmVector3(0.0f, b2_v2, a2_v2));
        stressMat->EBe13 = FmMatrix3(FmVector3(b3_v2, 0.0f, c3_v2), FmVector3(a3_v2, c3_v2, 0.0f), FmVector3(0.0f, b3_v2, a3_v2));
    }

    void FmComputeTetStressAndStiffnessMatrix(
        FmTetStressMatrix* stressMat, 
        FmTetStiffnessMatrix* stiffnessMat, 
        const FmMatrix3& DmInv,
        float volume,
        float youngsModulus,
        float poissonsRatio)
    {
        float a0 = -sum(DmInv.col0);
        float a1 = DmInv.col0.x;
        float a2 = DmInv.col0.y;
        float a3 = DmInv.col0.z;
        float b0 = -sum(DmInv.col1);
        float b1 = DmInv.col1.x;
        float b2 = DmInv.col1.y;
        float b3 = DmInv.col1.z;
        float c0 = -sum(DmInv.col2);
        float c1 = DmInv.col2.x;
        float c2 = DmInv.col2.y;
        float c3 = DmInv.col2.z;

        float scale = (youngsModulus / ((1.0f + poissonsRatio)*(1.0f - 2.0f * poissonsRatio)));

        float v0 = scale * (1.0f - poissonsRatio);
        float v1 = scale * poissonsRatio;
        float v2 = scale * (0.5f - poissonsRatio);

        float a0_v0 = a0*v0;
        float a0_v1 = a0*v1;
        float a0_v2 = a0*v2;
        float a1_v0 = a1*v0;
        float a1_v1 = a1*v1;
        float a1_v2 = a1*v2;
        float a2_v0 = a2*v0;
        float a2_v1 = a2*v1;
        float a2_v2 = a2*v2;
        float a3_v0 = a3*v0;
        float a3_v1 = a3*v1;
        float a3_v2 = a3*v2;
        float b0_v0 = b0*v0;
        float b0_v1 = b0*v1;
        float b0_v2 = b0*v2;
        float b1_v0 = b1*v0;
        float b1_v1 = b1*v1;
        float b1_v2 = b1*v2;
        float b2_v0 = b2*v0;
        float b2_v1 = b2*v1;
        float b2_v2 = b2*v2;
        float b3_v0 = b3*v0;
        float b3_v1 = b3*v1;
        float b3_v2 = b3*v2;
        float c0_v0 = c0*v0;
        float c0_v1 = c0*v1;
        float c0_v2 = c0*v2;
        float c1_v0 = c1*v0;
        float c1_v1 = c1*v1;
        float c1_v2 = c1*v2;
        float c2_v0 = c2*v0;
        float c2_v1 = c2*v1;
        float c2_v2 = c2*v2;
        float c3_v0 = c3*v0;
        float c3_v1 = c3*v1;
        float c3_v2 = c3*v2;

        stressMat->EBe00 = FmMatrix3(FmVector3(a0_v0, a0_v1, a0_v1), FmVector3(b0_v1, b0_v0, b0_v1), FmVector3(c0_v1, c0_v1, c0_v0));
        stressMat->EBe01 = FmMatrix3(FmVector3(a1_v0, a1_v1, a1_v1), FmVector3(b1_v1, b1_v0, b1_v1), FmVector3(c1_v1, c1_v1, c1_v0));
        stressMat->EBe02 = FmMatrix3(FmVector3(a2_v0, a2_v1, a2_v1), FmVector3(b2_v1, b2_v0, b2_v1), FmVector3(c2_v1, c2_v1, c2_v0));
        stressMat->EBe03 = FmMatrix3(FmVector3(a3_v0, a3_v1, a3_v1), FmVector3(b3_v1, b3_v0, b3_v1), FmVector3(c3_v1, c3_v1, c3_v0));
        stressMat->EBe10 = FmMatrix3(FmVector3(b0_v2, 0.0f, c0_v2), FmVector3(a0_v2, c0_v2, 0.0f), FmVector3(0.0f, b0_v2, a0_v2));
        stressMat->EBe11 = FmMatrix3(FmVector3(b1_v2, 0.0f, c1_v2), FmVector3(a1_v2, c1_v2, 0.0f), FmVector3(0.0f, b1_v2, a1_v2));
        stressMat->EBe12 = FmMatrix3(FmVector3(b2_v2, 0.0f, c2_v2), FmVector3(a2_v2, c2_v2, 0.0f), FmVector3(0.0f, b2_v2, a2_v2));
        stressMat->EBe13 = FmMatrix3(FmVector3(b3_v2, 0.0f, c3_v2), FmVector3(a3_v2, c3_v2, 0.0f), FmVector3(0.0f, b3_v2, a3_v2));

        stiffnessMat->Ke_diag0 = volume * FmMatrix3(FmVector3(c0*c0_v2 + b0*b0_v2 + a0*a0_v0, a0*b0_v2 + a0*b0_v1, a0*c0_v2 + a0*c0_v1), FmVector3(a0*b0_v2 + a0*b0_v1, c0*c0_v2 + a0*a0_v2 + b0*b0_v0, b0*c0_v2 + b0*c0_v1), FmVector3(a0*c0_v2 + a0*c0_v1, b0*c0_v2 + b0*c0_v1, b0*b0_v2 + a0*a0_v2 + c0*c0_v0));
        stiffnessMat->Ke_diag1 = volume * FmMatrix3(FmVector3(c1*c1_v2 + b1*b1_v2 + a1*a1_v0, a1*b1_v2 + a1*b1_v1, a1*c1_v2 + a1*c1_v1), FmVector3(a1*b1_v2 + a1*b1_v1, c1*c1_v2 + a1*a1_v2 + b1*b1_v0, b1*c1_v2 + b1*c1_v1), FmVector3(a1*c1_v2 + a1*c1_v1, b1*c1_v2 + b1*c1_v1, b1*b1_v2 + a1*a1_v2 + c1*c1_v0));
        stiffnessMat->Ke_diag2 = volume * FmMatrix3(FmVector3(c2*c2_v2 + b2*b2_v2 + a2*a2_v0, a2*b2_v2 + a2*b2_v1, a2*c2_v2 + a2*c2_v1), FmVector3(a2*b2_v2 + a2*b2_v1, c2*c2_v2 + a2*a2_v2 + b2*b2_v0, b2*c2_v2 + b2*c2_v1), FmVector3(a2*c2_v2 + a2*c2_v1, b2*c2_v2 + b2*c2_v1, b2*b2_v2 + a2*a2_v2 + c2*c2_v0));
        stiffnessMat->Ke_diag3 = volume * FmMatrix3(FmVector3(c3*c3_v2 + b3*b3_v2 + a3*a3_v0, a3*b3_v2 + a3*b3_v1, a3*c3_v2 + a3*c3_v1), FmVector3(a3*b3_v2 + a3*b3_v1, c3*c3_v2 + a3*a3_v2 + b3*b3_v0, b3*c3_v2 + b3*c3_v1), FmVector3(a3*c3_v2 + a3*c3_v1, b3*c3_v2 + b3*c3_v1, b3*b3_v2 + a3*a3_v2 + c3*c3_v0));
        stiffnessMat->Ke_lower0 = volume * FmMatrix3(FmVector3(c0*c1_v2 + b0*b1_v2 + a0*a1_v0, a1*b0_v2 + a0*b1_v1, a1*c0_v2 + a0*c1_v1), FmVector3(a0*b1_v2 + a1*b0_v1, c0*c1_v2 + a0*a1_v2 + b0*b1_v0, b1*c0_v2 + b0*c1_v1), FmVector3(a0*c1_v2 + a1*c0_v1, b0*c1_v2 + b1*c0_v1, b0*b1_v2 + a0*a1_v2 + c0*c1_v0));
        stiffnessMat->Ke_lower1 = volume * FmMatrix3(FmVector3(c0*c2_v2 + b0*b2_v2 + a0*a2_v0, a2*b0_v2 + a0*b2_v1, a2*c0_v2 + a0*c2_v1), FmVector3(a0*b2_v2 + a2*b0_v1, c0*c2_v2 + a0*a2_v2 + b0*b2_v0, b2*c0_v2 + b0*c2_v1), FmVector3(a0*c2_v2 + a2*c0_v1, b0*c2_v2 + b2*c0_v1, b0*b2_v2 + a0*a2_v2 + c0*c2_v0));
        stiffnessMat->Ke_lower2 = volume * FmMatrix3(FmVector3(c0*c3_v2 + b0*b3_v2 + a0*a3_v0, a3*b0_v2 + a0*b3_v1, a3*c0_v2 + a0*c3_v1), FmVector3(a0*b3_v2 + a3*b0_v1, c0*c3_v2 + a0*a3_v2 + b0*b3_v0, b3*c0_v2 + b0*c3_v1), FmVector3(a0*c3_v2 + a3*c0_v1, b0*c3_v2 + b3*c0_v1, b0*b3_v2 + a0*a3_v2 + c0*c3_v0));
        stiffnessMat->Ke_lower3 = volume * FmMatrix3(FmVector3(c1*c2_v2 + b1*b2_v2 + a1*a2_v0, a2*b1_v2 + a1*b2_v1, a2*c1_v2 + a1*c2_v1), FmVector3(a1*b2_v2 + a2*b1_v1, c1*c2_v2 + a1*a2_v2 + b1*b2_v0, b2*c1_v2 + b1*c2_v1), FmVector3(a1*c2_v2 + a2*c1_v1, b1*c2_v2 + b2*c1_v1, b1*b2_v2 + a1*a2_v2 + c1*c2_v0));
        stiffnessMat->Ke_lower4 = volume * FmMatrix3(FmVector3(c1*c3_v2 + b1*b3_v2 + a1*a3_v0, a3*b1_v2 + a1*b3_v1, a3*c1_v2 + a1*c3_v1), FmVector3(a1*b3_v2 + a3*b1_v1, c1*c3_v2 + a1*a3_v2 + b1*b3_v0, b3*c1_v2 + b1*c3_v1), FmVector3(a1*c3_v2 + a3*c1_v1, b1*c3_v2 + b3*c1_v1, b1*b3_v2 + a1*a3_v2 + c1*c3_v0));
        stiffnessMat->Ke_lower5 = volume * FmMatrix3(FmVector3(c2*c3_v2 + b2*b3_v2 + a2*a3_v0, a3*b2_v2 + a2*b3_v1, a3*c2_v2 + a2*c3_v1), FmVector3(a2*b3_v2 + a3*b2_v1, c2*c3_v2 + a2*a3_v2 + b2*b3_v0, b3*c2_v2 + b2*c3_v1), FmVector3(a2*c3_v2 + a3*c2_v1, b2*c3_v2 + b3*c2_v1, b2*b3_v2 + a2*a3_v2 + c2*c3_v0));
    }

    void FmComputeRotatedStiffnessMatrix(
        FmTetRotatedStiffnessMatrix* rotatedStiffnessMat,
        const FmTetStiffnessMatrix& stiffnessMat,
        const FmMatrix3& tetRestShapeMatrix,
        const FmMatrix3& tetRotation)
    {
        FmMatrix3 ReKe[4][4];

        FmMatrix3 tetRotationInv = transpose(tetRotation);

        ReKe[0][0] = tetRotation * stiffnessMat.Ke_diag0;
        ReKe[0][1] = tetRotation * transpose(stiffnessMat.Ke_lower0);
        ReKe[0][2] = tetRotation * transpose(stiffnessMat.Ke_lower1);
        ReKe[0][3] = tetRotation * transpose(stiffnessMat.Ke_lower2);
        ReKe[1][0] = tetRotation * stiffnessMat.Ke_lower0;
        ReKe[1][1] = tetRotation * stiffnessMat.Ke_diag1;
        ReKe[1][2] = tetRotation * transpose(stiffnessMat.Ke_lower3);
        ReKe[1][3] = tetRotation * transpose(stiffnessMat.Ke_lower4);
        ReKe[2][0] = tetRotation * stiffnessMat.Ke_lower1;
        ReKe[2][1] = tetRotation * stiffnessMat.Ke_lower3;
        ReKe[2][2] = tetRotation * stiffnessMat.Ke_diag2;
        ReKe[2][3] = tetRotation * transpose(stiffnessMat.Ke_lower5);
        ReKe[3][0] = tetRotation * stiffnessMat.Ke_lower2;
        ReKe[3][1] = tetRotation * stiffnessMat.Ke_lower4;
        ReKe[3][2] = tetRotation * stiffnessMat.Ke_lower5;
        ReKe[3][3] = tetRotation * stiffnessMat.Ke_diag3;

        // vec0 is 0
        FmVector3 vec1 = -tetRestShapeMatrix.col0;
        FmVector3 vec2 = -tetRestShapeMatrix.col1;
        FmVector3 vec3 = -tetRestShapeMatrix.col2;

        rotatedStiffnessMat->f0eprime0 = /* ReKe[0][0] * vec0 + */ ReKe[0][1] * vec1 + ReKe[0][2] * vec2 + ReKe[0][3] * vec3;
        rotatedStiffnessMat->f0eprime1 = /* ReKe[1][0] * vec0 + */ ReKe[1][1] * vec1 + ReKe[1][2] * vec2 + ReKe[1][3] * vec3;
        rotatedStiffnessMat->f0eprime2 = /* ReKe[2][0] * vec0 + */ ReKe[2][1] * vec1 + ReKe[2][2] * vec2 + ReKe[2][3] * vec3;
        rotatedStiffnessMat->f0eprime3 = /* ReKe[3][0] * vec0 + */ ReKe[3][1] * vec1 + ReKe[3][2] * vec2 + ReKe[3][3] * vec3;

        rotatedStiffnessMat->Keprime_diag0 = ReKe[0][0] * tetRotationInv;
        rotatedStiffnessMat->Keprime_lower0 = ReKe[1][0] * tetRotationInv;
        rotatedStiffnessMat->Keprime_lower1 = ReKe[2][0] * tetRotationInv;
        rotatedStiffnessMat->Keprime_lower2 = ReKe[3][0] * tetRotationInv;
        rotatedStiffnessMat->Keprime_diag1 = ReKe[1][1] * tetRotationInv;
        rotatedStiffnessMat->Keprime_lower3 = ReKe[2][1] * tetRotationInv;
        rotatedStiffnessMat->Keprime_lower4 = ReKe[3][1] * tetRotationInv;

        rotatedStiffnessMat->Keprime_diag2 = ReKe[2][2] * tetRotationInv;
        rotatedStiffnessMat->Keprime_lower5 = ReKe[3][2] * tetRotationInv;

        rotatedStiffnessMat->Keprime_diag3 = ReKe[3][3] * tetRotationInv;
    }

    static FM_FORCE_INLINE float FmFrobeniusNorm(const FmMatrix3& mat)
    {
        float m00 = mat.getElem(0, 0);
        float m01 = mat.getElem(1, 0);
        float m02 = mat.getElem(2, 0);
        float m10 = mat.getElem(0, 1);
        float m11 = mat.getElem(1, 1);
        float m12 = mat.getElem(2, 1);
        float m20 = mat.getElem(0, 2);
        float m21 = mat.getElem(1, 2);
        float m22 = mat.getElem(2, 2);

        return sqrtf(
            m00*m00 +
            m01*m01 +
            m02*m02 +
            m10*m10 +
            m11*m11 +
            m12*m12 +
            m20*m20 +
            m21*m21 +
            m22*m22);
    }

    // Reference: Higham, "Computing the Polar Decomposition -- with Applications"
    FmMatrix3 FmComputeTetRotation(const FmMatrix3& deformationGradient)
    {
        FmMatrix3 U = deformationGradient;

        const int numIterations = 15;
        for (int i = 0; i < numIterations; i++)
        {
            float scale = sqrtf(FmFrobeniusNorm(inverse(U)) / FmFrobeniusNorm(U));
            U = (scale * U + (1.0f / scale) * inverse(transpose(U))) * 0.5f;
        }

        return U;
    }

    FmMatrix3 FmComputeTetRotationSvd(const FmMatrix3& deformationGradient)
    {
        FmMatrix3 U, V;
        FmVector3 sigma;
        FmSvd3x3(&U, &sigma, &V, deformationGradient);

        return U * transpose(V);
    }
}
