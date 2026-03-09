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

#pragma once

#include "FEMFXCommonInternal.h"
#include "FEMFXVectorMath.h"
#include "FEMFXSvd3x3.h"

namespace AMD
{
    // Parameters that describe tetrahedron shape and mapping from position to barycentric values.
    // Strain matrix is easily computed from these parameters.
    struct FmTetShapeParams
    {
        FmMatrix3 DmInv;      // Inverse of material/rest shape matrix = [p1 - p0, p2 - p0, p3 - p0]
        FmVector4 baryCol3;   // Barycentric basis matrix col 3; full matrix can be made from this and DmInv
        float det = 0.0f;     // Determinant of rest shape matrix; volume = det/6
        bool clamped = false; // Set if SVD sigma values were clamped (high aspect-ratio tet)

        inline float GetA0() const { return -sum(DmInv.col0); }
        inline float GetA1() const { return DmInv.col0.x; }
        inline float GetA2() const { return DmInv.col0.y; }
        inline float GetA3() const { return DmInv.col0.z; }

        inline float GetB0() const { return -sum(DmInv.col1); }
        inline float GetB1() const { return DmInv.col1.x; }
        inline float GetB2() const { return DmInv.col1.y; }
        inline float GetB3() const { return DmInv.col1.z; }

        inline float GetC0() const { return -sum(DmInv.col2); }
        inline float GetC1() const { return DmInv.col2.x; }
        inline float GetC2() const { return DmInv.col2.y; }
        inline float GetC3() const { return DmInv.col2.z; }

        inline float GetArea0() const { return 0.5f*det*sqrtf(GetA0()*GetA0() + GetB0()*GetB0() + GetC0()*GetC0()); }
        inline float GetArea1() const { return 0.5f*det*sqrtf(GetA1()*GetA1() + GetB1()*GetB1() + GetC1()*GetC1()); }
        inline float GetArea2() const { return 0.5f*det*sqrtf(GetA2()*GetA2() + GetB2()*GetB2() + GetC2()*GetC2()); }
        inline float GetArea3() const { return 0.5f*det*sqrtf(GetA3()*GetA3() + GetB3()*GetB3() + GetC3()*GetC3()); }
        inline FmVector3 GetNormal0() const { return -normalize(FmVector3(GetA0(), GetB0(), GetC0())); }
        inline FmVector3 GetNormal1() const { return -normalize(FmVector3(GetA1(), GetB1(), GetC1())); }
        inline FmVector3 GetNormal2() const { return -normalize(FmVector3(GetA2(), GetB2(), GetC2())); }
        inline FmVector3 GetNormal3() const { return -normalize(FmVector3(GetA3(), GetB3(), GetC3())); }
        inline float GetVolume() const { return det * (1.0f / 6.0f); }

        inline FmMatrix4 ComputeBaryMatrix() const
        {
            FmVector4 col0(GetA0(), GetA1(), GetA2(), GetA3());
            FmVector4 col1(GetB0(), GetB1(), GetB2(), GetB3());
            FmVector4 col2(GetC0(), GetC1(), GetC2(), GetC3());
            return FmMatrix4(col0, col1, col2, baryCol3);
        }

        inline FmVector4 ComputeBarycentricsOfRestPosition(const FmVector3& restPosition) const
        {
            return ComputeBaryMatrix() * FmVector4(restPosition, 1.0f);
        }
    };

    // Strain matrix, relating position offsets of tet vertices to strain
    struct FmTetStrainMatrix
    {
        FmMatrix3 Be00;   // 6 x 12 strain matrix
        FmMatrix3 Be01;
        FmMatrix3 Be02;
        FmMatrix3 Be03;
        FmMatrix3 Be10;
        FmMatrix3 Be11;
        FmMatrix3 Be12;
        FmMatrix3 Be13;
    };

    // Stress matrix, relating position offsets of tet vertices to stress
    struct FmTetStressMatrix
    {
        FmMatrix3 EBe00;  // E . Be which is 6 x 12 stress matrix
        FmMatrix3 EBe01;
        FmMatrix3 EBe02;
        FmMatrix3 EBe03;
        FmMatrix3 EBe10;
        FmMatrix3 EBe11;
        FmMatrix3 EBe12;
        FmMatrix3 EBe13;
    };

    // Stiffness matrix, relating position offsets of tet vertices to internal force
    struct FmTetStiffnessMatrix
    {
        FmMatrix3 Ke_diag0;   // Symmetric 12 x 12 stiffness matrix, submatrices on diagonal
        FmMatrix3 Ke_diag1;   
        FmMatrix3 Ke_diag2;   
        FmMatrix3 Ke_diag3;   
        FmMatrix3 Ke_lower0;  // Symmetric 12 x 12 stiffness matrix, submatrices below diagonal
        FmMatrix3 Ke_lower1;
        FmMatrix3 Ke_lower2;
        FmMatrix3 Ke_lower3;
        FmMatrix3 Ke_lower4;
        FmMatrix3 Ke_lower5;
    };

    // Rotated stiffness matrix.
    // Matrix combines steps of un-rotating deformed element positions, applying stiffness, and rotating forces back.
    // Reference: Muller and Gross, "Interactive Virtual Materials"
    struct FmTetRotatedStiffnessMatrix
    {
        FmMatrix3 Keprime_diag0;    // Submatrices on diagonal
        FmMatrix3 Keprime_diag1;
        FmMatrix3 Keprime_diag2;
        FmMatrix3 Keprime_diag3;
        FmMatrix3 Keprime_lower0;   // Submatrices below diagonal
        FmMatrix3 Keprime_lower1;
        FmMatrix3 Keprime_lower2;
        FmMatrix3 Keprime_lower3;
        FmMatrix3 Keprime_lower4;
        FmMatrix3 Keprime_lower5;
        FmVector3 f0eprime0;        // Term of the internal force expression containing the rest positions
        FmVector3 f0eprime1; 
        FmVector3 f0eprime2; 
        FmVector3 f0eprime3; 
    };

    // Compute quantities associated with shape of tet in rest position.
    // If minSingularValueRatio or minSingularValue > 0, computes SVD of shape matrix and clamps singular values to improve condition.
    // Similar to method in: Iben and O'Brien "Generating surface crack patterns".
    void FmComputeShapeParams(FmTetShapeParams* shapeParams, const FmVector3& p0, const FmVector3& p1, const FmVector3& p2, const FmVector3& p3, float minSingularValueRatio = 0.0f, float minSingularValue = 0.0f);

    // Compute strain matrix from shape parameters
    void FmComputeTetStrainMatrix(FmTetStrainMatrix* strainMat, const FmTetShapeParams& shapeParams);

    // Compute strain from offsets between unrotated positions and rest positions.
    // Assuming unrotated and rest tet are translated to align position 0, so offset 0 is zero.
    void FmComputeTetStrain(
        FmVector3 strain[2],
        const FmTetShapeParams& shapeParams,
        const FmVector3& unrotatedOffset1,
        const FmVector3& unrotatedOffset2,
        const FmVector3& unrotatedOffset3);

    // Compute stress matrix from shape and material parameters
    void FmComputeTetStressMatrix(
        FmTetStressMatrix* stressMat,
        const FmMatrix3& DmInv,
        float youngsModulus,
        float poissonsRatio);

    // Compute stress and stiffness matrix from shape and material parameters
    void FmComputeTetStressAndStiffnessMatrix(
        FmTetStressMatrix* stressMat,
        FmTetStiffnessMatrix* stiffnessMat,
        const FmMatrix3& DmInv,
        float volume,
        float youngsModulus,
        float poissonsRatio);

    // Extract rotation from the transform of rest positions to deformed positions, using polar decomposition
    FmMatrix3 FmComputeTetRotation(const FmMatrix3& deformationGradient);

    // Extract rotation from the transform of rest positions to deformed positions, using polar decomposition computed by SVD.
    // Robust in cases of near singularity or inversion.
    FmMatrix3 FmComputeTetRotationSvd(const FmMatrix3& deformationGradient);

    // Compute rotated stiffness used to compute forces at each timestep
    void FmComputeRotatedStiffnessMatrix(
        FmTetRotatedStiffnessMatrix* rotatedStiffnessMat,
        const FmTetStiffnessMatrix& stiffnessMat,
        const FmMatrix3& tetRestShapeMatrix,
        const FmMatrix3& tetRotation);

    // Compute shape matrix for a tetrahedron, needed for deformation gradient
    static FM_FORCE_INLINE FmMatrix3 FmComputeTetShapeMatrix(const FmVector3& position0, const FmVector3& position1, const FmVector3& position2, const FmVector3& position3)
    {
        FmMatrix3 shapeMatrix;
        shapeMatrix.col0 = position1 - position0;
        shapeMatrix.col1 = position2 - position0;
        shapeMatrix.col2 = position3 - position0;
        return shapeMatrix;
    }

    // Compute elastic deformation as the total deformation gradient multiplied by inverse of plastic deformation matrix.
    // Returns the SVD of this elastic deformation = U * Fhat * V^T, where each component of Fhat gives a stretch or compression factor.
    // The plastic deformation matrix can be updated using this decomposition.
    static FM_FORCE_INLINE void FmComputeTetElasticDeformationGradient(
        FmMatrix3* U,
        FmVector3* Fhat,
        FmMatrix3* V,
        const FmMatrix3& deformedShapeMatrix,
        const FmMatrix3& restShapeMatrixInv,
        const FmMatrix3& plasticDeformationMatrixInv)
    {
        FmMatrix3 F = deformedShapeMatrix * restShapeMatrixInv * plasticDeformationMatrixInv;

        FmSvd3x3(U, Fhat, V, F);
    }

    static FM_FORCE_INLINE bool FmIsTetInverted(const FmVector3 positions[4])
    {
        return dot(positions[3] - positions[0], cross(positions[1] - positions[0], positions[2] - positions[0])) < 0.0f;
    }

    // Ratio of maximum edge length to minimum height of vertex from plane containing other vertices.
    // Measure of tetrahedron quality.
    float FmComputeTetAspectRatio(const FmVector3 positions[4]);
    float FmComputeTetAspectRatio(const FmVector3& position0, const FmVector3& position1, const FmVector3& position2, const FmVector3& position3);
}
