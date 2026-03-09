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

#include "AMD_FEMFX.h"
#include "FEMFXThreading.h"
#if FM_SOA_TET_MATH
#include "FEMFXSoaTetMath.h"
#include "FEMFXSoaSvd3x3.h"
#endif
#include "FEMFXScene.h"
#include "FEMFXUpdateTetState.h"

#define FM_UPDATE_TET_BATCH_SIZE 64

namespace AMD
{
    // Input data required for updating tet state: tet rotation, stiffness, plasticity, and testing fracture
    struct FmUpdateTetStateInput
    {
        // Deformed shape matrix
        FmMatrix3 deformedShapeMatrix;

        // Rest shape matrix
        FmMatrix3 restShapeMatrix;

        // Rest shape matrix used for stress calculation; from original tet mesh or virtual rest positions after plastic deformation
        FmMatrix3 stressRestShapeMatrix;

        FmTetShapeParams shapeParams;         // Shape params based on original rest positions

        // Matrices from plasticity state
        FmMatrix3 plasticDeformationMatrix;

        // Copied material parameters
        float youngsModulus = 0.0f;
        float poissonsRatio = 0.0f;
        float fractureStressThreshold = 0.0f;
        float plasticYieldThreshold = 0.0f;
        float plasticCreep = 0.0f;
        float plasticMin = 0.0f;
        float plasticMax = 0.0f;
        bool  preserveVolume = false;

        bool  testFracture = false;       // Based on tet properties, set if need to test fracture
        bool  updatePlasticity = false;   // Based on tet properties, set if need to update plasticity
    };

    // Output tet state which must be stored into mesh
    struct FmUpdateTetStateOutput
    {
        FmMatrix3 tetRotation; // Tet rotation computed from original rest positions
        FmQuat    tetQuat;     // Rotation converted to quaternion for average at vertices

#if !FM_MATRIX_ASSEMBLY_BY_TETS
        FmTetRotatedStiffnessMatrix rotatedStiffnessMat; // Tet stiffness matrix
#endif

        // Updated plasticity state
        FmMatrix3 plasticDeformationMatrix;

        // Stress measure for fracture
        FmVector3 maxStressDirection;
        float maxStressEigenvalue = 0.0f;

        // Strain magnitude
        float strainMag = 0.0f;

        // Copied from input
        float fractureStressThreshold = 0.0f;
        bool testFracture = false;
        bool updatePlasticity = false;
    };

    static FM_FORCE_INLINE void FmAddToQuat(FmQuat* dst, const FmQuat& src)
    {
        if (dot(*dst, src) > 0.0f)
        {
            *dst += src;
        }
        else
        {
            *dst += -src;
        }
    }

#if FM_SOA_TET_MATH
    // Input data required for updating tet state: tet rotation, stiffness, plasticity, and testing fracture
    template<class T>
    struct FmSoaUpdateTetStateInput
    {
        // Deformed shape matrix
        typename T::SoaMatrix3 deformedShapeMatrix;

        // Rest shape matrix
        typename T::SoaMatrix3 restShapeMatrix;

        // Rest shape matrix used for stress calculation; from original tet mesh or virtual rest positions after plastic deformation
        typename T::SoaMatrix3 stressRestShapeMatrix;

        FmSoaTetShapeParams<T> shapeParams;         // Shape params based on original rest positions

        // Matrices from plasticity state
        typename T::SoaMatrix3 plasticDeformationMatrix;

        // Copied material parameters
        typename T::SoaFloat youngsModulus = 0.0f;
        typename T::SoaFloat poissonsRatio = 0.0f;
        typename T::SoaFloat fractureStressThreshold = 0.0f;
        typename T::SoaFloat plasticYieldThreshold = 0.0f;
        typename T::SoaFloat plasticCreep = 0.0f;
        typename T::SoaFloat plasticMin = 0.0f;
        typename T::SoaFloat plasticMax = 0.0f;
        typename T::SoaBool  preserveVolume = false;

        typename T::SoaBool  testFracture = false;       // Based on tet properties, set if need to test fracture
        typename T::SoaBool  updatePlasticity = false;   // Based on tet properties, set if need to test/update plasticity
    };

    // Output tet state which must be stored into mesh
    template<class T>
    struct FmSoaUpdateTetStateOutput
    {
        typename T::SoaMatrix3 tetRotation; // Tet rotation computed from original rest positions

#if !FM_MATRIX_ASSEMBLY_BY_TETS
        typename FmSoaTetRotatedStiffnessMatrix<T> rotatedStiffnessMat; // Tet stiffness matrix
#endif

        // Updated plasticity state
        typename T::SoaMatrix3 plasticDeformationMatrix;

        // Stress measure for fracture
        typename T::SoaVector3 maxStressDirection;
        typename T::SoaFloat maxStressEigenvalue = 0.0f;

        // Deformation value
        typename T::SoaFloat strainMag = 0.0f;

        // Copied from input
        typename T::SoaFloat fractureStressThreshold = 0.0f;
        typename T::SoaBool testFracture = false;
        typename T::SoaBool updatePlasticity = false;
    };

    template<class SoaTypes>
    struct FmUpdateTetStateBatch
    {
        uint baseTetIdx;
        uint numTets; // number of tets batched

        FmSoaUpdateTetStateInput<SoaTypes> input;
        FmSoaUpdateTetStateOutput<SoaTypes> output;

        FM_ALIGN(16) FmVector3 aosDeformedShapeMatCol0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosDeformedShapeMatCol1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosDeformedShapeMatCol2[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosRestShapeMatCol0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosRestShapeMatCol1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosRestShapeMatCol2[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosStressRestShapeMatCol0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosStressRestShapeMatCol1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosStressRestShapeMatCol2[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosShapeParamsDmInvCol0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosShapeParamsDmInvCol1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosShapeParamsDmInvCol2[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosPlasticDeformationMatrixCol0[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosPlasticDeformationMatrixCol1[SoaTypes::width] FM_ALIGN_END(16);
        FM_ALIGN(16) FmVector3 aosPlasticDeformationMatrixCol2[SoaTypes::width] FM_ALIGN_END(16);

        FmUpdateTetStateBatch()
        {
            baseTetIdx = 0;
            numTets = 0;

            for (uint sliceIdx = 0; sliceIdx < SoaTypes::width; sliceIdx++)
            {
                aosDeformedShapeMatCol0[sliceIdx] = FmVector3(0.0f);
                aosDeformedShapeMatCol1[sliceIdx] = FmVector3(0.0f);
                aosDeformedShapeMatCol2[sliceIdx] = FmVector3(0.0f);
                aosRestShapeMatCol0[sliceIdx] = FmVector3(0.0f);
                aosRestShapeMatCol1[sliceIdx] = FmVector3(0.0f);
                aosRestShapeMatCol2[sliceIdx] = FmVector3(0.0f);
                aosStressRestShapeMatCol0[sliceIdx] = FmVector3(0.0f);
                aosStressRestShapeMatCol1[sliceIdx] = FmVector3(0.0f);
                aosStressRestShapeMatCol2[sliceIdx] = FmVector3(0.0f);
                aosShapeParamsDmInvCol0[sliceIdx] = FmVector3(0.0f);
                aosShapeParamsDmInvCol1[sliceIdx] = FmVector3(0.0f);
                aosShapeParamsDmInvCol2[sliceIdx] = FmVector3(0.0f);
                aosPlasticDeformationMatrixCol0[sliceIdx] = FmVector3(0.0f);
                aosPlasticDeformationMatrixCol1[sliceIdx] = FmVector3(0.0f);
                aosPlasticDeformationMatrixCol2[sliceIdx] = FmVector3(0.0f);
            }

            input.shapeParams.det = SoaTypes::SoaFloat(0.0f);
            input.youngsModulus = SoaTypes::SoaFloat(0.0f);
            input.poissonsRatio = SoaTypes::SoaFloat(0.0f);
            input.fractureStressThreshold = SoaTypes::SoaFloat(0.0f);
            input.plasticYieldThreshold = SoaTypes::SoaFloat(0.0f);
            input.plasticCreep = SoaTypes::SoaFloat(0.0f);
            input.plasticMin = SoaTypes::SoaFloat(0.0f);
            input.plasticMax = SoaTypes::SoaFloat(0.0f);
            input.preserveVolume = SoaTypes::SoaFloat(0.0f);
            input.testFracture = SoaTypes::SoaFloat(0.0f);
            input.updatePlasticity = SoaTypes::SoaFloat(0.0f);
        }

        void SetBatchSlice(
            uint idx,
            const FmUpdateTetStateInput& inputSlice,
            bool meshSupportsPlasticity)
        {
            aosDeformedShapeMatCol0[idx] = inputSlice.deformedShapeMatrix.col0;
            aosDeformedShapeMatCol1[idx] = inputSlice.deformedShapeMatrix.col1;
            aosDeformedShapeMatCol2[idx] = inputSlice.deformedShapeMatrix.col2;
            aosRestShapeMatCol0[idx] = inputSlice.restShapeMatrix.col0;
            aosRestShapeMatCol1[idx] = inputSlice.restShapeMatrix.col1;
            aosRestShapeMatCol2[idx] = inputSlice.restShapeMatrix.col2;
            aosStressRestShapeMatCol0[idx] = inputSlice.stressRestShapeMatrix.col0;
            aosStressRestShapeMatCol1[idx] = inputSlice.stressRestShapeMatrix.col1;
            aosStressRestShapeMatCol2[idx] = inputSlice.stressRestShapeMatrix.col2;
            aosShapeParamsDmInvCol0[idx] = inputSlice.shapeParams.DmInv.col0;
            aosShapeParamsDmInvCol1[idx] = inputSlice.shapeParams.DmInv.col1;
            aosShapeParamsDmInvCol2[idx] = inputSlice.shapeParams.DmInv.col2;

            input.shapeParams.det.setSlice(idx, inputSlice.shapeParams.det);

            if (meshSupportsPlasticity)
            {
                aosPlasticDeformationMatrixCol0[idx] = inputSlice.plasticDeformationMatrix.col0;
                aosPlasticDeformationMatrixCol1[idx] = inputSlice.plasticDeformationMatrix.col1;
                aosPlasticDeformationMatrixCol2[idx] = inputSlice.plasticDeformationMatrix.col2;
            }

            input.youngsModulus.setSlice(idx, inputSlice.youngsModulus);
            input.poissonsRatio.setSlice(idx, inputSlice.poissonsRatio);

            if (inputSlice.testFracture)
            {
                input.fractureStressThreshold.setSlice(idx, inputSlice.fractureStressThreshold);
            }

            if (inputSlice.updatePlasticity)
            {
                input.plasticYieldThreshold.setSlice(idx, inputSlice.plasticYieldThreshold);
                input.plasticCreep.setSlice(idx, inputSlice.plasticCreep);
                input.plasticMin.setSlice(idx, inputSlice.plasticMin);
                input.plasticMax.setSlice(idx, inputSlice.plasticMax);
                input.preserveVolume.setSlice(idx, inputSlice.preserveVolume);
            }

            input.testFracture.setSlice(idx, inputSlice.testFracture);
            input.updatePlasticity.setSlice(idx, inputSlice.updatePlasticity);
        }

        void GetBatchSlice(
            FmUpdateTetStateOutput* outputSlice,
            uint idx,
            bool meshSupportsPlasticity,
            bool computingStrainMag)
        {
            outputSlice->tetRotation = FmGetSlice<SoaTypes>(output.tetRotation, idx);

#if !FM_MATRIX_ASSEMBLY_BY_TETS
            FmGetSlice<SoaTypes>(&outputSlice->rotatedStiffnessMat, output.rotatedStiffnessMat, idx);
#endif

            if (meshSupportsPlasticity)
            {
                outputSlice->plasticDeformationMatrix = FmGetSlice<SoaTypes>(output.plasticDeformationMatrix, idx);
            }

            // Stress measure for fracture
            outputSlice->maxStressDirection = FmGetSlice<SoaTypes>(output.maxStressDirection, idx);
            outputSlice->maxStressEigenvalue = output.maxStressEigenvalue.getSlice(idx);

            if (computingStrainMag)
            {
                outputSlice->strainMag = output.strainMag.getSlice(idx);
            }

            outputSlice->fractureStressThreshold = output.fractureStressThreshold.getSlice(idx);
            outputSlice->testFracture = output.testFracture.getSlice(idx);
            outputSlice->updatePlasticity = output.updatePlasticity.getSlice(idx);
        }

        void ConvertAosToSoa(bool meshSupportsPlasticity)
        {
            FmSetSoaFromAlignedAos(&input.deformedShapeMatrix.col0, aosDeformedShapeMatCol0);
            FmSetSoaFromAlignedAos(&input.deformedShapeMatrix.col1, aosDeformedShapeMatCol1);
            FmSetSoaFromAlignedAos(&input.deformedShapeMatrix.col2, aosDeformedShapeMatCol2);

            FmSetSoaFromAlignedAos(&input.restShapeMatrix.col0, aosRestShapeMatCol0);
            FmSetSoaFromAlignedAos(&input.restShapeMatrix.col1, aosRestShapeMatCol1);
            FmSetSoaFromAlignedAos(&input.restShapeMatrix.col2, aosRestShapeMatCol2);

            FmSetSoaFromAlignedAos(&input.stressRestShapeMatrix.col0, aosStressRestShapeMatCol0);
            FmSetSoaFromAlignedAos(&input.stressRestShapeMatrix.col1, aosStressRestShapeMatCol1);
            FmSetSoaFromAlignedAos(&input.stressRestShapeMatrix.col2, aosStressRestShapeMatCol2);

            FmSetSoaFromAlignedAos(&input.shapeParams.DmInv.col0, aosShapeParamsDmInvCol0);
            FmSetSoaFromAlignedAos(&input.shapeParams.DmInv.col1, aosShapeParamsDmInvCol1);
            FmSetSoaFromAlignedAos(&input.shapeParams.DmInv.col2, aosShapeParamsDmInvCol2);

            if (meshSupportsPlasticity)
            {
                FmSetSoaFromAlignedAos(&input.plasticDeformationMatrix.col0, aosPlasticDeformationMatrixCol0);
                FmSetSoaFromAlignedAos(&input.plasticDeformationMatrix.col1, aosPlasticDeformationMatrixCol1);
                FmSetSoaFromAlignedAos(&input.plasticDeformationMatrix.col2, aosPlasticDeformationMatrixCol2);
            }
        }
    };
#endif

    // Initialize FmUpdateTetStateInput for tet
    static FM_FORCE_INLINE void FmGatherUpdateTetStateInput(
        FmUpdateTetStateInput* tetUpdateInput,
        const FmTetMesh& tetMesh, uint tetId,
        bool testingFracture,
        bool updatingPlasticity)
    {
        const FmTetShapeParams& tetShapeParams = tetMesh.tetsShapeParams[tetId];
        FmTetStressMaterialParams tetStressMaterialParams = tetMesh.tetsStressMaterialParams[tetId];
        FmTetPlasticityMaterialParams tetPlasticityMaterialParams = tetMesh.tetsPlasticityMaterialParams ? tetMesh.tetsPlasticityMaterialParams[tetId] : FmTetPlasticityMaterialParams();
        FmTetFractureMaterialParams tetFractureMaterialParams = tetMesh.tetsFractureMaterialParams ? tetMesh.tetsFractureMaterialParams[tetId] : FmTetFractureMaterialParams();
        uint16_t tetFlags = tetMesh.tetsFlags[tetId];

        FmTetVertIds tetVertIds = tetMesh.tetsVertIds[tetId];
        uint vId0 = tetVertIds.ids[0];
        uint vId1 = tetVertIds.ids[1];
        uint vId2 = tetVertIds.ids[2];
        uint vId3 = tetVertIds.ids[3];

        FM_ASSERT(vId0 < tetMesh.numVerts && vId1 < tetMesh.numVerts && vId2 < tetMesh.numVerts && vId3 < tetMesh.numVerts);

        // Compute tet rotation based on deformed positions
        FmVector3 deformedPosition0 = tetMesh.vertsPos[vId0];
        FmVector3 deformedPosition1 = tetMesh.vertsPos[vId1];
        FmVector3 deformedPosition2 = tetMesh.vertsPos[vId2];
        FmVector3 deformedPosition3 = tetMesh.vertsPos[vId3];
        deformedPosition1 -= deformedPosition0;
        deformedPosition2 -= deformedPosition0;
        deformedPosition3 -= deformedPosition0;
        deformedPosition0 = FmVector3(0.0f);

        FmVector3 restPosition0 = tetMesh.vertsRestPos[vId0];
        FmVector3 restPosition1 = tetMesh.vertsRestPos[vId1];
        FmVector3 restPosition2 = tetMesh.vertsRestPos[vId2];
        FmVector3 restPosition3 = tetMesh.vertsRestPos[vId3];
        restPosition1 -= restPosition0;
        restPosition2 -= restPosition0;
        restPosition3 -= restPosition0;
        restPosition0 = FmVector3(0.0f);

        // Get tet rest positions.
        // For plastic deformation, modify rest positions with current plastic deformation
        FmVector3 stressRestPosition1, stressRestPosition2, stressRestPosition3;

        bool meshSupportsPlasticity = (tetMesh.tetsPlasticity != nullptr);

        if (meshSupportsPlasticity)
        {
            const FmTetPlasticityState& plasticityState = tetMesh.tetsPlasticity[tetId];

            stressRestPosition1 = plasticityState.plasticDeformationMatrix * restPosition1;
            stressRestPosition2 = plasticityState.plasticDeformationMatrix * restPosition2;
            stressRestPosition3 = plasticityState.plasticDeformationMatrix * restPosition3;

            tetUpdateInput->plasticDeformationMatrix = plasticityState.plasticDeformationMatrix;
        }
        else
        {
            stressRestPosition1 = restPosition1;
            stressRestPosition2 = restPosition2;
            stressRestPosition3 = restPosition3;
        }

        bool meshSupportsFracture = (tetMesh.tetsToFracture != nullptr);

        bool tetTestFracture = testingFracture
            && meshSupportsFracture
            && !FM_IS_SET(tetFlags, FM_TET_FLAG_KINEMATIC)
            && !FM_ALL_SET(tetFlags,
                FM_TET_FLAG_FACE0_FRACTURE_DISABLED |
                FM_TET_FLAG_FACE1_FRACTURE_DISABLED |
                FM_TET_FLAG_FACE2_FRACTURE_DISABLED |
                FM_TET_FLAG_FACE3_FRACTURE_DISABLED);

        bool tetUpdatePlasticity = updatingPlasticity
            && meshSupportsPlasticity
            && tetPlasticityMaterialParams.plasticCreep > 0.0f
            && FM_NOT_SET(tetFlags, FM_TET_FLAG_PLASTICITY_DISABLED);

        tetUpdateInput->deformedShapeMatrix.col0 = deformedPosition1;
        tetUpdateInput->deformedShapeMatrix.col1 = deformedPosition2;
        tetUpdateInput->deformedShapeMatrix.col2 = deformedPosition3;

        tetUpdateInput->restShapeMatrix.col0 = restPosition1;
        tetUpdateInput->restShapeMatrix.col1 = restPosition2;
        tetUpdateInput->restShapeMatrix.col2 = restPosition3;

        tetUpdateInput->stressRestShapeMatrix.col0 = stressRestPosition1;
        tetUpdateInput->stressRestShapeMatrix.col1 = stressRestPosition2;
        tetUpdateInput->stressRestShapeMatrix.col2 = stressRestPosition3;

        tetUpdateInput->shapeParams = tetShapeParams;

        tetUpdateInput->youngsModulus = tetStressMaterialParams.youngsModulus;
        tetUpdateInput->poissonsRatio = tetStressMaterialParams.poissonsRatio;

        tetUpdateInput->fractureStressThreshold = tetFractureMaterialParams.fractureStressThreshold;

        if (tetUpdatePlasticity)
        {
            tetUpdateInput->plasticYieldThreshold = tetPlasticityMaterialParams.plasticYieldThreshold;
            tetUpdateInput->plasticCreep = tetPlasticityMaterialParams.plasticCreep;
            tetUpdateInput->plasticMin = tetPlasticityMaterialParams.plasticMin;
            tetUpdateInput->plasticMax = tetPlasticityMaterialParams.plasticMax;
            tetUpdateInput->preserveVolume = ((tetFlags & FM_TET_FLAG_VOLUME_PRESERVING_PLASTICITY) != 0);
        }

        tetUpdateInput->testFracture = tetTestFracture;
        tetUpdateInput->updatePlasticity = tetUpdatePlasticity;
    }

    // Update state of the multiplicative plasticity model.
    // If return is true, outputs updated plasticDeformationMatrix and shape params.
    // References:
    // - Irving et al., "Invertible Finite Elements For Robust Simulation of Large Deformation"
    // - Bargteil et al., "A Finite Element Method for Animating Large Viscoplastic Flow"
    static FM_FORCE_INLINE bool FmUpdateTetPlasticity(
        FmMatrix3* outPlasticDeformationMatrix,
        const FmMatrix3& inPlasticDeformationMatrix,
        const FmMatrix3& tetRotationInv,
        const FmMatrix3& restShapeMatrixInv,
        const FmMatrix3& deformedShapeMatrix,
        const FmVector3& stress0,
        const FmVector3& stress1,
        float plasticYieldThreshold, float plasticCreep, float plasticMin, float plasticMax, bool preserveVolume)
    {
        (void)tetRotationInv;
        FmMatrix3 plasticDeformationMatrix = inPlasticDeformationMatrix;

        float elasticStressMag = sqrtf(lengthSqr(stress0) + lengthSqr(stress1));
        float yieldStress = plasticYieldThreshold;

        bool updateNeeded = false;

        if (elasticStressMag > yieldStress)
        {
            // Compute deformation gradient (decomposed into SVD), after applying plastic deformation offset
            FmMatrix3 U, V;
            FmVector3 Fhat;
            FmComputeTetElasticDeformationGradient(&U, &Fhat, &V,
                deformedShapeMatrix, restShapeMatrixInv, inverse(plasticDeformationMatrix));

            FmVector3 elasticDeformation = Fhat;

            const float deformationThreshold = 0.01f;
            if (elasticDeformation.x > deformationThreshold       // Do nothing for low or inverted elastic deformation
                && elasticDeformation.y > deformationThreshold
                && elasticDeformation.z > deformationThreshold)
            {
                updateNeeded = true;

                float creep = plasticCreep;
                float minDeformation = FmMaxFloat(plasticMin, deformationThreshold);
                float maxDeformation = plasticMax;

                if (preserveVolume)
                {
                    // Scale the elastic deformation such that determinant is 1.0, which represents a volume preserving deformation.
                    float elasticDet = elasticDeformation.x * elasticDeformation.y * elasticDeformation.z;
                    elasticDeformation = elasticDeformation * powf(elasticDet, -(1.0f / 3.0f));
                }

                // Exponentiate elastic deformation to compute a contribution to plastic deformation
                float plasticPower = FmMinFloat(creep * (elasticStressMag - yieldStress) / elasticStressMag, 1.0f);

                FmVector3 Fphat;
                Fphat.x = powf(elasticDeformation.x, plasticPower);
                Fphat.y = powf(elasticDeformation.y, plasticPower);
                Fphat.z = powf(elasticDeformation.z, plasticPower);

                FmMatrix3 contribution = V * FmMatrix3::scale(Fphat) * transpose(V);

                plasticDeformationMatrix = contribution * plasticDeformationMatrix;

                // Limit total plastic deformation by limiting singular values
                FmVector3 totalPlasticDeformation;
                FmSvd3x3(&U, &totalPlasticDeformation, &V, plasticDeformationMatrix);

                totalPlasticDeformation.x = FmMaxFloat(totalPlasticDeformation.x, minDeformation);
                totalPlasticDeformation.y = FmMaxFloat(totalPlasticDeformation.y, minDeformation);
                totalPlasticDeformation.z = FmMaxFloat(totalPlasticDeformation.z, minDeformation);

                totalPlasticDeformation.x = FmMinFloat(totalPlasticDeformation.x, maxDeformation);
                totalPlasticDeformation.y = FmMinFloat(totalPlasticDeformation.y, maxDeformation);
                totalPlasticDeformation.z = FmMinFloat(totalPlasticDeformation.z, maxDeformation);

                if (preserveVolume)
                {
                    // Resize total to preserve volume
                    float plasticDet = totalPlasticDeformation.x * totalPlasticDeformation.y * totalPlasticDeformation.z;
                    totalPlasticDeformation = totalPlasticDeformation * powf(plasticDet, -(1.0f / 3.0f));
                }

                plasticDeformationMatrix = V * FmMatrix3::scale(totalPlasticDeformation) * transpose(V);

                *outPlasticDeformationMatrix = plasticDeformationMatrix;
            }
        }

        return updateNeeded;
    }

#if FM_SOA_TET_MATH
    // Update state of the multiplicative plasticity model.
    // If return is true, outputs updated plasticDeformationMatrix and shape params.
    // References:
    // - Irving et al., "Invertible Finite Elements For Robust Simulation of Large Deformation"
    // - Bargteil et al., "A Finite Element Method for Animating Large Viscoplastic Flow"
    template<class T>
    typename T::SoaBool FmUpdateTetPlasticity(
        typename T::SoaMatrix3* outPlasticDeformationMatrix,
        const typename T::SoaMatrix3& inPlasticDeformationMatrix,
        const typename T::SoaMatrix3& tetRotationInv,
        const typename T::SoaMatrix3& restShapeMatrixInv,
        const typename T::SoaMatrix3& deformedShapeMatrix,
        const typename T::SoaVector3& stress0,
        const typename T::SoaVector3& stress1,
        typename T::SoaFloat plasticYieldThreshold, typename T::SoaFloat plasticCreep, typename T::SoaFloat plasticMin, typename T::SoaFloat plasticMax, typename T::SoaBool preserveVolume)
    {
        (void)tetRotationInv;
        typename T::SoaMatrix3 plasticDeformationMatrix = inPlasticDeformationMatrix;

        typename T::SoaFloat elasticStressMag = sqrtf(lengthSqr(stress0) + lengthSqr(stress1));
        typename T::SoaFloat yieldStress = plasticYieldThreshold;

        typename T::SoaBool stressOverYield = elasticStressMag > yieldStress;
        typename T::SoaBool updateNeeded = stressOverYield;

        if (any(updateNeeded))
        {
            // Compute deformation gradient (decomposed into SVD), after applying plastic deformation offset
            typename T::SoaMatrix3 U, V;
            typename T::SoaVector3 Fhat;
            FmComputeTetElasticDeformationGradient<T>(&U, &Fhat, &V,
                deformedShapeMatrix, restShapeMatrixInv, inverse(plasticDeformationMatrix));

            typename T::SoaVector3 elasticDeformation = Fhat;

            const typename T::SoaFloat deformationThreshold = 0.01f;

            typename T::SoaBool overThreshold = 
                (elasticDeformation.x > deformationThreshold)
                & (elasticDeformation.y > deformationThreshold)
                & (elasticDeformation.z > deformationThreshold);

            updateNeeded = updateNeeded & overThreshold;

            if (any(updateNeeded))       // Do nothing for low or inverted elastic deformation
            {
                typename T::SoaFloat creep = plasticCreep;
                typename T::SoaFloat minDeformation = max(plasticMin, deformationThreshold);
                typename T::SoaFloat maxDeformation = plasticMax;

                preserveVolume = preserveVolume & updateNeeded;
                if (any(preserveVolume))
                {
                    // Scale the elastic deformation such that determinant is 1.0, which represents a volume preserving deformation.
                    typename T::SoaFloat elasticDet = elasticDeformation.x * elasticDeformation.y * elasticDeformation.z;
                    elasticDeformation = select(elasticDeformation, elasticDeformation * powf(elasticDet, -(1.0f / 3.0f)), preserveVolume);
                }

                // Exponentiate elastic deformation to compute a contribution to plastic deformation
                typename T::SoaFloat plasticPower = min(creep * (elasticStressMag - yieldStress) / elasticStressMag, 1.0f);

                typename T::SoaVector3 Fphat;
                Fphat.x = powf(elasticDeformation.x, plasticPower);
                Fphat.y = powf(elasticDeformation.y, plasticPower);
                Fphat.z = powf(elasticDeformation.z, plasticPower);

                typename T::SoaMatrix3 contribution = V * typename T::SoaMatrix3::scale(Fphat) * transpose(V);

                plasticDeformationMatrix = contribution * plasticDeformationMatrix;

                // Limit total plastic deformation by limiting singular values
                typename T::SoaVector3 totalPlasticDeformation;
                FmSvd3x3<T>(&U, &totalPlasticDeformation, &V, plasticDeformationMatrix);

                totalPlasticDeformation.x = max(totalPlasticDeformation.x, minDeformation);
                totalPlasticDeformation.y = max(totalPlasticDeformation.y, minDeformation);
                totalPlasticDeformation.z = max(totalPlasticDeformation.z, minDeformation);

                totalPlasticDeformation.x = min(totalPlasticDeformation.x, maxDeformation);
                totalPlasticDeformation.y = min(totalPlasticDeformation.y, maxDeformation);
                totalPlasticDeformation.z = min(totalPlasticDeformation.z, maxDeformation);

                if (any(preserveVolume))
                {
                    // Resize total to preserve volume
                    typename T::SoaFloat plasticDet = totalPlasticDeformation.x * totalPlasticDeformation.y * totalPlasticDeformation.z;
                    totalPlasticDeformation = select(totalPlasticDeformation, totalPlasticDeformation * powf(plasticDet, -(1.0f / 3.0f)), preserveVolume);
                }

                plasticDeformationMatrix = V * typename T::SoaMatrix3::scale(totalPlasticDeformation) * transpose(V);

                *outPlasticDeformationMatrix = plasticDeformationMatrix;
            }
        }

        return updateNeeded;
    }
#endif

    void FmComputeUpdateTetStateOutput(
        FmUpdateTetStateOutput* tetUpdateOutput,
        const FmUpdateTetStateInput& tetUpdateInput,
        bool meshHasPlasticity, bool computingTetDeformation)
    {
        tetUpdateOutput->fractureStressThreshold = tetUpdateInput.fractureStressThreshold;
        tetUpdateOutput->testFracture = tetUpdateInput.testFracture;
        tetUpdateOutput->updatePlasticity = tetUpdateInput.updatePlasticity;

        FmMatrix3 deformationGradient = tetUpdateInput.deformedShapeMatrix * tetUpdateInput.shapeParams.DmInv;
        FmMatrix3 tetRotation = FmComputeTetRotationSvd(deformationGradient);

        tetUpdateOutput->tetRotation = tetRotation;

        FmMatrix3 stressShapeParamsDmInv = tetUpdateInput.shapeParams.DmInv;
        if (meshHasPlasticity)
        {
            stressShapeParamsDmInv *= inverse(tetUpdateInput.plasticDeformationMatrix);
        }

        FmTetStressMatrix stressMat;

#if !FM_MATRIX_ASSEMBLY_BY_TETS
        float stressShapeParamsV0 = tetUpdateInput.shapeParams.GetVolume();
        if (meshHasPlasticity)
        {
            stressShapeParamsV0 *= determinant(tetUpdateInput.plasticDeformationMatrix);
        }
        FmTetStiffnessMatrix stiffnessMat;
        FmComputeTetStressAndStiffnessMatrix(&stressMat, &stiffnessMat,
            stressShapeParamsDmInv,
            stressShapeParamsV0,
            tetUpdateInput.youngsModulus, tetUpdateInput.poissonsRatio);
#else
        FmComputeTetStressMatrix(&stressMat,
            stressShapeParamsDmInv,
            tetUpdateInput.youngsModulus, tetUpdateInput.poissonsRatio);
#endif

#if !FM_MATRIX_ASSEMBLY_BY_TETS
        // Compute rotated stiffness for matrix assembly
        FmComputeRotatedStiffnessMatrix(&tetUpdateOutput->rotatedStiffnessMat, stiffnessMat,
            tetUpdateInput.restShapeMatrix, tetRotation);
#endif

        if (computingTetDeformation || tetUpdateInput.testFracture || tetUpdateInput.updatePlasticity)
        {
            FmMatrix3 tetRotationInv = transpose(tetRotation);

            if (computingTetDeformation)
            {
                // Compute deformation offsets for strain calculation.
                // Use the original rest positions for total deformation.
                FmVector3 unrotatedOffset1 = (tetRotationInv * tetUpdateInput.deformedShapeMatrix.col0) - tetUpdateInput.restShapeMatrix.col0;
                FmVector3 unrotatedOffset2 = (tetRotationInv * tetUpdateInput.deformedShapeMatrix.col1) - tetUpdateInput.restShapeMatrix.col1;
                FmVector3 unrotatedOffset3 = (tetRotationInv * tetUpdateInput.deformedShapeMatrix.col2) - tetUpdateInput.restShapeMatrix.col2;

                FmVector3 strain[2];
                FmComputeTetStrain(strain, tetUpdateInput.shapeParams, unrotatedOffset1, unrotatedOffset2, unrotatedOffset3);
                float strainMag = sqrtf(lengthSqr(strain[0]) + lengthSqr(strain[1]));

                tetUpdateOutput->strainMag = strainMag;
            }
               
            if (tetUpdateInput.testFracture || tetUpdateInput.updatePlasticity)
            {
                // Compute elastic stress
                FmVector3 unrotatedOffset1 = (tetRotationInv * tetUpdateInput.deformedShapeMatrix.col0) - tetUpdateInput.stressRestShapeMatrix.col0;
                FmVector3 unrotatedOffset2 = (tetRotationInv * tetUpdateInput.deformedShapeMatrix.col1) - tetUpdateInput.stressRestShapeMatrix.col1;
                FmVector3 unrotatedOffset3 = (tetRotationInv * tetUpdateInput.deformedShapeMatrix.col2) - tetUpdateInput.stressRestShapeMatrix.col2;

                FmVector3 stress0 =
                    /* stressMat.EBe00 * unrotatedOffset0 + */
                    stressMat.EBe01 * unrotatedOffset1 +
                    stressMat.EBe02 * unrotatedOffset2 +
                    stressMat.EBe03 * unrotatedOffset3;
                FmVector3 stress1 =
                    /* stressMat.EBe10 * unrotatedOffset0 + */
                    stressMat.EBe11 * unrotatedOffset1 +
                    stressMat.EBe12 * unrotatedOffset2 +
                    stressMat.EBe13 * unrotatedOffset3;

                if (tetUpdateInput.testFracture)
                {
                    // Create symmetric 3x3 matrix from stress components
                    float sxx = stress0.x;
                    float syy = stress0.y;
                    float szz = stress0.z;
                    float syz = stress1.x;
                    float szx = stress1.y;
                    float sxy = stress1.z;

                    FmMatrix3 stress3(
                        FmVector3(sxx, sxy, szx),
                        FmVector3(sxy, syy, syz),
                        FmVector3(szx, syz, szz));

                    // primary direction of stress is eigenvector for largest eigenvalue
                    FmVector3 eigenvals;
                    FmMatrix3 eigenvecs;
                    FmEigenSym3x3(&eigenvals, &eigenvecs, stress3);

                    FmVector3 absEigenvals = abs(eigenvals);
                    FmVector3 maxDirection = eigenvecs.col0;
                    float maxEigenvalue = absEigenvals.x;
                    bool swap = absEigenvals.y > maxEigenvalue;
                    maxDirection = swap ? eigenvecs.col1 : maxDirection;
                    maxEigenvalue = swap ? absEigenvals.y : maxEigenvalue;
                    swap = absEigenvals.z > maxEigenvalue;
                    maxDirection = swap ? eigenvecs.col2 : maxDirection;
                    maxEigenvalue = swap ? absEigenvals.z : maxEigenvalue;

                    tetUpdateOutput->maxStressDirection = maxDirection;
                    tetUpdateOutput->maxStressEigenvalue = maxEigenvalue;
                }

                if (tetUpdateInput.updatePlasticity)
                {
                    bool updateNeeded = 
                        FmUpdateTetPlasticity(
                            &tetUpdateOutput->plasticDeformationMatrix,
                            tetUpdateInput.plasticDeformationMatrix,
                            tetRotationInv,
                            tetUpdateInput.shapeParams.DmInv,
                            tetUpdateInput.deformedShapeMatrix,
                            stress0,
                            stress1,
                            tetUpdateInput.plasticYieldThreshold, tetUpdateInput.plasticCreep, tetUpdateInput.plasticMin, tetUpdateInput.plasticMax, tetUpdateInput.preserveVolume);

                    tetUpdateOutput->updatePlasticity = updateNeeded;
                }
            }
        }
    }

#if FM_SOA_TET_MATH
    template<class T>
    void FmComputeUpdateTetStateOutput(
        FmSoaUpdateTetStateOutput<T>* tetUpdateOutput,
        const FmSoaUpdateTetStateInput<T>& tetUpdateInput,
        bool meshHasPlasticity, bool computingTetDeformation)
    {
        tetUpdateOutput->fractureStressThreshold = tetUpdateInput.fractureStressThreshold;
        tetUpdateOutput->testFracture = tetUpdateInput.testFracture;
        tetUpdateOutput->updatePlasticity = tetUpdateInput.updatePlasticity;

        typename T::SoaMatrix3 deformationGradient = tetUpdateInput.deformedShapeMatrix * tetUpdateInput.shapeParams.DmInv;
        typename T::SoaMatrix3 tetRotation = FmComputeTetRotationSvd<T>(deformationGradient);

        tetUpdateOutput->tetRotation = tetRotation;

        typename T::SoaMatrix3 stressShapeParamsDmInv = tetUpdateInput.shapeParams.DmInv;
        if (meshHasPlasticity)
        {
            stressShapeParamsDmInv *= inverse(tetUpdateInput.plasticDeformationMatrix);
        }

        FmSoaTetStressMatrix<T> stressMat;

#if !FM_MATRIX_ASSEMBLY_BY_TETS
        typename T::SoaFloat stressShapeParamsV0 = tetUpdateInput.shapeParams.GetVolume();
        if (meshHasPlasticity)
        {
            stressShapeParamsV0 *= determinant(tetUpdateInput.plasticDeformationMatrix);
        }
        FmSoaTetStiffnessMatrix<T> stiffnessMat;
        FmComputeTetStressAndStiffnessMatrix(&stressMat, &stiffnessMat,
            stressShapeParamsDmInv,
            stressShapeParamsV0,
            tetUpdateInput.youngsModulus, tetUpdateInput.poissonsRatio);
#else
        FmComputeTetStressMatrix<T>(&stressMat,
            stressShapeParamsDmInv,
            tetUpdateInput.youngsModulus, tetUpdateInput.poissonsRatio);
#endif

#if !FM_MATRIX_ASSEMBLY_BY_TETS
        // Compute rotated stiffness for matrix assembly
        FmComputeRotatedStiffnessMatrix(&tetUpdateOutput->rotatedStiffnessMat, stiffnessMat,
            tetUpdateInput.restShapeMatrix, tetRotation);
#endif

        if (computingTetDeformation || any(tetUpdateInput.testFracture | tetUpdateInput.updatePlasticity))
        {
            typename T::SoaMatrix3 tetRotationInv = transpose(tetRotation);

            if (computingTetDeformation)
            {
                // Compute deformation offsets for strain calculation.
                // Use the original rest positions for total deformation.
                typename T::SoaVector3 unrotatedOffset1 = (tetRotationInv * tetUpdateInput.deformedShapeMatrix.col0) - tetUpdateInput.restShapeMatrix.col0;
                typename T::SoaVector3 unrotatedOffset2 = (tetRotationInv * tetUpdateInput.deformedShapeMatrix.col1) - tetUpdateInput.restShapeMatrix.col1;
                typename T::SoaVector3 unrotatedOffset3 = (tetRotationInv * tetUpdateInput.deformedShapeMatrix.col2) - tetUpdateInput.restShapeMatrix.col2;

                typename T::SoaVector3 strain[2];
                FmComputeTetStrain<T>(strain, tetUpdateInput.shapeParams, unrotatedOffset1, unrotatedOffset2, unrotatedOffset3);
                typename T::SoaFloat tetDeformation = sqrtf(lengthSqr(strain[0]) + lengthSqr(strain[1]));

                tetUpdateOutput->strainMag = tetDeformation;
            }

            if (any(tetUpdateInput.testFracture | tetUpdateInput.updatePlasticity))
            {
                // Compute elastic stress
                typename T::SoaVector3 unrotatedOffset1 = (tetRotationInv * tetUpdateInput.deformedShapeMatrix.col0) - tetUpdateInput.stressRestShapeMatrix.col0;
                typename T::SoaVector3 unrotatedOffset2 = (tetRotationInv * tetUpdateInput.deformedShapeMatrix.col1) - tetUpdateInput.stressRestShapeMatrix.col1;
                typename T::SoaVector3 unrotatedOffset3 = (tetRotationInv * tetUpdateInput.deformedShapeMatrix.col2) - tetUpdateInput.stressRestShapeMatrix.col2;

                typename T::SoaVector3 stress0 =
                    /* stressMat.EBe00 * unrotatedOffset0 + */
                    stressMat.EBe01 * unrotatedOffset1 +
                    stressMat.EBe02 * unrotatedOffset2 +
                    stressMat.EBe03 * unrotatedOffset3;
                typename T::SoaVector3 stress1 =
                    /* stressMat.EBe10 * unrotatedOffset0 + */
                    stressMat.EBe11 * unrotatedOffset1 +
                    stressMat.EBe12 * unrotatedOffset2 +
                    stressMat.EBe13 * unrotatedOffset3;

                if (any(tetUpdateInput.testFracture))
                {
                    // Create symmetric 3x3 matrix from stress components
                    typename T::SoaFloat sxx = stress0.x;
                    typename T::SoaFloat syy = stress0.y;
                    typename T::SoaFloat szz = stress0.z;
                    typename T::SoaFloat syz = stress1.x;
                    typename T::SoaFloat szx = stress1.y;
                    typename T::SoaFloat sxy = stress1.z;

                    typename T::SoaMatrix3 stress3(
                        typename T::SoaVector3(sxx, sxy, szx),
                        typename T::SoaVector3(sxy, syy, syz),
                        typename T::SoaVector3(szx, syz, szz));

                    // primary direction of stress is eigenvector for largest eigenvalue
                    typename T::SoaVector3 eigenvals;
                    typename T::SoaMatrix3 eigenvecs;
                    FmEigenSym3x3<T>(&eigenvals, &eigenvecs, stress3);

                    typename T::SoaVector3 absEigenvals = abs(eigenvals);
                    typename T::SoaVector3 maxDirection = eigenvecs.col0;
                    typename T::SoaFloat maxEigenvalue = absEigenvals.x;
                    typename T::SoaBool swap = absEigenvals.y > maxEigenvalue;
                    maxDirection = select(maxDirection, eigenvecs.col1, swap);
                    maxEigenvalue = select(maxEigenvalue, absEigenvals.y, swap);
                    swap = absEigenvals.z > maxEigenvalue;
                    maxDirection = select(maxDirection, eigenvecs.col2, swap);
                    maxEigenvalue = select(maxEigenvalue, absEigenvals.z, swap);

                    tetUpdateOutput->maxStressDirection = maxDirection;
                    tetUpdateOutput->maxStressEigenvalue = maxEigenvalue;
                }

                if (any(tetUpdateInput.updatePlasticity))
                {
                    typename T::SoaBool updateNeeded = 
                        FmUpdateTetPlasticity<T>(
                            &tetUpdateOutput->plasticDeformationMatrix,
                            tetUpdateInput.plasticDeformationMatrix,
                            tetRotationInv,
                            tetUpdateInput.shapeParams.DmInv,
                            tetUpdateInput.deformedShapeMatrix,
                            stress0,
                            stress1,
                            tetUpdateInput.plasticYieldThreshold, tetUpdateInput.plasticCreep, tetUpdateInput.plasticMin, tetUpdateInput.plasticMax, tetUpdateInput.preserveVolume);

                    tetUpdateOutput->updatePlasticity = tetUpdateInput.updatePlasticity & updateNeeded;
                }
            }
        }
    }
#endif

    bool FmStoreUpdatedTetState(FmTetMesh* tetMesh, uint tetId,
        const FmUpdateTetStateOutput& updatedState,
        bool computingStrainMag)
    {
        FmTetVertIds tetVertIds = tetMesh->tetsVertIds[tetId];

        // Store tet rotation
        tetMesh->tetsRotation[tetId] = updatedState.tetRotation;

        // Store strain mag
        if (computingStrainMag)
        {
            tetMesh->tetsStrainMag[tetId] = updatedState.strainMag;
        }

#if !FM_MATRIX_ASSEMBLY_BY_TETS
        tetMesh->tetsStiffness[tetId].rotatedStiffnessMat = updatedState.rotatedStiffnessMat;
#endif

        if (updatedState.updatePlasticity)
        {
            FmTetPlasticityState& plasticityState = tetMesh->tetsPlasticity[tetId];

            plasticityState.plasticDeformationMatrix = updatedState.plasticDeformationMatrix;
        }

        bool removedKinematicFlag = false;

        if (updatedState.testFracture)
        {
            if (updatedState.maxStressEigenvalue > updatedState.fractureStressThreshold)
            {
                uint tetToFractureIndex = FmAtomicIncrement(&tetMesh->numTetsToFracture.val) - 1;
                FmTetToFracture& tetToFracture = tetMesh->tetsToFracture[tetToFractureIndex];
                tetToFracture.tetId = tetId;
                tetToFracture.fractureDirection = updatedState.maxStressDirection; // direction in element's rest orientation
            }

            // Break kinematic flags based on stress threshold, if allowed.
            // Multiple threads may update same vertex.
            if (updatedState.maxStressEigenvalue > tetMesh->removeKinematicStressThreshold)
            {
                for (uint cornerIdx = 0; cornerIdx < 4; cornerIdx++)
                {
                    uint vIdx = tetVertIds.ids[cornerIdx];

                    if (FM_ALL_SET(tetMesh->vertsFlags[vIdx], FM_VERT_FLAG_KINEMATIC_REMOVABLE | FM_VERT_FLAG_KINEMATIC))
                    {
                        tetMesh->vertsFlags[vIdx] &= ~FM_VERT_FLAG_KINEMATIC;
                        removedKinematicFlag = true;
                    }
                }
            }
        }

        return removedKinematicFlag;
    }

    // Accumulate tet quaternion and strain values in vertices
    void FmAccumulateTetQuatAndStrainMag(FmTetMesh* tetMesh, bool computingStrainMag)
    {
        uint numTets = tetMesh->numTets;
        for (uint tetId = 0; tetId < numTets; tetId++)
        {
            float tetStrainMag = tetMesh->tetsStrainMag[tetId];
            FmTetVertIds tetVertIds = tetMesh->tetsVertIds[tetId];
            uint vId0 = tetVertIds.ids[0];
            uint vId1 = tetVertIds.ids[1];
            uint vId2 = tetVertIds.ids[2];
            uint vId3 = tetVertIds.ids[3];

            FmQuat tetQuat = FmQuat(tetMesh->tetsRotation[tetId]);

            // Accumulate quaternions for average vertex orientation
            FmAddToQuat(&tetMesh->vertsTetValues[vId0].tetQuatSum, tetQuat);
            FmAddToQuat(&tetMesh->vertsTetValues[vId1].tetQuatSum, tetQuat);
            FmAddToQuat(&tetMesh->vertsTetValues[vId2].tetQuatSum, tetQuat);
            FmAddToQuat(&tetMesh->vertsTetValues[vId3].tetQuatSum, tetQuat);

            if (computingStrainMag)
            {
                tetMesh->vertsTetValues[vId0].tetStrainMagAvg += tetStrainMag * (1.0f / (float)tetMesh->vertsNeighbors[vId0].numIncidentTets);
                tetMesh->vertsTetValues[vId1].tetStrainMagAvg += tetStrainMag * (1.0f / (float)tetMesh->vertsNeighbors[vId1].numIncidentTets);
                tetMesh->vertsTetValues[vId2].tetStrainMagAvg += tetStrainMag * (1.0f / (float)tetMesh->vertsNeighbors[vId2].numIncidentTets);
                tetMesh->vertsTetValues[vId3].tetStrainMagAvg += tetStrainMag * (1.0f / (float)tetMesh->vertsNeighbors[vId3].numIncidentTets);

                tetMesh->vertsTetValues[vId0].tetStrainMagMax = std::max(tetMesh->vertsTetValues[vId0].tetStrainMagMax, tetMesh->vertsTetValues[vId0].tetStrainMagAvg);
                tetMesh->vertsTetValues[vId1].tetStrainMagMax = std::max(tetMesh->vertsTetValues[vId1].tetStrainMagMax, tetMesh->vertsTetValues[vId1].tetStrainMagAvg);
                tetMesh->vertsTetValues[vId2].tetStrainMagMax = std::max(tetMesh->vertsTetValues[vId2].tetStrainMagMax, tetMesh->vertsTetValues[vId2].tetStrainMagAvg);
                tetMesh->vertsTetValues[vId3].tetStrainMagMax = std::max(tetMesh->vertsTetValues[vId3].tetStrainMagMax, tetMesh->vertsTetValues[vId3].tetStrainMagAvg);
            }
        }
    }

    // Update a range of tets from beginIdx to endIdx.
    // runFracture = mesh supports fracture and fracture processing should be done.
    // updatePlasticity = mesh supports plasticity and plasticity processing should be done.
    void FmUpdateTetsRange(
        bool* outRemovedKinematicFlag,
        uint* outMaxUnconstrainedSolveIterations,
        FmTetMesh* tetMesh, uint beginIdx, uint endIdx,
        bool runFracture, bool updatePlasticity, bool meshSupportsPlasticity, bool computingStrainMag)
    {
        bool removedKinematicFlag = false;

        // Compute max iterations over tetrahedra
        uint maxUnconstrainedSolveIterations = 0;

#if FM_SOA_TET_MATH
        FmUpdateTetStateBatch<FmSoaTypes> tetUpdateBatch;
        tetUpdateBatch.baseTetIdx = beginIdx;
#endif

        // For all tets compute rotation, update stiffness matrices
        // Optionally update plasticity state, test fracture and collect fracturing tets
        for (uint tetId = beginIdx; tetId < endIdx; tetId++)
        {
            uint tetMaxUnconstrainedSolveIterations = tetMesh->tetsMaxUnconstrainedSolveIterations[tetId];

            maxUnconstrainedSolveIterations = FmMaxUint(maxUnconstrainedSolveIterations, tetMaxUnconstrainedSolveIterations);

            // Collect input data from tet
            FmUpdateTetStateInput tetUpdateInput;
            FmGatherUpdateTetStateInput(&tetUpdateInput, *tetMesh, tetId, runFracture, updatePlasticity);

            // Compute tet rotations, stress, stiffness matrices
#if FM_SOA_TET_MATH
            uint batchNumTets = tetUpdateBatch.numTets;

            if (batchNumTets == FmSoaTypes::width)
            {
                tetUpdateBatch.ConvertAosToSoa(meshSupportsPlasticity);
                FmComputeUpdateTetStateOutput<FmSoaTypes>(&tetUpdateBatch.output, tetUpdateBatch.input, meshSupportsPlasticity, computingStrainMag);

                uint baseTetIdx = tetUpdateBatch.baseTetIdx;

                for (uint batchIdx = 0; batchIdx < batchNumTets; batchIdx++)
                {
                    FmUpdateTetStateOutput tetUpdateOutput;
                    tetUpdateBatch.GetBatchSlice(&tetUpdateOutput, batchIdx, meshSupportsPlasticity, computingStrainMag);

                    FmStoreUpdatedTetState(tetMesh, baseTetIdx + batchIdx, tetUpdateOutput, computingStrainMag);
                }

                tetUpdateBatch.baseTetIdx = tetId;
                tetUpdateBatch.numTets = 0;
            }

            tetUpdateBatch.SetBatchSlice(tetUpdateBatch.numTets, tetUpdateInput, meshSupportsPlasticity);
            tetUpdateBatch.numTets++;
#else
            FmUpdateTetStateOutput tetUpdateOutput;
            FmComputeUpdateTetStateOutput(&tetUpdateOutput, tetUpdateInput, meshSupportsPlasticity, computingStrainMag);

            FmStoreUpdatedTetState(tetMesh, tetId, tetUpdateOutput, computingStrainMag);
#endif
        }

#if FM_SOA_TET_MATH
        uint batchNumTets = tetUpdateBatch.numTets;
        if (batchNumTets > 0)
        {
            tetUpdateBatch.ConvertAosToSoa(meshSupportsPlasticity);
            FmComputeUpdateTetStateOutput<FmSoaTypes>(&tetUpdateBatch.output, tetUpdateBatch.input, meshSupportsPlasticity, computingStrainMag);

            uint baseTetIdx = tetUpdateBatch.baseTetIdx;

            for (uint batchIdx = 0; batchIdx < batchNumTets; batchIdx++)
            {
                FmUpdateTetStateOutput tetUpdateOutput;
                tetUpdateBatch.GetBatchSlice(&tetUpdateOutput, batchIdx, meshSupportsPlasticity, computingStrainMag);

                FmStoreUpdatedTetState(tetMesh, baseTetIdx + batchIdx, tetUpdateOutput, computingStrainMag);
            }
        }
#endif

        *outRemovedKinematicFlag = removedKinematicFlag;
        *outMaxUnconstrainedSolveIterations = maxUnconstrainedSolveIterations;
    }

    class FmTaskDataUpdateTetState : public TLTaskDataBase
    {
    public:
        FM_CLASS_NEW_DELETE(FmTaskDataUpdateTetState)

        FmAtomicUint removedKinematicFlag;
        FmAtomicUint maxUnconstrainedSolveIterations;

        FmScene* scene;
        FmTetMesh* tetMesh;
        uint numTets;
        uint numTetTasks;

        bool runFracture;
        bool updatePlasticity;
        bool meshSupportsPlasticity;
        bool computingStrainMag;

        TLTaskDataBase* updateIslandsProgress;

        FmTaskDataUpdateTetState(
            FmScene* inScene, FmTetMesh* inTetMesh,
            uint inNumTets, uint inNumTetTasks,
            bool inRunFracture, bool inUpdatePlasticity, bool inMeshSupportsPlasticity, bool inComputingDeformation)
        {
            removedKinematicFlag.val = 0;
            maxUnconstrainedSolveIterations.val = 0;
            scene = inScene;
            tetMesh = inTetMesh;
            numTets = inNumTets;
            numTetTasks = inNumTetTasks;
            runFracture = inRunFracture;
            updatePlasticity = inUpdatePlasticity;
            meshSupportsPlasticity = inMeshSupportsPlasticity;
            computingStrainMag = inComputingDeformation;
            updateIslandsProgress = nullptr;
        }
    };

    void FmTaskFuncUpdateTetsBatch(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT("UpdateTetState");

        FmTaskDataUpdateTetState* taskData = (FmTaskDataUpdateTetState *)inTaskData;

        FmTetMesh* tetMesh = taskData->tetMesh;
        bool runFracture = taskData->runFracture;
        bool updatePlasticity = taskData->updatePlasticity;
        bool meshSupportsPlasticity = taskData->meshSupportsPlasticity;
        bool computingStrainMag = taskData->computingStrainMag;

        uint beginIdx, endIdx;
        FmGetIndexRange(&beginIdx, &endIdx, (uint)inTaskBeginIndex, FM_UPDATE_TET_BATCH_SIZE, taskData->numTets);

        bool removedKinematicFlag;
        uint maxUnconstrainedSolveIterations;

        FmUpdateTetsRange(&removedKinematicFlag, &maxUnconstrainedSolveIterations, tetMesh, beginIdx, endIdx, runFracture, updatePlasticity, meshSupportsPlasticity, computingStrainMag);

        if (removedKinematicFlag)
        {
            FmAtomicWrite(&taskData->removedKinematicFlag.val, 1);
        }

        FmAtomicMax(&taskData->maxUnconstrainedSolveIterations.val, maxUnconstrainedSolveIterations);
    }

    FM_ASYNC_TASK(FmTaskFuncFinishUpdateTetsAndFracture)    {
        FM_TRACE_SCOPED_EVENT("FinishUpdateTetsFracture");

        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmTaskDataUpdateTetState* taskData = (FmTaskDataUpdateTetState *)inTaskData;

        FmScene* scene = taskData->scene;
        FmTetMesh* tetMesh = taskData->tetMesh;
        bool runFracture = taskData->runFracture;
        bool removedKinematicFlag = (FmAtomicRead(&taskData->removedKinematicFlag.val) != 0);
        bool computingStrainMag = taskData->computingStrainMag;
        uint maxUnconstrainedSolveIterations = FmAtomicRead(&taskData->maxUnconstrainedSolveIterations.val);

        if (removedKinematicFlag)
        {
            tetMesh->flags |= FM_OBJECT_FLAG_REMOVED_KINEMATIC;
        }
        tetMesh->maxUnconstrainedSolveIterations = maxUnconstrainedSolveIterations;

        FmAccumulateTetQuatAndStrainMag(tetMesh, computingStrainMag);

        if (runFracture)
        {
            FmFractureMesh(scene, tetMesh);
        }

        // Mark progress in island
#if FM_ASYNC_THREADING
        taskData->updateIslandsProgress->WorkItemFinished(taskData->updateIslandsProgress);
#else
        taskData->updateIslandsProgress->WorkItemFinished();
#endif

        delete taskData;
    }

    // Compute tet rotation and rotated stiffness matrix.
    // Compute plastic deformation state change.
    // Create list of tets with sufficient stress for fracture.
    // Set mesh's unconstrained solver iterations as max of tet settings.
    void FmUpdateTetStateAndFracture(FmScene* scene, FmTetMesh* tetMesh, 
        bool runFracture, bool updatePlasticity,
        TLTaskDataBase* updateIslandsProgress)
    {
        uint numTets = tetMesh->numTets;

        // Clear fracture state
        // Keep existing crack tips to continue fracture from them
        tetMesh->numTetsToFracture.val = 0;
        tetMesh->flags &= ~(FM_OBJECT_FLAG_REMOVED_KINEMATIC | FM_OBJECT_FLAG_HIT_MAX_VERTS | FM_OBJECT_FLAG_HIT_MAX_EXTERIOR_FACES);

        bool meshSupportsFracture = (tetMesh->tetsToFracture != nullptr);
        bool meshSupportsPlasticity = (tetMesh->tetsPlasticity != nullptr);

        // Set runFracture only if fracture supported on this mesh and scene pointer valid
        runFracture = runFracture && meshSupportsFracture && (scene != nullptr);

        bool computingStrainMag = FM_IS_SET(tetMesh->flags, FM_OBJECT_FLAG_COMPUTE_TET_STRAIN_MAG);
        
#if FM_ASYNC_THREADING
        uint numUpdateTetTasks = FmGetNumTasks(tetMesh->numTets, FM_UPDATE_TET_BATCH_SIZE);

        if (updateIslandsProgress)
        {
            if (numTets > 128)
            {
                // If enough tets, parallelize using a parallel for
                FmTaskDataUpdateTetState* taskData = new FmTaskDataUpdateTetState(
                    scene, tetMesh, numTets, numUpdateTetTasks, runFracture, updatePlasticity, meshSupportsPlasticity, computingStrainMag);

                taskData->updateIslandsProgress = updateIslandsProgress;

                TLTask followTask(FmTaskFuncFinishUpdateTetsAndFracture, taskData);

                TLParallelForAsync(FmTaskFuncUpdateTetsBatch, taskData, 0, numUpdateTetTasks, TLParallelForOptions::GrainSize(1), followTask, false);
                TLFlushTaskQueue();
            }
            else
            {
                // Update tets and record the completion of task/mesh in global progress
                bool removedKinematicFlag;
                uint maxUnconstrainedSolveIterations;

                FmUpdateTetsRange(&removedKinematicFlag, &maxUnconstrainedSolveIterations, tetMesh, 0, numTets, runFracture, updatePlasticity, meshSupportsPlasticity, computingStrainMag);

                if (removedKinematicFlag)
                {
                    tetMesh->flags |= FM_OBJECT_FLAG_REMOVED_KINEMATIC;
                }
                tetMesh->maxUnconstrainedSolveIterations = maxUnconstrainedSolveIterations;

                FmAccumulateTetQuatAndStrainMag(tetMesh, computingStrainMag);

                if (runFracture)
                {
                    FmFractureMesh(scene, tetMesh);
                }

                updateIslandsProgress->WorkItemFinished(updateIslandsProgress);
            }
        }
        else
#endif
        {
            (void)updateIslandsProgress;

            bool removedKinematicFlag;
            uint maxUnconstrainedSolveIterations;

            FmUpdateTetsRange(&removedKinematicFlag, &maxUnconstrainedSolveIterations, tetMesh, 0, numTets, runFracture, updatePlasticity, meshSupportsPlasticity, computingStrainMag);

            if (removedKinematicFlag)
            {
                tetMesh->flags |= FM_OBJECT_FLAG_REMOVED_KINEMATIC;
            }
            tetMesh->maxUnconstrainedSolveIterations = maxUnconstrainedSolveIterations;

            FmAccumulateTetQuatAndStrainMag(tetMesh, computingStrainMag);

            if (runFracture)
            {
                FmFractureMesh(scene, tetMesh);
            }
        }
    }
}
