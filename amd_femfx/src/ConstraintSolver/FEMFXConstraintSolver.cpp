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
// Implementation of constraint solver
//---------------------------------------------------------------------------------------

#include "FEMFXConstraintSolver.h"
#include "FEMFXConstraintIslands.h"
#include "FEMFXGsSolver.h"
#include "FEMFXThreading.h"
#include "FEMFXPartitioning.h"
#include "FEMFXConstraintSolveTaskGraph.h"
#include "FEMFXScene.h"
#include "FEMFXThreadTempMemory.h"
#include "FEMFXSleeping.h"
#include "FEMFXUpdateTetState.h"

#define FM_RIGID_BODY_APPLY_DELTAS_SLEEPING_BATCH_SIZE 128

// Bound memory given maximum total elements, split into maximum number of separate arrays.
// If more than one array, assumes each may have extra padding of alignment-1
#define FM_PAD_ARRAY_16(type, maxElements, maxArrays) FM_PAD_16((sizeof(type)*(size_t)(maxElements)) + ((maxArrays == 1)? 0 : (size_t)maxArrays*15))
#define FM_PAD_ARRAY_64(type, maxElements, maxArrays) FM_PAD_64((sizeof(type)*(size_t)(maxElements)) + ((maxArrays == 1)? 0 : (size_t)maxArrays*63))

namespace AMD
{

    size_t FmGetConstraintIslandSolverDataSize(uint numStateVecs3, uint numObjects, uint numConstraints, uint numJacobianSubmats, uint maxSubIslands)
    {
        uint maxBvhNodes = FmNumBvhNodes(numObjects);
        uint maxPartitions = numObjects;
        uint maxPartitionPairs = numConstraints;
        uint numPartitionPairSetElements = maxPartitionPairs * 2;
        uint maxPartitionObjects = numConstraints * 2;
        uint numPartitionObjectSetElements = maxPartitionObjects * 2;

        // maxSubIslands used to bound size of padding when buffer is split between islands.

        uint numBytes =
            FM_PAD_ARRAY_16(*FmConstraintSolverData::DAinv, numStateVecs3, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::W, numStateVecs3, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::JTlambda, numStateVecs3, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::J.params, numConstraints, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::J.submats, numJacobianSubmats, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::J.indices, numJacobianSubmats, maxSubIslands)
#if FM_CONSTRAINT_STABILIZATION_SOLVE
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::deltaPos, numStateVecs3, maxSubIslands)
#endif
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::deltaVel, numStateVecs3, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::pgsDeltaVelTerm, numStateVecs3, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::velTemp, numStateVecs3, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::pgsRhsConstant, numConstraints, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::pgsRhs, numConstraints, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::lambda3, numConstraints, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::lambda3Temp, numConstraints, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::objectPartitionData, numObjects, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::allPartitionConstraintIndices, numConstraints, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::allPartitionObjectIds, maxPartitionObjects, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::allPartitionObjectNumVertices, maxPartitionObjects, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::allPartitionObjectSetElements, numPartitionObjectSetElements, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::partitionPairSet.elements, numPartitionPairSetElements, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::partitionPairs, maxPartitionPairs, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::partitionsHierarchy.nodes, maxBvhNodes, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::partitionsHierarchy.primBoxes, numObjects, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::partitionsHierarchy.mortonCodesSorted, numObjects, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::partitionsHierarchy.primIndicesSorted, numObjects, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::nodeCounts, maxBvhNodes, maxSubIslands)
            + FM_PAD_ARRAY_64(*FmConstraintSolverData::partitions, maxPartitions, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::partitionPairMinSetElements, maxPartitionPairs, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::partitionPairMaxSetElements, maxPartitionPairs, maxSubIslands)
            + FM_PAD_ARRAY_16(*FmConstraintSolverData::partitionPairIndependentSets, maxPartitionPairs, maxSubIslands);

        // Bound the padding after each island since padding up to 64-bytes
        size_t max64BytePadding = (maxSubIslands == 1)? 0 : (size_t)maxSubIslands*63;

        return FM_PAD_64(numBytes + max64BytePadding);
    }

    size_t FmEstimateSceneConstraintSolverDataSize(const FmSceneSetupParams& params)
    {
        uint maxTetMeshes = params.maxTetMeshes;
        uint maxTetMeshVerts = params.maxSceneVerts;
        uint maxRigidBodies = params.maxRigidBodies;
        uint maxObjects = maxTetMeshes + maxRigidBodies;
        uint maxConstraintIslands = maxTetMeshes + maxRigidBodies;
        uint maxConstraints = params.maxDistanceContacts + params.maxVolumeContacts + params.maxGlueConstraints + params.maxPlaneConstraints + params.maxDeformationConstraints;
        uint maxStateVecs3 = maxTetMeshVerts + maxRigidBodies * 2;

        uint estNumJacobianSubmats = maxConstraints * 4;

        return FmGetConstraintIslandSolverDataSize(maxStateVecs3, maxObjects, maxConstraints, estNumJacobianSubmats, maxConstraintIslands);
    }

    size_t FmGetConstraintSolverBufferSize(const FmConstraintSolverBufferSetupParams& params)
    {
        uint maxTetMeshes = params.maxTetMeshes;
        uint maxRigidBodies = params.maxRigidBodies;
        uint maxConstraintIslands = maxTetMeshes + maxRigidBodies;
        size_t maxConstraintSolverDataSize = params.maxConstraintSolverDataSize;

        size_t numBytes =
            FM_PAD_64(sizeof(FmConstraintSolverBuffer))
            + FM_PAD_64(sizeof(*FmConstraintSolverBuffer::islandSolverData) * maxConstraintIslands)
            + FM_PAD_16(sizeof(*FmConstraintSolverBuffer::rigidBodySolverOffsets) * maxRigidBodies)
            + FM_PAD_16(sizeof(*FmConstraintSolverBuffer::tetMeshIdxFromId) * maxTetMeshes)
            + FM_PAD_16(sizeof(*FmConstraintSolverBuffer::rigidBodyIdxFromId) * maxRigidBodies)
            + maxConstraintSolverDataSize;

        return FM_PAD_64(numBytes);
    }

    void FmAllocConstraintIslandSolverDataFromBuffer(
        FmConstraintSolverData* constraintSolverData,
        FmConstraintIsland& island,
        uint8_t*& pBuffer, uint8_t* pBufferEnd)
    {
        FM_ASSERT(((uintptr_t)pBuffer & 0x3f) == 0);

        uint numStateVecs3 = island.numStateVecs3;
        uint numConstraints = island.numConstraints;
        uint numJacobianSubmats = island.numJacobianSubmats;

        uint numObjects = island.numTetMeshes + island.numRigidBodiesInFEMSolve;
        uint maxBvhNodes = FmNumBvhNodes(numObjects);
        uint maxPartitions = numObjects;
        uint maxPartitionPairs = numConstraints;
        uint numPartitionPairSetElements = maxPartitionPairs * 2;
        uint maxPartitionObjects = numConstraints * 2;
        uint numPartitionObjectSetElements = maxPartitionObjects * 2;

        FmPartition* partitions = FmAllocFromBuffer64<FmPartition>(&pBuffer, maxPartitions, pBufferEnd);

        constraintSolverData->DAinv = FmAllocFromBuffer<FmSMatrix3>(&pBuffer, numStateVecs3, pBufferEnd);
        constraintSolverData->W = FmAllocFromBuffer<FmSMatrix3>(&pBuffer, numStateVecs3, pBufferEnd);
        constraintSolverData->JTlambda = FmAllocFromBuffer<FmSVector3>(&pBuffer, numStateVecs3, pBufferEnd);
        constraintSolverData->J.params = FmAllocFromBuffer<FmConstraintParams>(&pBuffer, numConstraints, pBufferEnd);
        constraintSolverData->J.submats = FmAllocFromBuffer<FmSMatrix3>(&pBuffer, numJacobianSubmats, pBufferEnd);
        constraintSolverData->J.indices = FmAllocFromBuffer<uint>(&pBuffer, numJacobianSubmats, pBufferEnd);
#if FM_CONSTRAINT_STABILIZATION_SOLVE
        constraintSolverData->deltaPos = FmAllocFromBuffer<FmSVector3>(&pBuffer, numStateVecs3, pBufferEnd);
#endif
        constraintSolverData->deltaVel = FmAllocFromBuffer<FmSVector3>(&pBuffer, numStateVecs3, pBufferEnd);
        constraintSolverData->pgsDeltaVelTerm = FmAllocFromBuffer<FmSVector3>(&pBuffer, numStateVecs3, pBufferEnd);
        constraintSolverData->velTemp = FmAllocFromBuffer<FmSVector3>(&pBuffer, numStateVecs3, pBufferEnd);
        constraintSolverData->pgsRhsConstant = FmAllocFromBuffer<FmSVector3>(&pBuffer, numConstraints, pBufferEnd);
        constraintSolverData->pgsRhs = FmAllocFromBuffer<FmSVector3>(&pBuffer, numConstraints, pBufferEnd);
        constraintSolverData->lambda3 = FmAllocFromBuffer<FmSVector3>(&pBuffer, numConstraints, pBufferEnd);
        constraintSolverData->lambda3Temp = FmAllocFromBuffer<FmSVector3>(&pBuffer, numConstraints, pBufferEnd);
        constraintSolverData->objectPartitionData = FmAllocFromBuffer<FmObjectPartitionData>(&pBuffer, numObjects, pBufferEnd);
        constraintSolverData->allPartitionConstraintIndices = FmAllocFromBuffer<uint>(&pBuffer, numConstraints, pBufferEnd);
        constraintSolverData->allPartitionObjectIds = FmAllocFromBuffer<uint>(&pBuffer, maxPartitionObjects, pBufferEnd);
        constraintSolverData->allPartitionObjectNumVertices = FmAllocFromBuffer<uint>(&pBuffer, maxPartitionObjects, pBufferEnd);
        constraintSolverData->allPartitionObjectSetElements = FmAllocFromBuffer<FmPartitionObjectSetElement>(&pBuffer, numPartitionObjectSetElements, pBufferEnd);
        FmPartitionPairSetElement *partitionPairSetElements = FmAllocFromBuffer<FmPartitionPairSetElement>(&pBuffer, numPartitionPairSetElements, pBufferEnd);

        FmInitHashSet(&constraintSolverData->partitionPairSet, partitionPairSetElements, numPartitionPairSetElements);

        constraintSolverData->partitionPairs = FmAllocFromBuffer<FmPartitionPair>(&pBuffer, maxPartitionPairs, pBufferEnd);

        constraintSolverData->partitionsHierarchy.nodes = FmAllocFromBuffer<FmBvhNode>(&pBuffer, maxBvhNodes, pBufferEnd);
        constraintSolverData->partitionsHierarchy.primBoxes = FmAllocFromBuffer<FmAabb>(&pBuffer, numObjects, pBufferEnd);
        constraintSolverData->partitionsHierarchy.mortonCodesSorted = FmAllocFromBuffer<int>(&pBuffer, numObjects, pBufferEnd);
        constraintSolverData->partitionsHierarchy.primIndicesSorted = FmAllocFromBuffer<int>(&pBuffer, numObjects, pBufferEnd);
        constraintSolverData->nodeCounts = FmAllocFromBuffer<uint>(&pBuffer, maxBvhNodes, pBufferEnd);
        constraintSolverData->partitions = partitions;

        constraintSolverData->partitionPairMinSetElements = FmAllocFromBuffer<uint>(&pBuffer, maxPartitionPairs, pBufferEnd);
        constraintSolverData->partitionPairMaxSetElements = FmAllocFromBuffer<uint>(&pBuffer, maxPartitionPairs, pBufferEnd);
        constraintSolverData->partitionPairIndependentSets = FmAllocFromBuffer<FmGraphColoringSet>(&pBuffer, maxPartitionPairs, pBufferEnd);
        constraintSolverData->maxStateVecs3 = numStateVecs3;
        constraintSolverData->maxConstraints = numConstraints;
        constraintSolverData->maxJacobianSubmats = numJacobianSubmats;

        constraintSolverData->isAllocated = true;

        pBuffer = (uint8_t*)FM_PAD_64((uintptr_t)pBuffer);

        FM_ASSERT(((uintptr_t)pBuffer & 0x3f) == 0);
    }

    FmConstraintSolverBuffer* FmSetupConstraintSolverBuffer(const FmConstraintSolverBufferSetupParams& params, uint8_t*& pBuffer, size_t bufferNumBytes)
    {
        FM_ASSERT(((uintptr_t)pBuffer & 0x3f) == 0);

        uint maxTetMeshes = params.maxTetMeshes;
        uint maxRigidBodies = params.maxRigidBodies;
        uint maxConstraintIslands = maxTetMeshes + maxRigidBodies;
        size_t maxConstraintSolverDataSize = params.maxConstraintSolverDataSize;

        //uint8_t* pBufferStart = pBuffer;
        uint8_t* pBufferEnd = pBuffer + bufferNumBytes;

        FmConstraintSolverBuffer* pConstraintSolverBuffer = FmAllocFromBuffer64<FmConstraintSolverBuffer>(&pBuffer, 1, pBufferEnd);
        FmInitConstraintSolverBuffer(pConstraintSolverBuffer);

        pConstraintSolverBuffer->bufferNumBytes = bufferNumBytes;

        // Allocate memory for all island solver data
        size_t islandSolverDataArraysMaxBytes = maxConstraintSolverDataSize;
        uint8_t* islandSolverDataArraysStart = FmAllocFromBuffer64<uint8_t>(&pBuffer, islandSolverDataArraysMaxBytes, pBufferEnd);
        pConstraintSolverBuffer->islandSolverData = FmAllocFromBuffer64<FmConstraintSolverData>(&pBuffer, maxConstraintIslands, pBufferEnd);

        pConstraintSolverBuffer->rigidBodySolverOffsets = FmAllocFromBuffer<uint>(&pBuffer, maxRigidBodies, pBufferEnd);
        pConstraintSolverBuffer->tetMeshIdxFromId = FmAllocFromBuffer<uint>(&pBuffer, maxTetMeshes, pBufferEnd);
        pConstraintSolverBuffer->rigidBodyIdxFromId = FmAllocFromBuffer<uint>(&pBuffer, maxRigidBodies, pBufferEnd);

        pConstraintSolverBuffer->islandSolverDataArraysMaxBytes = islandSolverDataArraysMaxBytes;
        pConstraintSolverBuffer->islandSolverDataArraysNumBytes = 0;
        pConstraintSolverBuffer->islandSolverDataArraysHighWaterMark = 0;

        pConstraintSolverBuffer->islandSolverDataArraysStart = islandSolverDataArraysStart;

        for (uint i = 0; i < maxConstraintIslands; i++)
        {
            FmConstraintSolverData& islandSolverData = pConstraintSolverBuffer->islandSolverData[i];
            FmInitConstraintSolverData(&islandSolverData, FmConstraintSolverControlParams(), FmConstraintSolverControlParams()); // control params set later
        }

        pBuffer = (uint8_t*)FM_PAD_64((uintptr_t)pBuffer);

        FM_ASSERT(((uintptr_t)pBuffer & 0x3f) == 0);
        return pConstraintSolverBuffer;
    }

    // result = diag * -OffDiag(A) * v
    static inline void FmDiagxNegOffDiagMxV(FmSVector3* result, const FmMpcgSolverData& meshData, const FmSVector3* v)
    {
        uint numRows3 = meshData.A.numRows;
        uint vertOffset = meshData.solverStateOffset;
        const FmSparseMatrixSubmat3& A = meshData.A;
        const FmSMatrix3* diag = meshData.PInvDiag;
        const bool* kinematicFlags = meshData.kinematicFlags;

        for (uint row3 = 0; row3 < numRows3; row3++)
        {
            bool kinematic = kinematicFlags[row3];

            if (kinematic)
            {
                result[vertOffset + row3] = FmSVector3(0.0f);
            }
            else
            {
                uint rowStart = A.rowStarts[row3];
                uint rowEnd = A.rowStarts[row3 + 1];

                FmSVector3 rowResult = FmSVector3(0.0f);

                FmSMatrix3 diag_row = diag[row3];

                // multiply non-diagonal entries

                FM_ASSERT(rowEnd >= rowStart);

                for (uint i = rowStart; i < rowEnd; i++)
                {
                    FmSMatrix3 submat = A.submats[i];
                    uint idx = A.indices[i];
                    FmSVector3 v_idx = v[vertOffset + idx];

                    rowResult -= submat * v_idx;
                }

                result[vertOffset + row3] = diag_row * rowResult;
            }
        }
    }

#if !FM_ASYNC_THREADING
    // result = diag * -OffDiag(M) * v
    static inline void FmDiagxNegOffDiagMxV(FmSVector3* result, FmScene* scene, const FmConstraintIsland& constraintIsland, const FmSVector3* v)
    {
        uint numIslandMeshes = constraintIsland.numTetMeshes;
        for (uint islandMeshIdx = 0; islandMeshIdx < numIslandMeshes; islandMeshIdx++)
        {
            const FmMpcgSolverData& meshData = *FmGetSolverDataById(*scene, constraintIsland.tetMeshIds[islandMeshIdx]);

            uint numRows3 = meshData.A.numRows;
            uint vertOffset = meshData.solverStateOffset;
            const FmSparseMatrixSubmat3& A = meshData.A;
            const FmSMatrix3* diag = meshData.PInvDiag;
            const bool* kinematicFlags = meshData.kinematicFlags;

            for (uint row3 = 0; row3 < numRows3; row3++)
            {
                bool kinematic = kinematicFlags[row3];

                if (kinematic)
                {
                    result[vertOffset + row3] = FmSVector3(0.0f);
                }
                else
                {
                    uint rowStart = A.rowStarts[row3];
                    uint rowEnd = A.rowStarts[row3 + 1];

                    FmSVector3 rowResult = FmSVector3(0.0f);

                    FmSMatrix3 diag_row = diag[row3];

                    // multiply non-diagonal entries

                    FM_ASSERT(rowEnd >= rowStart);

                    for (uint i = rowStart; i < rowEnd; i++)
                    {
                        FmSMatrix3 submat = A.submats[i];
                        uint idx = A.indices[i];
                        FmSVector3 v_idx = v[vertOffset + idx];

                        rowResult -= submat * v_idx;
                    }

                    result[vertOffset + row3] = diag_row * rowResult;
                }
            }
        }
    }
#endif

    // Solve one constraint (block) row in Projected Gauss-Seidel iteration.
    // Update error norm used in convergence test: inf norm of solution change.
    static FM_FORCE_INLINE void FmPgsIteration3Row(
        FmSolverIterationNorms* FM_RESTRICT norms,
        FmSVector3* FM_RESTRICT lambda3,
        FmSVector3* FM_RESTRICT JTlambda,
        const FmConstraintJacobian& FM_RESTRICT J,
        const FmSVector3* FM_RESTRICT pgsRhs,
        const FmSMatrix3* FM_RESTRICT DAinv,
        float omega,
        uint passIdx,
        uint rowIdx)
    {
        FmConstraintParams& constraintParams = J.params[rowIdx];
        float frictionCoeff = constraintParams.frictionCoeff;
        FmSolverConstraintType type = (FmSolverConstraintType)constraintParams.type;
        uint8_t flags = constraintParams.flags;

        FmSVector3 diagInv = constraintParams.diagInverse[passIdx];

        FmSVector3 rowResult = pgsRhs[rowIdx];
        FmSVector3 lambda3Orig = lambda3[rowIdx];

        FmSMatrix3* jacobianSubmats = (FmSMatrix3*)((uint8_t*)J.submats + constraintParams.jacobianSubmatsOffset);
        uint* jacobianIndices = (uint*)((uint8_t*)J.indices + constraintParams.jacobianIndicesOffset);
        uint rowSize = constraintParams.jacobiansNumStates;

        for (uint i = 0; i < rowSize; i++)
        {
            FmSMatrix3 submat = jacobianSubmats[i];
            uint idx = jacobianIndices[i];
            FmSMatrix3 DAinv_idx = DAinv[idx];
            FmSVector3 JTlambda_idx = JTlambda[idx];

            rowResult -= submat * (DAinv_idx * JTlambda_idx);
        }

        // rowResult is now c - B_lambda3 * lambda
        // Finish GS iteration for row
        rowResult = diagInv * rowResult + lambda3Orig;

        // Under-relaxation for stability benefit
        rowResult = (1.0f - omega) * lambda3Orig + omega * rowResult;

        float lambda = rowResult.x;
        float gamma1 = rowResult.y;
        float gamma2 = rowResult.z;

        if (type == FM_SOLVER_CONSTRAINT_TYPE_3D_NORMAL2DFRICTION)
        {
            // Normal and friction force projections.
            lambda = FmMaxFloat(lambda, 0.0f);

            float maxFriction = frictionCoeff * lambda;

#if FM_PROJECT_FRICTION_TO_CIRCLE
            // Projection to circle
            float length = sqrtf(gamma1*gamma1 + gamma2*gamma2);
            if (length > maxFriction)
            {
                float scale = maxFriction / length;
                gamma1 *= scale;
                gamma2 *= scale;
            }
#else
            // Projection to box
            gamma1 = FmMinFloat(gamma1, maxFriction);
            gamma1 = FmMaxFloat(gamma1, -maxFriction);
            gamma2 = FmMinFloat(gamma2, maxFriction);
            gamma2 = FmMaxFloat(gamma2, -maxFriction);
#endif
        }
        else if (type == FM_SOLVER_CONSTRAINT_TYPE_3D_JOINT1DFRICTION)
        {
            float maxFriction = frictionCoeff;
            lambda = FmMinFloat(lambda,  maxFriction);
            lambda = FmMaxFloat(lambda, -maxFriction);
        }
        else
        {
            if ((flags & FM_SOLVER_CONSTRAINT_FLAG_NONNEG0) != 0)
            {
                lambda = FmMaxFloat(lambda, 0.0f);
            }
            if ((flags & FM_SOLVER_CONSTRAINT_FLAG_NONNEG1) != 0)
            {
                gamma1 = FmMaxFloat(gamma1, 0.0f);
            }
            if ((flags & FM_SOLVER_CONSTRAINT_FLAG_NONNEG2) != 0)
            {
                gamma2 = FmMaxFloat(gamma2, 0.0f);
            }
        }

        FmSVector3 lambda3Update = FmSVector3(lambda, gamma1, gamma2);

        // Update J^T * lambda given change in lambda values
        FmSVector3 deltaLambda = lambda3Update - lambda3Orig;

#if FM_CONSTRAINT_SOLVER_CONVERGENCE_TEST
        norms->Update(deltaLambda, lambda3Update);
#else
        (void)norms;
#endif

        const float epsilon = FLT_EPSILON*FLT_EPSILON;
        if (lengthSqr(deltaLambda) > epsilon)
        {
            for (uint i = 0; i < rowSize; i++)
            {
                FmSMatrix3 rowSubmat = jacobianSubmats[i];
                uint outputRowIdx = jacobianIndices[i];
#if 1
                JTlambda[outputRowIdx] += FmTransposeMul(rowSubmat, deltaLambda);
#else
                FmSMatrix3 JTSubmat = transpose(rowSubmat);

                JTlambda[outputRowIdx] += JTSubmat * deltaLambda;
#endif
            }

            // update lambda
            lambda3[rowIdx] = lambda3Update;
        }
    }

    // Execute one iteration of Projected Gauss-Seidel.
    // Return error norm used in convergence test: inf norm of solution change.
    static inline void FmPgsIteration3(FmSolverIterationNorms* resultNorms, FmConstraintSolverData* constraintSolverData, uint* indices, uint numRows, uint passIdx)
    {
        FmSVector3* lambda3 = constraintSolverData->lambda3;
        FmSVector3* JTlambda = constraintSolverData->JTlambda;
        const FmConstraintJacobian& J = constraintSolverData->J;
        const FmSVector3* pgsRhs = constraintSolverData->pgsRhs;
        const FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;
        const FmSMatrix3* DAinv = constraintSolverData->GetBlockDiagInv(passIdx);
        float omega = controlParams.passParams[passIdx].kPgsRelaxationOmega;

        FmSolverIterationNorms norms;
        norms.Zero();

        for (uint indexId = 0; indexId < numRows; indexId++)
        {
            uint rowIdx = indices[indexId];

            FmPgsIteration3Row(&norms, lambda3, JTlambda, J, pgsRhs, DAinv, omega, passIdx, rowIdx);
        }

        *resultNorms = norms;
    }

    static inline void FmPgsIteration3Reverse(FmSolverIterationNorms* resultNorms, FmConstraintSolverData* constraintSolverData, uint* indices, uint numRows, uint passIdx)
    {
        FmSVector3* lambda3 = constraintSolverData->lambda3;
        FmSVector3* JTlambda = constraintSolverData->JTlambda;
        const FmConstraintJacobian& J = constraintSolverData->J;
        const FmSVector3* pgsRhs = constraintSolverData->pgsRhs;
        const FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;
        const FmSMatrix3* DAinv = constraintSolverData->GetBlockDiagInv(passIdx);
        float omega = controlParams.passParams[passIdx].kPgsRelaxationOmega;

        FmSolverIterationNorms norms;
        norms.Zero();

        for (int indexId = (int)numRows - 1; indexId >= 0; indexId--)
        {
            uint rowIdx = indices[indexId];

            FmPgsIteration3Row(&norms, lambda3, JTlambda, J, pgsRhs, DAinv, omega, passIdx, rowIdx);
        }

        *resultNorms = norms;
    }

    void FmAllocConstraintIslandSolverData(FmScene* scene)
    {
        FmConstraintSolverBuffer* constraintSolverBuffer = scene->constraintSolverBuffer;
        FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;
        uint numConstraintIslands = constraintsBuffer->numConstraintIslands;

        // Allocate constraint solver data arrays
        uint8_t *solverArraysStart = constraintSolverBuffer->islandSolverDataArraysStart;
        uint8_t *solverArraysEnd = solverArraysStart + constraintSolverBuffer->islandSolverDataArraysMaxBytes;

        size_t islandSolverDataArraysNumBytes = 0;

        for (uint islandIdx = 0; islandIdx < numConstraintIslands; islandIdx++)
        {
            FmConstraintIsland& island = constraintsBuffer->constraintIslands[islandIdx];

            island.randomState.Init(constraintsBuffer->islandRandomSeed++);
            island.innerIterationCallback = constraintsBuffer->innerIterationCallback;
            island.islandCompletedCallback = constraintsBuffer->islandCompletedCallback;
            island.userData = constraintsBuffer->userData;

            FmConstraintSolverData* constraintSolverData = &constraintSolverBuffer->islandSolverData[islandIdx];

            FmInitConstraintSolverData(constraintSolverData, scene->params.constraintSolveParams, scene->params.constraintStabilizationParams);

            size_t constraintIslandSolverDataSize =
                FmGetConstraintIslandSolverDataSize(island.numStateVecs3, island.numTetMeshes + island.numRigidBodiesInFEMSolve, island.numConstraints, island.numJacobianSubmats, 1);

            if (solverArraysStart + constraintIslandSolverDataSize <= solverArraysEnd)
            {
                // Sub-allocate from scene memory
#if defined(_DEBUG) || FM_DEBUG_CHECKS
                uint8_t* pDataStart = solverArraysStart;
#endif
                islandSolverDataArraysNumBytes += constraintIslandSolverDataSize;

                FmAllocConstraintIslandSolverDataFromBuffer(constraintSolverData, island, solverArraysStart, solverArraysEnd);

                FM_ASSERT(pDataStart + constraintIslandSolverDataSize == solverArraysStart);
            }
            else
            {
                // Dynamically allocate memory
                uint8_t* pIslandSolverDataArrays = (uint8_t*)FmAlignedMalloc(constraintIslandSolverDataSize, 64);

                if (pIslandSolverDataArrays)
                {
                    islandSolverDataArraysNumBytes += constraintIslandSolverDataSize;

                    constraintSolverData->pDynamicAllocatedBuffer = pIslandSolverDataArrays;
                    FmAllocConstraintIslandSolverDataFromBuffer(constraintSolverData, island, pIslandSolverDataArrays, pIslandSolverDataArrays + constraintIslandSolverDataSize);
                }

                FmAtomicOr(&scene->warningsReport.flags.val, FM_WARNING_FLAG_HIT_LIMIT_SCENE_CONSTRAINT_SOLVER_MEMORY);
            }
        }

        if (islandSolverDataArraysNumBytes > constraintSolverBuffer->islandSolverDataArraysHighWaterMark)
        {
            constraintSolverBuffer->islandSolverDataArraysHighWaterMark = islandSolverDataArraysNumBytes;
        }
        constraintSolverBuffer->islandSolverDataArraysNumBytes = islandSolverDataArraysNumBytes;
    }

    // Read changes to rigid body made in external solving.
    // Reset solverStateOffset.
    void FmReadRigidBodyDeltaVel(
        FmConstraintSolverData* constraintSolverData,
        const FmConstraintIsland& constraintIsland,
        const FmScene& scene)
    {
        uint numIslandTetMeshVerts = constraintIsland.numTetMeshVerts;
        uint numIslandRigidBodies = constraintIsland.numRigidBodiesInFEMSolve;

        for (uint islandRbIdx = 0; islandRbIdx < numIslandRigidBodies; islandRbIdx++)
        {
            const FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(scene, constraintIsland.rigidBodyIds[islandRbIdx]);

            uint stateIdx = numIslandTetMeshVerts + islandRbIdx * 2;

            constraintSolverData->deltaVel[stateIdx] = FmSVector3(rigidBody.deltaVel);
            constraintSolverData->deltaVel[stateIdx + 1] = FmSVector3(rigidBody.deltaAngVel);

            FmSVector3 JTlambda0 = FmSVector3(rigidBody.deltaVel * rigidBody.mass);
            FmSVector3 JTlambda1 = FmSVector3(rigidBody.worldInertiaTensor * rigidBody.deltaAngVel);

            constraintSolverData->JTlambda[stateIdx] = JTlambda0;
            constraintSolverData->JTlambda[stateIdx + 1] = JTlambda1;
        }
    }

    // Read changes to rigid body made in external solving.
    // Reset solverStateOffset.
    void FmReadRigidBodyDeltaPos(
        FmConstraintSolverData* constraintSolverData,
        const FmConstraintIsland& constraintIsland,
        const FmScene& scene)
    {
        uint numIslandTetMeshVerts = constraintIsland.numTetMeshVerts;
        uint numIslandRigidBodies = constraintIsland.numRigidBodiesInFEMSolve;

        for (uint islandRbIdx = 0; islandRbIdx < numIslandRigidBodies; islandRbIdx++)
        {
            const FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(scene, constraintIsland.rigidBodyIds[islandRbIdx]);

            uint stateIdx = numIslandTetMeshVerts + islandRbIdx * 2;

#if FM_CONSTRAINT_STABILIZATION_SOLVE
            constraintSolverData->deltaPos[stateIdx] = FmSVector3(rigidBody.deltaPos);
            constraintSolverData->deltaPos[stateIdx + 1] = FmSVector3(rigidBody.deltaAngPos);
#endif

            FmSVector3 JTlambda0 = FmSVector3(rigidBody.deltaPos * rigidBody.mass);
            FmSVector3 JTlambda1 = FmSVector3(rigidBody.worldInertiaTensor * rigidBody.deltaAngPos);

            constraintSolverData->JTlambda[stateIdx] = JTlambda0;
            constraintSolverData->JTlambda[stateIdx + 1] = JTlambda1;
        }
    }

    // Compute rigid body response to JTlambda and set in rigid bodies for use in external solver.
    void FmWriteRigidBodyDeltaVel(
        FmScene* scene,
        FmConstraintSolverData* constraintSolverData,
        const FmConstraintIsland& constraintIsland)
    {
        uint numIslandRigidBodies = constraintIsland.numRigidBodiesInFEMSolve;

        for (uint islandRbIdx = 0; islandRbIdx < numIslandRigidBodies; islandRbIdx++)
        {
            uint rbId = constraintIsland.rigidBodyIds[islandRbIdx];

            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, rbId);

            uint stateIdx = FmGetRigidBodySolverOffsetById(*scene->constraintSolverBuffer, rbId);

            rigidBody.deltaVel = FmVector3(constraintSolverData->deltaVel[stateIdx]);
            rigidBody.deltaAngVel = FmVector3(constraintSolverData->deltaVel[stateIdx + 1]);
        }
    }

#if FM_CONSTRAINT_STABILIZATION_SOLVE
    // Compute rigid body response to JTlambda and set in rigid bodies for use in external solver.
    void FmWriteRigidBodyDeltaPos(
        FmScene* scene,
        FmConstraintSolverData* constraintSolverData,
        const FmConstraintIsland& constraintIsland)
    {
        uint numIslandRigidBodies = constraintIsland.numRigidBodiesInFEMSolve;

        for (uint islandRbIdx = 0; islandRbIdx < numIslandRigidBodies; islandRbIdx++)
        {
            uint rbId = constraintIsland.rigidBodyIds[islandRbIdx];

            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, rbId);

            uint stateIdx = FmGetRigidBodySolverOffsetById(*scene->constraintSolverBuffer, rbId);

            rigidBody.deltaPos = FmVector3(constraintSolverData->deltaPos[stateIdx]);
            rigidBody.deltaAngPos = FmVector3(constraintSolverData->deltaPos[stateIdx + 1]);
        }
    }
#endif

    void FmExternalPgsIteration(FmTaskGraphSolveData& data)
    {
        FmScene* scene = data.scene;
        FmConstraintSolverData* constraintSolverData = data.constraintSolverData;
        const FmConstraintIsland& constraintIsland = *data.constraintIsland;

        bool inStabilization = constraintSolverData->IsInStabilization();

        if (constraintIsland.innerIterationCallback)
        {
            FmSolverIterationNorms externalNorms;
            externalNorms.Zero();

#if FM_CONSTRAINT_STABILIZATION_SOLVE
            if (inStabilization)
            {
                FmWriteRigidBodyDeltaPos(scene, constraintSolverData, constraintIsland);
            }
            else
#endif
            {
                FmWriteRigidBodyDeltaVel(scene, constraintSolverData, constraintIsland);
            }

            constraintIsland.innerIterationCallback(
                scene,
                &externalNorms,
                constraintIsland.userData,
                constraintIsland.rigidBodyIds, constraintIsland.numRigidBodiesInFEMSolve,
                constraintIsland.userRigidBodyIslandIndices, constraintIsland.numUserRigidBodyIslands, inStabilization);

#if FM_CONSTRAINT_STABILIZATION_SOLVE
            if (inStabilization)
            {
                FmReadRigidBodyDeltaPos(constraintSolverData, constraintIsland, *scene);
            }
            else
#endif
            {
                FmReadRigidBodyDeltaVel(constraintSolverData, constraintIsland, *scene);
            }

#if FM_CONSTRAINT_SOLVER_CONVERGENCE_TEST
            constraintSolverData->externaPgsNorms.Update(externalNorms);
#endif
        }
    }

    void FmUpdatePartitionPairPgsRhs(FmConstraintSolverData* constraintSolverData, const FmPartitionPair& partitionPair, const FmSVector3* passRhsInput)
    {
        uint numConstraints = partitionPair.numConstraints;
        for (uint partitionPairConstraintIdx = 0; partitionPairConstraintIdx < numConstraints; partitionPairConstraintIdx++)
        {
            uint constraintIdx = partitionPair.constraintIndices[partitionPairConstraintIdx];

            // Compute PGS right-hand-side
            FmVmMxV_row(constraintSolverData->pgsRhs, constraintSolverData->pgsRhsConstant, constraintSolverData->J, passRhsInput, constraintIdx);
        }
    }

#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
    class FmTaskDataPartitionGsIterationOrRbResponse : public TLTaskDataBase
    {
    public:
        FmScene* scene;
        FmConstraintSolverData* constraintSolverData;
        FmConstraintIsland* constraintIsland;
        uint partitionId;

        uint numTetMeshes;
        uint numRigidBodies;
        uint numRigidBodyBatches;
        
        bool isLastOuterIteration;
        bool isForwardDirection;

        FmTaskDataPartitionGsIterationOrRbResponse(
            FmScene* inScene,
            FmConstraintSolverData* inConstraintSolverData,
            FmConstraintIsland* inConstraintIsland,
            uint inPartitionId,
            uint inNumTetMeshes,
            uint inNumRigidBodies,
            uint inNumRigidBodyBatches,
            bool inIsLastOuterIteration,
            bool inIsForwardDirection)
        {
            scene = inScene;
            constraintSolverData = inConstraintSolverData;
            constraintIsland = inConstraintIsland;
            partitionId = inPartitionId;
            numTetMeshes = inNumTetMeshes;
            numRigidBodies = inNumRigidBodies;
            numRigidBodyBatches = inNumRigidBodyBatches;
            isLastOuterIteration = inIsLastOuterIteration;
            isForwardDirection = inIsForwardDirection;
        }
    };

    void FmTaskFuncPartitionGsIteration(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT("GsIterationWork");

        FmTaskDataPartitionGsIterationOrRbResponse* taskData = (FmTaskDataPartitionGsIterationOrRbResponse*)inTaskData;
        FmScene* scene = taskData->scene;

        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        uint partitionId = taskData->partitionId;

        bool isLastIteration = taskData->isLastOuterIteration;
        bool forwardDirection = taskData->isForwardDirection;
        
        FmPartition& partition = constraintSolverData->partitions[partitionId];

        uint beginIdx = (uint)inTaskBeginIndex;
        uint endIdx = (uint)inTaskEndIndex;

        FmSVector3* deltaVec = constraintSolverData->GetDeltaVec();
        FmSVector3* pgsDeltaVelTerm = constraintSolverData->pgsDeltaVelTerm;

        for (uint partitionObjectIdx = beginIdx; partitionObjectIdx < endIdx; partitionObjectIdx++)
        {
            uint islandObjectId = partition.objectIds[partitionObjectIdx];

            if (FM_IS_SET(islandObjectId, FM_RB_FLAG))
            {
                continue;
            }

            FmMpcgSolverData& meshData = *FmGetSolverDataById(*scene, islandObjectId);

            if (forwardDirection)
            {
                FmGsIteration(deltaVec, meshData, constraintSolverData->JTlambda);
            }
            else
            {
                FmGsIterationReverse(deltaVec, meshData, constraintSolverData->JTlambda);
            }

            if (!isLastIteration)
            {
                // Update pgsDeltaVelTerm = DAinv * (UA + LA) * deltaVel for next iteration
                FmDiagxNegOffDiagMxV(pgsDeltaVelTerm, meshData, deltaVec);
            }
        }
    }

    // For rigid bodies, compute delta pos or delta vel from JTlambda.
    // Only needed on final iteration of constraint solve
    void FmTaskFuncPartitionGsPassRbResponse(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FmTaskDataPartitionGsIterationOrRbResponse* taskData = (FmTaskDataPartitionGsIterationOrRbResponse*)inTaskData;

        if (taskData->numRigidBodies == 0)
            return;

        uint taskIdx = (uint)inTaskBeginIndex;

        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        FmConstraintSolverBuffer& constraintSolverBuffer = *taskData->scene->constraintSolverBuffer;
        uint partitionId = taskData->partitionId;

        uint numTetMeshes = taskData->numTetMeshes;

        FmPartition& partition = constraintSolverData->partitions[partitionId];

        FmSVector3* deltaVec = constraintSolverData->GetDeltaVec(); // delta_pos or delta_vel depending on stabilization or solve pass

        uint beginIdx, endIdx;
        FmGetIndexRangeEvenDistribution(&beginIdx, &endIdx, taskIdx, taskData->numRigidBodyBatches, taskData->numRigidBodies);

        FM_ASSERT(taskData->isLastOuterIteration);

        for (uint rigidBodyIdx = beginIdx; rigidBodyIdx < endIdx; rigidBodyIdx++)
        {
            uint objectId = partition.objectIds[numTetMeshes + rigidBodyIdx];  // rigid bodies after tet meshes in objectIds

            FM_ASSERT(FM_IS_SET(objectId, FM_RB_FLAG));

            uint stateOffset = FmGetRigidBodySolverOffsetById(constraintSolverBuffer, objectId);

            deltaVec[stateOffset] = constraintSolverData->DAinv[stateOffset] * constraintSolverData->JTlambda[stateOffset];
            deltaVec[stateOffset + 1] = constraintSolverData->DAinv[stateOffset + 1] * constraintSolverData->JTlambda[stateOffset + 1];
        }
    }

    int32_t FmGrainFuncPartitionGsIterationOrRbResponse(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        FmTaskDataPartitionGsIterationOrRbResponse* taskData = (FmTaskDataPartitionGsIterationOrRbResponse*)inTaskData;

        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        uint partitionId = taskData->partitionId;
        uint numTetMeshes = taskData->numTetMeshes;

        const uint minVerts = 128;

        uint numVerts = 0;
        uint objectIndex = (uint)inTaskBeginIndex;
        uint taskBeginIndex = (uint)inTaskBeginIndex;
        uint taskEndIndex = (uint)inTaskEndIndex;

        if (objectIndex < numTetMeshes)
        {
            FmPartition& partition = constraintSolverData->partitions[partitionId];

            while (numVerts < minVerts
                && objectIndex < taskEndIndex
                && objectIndex < numTetMeshes)
            {
                numVerts += partition.objectNumVertices[objectIndex];
                objectIndex++;
            }

            return objectIndex - taskBeginIndex;
        }
        else
        {
            return 1;
        }
    }

    void FmTaskFuncPartitionGsIterationOrRbResponse(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FmTaskDataPartitionGsIterationOrRbResponse* taskData = (FmTaskDataPartitionGsIterationOrRbResponse*)inTaskData;

        uint numTetMeshes = taskData->numTetMeshes;

        uint partitionObjectBeginIndex = (uint)inTaskBeginIndex;
        uint partitionObjectEndIndex = (uint)inTaskEndIndex;

        if (partitionObjectBeginIndex < taskData->numTetMeshes)
        {
            FmTaskFuncPartitionGsIteration(taskData, partitionObjectBeginIndex, partitionObjectEndIndex);
        }
        else
        {
            int32_t taskIndex = (int32_t)(partitionObjectBeginIndex - numTetMeshes);
            FmTaskFuncPartitionGsPassRbResponse(taskData, taskIndex, taskIndex + 1);
        }
    }

    class FmTaskDataPartitionRunMpcgOrRbResponse : public TLTaskDataBase
    {
    public:
        FmScene* scene;
        FmConstraintSolverData* constraintSolverData;
        FmConstraintIsland* constraintIsland;
        uint partitionId;

        uint numTetMeshes;
        uint numRigidBodies;
        uint numRigidBodyBatches;

        bool isFirstOuterIteration;
        bool isLastOuterIteration;

        FmTaskDataPartitionRunMpcgOrRbResponse(FmScene* inScene, FmConstraintSolverData* inConstraintSolverData, FmConstraintIsland* inConstraintIsland, uint inPartitionId, 
            uint inNumTetMeshes, uint inNumRigidBodies, uint inNumRigidBodyBatches,
            uint inIsFirstOuterIteration, bool inIsLastOuterIteration)
        {
            scene = inScene;
            constraintSolverData = inConstraintSolverData;
            constraintIsland = inConstraintIsland;
            partitionId = inPartitionId;
            numTetMeshes = inNumTetMeshes;
            numRigidBodies = inNumRigidBodies;
            numRigidBodyBatches = inNumRigidBodyBatches;
            isFirstOuterIteration = inIsFirstOuterIteration;
            isLastOuterIteration = inIsLastOuterIteration;
        }
    };

    int32_t FmGrainFuncPartitionRunMpcgOrRbResponse(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        FmTaskDataPartitionRunMpcgOrRbResponse* taskData = (FmTaskDataPartitionRunMpcgOrRbResponse*)inTaskData;

        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        uint partitionId = taskData->partitionId;
        uint numTetMeshes = taskData->numTetMeshes;

        const uint minVerts = 64;

        uint numVerts = 0;
        uint objectIndex = (uint)inTaskBeginIndex;
        uint taskBeginIndex = (uint)inTaskBeginIndex;
        uint taskEndIndex = (uint)inTaskEndIndex;

        if (objectIndex < numTetMeshes)
        {
            FmPartition& partition = constraintSolverData->partitions[partitionId];

            while (numVerts < minVerts
                && objectIndex < taskEndIndex
                && objectIndex < numTetMeshes)
            {
                numVerts += partition.objectNumVertices[objectIndex];
                objectIndex++;
            }

            return objectIndex - taskBeginIndex;
        }
        else
        {
            return 1;
        }
    }

    void FmTaskFuncPartitionRunMpcg(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        FM_TRACE_SCOPED_EVENT("MpcgWork");

        FmTaskDataPartitionRunMpcgOrRbResponse* taskData = (FmTaskDataPartitionRunMpcgOrRbResponse*)inTaskData;
        FmScene* scene = taskData->scene;
        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        uint partitionId = taskData->partitionId;
        bool isLastIteration = taskData->isLastOuterIteration;

        const FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;

        FmPartition& partition = constraintSolverData->partitions[partitionId];
        FmSVector3* islandDeltaVec = constraintSolverData->GetDeltaVec();

        uint beginIdx = (uint)inTaskBeginIndex;
        uint endIdx = (uint)inTaskEndIndex;

        for (uint partitionObjectIdx = beginIdx; partitionObjectIdx < endIdx; partitionObjectIdx++)
        {
            uint islandObjectId = partition.objectIds[partitionObjectIdx];

            if (FM_IS_SET(islandObjectId, FM_RB_FLAG))
            {
                continue;
            }

            FmMpcgSolverData& meshData = *FmGetSolverDataById(*scene, islandObjectId);

            FmVpV(meshData.b, meshData.b, &constraintSolverData->JTlambda[meshData.solverStateOffset], meshData.A.numRows);

            FmSVector3* deltavec = &islandDeltaVec[meshData.solverStateOffset];

            uint workerIndex = TLGetTaskSystemThreadIndex();
            uint8_t* tempBuffer = scene->threadTempMemoryBuffer->buffers[workerIndex];
            uint8_t* tempBufferEnd = tempBuffer + scene->threadTempMemoryBuffer->numBytesPerBuffer;

            FmMpcgSolverDataTemps temps;
            temps.Alloc(&tempBuffer, tempBufferEnd, meshData.A.numRows);

            float epsilon = controlParams.epsilonCgPass;
            uint maxIterations = controlParams.maxCgIterationsCgPass;
            FmRunMpcgSolve(deltavec, &meshData, &temps, epsilon, maxIterations);

            if (isLastIteration)
            {
                FmVcV(&constraintSolverData->JTlambda[meshData.solverStateOffset], meshData.b, meshData.A.numRows);

                // If finished the CG pass, update pgsDeltaVelTerm = DAinv * (UA + LA) * deltaVel for GS pass
                FmDiagxNegOffDiagMxV(constraintSolverData->pgsDeltaVelTerm, meshData, islandDeltaVec);
            }
            else
            {
                FmVfill(&constraintSolverData->JTlambda[meshData.solverStateOffset], FmSVector3(0.0f), meshData.A.numRows);
            }
        }
    }

    void FmTaskFuncPartitionCgPassRbResponse(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FmTaskDataPartitionRunMpcgOrRbResponse* taskData = (FmTaskDataPartitionRunMpcgOrRbResponse*)inTaskData;

        if (taskData->numRigidBodies == 0)
            return;

        uint taskIdx = (uint)inTaskBeginIndex;

        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        FmConstraintSolverBuffer& constraintSolverBuffer = *taskData->scene->constraintSolverBuffer;
        uint partitionId = taskData->partitionId;
        bool isFirstOuterIteration = taskData->isFirstOuterIteration;
        bool isLastOuterIteration = taskData->isLastOuterIteration;

        uint numTetMeshes = taskData->numTetMeshes;

        FmPartition& partition = constraintSolverData->partitions[partitionId];

        FmSVector3* deltaVec = constraintSolverData->GetDeltaVec(); // delta_pos or delta_vel depending on stabilization or solve pass

        uint beginIdx, endIdx;
        FmGetIndexRangeEvenDistribution(&beginIdx, &endIdx, taskIdx, taskData->numRigidBodyBatches, taskData->numRigidBodies);

        for (uint rigidBodyIdx = beginIdx; rigidBodyIdx < endIdx; rigidBodyIdx++)
        {
            uint objectId = partition.objectIds[numTetMeshes + rigidBodyIdx];  // rigid bodies after tet meshes in objectIds

            FM_ASSERT(FM_IS_SET(objectId, FM_RB_FLAG));

            uint stateOffset = FmGetRigidBodySolverOffsetById(constraintSolverBuffer, objectId);

            FmSVector3 totalJTlambda0;
            FmSVector3 totalJTlambda1;

            if (isFirstOuterIteration)
            {
                totalJTlambda0 = FmSVector3(0.0f);
                totalJTlambda1 = FmSVector3(0.0f);
            }
            else
            {
                totalJTlambda0 = constraintSolverData->velTemp[stateOffset];
                totalJTlambda1 = constraintSolverData->velTemp[stateOffset + 1];
            }

            totalJTlambda0 += constraintSolverData->JTlambda[stateOffset];
            totalJTlambda1 += constraintSolverData->JTlambda[stateOffset + 1];

            deltaVec[stateOffset] = constraintSolverData->DAinv[stateOffset] * totalJTlambda0;
            deltaVec[stateOffset + 1] = constraintSolverData->DAinv[stateOffset + 1] * totalJTlambda1;

            if (isLastOuterIteration)
            {
                constraintSolverData->JTlambda[stateOffset] = totalJTlambda0;
                constraintSolverData->JTlambda[stateOffset + 1] = totalJTlambda1;
            }
            else
            {
                constraintSolverData->velTemp[stateOffset] = totalJTlambda0;
                constraintSolverData->velTemp[stateOffset + 1] = totalJTlambda1;

                constraintSolverData->JTlambda[stateOffset] = FmSVector3(0.0f);
                constraintSolverData->JTlambda[stateOffset + 1] = FmSVector3(0.0f);
            }
        }
    }

    void FmTaskFuncPartitionRunMpcgOrRbResponse(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FmTaskDataPartitionRunMpcgOrRbResponse* taskData = (FmTaskDataPartitionRunMpcgOrRbResponse*)inTaskData;

        uint numTetMeshes = taskData->numTetMeshes;

        uint partitionObjectBeginIndex = (uint)inTaskBeginIndex;
        uint partitionObjectEndIndex = (uint)inTaskEndIndex;

        if (partitionObjectBeginIndex < numTetMeshes)
        {
            FmTaskFuncPartitionRunMpcg(taskData, partitionObjectBeginIndex, partitionObjectEndIndex);
        }
        else
        {
            int32_t taskIndex = (int32_t)(partitionObjectBeginIndex - numTetMeshes);
            FmTaskFuncPartitionCgPassRbResponse(taskData, taskIndex, taskIndex + 1);
        }
    }

    FM_NODE_TASK(FmNodeTaskFuncProcessPartitionPairConstraints)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT("PgsIterationWork");

        TLTaskGraphNode* node = (TLTaskGraphNode*)inTaskData;
        FmConstraintSolveTaskGraph* taskGraph = (FmConstraintSolveTaskGraph*)node->GetGraph();
        FmTaskGraphSolveData* taskData = &taskGraph->solveData;

        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;

        uint pairIdx = (uint)inTaskBeginIndex;

        FmPartitionPair& partitionPair = constraintSolverData->partitionPairs[pairIdx];

        FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;

        FmPartitionPairConstraintNodeState& nodeData = constraintSolverData->taskGraph->partitionPairConstraintNodes[pairIdx];
        nodeData.numTimesReached++;

        // Detect pass
        bool pass0Included = controlParams.passParams[FM_CG_PASS_IDX].maxOuterIterations > 0;
        uint passIdx = !pass0Included ? 1 : nodeData.passIdx;

        const FmConstraintSolverControlParams::PassParams& passParams = controlParams.passParams[passIdx];

        uint maxInnerIterations = passParams.maxInnerIterations;
        if (passParams.useInnerIterationsFalloff)
        {
            uint reduction = FmMinUint(nodeData.outerIteration, maxInnerIterations - 1);
            maxInnerIterations -= reduction;
        }

        uint maxOuterIterations = passParams.maxOuterIterations;

        // Save current node iterations
        uint innerIteration = nodeData.innerIteration;
        uint outerIteration = nodeData.outerIteration;

        // Update inner iteration
        uint nextInnerIteration;
        bool completedInnerIterations;
        if (innerIteration + 1 < maxInnerIterations)
        {
            // Next iteration of PGS
            nextInnerIteration = innerIteration + 1;
            completedInnerIterations = false;
        }
        else
        {
            // Next phase.  Reset iteration for next PGS execution
            nextInnerIteration = 0;
            completedInnerIterations = true;
        }
        nodeData.innerIteration = nextInnerIteration;

        bool forwardDirection = !passParams.useInnerAlternatingDirections || (innerIteration % 2 == 0);

        if ((passIdx == 1 && pass0Included) || (nodeData.outerIteration > 0))
        {
            FmSVector3* pgsRhsInput = constraintSolverData->GetRhsInput(passIdx);

            FmUpdatePartitionPairPgsRhs(constraintSolverData, partitionPair, pgsRhsInput);
        }

        if (forwardDirection)
        {
            FmPgsIteration3(&partitionPair.norms, constraintSolverData, partitionPair.constraintIndices, partitionPair.numConstraints, passIdx);
        }
        else
        {
            FmPgsIteration3Reverse(&partitionPair.norms, constraintSolverData, partitionPair.constraintIndices, partitionPair.numConstraints, passIdx);
        }

        if (completedInnerIterations)
        {
            // Increase or reset outer iteration
            bool isLastOuterIteration = outerIteration + 1 >= maxOuterIterations;

            if (isLastOuterIteration)
            {
                outerIteration = 0;

                // Increase or reset pass index (pass with MPCG or GS)
                nodeData.passIdx = 1 - passIdx;
            }
            else
            {
                outerIteration++;
            }

            nodeData.outerIteration = outerIteration;

            if (passIdx == FM_CG_PASS_IDX)
            {
                // Enable dependent MPCG solves
                FmPartitionMpcgTaskMessages(taskGraph, pairIdx, outerIteration);
            }
            else
            {
                // Enable dependent GS iterations
                FmPartitionGsTaskMessages(taskGraph, pairIdx, outerIteration);
            }
        }
        else
        {
            FmNextIterationMessages(taskGraph, pairIdx, innerIteration);
        }
    }

    FM_NODE_TASK(FmNodeTaskFuncExternalPgsIteration)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT("ExternalPgsIterationWork");

        uint pairIdx = (uint)inTaskBeginIndex;

        TLTaskGraphNode* node = (TLTaskGraphNode*)inTaskData;
        FmConstraintSolveTaskGraph* taskGraph = (FmConstraintSolveTaskGraph*)node->GetGraph();
        FmTaskGraphSolveData* taskData = &taskGraph->solveData;

        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        const FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;

        assert(pairIdx == constraintSolverData->numPartitionPairs);

        FmPartitionPairConstraintNodeState& nodeData = constraintSolverData->taskGraph->partitionPairConstraintNodes[pairIdx];
        nodeData.numTimesReached++;

        // Detect pass
        uint passIdx = controlParams.passParams[FM_CG_PASS_IDX].maxOuterIterations == 0 ? 1 : nodeData.passIdx;

        const FmConstraintSolverControlParams::PassParams& passParams = controlParams.passParams[passIdx];

        uint maxInnerIterations = passParams.maxInnerIterations;
        if (passParams.useInnerIterationsFalloff)
        {
            uint reduction = FmMinUint(nodeData.outerIteration, maxInnerIterations - 1);
            maxInnerIterations -= reduction;
        }

        uint maxOuterIterations = passParams.maxOuterIterations;

        // Update iteration
        uint innerIteration = nodeData.innerIteration;
        uint nextIteration;
        bool completedInnerIterations;
        if (innerIteration + 1 < maxInnerIterations)
        {
            // Next iteration of PGS
            nextIteration = innerIteration + 1;
            completedInnerIterations = false;
        }
        else
        {
            // Next phase.  Reset iteration for next PGS execution
            nextIteration = 0;
            nodeData.numTimesReached = 0;
            completedInnerIterations = true;
        }
        nodeData.innerIteration = nextIteration;

        FmExternalPgsIteration(*taskData);

        if (completedInnerIterations)
        {
            // Increase or reset outer iteration
            uint outerIteration = nodeData.outerIteration;
            bool isLastOuterIteration = outerIteration + 1 >= maxOuterIterations;

            if (isLastOuterIteration)
            {
                outerIteration = 0;

                // Increase or reset pass index (pass with MPCG or GS)
                nodeData.passIdx = 1 - passIdx;
            }
            else
            {
                outerIteration++;
            }

            nodeData.outerIteration = outerIteration;
        }
        else
        {
            FmNextIterationMessages(taskGraph, pairIdx, innerIteration);
        }
    }

#if FM_ASYNC_THREADING
    void FmNodeTaskFuncPartitionRunMpcgFinish(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);
#endif

    // Declared with FM_ASYNC_TASK not FM_NODE_TASK because second task takes care of marking the node complete
    FM_ASYNC_TASK(FmNodeTaskFuncProcessPartitionMpcg)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT("MpcgWork");

        TLTaskGraphNode* node = (TLTaskGraphNode*)inTaskData;
        FmConstraintSolveTaskGraph* taskGraph = (FmConstraintSolveTaskGraph*)node->GetGraph();
        FmTaskGraphSolveData* taskData = &taskGraph->solveData;

        FmScene* scene = taskData->scene;
        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        uint partitionIdx = (uint)inTaskBeginIndex;

        FmConstraintIsland* constraintIsland = constraintSolverData->constraintIsland;
        FmPartition& partition = constraintSolverData->partitions[partitionIdx];

        uint numTetMeshes = partition.numTetMeshes;
        uint numRigidBodies = partition.numObjects - numTetMeshes;
        uint rigidBodyBatchSize = FM_RIGID_BODY_APPLY_DELTAS_SLEEPING_BATCH_SIZE;
        uint numRigidBodyTasks = FmGetNumTasksLimited(numRigidBodies, rigidBodyBatchSize, scene->params.numThreads*4);

#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
        bool isFirstIteration = (node->GetPredecessorMessage() == 1);
        bool isLastIteration = (node->GetPredecessorMessage() == 0);

        //FM_ASSERT(isLastIteration == constraintSolverData->iterationParams.outerIsLastIteration);
#else
        bool isFirstIteration = iterationParams.outerIteration == 0;
        bool isLastIteration = iterationParams.outerIsLastIteration;
#endif

        uint numItemsToRun = numTetMeshes + numRigidBodyTasks;

#if FM_ASYNC_THREADING
        if (numItemsToRun > 0)
        {
            FmTaskDataPartitionRunMpcgOrRbResponse* runData = new FmTaskDataPartitionRunMpcgOrRbResponse(scene, constraintSolverData, constraintIsland, partitionIdx, numTetMeshes, numRigidBodies, numRigidBodyTasks, isFirstIteration, isLastIteration);

            TLTask followTask(FmNodeTaskFuncPartitionRunMpcgFinish, inTaskData, inTaskBeginIndex, inTaskBeginIndex + 1);

            TLParallelForAsync(FmTaskFuncPartitionRunMpcgOrRbResponse,
                runData, 0, numItemsToRun, TLParallelForOptions::GrainFunc(FmGrainFuncPartitionRunMpcgOrRbResponse), followTask, true);
        }
        else
        {
            TLSetNextTask(FmNodeTaskFuncPartitionRunMpcgFinish, inTaskData, inTaskBeginIndex, inTaskBeginIndex + 1);
        }
#else
        FmTaskDataPartitionRunMpcgOrRbResponse runData(scene, constraintSolverData, constraintIsland, partitionIdx, numTetMeshes, numRigidBodies, numRigidBodyTasks, isFirstIteration, isLastIteration);

        TLParallelFor(FmTaskFuncPartitionRunMpcgOrRbResponse, &runData, 0, numItemsToRun,
            TLParallelForOptions::GrainFunc(FmGrainFuncPartitionRunMpcgOrRbResponse));

        node->Finished(0);
#endif
    }

    FM_NODE_TASK(FmNodeTaskFuncPartitionRunMpcgFinish)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT("MpcgWork");

        // Signal to nodes in next outer iteration, which may be the start of the next (GS-based) pass.
        uint partitionId = (uint)inTaskBeginIndex;

        TLTaskGraphNode* node = (TLTaskGraphNode*)inTaskData;
        FmConstraintSolveTaskGraph* taskGraph = (FmConstraintSolveTaskGraph*)node->GetGraph();

        FmNextOuterIterationMessages(taskGraph, partitionId);
    }

#if FM_ASYNC_THREADING
    void FmNodeTaskFuncPartitionGsIterationFinish(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);
#endif

    FM_ASYNC_TASK(FmNodeTaskFuncProcessPartitionGsIterationOrRbResponse)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT("GsIterationWork");

        TLTaskGraphNode* node = (TLTaskGraphNode*)inTaskData;
        FmConstraintSolveTaskGraph* taskGraph = (FmConstraintSolveTaskGraph*)node->GetGraph();
        FmTaskGraphSolveData* taskData = &taskGraph->solveData;

        FmScene* scene = taskData->scene;
        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        uint partitionIdx = (uint)inTaskBeginIndex;

        FmConstraintIsland* constraintIsland = constraintSolverData->constraintIsland;
        FmPartition& partition = constraintSolverData->partitions[partitionIdx];

        uint numTetMeshes = partition.numTetMeshes;
        uint numRigidBodies = partition.numObjects - numTetMeshes;
        uint rigidBodyBatchSize = FM_RIGID_BODY_APPLY_DELTAS_SLEEPING_BATCH_SIZE;
        uint numRigidBodyTasks = FmGetNumTasksLimited(numRigidBodies, rigidBodyBatchSize, scene->params.numThreads*4);

        FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;

        uint passIdx = FM_GS_PASS_IDX;

#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
        uint outerIteration = (uint)node->GetPredecessorMessage();
        bool isLastIteration = (outerIteration == 0);

        FmConstraintSolverControlParams::PassParams& passParams = controlParams.passParams[passIdx];

        uint maxOuterIterations = passParams.maxOuterIterations;
        outerIteration = isLastIteration ? maxOuterIterations - 1 : outerIteration - 1;

        //FM_ASSERT(isLastIteration == constraintSolverData->iterationParams.outerIsLastIteration);
#else
        FmConstraintSolverControlParams::PassParams& passParams = controlParams.passParams[passIdx];

        FmConstraintSolverIterationParams& iterationParams = constraintSolverData->iterationParams;
        bool isLastIteration = iterationParams.outerIsLastIteration;
#endif

        bool forwardDirection = !passParams.useOuterAlternatingDirections || (outerIteration % 2 == 0);
        //FM_ASSERT(forwardDirection == constraintSolverData->iterationParams.outerForwardDirection);

        // If last iteration, include tasks which update the delta pos and delta vel values for rigid bodies based on JTLambda
        uint numItemsToRun = isLastIteration ? (numTetMeshes + numRigidBodyTasks) : numTetMeshes;

#if FM_ASYNC_THREADING
        if (numItemsToRun > 0)
        {
            FmTaskDataPartitionGsIterationOrRbResponse* runData = new FmTaskDataPartitionGsIterationOrRbResponse(scene, constraintSolverData, constraintIsland, partitionIdx, numTetMeshes, numRigidBodies, numRigidBodyTasks,
                isLastIteration, forwardDirection);

            TLTask followTask(FmNodeTaskFuncPartitionGsIterationFinish, inTaskData, inTaskBeginIndex, inTaskBeginIndex + 1);

            TLParallelForAsync(FmTaskFuncPartitionGsIterationOrRbResponse,
                runData, 0, numItemsToRun, TLParallelForOptions::GrainFunc(FmGrainFuncPartitionGsIterationOrRbResponse), followTask, true);
        }
        else
        {
            TLSetNextTask(FmNodeTaskFuncPartitionGsIterationFinish, inTaskData, inTaskBeginIndex, inTaskBeginIndex + 1);
        }
#else
        FmTaskDataPartitionGsIterationOrRbResponse runData(scene, constraintSolverData, constraintIsland, partitionIdx, numTetMeshes, numRigidBodies, numRigidBodyTasks,
            isLastIteration, forwardDirection);

        TLParallelFor(FmTaskFuncPartitionGsIterationOrRbResponse, &runData, 0, numItemsToRun,
            TLParallelForOptions::GrainFunc(FmGrainFuncPartitionGsIterationOrRbResponse));

        node->Finished(0);
#endif
    }

#if FM_ASYNC_THREADING
    FM_NODE_TASK(FmNodeTaskFuncPartitionGsIterationFinish)
    {
        FM_TRACE_SCOPED_EVENT("GsIterationWork");

        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;

        TLTaskGraphNode* node = (TLTaskGraphNode*)inTaskData;

        // Signal nodes in next outer iteration if not complete
        uint partitionId = (uint)inTaskBeginIndex;

        bool isLastOuterIteration = (node->GetPredecessorMessage() == 0);

        FmConstraintSolveTaskGraph* taskGraph = (FmConstraintSolveTaskGraph*)node->GetGraph();

        if (!isLastOuterIteration)
        {
            FmNextOuterIterationMessages(taskGraph, partitionId);
        }
    }
#endif

#else // !FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH

    class FmTaskDataIslandGsIterationOrRbResponse : public TLTaskDataBase
    {
    public:
        FmScene* scene;
        FmConstraintSolverData* constraintSolverData;
        const FmConstraintIsland* constraintIsland;

        uint numTetMeshes;
        uint numRigidBodies;
        uint numRigidBodyBatches;

        FmTaskDataIslandGsIterationOrRbResponse(FmScene* inScene, FmConstraintSolverData* inConstraintSolverData, const FmConstraintIsland* inConstraintIsland,
            uint inNumTetMeshes, uint inNumRigidBodies, uint inNumRigidBodyBatches)
        {
            scene = inScene;
            constraintSolverData = inConstraintSolverData;
            constraintIsland = inConstraintIsland;
            numTetMeshes = inNumTetMeshes;
            numRigidBodies = inNumRigidBodies;
            numRigidBodyBatches = inNumRigidBodyBatches;
        }
    };

    void FmTaskFuncIslandGsIteration(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT("GsIterationWork");

        FmTaskDataIslandGsIterationOrRbResponse* taskData = (FmTaskDataIslandGsIterationOrRbResponse*)inTaskData;
        FmScene* scene = taskData->scene;
        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        const FmConstraintIsland* constraintIsland = taskData->constraintIsland;

        FmConstraintSolverIterationParams& iterationParams = constraintSolverData->iterationParams;

        bool isLastIteration = iterationParams.outerIsLastIteration;
        bool forwardDirection = iterationParams.outerForwardDirection;

        uint beginIdx = (uint)inTaskBeginIndex;
        uint endIdx = (uint)inTaskBeginIndex + 1;

        FmSVector3* deltaVec = constraintSolverData->GetDeltaVec();
        FmSVector3* pgsDeltaVelTerm = constraintSolverData->pgsDeltaVelTerm;

        for (uint islandMeshIdx = beginIdx; islandMeshIdx < endIdx; islandMeshIdx++)
        {
            FmMpcgSolverData& meshData = *FmGetSolverDataById(*scene, constraintIsland->tetMeshIds[islandMeshIdx]);

            if (forwardDirection)
            {
                FmGsIteration(deltaVec, meshData, constraintSolverData->JTlambda);
            }
            else
            {
                FmGsIterationReverse(deltaVec, meshData, constraintSolverData->JTlambda);
            }

            if (!isLastIteration)
            {
                // Update pgsDeltaVelTerm = DAinv * (UA + LA) * deltaVel for next iteration
                FmDiagxNegOffDiagMxV(pgsDeltaVelTerm, meshData, deltaVec);
            }
        }
    }

    // For rigid bodies, compute delta pos or delta vel from JTlambda.
    // Only needed on final iteration of constraint solve
    void FmTaskFuncIslandGsPassRbResponse(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FmTaskDataIslandGsIterationOrRbResponse* taskData = (FmTaskDataIslandGsIterationOrRbResponse*)inTaskData;

        if (taskData->numRigidBodies == 0)
            return;

        uint taskIdx = (uint)inTaskBeginIndex;

        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        FmConstraintSolverBuffer& constraintSolverBuffer = *taskData->scene->constraintSolverBuffer;
        const FmConstraintIsland& constraintIsland = *taskData->constraintIsland;

        FmSVector3* deltaVec = constraintSolverData->GetDeltaVec(); // delta_pos or delta_vel depending on stabilization or solve pass

        uint beginIdx, endIdx;
        FmGetIndexRangeEvenDistribution(&beginIdx, &endIdx, taskIdx, taskData->numRigidBodyBatches, taskData->numRigidBodies);

        FM_ASSERT(constraintSolverData->iterationParams.outerIsLastIteration);

        for (uint rigidBodyIdx = beginIdx; rigidBodyIdx < endIdx; rigidBodyIdx++)
        {
            uint objectId = constraintIsland.rigidBodyIds[rigidBodyIdx];

            FM_ASSERT(FM_IS_SET(objectId, FM_RB_FLAG));

            uint stateOffset = FmGetRigidBodySolverOffsetById(constraintSolverBuffer, objectId);

            deltaVec[stateOffset] = constraintSolverData->DAinv[stateOffset] * constraintSolverData->JTlambda[stateOffset];
            deltaVec[stateOffset + 1] = constraintSolverData->DAinv[stateOffset + 1] * constraintSolverData->JTlambda[stateOffset + 1];
        }
    }

    void FmTaskFuncIslandGsIterationOrRbResponse(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FmTaskDataIslandGsIterationOrRbResponse* taskData = (FmTaskDataIslandGsIterationOrRbResponse*)inTaskData;

        uint numTetMeshes = taskData->numTetMeshes;

        uint objectIdx = (uint)inTaskBeginIndex;

        if (objectIdx < numTetMeshes)
        {
            FmTaskFuncIslandGsIteration(taskData, objectIdx, objectIdx + 1);
        }
        else
        {
            int32_t taskIndex = (int32_t)(objectIdx - numTetMeshes);
            FmTaskFuncIslandGsPassRbResponse(taskData, taskIndex, taskIndex + 1);
        }
    }

    class FmTaskDataIslandRunMpcgOrRbResponse : public TLTaskDataBase
    {
    public:
        FmScene* scene;
        FmConstraintSolverData* constraintSolverData;
        const FmConstraintIsland* constraintIsland;

        uint numRigidBodies;
        uint numRigidBodyBatches;

        FmTaskDataIslandRunMpcgOrRbResponse(FmScene* inScene, FmConstraintSolverData* inConstraintSolverData, const FmConstraintIsland* inConstraintIsland,
            uint inNumRigidBodies, uint inNumRigidBodyBatches)
        {
            scene = inScene;
            constraintSolverData = inConstraintSolverData;
            constraintIsland = inConstraintIsland;
            numRigidBodies = inNumRigidBodies;
            numRigidBodyBatches = inNumRigidBodyBatches;
        }
    };

    void FmTaskFuncIslandRunMpcg(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT("MpcgWork");

        FmTaskDataIslandRunMpcgOrRbResponse* taskData = (FmTaskDataIslandRunMpcgOrRbResponse*)inTaskData;
        FmScene* scene = taskData->scene;
        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        const FmConstraintIsland* constraintIsland = constraintSolverData->constraintIsland;

        FmConstraintSolverIterationParams& iterationParams = constraintSolverData->iterationParams;
        const FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;

        bool isLastIteration = iterationParams.outerIsLastIteration;

        uint beginIdx = (uint)inTaskBeginIndex;
        uint endIdx = (uint)inTaskEndIndex;

        FmSVector3* islandDeltaVec = constraintSolverData->GetDeltaVec();

        for (uint islandMeshIdx = beginIdx; islandMeshIdx < endIdx; islandMeshIdx++)
        {
            uint islandMeshId = constraintIsland->tetMeshIds[islandMeshIdx];

            FmMpcgSolverData& meshData = *FmGetSolverDataById(*scene, islandMeshId);

            FmVpV(meshData.b, meshData.b, &constraintSolverData->JTlambda[meshData.solverStateOffset], meshData.A.numRows);

            FmSVector3* deltavec = &islandDeltaVec[meshData.solverStateOffset];

            uint workerIndex = TLGetTaskSystemThreadIndex();
            uint8_t* tempBuffer = scene->threadTempMemoryBuffer->buffers[workerIndex];
            uint8_t* tempBufferEnd = tempBuffer + scene->threadTempMemoryBuffer->numBytesPerBuffer;

            FmMpcgSolverDataTemps temps;
            temps.Alloc(&tempBuffer, tempBufferEnd, meshData.A.numRows);

            float epsilon = controlParams.epsilonCgPass;
            uint maxIterations = controlParams.maxCgIterationsCgPass;
            FmRunMpcgSolve(deltavec, &meshData, &temps, epsilon, maxIterations);

            if (isLastIteration)
            {
                FmVcV(&constraintSolverData->JTlambda[meshData.solverStateOffset], meshData.b, meshData.A.numRows);

                // If finished the CG pass, update pgsDeltaVelTerm = DAinv * (UA + LA) * deltaVel for GS pass
                FmDiagxNegOffDiagMxV(constraintSolverData->pgsDeltaVelTerm, meshData, islandDeltaVec);
            }
            else
            {
                FmVfill(&constraintSolverData->JTlambda[meshData.solverStateOffset], FmSVector3(0.0f), meshData.A.numRows);
            }
        }
    }

    void FmTaskFuncIslandCgPassRbResponse(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FmTaskDataIslandRunMpcgOrRbResponse* taskData = (FmTaskDataIslandRunMpcgOrRbResponse*)inTaskData;

        if (taskData->numRigidBodies == 0)
            return;

        uint taskIdx = (uint)inTaskBeginIndex;

        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        FmConstraintSolverBuffer& constraintSolverBuffer = *taskData->scene->constraintSolverBuffer;
        const FmConstraintIsland* constraintIsland = constraintSolverData->constraintIsland;

        FmSVector3* deltaVec = constraintSolverData->GetDeltaVec(); // delta_pos or delta_vel depending on stabilization or solve pass

        FmConstraintSolverIterationParams& iterationParams = constraintSolverData->iterationParams;

        bool isFirstOuterIteration = iterationParams.outerIteration == 0;
        bool isLastOuterIteration = iterationParams.outerIsLastIteration;

        uint beginIdx, endIdx;
        FmGetIndexRangeEvenDistribution(&beginIdx, &endIdx, taskIdx, taskData->numRigidBodyBatches, taskData->numRigidBodies);

        for (uint rigidBodyIdx = beginIdx; rigidBodyIdx < endIdx; rigidBodyIdx++)
        {
            uint objectId = constraintIsland->rigidBodyIds[rigidBodyIdx];  // rigid bodies after tet meshes in objectIds

            FM_ASSERT(FM_IS_SET(objectId, FM_RB_FLAG));

            uint stateOffset = FmGetRigidBodySolverOffsetById(constraintSolverBuffer, objectId);

            FmSVector3 totalJTlambda0;
            FmSVector3 totalJTlambda1;

            if (isFirstOuterIteration)
            {
                totalJTlambda0 = FmSVector3(0.0f);
                totalJTlambda1 = FmSVector3(0.0f);
            }
            else
            {
                totalJTlambda0 = constraintSolverData->velTemp[stateOffset];
                totalJTlambda1 = constraintSolverData->velTemp[stateOffset + 1];
            }

            totalJTlambda0 += constraintSolverData->JTlambda[stateOffset];
            totalJTlambda1 += constraintSolverData->JTlambda[stateOffset + 1];

            deltaVec[stateOffset] = constraintSolverData->DAinv[stateOffset] * totalJTlambda0;
            deltaVec[stateOffset + 1] = constraintSolverData->DAinv[stateOffset + 1] * totalJTlambda1;

            if (isLastOuterIteration)
            {
                constraintSolverData->JTlambda[stateOffset] = totalJTlambda0;
                constraintSolverData->JTlambda[stateOffset + 1] = totalJTlambda1;
            }
            else
            {
                constraintSolverData->velTemp[stateOffset] = totalJTlambda0;
                constraintSolverData->velTemp[stateOffset + 1] = totalJTlambda1;

                constraintSolverData->JTlambda[stateOffset] = FmSVector3(0.0f);
                constraintSolverData->JTlambda[stateOffset + 1] = FmSVector3(0.0f);
            }
        }
    }

    void FmTaskFuncIslandRunMpcgOrRbResponse(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FmTaskDataIslandRunMpcgOrRbResponse* taskData = (FmTaskDataIslandRunMpcgOrRbResponse*)inTaskData;
        const FmConstraintIsland* constraintIsland = taskData->constraintIsland;

        uint numTetMeshes = constraintIsland->numTetMeshes;

        uint partitionObjectBeginIndex = (uint)inTaskBeginIndex;
        uint partitionObjectEndIndex = (uint)inTaskEndIndex;

        if (partitionObjectBeginIndex < numTetMeshes)
        {
            FmTaskFuncIslandRunMpcg(taskData, partitionObjectBeginIndex, partitionObjectEndIndex);
        }
        else
        {
            int32_t taskIndex = (int32_t)(partitionObjectBeginIndex - numTetMeshes);
            FmTaskFuncIslandCgPassRbResponse(taskData, taskIndex, taskIndex + 1);
        }
    }
#endif

    class FmTaskDataPartitionPairPgsIteration : public TLTaskDataBase
    {
    public:
        FmScene* scene;
        FmConstraintSolverData* constraintSolverData;
        FmGraphColoringSet* partitionPairSet;

        FmTaskDataPartitionPairPgsIteration(
            FmScene* inScene,
            FmConstraintSolverData* inConstraintSolverData,
            FmGraphColoringSet* inPartitionPairSet)
        {
            scene = inScene;
            constraintSolverData = inConstraintSolverData;
            partitionPairSet = inPartitionPairSet;
        }
    };


    FM_ASYNC_TASK(FmTaskFuncPartitionPairPgsIteration)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT("PgsIterationWork");

        FmTaskDataPartitionPairPgsIteration* taskData = (FmTaskDataPartitionPairPgsIteration*)inTaskData;
        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        FmGraphColoringSet* partitionPairSet = taskData->partitionPairSet;

        FmConstraintSolverIterationParams& iterationParams = constraintSolverData->iterationParams;

        uint outerIteration = iterationParams.outerIteration;

        bool pass0Included = constraintSolverData->currentControlParams->passParams[FM_CG_PASS_IDX].maxOuterIterations > 0;
        uint passIdx = constraintSolverData->currentPassIdx;
        bool forwardDirection = iterationParams.innerForwardDirection;

        uint beginIdx = (uint)inTaskBeginIndex;
        uint endIdx = (uint)inTaskBeginIndex + 1;

        uint* partitionPairIndicesStart = &partitionPairSet->pStart[beginIdx];
        uint numPartitionPairs = endIdx - beginIdx;

        FmSVector3* pgsRhsInput = constraintSolverData->GetRhsInput(passIdx);

        for (uint i = 0; i < numPartitionPairs; i++)
        {
            uint pairIdx = partitionPairIndicesStart[i];

            FmPartitionPair& partitionPair = constraintSolverData->partitionPairs[pairIdx];

            if ((passIdx == 1 && pass0Included) || (outerIteration > 0))
            {
                FmUpdatePartitionPairPgsRhs(constraintSolverData, partitionPair, pgsRhsInput);
            }

            partitionPair.norms.Zero();

            if (forwardDirection)
            {
                FmPgsIteration3(&partitionPair.norms, constraintSolverData, partitionPair.constraintIndices, partitionPair.numConstraints, passIdx);
            }
            else
            {
                FmPgsIteration3Reverse(&partitionPair.norms, constraintSolverData, partitionPair.constraintIndices, partitionPair.numConstraints, passIdx);
            }
        }
    }

#if !FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
    struct FmTaskDataRunMpcg : public TLTaskDataBase
    {
        FmScene* scene;
        FmConstraintSolverData* constraintSolverData;
        const FmConstraintIsland* constraintIsland;

        FM_CLASS_NEW_DELETE(FmTaskDataRunMpcg);

        FmTaskDataRunMpcg(FmScene* inScene, FmConstraintSolverData* inConstraintSolverData, const FmConstraintIsland* inConstraintIsland)
        {
            scene = inScene;
            constraintSolverData = inConstraintSolverData;
            constraintIsland = inConstraintIsland;
        }
    };

    void FmTaskFuncRunMpcg(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT("MpcgWork");

        FmTaskDataRunMpcg* taskData = (FmTaskDataRunMpcg*)inTaskData;
        FmScene* scene = taskData->scene;
        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        const FmConstraintIsland* constraintIsland = constraintSolverData->constraintIsland;

        const FmConstraintSolverIterationParams& iterationParams = constraintSolverData->iterationParams;
        const FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;

        bool isLastIteration = iterationParams.outerIsLastIteration;

        uint beginIdx = (uint)inTaskBeginIndex;
        uint endIdx = (uint)inTaskBeginIndex + 1;

        FmSVector3* islandDeltaVec = constraintSolverData->GetDeltaVec();

        for (uint islandMeshIdx = beginIdx; islandMeshIdx < endIdx; islandMeshIdx++)
        {
            FmMpcgSolverData& meshData = *FmGetSolverDataById(*scene, constraintIsland->tetMeshIds[islandMeshIdx]);

            FmVpV(meshData.b, meshData.b, &constraintSolverData->JTlambda[meshData.solverStateOffset], meshData.A.numRows);

            FmSVector3* deltavec = &islandDeltaVec[meshData.solverStateOffset];

            float epsilon = controlParams.epsilonCgPass;
            uint maxIterations = controlParams.maxCgIterationsCgPass;

            uint workerIndex = TLGetTaskSystemThreadIndex();
            uint8_t* tempBuffer = scene->threadTempMemoryBuffer->buffers[workerIndex];
            uint8_t* tempBufferEnd = tempBuffer + scene->threadTempMemoryBuffer->numBytesPerBuffer;
            FmMpcgSolverDataTemps temps;
            temps.Alloc(&tempBuffer, tempBufferEnd, meshData.A.numRows);

            FmRunMpcgSolve(deltavec, &meshData, &temps, epsilon, maxIterations);

            if (isLastIteration)
            {
                FmVcV(&constraintSolverData->JTlambda[meshData.solverStateOffset], meshData.b, meshData.A.numRows);

                // If finished the CG pass, update pgsDeltaVelTerm = DAinv * (UA + LA) * deltaVel for GS pass
                FmDiagxNegOffDiagMxV(constraintSolverData->pgsDeltaVelTerm, meshData, islandDeltaVec);
            }
            else
            {
                FmVfill(&constraintSolverData->JTlambda[meshData.solverStateOffset], FmSVector3(0.0f), meshData.A.numRows);
            }
        }
    }

    void FmPgsSolve(FmScene* scene, FmConstraintSolverData* constraintSolverData, const FmConstraintIsland& constraintIsland)
    {
        (void)constraintIsland;

        uint passIdx = constraintSolverData->currentPassIdx;
        FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;

        FmConstraintSolverControlParams::PassParams& passParams = controlParams.passParams[passIdx];

        FmConstraintSolverIterationParams& iterationParams = constraintSolverData->iterationParams;

        uint numPartitionPairSets = constraintSolverData->numPartitionPairIndependentSets;
        uint maxInnerIterations = passParams.currentMaxInnerIterations;

        for (uint innerIteration = 0; innerIteration < maxInnerIterations; innerIteration++)
        {
            bool forwardDirection = !passParams.useInnerAlternatingDirections || (innerIteration % 2 == 0);

            // Set parameters read in task function
            iterationParams.innerForwardDirection = forwardDirection;

            // Run a pass solving only TetMesh/TetMesh and TetMesh/RigidBody constraints.
            for (uint pairSetIdx = 0; pairSetIdx < numPartitionPairSets; pairSetIdx++)
            {
                FmGraphColoringSet& independentSet = constraintSolverData->partitionPairIndependentSets[pairSetIdx];

                FmTaskDataPartitionPairPgsIteration taskDataPairIteration(scene, constraintSolverData, &independentSet);
                TLParallelFor(FmTaskFuncPartitionPairPgsIteration, &taskDataPairIteration, 0, (int)independentSet.numElements);
            }

            FmTaskGraphSolveData externalPgsData;
            externalPgsData.scene = scene;
            externalPgsData.constraintSolverData = constraintSolverData;
            externalPgsData.constraintIsland = &constraintIsland;

            FmExternalPgsIteration(externalPgsData);
        }
    }
#endif

#if FM_ASYNC_THREADING
    class FmRunConstraintSolvePassIterationData
    {
    public:
        FM_CLASS_NEW_DELETE(FmRunConstraintSolvePassIterationData)

        FmScene* scene;
        FmConstraintSolverData* constraintSolverData;
        FmConstraintIsland* constraintIsland;
        uint outerIteration;

        TLTaskFuncCallback followTaskFunc;
        void* followTaskData;

        FmRunConstraintSolvePassIterationData(FmScene* inScene,
            FmConstraintSolverData* inConstraintSolverData,
            FmConstraintIsland* inConstraintIsland,
            TLTaskFuncCallback inFollowTaskFunc,
            void* inFollowTaskData)
        {
            scene = inScene;
            constraintSolverData = inConstraintSolverData;
            constraintIsland = inConstraintIsland;
            outerIteration = 0;
            followTaskFunc = inFollowTaskFunc;
            followTaskData = inFollowTaskData;
        }
    };

    void FmTaskFuncRunConstraintSolveCgPassIterationStart(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    FM_ASYNC_TASK(FmTaskFuncRunConstraintSolveBegin)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT("IslandSolveRunSolveBegin");

        (void)inTaskBeginIndex;
        FmRunConstraintSolvePassIterationData* iterationData = (FmRunConstraintSolvePassIterationData*)inTaskData;

        FmScene* scene = iterationData->scene;
        FmConstraintSolverData* constraintSolverData = iterationData->constraintSolverData;

        bool rigidBodiesExternal = scene->params.rigidBodiesExternal;

        if (rigidBodiesExternal)
        {
            FmConstraintIsland& constraintIsland = *iterationData->constraintIsland;

#if FM_CONSTRAINT_STABILIZATION_SOLVE
            // Copy any changes from solving RigidBody/RigidBody constraints externally.
            if (constraintSolverData->IsInStabilization())
            {
                FmReadRigidBodyDeltaPos(constraintSolverData, constraintIsland, *scene);
            }
            else
#endif
            {
                FmReadRigidBodyDeltaVel(constraintSolverData, constraintIsland, *scene);
            }
        }

        constraintSolverData->currentPassIdx = 0;

#if FM_CONSTRAINT_SOLVER_CONVERGENCE_TEST
        constraintSolverData->externaPgsNorms.Zero();
#endif

        TLSetNextTask(FmTaskFuncRunConstraintSolveCgPassIterationStart, iterationData, 0, 1);
    }

    void FmTaskFuncRunConstraintSolveCgPassIterationMiddle(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);
    void FmTaskFuncRunConstraintSolveMidPasses(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    FM_ASYNC_TASK(FmTaskFuncRunConstraintSolveCgPassIterationStart)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT("IslandSolveCgPassStart");

        (void)inTaskBeginIndex;

        FmRunConstraintSolvePassIterationData* iterationData = (FmRunConstraintSolvePassIterationData*)inTaskData;

        FmConstraintSolverData* constraintSolverData = iterationData->constraintSolverData;

        FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;
        FmConstraintSolverControlParams::PassParams& passParams = controlParams.passParams[FM_CG_PASS_IDX];

        const uint maxOuterIterationsCgPass = passParams.maxOuterIterations;

        if (iterationData->outerIteration >= maxOuterIterationsCgPass)
        {
            FmTaskFuncRunConstraintSolveMidPasses(iterationData, 0, 1);
            return;
        }

        // To help convergence, add an initial pass to the constraint solver which iteratively applies impulses to point 
        // masses and uses PCG to compute response of tet meshes.  Resulting constraint forces and velocity changes are 
        // used to warm start the standard GS-based solver.

        TLSetNextTask(FmTaskFuncRunConstraintSolveCgPassIterationMiddle, iterationData, 0, 1);
    }

    void FmTaskFuncRunConstraintSolveCgPassIterationEnd(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);
    void FmTaskFuncRunConstraintSolvePostGraph(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    FM_ASYNC_TASK(FmTaskFuncRunConstraintSolveCgPassIterationMiddle)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;

        FmRunConstraintSolvePassIterationData* iterationData = (FmRunConstraintSolvePassIterationData*)inTaskData;

        FmConstraintSolverData* constraintSolverData = iterationData->constraintSolverData;

        uint outerIteration = iterationData->outerIteration;

        FmConstraintSolverIterationParams& iterationParams = constraintSolverData->iterationParams;

        const FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;
        const FmConstraintSolverControlParams::PassParams& passParams = controlParams.passParams[FM_CG_PASS_IDX];

        const uint maxOuterIterationsCgPass = passParams.maxOuterIterations;

        bool isLastIteration = (outerIteration >= maxOuterIterationsCgPass - 1);

        // Set parameter read in task function
        iterationParams.outerIteration = outerIteration;
        iterationParams.outerIsLastIteration = isLastIteration;

#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
        // Run both CG and GS-based passes
        FmRunConstraintSolveTaskGraphAsync(constraintSolverData->taskGraph, FmTaskFuncRunConstraintSolvePostGraph, iterationData);
#else
        FmScene* scene = iterationData->scene;
        FmPgsSolve(scene, constraintSolverData, *iterationData->constraintIsland);

        uint numIslandTetMeshes = iterationData->constraintIsland->numTetMeshes;
        if (numIslandTetMeshes > 0)
        {
            FmTaskDataRunMpcg* taskDataRunMpcg = new FmTaskDataRunMpcg(scene, constraintSolverData, iterationData->constraintIsland);

            TLTask followTask(FmTaskFuncRunConstraintSolveCgPassIterationEnd, iterationData);

            TLParallelForAsync(FmTaskFuncRunMpcg, taskDataRunMpcg, 0, numIslandTetMeshes, TLParallelForOptions::GrainSize(1), followTask, true);
        }
        else
        {
            TLSetNextTask(FmTaskFuncRunConstraintSolveCgPassIterationEnd, iterationData, 0, 1);
        }
#endif
    }

    FM_ASYNC_TASK(FmTaskFuncRunConstraintSolveCgPassIterationEnd)
    {
        FM_TRACE_SCOPED_EVENT("IslandSolveCgPassEnd");

        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;

        FmRunConstraintSolvePassIterationData* iterationData = (FmRunConstraintSolvePassIterationData*)inTaskData;

        FmConstraintSolverData* constraintSolverData = iterationData->constraintSolverData;
        FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;
        FmConstraintSolverControlParams::PassParams& passParams = controlParams.passParams[FM_CG_PASS_IDX];

        if (passParams.useInnerIterationsFalloff && passParams.currentMaxInnerIterations > 1)
        {
            passParams.currentMaxInnerIterations--;
        }

        iterationData->outerIteration++;

        TLSetNextTask(FmTaskFuncRunConstraintSolveCgPassIterationStart, iterationData, 0, 1);
    }

    void FmTaskFuncRunConstraintSolveGsPassStart(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    FM_ASYNC_TASK(FmTaskFuncRunConstraintSolveMidPasses)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmRunConstraintSolvePassIterationData* iterationData = (FmRunConstraintSolvePassIterationData*)inTaskData;

        FmConstraintSolverData* constraintSolverData = iterationData->constraintSolverData;

        constraintSolverData->currentPassIdx = 1;

        iterationData->outerIteration = 0;

        FmTaskFuncRunConstraintSolveGsPassStart(iterationData, 0, 1);
    }

    void FmTaskFuncRunConstraintSolveGsPassMiddle(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    FM_ASYNC_TASK(FmTaskFuncRunConstraintSolveGsPassStart)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmRunConstraintSolvePassIterationData* iterationData = (FmRunConstraintSolvePassIterationData*)inTaskData;

        FmScene* scene = iterationData->scene;
        FmConstraintSolverData* constraintSolverData = iterationData->constraintSolverData;

        FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;
        FmConstraintSolverControlParams::PassParams& passParams = controlParams.passParams[FM_GS_PASS_IDX];
        bool rigidBodiesExternal = scene->params.rigidBodiesExternal;

        const uint maxOuterIterations = passParams.maxOuterIterations;

        if (iterationData->outerIteration >= maxOuterIterations)
        {
            FmConstraintIsland& constraintIsland = *iterationData->constraintIsland;

            if (rigidBodiesExternal)
            {
                // Copy constraint solver results into rigid bodies
#if FM_CONSTRAINT_STABILIZATION_SOLVE
                if (constraintSolverData->IsInStabilization())
                {
                    FmWriteRigidBodyDeltaPos(scene, constraintSolverData, constraintIsland);
                }
                else
#endif
                {
                    FmWriteRigidBodyDeltaVel(scene, constraintSolverData, constraintIsland);
                }
            }

            // Solve finished, setup next task, and delete solver iteration data.
            // The next task will get the island id from the task index.
            int32_t taskIndex = (int32_t)constraintIsland.islandId;
            TLSetNextTask(iterationData->followTaskFunc, iterationData->followTaskData, taskIndex, taskIndex + 1);

            delete iterationData;

            return;
        }

        // Run a pass solving only TetMesh/TetMesh and TetMesh/RigidBody constraints.

        TLSetNextTask(FmTaskFuncRunConstraintSolveGsPassMiddle, iterationData, 0, 1);
    }

    FM_ASYNC_TASK(FmTaskFuncRunConstraintSolvePostGraph)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmRunConstraintSolvePassIterationData* iterationData = (FmRunConstraintSolvePassIterationData*)inTaskData;

        FmScene* scene = iterationData->scene;
        FmConstraintSolverData* constraintSolverData = iterationData->constraintSolverData;
        FmConstraintIsland& constraintIsland = *iterationData->constraintIsland;

        bool rigidBodiesExternal = scene->params.rigidBodiesExternal;

        if (rigidBodiesExternal)
        {
            // Copy constraint solver results into rigid bodies
#if FM_CONSTRAINT_STABILIZATION_SOLVE
            if (constraintSolverData->IsInStabilization())
            {
                FmWriteRigidBodyDeltaPos(scene, constraintSolverData, constraintIsland);
            }
            else
#endif
            {
                FmWriteRigidBodyDeltaVel(scene, constraintSolverData, constraintIsland);
            }
        }

        // Solve finished, setup next task, and delete solver iteration data.
        // The next task will get the island id from the task index.
        int32_t taskIndex = (int32_t)constraintIsland.islandId;
        TLSetNextTask(iterationData->followTaskFunc, iterationData->followTaskData, taskIndex, taskIndex + 1);

        delete iterationData;
    }

    void FmTaskFuncRunConstraintSolveGsPassEnd(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    FM_ASYNC_TASK(FmTaskFuncRunConstraintSolveGsPassMiddle)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmRunConstraintSolvePassIterationData* iterationData = (FmRunConstraintSolvePassIterationData*)inTaskData;

        FmConstraintSolverData* constraintSolverData = iterationData->constraintSolverData;

        FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;
        FmConstraintSolverControlParams::PassParams& passParams = controlParams.passParams[FM_GS_PASS_IDX];

        FmConstraintSolverIterationParams& iterationParams = constraintSolverData->iterationParams;

        uint outerIteration = iterationData->outerIteration;
        uint maxOuterIterations = passParams.maxOuterIterations;

        bool isLastIteration = (outerIteration >= maxOuterIterations - 1);
        bool outerForwardDirection = !passParams.useOuterAlternatingDirections || (outerIteration % 2 == 0);

        // Set parameters read in task function
        iterationParams.outerIteration = outerIteration;
        iterationParams.outerIsLastIteration = isLastIteration;
        iterationParams.outerForwardDirection = outerForwardDirection;

        // Run GS-based pass
#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
        FmRunConstraintSolveTaskGraphAsync(constraintSolverData->taskGraph, FmTaskFuncRunConstraintSolvePostGraph, iterationData);
#else
        FmScene* scene = iterationData->scene;

        FmPgsSolve(scene, constraintSolverData, *iterationData->constraintIsland);

        FmConstraintIsland& constraintIsland = *iterationData->constraintIsland;

        uint numTetMeshes = constraintIsland.numTetMeshes;
        uint numRigidBodies = constraintIsland.numRigidBodiesInFEMSolve;
        uint rigidBodyBatchSize = FM_RIGID_BODY_APPLY_DELTAS_SLEEPING_BATCH_SIZE;
        uint numRigidBodyTasks = FmGetNumTasksLimited(numRigidBodies, rigidBodyBatchSize, scene->params.numThreads*4);

        // If last iteration, include tasks which update the delta pos and delta vel values for rigid bodies based on JTLambda
        uint numItemsToRun = isLastIteration ? (numTetMeshes + numRigidBodyTasks) : numTetMeshes;

        if (numItemsToRun > 0)
        {
            FmTaskDataIslandGsIterationOrRbResponse* taskData = new FmTaskDataIslandGsIterationOrRbResponse(scene, constraintSolverData, iterationData->constraintIsland,
                numTetMeshes, numRigidBodies, numRigidBodyTasks);

            TLTask followTask(FmTaskFuncRunConstraintSolveGsPassEnd, iterationData);

            TLParallelForAsync(FmTaskFuncIslandGsIterationOrRbResponse, taskData, 0, numItemsToRun, TLParallelForOptions::GrainSize(1), followTask, true);
        }
        else
        {
            TLSetNextTask(FmTaskFuncRunConstraintSolveGsPassEnd, iterationData, 0, 1);
        }
#endif
    }

    FM_ASYNC_TASK(FmTaskFuncRunConstraintSolveGsPassEnd)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        FmRunConstraintSolvePassIterationData* iterationData = (FmRunConstraintSolvePassIterationData*)inTaskData;

        FmConstraintSolverData* constraintSolverData = iterationData->constraintSolverData;

        FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;
        FmConstraintSolverControlParams::PassParams& passParams = controlParams.passParams[FM_GS_PASS_IDX];

        if (passParams.useInnerIterationsFalloff && passParams.currentMaxInnerIterations > 1)
        {
            passParams.currentMaxInnerIterations--;
        }

        iterationData->outerIteration++;

        TLSetNextTask(FmTaskFuncRunConstraintSolveGsPassStart, iterationData, 0, 1);
    }
#endif

    void FmTaskFuncRunConstraintSolveBegin(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    void FmRunConstraintSolve(FmScene* scene, FmConstraintSolverData* constraintSolverData, FmConstraintIsland& constraintIsland,
        TLTaskFuncCallback followTaskFunc, void* followTaskData)
    {
#if FM_ASYNC_THREADING
        if (followTaskFunc)
        {
            FmRunConstraintSolvePassIterationData* iterationData = new FmRunConstraintSolvePassIterationData(scene, constraintSolverData, &constraintIsland, followTaskFunc, followTaskData);

            FmTaskFuncRunConstraintSolveBegin(iterationData, 0, 1);

            return;
        }
#else
        (void)followTaskFunc;
        (void)followTaskData;

        FmConstraintSolverIterationParams& iterationParams = constraintSolverData->iterationParams;

        bool rigidBodiesExternal = scene->params.rigidBodiesExternal;
        bool inStabilization = constraintSolverData->IsInStabilization();

        if (rigidBodiesExternal)
        {
#if FM_CONSTRAINT_STABILIZATION_SOLVE
            // Copy any changes from solving RigidBody/RigidBody constraints externally.
            if (inStabilization)
            {
                FmReadRigidBodyDeltaPos(constraintSolverData, constraintIsland, *scene);
            }
            else
#endif
            {
                FmReadRigidBodyDeltaVel(constraintSolverData, constraintIsland, *scene);
            }
        }

        constraintSolverData->currentPassIdx = 0;

        // To help convergence, add an initial pass to the constraint solver which iteratively applies impulses to point 
        // masses and uses PCG to compute response of tet meshes.  Resulting constraint forces and velocity changes are 
        // used to warm start the standard GS-based solver.

        FmConstraintSolverControlParams& controlParams = *constraintSolverData->currentControlParams;

        FmConstraintSolverControlParams::PassParams passParamsCopy = controlParams.passParams[FM_CG_PASS_IDX];

        const uint maxOuterIterationsCgPass = passParamsCopy.maxOuterIterations;

        if (maxOuterIterationsCgPass > 0)
        {
            for (uint outerIteration = 0; outerIteration < maxOuterIterationsCgPass; outerIteration++)
            {
                bool isLastIteration = (outerIteration >= maxOuterIterationsCgPass - 1);

                // Set parameter read in task function
                iterationParams.outerIteration = outerIteration;
                iterationParams.outerIsLastIteration = isLastIteration;

#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
                FmRunConstraintSolveTaskGraph(constraintSolverData->taskGraph);
#else
                FmPgsSolve(scene, constraintSolverData, constraintIsland);

                uint numTetMeshes = constraintIsland.numTetMeshes;
                uint numRigidBodies = constraintIsland.numRigidBodiesInFEMSolve;
                uint rigidBodyBatchSize = FM_RIGID_BODY_APPLY_DELTAS_SLEEPING_BATCH_SIZE;
                uint numRigidBodyTasks = FmGetNumTasksLimited(numRigidBodies, rigidBodyBatchSize, scene->params.numThreads * 4);

                uint numItemsToRun = numTetMeshes + numRigidBodyTasks;

                FmTaskDataIslandRunMpcgOrRbResponse taskDataRunMpcg(scene, constraintSolverData, &constraintIsland, numRigidBodies, numRigidBodyTasks);
                TLParallelFor(FmTaskFuncIslandRunMpcgOrRbResponse, &taskDataRunMpcg, 0, numItemsToRun);
#endif

                if (passParamsCopy.useInnerIterationsFalloff && passParamsCopy.maxInnerIterations > 1)
                {
                    passParamsCopy.maxInnerIterations--;
                }
            }

            FmDiagxNegOffDiagMxV(constraintSolverData->pgsDeltaVelTerm, scene, constraintIsland, constraintSolverData->GetDeltaVec());
        }

        constraintSolverData->currentPassIdx = 1;
        passParamsCopy = controlParams.passParams[FM_GS_PASS_IDX];

        // Run a pass solving only TetMesh/TetMesh and TetMesh/RigidBody constraints.
        const uint maxOuterIterations = passParamsCopy.maxOuterIterations;
        for (uint outerIteration = 0; outerIteration < maxOuterIterations; outerIteration++)
        {
            bool isLastIteration = (outerIteration >= maxOuterIterations - 1);
            bool outerForwardDirection = !passParamsCopy.useOuterAlternatingDirections || (outerIteration % 2 == 0);

            // Set parameters read in task function
            iterationParams.outerIteration = outerIteration;
            iterationParams.outerIsLastIteration = isLastIteration;
            iterationParams.outerForwardDirection = outerForwardDirection;

            FmSolverIterationNorms outerNorms;
            outerNorms.Zero();

#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
            FmRunConstraintSolveTaskGraph(constraintSolverData->taskGraph);
#else
            FmPgsSolve(scene, constraintSolverData, constraintIsland);

            uint numTetMeshes = constraintIsland.numTetMeshes;
            uint numRigidBodies = constraintIsland.numRigidBodiesInFEMSolve;
            uint rigidBodyBatchSize = FM_RIGID_BODY_APPLY_DELTAS_SLEEPING_BATCH_SIZE;
            uint numRigidBodyTasks = FmGetNumTasksLimited(numRigidBodies, rigidBodyBatchSize, scene->params.numThreads * 4);

            // If last iteration, include tasks which update the delta pos and delta vel values for rigid bodies based on JTLambda
            uint numItemsToRun = isLastIteration ? (numTetMeshes + numRigidBodyTasks) : numTetMeshes;

            FmTaskDataIslandGsIterationOrRbResponse taskData(scene, constraintSolverData, &constraintIsland, numTetMeshes, numRigidBodies, numRigidBodyTasks);
            TLParallelFor(FmTaskFuncIslandGsIterationOrRbResponse, &taskData, 0, numItemsToRun);
#endif

            if (passParamsCopy.useInnerIterationsFalloff && passParamsCopy.maxInnerIterations > 1)
            {
                passParamsCopy.maxInnerIterations--;
            }
        }

        if (rigidBodiesExternal)
        {
            // Copy constraint solver results into rigid bodies
            if (inStabilization)
            {
                FmWriteRigidBodyDeltaPos(scene, constraintSolverData, constraintIsland);
            }
            else
            {
                FmWriteRigidBodyDeltaVel(scene, constraintSolverData, constraintIsland);
            }
        }
#endif
    }

    // Save constraint solver lambda results in the persistent constraints (currently glue, plane, and rb angle) to communicate to application.
    void FmUpdateConstraintLambdas(
        FmConstraintSolverData* constraintSolverData,
        FmConstraintsBuffer* constraintsBuffer,
        const FmConstraintIsland& constraintIsland)
    {
        // Update lambda values in glue and plane constraints, break glue
        for (uint islandConstraintIdx = 0; islandConstraintIdx < constraintIsland.numConstraints; islandConstraintIdx++)
        {
            FmConstraintReference& constraintRef = constraintIsland.constraintRefs[islandConstraintIdx];
            FmSVector3& lambda = constraintSolverData->lambda3[islandConstraintIdx];

            if (constraintRef.type == FM_CONSTRAINT_TYPE_PLANE)
            {
                FmPlaneConstraint& planeConstraint = constraintsBuffer->planeConstraints[constraintRef.idx];
                planeConstraint.lambda0 = lambda.x;
                planeConstraint.lambda1 = lambda.y;
                planeConstraint.lambda2 = lambda.z;
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_GLUE)
            {
                FmGlueConstraint& glueConstraint = constraintsBuffer->glueConstraints[constraintRef.idx];
                glueConstraint.lambdaX = lambda.x;
                glueConstraint.lambdaY = lambda.y;
                glueConstraint.lambdaZ = lambda.z;
            }
            else if (constraintRef.type == FM_CONSTRAINT_TYPE_RIGID_BODY_ANGLE)
            {
                FmRigidBodyAngleConstraint& angleConstraint = constraintsBuffer->rigidBodyAngleConstraints[constraintRef.idx];
                angleConstraint.lambda0 = lambda.x;
                angleConstraint.lambda1 = lambda.y;
                angleConstraint.lambda2 = lambda.z;
            }
        }
    }

    // Apply island solve position and velocity deltas, test sleeping, update tet mesh positions from velocity,
    // and compute new fractures or plastic deformation
    void FmPostIslandSolveUpdatePositionsPlasticityFracture(
        FmScene* scene, FmTetMesh* inTetMesh, 
        FmConstraintIsland* constraintIsland,
        const FmConstraintSolverData& constraintSolverData, float timestep,
        TLTaskDataBase* updateIslandsProgress)
    {
        FmTetMesh& tetMesh = *inTetMesh;

        float maxSpeedThreshold = tetMesh.sleepMaxSpeedThreshold;
        float avgSpeedThreshold = tetMesh.sleepAvgSpeedThreshold;
        uint stableCount = tetMesh.sleepStableCount;

        uint stateOffset = tetMesh.solverData->solverStateOffset;

        bool dynamicVert = false;
        uint numFixedPoints = 0;

        for (uint i = 0; i < tetMesh.numVerts; i++)
        {
            uint si = i;

            if (!FM_IS_SET(tetMesh.vertsFlags[i], FM_VERT_FLAG_KINEMATIC))
            {
                // Apply constraint solve corrections to position and velocity
#if FM_CONSTRAINT_STABILIZATION_SOLVE
                FmVector3 deltaPos = FmVector3(constraintSolverData.deltaPos[stateOffset + si]);
                tetMesh.vertsPos[i] += deltaPos;
#endif

                FmVector3 deltaVel = FmVector3(constraintSolverData.deltaVel[stateOffset + si]);
                tetMesh.vertsVel[i] += deltaVel;

                dynamicVert = true;
            }
            else if (FmIsZero(tetMesh.vertsVel[i]))
            {
                // Count kinematic and zero vel verts
                numFixedPoints++;
            }

            FmUpdateVelStats(&tetMesh.velStats, length(tetMesh.vertsVel[i]), 1000);
        }

        if (dynamicVert)
        {
            tetMesh.flags |= (FM_OBJECT_FLAG_POS_CHANGED | FM_OBJECT_FLAG_VEL_CHANGED);
        }

        // Update constraint island sleeping values
        if (numFixedPoints > 0)
        {
            FmAtomicAdd(&constraintIsland->numFixedPoints.val, numFixedPoints);
        }

        if (FM_IS_SET(tetMesh.flags, FM_OBJECT_FLAG_SLEEPING_DISABLED)
            || !FmCheckStable(&tetMesh.velStats, maxSpeedThreshold, avgSpeedThreshold, stableCount))
        {
            if (FmAtomicRead(&constraintIsland->isIslandStable.val) == 1)
            {
                FmAtomicWrite(&constraintIsland->isIslandStable.val, 0);
            }
        }

        FmUpdatePositionsFracturePlasticity(scene, &tetMesh, timestep, updateIslandsProgress);
    }

    FM_ASYNC_TASK(FmTaskFuncMeshApplySolveDeltasAndTestSleeping)
    {
        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;

        FM_TRACE_SCOPED_EVENT("IslandSolveMeshApplySolveDeltas");

        FmTaskDataApplySolveDeltasAndTestSleeping* taskData = (FmTaskDataApplySolveDeltasAndTestSleeping*)inTaskData;
        FmScene* scene = taskData->scene;
        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        FmConstraintIsland& constraintIsland = *taskData->constraintIsland;
        float timestep = taskData->timestep;

        uint beginIslandMeshIdx = (uint)inTaskBeginIndex;
        uint endIslandMeshIdx = (uint)inTaskEndIndex;

        // Apply the solve results and test sleeping for all the input tet meshes.

        for (uint islandMeshIdx = beginIslandMeshIdx; islandMeshIdx < endIslandMeshIdx; islandMeshIdx++)
        {
            FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, constraintIsland.tetMeshIds[islandMeshIdx]);

#if FM_ASYNC_THREADING
            FmPostIslandSolveUpdatePositionsPlasticityFracture(scene, &tetMesh, &constraintIsland, *constraintSolverData, timestep, taskData);

            // A task reached from FmPostIslandSolveUpdatePositionsPlasticityFracture will update the global progress and delete task data
            //taskData->WorkItemFinished(taskData);
#else
            FmPostIslandSolveUpdatePositionsPlasticityFracture(scene, &tetMesh, &constraintIsland, *constraintSolverData, timestep, nullptr);
#endif
        }
    }

    FM_ASYNC_TASK_COUNTED_WITH_DELETE(FmTaskFuncRbApplySolveDeltasAndTestSleeping, FmTaskDataApplySolveDeltasAndTestSleeping)
    {
        (void)inTaskEndIndex;
        FM_TRACE_SCOPED_EVENT("IslandSolveRbApplySolveDeltas");

        FmTaskDataApplySolveDeltasAndTestSleeping* taskData = (FmTaskDataApplySolveDeltasAndTestSleeping*)inTaskData;

        if (taskData->numRigidBodies == 0)
            return;

        FmScene* scene = taskData->scene;
        FmConstraintSolverData* constraintSolverData = taskData->constraintSolverData;
        const FmConstraintSolverBuffer& constraintSolverBuffer = *scene->constraintSolverBuffer;
        FmConstraintIsland& constraintIsland = *taskData->constraintIsland;

        bool rigidBodiesExternal = taskData->rigidBodiesExternal;
        float timestep = taskData->timestep;

        uint beginIdx, endIdx;
        FmGetIndexRangeEvenDistribution(&beginIdx, &endIdx, (uint)inTaskBeginIndex, taskData->numRigidBodyTasks, taskData->numRigidBodies);

        bool bodiesAreStatic = true;
        uint numFixedPoints = 0;

        for (uint islandRbIdx = beginIdx; islandRbIdx < endIdx; islandRbIdx++)
        {
            uint objectId = constraintIsland.rigidBodyIds[islandRbIdx];
            FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, objectId);

            // Apply corrections from constraint/stabilization solves
            if (islandRbIdx < constraintIsland.numRigidBodiesInFEMSolve)
            {
                if (FM_NOT_SET(rigidBody.flags, FM_OBJECT_FLAG_KINEMATIC))
                {
                    uint stateOffset = FmGetRigidBodySolverOffsetById(constraintSolverBuffer, objectId);
                    FmVector3 deltaVel = FmVector3(constraintSolverData->deltaVel[stateOffset]);
                    FmVector3 deltaAngVel = FmVector3(constraintSolverData->deltaVel[stateOffset+1]);
#if FM_CONSTRAINT_STABILIZATION_SOLVE
                    FmVector3 deltaPos = FmVector3(constraintSolverData->deltaPos[stateOffset]);
                    FmVector3 deltaAngPos = FmVector3(constraintSolverData->deltaPos[stateOffset + 1]);

                    rigidBody.state.pos = rigidBody.state.pos + deltaPos;
                    rigidBody.state.quat = FmIntegrateQuat(rigidBody.state.quat, deltaAngPos, 1.0f);
#endif

                    rigidBody.state.vel = rigidBody.state.vel + deltaVel;
                    rigidBody.state.angVel = rigidBody.state.angVel + deltaAngVel;
                    rigidBody.deltaPos = FmVector3(0.0f);
                    rigidBody.deltaAngPos = FmVector3(0.0f);
                    rigidBody.deltaVel = FmVector3(0.0f);
                    rigidBody.deltaAngVel = FmVector3(0.0f);
                    rigidBody.flags |= (FM_OBJECT_FLAG_POS_CHANGED | FM_OBJECT_FLAG_VEL_CHANGED);
                }
            }

            // Update velocity stats and test sleeping 
            float maxSpeedThreshold = rigidBody.sleepMaxSpeedThreshold;
            float avgSpeedThreshold = rigidBody.sleepAvgSpeedThreshold;
            uint stableCount = rigidBody.sleepStableCount;

            float speedBound = length(rigidBody.state.vel) + length(rigidBody.state.angVel) * rigidBody.maxRadius;

            FmUpdateVelStats(&rigidBody.velStats, speedBound, 1000);

            // Count fixed and zero-velocity elements in island
            if (FM_IS_SET(rigidBody.flags, FM_VERT_FLAG_KINEMATIC) && FmIsZero(rigidBody.state.vel) && FmIsZero(rigidBody.state.angVel))
            {
                numFixedPoints++;
            }

            if (FM_IS_SET(rigidBody.flags, FM_OBJECT_FLAG_SLEEPING_DISABLED)
                || !FmCheckStable(&rigidBody.velStats, maxSpeedThreshold, avgSpeedThreshold, stableCount))
            {
                bodiesAreStatic = false;
            }

            if (!rigidBodiesExternal)
            {
                // Integrate rigid body state
                FmStepState(rigidBody.state, timestep);
            }
        }

        // Update constraint island sleeping values
        if (numFixedPoints > 0)
        {
            FmAtomicAdd(&constraintIsland.numFixedPoints.val, numFixedPoints);
        }

        if (!bodiesAreStatic)
        {
            if (FmAtomicRead(&constraintIsland.isIslandStable.val) == 1)
            {
                FmAtomicWrite(&constraintIsland.isIslandStable.val, 0);
            }
        }
    }

    FM_ASYNC_TASK_COUNTED_WITH_DELETE(FmTaskFuncDestroyIslandDependencyGraph, FmTaskDataApplySolveDeltasAndTestSleeping)
    {
        FM_TRACE_SCOPED_EVENT("IslandSolveDestroyDependencyGraph");

        (void)inTaskBeginIndex;
        (void)inTaskEndIndex;
        (void)inTaskData;

#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
        FmTaskDataApplySolveDeltasAndTestSleeping* taskData = (FmTaskDataApplySolveDeltasAndTestSleeping*)inTaskData;

        if (taskData->constraintSolverData->taskGraph)
        {
            FmDestroyConstraintSolveTaskGraph(taskData->constraintSolverData->taskGraph);
            taskData->constraintSolverData->taskGraph = nullptr;
        }
#endif
    }

    FM_ASYNC_TASK(FmTaskFuncApplySolveDeltasAndTestSleeping)
    {
        FmTaskDataApplySolveDeltasAndTestSleeping* taskData = (FmTaskDataApplySolveDeltasAndTestSleeping*)inTaskData;

        int32_t taskBeginIndex = inTaskBeginIndex;
        int32_t taskEndIndex = inTaskEndIndex;

        if (taskBeginIndex < (int32_t)taskData->numTetMeshes)
        {
            FmTaskFuncMeshApplySolveDeltasAndTestSleeping(inTaskData, taskBeginIndex, taskEndIndex);
        }
        else if (taskBeginIndex < (int32_t)(taskData->numTetMeshes + taskData->numRigidBodyTasks))
        {
            taskBeginIndex -= taskData->numTetMeshes;
            FmTaskFuncRbApplySolveDeltasAndTestSleeping(inTaskData, taskBeginIndex, taskBeginIndex + 1);
        }
        else
        {
            FmTaskFuncDestroyIslandDependencyGraph(inTaskData, 0, 1);
        }
    }

    int32_t FmGrainFuncPartitionApplySolveDeltasAndTestSleeping(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        FmTaskDataApplySolveDeltasAndTestSleeping* taskData = (FmTaskDataApplySolveDeltasAndTestSleeping*)inTaskData;

        FmScene* scene = taskData->scene;
        FmConstraintIsland* constraintIsland = taskData->constraintIsland;
        uint numTetMeshes = taskData->numTetMeshes;

        const uint minVerts = 256;

        uint numVerts = 0;
        uint objectIndex = (uint)inTaskBeginIndex;
        uint taskBeginIndex = (uint)inTaskBeginIndex;
        uint taskEndIndex = (uint)inTaskEndIndex;

        if (objectIndex < numTetMeshes)
        {
            while (numVerts < minVerts
                && objectIndex < taskEndIndex
                && objectIndex < numTetMeshes)
            {
                uint islandObjectId = constraintIsland->tetMeshIds[objectIndex];

                FmMpcgSolverData& meshData = *FmGetSolverDataById(*scene, islandObjectId);
                numVerts += meshData.A.numRows;
                objectIndex++;
            }

            return objectIndex - taskBeginIndex;
        }
        else
        {
            return 1;
        }
    }

    void FmApplyConstraintSolveDeltasAndTestSleeping(
        FmScene* scene,
        FmConstraintSolverData* constraintSolverData,
        FmConstraintIsland& constraintIsland,
        TLTaskFuncCallback followTaskFunc,
        void* followTaskData)
    {
        constraintIsland.isIslandStable.val = 1;
        constraintIsland.numFixedPoints.val = 0;

        float timestep = scene->params.timestep;
        bool rigidBodiesExternal = scene->params.rigidBodiesExternal;

        uint numTetMeshes = constraintIsland.numTetMeshes;
        uint numRigidBodies = constraintIsland.numRigidBodiesConnected;
        uint rigidBodyBatchSize = FM_RIGID_BODY_APPLY_DELTAS_SLEEPING_BATCH_SIZE;
        uint numRigidBodyTasks = FmGetNumTasksLimited(numRigidBodies, rigidBodyBatchSize, scene->params.numThreads*4);

        uint numItems = numTetMeshes + numRigidBodyTasks;

#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
        // Extra task will destroy the dependency graph during pos/vel and sleeping updates
        numItems++;
#endif

#if FM_ASYNC_THREADING
        FM_ASSERT(followTaskFunc);

        if (followTaskFunc)
        {
            FmTaskDataApplySolveDeltasAndTestSleeping* taskData = new FmTaskDataApplySolveDeltasAndTestSleeping(scene, constraintSolverData, &constraintIsland, 
                numTetMeshes, numRigidBodies, numRigidBodyTasks,
                timestep, rigidBodiesExternal);

            int32_t taskIdx = (int32_t)constraintIsland.islandId;

            // NOTE: taskData used in asynchronous tasks after the parallel-for, must be managed by task
            TLTask followTask(followTaskFunc, followTaskData, taskIdx, taskIdx + 1);
            taskData->Init(numItems, followTask);

            TLParallelForAsync(FmTaskFuncApplySolveDeltasAndTestSleeping, taskData, 0, numItems,
                TLParallelForOptions::GrainFunc(FmGrainFuncPartitionApplySolveDeltasAndTestSleeping));
        }
        else
#endif
#if !FM_ASYNC_THREADING
        {
            (void)followTaskFunc;
            (void)followTaskData;

            FmTaskDataApplySolveDeltasAndTestSleeping taskData(scene, constraintSolverData, &constraintIsland, 
                numTetMeshes, numRigidBodies, numRigidBodyTasks,
                timestep, rigidBodiesExternal);

            TLParallelFor(FmTaskFuncApplySolveDeltasAndTestSleeping, &taskData, 0, numItems,
                TLParallelForOptions::GrainFunc(FmGrainFuncPartitionApplySolveDeltasAndTestSleeping));

            if (FmAtomicRead(&constraintIsland.isIslandStable.val) == 1
                && (constraintIsland.numFixedAttachments + FmAtomicRead(&constraintIsland.numFixedPoints.val)) >= 3)
            {
                FmMarkIslandForSleeping(scene, constraintIsland.islandId);
            }
        }
#else
        {
            FmTaskDataApplySolveDeltasAndTestSleeping taskData(scene, constraintSolverData, &constraintIsland, 
                numTetMeshes, numRigidBodies, numRigidBodyTasks,
                timestep, rigidBodiesExternal);

            for (int32_t taskIdx = 0; taskIdx < (int32_t)numItems; taskIdx++)
            {
                FmTaskFuncApplySolveDeltasAndTestSleeping(&taskData, taskIdx, taskIdx + 1);
            }

            if (FmAtomicRead(&constraintIsland.isIslandStable.val) == 1
                && (constraintIsland.numFixedAttachments + FmAtomicRead(&constraintIsland.numFixedPoints.val)) >= 3)
            {
                FmMarkIslandForSleeping(scene, constraintIsland.islandId);
            }
        }
#endif
    }

    // Test sleeping and update the velocity stats.
    // For islands with constraints, these steps are handled instead by FmPostIslandSolveUpdatePositionsPlasticityFracture.
    void FmTestSleepingAndUpdateStats(
        FmScene* scene,
        const FmConstraintIsland& constraintIsland)
    {
        bool islandStable = true;

        uint numFixedPoints = 0;

        for (uint islandMeshIdx = 0; islandMeshIdx < constraintIsland.numTetMeshes; islandMeshIdx++)
        {
            FmTetMesh& tetMesh = *FmGetTetMeshPtrById(*scene, constraintIsland.tetMeshIds[islandMeshIdx]);

            float maxSpeedThreshold = tetMesh.sleepMaxSpeedThreshold;
            float avgSpeedThreshold = tetMesh.sleepAvgSpeedThreshold;
            uint stableCount = tetMesh.sleepStableCount;

            for (uint i = 0; i < tetMesh.numVerts; i++)
            {
                FmVector3 vel = tetMesh.vertsVel[i];
                FmUpdateVelStats(&tetMesh.velStats, length(vel), 1000);

                if (FM_IS_SET(tetMesh.vertsFlags[i], FM_VERT_FLAG_KINEMATIC) && FmIsZero(vel))
                {
                    numFixedPoints++;
                }
            }

            if (FM_IS_SET(tetMesh.flags, FM_OBJECT_FLAG_SLEEPING_DISABLED)
                || !FmCheckStable(&tetMesh.velStats, maxSpeedThreshold, avgSpeedThreshold, stableCount))
            {
                islandStable = false;
            }
        }

        if (scene->rigidBodies)
        {
            for (uint islandRbIdx = 0; islandRbIdx < constraintIsland.numRigidBodiesConnected; islandRbIdx++)
            {
                FmRigidBody& rigidBody = *FmGetRigidBodyPtrById(*scene, constraintIsland.rigidBodyIds[islandRbIdx]);

                float maxSpeedThreshold = rigidBody.sleepMaxSpeedThreshold;
                float avgSpeedThreshold = rigidBody.sleepAvgSpeedThreshold;
                uint stableCount = rigidBody.sleepStableCount;

                float speedBound = length(rigidBody.state.vel) + length(rigidBody.state.angVel) * rigidBody.maxRadius;

                FmUpdateVelStats(&rigidBody.velStats, speedBound, 1000);

                if (FM_IS_SET(rigidBody.flags, FM_VERT_FLAG_KINEMATIC) && FmIsZero(rigidBody.state.vel) && FmIsZero(rigidBody.state.angVel))
                {
                    numFixedPoints++;
                }

                if (FM_IS_SET(rigidBody.flags, FM_OBJECT_FLAG_SLEEPING_DISABLED)
                    || !FmCheckStable(&rigidBody.velStats, maxSpeedThreshold, avgSpeedThreshold, stableCount))
                {
                    islandStable = false;
                }
            }
        }

        if (islandStable && (numFixedPoints + constraintIsland.numFixedAttachments) >= 3)  // Require 3 constraints or vertices/bodies that are kinematic and not moving.
        {
            FmMarkIslandForSleeping(scene, constraintIsland.islandId);
        }
    }

}