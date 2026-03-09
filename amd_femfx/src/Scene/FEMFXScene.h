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
// FEM scene definition
//---------------------------------------------------------------------------------------
#pragma once

#include "AMD_FEMFX.h"
#include "FEMFXFreeIds.h"
#include "FEMFXTetMesh.h"
#include "FEMFXRigidBody.h"
#include "FEMFXConstraintsBuffer.h"
#include "FEMFXConstraintIslands.h"

namespace AMD
{
    struct FmConstraintSolverBuffer;
    struct FmThreadTempMemoryBuffer;
    struct FmMpcgSolverData;
    struct FmConstraintIsland;
    struct FmSleepingConstraintIsland;

    // Structure defining all data for simulation scene
    struct FmScene
    {
        size_t                     bufferNumBytes = 0;

        // 64-byte alignment required
        FmConstraintsBuffer*       constraintsBuffer = nullptr;          // Pointer to buffer of contacts/constraints and connected component data for scene
        FmConstraintSolverBuffer*  constraintSolverBuffer = nullptr;     // Pointer to constraint solver buffers for scene
        FmThreadTempMemoryBuffer*  threadTempMemoryBuffer = nullptr;     // Pointer to temporary memory used in finding contacts

        FmTetMeshBuffer**          tetMeshBuffers = nullptr;             // Array of pointers to FmTetMesh buffers

        uint*                      awakeTetMeshIds = nullptr;            // Ids of awake tet meshes in current simulation step (that are not SIMULATION_DISABLED)
        uint*                      awakeRigidBodyIds = nullptr;          // Ids of awake rigid bodies in current simulation step (that are not SIMULATION_DISABLED)

        uint*                      sleepingTetMeshIds = nullptr;         // Ids of sleeping tet meshes in current simulation step (that are not SIMULATION_DISABLED); points into awakeTetMeshIds array
        uint*                      sleepingRigidBodyIds = nullptr;       // Ids of sleeping rigid bodies in current simulation step (that are not SIMULATION_DISABLED); points into awakeRigidBodyIds array

        FmTetMesh**                tetMeshPtrFromId = nullptr;           // Map from FmTetMesh::objectId to tet mesh pointer
        uint*                      tetMeshIdxFromId = nullptr;           // Map from FmTetMesh::objectId to index in awake ids list
        uint*                      rigidBodyIdxFromId = nullptr;         // Map from FmRigidBody::objectId to index in awake ids list

        FmFreeIds                  freeTetMeshIds;                       // Array of free mesh ids
        FmFreeIds                  freeRigidBodyIds;                     // Array of free rigid body ids

        FmRigidBody**              rigidBodies = nullptr;                // Array of pointers to rigid bodies

        uint                       numRigidBodySlots = 0;                // Number of rigid body slots used, some of which may be marked deleted
        uint                       numAwakeRigidBodies = 0;              // Number of awake rigid bodies simulated in current step
        uint                       numAwakenedRigidBodies = 0;           // Number of rigid bodies added to ids list on waking
        uint                       maxRigidBodies = 0;

        uint                       numTetMeshBufferSlots = 0;            // Number of entries in tetMeshBuffers array (some of which may be nullptr if removed)
        uint                       numAwakeTetMeshes = 0;                // Number of awake tet meshes simulated in current step
        uint                       numAwakenedTetMeshes = 0;             // Number of tet meshes added to ids list on waking
        uint                       numTetMeshesTotal = 0;                // Number of tet meshes created in all tet mesh buffers, must not exceed maxTetMeshes
        uint                       maxTetMeshBuffers = 0;
        uint                       maxTetMeshes = 0;
        uint                       maxSceneVerts = 0;
        uint                       maxTetMeshBufferFeatures = 0;

        uint                       numSleepingTetMeshes = 0;             // Number of sleeping tet meshes, placed at end awakeTetMeshIds array
        uint                       numSleepingRigidBodies = 0;           // Number of sleeping rigid bodies, placed at end awakeRigidBodyIds array

        // Scene callbacks
        TLTaskFuncCallback              postUpdateSceneCallback = nullptr;
        void*                           postUpdateSceneData = nullptr;
        FmCallbackSurfaceCollision      surfaceCollisionCallback = nullptr;   // If non-null, is called to add contacts between tet mesh vertcies and collision surface
        FmCallbackNotifyIslandWaking    islandWakingCallback = nullptr;
        FmCallbackNotifyIslandSleeping  islandSleepingCallback = nullptr;
        void*                           rigidBodiesUserData = nullptr;

        FmSceneControlParams       params;
        FmCollisionReport          collisionReport;            // Set buffers and parameters here to enable reporting of contacts
        FmFractureReport           fractureReport;             // Set buffers and parameters here to enable reporting of fractured objects
        FmWarningsReport           warningsReport;
    };

    // Initialize empty scene
    static inline void FmInitScene(FmScene* scene)
    {
        scene->bufferNumBytes = 0;
        scene->tetMeshBuffers = nullptr;
        scene->awakeTetMeshIds = nullptr;
        scene->awakeRigidBodyIds = nullptr;
        scene->tetMeshPtrFromId = nullptr;
        scene->tetMeshIdxFromId = nullptr;
        scene->rigidBodyIdxFromId = nullptr;
        FmInitFreeIds(&scene->freeTetMeshIds);
        FmInitFreeIds(&scene->freeRigidBodyIds);
        scene->constraintsBuffer = nullptr;
        scene->constraintSolverBuffer = nullptr;
        scene->threadTempMemoryBuffer = nullptr;
        scene->rigidBodies = nullptr;
        scene->numRigidBodySlots = 0;
        scene->numAwakeRigidBodies = 0;
        scene->numAwakenedRigidBodies = 0;
        scene->maxRigidBodies = 0;
        scene->numTetMeshBufferSlots = 0;
        scene->numAwakeTetMeshes = 0;
        scene->numAwakenedTetMeshes = 0;
        scene->numTetMeshesTotal = 0;
        scene->maxTetMeshBuffers = 0;
        scene->maxTetMeshes = 0;
        scene->maxSceneVerts = 0;
        scene->maxTetMeshBufferFeatures = 0;
        scene->sleepingTetMeshIds = nullptr;
        scene->sleepingRigidBodyIds = nullptr;
        scene->numSleepingTetMeshes = 0;
        scene->numSleepingRigidBodies = 0;
        scene->postUpdateSceneCallback = nullptr;
        scene->postUpdateSceneData = nullptr;
        scene->surfaceCollisionCallback = nullptr;
        scene->islandWakingCallback = nullptr;
        scene->islandSleepingCallback = nullptr;
        scene->rigidBodiesUserData = nullptr;
        scene->params = FmSceneControlParams();
        scene->collisionReport = FmCollisionReport();
        scene->fractureReport = FmFractureReport();
        scene->warningsReport = FmWarningsReport();
    }

    // Get total bytes needed for scene struct and its arrays.
    size_t FmGetSceneSize(const FmSceneSetupParams& params);

    // Suballocate memory from pBuffer to create scene and arrays.
    // NOTE: pBuffer must be 64-bit aligned memory
    FmScene* FmSetupScene(const FmSceneSetupParams& params, uint8_t* pBuffer64ByteAligned, size_t bufferNumBytes);

    // Add rigid body to the scene and return an id.
    uint FmAddRigidBodyToScene(FmScene* scene, FmRigidBody* inRigidBody);

    // Remove rigid body with the given id.
    void FmRemoveRigidBodyFromScene(FmScene* scene, uint rigidBodyId);

    static FM_FORCE_INLINE FmTetMesh* FmGetTetMeshPtrById(const FmScene& scene, uint objectId)
    {
        FM_ASSERT(objectId < scene.maxTetMeshes);
        return scene.tetMeshPtrFromId[objectId];
    }

    static FM_FORCE_INLINE void FmSetTetMeshPtrById(FmScene* scene, uint objectId, FmTetMesh* tetMesh)
    {
        FM_ASSERT(objectId < scene->maxTetMeshes);
        scene->tetMeshPtrFromId[objectId] = tetMesh;
    }

    static FM_FORCE_INLINE uint FmGetTetMeshIdxById(const FmScene& scene, uint objectId)
    {
        FM_ASSERT(objectId < scene.maxTetMeshes);
        return scene.tetMeshIdxFromId[objectId];
    }

    static FM_FORCE_INLINE void FmSetTetMeshIdxById(FmScene* scene, uint objectId, uint idx)
    {
        FM_ASSERT(objectId < scene->maxTetMeshes);
        scene->tetMeshIdxFromId[objectId] = idx;
    }

    static FM_FORCE_INLINE FmMpcgSolverData* FmGetSolverDataById(const FmScene& scene, uint objectId)
    {
        return FmGetTetMeshPtrById(scene, objectId)->solverData;
    }

    static FM_FORCE_INLINE FmRigidBody* FmGetRigidBodyPtrById(const FmScene& scene, uint objectId)
    {
        FM_ASSERT(scene.rigidBodies != nullptr && (objectId & ~FM_RB_FLAG) < scene.maxRigidBodies);
        return scene.rigidBodies[objectId & ~FM_RB_FLAG];
    }

    static FM_FORCE_INLINE uint FmGetRigidBodyIdxById(const FmScene& scene, uint objectId)
    {
        FM_ASSERT(scene.rigidBodies != nullptr && (objectId & ~FM_RB_FLAG) < scene.maxRigidBodies);
        return scene.rigidBodyIdxFromId[objectId & ~FM_RB_FLAG];
    }

    static FM_FORCE_INLINE void FmSetRigidBodyIdxById(FmScene* scene, uint objectId, uint idx)
    {
        FM_ASSERT(scene->rigidBodies != nullptr && (objectId & ~FM_RB_FLAG) < scene->maxRigidBodies);
        scene->rigidBodyIdxFromId[objectId & ~FM_RB_FLAG] = idx;
    }

    static FM_FORCE_INLINE FmConstraintIsland* FmGetConstraintIslandById(const FmScene& scene, uint islandId)
    {
        const FmConstraintsBuffer& constraintsBuffer = *scene.constraintsBuffer;
        uint islandIdx = islandId;
        FM_ASSERT(islandIdx < constraintsBuffer.numConstraintIslands);
        return &constraintsBuffer.constraintIslands[islandIdx];
    }

    static FM_FORCE_INLINE FmSleepingConstraintIsland* FmGetSleepingConstraintIslandById(const FmScene& scene, uint islandId)
    {
        const FmConstraintsBuffer& constraintsBuffer = *scene.constraintsBuffer;
        uint sleepingIslandIdx = constraintsBuffer.sleepingIslandIdToArrayIdx[islandId];
        FM_ASSERT(sleepingIslandIdx != FM_INVALID_ID);
        FM_ASSERT(sleepingIslandIdx < constraintsBuffer.numSleepingConstraintIslands);
        return &constraintsBuffer.sleepingConstraintIslands[sleepingIslandIdx];
    }

    // Remove id of tet mesh from either the scene awakeTetMeshIds or sleepingTetMeshIds, depending on FM_OBJECT_FLAG_SLEEPING
    // Assumes index of id is stored in FmTetMesh::idx
    void FmRemoveIdFromSceneAwakeTetMeshIds(FmScene* scene, uint objectId);
    void FmRemoveIdFromSceneSleepingTetMeshIds(FmScene* scene, uint objectId);

    // Remove id of rigid body from either the scene rigidBodyIds or sleepingRigidBodyIds, depending on FM_OBJECT_FLAG_SLEEPING
    // Assumes index of id is stored in FmRigidBody::idx
    void FmRemoveIdFromSceneAwakeRigidBodyIds(FmScene* scene, uint objectId);
    void FmRemoveIdFromSceneSleepingRigidBodyIds(FmScene* scene, uint objectId);

    // Add id of tet mesh to awakeTetMeshIds or sleepingTetMeshIds.
    // Sets FmTetMesh::idx to index of id.
    void FmAddIdToSceneAwakeTetMeshIds(FmScene* scene, uint objectId);
    void FmAddIdToSceneSleepingTetMeshIds(FmScene* scene, uint objectId);

    // Add id of rigid body to rigidBodyIds.
    // Sets FmRigidBody::idx to index of id.
    void FmAddIdToSceneAwakeRigidBodyIds(FmScene* scene, uint objectId);
    void FmAddIdToSceneSleepingRigidBodyIds(FmScene* scene, uint objectId);

    // Remove tet mesh id from its current island, either awake or sleeping
    void FmRemoveTetMeshIdFromIsland(FmScene* scene, uint objectId);

    // Remove rigid body id from its current island, either awake or sleeping
    void FmRemoveRigidBodyIdFromIsland(FmScene* scene, uint objectId);

    // Add a constraint to the scene; Returns an id, or if maximum constraints created, FM_INVALID_ID
    uint FmAddGlueConstraintToScene(FmScene* scene, const FmGlueConstraint& inGlueConstraint);

    // Add a constraint to the scene; Returns an id, or if maximum constraints created, FM_INVALID_ID
    uint FmAddPlaneConstraintToScene(FmScene* scene, const FmPlaneConstraint& inPlaneConstraint);

    // Add a constraint to the scene; Returns an id, or if maximum constraints created, FM_INVALID_ID
    uint FmAddRigidBodyAngleConstraintToScene(FmScene* scene, const FmRigidBodyAngleConstraint& rigidBodyAngleConstraint);

    // Get glue constraint, or nullptr if invalid id
    FmGlueConstraint* FmGetGlueConstraint(const FmScene& scene, uint glueConstraintId);

    // Get plane constraint, or nullptr if invalid id
    FmPlaneConstraint* FmGetPlaneConstraint(const FmScene& scene, uint planeConstraintId);

    // Get rigid body angle constraint, or nullptr if invalid id
    FmRigidBodyAngleConstraint* FmGetRigidBodyAngleConstraint(const FmScene& scene, uint rigidBodyAngleConstraintId);

    // Compute the relative angular velocity of rigid bodies around the hinge axis.
    float FmComputeHingeRelAngVel(const FmScene& scene, const FmRigidBodyAngleConstraint& constraint);

    void FmDisableObjectSimulation(FmScene* scene, uint objectId);


}