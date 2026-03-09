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
// Basic sample / viewer for FEM content, using GLFW
// Scene chosen by #defines in TestScenes.h
//---------------------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <string>
#if defined(_MSC_VER)
#include <Windows.h>
#endif

// GLFW includes
#if defined(_MSC_VER)
#pragma warning(push, 0)
#endif
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#define _VECTORMATH_DEBUG // For print functions
#include "ViewerCamera.h"
#include "TestScenes.h"
#include "FEMFXInternal.h"
#include <vector>

using namespace AMD;

// Config settings
#define DRAW_ROTATIONS 0
#define DRAW_DIST_CONTACTS 0
#define DRAW_GLUE 0
#define DRAW_FRACTURE_CONTACTS 0
#define DRAW_VOL_CONTACTS 0
#define DRAW_VOL_CONTACT_FRAME 0
#define DRAW_MESH_BVH 0
#define DRAW_BROAD_PHASE_BVH 0
#define DRAW_PARTITION_BVH   0
#define DRAW_GROUND 1

#define TRACE_TEST 1 && !FIX_INITIAL_CONDITIONS

#if PERF_TEST
#define PERF_TEST_MODE_MAX_THREADS                     0       // run max threads
#define PERF_TEST_MODE_MAX_TO_MIN_THREADS_DECREMENTING 1       // run from max to min threads, by decrementing
#define PERF_TEST_MODE_MAX_TO_MIN_THREADS_HALVING      2       // run from max to min threads, by halving

static int gPerfTestMode = PERF_TEST_MODE_MAX_THREADS;
static int gPerfTestMaxThreads = 1;   // initial number of threads
static int gPerfTestMinThreads = 1;
static int gPerfTestNumSeeds = 1;     // num different seed settings to use for random variatio
#endif

static std::string gTimingsPath(".");

// Global settings and state
static ViewerCamera gViewerCam;

static int gNumThreads = 1;
static int gRandomSeed = 0;

static int gFrameCounter = 0;

#if TRACE_TEST
static bool gRunning = true;
#else
static bool gRunning = false;
#endif
static bool gStepForward = false;
static bool gStepBackward = false;
static int gTargetFrame = 0;

static const float gTimestep = (1.0f / 60.0f);

static bool gDrawContacts = false;
static bool gDrawGlue = false;
static bool gDrawVolContacts = false;
static bool gDrawHighAspectRatioTets = false;
static bool gDrawKinematicTets = false;
static int  gDrawMode = 0;
static bool gDrawWireframe = false;
static bool gDrawFractureFlags = false;
static bool gDrawFracturableFaces = true;

static const int gWinWidth = 1280;
static const int gWinHeight = 720;
static const float gNearPlane = 0.1f;
static const float gFarPlane = 250.0f;
static const float gFovY = 45.0f;
static int gDragMode = -1;
static float gCursorX = 0;
static float gCursorY = 0;

namespace AMD
{
#if FM_DEBUG_MESHES
    extern FmTetMesh* gFEMFXPreConstraintSolveMeshes;
    extern FmRigidBody* gFEMFXPreConstraintSolveRigidBodies;
    extern uint gFEMFXNumPreConstraintSolveMeshes;
    extern uint gFEMFXNumPreConstraintSolveRigidBodies;
#endif
}

void PrintHelp()
{
    printf("===================================================================\n");
    printf("Controls:\n");
    printf("Left mouse drag : rotate the scene\n");
    printf("Middle mouse drag : pan\n");
    printf("Right mouse drag down / up : zoom in / out\n");
    printf("'h' : print help\n");
    printf("'p' : toggle paused / running\n");
    printf("'r' : reset simulation\n");
    printf("'q' : quit\n");
#if CARS_SCENE
    printf("'f' : fire projectile\n");
    printf("' ' : fire higher-speed projectile\n");
    printf("'l' : launch cars\n");
#endif
#if DRAW_DIST_CONTACTS
    printf("'c' : display distance contact debug\n");
#endif
#if DRAW_VOL_CONTACTS
    printf("'v' : display volume contact debug\n");
#endif
    printf("'d' : cycle drawing wireframe, fracturable, non-fracturable faces\n");
    printf("']' : step simulation forward\n");
#if FIX_INITIAL_CONDITIONS
    printf("'[' : step simulation backward\n");
#endif
    printf("===================================================================\n");
}

static inline float randfloat(uint seed)
{
    uint num = FmComputeHash(seed);
    return (float)(num % 0x7fff) / 0x7fff;
}

// Drawing functions
static void DrawCube(float r, float g, float b)
{
    GLfloat verts[] =
    {
        -1.0f, -1.0f, -1.0f,   1.0f, -1.0f, -1.0f,   1.0f, -1.0f,  1.0f,  -1.0f, -1.0f,  1.0f,
        -1.0f,  1.0f, -1.0f,  -1.0f,  1.0f,  1.0f,   1.0f,  1.0f,  1.0f,   1.0f,  1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f,  -1.0f,  1.0f, -1.0f,   1.0f,  1.0f, -1.0f,   1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f,   1.0f, -1.0f,  1.0f,   1.0f,  1.0f,  1.0f,  -1.0f,  1.0f,  1.0f,
        -1.0f, -1.0f, -1.0f,  -1.0f, -1.0f,  1.0f,  -1.0f,  1.0f,  1.0f,  -1.0f,  1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,   1.0f,  1.0f, -1.0f,   1.0f,  1.0f,  1.0f,   1.0f, -1.0f,  1.0f,
    };
    GLfloat normals[] =
    {
         0.0f, -1.0f,  0.0f,   0.0f, -1.0f,  0.0f,   0.0f, -1.0f,  0.0f,   0.0f, -1.0f,  0.0f,
         0.0f,  1.0f,  0.0f,   0.0f,  1.0f,  0.0f,   0.0f,  1.0f,  0.0f,   0.0f,  1.0f,  0.0f,
         0.0f,  0.0f, -1.0f,   0.0f,  0.0f, -1.0f,   0.0f,  0.0f, -1.0f,   0.0f,  0.0f, -1.0f,
         0.0f,  0.0f,  1.0f,   0.0f,  0.0f,  1.0f,   0.0f,  0.0f,  1.0f,   0.0f,  0.0f,  1.0f,
        -1.0f,  0.0f,  0.0f,  -1.0f,  0.0f,  0.0f,  -1.0f,  0.0f,  0.0f,  -1.0f,  0.0f,  0.0f,
         1.0f,  0.0f,  0.0f,   1.0f,  0.0f,  0.0f,   1.0f,  0.0f,  0.0f,   1.0f,  0.0f,  0.0f,
    };
    glColor3f(r, g, b);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, verts);
    glNormalPointer(GL_FLOAT, 0, normals);
    glDrawArrays(GL_QUADS, 0, 24);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
}

static void DrawTriangles(GLfloat* verts, GLfloat* normals, GLfloat* colors, int numVertices)
{
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, verts);
    glNormalPointer(GL_FLOAT, 0, normals);
    glColorPointer(3, GL_FLOAT, 0, colors);
    glDrawArrays(GL_TRIANGLES, 0, numVertices);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
}

static void DrawLines(GLfloat* verts, GLfloat* colors, int numVertices)
{
    glDisable(GL_LIGHTING);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, verts);
    glColorPointer(3, GL_FLOAT, 0, colors);
    glDrawArrays(GL_LINES, 0, numVertices);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glEnable(GL_LIGHTING);
}

static GLUquadric* gQuadric = nullptr;

#if DRAW_DIST_CONTACTS || DRAW_VOL_CONTACTS || DRAW_GLUE
static void DrawLine(const FmVector3& posA, const FmVector3& posB, float r, float g, float b)
{
    GLfloat verts[] =
    {
        posA.x, posA.y, posA.z,
        posB.x, posB.y, posB.z,
    };
    GLfloat colors[] =
    {
        r, g, b,
        r, g, b
    };
    DrawLines(verts, colors, 2);
}

static void DrawAxes(const FmVector3& pos, const FmVector3& xAxis, const FmVector3& yAxis, const FmVector3& zAxis)
{
    GLfloat verts[] =
    {
        pos.x, pos.y, pos.z,
        pos.x + xAxis.x, pos.y + xAxis.y, pos.z + xAxis.z,
        pos.x, pos.y, pos.z,
        pos.x + yAxis.x, pos.y + yAxis.y, pos.z + yAxis.z,
        pos.x, pos.y, pos.z,
        pos.x + zAxis.x, pos.y + zAxis.y, pos.z + zAxis.z,
    };
    GLfloat colors[] =
    {
        1.0, 0.0f, 0.0f,
        1.0, 0.0f, 0.0f,
        0.0, 1.0f, 0.0f,
        0.0, 1.0f, 0.0f,
        0.0, 0.0f, 1.0f,
        0.0, 0.0f, 1.0f
    };
    DrawLines(verts, colors, 6);
}

static void DrawSphere(FmVector3 pos, float radius, float r, float g, float b)
{
    if (gQuadric == nullptr)
    {
        gQuadric = gluNewQuadric();
    }
    glPushMatrix();
    glColor3f(r, g, b);
    glTranslatef(pos.x, pos.y, pos.z);
    gluSphere(gQuadric, radius, 10, 10);
    glPopMatrix();
}
#endif

static void DrawBox(float hx, float hy, float hz, float r, float g, float b)
{
    glEnable(GL_NORMALIZE);
    glPushMatrix();
    glScalef(hx, hy, hz);
    DrawCube(r, g, b);
    glPopMatrix();
    glDisable(GL_NORMALIZE);
}

#if DRAW_MESH_BVH || DRAW_BROAD_PHASE_BVH || DRAW_PARTITION_BVH
static void DrawAabb(FmAabb& aabb, float timestep, float r, float g, float b)
{
    FmVector3 pmin = aabb.pmin + aabb.vmin * timestep;
    FmVector3 pmax = aabb.pmax + aabb.vmax * timestep;

    glDisable(GL_LIGHTING);
    glDisable(GL_CULL_FACE);

    GLfloat verts[] =
    {
        pmin.x, pmin.y, pmin.z,  pmax.x, pmin.y, pmin.z,  pmin.x, pmin.y, pmax.z,  pmax.x, pmin.y, pmax.z,
        pmin.x, pmax.y, pmin.z,  pmax.x, pmax.y, pmin.z,  pmin.x, pmax.y, pmax.z,  pmax.x, pmax.y, pmax.z,
        pmin.x, pmin.y, pmin.z,  pmin.x, pmax.y, pmin.z,  pmin.x, pmin.y, pmax.z,  pmin.x, pmax.y, pmax.z,
        pmax.x, pmin.y, pmin.z,  pmax.x, pmax.y, pmin.z,  pmax.x, pmin.y, pmax.z,  pmax.x, pmax.y, pmax.z,
        pmin.x, pmin.y, pmin.z,  pmin.x, pmin.y, pmax.z,  pmin.x, pmax.y, pmin.z,  pmin.x, pmax.y, pmax.z,
        pmax.x, pmin.y, pmin.z,  pmax.x, pmin.y, pmax.z,  pmax.x, pmax.y, pmin.z,  pmax.x, pmax.y, pmax.z
    };
    GLfloat colors[] =
    {
        r, g, b,  r, g, b,  r, g, b,  r, g, b,
        r, g, b,  r, g, b,  r, g, b,  r, g, b,
        r, g, b,  r, g, b,  r, g, b,  r, g, b,
        r, g, b,  r, g, b,  r, g, b,  r, g, b,
        r, g, b,  r, g, b,  r, g, b,  r, g, b,
        r, g, b,  r, g, b,  r, g, b,  r, g, b
    };
    DrawLines(verts, colors, 24);
}

static void DrawBvhRecursive(FmBvh& hierarchy, uint nodeIndex, uint depth, uint minDepth, uint maxDepth, float timestep, float r, float g, float b)
{
    assert(nodeIndex < FmNumBvhNodes(hierarchy.numPrims));

    FmBvhNode& node = hierarchy.nodes[nodeIndex];
    bool isLeaf = node.left == node.right;

    if (depth > maxDepth)
        return;

    if (depth >= minDepth)
    {
        DrawAabb(hierarchy.nodes[nodeIndex].box, timestep, r, g, b);
    }

    if (!isLeaf)
    {
        DrawBvhRecursive(hierarchy, node.left, depth + 1, minDepth, maxDepth, timestep, r, g, b);
        DrawBvhRecursive(hierarchy, node.right, depth + 1, minDepth, maxDepth, timestep, r, g, b);
    }
}

static void DrawBvh(FmBvh& hierarchy, uint minDepth, uint maxDepth, float timestep, float r = 0.6f, float g = 0.0f, float b = 0.0f)
{
    if (hierarchy.numPrims)
    {
        DrawBvhRecursive(hierarchy, 0, 0, minDepth, maxDepth, timestep, r, g, b);
    }
}
#endif

#if DRAW_DIST_CONTACTS
static void DrawDistanceContact(FmDistanceContactPairInfo& contactPairInfo, FmDistanceContact& contact, float r = 1.0f, float g = 1.0f, float b = 1.0f)
{
    FmVector3 posA;

    if (contactPairInfo.objectIdA & FM_RB_FLAG)
    {
#if FM_DEBUG_MESHES
        uint rbIdxA = FmGetRigidBody(*gScene, contact.objectIdA)->idx;
        FmRigidBody& rigidBodyA = gFEMFXPreConstraintSolveRigidBodies[rbIdxA];
#else
        FmRigidBody& rigidBodyA = *FmGetRigidBody(*gScene, contactPairInfo.objectIdA);
#endif
        posA = rigidBodyA.state.pos + FmVector3(contact.comToPosA[0], contact.comToPosA[1], contact.comToPosA[2]);
    }
    else
    {
#if FM_DEBUG_MESHES
        uint meshIdxA = FmGetTetMesh(*gScene, contact.objectIdA)->idx;
        FmTetMesh& tetMeshA = gFEMFXPreConstraintSolveMeshes[meshIdxA];
#else
        FmTetMesh& tetMeshA = *FmGetTetMesh(*gScene, contactPairInfo.objectIdA);
#endif

        FmTetVertIds tetVertIdsA = FmGetTetVertIds(tetMeshA, contact.tetIdA);

        posA = FmInterpolate(contact.posBaryA,
            tetMeshA.vertsPos[tetVertIdsA.ids[0]],
            tetMeshA.vertsPos[tetVertIdsA.ids[1]],
            tetMeshA.vertsPos[tetVertIdsA.ids[2]],
            tetMeshA.vertsPos[tetVertIdsA.ids[3]]);
    }

    FmVector3 posB;

    if (contactPairInfo.objectIdB == FM_INVALID_ID)
    {
        posB = posA;
    }
    else if (contactPairInfo.objectIdB & FM_RB_FLAG)
    {
#if FM_DEBUG_MESHES
        uint rbIdxB = FmGetRigidBody(*gScene, contact.objectIdB)->idx;
        FmRigidBody& rigidBodyB = gFEMFXPreConstraintSolveRigidBodies[rbIdxB];
#else
        FmRigidBody& rigidBodyB = *FmGetRigidBody(*gScene, contactPairInfo.objectIdB);
#endif
        posB = rigidBodyB.state.pos + FmVector3(contact.comToPosB[0], contact.comToPosB[1], contact.comToPosB[2]);
    }
    else
    {
#if FM_DEBUG_MESHES
        uint meshIdxB = FmGetTetMesh(*gScene, contact.objectIdB)->idx;
        FmTetMesh& tetMeshB = gFEMFXPreConstraintSolveMeshes[meshIdxB];
#else
        FmTetMesh& tetMeshB = *FmGetTetMesh(*gScene, contactPairInfo.objectIdB);
#endif

        FmTetVertIds tetVertIdsB = FmGetTetVertIds(tetMeshB, contact.tetIdB);

        posB = FmInterpolate(contact.posBaryB,
            tetMeshB.vertsPos[tetVertIdsB.ids[0]],
            tetMeshB.vertsPos[tetVertIdsB.ids[1]],
            tetMeshB.vertsPos[tetVertIdsB.ids[2]],
            tetMeshB.vertsPos[tetVertIdsB.ids[3]]);
    }

    {
        const float vecScale = 0.25f;

        FmVector3 normal = contact.normal * vecScale;
        FmVector3 tangent1 = contact.tangent1 * vecScale;
        FmVector3 tangent2 = normalize(cross(normal, tangent1)) * vecScale;

        DrawSphere(posA, 0.01f, r, g, b);
        DrawSphere(posB, 0.01f, r, g, b);
        DrawAxes(posA, tangent1, tangent2, normal);
        DrawLine(posA, posB, 1.0f, 1.0f, 0.0f);
    }

}

static void DrawDistanceContacts(FmScene* scene)
{
    FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;
    uint numDistanceContacts = constraintsBuffer->numDistanceContacts.val;
    for (uint contactIdx = 0; contactIdx < numDistanceContacts; contactIdx++)
    {
        FmDistanceContactPairInfo& contactPairInfo = constraintsBuffer->distanceContactsPairInfo[contactIdx];
        FmDistanceContact& contact = constraintsBuffer->distanceContacts[contactIdx];
        DrawDistanceContact(contactPairInfo, contact);
    }
}
#endif

#if DRAW_VOL_CONTACTS
static void DrawVolumeContacts(FmScene* scene)
{
    FmConstraintsBuffer* constraintsBuffer = scene->constraintsBuffer;
    uint numVolumeContacts = constraintsBuffer->numVolumeContacts.val;
    for (uint contactIdx = 0; contactIdx < numVolumeContacts; contactIdx++)
    {
        FmVolumeContact& contact = constraintsBuffer->volumeContacts[contactIdx];

        FmVector3 posA = FmVector3(0.0f, 0.0f, 0.0f);
        for (uint vId = 0; vId < contact.numVolVertsA; vId++)
        {
            FmVector3 pos;
            if (contact.objectIdA & FM_RB_FLAG)
            {
#if FM_DEBUG_MESHES
                FmRigidBody& rigidBodyA = gFEMFXPreConstraintSolveRigidBodies[contact.objectIdA & ~FM_RB_FLAG];
#else
                FmRigidBody& rigidBodyA = *FmGetRigidBody(*gScene, contact.objectIdA);
#endif
                pos = rigidBodyA.state.pos +
                    constraintsBuffer->volumeContactVerts[contact.volVertsStartA + vId].centerToVert;
            }
            else
            {
#if FM_DEBUG_MESHES
                uint meshIdxA = FmGetTetMesh(*gScene, contact.objectIdA)->idx;
                FmTetMesh& tetMeshA = gFEMFXPreConstraintSolveMeshes[meshIdxA];
#else
                FmTetMesh& tetMeshA = *FmGetTetMesh(*gScene, contact.objectIdA);
#endif

                pos = tetMeshA.vertsPos[constraintsBuffer->volumeContactVerts[contact.volVertsStartA + vId].vertId];
            }

            posA += pos;

            FmVolumeContactVert& volVert = constraintsBuffer->volumeContactVerts[contact.volVertsStartA + vId];
            FmVector3 dVdp = volVert.dVdp;

            DrawSphere(pos, 0.02f, 1.0f, 1.0f, 0.0f);
            DrawLine(pos, pos + dVdp, 1.0f, 1.0f, 0.0f);
        }

        posA /= (float)contact.numVolVertsA;

#if DRAW_VOL_CONTACT_FRAME
        DrawSphere(posA, 0.01f, 1.0f, 1.0f, 0.0f);

        const float vecScale = 2.0f;
        FmVector3 normal = contact.normal * vecScale;
        FmVector3 tangent1 = contact.tangent1 * vecScale;
        FmVector3 tangent2 = normalize(cross(normal, tangent1)) * vecScale;

        DrawSphere(posA + normal, 0.01f, 0.0f, 0.0f, 1.0f);
        DrawAxes(posA, tangent1, tangent2, normal);
#endif

        if (contact.objectIdB != FM_INVALID_ID)
        {
            FmVector3 posB = FmVector3(0.0f, 0.0f, 0.0f);
            for (uint vId = 0; vId < contact.numVolVertsB; vId++)
            {
                FmVector3 pos;
                if (contact.objectIdB & FM_RB_FLAG)
                {
#if FM_DEBUG_MESHES
                    FmRigidBody& rigidBodyB = gFEMFXPreConstraintSolveRigidBodies[contact.objectIdB & ~FM_RB_FLAG];
#else
                    FmRigidBody& rigidBodyB = *FmGetRigidBody(*gScene, contact.objectIdB);
#endif
                    pos = rigidBodyB.state.pos +
                        constraintsBuffer->volumeContactVerts[contact.volVertsStartB + vId].centerToVert;
                }
                else
                {
#if FM_DEBUG_MESHES
                    uint meshIdxB = FmGetTetMesh(*gScene, contact.objectIdB)->idx;
                    FmTetMesh& tetMeshB = gFEMFXPreConstraintSolveMeshes[meshIdxB];
#else
                    FmTetMesh& tetMeshB = *FmGetTetMesh(*gScene, contact.objectIdB);
#endif

                    pos = tetMeshB.vertsPos[constraintsBuffer->volumeContactVerts[contact.volVertsStartB + vId].vertId];
                }

                posB += pos;

                FmVolumeContactVert& volVert = constraintsBuffer->volumeContactVerts[contact.volVertsStartB + vId];
                FmVector3 dVdp = volVert.dVdp * 1.0;

                DrawSphere(pos, 0.02f, 1.0f, 1.0f, 0.0f);
                DrawLine(pos, pos + dVdp, 1.0f, 1.0f, 0.0f);
            }

            posB /= (float)contact.numVolVertsB;
        }
    }
}
#endif

static void DrawTetWire(FmVector3 positions[4], float r, float g, float b)
{
    GLfloat verts[] =
    {
        positions[0].x, positions[0].y, positions[0].z,
        positions[1].x, positions[1].y, positions[1].z,
        positions[0].x, positions[0].y, positions[0].z,
        positions[2].x, positions[2].y, positions[2].z,
        positions[0].x, positions[0].y, positions[0].z,
        positions[3].x, positions[3].y, positions[3].z,
        positions[1].x, positions[1].y, positions[1].z,
        positions[2].x, positions[2].y, positions[2].z,
        positions[1].x, positions[1].y, positions[1].z,
        positions[3].x, positions[3].y, positions[3].z,
        positions[2].x, positions[2].y, positions[2].z,
        positions[3].x, positions[3].y, positions[3].z
    };
    GLfloat colors[] =
    {
        r, g, b,
        r, g, b,
        r, g, b,
        r, g, b,
        r, g, b,
        r, g, b,
        r, g, b,
        r, g, b,
        r, g, b,
        r, g, b,
        r, g, b,
        r, g, b
    };
    DrawLines(verts, colors, 12);
}

static void DrawTetWire(const FmTetMesh& tetMesh, uint tetId, float r, float g, float b)
{
    FmVector3 positions[4];

    FmTetVertIds tetVerts = tetMesh.tetsVertIds[tetId];

    positions[0] = tetMesh.vertsPos[tetVerts.ids[0]];
    positions[1] = tetMesh.vertsPos[tetVerts.ids[1]];
    positions[2] = tetMesh.vertsPos[tetVerts.ids[2]];
    positions[3] = tetMesh.vertsPos[tetVerts.ids[3]];

    DrawTetWire(positions, r, g, b);
}

static void SetColorById(float* r, float* g, float* b, uint id)
{
#if 1
    uint seed0 = id;
    uint seed1 = FmComputeHash(id);
    uint seed2 = FmComputeHash(seed1);

    *r = randfloat(seed0);
    *g = randfloat(seed1);
    *b = randfloat(seed2);
#endif
}

static void DrawTetMeshDisableFractureFlags(FmTetMesh& tetMesh, bool drawFracturable)
{
    std::vector<float> verts;
    std::vector<float> normals;
    std::vector<float> colors;

    float faceColors[4][3] = {
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f },
        { 1.0f, 1.0f, 0.0f },
    };

    uint numTets = tetMesh.numTets;
    for (uint tetId = 0; tetId < numTets; tetId++)
    {
        FmVector3 pos[4];

        FmTetVertIds tetVerts = tetMesh.tetsVertIds[tetId];

        pos[0] = tetMesh.vertsPos[tetVerts.ids[0]];
        pos[1] = tetMesh.vertsPos[tetVerts.ids[1]];
        pos[2] = tetMesh.vertsPos[tetVerts.ids[2]];
        pos[3] = tetMesh.vertsPos[tetVerts.ids[3]];

        for (uint faceId = 0; faceId < 4; faceId++)
        {
            FmFaceVertIds faceVertIds;
            FmGetFaceVertIds(&faceVertIds, faceId, tetVerts);

            FmFaceVertIds faceCorners;
            FmGetFaceTetCorners(&faceCorners, faceId);

            bool exteriorFace = FmIsExteriorFaceId(tetMesh.tetsFaceIncidentTetIds[tetId].ids[faceId]);

            bool canFracture = tetMesh.tetsFractureMaterialParams && ((tetMesh.tetsFlags[tetId] & (FM_TET_FLAG_FACE0_FRACTURE_DISABLED << faceId)) == 0);

            if (drawFracturable == canFracture && !exteriorFace)
            {
                FmVector3 pos0 = pos[faceCorners.ids[0]];
                FmVector3 pos1 = pos[faceCorners.ids[1]];
                FmVector3 pos2 = pos[faceCorners.ids[2]];
                FmVector3 normal = normalize(cross(pos1 - pos0, pos2 - pos0));
                for (int vId = 0; vId < 3; vId++)
                {
                    colors.push_back(faceColors[faceId][0]);
                    colors.push_back(faceColors[faceId][1]);
                    colors.push_back(faceColors[faceId][2]);
                    normals.push_back(normal.x);
                    normals.push_back(normal.y);
                    normals.push_back(normal.z);
                }
                verts.push_back(pos0.x);
                verts.push_back(pos0.y);
                verts.push_back(pos0.z);
                verts.push_back(pos1.x);
                verts.push_back(pos1.y);
                verts.push_back(pos1.z);
                verts.push_back(pos2.x);
                verts.push_back(pos2.y);
                verts.push_back(pos2.z);
            }
        }
    }

    DrawTriangles(verts.data(), normals.data(), colors.data(), static_cast<int>(verts.size()) / 3);
}

static void DrawTetMesh(FmTetMesh& tetMesh, float r, float g, float b)
{
    std::vector<float> verts;
    std::vector<float> normals;
    std::vector<float> colors;

    if (gDrawWireframe)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }
    else
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    // Draw exterior face polygons
    uint numFaces = tetMesh.numExteriorFaces;
    for (uint i = 0; i < numFaces; i++)
    {
        FmVector3 pos0, pos1, pos2, pos3, normal;

        FmExteriorFace& exteriorFace = tetMesh.exteriorFaces[i];

        FmTetVertIds tetVerts = tetMesh.tetsVertIds[exteriorFace.tetId];
        FmFaceVertIds faceVerts;
        FmGetFaceVertIds(&faceVerts, exteriorFace.faceId, tetVerts);

        pos0 = tetMesh.vertsPos[faceVerts.ids[0]];
        pos1 = tetMesh.vertsPos[faceVerts.ids[1]];
        pos2 = tetMesh.vertsPos[faceVerts.ids[2]];

        normal = normalize(cross(pos1 - pos0, pos2 - pos0));

        for (int vId = 0; vId < 3; vId++)
        {
            colors.push_back(r);
            colors.push_back(g);
            colors.push_back(b);
            normals.push_back(normal.x);
            normals.push_back(normal.y);
            normals.push_back(normal.z);
        }
        verts.push_back(pos0.x);
        verts.push_back(pos0.y);
        verts.push_back(pos0.z);
        verts.push_back(pos1.x);
        verts.push_back(pos1.y);
        verts.push_back(pos1.z);
        verts.push_back(pos2.x);
        verts.push_back(pos2.y);
        verts.push_back(pos2.z);
    }

    DrawTriangles(verts.data(), normals.data(), colors.data(), static_cast<int>(verts.size()) / 3);

#if DRAW_ROTATIONS
    {
        // Tet rotations
        glDisable(GL_LIGHTING);
        uint numTets = tetMesh.numTets;
        for (uint i = 0; i < numTets; i++)
        {
            FmVector3 pos0, pos1, pos2, pos3;

            FmVector3 positions[4];
            FmGetTetPositions(positions, tetMesh, i);
            pos0 = positions[0];
            pos1 = positions[1];
            pos2 = positions[2];
            pos3 = positions[3];

            FmMatrix3 rotation = tetMesh.tetsRotation[i];

            FmVector3 pos = 0.25f * (pos0 + pos1 + pos2 + pos3);

            DrawAxes(pos, rotation.col0, rotation.col1, rotation.col2);
        }
    }
#endif

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    if (gDrawHighAspectRatioTets)
    {
        // Draw tets with high aspect ratios
        uint numTets = tetMesh.numTets;
        for (uint i = 0; i < numTets; i++)
        {
            FmVector3 positions[4];

            FmTetVertIds tetVerts = tetMesh.tetsVertIds[i];

            positions[0] = tetMesh.vertsPos[tetVerts.ids[0]];
            positions[1] = tetMesh.vertsPos[tetVerts.ids[1]];
            positions[2] = tetMesh.vertsPos[tetVerts.ids[2]];
            positions[3] = tetMesh.vertsPos[tetVerts.ids[3]];

            float aspectRatio = FmComputeTetAspectRatio(positions);

            const float threshold = FM_MAX_TET_ASPECT_RATIO;

            if (aspectRatio > threshold)
            {
                DrawTetWire(tetMesh, i, 1.0f, 0.0f, 0.0f);
            }
        }
    }

    if (gDrawKinematicTets)
    {
        uint numTets = tetMesh.numTets;
        for (uint i = 0; i < numTets; i++)
        {
            if (FM_IS_SET(tetMesh.tetsFlags[i], FM_TET_FLAG_KINEMATIC))
            {
                DrawTetWire(tetMesh, i, 0.0f, 1.0f, 0.0f);
            }
        }
    }

    if (gDrawFractureFlags)
    {       
        DrawTetMeshDisableFractureFlags(tetMesh, gDrawFracturableFaces);
    }
}

static void DrawRigidBody(const FmRigidBody& rigidBody, float r, float g, float b)
{
    if (gDrawWireframe)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }
    else
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

#if 1
    FmMatrix4 boxMat = FmMatrix4(rigidBody.state.quat, rigidBody.state.pos);
    glPushMatrix();
    glMultMatrixf(&boxMat.col0.x);
    DrawBox(rigidBody.dims[0], rigidBody.dims[1], rigidBody.dims[2], r, g, b);
    glPopMatrix();
#else
    // Draw tet mesh collision obj
    FmTetMesh& tetMesh = *FmGetTetMesh(*(FmTetMeshBuffer*)rigidBody.collisionObj, 0);
    DrawTetMesh(tetMesh, r, g, b);
#endif

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

static void DrawScene()
{
#if FM_DEBUG_MESHES
    FmTetMesh* gDebugMeshes = gFEMFXPreConstraintSolveMeshes;
    FmRigidBody* gDebugRigidBodies = gFEMFXPreConstraintSolveRigidBodies;

    if (gDebugMeshes == nullptr || gDebugRigidBodies == nullptr)
        return;
#endif

#if DRAW_DIST_CONTACTS || DRAW_FRACTURE_CONTACTS
    if (gDrawContacts)
    {
        DrawDistanceContacts(gScene);

#if DRAW_FRACTURE_CONTACTS
        DrawFractureContacts(gScene);
#endif
    }
#endif

#if DRAW_GLUE
    if (gDrawGlue)
    {
        for (uint constraintIdx = 0; constraintIdx < gScene->constraintsBuffer->numGlueConstraints; constraintIdx++)
        {
            FmGlueConstraint& constraint = gScene->constraintsBuffer->glueConstraints[constraintIdx];

            float rA, gA, bA;
            float rB, gB, bB;

            if (constraint.bufferIdA == FM_INVALID_ID
                || constraint.objectIdA == FM_INVALID_ID
                || (((constraint.bufferIdA & FM_RB_FLAG) == 0) && !FmGetTetMeshBuffer(*gScene, constraint.bufferIdA))
                || (((constraint.bufferIdB & FM_RB_FLAG) == 0) && !FmGetTetMeshBuffer(*gScene, constraint.bufferIdB)))
            {
                continue;
            }

            FmVector3 posA;

            if ((constraint.objectIdA & FM_RB_FLAG)
                && gScene->rigidBodies)
            {
#if FM_DEBUG_MESHES
                FmRigidBody& rigidBodyA = gDebugRigidBodies[constraint.objectIdA & ~FM_RB_FLAG];
#else
                FmRigidBody& rigidBodyA = *FmGetRigidBody(*gScene, constraint.objectIdA);
#endif
                posA = rigidBodyA.state.pos + constraint.comToPosA;

                rA = 1.0f;
                gA = 0.0f;
                bA = 0.0f;
            }
            else
            {
#if FM_DEBUG_MESHES
                uint meshIdxA = FmGetTetMesh(*gScene, constraint.objectIdA)->idx;
                FmTetMesh& tetMeshA = gDebugMeshes[meshIdxA];
#else
                FmTetMesh& tetMeshA = *FmGetTetMesh(*gScene, constraint.objectIdA);
#endif

                FmTetVertIds tetVertIdsA = tetMeshA.tetsVertIds[constraint.tetIdA];

                posA = FmInterpolate(constraint.posBaryA,
                    tetMeshA.vertsPos[tetVertIdsA.ids[0]],
                    tetMeshA.vertsPos[tetVertIdsA.ids[1]],
                    tetMeshA.vertsPos[tetVertIdsA.ids[2]],
                    tetMeshA.vertsPos[tetVertIdsA.ids[3]]);

                DrawTetWire(tetMeshA, constraint.tetIdA, 1.0f, 1.0f, 0.0f);

                SetColorById(&rA, &gA, &bA, tetMeshA.objectId);
            }

            DrawSphere(posA, 0.02f, rA, gA, bA);

            FmVector3 posB;
            if (constraint.objectIdB == FM_INVALID_ID)
            {
                posB = FmVector3(constraint.posWorldB[0], constraint.posWorldB[1], constraint.posWorldB[2]);
                rB = 1.0f;
                gB = 1.0f;
                bB = 1.0f;
            }
            else if ((constraint.objectIdB & FM_RB_FLAG)
                && gScene->rigidBodies)
            {
#if FM_DEBUG_MESHES
                uint rbIdxB = FmGetRigidBody(*gScene, constraint.objectIdB)->idx;
                FmRigidBody& rigidBodyB = gDebugRigidBodies[rbIdxB];
#else
                FmRigidBody& rigidBodyB = *FmGetRigidBody(*gScene, constraint.objectIdB);
#endif
                posB = rigidBodyB.state.pos + constraint.comToPosB;

                rB = 0.8f;
                gB = 0.1f;
                bB = 0.1f;
            }
            else
            {
#if FM_DEBUG_MESHES
                uint meshIdxB = FmGetTetMesh(*gScene, constraint.objectIdB)->idx;
                FmTetMesh& tetMeshB = gDebugMeshes[meshIdxB];
#else
                FmTetMesh& tetMeshB = *FmGetTetMesh(*gScene, constraint.objectIdB);
#endif

                FmTetVertIds tetVertIdsB = tetMeshB.tetsVertIds[constraint.tetIdB];

                posB = FmInterpolate(constraint.posBaryB,
                    tetMeshB.vertsPos[tetVertIdsB.ids[0]],
                    tetMeshB.vertsPos[tetVertIdsB.ids[1]],
                    tetMeshB.vertsPos[tetVertIdsB.ids[2]],
                    tetMeshB.vertsPos[tetVertIdsB.ids[3]]);

                DrawTetWire(tetMeshB, constraint.tetIdB, 1.0f, 1.0f, 0.0f);

                SetColorById(&rB, &gB, &bB, tetMeshB.objectId);
            }

            DrawSphere(posB, 0.02f, rB, gB, bB);

            DrawLine(posB, posB + constraint.deltaPos, rB, gB, bB);
        }


        for (uint constraintIdx = 0; constraintIdx < gScene->constraintsBuffer->numPlaneConstraints; constraintIdx++)
        {
            FmPlaneConstraint& constraint = gScene->constraintsBuffer->planeConstraints[constraintIdx];

            if (constraint.bufferIdA == FM_INVALID_ID
                || constraint.objectIdA == FM_INVALID_ID
                || (((constraint.bufferIdA & FM_RB_FLAG) == 0) && !FmGetTetMeshBuffer(*gScene, constraint.bufferIdA))
                || (((constraint.bufferIdB & FM_RB_FLAG) == 0) && !FmGetTetMeshBuffer(*gScene, constraint.bufferIdB)))
            {
                continue;
            }

            FmVector3 posA;

            if ((constraint.objectIdA & FM_RB_FLAG)
                && gScene->rigidBodies)
            {
#if FM_DEBUG_MESHES
                uint rbIdxA = FmGetRigidBody(*gScene, constraint.objectIdA)->idx;
                FmRigidBody& rigidBodyA = gDebugRigidBodies[rbIdxA];
#else
                FmRigidBody& rigidBodyA = *FmGetRigidBody(*gScene, constraint.objectIdA);
#endif
                posA = rigidBodyA.state.pos + constraint.comToPosA;
            }
            else
            {
#if FM_DEBUG_MESHES
                uint meshIdxA = FmGetTetMesh(*gScene, constraint.objectIdA)->idx;
                FmTetMesh& tetMeshA = gDebugMeshes[meshIdxA];
#else
                FmTetMesh& tetMeshA = *FmGetTetMesh(*gScene, constraint.objectIdA);
#endif

                FmTetVertIds tetVertIdsA = FmGetTetVertIds(tetMeshA, constraint.tetIdA);

                posA = FmInterpolate(constraint.posBaryA,
                    tetMeshA.vertsPos[tetVertIdsA.ids[0]],
                    tetMeshA.vertsPos[tetVertIdsA.ids[1]],
                    tetMeshA.vertsPos[tetVertIdsA.ids[2]],
                    tetMeshA.vertsPos[tetVertIdsA.ids[3]]);
            }

            DrawSphere(posA, 0.01f, 1.0f, 1.0f, 1.0f);

            FmVector3 posB;
            if (constraint.objectIdB == FM_INVALID_ID)
            {
                posB = FmVector3(constraint.posWorldB[0], constraint.posWorldB[1], constraint.posWorldB[2]);
            }
            else if ((constraint.objectIdB & FM_RB_FLAG)
                && gScene->rigidBodies)
            {
#if FM_DEBUG_MESHES
                uint rbIdxB = FmGetRigidBody(*gScene, constraint.objectIdB)->idx;
                FmRigidBody& rigidBodyB = gDebugRigidBodies[rbIdxB];
#else
                FmRigidBody& rigidBodyB = *FmGetRigidBody(*gScene, constraint.objectIdB);
#endif
                posB = rigidBodyB.state.pos + constraint.comToPosB;
            }
            else
            {
#if FM_DEBUG_MESHES
                uint meshIdxB = FmGetTetMesh(*gScene, constraint.objectIdB)->idx;
                FmTetMesh& tetMeshB = gDebugMeshes[meshIdxB];
#else
                FmTetMesh& tetMeshB = *FmGetTetMesh(*gScene, constraint.objectIdB);
#endif

                FmTetVertIds tetVertIdsB = FmGetTetVertIds(tetMeshB, constraint.tetIdB);

                posB = FmInterpolate(constraint.posBaryB,
                    tetMeshB.vertsPos[tetVertIdsB.ids[0]],
                    tetMeshB.vertsPos[tetVertIdsB.ids[1]],
                    tetMeshB.vertsPos[tetVertIdsB.ids[2]],
                    tetMeshB.vertsPos[tetVertIdsB.ids[3]]);
            }

            GLfloat verts[] =
            {
                posA.x, posA.y, posA.z,
                posB.x, posB.y, posB.z,
                posB.x, posB.y, posB.z,
                posB.x + constraint.planeNormal0.x, posB.y + constraint.planeNormal0.y, posB.z + constraint.planeNormal0.z,
                posB.x, posB.y, posB.z,
                posB.x + constraint.planeNormal1.x, posB.y + constraint.planeNormal1.y, posB.z + constraint.planeNormal1.z,
                posB.x, posB.y, posB.z,
                posB.x + constraint.planeNormal2.x, posB.y + constraint.planeNormal2.y, posB.z + constraint.planeNormal2.z
            };
            GLfloat colors[] =
            {
                1.0, 1.0f, 0.0f,
                1.0, 1.0f, 0.0f,
                1.0, 0.0f, 0.0f,
                1.0, 0.0f, 0.0f,
                1.0, 0.0f, 0.0f,
                1.0, 0.0f, 0.0f,
                1.0, 0.0f, 0.0f,
                1.0, 0.0f, 0.0f
            };
            DrawLines(verts, colors, 8);
            DrawSphere(posB, 0.01f, 1.0f, 1.0f, 1.0f);
        }
    }
#endif

#if DRAW_VOL_CONTACTS
    if (gDrawVolContacts)
    {
        DrawVolumeContacts(gScene);
    }
#endif

#if DRAW_BROAD_PHASE_BVH
    DrawBvh(gScene->constraintsBuffer->broadPhaseHierarchy, 0, 10, 0.0f);
#endif

#if DRAW_PARTITION_BVH
    for (uint islandIdx = 0; islandIdx < gScene->constraintsBuffer->numConstraintIslands; islandIdx++)
    {
        FmConstraintSolverData* solverData = &gScene->constraintSolverBuffer->islandSolverData[islandIdx];

        float r, g, b;
        SetColorById(&r, &g, &b, islandIdx);

        DrawBvh(solverData->partitionsHierarchy, 0, 5, 0.0f, r, g, b);
    }
#endif

#if FM_DEBUG_MESHES
    for (uint meshIdx = 0; meshIdx < gFEMFXNumPreConstraintSolveMeshes; meshIdx++)
    {
        FmTetMesh& tetMesh = gDebugMeshes[meshIdx];
#else
    uint numSceneMeshes = FmGetNumEnabledTetMeshes(*gScene);
    for (uint meshIdx = 0; meshIdx < numSceneMeshes; meshIdx++)
    {
        FmTetMesh& tetMesh = *FmGetTetMesh(*gScene, FmGetEnabledTetMeshId(*gScene, meshIdx));
#endif    
        float r, g, b;
        SetColorById(&r, &g, &b, tetMesh.objectId);

        if (tetMesh.flags & FM_OBJECT_FLAG_SLEEPING)
        {
            r = (r + 0.2f)*0.6f;
            g = (g + 0.2f)*0.6f;
            b = (b + 0.2f)*0.6f;
        }

#if DRAW_MESH_BVH
        DrawBvh(tetMesh.bvh, 2, 2, 0.0f, r, g, b);
#endif
        DrawTetMesh(tetMesh, r, g, b);
    }

#if EXTERNAL_RIGIDBODIES
    if (gRbScene)
    {
        FmScene* rbFemScene = gRbScene->scene;

#if DRAW_DIST_CONTACTS
        if (gDrawContacts)
        {
            DrawDistanceContacts(rbFemScene);
        }
#endif
#if DRAW_VOL_CONTACTS
        if (gDrawVolContacts)
        {
            DrawVolumeContacts(rbFemScene);
        }
#endif

        uint numSceneRigidBodies = FmGetNumEnabledRigidBodies(*gScene);

        float rigidBodyColorR = 0.8f;
        float rigidBodyColorG = 0.1f;
        float rigidBodyColorB = 0.1f;
#if FM_DEBUG_MESHES
        for (uint rbIdx = 0; rbIdx < gFEMFXNumPreConstraintSolveRigidBodies; rbIdx++)
        {
            FmRigidBody& rigidBody = gDebugRigidBodies[rbIdx];
#else
        for (uint rbIdx = 0; rbIdx < numSceneRigidBodies; rbIdx++)
        {
            FmRigidBody& rigidBody = *FmGetRigidBody(*gScene, FmGetEnabledRigidBodyId(*gScene, rbIdx));
#endif

            if (rigidBody.flags & FM_OBJECT_FLAG_SLEEPING)
            {
                DrawRigidBody(rigidBody, (rigidBodyColorR + 0.1f)*0.6f, (rigidBodyColorG + 0.1f)*0.6f, (rigidBodyColorB + 0.1f)*0.6f);
            }
            else
            {
                DrawRigidBody(rigidBody, rigidBodyColorR, rigidBodyColorG, rigidBodyColorB);
            }
        }
    }
#else
    uint numSceneRigidBodies = FmGetNumEnabledRigidBodies(*gScene);

#if FM_DEBUG_MESHES
    for (uint rbIdx = 0; rbIdx < gFEMFXNumPreConstraintSolveRigidBodies; rbIdx++)
    {
        FmRigidBody& rigidBody = gDebugRigidBodies[rbIdx];
#else
    for (uint rbIdx = 0; rbIdx < numSceneRigidBodies; rbIdx++)
    {
        FmRigidBody& rigidBody = *FmGetRigidBody(*gScene, FmGetEnabledRigidBodyId(*gScene, rbIdx));
#endif
        float r, g, b;
        SetColorById(&r, &g, &b, rigidBody.objectId);

        if (rigidBody.flags & FM_OBJECT_FLAG_SLEEPING)
        {
            DrawRigidBody(rigidBody, (r + 0.1f)*0.6f, (g + 0.1f)*0.6f, (b + 0.1f)*0.6f);
        }
        else
        {
            DrawRigidBody(rigidBody, r, g, b);
        }
    }
#endif

    const FmSceneControlParams& params = FmGetSceneControlParams(*gScene);
#if DRAW_GROUND
    {
        const float quadHalf = 40.0f;
        float r = 0.6f;
        float g = 0.6f;
        float b = 0.6f;
        GLfloat verts[] =
        {
            -quadHalf, params.collisionPlanes.minY, -quadHalf,
            -quadHalf, params.collisionPlanes.minY, quadHalf,
            quadHalf, params.collisionPlanes.minY, quadHalf,
            -quadHalf, params.collisionPlanes.minY, -quadHalf,
            quadHalf, params.collisionPlanes.minY, quadHalf,
            quadHalf, params.collisionPlanes.minY, -quadHalf,
        };
        GLfloat normals[] =
        {
            0.0f, 1.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
        };
        GLfloat colors[] =
        {
            r, g, b, r, g, b, r, g, b, r, g, b, r, g, b, r, g, b
        };
        DrawTriangles(verts, normals, colors, 6);
    }
#endif

    {
        FmVector3 boxMin(params.collisionPlanes.minX, params.collisionPlanes.minY, params.collisionPlanes.minZ);
        FmVector3 boxMax(params.collisionPlanes.maxX, params.collisionPlanes.maxY, params.collisionPlanes.maxZ);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        GLfloat verts[] = {
            boxMin.x, boxMin.y, boxMin.z,
            boxMax.x, boxMin.y, boxMin.z,
            boxMin.x, boxMax.y, boxMin.z,
            boxMax.x, boxMax.y, boxMin.z,
            boxMin.x, boxMin.y, boxMax.z,
            boxMax.x, boxMin.y, boxMax.z,
            boxMin.x, boxMax.y, boxMax.z,
            boxMax.x, boxMax.y, boxMax.z,
            boxMin.x, boxMin.y, boxMin.z,
            boxMin.x, boxMax.y, boxMin.z,
            boxMax.x, boxMin.y, boxMin.z,
            boxMax.x, boxMax.y, boxMin.z,
            boxMin.x, boxMin.y, boxMax.z,
            boxMin.x, boxMax.y, boxMax.z,
            boxMax.x, boxMin.y, boxMax.z,
            boxMax.x, boxMax.y, boxMax.z,
            boxMin.x, boxMin.y, boxMin.z,
            boxMin.x, boxMin.y, boxMax.z,
            boxMax.x, boxMin.y, boxMin.z,
            boxMax.x, boxMin.y, boxMax.z,
            boxMin.x, boxMax.y, boxMin.z,
            boxMin.x, boxMax.y, boxMax.z,
            boxMax.x, boxMax.y, boxMin.z,
            boxMax.x, boxMax.y, boxMax.z
        };
        GLfloat colors[] = {
            1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
        };
        DrawLines(verts, colors, 24);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
}

// Set up view and default color and shading settings 
static void InitGLState()
{
    gViewerCam.SetElevation(-0.017f*20.0f);
    gViewerCam.SetAzimuth(0.017f*135.0f);

    float lookatBaseX = 0.0f;
    float lookatBaseY = 0.0f;
    float lookatBaseZ = 0.0f;
    if (gScene)
    {
        const FmSceneControlParams& params = FmGetSceneControlParams(*gScene);
        lookatBaseX = (params.collisionPlanes.minX + params.collisionPlanes.maxX) * 0.5f;
        lookatBaseY = params.collisionPlanes.minY;
        lookatBaseZ = (params.collisionPlanes.minZ + params.collisionPlanes.maxZ) * 0.5f;
    }
    gViewerCam.SetLookAtX(lookatBaseX + 0.0f);
    gViewerCam.SetLookAtY(lookatBaseY + 2.0f);
    gViewerCam.SetLookAtZ(lookatBaseZ + 0.0f);
    gViewerCam.SetRadius(40.0f);

    gViewerCam.SetAzimuthInputScale(-0.01f);
    gViewerCam.SetElevationInputScale(-0.01f);
    gViewerCam.SetRadiusInputScale(-0.01f);
    gViewerCam.SetLookAtXInputScale(-0.01f);
    gViewerCam.SetLookAtYInputScale(0.01f);
    gViewerCam.SetLookAtZInputScale(-0.001f);

    GLfloat Ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
    GLfloat Diffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    GLfloat Specular[] = { .2f, .2f, .2f, 1.f };
    GLfloat SpecularExp[] = { 50.f };
    GLfloat Emission[] = { 0.1f, 0.1f, 0.1f, 1.0f };

    glMaterialfv(GL_FRONT, GL_AMBIENT, Ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, Diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, Specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, SpecularExp);
    glMaterialfv(GL_FRONT, GL_EMISSION, Emission);

    glMaterialfv(GL_BACK, GL_AMBIENT, Ambient);
    glMaterialfv(GL_BACK, GL_DIFFUSE, Diffuse);
    glMaterialfv(GL_BACK, GL_SPECULAR, Specular);
    glMaterialfv(GL_BACK, GL_SHININESS, SpecularExp);
    glMaterialfv(GL_BACK, GL_EMISSION, Emission);

    glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);

    GLfloat light_position[] = { 1.0f, 1.0f, 1.0f, 0.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);

    glShadeModel(GL_SMOOTH);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(gFovY, (double)gWinWidth / (double)gWinHeight, gNearPlane, gFarPlane);
    glMatrixMode(GL_MODELVIEW);
}

static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    (void)mods;
    (void)window;
    if (action == GLFW_RELEASE)
    {
        gDragMode = -1;
        return;
    }

    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        gDragMode = 0;
        gViewerCam.AzimuthDragStart(gCursorX);
        gViewerCam.ElevationDragStart(gCursorY);
    }
    else if (button == GLFW_MOUSE_BUTTON_RIGHT)
    {
        gDragMode = 1;
        gViewerCam.RadiusDragStart(gCursorY);
    }
    else
    {
        gDragMode = 2;
        gViewerCam.LookAtDragStart(gCursorX, gCursorY, 0);
    }
}

static void CursorPositionCallback(GLFWwindow* window, double x, double y)
{
    (void)window;

    gCursorX = (float)x;
    gCursorY = (float)y;

    if (gDragMode == 0)
    {
        gViewerCam.AzimuthDragCurrent(gCursorX);
        gViewerCam.ElevationDragCurrent(gCursorY);
    }
    else if (gDragMode == 1)
    {
        gViewerCam.RadiusDragCurrent(gCursorY);
    }
    else if (gDragMode == 2)
    {
        gViewerCam.LookAtDragCurrent(gCursorX, gCursorY, 0);
    }
}

static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    (void)window;
    (void)scancode;
    (void)mods;


    if (action != GLFW_PRESS)
    {
        return;
    }

    FmMatrix3 viewRotation;
    viewRotation.col0.x = gViewerCam.mViewRotation.col0.x;
    viewRotation.col0.y = gViewerCam.mViewRotation.col0.y;
    viewRotation.col0.z = gViewerCam.mViewRotation.col0.z;
    viewRotation.col1.x = gViewerCam.mViewRotation.col1.x;
    viewRotation.col1.y = gViewerCam.mViewRotation.col1.y;
    viewRotation.col1.z = gViewerCam.mViewRotation.col1.z;
    viewRotation.col2.x = gViewerCam.mViewRotation.col2.x;
    viewRotation.col2.y = gViewerCam.mViewRotation.col2.y;
    viewRotation.col2.z = gViewerCam.mViewRotation.col2.z;
    FmVector3 viewPosition;
    viewPosition.x = gViewerCam.mViewPosition.x;
    viewPosition.y = gViewerCam.mViewPosition.y;
    viewPosition.z = gViewerCam.mViewPosition.z;
    float speed = 30.0f;
    float highSpeed = 60.0f;

    bool fixedPosForWoodPanels = false;

    switch (key)
    {
    case GLFW_KEY_H:
    {
        PrintHelp();
        break;
    }
    case GLFW_KEY_Q:
    {
        FreeScene();
        FM_SHUTDOWN_TRACE();
        glfwSetWindowShouldClose(window, GLFW_TRUE);
        exit(1);
        break;
    }
    case GLFW_KEY_P:
    {
        gRunning = !gRunning;
        break;
    }
#if !PERF_TEST
    case GLFW_KEY_R:
    {
#if !FIX_INITIAL_CONDITIONS
        gRandomSeed++;
#endif
        FreeScene();
        InitScene("models/", gTimingsPath.c_str(), gNumThreads, gRandomSeed);
        gFrameCounter = 0;

        break;
    }
#endif
#if CARS_SCENE
    case GLFW_KEY_L:
    {
        for (uint i = 0; i < NUM_INSTANCES; i++)
        {
            LaunchCarSimObject(i, 36.0f);
        }
        break;
    }
#endif
    case GLFW_KEY_F:
    {
        FireProjectile(viewRotation, viewPosition, speed, fixedPosForWoodPanels);
        break;
    }
    case GLFW_KEY_SPACE:
    {
        FireProjectile(viewRotation, viewPosition, highSpeed, fixedPosForWoodPanels);
        break;
    }
    case GLFW_KEY_RIGHT_BRACKET:
    {
        gStepForward = true;
        gRunning = false;
        break;
    }
#if FIX_INITIAL_CONDITIONS
    case GLFW_KEY_LEFT_BRACKET:
    {
        gStepBackward = true;
        gTargetFrame = gFrameCounter - 1;
        gRunning = false;
        break;
    }
#endif
#if DRAW_DIST_CONTACTS
    case GLFW_KEY_C:
    {
        gDrawContacts = !gDrawContacts;
        printf("Drawing distance contacts: %i\n", (int)gDrawContacts);
        break;
    }
#endif
#if DRAW_GLUE
    case GLFW_KEY_G:
    {
        gDrawGlue = !gDrawGlue;
        printf("Drawing glue: %i\n", (int)gDrawGlue);
        break;
    }
#endif
#if DRAW_VOL_CONTACTS
    case GLFW_KEY_V:
    {
        gDrawVolContacts = !gDrawVolContacts;
        printf("Drawing volume contacts: %i\n", (int)gDrawVolContacts);
        break;
    }
#endif
    case GLFW_KEY_D:
    {
        gDrawMode = (gDrawMode + 1) % 4;
        if (gDrawMode == 0)
        {
            gDrawWireframe = false;
            gDrawFractureFlags = false;
        }
        else if (gDrawMode == 1)
        {
            gDrawWireframe = true;
            gDrawFractureFlags = false;
            printf("Drawing wireframe\n");
        }
        else if (gDrawMode == 2)
        {
            gDrawWireframe = true;
            gDrawFractureFlags = true;
            gDrawFracturableFaces = true;
            printf("Drawing fracturable faces\n");
        }
        else if (gDrawMode == 3)
        {
            gDrawWireframe = true;
            gDrawFractureFlags = true;
            gDrawFracturableFaces = false;
            printf("Drawing non-fracturable faces\n");
        }
        break;
    }
    case GLFW_KEY_A:
    {
        gDrawHighAspectRatioTets = !gDrawHighAspectRatioTets;
        printf("Drawing high aspect-ratio tets: %i\n", (int)gDrawHighAspectRatioTets);
        break;
    }
    case GLFW_KEY_K:
    {
        gDrawKinematicTets = !gDrawKinematicTets;
        printf("Drawing kinematic tets: %i\n", (int)gDrawKinematicTets);
        break;
    }
    case GLFW_KEY_T:
    {
        FM_ENABLE_TRACE();
        break;
    }
    default:
        break;
    }
}

static void ErrorCallback(int error, const char* description)
{
    fprintf(stderr, "Error: (%x) %s\n", error, description);
}

static void WindowSizeCallback(GLFWwindow*, int width, int height)
{
    glViewport(0, 0, width, height);
}

#if !PERF_TEST
#if defined(_MSC_VER)
static FM_FORCE_INLINE double GetTimeMs()
{
    LARGE_INTEGER freq, counter;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&counter);

    uint64_t timeseconds = counter.QuadPart / freq.QuadPart;
    uint64_t timeremainder = counter.QuadPart % freq.QuadPart;

    return ((double)timeseconds + (double)timeremainder / (double)freq.QuadPart) * 1000.0;
}
#elif defined(__GNUC__)
static FM_FORCE_INLINE double GetTimeMs()
{
    struct timeval val;
    gettimeofday(&val, nullptr);
    return (double)val.tv_sec * 1000.0 + (double)val.tv_usec * 0.001;
}
#endif
#endif

static void Update()
{
#if !PERF_TEST
    static double lastTime = 0;
    double newTime = GetTimeMs();

    if (newTime - lastTime < 16.666666f)
        return;

    lastTime = newTime;
#endif

#if TRACE_TEST && CARS_SCENE
    static int count = 0;
    if (count++ == 120)
    {
        for (uint i = 0; i < NUM_INSTANCES; i++)
        {
            LaunchCarSimObject(i, 36.0f);
        }
    }
#endif

#if PERF_TEST
#if RIGIDBODIES_IN_SCENE
    FmUpdateSceneRb(gRbScene, gScene, gTimestep);
#else
    FmUpdateScene(gScene, gTimestep);
#endif
    UpdateObjects();

    gFrameCounter++;

#if BLOCKS_SCENE
    int maxFrames = 500;
#else
    int maxFrames = 200;
#endif

    if (gFrameCounter >= maxFrames)
    {
        // End or run; run same scenario with different thread count
        if (gPerfTestMode == PERF_TEST_MODE_MAX_TO_MIN_THREADS_DECREMENTING)
        {
            gNumThreads--;
        }
        else
        {
            gNumThreads /= 2;
        }

        if (gNumThreads < gPerfTestMinThreads)
        {
            // Ran all thread numbers; repeat test with different seed
            if (gRandomSeed < gPerfTestNumSeeds - 1)
            {
                gNumThreads = gPerfTestMaxThreads;
                gRandomSeed++;
            }
            else
            {
                FreeScene();
                exit(0);
            }
        }

        FreeScene();
        InitScene("models/", gTimingsPath.c_str(), gNumThreads, gRandomSeed);
        gFrameCounter = 0;
    }
#else
    if (gStepBackward)
    {
        FreeScene();
        InitScene("models/", gTimingsPath.c_str(), gNumThreads, gRandomSeed);
        gFrameCounter = 0;
        while (gFrameCounter < gTargetFrame)
        {
#if EXTERNAL_RIGIDBODIES
            UpdateSceneRb(gRbScene, gScene, gTimestep);
#else
            FmUpdateScene(gScene, gTimestep);
#endif
            UpdateObjects();
            gFrameCounter++;
        }
        gStepBackward = false;
    }
    else if (gRunning || gStepForward)
    {
#if TRACE_TEST
        if (gFrameCounter == 150)
        {
            FM_ENABLE_TRACE();
        }
#endif
#if EXTERNAL_RIGIDBODIES
        UpdateSceneRb(gRbScene, gScene, gTimestep);
#else
        FmUpdateScene(gScene, gTimestep);
#endif
        UpdateObjects();

        gFrameCounter++;
        gStepForward = false;

#if TRACE_TEST
        FM_DISABLE_TRACE();
#endif
    }
#endif

    if (gScene->warningsReport.flags.val != 0)
    {
        printf("warnings flags: %u\n", gScene->warningsReport.flags.val);
    }
}

int __cdecl main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
#if PERF_TEST
    if (argc > 1)
    {
        gTimingsPath = std::string(argv[1]);
    }

    if (argc > 2)
    {
        gPerfTestMode = atoi(argv[2]);
    }

    if (argc > 3)
    {
        gPerfTestNumSeeds = atoi(argv[3]);
    }

    if (argc > 4)
    {
        gPerfTestMinThreads = atoi(argv[4]);
    }
    else
    {
        gPerfTestMinThreads = 1;
    }

    if (argc > 5)
    {
        gPerfTestMaxThreads = atoi(argv[5]);
        if (gPerfTestMaxThreads <= 0)
        {
            gPerfTestMaxThreads = TLGetTaskSystemDefaultNumThreads();
        }
    }
    else
    {
        gPerfTestMaxThreads = TLGetTaskSystemDefaultNumThreads();
    }

    if (gPerfTestMode == PERF_TEST_MODE_MAX_THREADS)
    {
        gPerfTestMinThreads = gPerfTestMaxThreads;
    }

    gNumThreads = gPerfTestMaxThreads;
#else
    gNumThreads = TLGetTaskSystemDefaultNumThreads();
#endif

    FM_INIT_TRACE();

    InitScene("models/", gTimingsPath.c_str(), gNumThreads, gRandomSeed);

    gFrameCounter = 0;

    // Setup GLFW and window
    GLFWwindow* window;

    if (!glfwInit())
    {
        printf("Failed to init GLFW\n");
        exit(EXIT_FAILURE);
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    window = glfwCreateWindow(gWinWidth, gWinHeight, "FEMFXViewer", nullptr, nullptr);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwSetKeyCallback(window, KeyCallback);
    glfwSetMouseButtonCallback(window, MouseButtonCallback);
    glfwSetCursorPosCallback(window, CursorPositionCallback);
    glfwSetErrorCallback(ErrorCallback);
    glfwSetWindowSizeCallback(window, WindowSizeCallback);

    glfwMakeContextCurrent(window);
    glfwSwapInterval(0);

    glfwSetTime(0.0);

    InitGLState();

    PrintHelp();

    // Draw loop
    while (!glfwWindowShouldClose(window))
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        float viewMatrix[16];
        gViewerCam.GetViewMatrix(viewMatrix);

        glLoadMatrixf(viewMatrix);

        Update();
        DrawScene();

        glFlush();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    gluDeleteQuadric(gQuadric);

    glfwDestroyWindow(window);
    glfwTerminate();

    exit(EXIT_SUCCESS);
}
