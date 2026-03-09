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
// Result types and workspaces used in collision detection of object pairs
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommonInternal.h"
#include "FEMFXVectorMath.h"
#include "FEMFXTriCcd.h"
#include "FEMFXTriIntersection.h"
#include "FEMFXTetMeshConnectivity.h"
#include "FEMFXHashSet.h"

// Max object-pair surface-intersection contacts to limit performance impact.  
// It's included to help constrain and push out small intersections.
// TODO: some criteria for subset of contacts to add.
#define FM_SURFACE_INTERSECTION_MAX_CONTACTS 16

// Store contacts in a thread local temporary buffer before copying into global array.
// Should improve locality of object pair's contacts, but perf impact seems minor.
#define FM_MAX_TEMP_DISTANCE_CONTACTS 128

// Size of temporary buffer holding triangle pairs reached in mid-phase
#define FM_MAX_MESH_COLLISION_TRI_PAIR 128

namespace AMD
{
    struct FmBvh;
    struct FmTetMesh;
    struct FmConstraintsBuffer;
    struct FmCollisionReport;
    struct FmDistanceContact;
    struct FmDistanceContactPairInfo;

    // Triangle pair found during mesh BVH collision detection, and data for performing intersection and CCD
    struct FmMeshCollisionTriPair
    {
        uint                    exteriorFaceIdA = FM_INVALID_ID;
        uint                    exteriorFaceIdB = FM_INVALID_ID;
        FmTri                   triA;
        FmTri                   triB;
        uint                    tetIdA = FM_INVALID_ID;
        uint                    tetIdB = FM_INVALID_ID;
        FmFaceVertIds           tetCornersA;
        FmFaceVertIds           tetCornersB;
        FmFaceVertIds           faceVertIdsA;
        FmFaceVertIds           faceVertIdsB;
        FmTriIntersectionPoints triIntersectionPoints;
#if FM_SOA_TRI_CCD
        uint16_t                vertVertFound = 0;  // to mark feature pairs already found for contacts
        uint16_t                vertEdgeFound = 0;
        uint16_t                edgeVertFound = 0;
#endif
        bool                    pairIntersects = false;
        bool                    isTetInverted = false;
    };

#if FM_SURFACE_INTERSECTION_CONTACTS
    struct FmIntersectingFacePair
    {
        uint exteriorFaceIdA = FM_INVALID_ID;
        uint exteriorFaceIdB = FM_INVALID_ID;
        FmVector3 pos;
    };
#endif

    // Data structures for finding volume contacts.
    // Method needs to build a set of vertices found during BVH traversal that have non-zero volume gradients, 
    // and accumulate all subsequent gradient contributions.

    struct FmVolumeContactVertSetElement
    {
        uint      key = FM_INVALID_ID;
        FmVector3 dVdp;
        FmVector3 centerToVert;
        float     V = 0.0f;
        uint      ownerExteriorFaceId = FM_INVALID_ID;
        int       intersectionValue = 0;
    };

    struct FmVolumeContactVertSetFuncs
    {
        static bool IsValidKey(uint key)
        {
            return (key != FM_INVALID_ID);
        }

        static bool KeysEqual(uint keyA, uint keyB)
        {
            return (keyA == keyB);
        }

        static void SetInvalidKey(uint& key)
        {
            key = FM_INVALID_ID;
        }

        static uint HashFunc(uint key)
        {
            return FmComputeHash(key);
        }

        static void InitValue(FmVolumeContactVertSetElement& elem)
        {
            elem.dVdp = FmVector3(0.0f);
            elem.centerToVert = FmVector3(0.0f);
            elem.V = 0.0f;
            elem.ownerExteriorFaceId = FM_INVALID_ID;
            elem.intersectionValue = 0;
        }
    };

    typedef FmHashSet< FmVolumeContactVertSetElement, FmVolumeContactVertSetFuncs > FmVolumeContactVertSet;
    typedef FmExpandableHashSet< FmVolumeContactVertSetElement, FmVolumeContactVertSetFuncs > FmVolumeContactVertSetExpandable;

    struct FmVolumeContactFace
    {
        int       numFullFaceIntegrations = 0;
        int       normalScale = 1;
        bool      isFullFace = false;

        uint      vertIds[3] = { FM_INVALID_ID, FM_INVALID_ID, FM_INVALID_ID };
        int       intersectionVals[3] = { 0, 0, 0 };
        int       intersectionValsB[3] = { 0, 0, 0 };
        FmVector3 partialFacedVdp[3];
        float     partialFaceV = 0.0f;
    };

    // Definition of temporary memory containing all vertex sets created during BVH traversal.
    struct FmVolumeContactWorkspace
    {
        FmVolumeContactVertSetExpandable meshAVertSets;
        FmVolumeContactVertSetExpandable meshBVertSets;
        FmVolumeContactFace*             meshAFaces = nullptr;     // for self-collision
#if FM_SURFACE_INTERSECTION_CONTACTS
        FmIntersectingFacePair* intersectingFacePairs = nullptr;
        uint                    numIntersectingFacePairs = 0;
        uint                    maxIntersectingFacePairs = 0;
#endif
    };

#if FM_CONTACT_REDUCTION
#define FM_MAX_TIME_WEIGHTED_CONTACTS_PER_VERTEX 1
#define FM_MAX_DIST_WEIGHTED_CONTACTS_PER_VERTEX 1
#define FM_TIME_WEIGHTED_CONTACTS_SIZE ((FM_MAX_TIME_WEIGHTED_CONTACTS_PER_VERTEX == 0)? 1 : FM_MAX_TIME_WEIGHTED_CONTACTS_PER_VERTEX)
#define FM_DIST_WEIGHTED_CONTACTS_SIZE ((FM_MAX_DIST_WEIGHTED_CONTACTS_PER_VERTEX == 0)? 1 : FM_MAX_DIST_WEIGHTED_CONTACTS_PER_VERTEX)

    // Importance of contact for vert is measured by time of impact and distance to the contact point.
    // Keeping a few of each type
    struct FmVertKeptContact
    {
        uint  idx = FM_INVALID_ID;
        float impactTime = 0.0f;
        float distanceSqr = 0.0f;
        bool isTempContact = false;
    };

    struct FmVertKeptContactSet
    {
        FmVertKeptContact contactsTimeWeighted[FM_TIME_WEIGHTED_CONTACTS_SIZE];
        FmVertKeptContact contactsDistanceWeighted[FM_DIST_WEIGHTED_CONTACTS_SIZE];
        uint numContactsTimeWeighted = 0;
        uint numContactsDistWeighted = 0;
    };

    struct FmContactReductionWorkspace
    {
        FmVertKeptContactSet* vertsContactsA = nullptr;
        FmVertKeptContactSet* vertsContactsB = nullptr;
        uint numVertsA = 0;
        uint numVertsB = 0;
    };
#endif

    struct FmDistanceContactTetVertIds
    {
        FmTetVertIds tetVertIdsA;
        FmTetVertIds tetVertIdsB;
    };

    struct FmCollidedObjectPairTemps
    {
        FmVolumeContactWorkspace* volContactWorkspace = nullptr;  // Workspace for volume contact verts/gradients

#if FM_CONTACT_REDUCTION
        FmContactReductionWorkspace* contactReductionWorkspace = nullptr;
#endif

#if FM_SOA_TRI_INTERSECTION
        FmMeshCollisionTriPair* meshCollisionTriPairs = nullptr;
        uint                    numMeshCollisionTriPairs = 0;
#endif

        // Temporary storage for contacts, later copied into global array
        FmDistanceContactPairInfo* distanceContactPairInfoBuffer = nullptr;
        FmDistanceContact* distanceContactBuffer = nullptr;
        FmDistanceContactTetVertIds* distanceContactTetVertIds = nullptr;
        uint numDistanceContacts = 0;
        uint numDistanceContactsNonzeroRefCount = 0;
    };

    // Object pair data passed into mesh BVH collision detection
    struct FmCollidedObjectPair
    {
        void*                     objectA = nullptr;              // Tet mesh or rigid body
        void*                     objectB = nullptr;              // Tet mesh or rigid body
        FmTetMesh*                tetMeshA = nullptr;             // Tet mesh or collision object for rigid body
        FmTetMesh*                tetMeshB = nullptr;             // Tet mesh or collision object for rigid body
        uint                      objectAId = FM_INVALID_ID;
        uint                      objectBId = FM_INVALID_ID;
        FmBvh*                    objectAHierarchy = nullptr;
        FmBvh*                    objectBHierarchy = nullptr;
        FmVector3                 objectAMinPosition;
        FmVector3                 objectAMaxPosition;
        FmVector3                 objectBMinPosition;
        FmVector3                 objectBMaxPosition;
        FmConstraintsBuffer*      constraintsBuffer = nullptr;    // Stores generated contacts
        float                     timestep = (1.0f / 60.0f);
        float                     distContactBias = 0.0f;
        float                     distContactThreshold = 0.0f;
        float                     volContactBias = 0.0f;
        float                     volContactThreshold = 0.0f;
        uint                      numDistanceContacts = 0;
        uint                      numVolumeContacts = 0;
        bool                      objectAContactWithDynamicVerts = false;  // Contacts found with dynamics verts in objectA (affects wakeup)
        bool                      objectBContactWithDynamicVerts = false;  // Contacts found with dynamics verts in objectB (affects wakeup)
        FmVector3                 volContactCenter;                // Origin for volume contact calculations to improve precision
        FmVector3                 volContactObjectACenterPos;      // Object center position (used for rigid bodies)
        FmVector3                 volContactObjectBCenterPos;      // Object center position (used for rigid bodies)
        FmVector3                 volContactNormal;                // Average volume contact normal
        float                     volContactV = 0.0f;                     // Signed volume (negative for overlap)
        FmCollisionReport*        collisionReport = nullptr;
        uint                      numDistanceContactReports = 0;
        uint                      numVolumeContactReports = 0;
        FmCollidedObjectPairTemps temps;
    };

    struct FmBoxBoxCcdTemps
    {
        static const int maxContacts = 8 * 6 + 12 * 12 + 6 * 8;
        FmCcdResult boxBoxContacts[maxContacts];
        uint numCcdContacts = 0;

        uint vertVertFound[8][8] = { { 0 } };
        uint vertEdgeFound[8][12] = { { 0 } };
        uint edgeVertFound[12][8] = { { 0 } };

        inline void Clear()
        {
            numCcdContacts = 0;
            memset(vertVertFound, 0, sizeof(uint) * 8 * 8);
            memset(vertEdgeFound, 0, sizeof(uint) * 8 * 12);
            memset(edgeVertFound, 0, sizeof(uint) * 12 * 8);
        }
    };

    static inline FmBoxBoxCcdTemps* FmAllocBoxCcdTemps(uint8_t *& pBuffer, size_t maxFreeBytes)
    {
        uint numBytes = FM_PAD_16(sizeof(FmBoxBoxCcdTemps));

        if (numBytes > maxFreeBytes)
        {
            return nullptr;
        }

        FmBoxBoxCcdTemps* pBoxCcdTemps = (FmBoxBoxCcdTemps*)pBuffer;

        pBuffer += numBytes;

        return pBoxCcdTemps;
    }
}