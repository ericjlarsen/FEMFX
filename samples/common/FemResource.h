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
// Container for data loaded from .FEM file.
// Adapted from UE4 plugin code.
//---------------------------------------------------------------------------------------
#pragma once

#include "AMD_FEMFX.h"
#include "FEMFXArray.h"

#include <vector>
#include <string>

namespace AMD
{
    struct FVector
    {
        float X = 0.0f;
        float Y = 0.0f;
        float Z = 0.0f;

        FVector() {}
        FVector(float inX, float inY, float inZ)
        {
            X = inX;
            Y = inY;
            Z = inZ;
        }
    };

    struct FVector4
    {
        float X = 0.0f;
        float Y = 0.0f;
        float Z = 0.0f;
        float W = 0.0f;

        FVector4() {}
        FVector4(float inX, float inY, float inZ, float inW)
        {
            X = inX;
            Y = inY;
            Z = inZ;
            W = inW;
        }
    };

    /**
    * This class is going to mimick the .fem file structure
    */
    struct FTet
    {
        int TetIndex = 0;
        std::vector<int> Indices;
    };


    struct FNodeResource
    {
        bool IsBoundaryMarker = false;
        int NumAttributes = 0;
        int NumDimensions = 0;
        int NumPoints = 0;
        std::vector<float> Data;
    };

    struct FEleResource
    {
        bool IsRegionAttribute = false;
        int NumNodesPerTets = 0;
        int NumTetrahedra = 0;
        std::vector<FTet> Data;
    };

    struct FNameIndexMap
    {
        std::string Name = "";
        std::vector<uint32_t> TetIds;
        std::vector<uint32_t> NoFractureFaces;
    };

    struct FRigidBody
    {
        std::string Name;
        FVector Position;
        FVector Dimensions;
        FVector4 Rotation;
        float mass = 0.0f;
        std::vector<float> BodyInertiaTensor;
    };

    struct FAngleConstraint
    {
        std::string Name;
        unsigned int BodyA = 0;
        unsigned int BodyB = 0;
        FVector AxisBodySpaceA;
        FVector AxisBodySpaceB;
    };

    struct FGlueConstraint
    {
        std::string Name;
        unsigned int BodyA = 0;
        bool IsRigidBodyA = false;
        unsigned int BodyB = 0;
        bool IsRigidBodyB = false;
        FVector4 PosBodySpaceA;
        FVector4 PosBodySpaceB;
        unsigned int TetIdA = 0;
        unsigned int TetIdB = 0;
        float BreakThreshold = 0.0f;
        unsigned int MinGlueConstraints = 0;
    };

    struct FFEMPlane
    {
        float Bias = 0.0f;
        bool NonNegative = false;
        FVector PlaneNormal;
    };

    struct FPlaneConstraint : public FGlueConstraint
    {
        int NumberOfPlanes = 0;
        std::vector<FFEMPlane> Planes;
    };


    struct FComponent
    {
        int32_t NumberOfCornersPerShard = 0;
        std::string Name;
        bool IsFracturable = false;
        int CollisionGroup = 0;
        int NumTags = 0;
        std::vector<FNameIndexMap> Tags;
        int NumMaterials = 0;
        std::vector<FNameIndexMap> Materials;
        FEleResource EleFile;
        FNodeResource NodeFile;
        int NumFBXFiles = 0;
        std::vector<std::string> FBXFiles;
        std::vector<int> AssignedTetFace;
        std::vector<float> Barycentrics;
        std::vector<int32_t> BarycentricsPosIds;
        std::vector<int> TetAssignment;
        std::vector<float> VertexColor;
        std::vector<float> VertexNormal;
        std::vector<float> VertexPosition;
        std::vector<float> VertexTangent;
        std::vector<float> VertexUVs;
        std::vector<int32_t> ShardIds;
        std::vector<int32_t> Triangles;
        std::vector<float> Centroids;
        int32_t NumberOfShards = 0;
    };

    struct FTetVertIds
    {
        unsigned int Ids[4] = { 0, 0, 0, 0 };
    };

    struct FTetIdxToMaterial
    {
        unsigned int tetIndex = 0;
    };

    struct FBVSizes
    {
        int totalSize = 0;
        int nodeSize = 0;
        int primBoxesSize = 0;
        int boxesSize = 0;
        int mortonCodesSize = 0;
        int mortonCodesSortedSize = 0;
        int primIndicesSortedSize = 0;
        int numPrimsSize = sizeof(unsigned int);

        FBVSizes() {}

        void CalculateTotalSize()
        {
            totalSize += nodeSize;
            totalSize += primBoxesSize;
            totalSize += boxesSize;
            totalSize += mortonCodesSize;
            totalSize += mortonCodesSortedSize;
            totalSize += primIndicesSortedSize;
            totalSize += numPrimsSize;
        }
    };

    struct FTetMeshBufferInfo
    {
        uint32_t pBufferOffset = 0;
        uint32_t tetMeshOffset = 0;
        uint32_t solverDataOffset = 0;
        uint32_t tetMeshVertOffsetsOffset = 0;
        uint32_t vertReferenceOffset = 0;
        uint32_t tetReferenceOffset = 0;
    };

    struct FComponentResources
    {
        int32_t NumberOfCornersPerShard = 0;
        std::string Name;
        int CollisionGroup = 0;
        std::vector<FNameIndexMap> Tags;
        std::vector<FNameIndexMap> Materials;
        std::vector<std::string> FBXFiles;
        int NumVerts = 0;
        int NumTets = 0;
        float RestVolume = 0.0f;
        FVector minPos;
        FVector maxPos;
        int MaxVerts = 0;
        std::vector<uint8_t> restPositions; // 3 * NumVerts floats
        std::vector<uint8_t> tetVertIds;
        std::vector<unsigned int> vertIncidentTets;
        std::vector<int> VertexIndices;
        std::vector<int> AssignedTetFace;
        std::vector<float> Barycentrics;
        std::vector<int> TetAssignment;
        std::vector<int> BarycentricsPosIds;
        std::vector<float> VertexColor;
        std::vector<float> VertexNormal;
        std::vector<float> VertexPosition;
        std::vector<float> VertexTangent;
        std::vector<float> VertexUVs;
        std::vector<int> ShardIds;
        std::vector<int> Triangles;
        std::vector<float> Centroids;
        std::vector<unsigned int> VertPermutation;
        std::vector<unsigned int> TetPermutation;
        int32_t NumberOfShards = 0;
    };

    struct FActorResource
    {
        std::vector<FRigidBody> RigidBodies;
        std::vector<FAngleConstraint> AngleConstraints;
        std::vector<FGlueConstraint> GlueConstraints;
        std::vector<FPlaneConstraint> PlaneConstraints;
    };

    class FEMResource
    {
    public:
        std::string Version;
        std::vector<FComponentResources> ComponentResources;
        FActorResource ActorResource;
        static FComponentResources ProcessResource(
            std::vector<unsigned int>& vertPermutation, std::vector<unsigned int>& tetPermutation,
            AMD::FmVector3* restPositions, AMD::FmTetVertIds* tetVertIds, std::vector<unsigned int>* vertIncidentTets, int numVerts, int numTets);
        void ProcessResource();
        void AddComponent(FComponent comp);

    private:

        std::vector<FComponent> Components;
    };

}