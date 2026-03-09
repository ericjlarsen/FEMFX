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

#pragma once

#include "AMD_FEMFX.h"
#include "FEMFXThreading.h"

namespace AMD
{
    struct FmConstraintSolverData;
    struct FmConstraintIsland;

    struct FmTaskGraphSolveData
    {
        FmScene* scene = nullptr;
        FmConstraintSolverData* constraintSolverData = nullptr;
        const FmConstraintIsland* constraintIsland = nullptr;
    };

#if FM_CONSTRAINT_ISLAND_DEPENDENCY_GRAPH
    // State per each partition-pair node needed for repeating iteration or calling subsequent tasks
    class FmPartitionPairConstraintNodeState
    {
    public:
        FM_CLASS_NEW_DELETE(FmPartitionPairConstraintNodeState)

        uint passIdx = 0;
        uint innerIteration = 0;
        uint outerIteration = 0;
        uint numPredecessorsSameIteration = 0;
        uint numTimesReached = 0;
        bool hasSameIterationPredecessorWithRigidBodies = false;

        TLTaskGraphNode firstIterationNode;  // Node for task in the first iteration; start node may link to this
        TLTaskGraphNode repeat0Node;         // Alternating between two copies of nodes to support PGS solve inner loop
        TLTaskGraphNode repeat1Node;
        TLTaskGraphNode nextOuterIterationNode;  // Node for task in the next outer iteration

        TLTaskGraphEvent nextIteration0Successors; // Successor partition pairs, which are messaged if continuing to iterate the PGS constraint solve
        TLTaskGraphEvent nextIteration1Successors; // Successor partition pairs, which are messaged if continuing to iterate the PGS constraint solve
        TLTaskGraphEvent partitionMpcgSuccessors;  // Successor partitions, which are messaged if moving to object GS or MPCG solving
        TLTaskGraphEvent partitionGsSuccessors;    // Successor partitions, which are messaged if moving to object GS or MPCG solving
    };

    // Graph nodes for partition object solving tasks.
    class FmPartitionObjectNodeState
    {
    public:
        FM_CLASS_NEW_DELETE(FmPartitionObjectNodeState)

        TLTaskGraphNode  gsIterationRbDeltaNode;
        TLTaskGraphNode  mpcgSolveNode;
        TLTaskGraphEvent partitionPairSuccessors;     // Successor partition pairs, when restarting outer iteration
    };

    // Contains the task graph and nodes needed for one outer iteration of constraint solve.
    class FmConstraintSolveTaskGraph : public TLTaskGraph
    {
    public:
        FM_CLASS_NEW_DELETE(FmConstraintSolveTaskGraph)

        FmTaskGraphSolveData                 solveData;
        FmPartitionPairConstraintNodeState*  partitionPairConstraintNodes = nullptr;
        FmPartitionObjectNodeState*          partitionObjectNodes = nullptr;
        uint                                 numPartitions = 0;
        uint                                 numPartitionPairs = 0;
    };

    // Create task graph
    void FmCreateConstraintSolveTaskGraph(FmScene* scene, FmConstraintSolverData* constraintSolverData, const FmConstraintIsland* constraintIsland);

    // Free task graph
    void FmDestroyConstraintSolveTaskGraph(FmConstraintSolveTaskGraph* taskGraph);

    // Run task graph to execute one outer iteration of constraint solve.
    void FmRunConstraintSolveTaskGraph(FmConstraintSolveTaskGraph* taskGraph);

    // Run task graph to execute both CG and GS passes.
    // This will skip over the Start, Middle, End tasks that used to implement the outer loops and transition between passes.  
    // Loop counters and other control parameters are accessible or updated by the graph. 
    // The graph will start with the GS pass if there are no CG pass iterations.
    void FmRunConstraintSolveTaskGraphAsync(FmConstraintSolveTaskGraph* taskGraph, TLTaskFuncCallback followTask, void* followTaskData);

    // Make edge from start node to partition pair node.
    void FmMakeStartDependency(FmConstraintSolveTaskGraph* graph, uint partitionPairIdx);

    // Make edge from partition pair node to different node in the same PGS iteration.
    void FmMakeSameIterationDependency(FmConstraintSolveTaskGraph* graph, uint partitionPairAIdx, uint partitionPairBIdx);

    // Register dependency between partition pair node and node in the next PGS iteration.
    void FmMakeNextIterationDependency(FmConstraintSolveTaskGraph* graph, uint partitionPairAIdx, uint partitionPairBIdx);

    // Register dependency between partition pair node and partition node in the next solving phase.
    void FmMakePartitionTaskDependency(FmConstraintSolveTaskGraph* graph, uint partitionPairIdx, uint partitionIdx);

    // Register dependency between partition node and partition pair node in the next outer solve iteration
    void FmMakeNextOuterIterationDependency(FmConstraintSolveTaskGraph* graph, uint partitionIdx, uint partitionPairIdx);

    // Send messages to partition pair nodes in next iteration.
    void FmNextIterationMessages(FmConstraintSolveTaskGraph* graph, uint partitionPairIdx, uint iteration);

    // Send messages to partition nodes to run MPCG.
    void FmPartitionMpcgTaskMessages(FmConstraintSolveTaskGraph* graph, uint partitionPairIdx, uint outerIteration);

    // Send messages to partition nodes to run GS iteration.
    void FmPartitionGsTaskMessages(FmConstraintSolveTaskGraph* graph, uint partitionPairIdx, uint outerIteration);

    // Send messages to partition pair nodes in next outer iteration.
    void FmNextOuterIterationMessages(FmConstraintSolveTaskGraph* graph, uint partitionIdx);
#endif
}