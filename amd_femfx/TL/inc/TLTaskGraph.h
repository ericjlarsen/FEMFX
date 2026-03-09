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
// A task graph that lets you specify a set of tasks and dependencies between them.
// Running task will message dependent nodes and nodes with all dependencies met are
// submitted to scheduler. Includes event type that allows tasks to dynamically
// message other nodes.
//---------------------------------------------------------------------------------------

#pragma once

#include "TLTaskSystemInterface.h"
#include "TLArray.h"

namespace AMD
{
    class TLTaskGraph;

    // Node in a task graph that contains a task function and links to successor nodes.
    // When the graph calls the task function it passes a pointer the to TLTaskGraphNode.
    // Additional data can be specified when initializing the node, accessible by node->GetTaskData(),
    // or by adding data to a graph derived from TLTaskGraph, accessible through node->GetGraph().
    // The nodeFunc task should be declared with TL_NODE_TASK or TL_ASYNC_TASK.
    // TL_NODE_TASK will automatically call WorkItemFinished to mark completion and start any successors.
    // TL_ASYNC_TASK can be used if the task calls WorkItemFinished manually or it calls a second TL_NODE_TASK task.
    class TLTaskGraphNode
    {
        TLAtomicInt numPredecessorsIncomplete;  // num predecessors still incomplete
        int32_t numPredecessors = 0;            // num predecessors that must be completed before executing this node

        TLTaskFuncCallback nodeFunc = nullptr;
        void*              nodeData = nullptr;
        int32_t            nodeIndex = 0;

        int32_t predecessorMessage = -1; // Value stored by last completed predecessor

        TLArray<TLTaskGraphNode*> successors;
        TLTaskGraph* graph = nullptr;

        // Called to signal completion to successor node, which may be ready to run.
        // If ppNextNode non-null, and *ppNextNode initialized to nullptr, one of the ready successors will be returned for this thread to run.
        // The remaining ready nodes will be submitted tasks.
        void SignalSuccessors(int32_t message)
        {
            int numSuccessors = (int)successors.GetNumElems();
            for (int i = 0; i < numSuccessors; i++)
            {
                successors[i]->PredecessorComplete(message);
            }
        }

    public:
        TL_CLASS_NEW_DELETE(TLTaskGraphNode)

        TLTaskGraphNode() {}

        TLTaskGraphNode(TLTaskGraph* inGraph, TLTaskFuncCallback inNodeTaskFunc, void* inNodeTaskData, int32_t inNodeIndex)
        {
            Init(inGraph, inNodeTaskFunc, inNodeTaskData, inNodeIndex);
        }

        void Init(TLTaskGraph* inGraph, TLTaskFuncCallback inNodeTaskFunc, void* inNodeTaskData, int32_t inNodeIndex)
        {
            numPredecessorsIncomplete.val = 0;
            numPredecessors = 0;

            nodeFunc = inNodeTaskFunc;
            nodeData = inNodeTaskData;
            nodeIndex = inNodeIndex;

            predecessorMessage = -1;

            graph = inGraph;
        }

        inline TLTaskFuncCallback GetTaskFunc() { return nodeFunc; }
        inline void* GetTaskData() { return nodeData; }
        inline int32_t GetIndex() { return nodeIndex; }
        TLTaskGraph* GetGraph() { return graph; }
        inline int32_t GetPredecessorMessage() const { return predecessorMessage; }

        void AddSuccessor(TLTaskGraphNode* node)
        {
            successors.Add(node);
            node->IncrementNumPredecessors();
        }

        void SetNumPredecessors(int32_t inNumPredecessors)
        {
            numPredecessors = inNumPredecessors;
            numPredecessorsIncomplete.val = inNumPredecessors;
        }

        void IncrementNumPredecessors()
        {
            numPredecessors++;
            numPredecessorsIncomplete.val++;
        }

        // Called to signal completion to successor node, which may be ready to run.
        inline void PredecessorComplete(int32_t message);

        // Called from scheduled task
        inline void Run();

        // Signal successors and update graph progress
        inline void Finished(int32_t message);
    };

    // An event is just a list of successor nodes that can be signaled.
    // These can be used to implement some dynamic branching in the task graph.
    class TLTaskGraphEvent
    {
        TLArray<TLTaskGraphNode*> successors;

    public:
        TL_CLASS_NEW_DELETE(TLTaskGraphEvent)

        void AddSuccessor(TLTaskGraphNode* node)
        {
            successors.Add(node);
            node->IncrementNumPredecessors();
        }

        // Called to signal completion to successor node, which may be ready to run.
        void SignalSuccessors(int32_t message)
        {
            int numSuccessors = (int)successors.GetNumElems();
            for (int i = 0; i < numSuccessors; i++)
            {
                successors[i]->PredecessorComplete(message);
            }
        }

        int32_t GetNumSuccessors()
        {
            return successors.GetNumElems();
        }

        TLArray<TLTaskGraphNode*>& GetSuccessors()
        {
            return successors;
        }
    };

    // A task graph that can be run either with a wait until completion, or asychronously with a follow-up task to run once completion is detected.
    class TLTaskGraph
    {
        TLTaskDataBase progress;
        TLTaskWaitCounter* waitCounter = nullptr; // Count of running nodes, after Start() value of 0 will signify graph is complete
        TLTaskGraphEvent   startEvent;
        bool               isAsync = true;

        void Start(bool inIsAsync)
        {
            isAsync = inIsAsync;

            if (!isAsync)
            {
                waitCounter = TLCreateTaskWaitCounter();
                TLIncrementTaskWaitCounter(waitCounter);
            }

            // The active node count is always incremented by one before a node is submitted, and that node won't decrement until it has
            // finished incrementing the count by one for each of the successor nodes that it submits. This means that the initial
            // increment can't be canceled out until all successive work for that node has completed.
            // By the same logic we first increment by one before signalling the start nodes, and decrement after.
            NodeStarting();

            // Signal starting nodes
            startEvent.SignalSuccessors(0);

            NodeFinished();

            if (!isAsync)
            {
                TLWaitForTaskWaitCounter(waitCounter);
                TLDestroyTaskWaitCounter(waitCounter);
                waitCounter = nullptr;
            }
        }

    public:
        TL_CLASS_NEW_DELETE(TLTaskGraph)

        void Destroy()
        {
            startEvent.GetSuccessors().Clear();
        }

        void AddToStart(TLTaskGraphNode* node)
        {
            startEvent.AddSuccessor(node);
        }

        void SubmitNodeTask(TLTaskGraphNode* node)
        {
            NodeStarting();

            if (isAsync)
            {
                // Using TLSetNextTask since the thread will be free to run this task after starting any ready successors.
                // TLSetNextTask is safe if called multiple times before running next-task (current next-task submitted with TLSubmitAsyncTask)
                TLSetNextTask(node->GetTaskFunc(), node, node->GetIndex(), node->GetIndex() + 1);
            }
            else
            {
                TLSubmitAsyncTask(node->GetTaskFunc(), node, node->GetIndex(), node->GetIndex() + 1);
            }
        }

        // Signal start and wait for graph to complete
        void StartAndWait()
        {
            Start(false);
        }

        // Start graph using asynchronous threading. Provide a task to run when complete
        void StartAsync(const TLTask& followTask = TLTask())
        {
            progress.Init(0, followTask);
            Start(true);
        }

        // Must use to increment running task count before submitting a task
        void NodeStarting()
        {
            progress.WorkItemStarting();
        }

        // Indicates task is complete, and must call from a node's task function
        void NodeFinished()
        {
            if (isAsync)
            {
                progress.WorkItemFinished();
            }
            else
            {
                if (progress.WorkItemFinished())
                {
                    TLDecrementTaskWaitCounter(waitCounter);
                }
            }
        }
    };

    inline void TLTaskGraphNode::PredecessorComplete(int32_t message)
    {
        int32_t newNumPredecessorsIncomplete = TLAtomicDecrement(&numPredecessorsIncomplete.val);

        TL_ASSERT(newNumPredecessorsIncomplete >= 0);

        // If this was the last predecessor being waited on, enqueue the task.
        // It will signal successors after it is run
        if (newNumPredecessorsIncomplete == 0)
        {
            predecessorMessage = message;

            // Reset num predecessors for subsequent iteration, before submitting
            TLAtomicWrite(&numPredecessorsIncomplete.val, numPredecessors);

            graph->SubmitNodeTask(this);
        }
    }

    inline void TLTaskGraphNode::Run()
    {
        nodeFunc(this, nodeIndex, 0);
    }

    inline void TLTaskGraphNode::Finished(int32_t message)
    {
        SignalSuccessors(message);
        graph->NodeFinished();
    }

    // Utility to modify a task function for TLTaskGraph
    // Adds completed tasks, which may delete task data and call TLSetNextTask.
    template<void (*Func)(void*, int32_t, int32_t)>
    TL_FORCE_INLINE void TLNodeTaskFunc(void* inTaskData, int32_t inBeginIndex, int32_t inEndIndex)
    {
        // Check if the task should run a loop to catch TLSetNextTask
        bool runLoop = !gTLNextTaskLoopActive;
        gTLNextTaskLoopActive = true;

        // Run the task function
        TLTaskGraphNode* node = (TLTaskGraphNode*)inTaskData;
        Func(node, inBeginIndex, inEndIndex);

        node->Finished(0);

        // Run all next tasks directly, skipping task queue
        if (runLoop)
        {
            while (gTLNextTask.func != nullptr)
            {
                TLTaskFuncCallback NextTaskFunc = gTLNextTask.func;
                gTLNextTask.func = nullptr;
                NextTaskFunc(gTLNextTask.data, gTLNextTask.beginIndex, gTLNextTask.endIndex);
            }
            gTLNextTaskLoopActive = false;
        }
    }

    // Macro to apply the TLNodeTaskFunc modification
#define TL_NODE_TASK(Name) \
    TL_FORCE_INLINE void Name##Impl(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex); \
    void Name(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex) \
    { \
        TLNodeTaskFunc<Name##Impl>(inTaskData, inTaskBeginIndex, inTaskEndIndex); \
    } \
    TL_FORCE_INLINE void Name##Impl(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)

}
