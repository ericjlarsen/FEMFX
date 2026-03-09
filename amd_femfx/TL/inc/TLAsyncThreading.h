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
// Support for asynchronous approach for task scheduling, where dispatched tasks
// track progress and issue follow-up tasks after work complete.
//---------------------------------------------------------------------------------------

#pragma once

#include "TLCommon.h"
#include "TLTaskSystemInterface.h"

namespace AMD
{
    extern TL_THREAD_LOCAL_STORAGE TLTask gTLNextTask;
    extern TL_THREAD_LOCAL_STORAGE bool gTLNextTaskLoopActive;

    // TLSetNextTask
    // With asynchronous multi-threading, tasks that process a set of work items also check for completion of the work,
    // and the one that detects this will submit the next task. The next task could be submitted to the task queue system,
    // but often the same thread will claim this task. To shortcut this, TLSetNextTask caches the task in a thread-local
    // variable, checked by a function lower in the callstack, to reduce stack growth.
    //
    // For this optimization, some tasks need to be modified to check the thread-local variable, which can be done with the
    // TL_ASYNC_TASK macros below. Note the modification isn't required for a task run with TLParallelForAsync.
    TL_FORCE_INLINE void TLSetNextTask(TLTaskFuncCallback TaskFunc, void* taskData, int32_t taskBeginIndex, int32_t taskEndIndex)
    {
        if (gTLNextTaskLoopActive)
        {
            // If there's an existing next task submit it.
            // Loop in case the submit may set next task.
            while (gTLNextTask.func != nullptr)
            {
                TLTask nextTask = gTLNextTask;
                gTLNextTask.func = nullptr;
                TLSubmitAsyncTask(nextTask.func, nextTask.data, nextTask.beginIndex, nextTask.endIndex);
            }

            // Set next task
            gTLNextTask = TLTask(TaskFunc, taskData, taskBeginIndex, taskEndIndex);
        }
        else
        {
            TL_ASSERT(0);

            // If gTLNextTaskLoopActive not set, just submit
            TLSubmitAsyncTask(TaskFunc, taskData, taskBeginIndex, taskEndIndex);
        }
    }

    // Utility to modify a task function so TLSetNextTask may be called from this or child tasks
    template<void (*Func)(void*, int32_t, int32_t)>
    TL_FORCE_INLINE void TLAsyncTaskFunc(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        // Check if the task should run a loop to catch TLSetNextTask
        bool runLoop = !gTLNextTaskLoopActive;
        gTLNextTaskLoopActive = true;

        // Run the task function
        Func(inTaskData, inTaskBeginIndex, inTaskEndIndex);

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

    // Utility to modify a task function so TLSetNextTask may be called from this or child tasks.
    // Also modifies the task to call the input data's WorkItemsFinished, which may call TLSetNextTask.
    // NOTE: assumes task data has interface derived from TLTaskDataBase.
    template<void (*Func)(void*, int32_t, int32_t), typename Data>
    TL_FORCE_INLINE void TLAsyncTaskFuncCounted(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        // Check if the task should run a loop to catch TLSetNextTask
        bool runLoop = !gTLNextTaskLoopActive;
        gTLNextTaskLoopActive = true;

        // Run the task function
        Func(inTaskData, inTaskBeginIndex, inTaskEndIndex);

        // Record finished work and if complete, set the next task and delete task data
        Data* data = (Data*)inTaskData;
        data->WorkItemsFinished(inTaskEndIndex - inTaskBeginIndex);

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

    // Utility to modify a task function so TLSetNextTask may be called from this or child tasks.
    // Also modifies the task to call the input data's WorkItemsFinished, which may delete the task data and call TLSetNextTask.
    // NOTE: assumes task data has interface derived from TLTaskDataBase.
    template<void (*Func)(void*, int32_t, int32_t), typename Data>
    TL_FORCE_INLINE void TLAsyncTaskFuncCountedWithDelete(void* inData, int32_t inBeginIndex, int32_t inEndIndex)
    {
        // Check if the task should run a loop to catch TLSetNextTask
        bool runLoop = !gTLNextTaskLoopActive;
        gTLNextTaskLoopActive = true;

        // Run the task function
        Func(inData, inBeginIndex, inEndIndex);

        // Record finished work and if complete, set the next task and delete task data
        Data* data = (Data*)inData;
        data->WorkItemsFinished(inEndIndex - inBeginIndex, data);

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

    // Macro to apply the TLAsyncTaskFunc modification
#define TL_ASYNC_TASK(Name) \
    TL_FORCE_INLINE void Name##Impl(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex); \
    void Name(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex) \
    { \
        TLAsyncTaskFunc<Name##Impl>(inTaskData, inTaskBeginIndex, inTaskEndIndex); \
    } \
    TL_FORCE_INLINE void Name##Impl(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)

    // Macro to apply the TLAsyncTaskFuncCounted modification
#define TL_ASYNC_TASK_COUNTED(FuncName, DataName) \
    TL_FORCE_INLINE void FuncName##Impl(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex); \
    void FuncName(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex) \
    { \
        TLAsyncTaskFuncCounted<FuncName##Impl, DataName>(inTaskData, inTaskBeginIndex, inTaskEndIndex); \
    } \
    TL_FORCE_INLINE void FuncName##Impl(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)

    // Macro to apply the TLAsyncTaskFuncCountedWithDelete modification
#define TL_ASYNC_TASK_COUNTED_WITH_DELETE(FuncName, DataName) \
    TL_FORCE_INLINE void FuncName##Impl(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex); \
    void FuncName(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex) \
    { \
        TLAsyncTaskFuncCountedWithDelete<FuncName##Impl, DataName>(inTaskData, inTaskBeginIndex, inTaskEndIndex); \
    } \
    TL_FORCE_INLINE void FuncName##Impl(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)

    // State to track progress of asynchronously processed work and continue execution.
    // Holds an atomic count of active work items, incremented before new work items are submitted, and
    // decremented after work items are complete. In the asynchronous threading pattern, there is no waiting thread,
    // and the final task completed is responsible for submitting a follow-up task and deleting temporary data.
    class TLTaskDataBase
    {
        TLAtomicInt nextWorkItemIndex;   // Atomic index used to start processing work items in sorted order
        TLAtomicInt numActiveWorkItems;  // Atomic count of active work items to detect completion
        TLTask followTask;               // Optional task to submit following completion of work items

    public:
        TL_CLASS_NEW_DELETE(TLTaskDataBase);

        TLTaskDataBase() { }

        // virtual for proper deletion of task data derived from this
        virtual ~TLTaskDataBase()
        {
            TL_ASSERT(TLAtomicRead(&numActiveWorkItems.val) <= 0);
        }

        void Init(int32_t inNumWorkItemsToRun, const TLTask& inFollowTask)
        {
            nextWorkItemIndex.val = 0;
            numActiveWorkItems.val = inNumWorkItemsToRun;
            followTask = inFollowTask;
        }

        void ResetNextWorkItemIndex()
        {
            TLAtomicWrite(&nextWorkItemIndex.val, 0);
        }

        int32_t GetNextWorkItemIndex()
        {
            return TLAtomicIncrement(&nextWorkItemIndex.val) - 1;
        }

        int32_t GetNumActiveWorkItems()
        {
            return TLAtomicRead(&numActiveWorkItems.val);
        }

        // Increment number of active work items.
        void WorkItemStarting()
        {
            TLAtomicIncrement(&numActiveWorkItems.val);
        }

        // Add to number of active work items.
        void WorkItemsStarting(int32_t numItems)
        {
            TLAtomicAdd(&numActiveWorkItems.val, numItems);
        }

        // Decrement number of active work items.
        // If this is last work item, delete task data, and call TLSetNextTask on follow task.
        // This TLTaskDataBase can belong to the task data deleted.
        // Return whether all work items are finished.
        template<class T>
        bool WorkItemsFinished(int32_t numFinishedItems, T* taskDataToDelete)
        {
            int32_t numActiveItems = TLAtomicSubtract(&numActiveWorkItems.val, numFinishedItems);

            if (numActiveItems == 0)
            {
                if (followTask.func)
                {
                    TLSetNextTask(followTask.func, followTask.data, followTask.beginIndex, followTask.endIndex);
                }

                if (taskDataToDelete)  // task data might include this TLTaskDataBase, but done reading
                {
                    delete taskDataToDelete;
                }

                return true;
            }
            else
            {
                return false;
            }
        }

        template<class T>
        bool WorkItemFinished(T* taskDataToDelete)
        {
            return WorkItemsFinished(1, taskDataToDelete);
        }

        bool WorkItemsFinished(int32_t numFinishedItems)
        {
            int32_t numActiveItems = TLAtomicSubtract(&numActiveWorkItems.val, numFinishedItems);

            if (numActiveItems == 0)
            {
                if (followTask.func)
                {
                    TLSetNextTask(followTask.func, followTask.data, followTask.beginIndex, followTask.endIndex);
                }

                return true;
            }
            else
            {
                return false;
            }
        }

        bool WorkItemFinished()
        {
            return WorkItemsFinished(1);
        }
    };
}
