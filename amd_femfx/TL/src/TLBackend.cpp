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

#include "TLTaskSystemInterface.h"

#define USE_MULTITHREADING 1

#if USE_MULTITHREADING
#include "TLTaskSystem.h"
#include "TLParallelFor.h"
#include "TLAsyncThreading.h"
#else
#include <stdio.h>
#include "TLCommon.h"
#endif

namespace AMD
{
    int gTLTaskSystemNumThreads = 1;

    void TLInitTaskSystem(int numThreads)
    {
        if (numThreads == 0)
        {
            numThreads = TLGetTaskSystemDefaultNumThreads();
        }

        gTLTaskSystemNumThreads = numThreads;

#if USE_MULTITHREADING
        TLTaskSystem::Create(numThreads);
#else
        (void)numThreads;
#endif
    }

    void TLDestroyTaskSystem()
    {
#if USE_MULTITHREADING
        TLTaskSystem::Destroy();
#endif
    }

    void TLWaitForAllThreadsToStart()
    {
#if USE_MULTITHREADING
        TLTaskSystem::Get()->WaitForAllWorkersToStart();
#endif
    }

    int TLGetTaskSystemNumThreads()
    {
        return gTLTaskSystemNumThreads;
    }

    int TLGetTaskSystemDefaultNumThreads()
    {
#if USE_MULTITHREADING
        return TLTaskSystem::GetDefaultNumThreads();
#else
        return 1;
#endif
    }

    // Return index of current worker thread, >= 0 and < numThreads specified to TLInitTaskSystem(), unique between threads running concurrently.
    int TLGetTaskSystemThreadIndex()
    {
#if USE_MULTITHREADING
        TLTaskSystem* taskSys = TLTaskSystem::Get();
        return taskSys->GetThreadIndex();
#else
        return 0;
#endif
    }

    void TLSubmitAsyncTask(TLTaskFuncCallback TaskFunc, void* taskData, int32_t taskBeginIndex, int32_t taskEndIndex)
    {
#if USE_MULTITHREADING
        TLTaskSystem* taskSys = TLTaskSystem::Get();

        // If next-task currently set, convert to task to maintain last-in first-out order
        if (gTLNextTask.func != nullptr)
        {
            TLTask nextTask = gTLNextTask;
            gTLNextTask.func = nullptr;

            taskSys->SubmitTask(nextTask);
        }

        TLTask task;
        task.func = TaskFunc;
        task.data = taskData;
        task.beginIndex = taskBeginIndex;
        task.endIndex = taskEndIndex;

        taskSys->SubmitTask(task);
#else
        TaskFunc(taskData, taskBeginIndex, taskEndIndex);
#endif
    }

    TLSyncEvent* TLCreateSyncEvent()
    {
#if USE_MULTITHREADING
        TLCounter* taskCounter = new TLCounter();
        taskCounter->Increment();
        return taskCounter;
#else
        bool* eventFlag = new bool(false);
        return eventFlag;
#endif
    }

    void TLDestroySyncEvent(TLSyncEvent* taskEvent)
    {
#if USE_MULTITHREADING
        TLCounter* taskCounter = (TLCounter*)taskEvent;
        delete taskCounter;
#else
        bool* eventFlag = (bool *)taskEvent;
        delete eventFlag;
#endif
    }

    void TLWaitForSyncEvent(TLSyncEvent* taskEvent)
    {
#if USE_MULTITHREADING
        TLCounter* taskCounter = (TLCounter*)taskEvent;
        taskCounter->WaitUntilZero();
#else
        bool* eventFlag = (bool *)taskEvent;
        while (*eventFlag == false)
        {
        }
#endif
    }

    void TLTriggerSyncEvent(TLSyncEvent* taskEvent)
    {
#if USE_MULTITHREADING
        TLCounter* taskCounter = (TLCounter*)taskEvent;
        taskCounter->Decrement();
#else
        bool* eventFlag = (bool *)taskEvent;
        *eventFlag = true;
#endif
    }

    // This is not optimized, but enables running the synchronous-threaded version
    void TLWaitForTaskWaitCounter(TLTaskWaitCounter* counter)
    {
#if USE_MULTITHREADING
        // Have worker try to do more work before waiting
        TLTaskSystem* taskSystem = TLTaskSystem::Get();
        TLCounter* tlCounter = (TLCounter*)counter;

        if (taskSystem->IsWorkerThread())
        {
            while (tlCounter->GetCounter() > 0 && taskSystem->TryProcessTask())
            {
            }
        }

        tlCounter->WaitUntilZero();
#else
        (void)counter;
#endif
    }

    TLTaskWaitCounter* TLCreateTaskWaitCounter()
    {
#if USE_MULTITHREADING
        return new TLCounter();
#else
        return nullptr;
#endif
    }

    void TLDestroyTaskWaitCounter(TLTaskWaitCounter* counter)
    {
#if USE_MULTITHREADING
        TLCounter* tlCounter = (TLCounter*)counter;
        delete tlCounter;
#else
        (void)counter;
#endif
    }

    void TLIncrementTaskWaitCounter(TLTaskWaitCounter* counter)
    {
#if USE_MULTITHREADING
        TLCounter* tlCounter = (TLCounter*)counter;
        tlCounter->Increment();
#else
        (void)counter;
#endif
    }

    void TLDecrementTaskWaitCounter(TLTaskWaitCounter* counter)
    {
#if USE_MULTITHREADING
        TLCounter* tlCounter = (TLCounter*)counter;
        tlCounter->Decrement();
#else
        (void)counter;
#endif
    }

#if USE_MULTITHREADING
    // Task data that bundles a task with a wait counter
    struct TLTaskWithWaitCounterData
    {
        TL_CLASS_NEW_DELETE(TLTaskWithWaitCounterData);
        TLTaskFuncCallback taskFunc = nullptr;
        void* taskData = nullptr;
        TLCounter* waitCounter = nullptr;
    };

    // Run a task and decrement wait counter
    void TLRunTaskTaskWithWaitCounter(void* taskData, int32_t taskBeginIndex, int32_t taskEndIndex)
    {
        TLTaskWithWaitCounterData* taskAndCounter = (TLTaskWithWaitCounterData*)taskData;
        taskAndCounter->taskFunc(taskAndCounter->taskData, taskBeginIndex, taskEndIndex);
        taskAndCounter->waitCounter->Decrement();
        delete taskAndCounter;
    }
#endif

    void TLSubmitTask(TLTaskFuncCallback TaskFunc, void* taskData, int32_t taskBeginIndex, int32_t taskEndIndex, TLTaskWaitCounter* waitCounter)
    {
#if USE_MULTITHREADING
        TLTaskSystem* taskSystem = TLTaskSystem::Get();

        if (waitCounter)
        {
            ((TLCounter*)waitCounter)->Increment();

            TLTaskWithWaitCounterData* taskAndCounter = new TLTaskWithWaitCounterData();
            taskAndCounter->taskFunc = TaskFunc;
            taskAndCounter->taskData = taskData;
            taskAndCounter->waitCounter = (TLCounter*)waitCounter;

            TLTask task;
            task.func = TLRunTaskTaskWithWaitCounter;
            task.data = taskAndCounter;
            task.beginIndex = taskBeginIndex;
            task.endIndex = taskEndIndex;

            taskSystem->SubmitTask(task);
        }
        else
        {
            TLTask task;
            task.func = TaskFunc;
            task.data = taskData;
            task.beginIndex = taskBeginIndex;
            task.endIndex = taskEndIndex;

            taskSystem->SubmitTask(task);
        }
#else
        (void)waitCounter;
        TaskFunc(taskData, taskBeginIndex, taskEndIndex);
#endif
    }

    void TLFlushTaskQueue()
    {
#if USE_MULTITHREADING
        TLTaskSystem* taskSystem = TLTaskSystem::Get();

        if (taskSystem->IsWorkerThread())
        {
            bool foundTask;
            do
            {
                foundTask = false;

                while (gTLNextTask.func != nullptr)
                {
                    TLTaskFuncCallback NextTaskFunc = gTLNextTask.func;
                    gTLNextTask.func = nullptr;
                    NextTaskFunc(gTLNextTask.data, gTLNextTask.beginIndex, gTLNextTask.endIndex);
                    foundTask = true;
                }

                if (taskSystem->TryProcessThreadQueueTask())
                {
                    foundTask = true;
                }
            } while (foundTask);
        }
#endif
    }
}

