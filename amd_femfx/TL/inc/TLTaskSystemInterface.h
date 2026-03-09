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

#include <stdint.h>

// Interface to backend task-system implementation, including task queues and synchronization primitives
namespace AMD
{
    typedef void(*TLTaskFuncCallback)(void* taskData, int32_t taskBeginIndex, int32_t taskEndIndex);

    // Task system initialization
    void TLInitTaskSystem(int numThreads);
    void TLDestroyTaskSystem();

    int32_t TLGetTaskSystemNumThreads();
    int32_t TLGetTaskSystemDefaultNumThreads();

    void TLWaitForAllThreadsToStart();

    // Return index of current worker thread, between 0 and numThreads-1, unique between workers running concurrently
    int32_t TLGetTaskSystemThreadIndex();

    typedef void TLSyncEvent;

    // Submit asynchronous task to task scheduler, which will invoke TaskFunc with taskData and taskIndex arguments
    void TLSubmitAsyncTask(TLTaskFuncCallback TaskFunc, void* taskData, int32_t taskBeginIndex, int32_t taskEndIndex);

    // Call at the end of an asynchronous task to set a follow-up task, when all prerequisites complete.
    // (Lower overhead than TLSubmitAsyncTask for this case)
    void TLSetNextTask(TLTaskFuncCallback taskFunc, void* taskData, int32_t taskBeginIndex, int32_t taskEndIndex);

    // Create synchronization event
    TLSyncEvent* TLCreateSyncEvent();

    // Destroy synchronization event
    void TLDestroySyncEvent(TLSyncEvent* taskEvent);

    // Wait for synchronization event to be triggered
    void TLWaitForSyncEvent(TLSyncEvent* taskEvent);

    // Trigger synchronization event
    void TLTriggerSyncEvent(TLSyncEvent* taskEvent);

    typedef void TLTaskWaitCounter;

    // Create counter with count indicating unblocked
    TLTaskWaitCounter* TLCreateTaskWaitCounter();

    // Increment wait counter
    void TLIncrementTaskWaitCounter(TLTaskWaitCounter* counter);

    // Decrement wait counter
    void TLDecrementTaskWaitCounter(TLTaskWaitCounter* counter);

    // Wait on counter
    void TLWaitForTaskWaitCounter(TLTaskWaitCounter* counter);

    // Destroy counter
    void TLDestroyTaskWaitCounter(TLTaskWaitCounter* counter);

    // Submit task to task scheduler which runs TaskFunc with taskData and taskIndex arguments.
    // If waitCounter non-null, this call will increment counter, and task will decrement on completion
    void TLSubmitTask(TLTaskFuncCallback TaskFunc, void* taskData, int32_t taskBeginIndex, int32_t taskEndIndex, TLTaskWaitCounter* waitCounter);

    // Run any tasks on worker thread's task queue
    void TLFlushTaskQueue();
}
