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

#include "FEMFXCommon.h"
#include "TL.h"

namespace AMD
{
#if FM_ASYNC_THREADING
#define FM_ASYNC_TASK TL_ASYNC_TASK
#define FM_ASYNC_TASK_COUNTED TL_ASYNC_TASK_COUNTED
#define FM_ASYNC_TASK_COUNTED_WITH_DELETE TL_ASYNC_TASK_COUNTED_WITH_DELETE
#else
// Synchronous version of code does not use TLSetNextTask or track completed tasks
#define FM_ASYNC_TASK(Name) void Name(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
#define FM_ASYNC_TASK_COUNTED(FuncName, DataName) void FuncName(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
#define FM_ASYNC_TASK_COUNTED_WITH_DELETE(FuncName, DataName) void FuncName(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
#endif
#define FM_NODE_TASK TL_NODE_TASK

    // Get number of tasks assuming maximum batch size per task
    static FM_FORCE_INLINE uint FmGetNumTasks(uint problemSize, uint maxTaskBatchSize)
    {
        return (problemSize + maxTaskBatchSize - 1) / maxTaskBatchSize;
    }

    // Get number of tasks based on desired batch size per task, but limited to at most maxTasks
    static FM_FORCE_INLINE uint FmGetNumTasksLimited(uint problemSize, uint taskBatchSize, uint maxTasks)
    {
        uint numTasks = (problemSize + taskBatchSize - 1) / taskBatchSize;
        numTasks = FmMinUint(maxTasks, numTasks);
        return numTasks;
    }

    // Get problem index range for the specified task index, assuming TLGetNumTasks() tasks
    static FM_FORCE_INLINE void FmGetIndexRange(uint* beginIndex, uint* endIndex, uint taskIndex, uint maxTaskBatchSize, uint problemSize)
    {
        uint begin = taskIndex * maxTaskBatchSize;
        uint end = begin + maxTaskBatchSize;
        begin = FmMinUint(begin, problemSize);
        end = FmMinUint(end, problemSize);
        *beginIndex = begin;
        *endIndex = end;
    }

    // Get number of tasks for a minimum batch size per task, assuming remainder will be evenly distributed to all tasks.
    static FM_FORCE_INLINE uint FmGetNumTasksMinBatchSize(uint problemSize, uint minTaskBatchSize)
    {
        return FmMaxUint(problemSize / minTaskBatchSize, 1);
    }

    static FM_FORCE_INLINE void FmGetIndexRangeEvenDistribution(uint* beginIndex, uint* endIndex, uint taskIndex, uint taskCount, uint problemSize)
    {
        TLGetIndexRangeEvenDistributionT<uint>(beginIndex, endIndex, taskIndex, taskCount, problemSize);
    }
}
