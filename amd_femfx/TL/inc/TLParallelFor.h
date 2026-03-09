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

#include "TLTaskQueue.h"
#include "TLCounter.h"
#include "TLAsyncThreading.h"

namespace AMD
{
    // Callback function which can be provided to TLParallelFor or TLParallelForAsync for workers to adjust granularity of tasks submitted to the task system
    typedef int32_t(*TLGrainFuncCallback)(void* taskData, int32_t taskBeginIndex, int32_t taskEndIndex);

    // Options for TLParallelFor or TLParallelForAsync.
    // TODO: gives a few manual ways to set task granularity, but no automatic option
    struct TLParallelForOptions
    {
        TLGrainFuncCallback grainFunc = nullptr; // If non-null applies this function to get a count of elements to process
        int32_t grainSize = 1;                   // If grainFunc null, sets fixed grain; calls TLTaskFuncCallback with at most grainSize elements
        bool split = false;                      // If grainFunc null, splits work items evenly among threads
        int32_t threadScale = 1;                 // For grainFunc or split, create up to threadScale * numThreads tasks

        TLParallelForOptions() {}
        TLParallelForOptions(TLGrainFuncCallback inGrainFunc, int32_t inGrainSize, bool inSplit, int32_t inThreadScale) :
            grainFunc(inGrainFunc), grainSize(inGrainSize), split(inSplit), threadScale(inThreadScale) { }

        static TLParallelForOptions GrainFunc(TLGrainFuncCallback grainFunc)
        {
            return TLParallelForOptions(grainFunc, 1, false, 1);
        }
        static TLParallelForOptions GrainSize(int32_t grainSize)
        {
            return TLParallelForOptions(nullptr, grainSize, false, 1);
        }
        static TLParallelForOptions Split(int32_t threadScale, int32_t grainSize = 1)
        {
            return TLParallelForOptions(nullptr, grainSize, true, threadScale);
        }
    };

    // Synchronous/blocking parallel-for.
    void TLParallelFor(TLTaskFuncCallback TaskFunc, void* taskData,
        int32_t beginIndex, int32_t endIndex, const TLParallelForOptions& options = TLParallelForOptions::GrainSize(1));

    // Asynchronous parallel-for.
    // Optionally can specify follow-task and delete task data when work items processed.
    void TLParallelForAsync(TLTaskFuncCallback TaskFunc, TLTaskDataBase* taskData,
        int32_t beginIndex, int32_t endIndex, const TLParallelForOptions& options = TLParallelForOptions::GrainSize(1), const TLTask& followTask = TLTask(), bool deleteTaskData = false);

    // Get number of tasks assuming maximum batch size per task
    static TL_FORCE_INLINE int32_t TLGetNumTasks(int32_t problemSize, int32_t maxTaskBatchSize)
    {
        return (problemSize + maxTaskBatchSize - 1) / maxTaskBatchSize;
    }

    // Get number of tasks based on desired batch size per task, but limited to at most maxTasks
    static TL_FORCE_INLINE int32_t TLGetNumTasksLimited(int32_t problemSize, int32_t taskBatchSize, int32_t maxTasks)
    {
        int32_t numTasks = (problemSize + taskBatchSize - 1) / taskBatchSize;
        numTasks = TLMinInt(maxTasks, numTasks);
        return numTasks;
    }

    // Get problem index range for the specified task index, assuming TLGetNumTasks() tasks
    static TL_FORCE_INLINE void TLGetIndexRange(int32_t* beginIndex, int32_t* endIndex, int32_t taskIndex, int32_t maxTaskBatchSize, int32_t problemSize)
    {
        int32_t begin = taskIndex * maxTaskBatchSize;
        int32_t end = begin + maxTaskBatchSize;
        begin = TLMinInt(begin, problemSize);
        end = TLMinInt(end, problemSize);
        *beginIndex = begin;
        *endIndex = end;
    }

    // Get number of tasks for a minimum batch size per task, assuming remainder will be evenly distributed to all tasks.
    static TL_FORCE_INLINE int32_t TLGetNumTasksMinBatchSize(int32_t problemSize, int32_t minTaskBatchSize)
    {
        return TLMaxInt(problemSize / minTaskBatchSize, 1);
    }

    // Get problem index range for the specified task index, assuming problem is distributed to tasks as evenly as possible.
    // NOTE: output range may be zero-sized if problemSize < taskCount
    template<typename T>
    static TL_FORCE_INLINE void TLGetIndexRangeEvenDistributionT(T* beginIndex, T* endIndex, T taskIndex, T taskCount, T problemSize)
    {
        TL_ASSERT(taskCount > 0 && taskCount <= problemSize && taskIndex < taskCount);

        T taskBatchSize = problemSize / taskCount;
        T remainderBatchSize = problemSize % taskCount;

        T taskExtra = remainderBatchSize / taskCount;
        T remainder = remainderBatchSize % taskCount;

        taskBatchSize += taskExtra;

        T begin, end;
        if (taskIndex < remainder)
        {
            taskBatchSize++;
            begin = taskIndex * taskBatchSize;
        }
        else
        {
            begin = remainder * (taskBatchSize + 1) + (taskIndex - remainder) * taskBatchSize;
        }

        end = begin + taskBatchSize;

        *beginIndex = begin;
        *endIndex = end;
    }

    static TL_FORCE_INLINE void TLGetIndexRangeEvenDistribution(int32_t* beginIndex, int32_t* endIndex, int32_t taskIndex, int32_t taskCount, int32_t problemSize)
    {
        TLGetIndexRangeEvenDistributionT<int32_t>(beginIndex, endIndex, taskIndex, taskCount, problemSize);
    }
}