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

#include "TLParallelFor.h"
#include "TLAsyncThreading.h"
#include "TLTaskSystemInterface.h"

namespace AMD
{
    // Data needed for a thread to dispatch a portion of parallel-for tasks
    class TLParallelForDispatcherData : public TLTaskDataBase
    {
    public:
        TL_CLASS_NEW_DELETE(TLParallelForDispatcherData);

        TLTaskWaitCounter* waitCounter;

        TLTaskFuncCallback TaskFunc;
        TLGrainFuncCallback GrainFunc;
        TLTaskDataBase* taskData;
        int32_t beginIndex;
        int32_t endIndex;
        int32_t numThreads;
        int32_t numDispatchers;
        int32_t grainSize;
        bool deleteTaskData;

        TLParallelForDispatcherData(
            TLTaskFuncCallback inTaskFunc,
            TLGrainFuncCallback inGrainFunc,
            TLTaskDataBase* inTaskData,
            int32_t inBeginIndex,
            int32_t inEndIndex,
            int32_t inNumThreads,
            int32_t inNumDispatchers,
            int32_t inGrainSize = 1,
            bool inDeleteTaskData = false)
        {
            waitCounter = nullptr;
            TaskFunc = inTaskFunc;
            GrainFunc = inGrainFunc;
            taskData = inTaskData;
            beginIndex = inBeginIndex;
            endIndex = TLMaxInt(beginIndex, inEndIndex);
            numThreads = inNumThreads;
            numDispatchers = inNumDispatchers;
            grainSize = inGrainSize;
            deleteTaskData = inDeleteTaskData;
            TL_ASSERT(numDispatchers <= (endIndex - beginIndex));
        }

        ~TLParallelForDispatcherData()
        {
            if (deleteTaskData)
                delete taskData;
        }
    };

    void TLTaskFuncParallelForNextBatch(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);
    void TLTaskFuncParallelForAsyncNextBatch(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    template <bool IsAsync>
    TL_FORCE_INLINE void TLTaskFuncParallelForNextBatchImpl(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        TLParallelForDispatcherData* dispatcherData = (TLParallelForDispatcherData*)inTaskData;

        int32_t batchBeginIndex = inTaskBeginIndex;
        int32_t rangeEndIndex = inTaskEndIndex;

        TLGrainFuncCallback GrainFunc = dispatcherData->GrainFunc;
        TL_ASSERT(GrainFunc);
        void* taskData = dispatcherData->taskData;

        int32_t batchSize = (int32_t)GrainFunc(taskData, batchBeginIndex, rangeEndIndex);
        int32_t batchEndIndex = batchBeginIndex + batchSize;

        if (batchEndIndex < rangeEndIndex)
        {
            dispatcherData->WorkItemStarting();

            TLSubmitAsyncTask(IsAsync? TLTaskFuncParallelForAsyncNextBatch : TLTaskFuncParallelForNextBatch, dispatcherData, batchEndIndex, rangeEndIndex);
        }

        dispatcherData->TaskFunc(taskData, batchBeginIndex, batchEndIndex);

        if (IsAsync)
        {
            dispatcherData->WorkItemFinished(dispatcherData);
        }
        else
        {
            if (dispatcherData->WorkItemFinished())
            {
                TLDecrementTaskWaitCounter(dispatcherData->waitCounter);
            }
        }
    }

    void TLTaskFuncParallelForNextBatch(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        TLTaskFuncParallelForNextBatchImpl<false>(inTaskData, inTaskBeginIndex, inTaskEndIndex);
    }

    void TLTaskFuncParallelForAsyncNextBatch(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        TLAsyncTaskFunc<TLTaskFuncParallelForNextBatchImpl<true>>(inTaskData, inTaskBeginIndex, inTaskEndIndex);
    }

    template <bool IsAsync>
    TL_FORCE_INLINE void TLTaskFuncParallelForBatchingDispatcherImpl(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;

        TLParallelForDispatcherData* dispatcherData = (TLParallelForDispatcherData*)inTaskData;

        int32_t dispatcherIndex = inTaskBeginIndex;
        int32_t problemBeginIndex = dispatcherData->beginIndex;
        int32_t problemEndIndex = dispatcherData->endIndex;
        int32_t numDispatchers = dispatcherData->numDispatchers;
        int32_t problemSize = problemEndIndex - problemBeginIndex;

        // Compute sub-range of work items this dispatcher covers
        int32_t rangeBeginIndex, rangeEndIndex;
        TLGetIndexRangeEvenDistribution(&rangeBeginIndex, &rangeEndIndex, dispatcherIndex, numDispatchers, problemSize);
        rangeBeginIndex += problemBeginIndex;
        rangeEndIndex += problemBeginIndex;

        // Compute a batch size
        TLGrainFuncCallback GrainFunc = dispatcherData->GrainFunc;
        TL_ASSERT(GrainFunc);
        void* taskData = dispatcherData->taskData;

        int32_t batchSize = GrainFunc(taskData, rangeBeginIndex, rangeEndIndex);
        int32_t batchBeginIndex = rangeBeginIndex;
        int32_t batchEndIndex = batchBeginIndex + batchSize;

        // Submit another task to continue batching remaining elements
        if (batchEndIndex < rangeEndIndex)
        {
            dispatcherData->WorkItemStarting();

            TLSubmitAsyncTask(IsAsync ? TLTaskFuncParallelForAsyncNextBatch : TLTaskFuncParallelForNextBatch, dispatcherData, batchEndIndex, rangeEndIndex);
        }

        // Process work items
        dispatcherData->TaskFunc(taskData, batchBeginIndex, batchEndIndex);

        if (IsAsync)
        {
            dispatcherData->WorkItemFinished(dispatcherData);
        }
        else
        {
            if (dispatcherData->WorkItemFinished())
            {
                TLDecrementTaskWaitCounter(dispatcherData->waitCounter);
            }
        }
    }

    void TLTaskFuncParallelForBatchingDispatcher(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        TLTaskFuncParallelForBatchingDispatcherImpl<false>(inTaskData, inTaskBeginIndex, inTaskEndIndex);
    }

    void TLTaskFuncParallelForAsyncBatchingDispatcher(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        TLAsyncTaskFunc<TLTaskFuncParallelForBatchingDispatcherImpl<true>>(inTaskData, inTaskBeginIndex, inTaskEndIndex);
    }

    template <bool IsAsync>
    TL_FORCE_INLINE void TLTaskFuncParallelForProcessSubRangeImpl(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;

        TLParallelForDispatcherData* dispatcherData = (TLParallelForDispatcherData*)inTaskData;

        int32_t dispatcherIndex = inTaskBeginIndex;
        int32_t problemBeginIndex = dispatcherData->beginIndex;
        int32_t problemEndIndex = dispatcherData->endIndex;
        int32_t numDispatchers = dispatcherData->numDispatchers;
        int32_t grainSize = dispatcherData->grainSize;
        void* taskData = dispatcherData->taskData;
        int32_t problemSize = problemEndIndex - problemBeginIndex;

        // Compute sub-range of work items to process
        int32_t rangeBeginIndex, rangeEndIndex;
        if (grainSize > 0)
        {
            // Use grainSize to limit range of each TaskFunc call
            TLGetIndexRangeEvenDistribution(&rangeBeginIndex, &rangeEndIndex, dispatcherIndex, numDispatchers, (problemSize + grainSize - 1) / grainSize);
            rangeBeginIndex *= grainSize;
            rangeEndIndex *= grainSize;
            rangeBeginIndex += problemBeginIndex;
            rangeEndIndex += problemBeginIndex;
            rangeEndIndex = TLMinInt(rangeEndIndex, problemEndIndex);

            int32_t taskBeginIndex = rangeBeginIndex;
            int32_t taskEndIndex = TLMinInt(taskBeginIndex + grainSize, rangeEndIndex);
            while (taskBeginIndex < rangeEndIndex)
            {
                dispatcherData->TaskFunc(taskData, taskBeginIndex, taskEndIndex);
                taskBeginIndex = taskEndIndex;
                taskEndIndex = TLMinInt(taskBeginIndex + grainSize, rangeEndIndex);
            }
        }
        else
        {
            TLGetIndexRangeEvenDistribution(&rangeBeginIndex, &rangeEndIndex, dispatcherIndex, numDispatchers, problemSize);
            rangeBeginIndex += problemBeginIndex;
            rangeEndIndex += problemBeginIndex;

            dispatcherData->TaskFunc(taskData, rangeBeginIndex, rangeEndIndex);
        }

        if (IsAsync)
        {
            dispatcherData->WorkItemFinished(dispatcherData);
        }
        else
        {
            if (dispatcherData->WorkItemFinished())
            {
                TLDecrementTaskWaitCounter(dispatcherData->waitCounter);
            }
        }
    }

    void TLTaskFuncParallelForProcessSubRange(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        TLTaskFuncParallelForProcessSubRangeImpl<false>(inTaskData, inTaskBeginIndex, inTaskEndIndex);
    }

    void TLTaskFuncParallelForProcessSubRangeAsync(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        TLAsyncTaskFunc<TLTaskFuncParallelForProcessSubRangeImpl<true>>(inTaskData, inTaskBeginIndex, inTaskEndIndex);
    }

    void TLTaskFuncParallelForDispatcher(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);
    void TLTaskFuncParallelForAsyncDispatcher(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex);

    template <bool IsAsync>
    TL_FORCE_INLINE void TLTaskFuncParallelForDispatcherImpl(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        (void)inTaskEndIndex;

        TLParallelForDispatcherData* dispatcherData = (TLParallelForDispatcherData*)inTaskData;
        int32_t beginIndex = inTaskBeginIndex;
        int32_t grainSize = dispatcherData->grainSize;
        int32_t numThreads = dispatcherData->numThreads;
        int32_t problemEndIndex = dispatcherData->endIndex;

        // Submit additional dispatcher task if not all submitted.
        int32_t nextIndex = beginIndex + numThreads * grainSize;
        if (nextIndex < problemEndIndex)
        {
            TLSubmitAsyncTask(IsAsync? TLTaskFuncParallelForAsyncDispatcher : TLTaskFuncParallelForDispatcher, dispatcherData, nextIndex, 0);
        }

        int32_t endIndex = TLMinInt(beginIndex + grainSize, problemEndIndex);
        dispatcherData->TaskFunc(dispatcherData->taskData, beginIndex, endIndex);

        if (IsAsync)
        {
            dispatcherData->WorkItemFinished(dispatcherData);
        }
        else
        {
            if (dispatcherData->WorkItemFinished())
            {
                TLDecrementTaskWaitCounter(dispatcherData->waitCounter);
            }
        }
    }

    void TLTaskFuncParallelForDispatcher(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        TLTaskFuncParallelForDispatcherImpl<false>(inTaskData, inTaskBeginIndex, inTaskEndIndex);
    }

    void TLTaskFuncParallelForAsyncDispatcher(void* inTaskData, int32_t inTaskBeginIndex, int32_t inTaskEndIndex)
    {
        TLAsyncTaskFunc<TLTaskFuncParallelForDispatcherImpl<true>>(inTaskData, inTaskBeginIndex, inTaskEndIndex);
    }

    void TLParallelForAsync(TLTaskFuncCallback TaskFunc, TLTaskDataBase* taskData, int32_t beginIndex, int32_t endIndex, const TLParallelForOptions& options, const TLTask& followTask, bool deleteTaskData)
    {
        int32_t numElements = endIndex - beginIndex;

        if (numElements <= 0)
        {
            if (followTask.func)
            {
                TLSetNextTask(followTask.func, followTask.data, followTask.beginIndex, followTask.endIndex);
            }
            if (deleteTaskData)
            {
                delete taskData;
            }
            return;
        }

        int32_t numThreads = TLGetTaskSystemNumThreads();

        if (options.grainFunc || options.split)
        {
            // Split range based on thread count, submit tasks to process sub-range items or batch items with grainFunc
            int32_t numDispatchers = numThreads * options.threadScale;

            const int32_t minSubRangeSize = options.grainFunc ? 16 : TLMaxInt(options.grainSize, 1);
            int32_t numSubRanges = TLGetNumTasks((int32_t)numElements, minSubRangeSize);
            numDispatchers = TLMinInt(numSubRanges, numDispatchers);

            TLParallelForDispatcherData* dispatcherData = new TLParallelForDispatcherData(TaskFunc, options.grainFunc, taskData, beginIndex, endIndex, numThreads, numDispatchers, options.grainSize, deleteTaskData);
            dispatcherData->Init(numDispatchers, followTask);

            TLTaskFuncCallback DispatcherFunc = options.grainFunc ? TLTaskFuncParallelForAsyncBatchingDispatcher : TLTaskFuncParallelForProcessSubRangeAsync;

            int32_t numToSubmit = numDispatchers;
            for (int32_t i = 1; i < numToSubmit; i++)
            {
                TLSubmitAsyncTask(DispatcherFunc, dispatcherData, i, i + 1);
            }

            DispatcherFunc(dispatcherData, 0, 1);
        }
        else
        {
            // Submit tasks that will each process grainSize items and submit another task if necessary
            int32_t grainSize = options.grainSize;
            int32_t numDispatchers = (numElements + grainSize - 1) / grainSize;

            TLParallelForDispatcherData* dispatcherData = new TLParallelForDispatcherData(TaskFunc, nullptr, taskData, beginIndex, endIndex, numThreads, numDispatchers, grainSize, deleteTaskData);
            dispatcherData->Init(numDispatchers, followTask);

            int numToSubmit = TLMinInt(numThreads, numDispatchers);
            for (int32_t i = 1; i < numToSubmit; i++)
            {
                TLSubmitAsyncTask(TLTaskFuncParallelForAsyncDispatcher, dispatcherData, beginIndex + i * grainSize, 0);
            }

            TLTaskFuncParallelForAsyncDispatcher(dispatcherData, beginIndex, 0);
        }
    }

    void TLParallelFor(TLTaskFuncCallback TaskFunc, void* taskData, int32_t beginIndex, int32_t endIndex, const TLParallelForOptions& options)
    {
        int32_t numElements = endIndex - beginIndex;

        if (numElements <= 0)
        {
            return;
        }

        int32_t numThreads = TLGetTaskSystemNumThreads();

        if (options.grainFunc || options.split)
        {
            // Split range based on thread count, submit tasks to process sub-range items or batch items with grainFunc
            int32_t numDispatchers = numThreads * options.threadScale;

            const int32_t minSubRangeSize = options.grainFunc ? 16 : TLMaxInt(options.grainSize, 1);
            int32_t numSubRanges = TLGetNumTasks((int32_t)numElements, minSubRangeSize);
            numDispatchers = TLMinInt(numSubRanges, numDispatchers);

            TLParallelForDispatcherData dispatcherData(TaskFunc, options.grainFunc, reinterpret_cast<TLTaskDataBase*>(taskData), beginIndex, endIndex, numThreads, numDispatchers, options.grainSize, false);
            dispatcherData.Init(numDispatchers, TLTask());

            TLTaskWaitCounter* waitCounter = TLCreateTaskWaitCounter();
            TLIncrementTaskWaitCounter(waitCounter);
            dispatcherData.waitCounter = waitCounter;

            TLTaskFuncCallback DispatcherFunc = options.grainFunc ? TLTaskFuncParallelForBatchingDispatcher : TLTaskFuncParallelForProcessSubRange;

            int32_t numToSubmit = numDispatchers;
            for (int32_t i = 1; i < numToSubmit; i++)
            {
                TLSubmitAsyncTask(DispatcherFunc, &dispatcherData, i, i + 1);
            }

            DispatcherFunc(&dispatcherData, 0, 1);

            TLWaitForTaskWaitCounter(waitCounter);
            TLDestroyTaskWaitCounter(waitCounter);
        }
        else
        {
            // Submit tasks that will each process grainSize items and submit another task if necessary
            int32_t grainSize = options.grainSize;
            int32_t numDispatchers = (numElements + grainSize - 1) / grainSize;

            TLParallelForDispatcherData dispatcherData(TaskFunc, nullptr, reinterpret_cast<TLTaskDataBase*>(taskData), beginIndex, endIndex, numThreads, numDispatchers, grainSize, false);
            dispatcherData.Init(numDispatchers, TLTask());

            TLTaskWaitCounter* waitCounter = TLCreateTaskWaitCounter();
            TLIncrementTaskWaitCounter(waitCounter);
            dispatcherData.waitCounter = waitCounter;

            int numToSubmit = TLMinInt(numThreads, numDispatchers);
            for (int32_t i = 1; i < numToSubmit; i++)
            {
                TLSubmitAsyncTask(TLTaskFuncParallelForDispatcher, &dispatcherData, beginIndex + i * grainSize, 0);
            }

            TLTaskFuncParallelForDispatcher(&dispatcherData, beginIndex, 0);

            TLWaitForTaskWaitCounter(waitCounter);
            TLDestroyTaskWaitCounter(waitCounter);
        }
    }
}
