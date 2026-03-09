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
// Task system with multiple task queues and a pool of worker threads
//---------------------------------------------------------------------------------------

#pragma once

#include "TLCommon.h"
#include "TLTaskQueue.h"

#define TL_WAIT_COUNTER    1  // Enable condition variable waiting (otherwise only uses spin-waits/yields)

namespace AMD
{
    // Thread-local state initialized when worker thread runs
    extern TL_THREAD_LOCAL_STORAGE int32_t gTLThreadIndex;

    extern TL_THREAD_LOCAL_STORAGE TLTask gTLNextTask;

    // Worker thread loop
#if defined(_MSC_VER)
    uint32_t TLWorkerThread(void *inData);
#else
    void* TLWorkerThread(void* inData);
#endif

    // For detecting core count
    extern int32_t TLGetProcessorInfo(int32_t* pNumPhysicalCores, int32_t* pNumLogicalCores);

    // Task system that creates a pool of worker threads that each own a task queue, but may also take tasks from other queues.
    // NOTE: The implementation assumes that only one other thread outside the thread pool submits tasks.
    //
    // When tasks run out, worker threads will start a back-off process that includes spin-waits, yields, and waiting on a condition variable.
    //
    // Best performance was seen by having workers remove tasks from the end of their queues, but steal from the beginning of other queues.
    class TLTaskSystem
    {
    private:
        TLAtomicInt numWorkersStarted; // Atomic count of started workers to give each a unique index
        TLAtomicInt quitSignal;        // Atomic flag workers check to quit
        TLAtomicInt numTasks;          // Total number of tasks in all queues, checked before waiting on condition var

        // Counter/condition var is created to allow sleeping when there are insufficient tasks.
        // A value <= 0 indicates tasks available, and >0 unavailable.
        // These only need to be modified when task counts reach certain thresholds, reducing number of critical section locks.
        // Note increment/decrement operations may occur out of order and so be out of sync with the availability of tasks.
        // However as threads complete counter values will settle.
            
#if TL_WAIT_COUNTER
        TLCounter waitCounter;   // Condition var to put threads to sleep
#endif

        int32_t numWorkerThreads = 1;

        int32_t numPhysicalCores = 1;
        int32_t numLogicalCores = 1;

        TLThread* workerThreads = nullptr;
        TLTaskQueueList* taskQueues = nullptr;   // Each worker and main thread has a task queue (a list of them to support overflow); workers can steal from other queues

        static TLTaskSystem* pInstance;   // For singleton

        TLTaskSystem(int32_t inNumWorkerThreads, int32_t inNumPhysicalCores, int32_t inNumLogicalCores) 
        {
            pInstance = this;

            numWorkersStarted.val = 0;
            quitSignal.val = 0;
            numTasks.val = 0;

            numPhysicalCores = inNumPhysicalCores;
            numLogicalCores = inNumLogicalCores;

            CheckSetupParams(&numWorkerThreads, inNumWorkerThreads, inNumLogicalCores);

#if TL_WAIT_COUNTER
            waitCounter.Increment();
#endif
            workerThreads = new TLThread[numWorkerThreads];
            taskQueues = new TLTaskQueueList[numWorkerThreads + 1];

            for (int i = 0; i < numWorkerThreads; i++)
            {
                TLCreateJoinableThread(&workerThreads[i], TLWorkerThread, this, 0, "TLTaskSystem Worker Thread");
            }
        }

        ~TLTaskSystem()
        {
#if TL_WAIT_COUNTER
            // Wake workers and prevent additional waiting
            waitCounter.Decrement();
#endif

            // Set value threads are checking to quit
            TLAtomicWrite(&quitSignal.val, 1);

            for (int i = 0; i < numWorkerThreads; i++)
            {
                TLJoinThread(workerThreads[i]);
            }

            delete[] workerThreads;

            delete[] taskQueues;
        }

        TLTaskSystem(const TLTaskSystem &) = delete;
        TLTaskSystem& operator=(const TLTaskSystem &) = delete;

        TL_FORCE_INLINE int32_t GetNumTasks()
        {
            return TLAtomicRead(&numTasks.val);
        }

        // Decrement num tasks and trigger sleep if 0
        TL_FORCE_INLINE void DecrementNumTasks()
        {
            int32_t newNumTasks = TLAtomicDecrement(&numTasks.val);

#if TL_WAIT_COUNTER
            // If removing last task, set wait condition
            if (newNumTasks == 0)
            {
                waitCounter.Increment();
            }
#else
            (void)newNumTasks;
#endif
        }

        // Increment num tasks and trigger wake if 1
        TL_FORCE_INLINE void IncrementNumTasks()
        {
            int32_t newNumTasks = TLAtomicIncrement(&numTasks.val);

#if TL_WAIT_COUNTER
            // If adding first task, wake threads
            if (newNumTasks == 1)
            {
                waitCounter.Decrement();
            }
#else
            (void)newNumTasks;
#endif
        }

        // Try to submit task.
        // Only safe for one thread to submit at a time.
        TL_FORCE_INLINE bool TrySubmitTaskAtEnd(const TLTask& task, int32_t queueIndex)
        {
            return taskQueues[queueIndex].TrySubmitTaskAtEnd(task);
        }

        // Try to find task to run on worker thread's queue
        TL_FORCE_INLINE bool TryClaimThreadQueueTask(TLTask* task)
        {
            if (GetNumTasks() == 0)
            {
                return false;
            }

            int32_t workerIndex = gTLThreadIndex;

            if (taskQueues[workerIndex].TryClaimTaskFromEnd(task))
            {
                DecrementNumTasks();
                return true;
            }

            return false;
        }

        // Try to find task to run.
        // Multiple threads can call this concurrently
        TL_FORCE_INLINE bool TryClaimTask(TLTask* task)
        {
            if (GetNumTasks() == 0)
            {
                return false;
            }

            // Check own queue first.  Claim from end to improve locality.
            int32_t workerIndex = gTLThreadIndex;

            if (taskQueues[workerIndex].TryClaimTaskFromEnd(task))
            {
                DecrementNumTasks();
                return true;
            }

            // Check main thread queue first
            if (taskQueues[numWorkerThreads].TryClaimTaskFromBeginning(task))
            {
                DecrementNumTasks();
                return true;
            }

            // Steal from other worker queues.
            for (int32_t offset = 1; offset < numWorkerThreads; offset++)
            {
                if (GetNumTasks() == 0)
                {
                    return false;
                }

                int32_t qIdx = workerIndex + offset;
                qIdx = (qIdx >= numWorkerThreads) ? qIdx - numWorkerThreads: qIdx;

                if (taskQueues[qIdx].TryClaimTaskFromBeginning(task))
                {
                    DecrementNumTasks();
                    return true;
                }
            }

            return false;
        }

        static TL_FORCE_INLINE void CheckSetupParams(
            int32_t* outNumWorkerThreads,
            int32_t inNumWorkerThreads,
            int32_t inNumLogicalCores)
        {
            int32_t maxWorkerThreads = inNumLogicalCores;
            int32_t numWorkerThreads = (inNumWorkerThreads <= 0 || inNumWorkerThreads > maxWorkerThreads) ? maxWorkerThreads : inNumWorkerThreads;

            *outNumWorkerThreads = numWorkerThreads;
        }

    public:
        TL_CLASS_NEW_DELETE(TLTaskSystem)

        // Create task system and worker threads.
        // If inNumWorkerThreads is 0, sets number of workers to GetDefaultNumThreads().
        static void Create(int32_t inNumWorkerThreads)
        {
            // Get processor info
            int32_t numPhysical, numLogical;
            TLGetProcessorInfo(&numPhysical, &numLogical);

            // Check if input parameters match the current instance
            int32_t numWorkers;
            CheckSetupParams(&numWorkers, inNumWorkerThreads, numLogical);

            if (pInstance == nullptr 
                || pInstance->numWorkerThreads != numWorkers)
            {
                delete pInstance;
                pInstance = new TLTaskSystem(numWorkers, numPhysical, numLogical);
            }
        }

        // Shutdown worker threads and destroy resource
        static void Destroy()
        {
            delete pInstance;
            pInstance = nullptr;
        }

        // Get singleton
        static TLTaskSystem* Get()
        {
            return pInstance;
        }        

        TL_FORCE_INLINE int32_t GetNumWorkerThreads()
        {
            return numWorkerThreads;
        }

        TL_FORCE_INLINE int32_t GetNumPhysicalCores()
        {
            return numPhysicalCores;
        }

        TL_FORCE_INLINE int32_t GetNumLogicalCores()
        {
            return numLogicalCores;
        }

        static TL_FORCE_INLINE int32_t GetDefaultNumThreads()
        {
            int32_t numPhysicalCores, numLogicalCores;
            TLGetProcessorInfo(&numPhysicalCores, &numLogicalCores);
            return numLogicalCores;
        }

        // Called by worker thread on creation to reserve a worker index
        TL_FORCE_INLINE int32_t ReserveWorkerIndex()
        {
            return TLAtomicIncrement(&numWorkersStarted.val) - 1;
        }

        TL_FORCE_INLINE void WaitForAllWorkersToStart()
        {
            while (TLAtomicRead(&numWorkersStarted.val) < numWorkerThreads)
            {
                TLPause();
            }
        }

        // Return current thread index.
        // -1 for thread on which task system initialized
        // >= 0 and < numWorkerThreads for worker threads
        TL_FORCE_INLINE int32_t GetThreadIndex()
        {
            return gTLThreadIndex;
        }

        // Only worker threads will process tasks
        TL_FORCE_INLINE bool IsWorkerThread()
        {
            return (gTLThreadIndex >= 0);
        }

        // Called by worker thread to check for signal to quit
        TL_FORCE_INLINE int32_t GetQuitSignal()
        {
            return TLAtomicRead(&quitSignal.val);
        }

        // Process a submitted task, or sleep until signal that tasks available.
        // After wake-up will return whether task can be claimed or not.
        // Returns if task was processed.
        // NOTE: Should only be called by worker thread.
        bool TryProcessTaskOrWait(bool* didSleep)
        {
            *didSleep = false;

#if TL_WAIT_COUNTER
            if (GetNumTasks() == 0)
            {
                // Sleep on condition var, but if woken, return to poll for new tasks, in case more about to be added.
                *didSleep = waitCounter.WaitOneWakeup();
            }
#endif

            return TryProcessTask();
        }

        // Process submitted task on thread's queue if available.
        // Return if task was processed.
        // NOTE: Should only be called by worker thread.
        TL_FORCE_INLINE bool TryProcessThreadQueueTask()
        {
            TLTask task;
            if (TryClaimThreadQueueTask(&task))
            {
                // Run claimed task
                task.func(task.data, task.beginIndex, task.endIndex);

                return true;
            }

            return false;
        }

        // Process a submitted task if available.
        // Return if task was processed.
        // NOTE: Should only be called by worker thread.
        TL_FORCE_INLINE bool TryProcessTask()
        {
            TLTask task;
            if (TryClaimTask(&task))
            {
                // Run claimed task
                task.func(task.data, task.beginIndex, task.endIndex);

                TL_ASSERT(gTLNextTask.func == nullptr);

                return true;
            }

            return false;
        }

        // Called by worker thread to submit task to task system.
        // May be called by one thread outside of task system (such as main thread).
        void SubmitTask(const TLTask& task)
        {
            int32_t queueIndex = gTLThreadIndex;
            if (queueIndex == -1)
            {
                queueIndex = numWorkerThreads;
            }
            while(!TrySubmitTaskAtEnd(task, queueIndex))
            {
            }

            IncrementNumTasks();
        }
    };
}