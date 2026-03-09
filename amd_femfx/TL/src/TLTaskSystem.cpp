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

#include "TLTaskSystem.h"

namespace AMD
{
    TL_THREAD_LOCAL_STORAGE int32_t gTLThreadIndex = -1; // -1 for thread that initialize task system; 1..n-1 for worker threads
    TL_THREAD_LOCAL_STORAGE TLTask gTLNextTask;
    TL_THREAD_LOCAL_STORAGE bool gTLNextTaskLoopActive = false;
    bool gTLTraceEnabled = false;

    TLTaskSystem* TLTaskSystem::pInstance = nullptr;

#if defined(_MSC_VER)
    uint32_t TLWorkerThread(void* inData)
#else
    void* TLWorkerThread(void* inData)
#endif
    {
        (void)inData;
        TLTaskSystem* taskSys = TLTaskSystem::Get();
        gTLThreadIndex = taskSys->ReserveWorkerIndex();

        // Loop to process tasks until a signal to quit.
        // When there are no tasks ready there is a back-off process:
        // The thread will spin-wait for a fixed number of iterations, yield, then repeat.
        // After a fixed amount of time without a task, the thread may sleep on a condition variable.

        const int32_t maxSpinsBeforeYield = 1000;
        const int64_t maxMicrosecondsWithoutTask = 10000;

        int32_t numSpins = 0;

        // Limit time without a task before sleeping
        TLTimer timeWithoutTask;

        timeWithoutTask.SetStartTime();
        while (true)
        {
            if (taskSys->TryProcessTask())
            {
                // Task processed, reset spin counter and time
                numSpins = 0;
#if TL_WAIT_COUNTER
                timeWithoutTask.SetStartTime();
#endif
            }
            else
            {
                if (++numSpins >= maxSpinsBeforeYield)
                {
                    numSpins = 0;

                    // Yield to another thread
                    TLYield();

#if TL_WAIT_COUNTER
                    if (timeWithoutTask.HasElapsedMicroseconds(maxMicrosecondsWithoutTask))
                    {
                        // Allow the thread to sleep if no task found, resuming the spin loop after sleep
                        while (true)
                        {
                            bool didSleep;
                            bool processedTask = taskSys->TryProcessTaskOrWait(&didSleep);

                            // If a task was found or if just woken, start over with spinning.
                            // Otherwise check quit signal
                            if (processedTask || didSleep)
                            {
                                timeWithoutTask.SetStartTime();
                                break;
                            }
                            else if (taskSys->GetQuitSignal())
                            {
                                TLExitJoinableThread();
                                return 0;
                            }
                        }
                    }
#else
                    // Check quit signal
                    if (taskSys->GetQuitSignal())
                    {
                        TLExitJoinableThread();
                        return 0;
                    }
#endif
                }

                TLPause();
            }
        }

        TLExitJoinableThread();
#if defined(_MSC_VER)
        return 0;
#endif
    }
}