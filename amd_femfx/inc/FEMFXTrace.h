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

#include "minitrace.h"

namespace AMD
{
    extern bool gFEMFXTraceEnabled;

    class FmTraceScopedEvent
    {
        const char* name;
    public:
        FM_FORCE_INLINE FmTraceScopedEvent(const char* inName) : name(inName)
        {
            if (gFEMFXTraceEnabled)
            {
                MTR_BEGIN("FEMFX", name);
            }
        }
        FM_FORCE_INLINE ~FmTraceScopedEvent()
        {
            if (gFEMFXTraceEnabled)
            {
                MTR_END("FEMFX", name);
            }
        }
    };
}

#define FM_INIT_TRACE() mtr_init("femfx_trace.json")
#define FM_SHUTDOWN_TRACE() mtr_shutdown()
#define FM_ENABLE_TRACE() gFEMFXTraceEnabled = true
#define FM_DISABLE_TRACE() gFEMFXTraceEnabled = false
#define FM_TRACE_START_EVENT(name) if (gFEMFXTraceEnabled) MTR_BEGIN("FEMFX", name);
#define FM_TRACE_STOP_EVENT(name) if (gFEMFXTraceEnabled) MTR_END("FEMFX", name);
#define FM_TRACE_SCOPED_EVENT(name) FmTraceScopedEvent __FmTraceScopedEvent(name)
#define FM_TRACE_START_FRAME()
#define FM_TRACE_STOP_FRAME()
