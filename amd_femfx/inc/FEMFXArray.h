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
// Resizable array, has incomplete API but is just for limited uses in the lib.
//---------------------------------------------------------------------------------------

#pragma once

#include "FEMFXCommon.h"
#include <type_traits>

namespace AMD
{
    template< class T >
    class FmArray
    {
        T* pElems = nullptr;
        uint32_t numElems = 0;
        uint32_t maxElems = 0;

        inline void Extend(uint32_t inMaxElems)
        {
            FM_ASSERT(inMaxElems > numElems);
            T* pOldElems = pElems;
            pElems = (T*)FmAlignedMalloc(sizeof(T) * inMaxElems, FM_ALIGN_OF(T));
            for (uint32_t elemIdx = 0; elemIdx < numElems; elemIdx++)
            {
                new (pElems + elemIdx) T(std::move(pOldElems[elemIdx]));
                pOldElems[elemIdx].~T();
            }
            if (pOldElems != nullptr)
            {
                FmAlignedFree(pOldElems);
            }
            maxElems = inMaxElems;
        }

    public:
        FM_CLASS_NEW_DELETE(FmArray)

        inline FmArray() {}

        inline FmArray(const FmArray& other)
        {
            *this = other;
        }

        inline ~FmArray()
        {
            Clear();
        }

        inline FmArray& operator =(const FmArray& other)
        {
            Clear();
            uint otherNumElems = other.numElems;
            Reserve(otherNumElems);
            for (uint i = 0; i < otherNumElems; i++)
            {
                Add(other[i]);
            }
            return *this;
        }

        inline T* GetData()
        {
            return pElems;
        }

        inline uint32_t GetNumElems() const
        {
            return numElems;
        }

        inline T& operator[] (uint32_t i)
        {
            return pElems[i];
        }

        inline const T& operator[] (uint32_t i) const
        {
            return pElems[i];
        }

        inline void Add(const T& inElem)
        {
            if (numElems >= maxElems)
            {
                Extend(maxElems == 0 ? 1 : maxElems * 2);
            }
            pElems[numElems] = inElem;
            numElems++;
        }

        inline void Reserve(uint inMaxElems)
        {
            if (inMaxElems > maxElems)
            {
                Extend(inMaxElems);
            }
        }

        inline void Clear()
        {
            for (uint32_t i = 0; i < numElems; i++)
            {
                pElems[i].~T();
            }
            if (pElems != nullptr)
            {
                FmAlignedFree(pElems);
                pElems = nullptr;
            }
            numElems = 0;
            maxElems = 0;
        }
    };
}
