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

#include "TLCommon.h"
#include <type_traits>

namespace AMD
{
    template< class T, int N = 1 >
    class TLArray
    {
        T* pElems = nullptr;
        T firstElems[N];
        int32_t numElems = 0;
        int32_t maxElems = 0;

        inline void Extend(int32_t inMaxElems)
        {
            TL_ASSERT(inMaxElems > numElems);
            T* pOldElems = pElems;
            pElems = (T*)TL_ALIGNED_MALLOC(sizeof(T) * inMaxElems, TL_ALIGN_OF(T));
            for (int32_t elemIdx = 0; elemIdx < numElems; elemIdx++)
            {
                new (pElems + elemIdx) T(std::move(pOldElems[elemIdx]));
                pOldElems[elemIdx].~T();
            }
            if (pOldElems != firstElems)
            {
                TL_ALIGNED_FREE(pOldElems);
            }
            maxElems = inMaxElems;
        }

    public:
        TL_CLASS_NEW_DELETE(TLArray)

        inline TLArray() {}

        inline TLArray(const TLArray& other)
        {
            *this = other;
        }

        inline ~TLArray()
        {
            Clear();
        }

        inline TLArray& operator =(const TLArray& other)
        {
            Clear();
            int32_t otherNumElems = other.numElems;
            Reserve(otherNumElems);
            for (int32_t i = 0; i < otherNumElems; i++)
            {
                Add(other[i]);
            }
            return *this;
        }

        inline T* GetData()
        {
            return pElems;
        }

        inline int32_t GetNumElems() const
        {
            return numElems;
        }

        inline T& operator[] (int32_t i)
        {
            return pElems[i];
        }

        inline const T& operator[] (int32_t i) const
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

        inline void Reserve(int32_t inMaxElems)
        {
            if (inMaxElems > maxElems)
            {
                Extend(inMaxElems);
            }
        }

        inline void Clear()
        {
            for (int32_t i = 0; i < numElems; i++)
            {
                pElems[i].~T();
            }
            if (pElems != firstElems)
            {
                TL_ALIGNED_FREE(pElems);
                pElems = firstElems;
            }
            numElems = 0;
            maxElems = N;
        }
    };
}
