/*************************************************************************
 *                                                                       *
 * ODER's Utilities Library. Copyright (C) 2008 Oleh Derevenko.          *
 * All rights reserved.  e-mail: odar@eleks.com (change all "a" to "e")  *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 3 of the License, or (at    *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE-LESSER.TXT. Since LGPL is the extension of GPL     *
 *       the text of GNU General Public License is also provided for     *
 *       your information in file LICENSE.TXT.                           *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *   (3) The zlib/libpng license that is included with this library in   *
 *       the file LICENSE-ZLIB.TXT                                       *
 *                                                                       *
 * This library is distributed WITHOUT ANY WARRANTY, including implied   *
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.      *
 * See the files LICENSE.TXT and LICENSE-LESSER.TXT or LICENSE-BSD.TXT   *
 * or LICENSE-ZLIB.TXT for more details.                                 *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode.ou;

import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.Common.dNODEBUG;

// CEnumSortedElementArray definition

/*
 *	Implementation Note:
 *	The array is intended to store static constant data.
 *	Therefore CElementEqualType should not ever need a nontrivial constructor
 *	and it is acceptable to have it as template parameter.
 * template<
 *      typename EnumType,
 *      const EnumType EnumMax,
 *      typename ElementType,
 *      const int Instance=0,
 *      class CElementLessType=CTypeStandardLess<ElementType> >
 */
public class CEnumSortedElementArray
{
    //public:
    //_OU_INLINE _OU_CONVENTION_METHOD
    public CEnumSortedElementArray(int[] m_aetElementArray, int EnumMax)
    {
        //    #if !defined(NDEBUG)
        //
        //    #if _OU_COMPILER != _OU_COMPILER_GCC || _OU_COMPILER_VERSION == _OU_COMPILER_VERSION_GCCLT4
        //
        //    OU_ASSERT(OU_ARRAY_SIZE(m_aetElementArray) == EnumMax);
        //
        //
        //    #endif // #if _OU_COMPILER != _OU_COMPILER_GCC || _OU_COMPILER_VERSION == _OU_COMPILER_VERSION_GCCLT4
        if (!dNODEBUG) {
            dIASSERT(m_aetElementArray.length == EnumMax);

		    //final CElementLessType ltElementLess = CElementLessType();

            for (int nElementIndex = 1; nElementIndex < EnumMax; ++nElementIndex) {
                //OU_ASSERT(ltElementLess(m_aetElementArray[nElementIndex - 1], m_aetElementArray[nElementIndex])); //
                dIASSERT(m_aetElementArray[nElementIndex - 1] < m_aetElementArray[nElementIndex]); //
                // Element values must be sorted
            }
        }
        this.m_aetElementArray = m_aetElementArray;
       //#endif // #if !defined(NDEBUG)
    }

    //    static _OU_ALWAYSINLINE const EnumType _OU_CONVENTION_API
    //    /*const EnumType */Decode(const ElementType &etValue)
    public int Decode(final int etValue)
    {
		//const CElementLessType ltElementLess = CElementLessType();

        //EnumType etResult = EnumMax;
        int etResult = m_aetElementArray.length;

		//const ElementType *itElementFound = FindValueLowerBound(m_aetElementArray, m_aetElementArray + EnumMax, etValue);
        final int itElementFound = FindValueLowerBound(etValue);

        //if (itElementFound != m_aetElementArray + EnumMax)
        if (itElementFound != m_aetElementArray.length)
        {
            //if (!ltElementLess(etValue, *itElementFound))
            if (! (etValue < m_aetElementArray[itElementFound]))
            {
                //etResult = (EnumType)(itElementFound - m_aetElementArray);
                etResult = itElementFound;
            }
        }

        return etResult;
    }

    //    static _OU_ALWAYSINLINE const ElementType &_OU_CONVENTION_API
    //    /*const ElementType &*/Encode(const EnumType &etValue)
    public int Encode(final int etValue)
    {
        //    OU_ASSERT(sizeof(EnumType) <= sizeof(int));
        //    OU_ASSERT(OU_IN_INT_RANGE(etValue, 0, EnumMax));
        return m_aetElementArray[etValue];
    }

    //    static _OU_ALWAYSINLINE bool _OU_CONVENTION_API
    //    /*bool */IsValidDecode(const EnumType &etValue)
    public boolean IsValidDecode(final int etValue)
    {
        //return etValue != EnumMax;
        return etValue != m_aetElementArray.length;
    }

    //    static _OU_ALWAYSINLINE const ElementType *_OU_CONVENTION_API
    //    /*const ElementType **/GetStoragePointer()
    public int[] GetStoragePointer()
    {
        return m_aetElementArray;
    }

    //private:
    //static const ElementType *_OU_CONVENTION_API FindValueLowerBound(const ElementType *petArrayBegin, const ElementType *petArrayEnd, const ElementType &etValue)
    private int FindValueLowerBound(final int etValue)
    {
		//const CElementLessType ltElementLess = CElementLessType();
        //const ElementType *petCurrentRangeBegin = petArrayBegin;
        //const ElementType *petCurrentRangeEnd = petArrayEnd;
        int petCurrentRangeBegin = 0;
        int petCurrentRangeEnd = m_aetElementArray.length;

        while (petCurrentRangeBegin != petCurrentRangeEnd) {
            //const ElementType *petCurrentRangeMiddle = petCurrentRangeBegin + (petCurrentRangeEnd -
            // petCurrentRangeBegin) / 2;
            final int petCurrentRangeMiddle = petCurrentRangeBegin + (petCurrentRangeEnd - petCurrentRangeBegin) / 2;

            //if (ltElementLess(*petCurrentRangeMiddle, etValue))
            if (m_aetElementArray[petCurrentRangeMiddle] < etValue) {
                petCurrentRangeBegin = petCurrentRangeMiddle + 1;
            } else {
                petCurrentRangeEnd = petCurrentRangeMiddle;
            }
        }

        return petCurrentRangeBegin;
    }

    //private:
    //static const ElementType m_aetElementArray[];
    private final int[] m_aetElementArray;
}
