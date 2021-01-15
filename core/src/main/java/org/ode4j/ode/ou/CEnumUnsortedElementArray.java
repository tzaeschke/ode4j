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

// CEnumUnsortedElementArray definition

/*
 *	Implementation Note:
 *	The array is intended to store static constant data.
 *	Therefore CElementEqualType should not ever need a nontrivial constructor
 *	and it is acceptable to have it as template parameter.
 */
//   template<typename EnumType, const EnumType EnumMax, typename ElementType, const int Instance=0, class CElementEqualType=CTypeStandardEqual<ElementType> >
public class CEnumUnsortedElementArray {

        public
         CEnumUnsortedElementArray(int[] m_aetElementArray, int EnumMax)
        {
            this.EnumMax = EnumMax;
            this.m_aetElementArray = m_aetElementArray;
//#if !defined(NDEBUG)
//
//#if _OU_COMPILER != _OU_COMPILER_GCC || _OU_COMPILER_VERSION == _OU_COMPILER_VERSION_GCCLT4
//
//            OU_ASSERT(OU_ARRAY_SIZE(m_aetElementArray) == EnumMax);
//
//
//#endif // #if _OU_COMPILER != _OU_COMPILER_GCC || _OU_COMPILER_VERSION == _OU_COMPILER_VERSION_GCCLT4
//
//
//#endif // #if !defined(NDEBUG)
        }

    //        public:
    //        static _OU_ALWAYSINLINE const EnumType _OU_CONVENTION_API
    //        /*const EnumType */Decode(const ElementType &etValue)
    //        {
    //		const ElementType *itElementFound = FindValueSequentially(m_aetElementArray, m_aetElementArray + EnumMax, etValue);
    //
    //            EnumType etResult = (EnumType)(itElementFound - m_aetElementArray);
    //            return etResult;
    //        }
    //public int Decode(final ElementType etValue)
    public int Decode(final int etValue) {
        //const ElementType *itElementFound = FindValueSequentially(m_aetElementArray, m_aetElementArray + EnumMax,
        // etValue);
        int itElementFoundOfs = FindValueSequentially(m_aetElementArray, EnumMax, etValue);

        //EnumType etResult = (EnumType) (itElementFound - m_aetElementArray);
        //return etResult;
        return itElementFoundOfs;
    }


    //    static _OU_ALWAYSINLINE const ElementType &_OU_CONVENTION_API
    //        /*const ElementType &*/Encode(const EnumType &etValue)
    //    {
    //        OU_ASSERT(sizeof(EnumType) <= sizeof(int));
    //        OU_ASSERT(OU_IN_INT_RANGE(etValue, 0, EnumMax));
    //
    //        return m_aetElementArray[etValue];
    //    }

    //public ElementType Encode(final EnumType etValue)
    public int Encode(final int etValue) {
        //OU_ASSERT(sizeof(EnumType) <= sizeof(int));
        //OU_ASSERT(OU_IN_INT_RANGE(etValue, 0, EnumMax));
        return m_aetElementArray[etValue];
    }

    //        static _OU_ALWAYSINLINE bool _OU_CONVENTION_API
    //        /*bool */IsValidDecode(const EnumType &etValue)
    //    {
    //        return etValue != EnumMax;
    //    }
    //public static boolean IsValidDecode(EnumType etValue) {
    public boolean IsValidDecode(int etValue) {
        return etValue != EnumMax;
    }

    //        static _OU_ALWAYSINLINE const ElementType *_OU_CONVENTION_API
    //        /*const ElementType **/GetStoragePointer()
    //    {
    //        return m_aetElementArray;
    //    }
    //public ElementType[] GetStoragePointer() {
    public int[] GetStoragePointer() {
        return m_aetElementArray;
    }

    //        private:
    //        static const ElementType *_OU_CONVENTION_API FindValueSequentially(const ElementType *petArrayBegin, const ElementType *petArrayEnd, const ElementType &etValue)
    //    {
    //		const CElementEqualType etElementEqual = CElementEqualType();
    //
    //		const ElementType *petCurrentElement = petArrayBegin;
    //
    //        for (; petCurrentElement != petArrayEnd; ++petCurrentElement)
    //        {
    //            if (etElementEqual(*petCurrentElement, etValue))
    //            {
    //                break;
    //            }
    //        }
    //
    //        return petCurrentElement;
    //    }

    //private ElementType FindValueSequentially(ElementType petArrayBegin, ElementType petArrayEnd, ElementType etValue)
    private int FindValueSequentially(int[] petArray, int petArraySize, int etValue) {
        //final CElementEqualType etElementEqual = CElementEqualType();

        int petCurrentElementOfs = 0;//final ElementType petCurrentElement = petArrayBegin;

        for (; petCurrentElementOfs != petArraySize; ++petCurrentElementOfs) {
            // if (etElementEqual(petArray[petCurrentElementOfs], etValue))
            if (petArray[petCurrentElementOfs] == etValue) {
                break;
            }
        }

        return petCurrentElementOfs;
    }

    private final int EnumMax;
    //private final ElementType[] m_aetElementArray;
    private final int[] m_aetElementArray;
}
