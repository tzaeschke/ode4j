/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zaeschke     *
 * All rights reserved.  Email: ode4j@gmx.de   Web: www.ode4j.org        *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.         *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT, ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT for more   *
 * details.                                                              *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode.internal.trimesh;

import static org.ode4j.ode.internal.Common.*;
import static org.ode4j.ode.internal.trimesh.DxTriDataBase.*;

//    template<typename TStorageType, EdgeStorageSignInclusion t_SignInclusion>
//    class FaceAngleStorageCodec;

//template<typename TStorageType>
//<TStorageType extends Number>;

/**
 * SSI_SIGNED_STORED.
 * UNSIGNED storage is not supported
 * <p>
 * //@param <TStorageType>
 * //@param <SSI_SIGNED_STORED>
 */
//class FaceAngleStorageCodec {//<TStorageType extends Number, SSI_SIGNED_STORED> {
//
//    //    template<typename TStorageType>
//    //    class FaceAngleStorageCodec<TStorageType, SSI_SIGNED_STORED>
//    //    {
//    //public:
//    //typedef typename _make_signed<TStorageType>::type storage_type;
//    //enum
//    //{
//    //STORAGE_TYPE_MAX = (typename _make_unsigned<TStorageType>::type)(~(typename _make_unsigned<TStorageType>::type)
//    // 0) >> 1,
//    private static final int STORAGE_TYPE_MAX = Integer.MAX_VALUE;
//    //};
//
//    static boolean areNegativeAnglesCoded() {
//        return true;
//    }
//
//    //static storage_type encodeForStorage(dReal angleValue)
//    static int encodeForStorage(double angleValue) {
//        int angleAsInt = (int) dFloor(dFabs(angleValue) * (double) (STORAGE_TYPE_MAX / M_PI));
//        int limitedAngleAsInt = dMACRO_MIN(angleAsInt, STORAGE_TYPE_MAX);
//        int result = angleValue < (0.0) ? -(int) limitedAngleAsInt : (int) limitedAngleAsInt;
//        return result;
//    }
//
//    //static FaceAngleDomain classifyStorageValue(storage_type storedValue)
//    static FaceAngleDomain classifyStorageValue(double storedValue) {
//        dSASSERT(EAD__MAX == 3);
//
//        return storedValue < 0 ? FaceAngleDomain.FAD_CONCAVE : (storedValue == 0 ?
//                FaceAngleDomain.FAD_FLAT : FaceAngleDomain.FAD_CONVEX);
//    }
//
//    //static bool isAngleDomainStored(FaceAngleDomain domainValue)
//    static boolean isAngleDomainStored(FaceAngleDomain domainValue) {
//        //return !dTMPL_IN_RANGE(domainValue, FAD__SIGNSTORED_IMPLICITVALUE_MIN, FAD__SIGNSTORED_IMPLICITVALUE_MAX);
//        return domainValue != FaceAngleDomain.FAD_FLAT;
//    }
//
//    //static dReal decodeStorageValue(storage_type storedValue)
//    static double decodeStorageValue(double storedValue) {
//        return storedValue * (double) (M_PI / STORAGE_TYPE_MAX);
//    }
//}
class FaceAngleStorageCodec {

    static boolean areNegativeAnglesCoded() {
        return true;
    }

    static float encodeForStorage(double angleValue) {
        return (float) (angleValue >= 0.0 ? Math.min(angleValue, M_PI) : Math.max(angleValue, -M_PI));
    }

    static FaceAngleDomain classifyStorageValue(double storedValue) {
        return storedValue < 0 ? FaceAngleDomain.FAD_CONCAVE : (storedValue == 0 ?
                FaceAngleDomain.FAD_FLAT : FaceAngleDomain.FAD_CONVEX);
    }

    static boolean isAngleDomainStored(FaceAngleDomain domainValue) {
        return domainValue != FaceAngleDomain.FAD_FLAT;
    }

    static double decodeStorageValue(double storedValue) {
        return storedValue;
    }
}



//    template<typename TStorageType>
//    class FaceAngleStorageCodec<TStorageType, SSI_POSITIVE_STORED>
//    {
//public:
//typedef typename _make_unsigned<TStorageType>::type storage_type;

//        enum
//        {
//            STORAGE_TYPE_MIN = 0,
//                    STORAGE_TYPE_MAX = (storage_type)(~(storage_type)0),
//        };
//    private static final short STORAGE_TYPE_MIN = 0;
//    private static final short STORAGE_TYPE_MAX = Short.MAX_VALUE;
//
//
//    static boolean areNegativeAnglesCoded()
//        {
//            return false;
//        }
//
//        //static storage_type encodeForStorage(double angleValue)
//        static short encodeForStorage(double angleValue)
//        {
//            short result = STORAGE_TYPE_MIN;
//
//            if (angleValue >= 0.0)
//            {
//                int angleAsInt = (int)dFloor(angleValue * (double)(((STORAGE_TYPE_MAX - STORAGE_TYPE_MIN - 1) /
//                M_PI)));
//                result = (STORAGE_TYPE_MIN + 1) + dMACRO_MIN(angleAsInt, STORAGE_TYPE_MAX - STORAGE_TYPE_MIN - 1);
//            }
//
//            return  result;
//        }
//
//        // static FaceAngleDomain classifyStorageValue(storage_type storedValue)
//        static int classifyStorageValue(short storedValue)
//        {
//            dSASSERT(EAD__MAX == 3);
//
//            return storedValue < STORAGE_TYPE_MIN + 1 ? FAD_CONCAVE : (storedValue == STORAGE_TYPE_MIN + 1 ?
//            FAD_FLAT : FAD_CONVEX);
//        }
//
//        // static bool isAngleDomainStored(FaceAngleDomain domainValue)
//        static boolean isAngleDomainStored(int domainValue)
//        {
//            return dTMPL_IN_RANGE(domainValue, FAD__BYTEPOS_STORED_MIN, FAD__BYTEPOS_STORED_MAX);
//        }
//
//        //static dReal decodeStorageValue(storage_type storedValue)
//        static int decodeStorageValue(short storedValue)
//        {
//            dIASSERT(storedValue >= (STORAGE_TYPE_MIN + 1));
//
//            return (storedValue - (STORAGE_TYPE_MIN + 1)) * (double)(M_PI / (STORAGE_TYPE_MAX - STORAGE_TYPE_MIN -
//            1));
//        }
////    };
//
//
//
//    //public:
//    //typedef typename _make_signed<TStorageType>::type storage_type;
//    //enum
//    //{
//    //STORAGE_TYPE_MAX = (typename _make_unsigned<TStorageType>::type)(~(typename _make_unsigned<TStorageType>::type)
//    // 0) >> 1,
//    final int STORAGE_TYPE_MAX = TStorageType.MAX_VALUE;
//    //};
//
//    static boolean areNegativeAnglesCoded() {
//        return true;
//    }
//
//    static int encodeForStorage(double angleValue) {
//        int angleAsInt = (int) dFloor(dFabs(angleValue) * (double) (STORAGE_TYPE_MAX / M_PI));
//        int limitedAngleAsInt = dMACRO_MIN(angleAsInt, STORAGE_TYPE_MAX);
//        int result = angleValue < (0.0) ? -(int) limitedAngleAsInt :
//                (int) limitedAngleAsInt;
//        return result;
//    }
//
//    //static FaceAngleDomain classifyStorageValue(storage_type storedValue) {
//    static int classifyStorageValue(int storedValue) {
//        dSASSERT(EAD__MAX == 3);
//
//        return storedValue < 0 ? FAD_CONCAVE : (storedValue == 0 ? FAD_FLAT :
//                DxTriDataBase.FAD_CONVEX);
//    }
//
//    //static boolean isAngleDomainStored(FaceAngleDomain domainValue) {
//    static boolean isAngleDomainStored(int domainValue) {
//        return !dTMPL_IN_RANGE(domainValue, DxTriDataBase.FAD__SIGNSTORED_IMPLICITVALUE_MIN,
//                DxTriDataBase.FAD__SIGNSTORED_IMPLICITVALUE_MAX);
//    }
//
//    //static double decodeStorageValue(storage_type storedValue) {
//    static double decodeStorageValue(int storedValue) {
//        return storedValue * (M_PI / STORAGE_TYPE_MAX);
//    }
//}
