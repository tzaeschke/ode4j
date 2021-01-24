package org.ode4j.ode.internal.trimesh;

import static org.ode4j.ode.internal.Common.*;
import static org.ode4j.ode.internal.trimesh.DxTriDataBase.*;

//    template<typename TStorageType, EdgeStorageSignInclusion t_SignInclusion>
//    class FaceAngleStorageCodec;

//template<typename TStorageType>
//<TStorageType extends Number>;
class FaceAngleStorageCodec<TStorageType extends Number, SSI_SIGNED_STORED> {


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
    private static final short STORAGE_TYPE_MIN = 0;
    private static final short STORAGE_TYPE_MAX = Short.MAX_VALUE;


    static boolean areNegativeAnglesCoded()
        {
            return false;
        }

        //static storage_type encodeForStorage(double angleValue)
        static short encodeForStorage(double angleValue)
        {
            short result = STORAGE_TYPE_MIN;

            if (angleValue >= 0.0)
            {
                int angleAsInt = (int)dFloor(angleValue * (double)(((STORAGE_TYPE_MAX - STORAGE_TYPE_MIN - 1) / M_PI)));
                result = (STORAGE_TYPE_MIN + 1) + dMACRO_MIN(angleAsInt, STORAGE_TYPE_MAX - STORAGE_TYPE_MIN - 1);
            }

            return  result;
        }

        // static FaceAngleDomain classifyStorageValue(storage_type storedValue)
        static int classifyStorageValue(short storedValue)
        {
            dSASSERT(EAD__MAX == 3);

            return storedValue < STORAGE_TYPE_MIN + 1 ? FAD_CONCAVE : (storedValue == STORAGE_TYPE_MIN + 1 ? FAD_FLAT : FAD_CONVEX);
        }

        // static bool isAngleDomainStored(FaceAngleDomain domainValue)
        static boolean isAngleDomainStored(int domainValue)
        {
            return dTMPL_IN_RANGE(domainValue, FAD__BYTEPOS_STORED_MIN, FAD__BYTEPOS_STORED_MAX);
        }

        //static dReal decodeStorageValue(storage_type storedValue)
        static int decodeStorageValue(short storedValue)
        {
            dIASSERT(storedValue >= (STORAGE_TYPE_MIN + 1));

            return (storedValue - (STORAGE_TYPE_MIN + 1)) * (double)(M_PI / (STORAGE_TYPE_MAX - STORAGE_TYPE_MIN - 1));
        }
//    };



    //public:
    //typedef typename _make_signed<TStorageType>::type storage_type;
    //enum
    //{
    //STORAGE_TYPE_MAX = (typename _make_unsigned<TStorageType>::type)(~(typename _make_unsigned<TStorageType>::type)
    // 0) >> 1,
    final int STORAGE_TYPE_MAX = TStorageType.MAX_VALUE;
    //};

    static boolean areNegativeAnglesCoded() {
        return true;
    }

    static storage_type encodeForStorage(dReal angleValue) {
        unsigned angleAsInt = (unsigned) dFloor(dFabs(angleValue) * (dReal) (STORAGE_TYPE_MAX / M_PI));
        unsigned limitedAngleAsInt = dMACRO_MIN(angleAsInt, STORAGE_TYPE_MAX);
        storage_type result = angleValue < REAL(0.0) ? -(storage_type) limitedAngleAsInt :
                (storage_type) limitedAngleAsInt;
        return result;
    }

    static FaceAngleDomain classifyStorageValue(storage_type storedValue) {
        dSASSERT(EAD__MAX == 3);

        return storedValue < 0 ? FAD_CONCAVE : (storedValue == 0 ? FAD_FLAT :
                DxTriDataBase.FAD_CONVEX);
    }

    static boolean isAngleDomainStored(FaceAngleDomain domainValue) {
        return !dTMPL_IN_RANGE(domainValue, DxTriDataBase.FAD__SIGNSTORED_IMPLICITVALUE_MIN,
                DxTriDataBase.FAD__SIGNSTORED_IMPLICITVALUE_MAX);
    }

    static double decodeStorageValue(storage_type storedValue) {
        return storedValue * (double) (M_PI / STORAGE_TYPE_MAX);
    }
}
