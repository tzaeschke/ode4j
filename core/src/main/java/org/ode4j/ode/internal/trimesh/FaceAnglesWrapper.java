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

import org.ode4j.ode.internal.cpp4j.java.RefDouble;
import static org.ode4j.ode.internal.trimesh.DxTriDataBase.FaceAngleDomain;

//    template<class TStorageCodec>
//        :
//public DxTriDataBase.IFaceAngleStorageControl,
//public IFaceAngleStorageView
//class FaceAnglesWrapper<TStorageCodec extends FaceAngleStorageCodec> implements IFaceAngleStorageControl,
//        IFaceAngleStorageView {
//class FaceAnglesWrapper<TStorageCodec extends FaceAngleStorageCodec> implements IFaceAngleStorageControl,
//        IFaceAngleStorageView {
class FaceAnglesWrapper implements IFaceAngleStorageControl, IFaceAngleStorageView {

    protected FaceAnglesWrapper(int triangleCount) {
        //setAllocatedTriangleCount(triangleCount);
        m_triangleFaceAngles = new float[triangleCount * 3];
    }

    //public:
    //virtual ~FaceAnglesWrapper();

    //static DxTriDataBase.IFaceAngleStorageControl *allocateInstance(unsigned triangleCount, IFaceAngleStorageView
    // *&out_storageView);

    //static boolean calculateInstanceSizeRequired(size_t &out_sizeRequired, unsigned triangleCount);

    //private:
    //void freeInstance();

    //private:
    //typedef typename TStorageCodec::storage_type storage_type;
    //typedef storage_type TriangleFaceAngles[dMTV__MAX];


    //    private static class StorageRecord {
    //        StorageRecord() {
    //            m_triangleCount = 0;
    //        }
    //
    //        int m_triangleCount;
    //        //TriangleFaceAngles[] m_triangleFaceAngles = new TriangleFaceAngles[1];
    //        float[][] m_triangleFaceAngles;// = new double[1][3];
    //    }
    //
    //    //static
    //    @Deprecated
    //    private int calculateStorageSizeForTriangleCount(int triangleCount) {
    //        //        final int baseIncludedTriangleCount = m_triangleFaceAngles.length ;
    //        //                //dSTATIC_ARRAY_SIZE(FaceAnglesWrapper<TStorageCodec>::StorageRecord,
    //        m_triangleFaceAngles);
    //        //        final int singleTriangleSize = membersize(FaceAnglesWrapper<TStorageCodec>::StorageRecord,
    //        //                m_triangleFaceAngles[0]);
    //        //        return sizeof(FaceAnglesWrapper < TStorageCodec >) + (triangleCount >
    //        baseIncludedTriangleCount ?
    //        //                (triangleCount - baseIncludedTriangleCount) * singleTriangleSize : 0U);
    //        // TODO TZ-CHECK Can we use this to pool arrays?
    //        return triangleCount;
    //    }
    //
    //    //static
    //    @Deprecated
    //    private int calculateTriangleCountForStorageSize(int storageSize) {
    //        //        dIASSERT(storageSize >= sizeof(FaceAnglesWrapper < TStorageCodec >));
    //        //
    //        //        final int baseIncludedTriangleCount = dSTATIC_ARRAY_SIZE
    //        (FaceAnglesWrapper<TStorageCodec>::StorageRecord,
    //        //                m_triangleFaceAngles);
    //        //        final int singleTriangleSize = membersize(FaceAnglesWrapper<TStorageCodec>::StorageRecord,
    //        //                m_triangleFaceAngles[0]);
    //        //        return (storageSize - sizeof(FaceAnglesWrapper < TStorageCodec >)) / singleTriangleSize +
    //        baseIncludedTriangleCount;
    //        return storageSize;
    //    }

    // private: // IFaceAngleStorageControl
    // virtual void disposeStorage();
    // virtual bool areNegativeAnglesStored() const;
    // virtual void assignFacesAngleIntoStorage(unsigned triangleIndex, dMeshTriangleVertex vertexIndex, dReal
    // dAngleValue);

    // private: // IFaceAngleStorageView
    // virtual FaceAngleDomain retrieveFacesAngleFromStorage(dReal &out_angleValue, unsigned triangleIndex,
    // dMeshTriangleVertex vertexIndex);

    //public:
    // public void setFaceAngle(unsigned triangleIndex, dMeshTriangleVertex vertexIndex, dReal dAngleValue)
    public void setFaceAngle(int triangleIndex, int vertexIndex, double dAngleValue) {
        //dIASSERT(dTMPL_IN_RANGE(triangleIndex, 0, getAllocatedTriangleCount()));
        //dIASSERT(dTMPL_IN_RANGE(vertexIndex, dMTV__MIN, dMTV__MAX));

        //m_record.m_triangleFaceAngles[triangleIndex][vertexIndex] = TStorageCodec.encodeForStorage (dAngleValue);
        m_triangleFaceAngles[triangleIndex * 3 + vertexIndex] = FaceAngleStorageCodec.encodeForStorage(dAngleValue);
    }

    // FaceAngleDomain getFaceAngle(dReal &out_angleValue, unsigned triangleIndex, dMeshTriangleVertex vertexIndex)
    // const
    public FaceAngleDomain getFaceAngle(RefDouble out_angleValue, int triangleIndex, int vertexIndex) {
        //dIASSERT(dTMPL_IN_RANGE(triangleIndex, 0, getAllocatedTriangleCount()));
        //dIASSERT(dTMPL_IN_RANGE(vertexIndex, dMTV__MIN, dMTV__MAX));

        //storage_type storedValue = m_record.m_triangleFaceAngles[triangleIndex][vertexIndex];
        double storedValue = m_triangleFaceAngles[triangleIndex * 3 + vertexIndex];
        //    FaceAngleDomain resultDomain = TStorageCodec.classifyStorageValue(storedValue);
        FaceAngleDomain resultDomain = FaceAngleStorageCodec.classifyStorageValue(storedValue);

        //        out_angleValue.set(TStorageCodec.isAngleDomainStored(resultDomain) ?
        //                TStorageCodec.decodeStorageValue(storedValue) : 0.0);
        out_angleValue.set(FaceAngleStorageCodec.isAngleDomainStored(resultDomain) ?
                FaceAngleStorageCodec.decodeStorageValue(storedValue) : 0.0);
        return resultDomain;
    }

    //private:
    //unsigned getAllocatedTriangleCount() const { return m_record.m_triangleCount; }
    //void setAllocatedTriangleCount(unsigned triangleCount) { m_record.m_triangleCount = triangleCount; }
    //    private int getAllocatedTriangleCount() {
    //        return m_record.m_triangleCount;
    //    }
    //
    //    private void setAllocatedTriangleCount(int triangleCount) {
    //        m_record.m_triangleCount = triangleCount;
    //    }

    //private final StorageRecord m_record = new StorageRecord();
    //private final float[][] m_triangleFaceAngles;
    private final float[] m_triangleFaceAngles;


    //    template<class TStorageCodec>
    //    FaceAnglesWrapper<DxTriDataBase.TStorageCodec>::~FaceAnglesWrapper()
    public void DESTRUCTOR() {
        // nothing
    }


    //template<class TStorageCodec>
    /*static */
    //DxTriDataBase.IFaceAngleStorageControl *FaceAnglesWrapper<DxTriDataBase.TStorageCodec>::allocateInstance
    // (unsigned triangleCount, DxTriDataBase.IFaceAngleStorageView *&out_storageView)
    static FaceAnglesWrapper allocateInstance(int triangleCount) {//, Ref<IFaceAngleStorageView> out_storageView) {
        return new FaceAnglesWrapper(triangleCount);
        //FaceAnglesWrapper<DxTriDataBase.TStorageCodec> *result = NULL;
        //        FaceAnglesWrapper<TStorageCodec> result = null;
        //
        //        do {
        //            RefInt sizeRequired = new RefInt();
        //            if (!calculateInstanceSizeRequired(sizeRequired, triangleCount))
        //            {
        //                break;
        //            }
        //
        //            //void *bufferPointer = dAlloc(sizeRequired);
        //            //if (bufferPointer == null) {
        //            //    break;
        //            //}
        //
        //            //result = (FaceAnglesWrapper < TStorageCodec > *)bufferPointer;
        //            //new (result) FaceAnglesWrapper < TStorageCodec > (triangleCount);
        //            result = new FaceAnglesWrapper<>(triangleCount);
        //
        //            out_storageView.set(result);
        //        } while (false);
        //
        //        return result;
    }

    //template<class TStorageCodec>
    /*static */
    // bool FaceAnglesWrapper<TStorageCodec>::calculateInstanceSizeRequired(size_t &out_sizeRequired, unsigned
    // triangleCount)
    //    private boolean calculateInstanceSizeRequired(RefInt out_sizeRequired, int triangleCount) {
    //        boolean result = false;
    //
    //        do {
    //            int triangleMaximumCount = calculateTriangleCountForStorageSize(SIZE_MAX);
    //            dIASSERT(triangleCount <= triangleMaximumCount);
    //
    //            if (triangleCount > triangleMaximumCount) // Check for overflow
    //            {
    //                break;
    //            }
    //
    //            out_sizeRequired.set(calculateStorageSizeForTriangleCount(triangleCount)); // Trailing alignment is
    //            going
    //            // to be added by memory manager automatically
    //            result = true;
    //        } while (false);
    //
    //        return result;
    //    }

    //template<class TStorageCodec>
    //void FaceAnglesWrapper<TStorageCodec>::freeInstance()
    private void freeInstance() {
        //int triangleCount = getAllocatedTriangleCount();

        //this.FaceAnglesWrapper<DxTriDataBase.TStorageCodec>::~FaceAnglesWrapper();
        DESTRUCTOR();

        //int memoryBlockSize = calculateStorageSizeForTriangleCount(triangleCount);
        //dFree(this, memoryBlockSize);
    }


    //template<class TStorageCodec>
    /*virtual */
    //void FaceAnglesWrapper<TStorageCodec>::disposeStorage()
    public void disposeStorage() {
        freeInstance();
    }

    //template<class TStorageCodec>
    /*virtual */
    //boolean FaceAnglesWrapper<TStorageCodec>::areNegativeAnglesStored() const
    public boolean areNegativeAnglesStored() {
        // return TStorageCodec.areNegativeAnglesCoded();
        return FaceAngleStorageCodec.areNegativeAnglesCoded();
    }

    //template<class TStorageCodec>
    /*virtual */
    //void FaceAnglesWrapper<TStorageCodec>::assignFacesAngleIntoStorage(unsigned triangleIndex, dMeshTriangleVertex
    // vertexIndex, dReal dAngleValue)
    public void assignFacesAngleIntoStorage(int triangleIndex, int vertexIndex, double dAngleValue) {
        setFaceAngle(triangleIndex, vertexIndex, dAngleValue);
    }

    //template<class TStorageCodec>
    /*virtual */
    //int FaceAnglesWrapper<TStorageCodec>::retrieveFacesAngleFromStorage(dReal &out_angleValue, unsigned
    // triangleIndex, dMeshTriangleVertex vertexIndex)
    public FaceAngleDomain retrieveFacesAngleFromStorage(RefDouble out_angleValue, int triangleIndex, int vertexIndex) {
        return getFaceAngle(out_angleValue, triangleIndex, vertexIndex);
    }

}
