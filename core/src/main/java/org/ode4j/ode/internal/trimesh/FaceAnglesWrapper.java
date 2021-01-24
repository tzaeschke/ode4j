package org.ode4j.ode.internal.trimesh;

import org.ode4j.ode.internal.cpp4j.java.RefDouble;
import org.ode4j.ode.internal.cpp4j.java.RefInt;

import static org.ode4j.ode.DTriMesh.dMTV__MAX;
import static org.ode4j.ode.DTriMesh.dMTV__MIN;
import static org.ode4j.ode.internal.Common.SIZE_MAX;
import static org.ode4j.ode.internal.Common.dIASSERT;

//    template<class TStorageCodec>
//        :
//public DxTriDataBase.IFaceAngleStorageControl,
//public IFaceAngleStorageView
class FaceAnglesWrapper<TStorageCodec> implements IFaceAngleStorageControl, IFaceAngleStorageView {


    protected FaceAnglesWrapper(int triangleCount) {
        setAllocatedTriangleCount(triangleCount);
    }

    //public:
    //virtual ~FaceAnglesWrapper();

    //static DxTriDataBase.IFaceAngleStorageControl *allocateInstance(unsigned triangleCount, IFaceAngleStorageView
    // *&out_storageView);

    //static boolean calculateInstanceSizeRequired(size_t &out_sizeRequired, unsigned triangleCount);

    //private:
    //void freeInstance();

    //private:
    typedef typename TStorageCodec::storage_type storage_type;
    typedef storage_type TriangleFaceAngles[dMTV__MAX];

    static class StorageRecord {
        StorageRecord() {
            m_triangleCount = 0;
        }

        int m_triangleCount;
        TriangleFaceAngles[] m_triangleFaceAngles = new TriangleFaceAngles[1];
    }

    ;

    static int calculateStorageSizeForTriangleCount(int triangleCount) {
        final int baseIncludedTriangleCount = dSTATIC_ARRAY_SIZE(FaceAnglesWrapper<TStorageCodec>::StorageRecord,
                m_triangleFaceAngles);
        final int singleTriangleSize = membersize(FaceAnglesWrapper<TStorageCodec>::StorageRecord,
                m_triangleFaceAngles[0]);
        return sizeof(FaceAnglesWrapper < TStorageCodec >) + (triangleCount > baseIncludedTriangleCount ?
                (triangleCount - baseIncludedTriangleCount) * singleTriangleSize : 0
        U);
    }

    static int calculateTriangleCountForStorageSize(int storageSize) {
        dIASSERT(storageSize >= sizeof(FaceAnglesWrapper < TStorageCodec >));

        final int baseIncludedTriangleCount = dSTATIC_ARRAY_SIZE(FaceAnglesWrapper<TStorageCodec>::StorageRecord,
                m_triangleFaceAngles);
        final int singleTriangleSize = membersize(FaceAnglesWrapper<TStorageCodec>::StorageRecord,
                m_triangleFaceAngles[0]);
        return (storageSize - sizeof(FaceAnglesWrapper < TStorageCodec >)) / singleTriangleSize + baseIncludedTriangleCount;
    }

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
        dIASSERT(dTMPL_IN_RANGE(triangleIndex, 0, getAllocatedTriangleCount()));
        dIASSERT(dTMPL_IN_RANGE(vertexIndex, dMTV__MIN, dMTV__MAX));

        m_record.m_triangleFaceAngles[triangleIndex][vertexIndex] = TStorageCodec::encodeForStorage (dAngleValue);
    }

    // FaceAngleDomain getFaceAngle(dReal &out_angleValue, unsigned triangleIndex, dMeshTriangleVertex vertexIndex)
    // const
    public int getFaceAngle(RefDouble out_angleValue, int triangleIndex, int vertexIndex) {
        dIASSERT(dTMPL_IN_RANGE(triangleIndex, 0, getAllocatedTriangleCount()));
        dIASSERT(dTMPL_IN_RANGE(vertexIndex, dMTV__MIN, dMTV__MAX));

        storage_type storedValue = m_record.m_triangleFaceAngles[triangleIndex][vertexIndex];
        //FaceAngleDomain
        int resultDomain = TStorageCodec::classifyStorageValue (storedValue);

        out_angleValue = TStorageCodec::isAngleDomainStored (resultDomain) ? TStorageCodec::decodeStorageValue
        (storedValue) :REAL(0.0);
        return resultDomain;
    }

    //private:
    //unsigned getAllocatedTriangleCount() const { return m_record.m_triangleCount; }
    //void setAllocatedTriangleCount(unsigned triangleCount) { m_record.m_triangleCount = triangleCount; }
    private int getAllocatedTriangleCount() {
        return m_record.m_triangleCount;
    }

    private void setAllocatedTriangleCount(int triangleCount) {
        m_record.m_triangleCount = triangleCount;
    }

    private StorageRecord m_record;


    //    template<class TStorageCodec>
    //    FaceAnglesWrapper<DxTriDataBase.TStorageCodec>::~FaceAnglesWrapper()
    public void DESTRUCTOR() {
        // nothing
    }


    //template<class TStorageCodec>
    /*static */
    //DxTriDataBase.IFaceAngleStorageControl *FaceAnglesWrapper<DxTriDataBase.TStorageCodec>::allocateInstance
    // (unsigned triangleCount, DxTriDataBase.IFaceAngleStorageView *&out_storageView)
    IFaceAngleStorageControl allocateInstance(int triangleCount, IFaceAngleStorageView[] out_storageView) {
        FaceAnglesWrapper<DxTriDataBase.TStorageCodec> *result = NULL;

        do {
            int sizeRequired;
            if (!FaceAnglesWrapper<DxTriDataBase.TStorageCodec>::calculateInstanceSizeRequired
            (sizeRequired, triangleCount))
            {
                break;
            }

            void *bufferPointer = dAlloc(sizeRequired);
            if (bufferPointer == NULL) {
                break;
            }

            result = (FaceAnglesWrapper < TStorageCodec > *)bufferPointer;
            new (result) FaceAnglesWrapper < TStorageCodec > (triangleCount);

            out_storageView = result;
        } while (false);

        return result;
    }

    //template<class TStorageCodec>
    /*static */
    // bool FaceAnglesWrapper<TStorageCodec>::calculateInstanceSizeRequired(size_t &out_sizeRequired, unsigned
    // triangleCount)
    boolean calculateInstanceSizeRequired(RefInt out_sizeRequired, int triangleCount) {
        boolean result = false;

        do {
            int triangleMaximumCount = calculateTriangleCountForStorageSize(SIZE_MAX);
            dIASSERT(triangleCount <= triangleMaximumCount);

            if (triangleCount > triangleMaximumCount) // Check for overflow
            {
                break;
            }

            out_sizeRequired.set(calculateStorageSizeForTriangleCount(triangleCount)); // Trailing alignment is going
            // to be added by memory manager automatically
            result = true;
        } while (false);

        return result;
    }

    //template<class TStorageCodec>
    //void FaceAnglesWrapper<TStorageCodec>::freeInstance()
    void freeInstance() {
        int triangleCount = getAllocatedTriangleCount();

        //this.FaceAnglesWrapper<DxTriDataBase.TStorageCodec>::~FaceAnglesWrapper();
        DESTRUCTOR();

        int memoryBlockSize = calculateStorageSizeForTriangleCount(triangleCount);
        dFree(this, memoryBlockSize);
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
        return DxTriDataBase.TStorageCodec::areNegativeAnglesCoded ();
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
    public int retrieveFacesAngleFromStorage(RefDouble out_angleValue, int triangleIndex, int vertexIndex) {
        return getFaceAngle(out_angleValue, triangleIndex, vertexIndex);
    }

}
