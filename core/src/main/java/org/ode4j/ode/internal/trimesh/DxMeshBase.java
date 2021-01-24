package org.ode4j.ode.internal.trimesh;

import org.ode4j.ode.DTriMesh;
import org.ode4j.ode.internal.DxGeom;
import org.ode4j.ode.internal.DxSpace;

import java.util.Arrays;

import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.Common.dIN_RANGE;

//    typedef dxGeom dxMeshBase_Parent;
//    struct dxMeshBase:
//    public dxMeshBase_Parent
public abstract class DxMeshBase extends DxGeom {
    //        public:
    //        dxMeshBase(dxSpace *Space, dxTriDataBase *Data,
    //                dTriCallback *Callback, dTriArrayCallback *ArrayCallback, dTriRayCallback *RayCallback,
    //                bool doTCs=false):
    public DxMeshBase(DxSpace Space, DxTriDataBase Data, DTriMesh.DTriCallback Callback,
                      DTriMesh.DTriArrayCallback ArrayCallback, DTriMesh.DTriRayCallback RayCallback) {
        this(Space, Data, Callback, ArrayCallback, RayCallback, false);
    }
    public DxMeshBase(DxSpace Space, DxTriDataBase Data, DTriMesh.DTriCallback Callback,
                      DTriMesh.DTriArrayCallback ArrayCallback, DTriMesh.DTriRayCallback RayCallback, boolean doTCs) {
        super(Space, true);
        m_Callback = Callback;
        m_ArrayCallback = ArrayCallback;
        m_RayCallback = RayCallback;
        m_TriMergeCallback = null;
        m_Data = Data;
        //std::fill(m_DoTCs, m_DoTCs + dARRAY_SIZE(m_DoTCs), doTCs);
        Arrays.fill(m_DoTCs, doTCs);
        type = dTriMeshClass;
    }

    boolean invokeCallback(DxGeom Object, int TriIndex) {
        return m_Callback == null || m_Callback.call(this, Object, TriIndex) != 0;
    }

    //    public enum TRIMESHTC {
    //        TTC__MIN,
    //
    //        TTC_SPHERE =TTC__MIN,
    //        TTC_BOX,
    //        TTC_CAPSULE,
    //
    //        TTC__MAX,
    //    }
    // enum TRIMESHTC
    public static final int TTC__MIN = 0;
    public static final int TTC_SPHERE =TTC__MIN;
    public static final int TTC_BOX = TTC_SPHERE + 1;
    public static final int TTC_CAPSULE = TTC_BOX + 1;
    public static final int TTC__MAX = TTC_CAPSULE + 1;


    //public:
    public void assignCallback(DTriMesh.DTriCallback value) {
        m_Callback = value;
    }

    public DTriMesh.DTriCallback retrieveCallback() {
        return m_Callback;
    }

    public void assignArrayCallback(DTriMesh.DTriArrayCallback value) {
        m_ArrayCallback = value;
    }

    public DTriMesh.DTriArrayCallback retrieveArrayCallback() {
        return m_ArrayCallback;
    }

    public void assignRayCallback(DTriMesh.DTriRayCallback value) {
        m_RayCallback = value;
    }

    public DTriMesh.DTriRayCallback retrieveRayCallback() {
        return m_RayCallback;
    }

    public void assignTriMergeCallback(DTriMesh.DTriTriMergeCallback value) {
        m_TriMergeCallback = value;
    }

    public DTriMesh.DTriTriMergeCallback retrieveTriMergeCallback() {
        return m_TriMergeCallback;
    }

    public void assignMeshData(DxTriDataBase instance) {
        setMeshData(instance);
        // I changed my data -- I know nothing about my own AABB anymore.
        markAABBBad();
    }

    //dxTriDataBase *retrieveMeshData() const { return getMeshData(); }
    public DxTriDataBase retrieveMeshData() {
        return getMeshData();
    }

    //        IFaceAngleStorageControl *retrieveFaceAngleStorage() { return m_Data.retrieveFaceAngles(); }
    //        IFaceAngleStorageView *retrieveFaceAngleView() { return m_Data.retrieveFaceAngleView(); }
    public IFaceAngleStorageControl[] retrieveFaceAngleStorage() {
        return m_Data.retrieveFaceAngles();
    }

    public IFaceAngleStorageView[] retrieveFaceAngleView() {
        return m_Data.retrieveFaceAngleView();
    }


    public void assignDoTC(int tc, boolean value) {
        setDoTC(tc, value);
    }

    public boolean retrieveDoTC(int tc) {
        return getDoTC(tc);
    }

    //public:
    //  void setDoTC(TRIMESHTC tc, bool value) { dIASSERT(dIN_RANGE(tc, TTC__MIN, TTC__MAX)); m_DoTCs[tc] = value; }
    //  bool getDoTC(TRIMESHTC tc) const { dIASSERT(dIN_RANGE(tc, TTC__MIN, TTC__MAX)); return m_DoTCs[tc]; }
    public void setDoTC(int tc, boolean value) {
        dIASSERT(dIN_RANGE(tc, TTC__MIN, TTC__MAX));
        m_DoTCs[tc] = value;
    }

    public boolean getDoTC(int tc) {
        dIASSERT(dIN_RANGE(tc, TTC__MIN, TTC__MAX));
        return m_DoTCs[tc];
    }


    //private:
    private void setMeshData(DxTriDataBase Data) {
        m_Data = Data;
    }

    //protected:
    protected DxTriDataBase getMeshData() {
        return m_Data;
    }

    //public:
    // Callbacks
    public DTriMesh.DTriCallback m_Callback;
    public DTriMesh.DTriArrayCallback m_ArrayCallback;
    public DTriMesh.DTriRayCallback m_RayCallback;
    public DTriMesh.DTriTriMergeCallback m_TriMergeCallback;

    // Data types
    private DxTriDataBase m_Data;

    public boolean[] m_DoTCs = new boolean[TTC__MAX];
}
