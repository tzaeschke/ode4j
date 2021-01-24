package org.ode4j.ode.internal;

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DTriMesh;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;

import java.util.Arrays;

import static org.ode4j.ode.DTriMesh.*;
import static org.ode4j.ode.DTriMesh.dMESHDATAUSE.*;
import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.Common.*;
import static org.ode4j.ode.internal.CommonEnums.*;

public class DxTriDataBase {
//
//
//    // **************************************************
//    //  collision_trimesh_trimesh.cpp
//    // **************************************************
//
//    //#if !dTLS_ENABLED
//    // Have collider cache instance unconditionally of OPCODE or GIMPACT selection
//    public static final TrimeshCollidersCache g_ccTrimeshCollidersCache;
//    //#endif
//
//
//    // **************************************************
//    //  collision_trimesh_internal.h
//    // **************************************************
//
//    // TriMesh code by Erwin de Vries.
//    // Modified for FreeSOLID Compatibility by Rodrigo Hernandez
//    // TriMesh caches separation by Oleh Derevenko
//    // TriMesh storage classes refactoring and face angle computation code by Oleh Derevenko (C) 2016-2017
//
//
//    //  #ifndef _ODE_COLLISION_TRIMESH_INTERNAL_H_
//    //  #define _ODE_COLLISION_TRIMESH_INTERNAL_H_
//
//
//    //****************************************************************************
//    // dxTriMesh class
//
//    // struct TrimeshCollidersCache;
//    // struct dxTriMeshData;
//
//
//    //    static inline
//    //    TrimeshCollidersCache *GetTrimeshCollidersCache(unsigned uiTLSKind)
//    static TrimeshCollidersCache GetTrimeshCollidersCache(int uiTLSKind) {
//        //    #if dTLS_ENABLED
//        //            EODETLSKIND tkTLSKind = (EODETLSKIND)uiTLSKind;
//        //            return COdeTls::GetTrimeshCollidersCache(tkTLSKind);
//        //    #else // dTLS_ENABLED
//        //(void)uiTLSKind; // unused
//        //extern TrimeshCollidersCache g_ccTrimeshCollidersCache;
//        return g_ccTrimeshCollidersCache;
//        // #endif // dTLS_ENABLED
//    }
//
//
//    //enum FaceAngleStorageMethod {
//    public static final int ASM__MIN = 0;
//    public static final int ASM_BYTE_SIGNED = ASM__MIN;
//    public static final int ASM_BYTE_POSITIVE = ASM_BYTE_SIGNED + 1;
//    public static final int ASM_WORD_SIGNED = ASM_BYTE_POSITIVE + 1;
//    public static final int ASM__MAX = ASM_WORD_SIGNED + 1;
//    public static final int ASM__INVALID = ASM__MAX;
//
//    // enum FaceAngleDomain {
//    public static final int FAD__MIN = 0;
//    public static final int FAD_CONCAVE = FAD__MIN;
//    public static final int FAD__SIGNSTORED_IMPLICITVALUE_MIN = FAD_CONCAVE + 1;
//    public static final int FAD_FLAT = FAD__SIGNSTORED_IMPLICITVALUE_MIN;
//    public static final int FAD__SIGNSTORED_IMPLICITVALUE_MAX = FAD_FLAT + 1;
//    public static final int FAD__BYTEPOS_STORED_MIN = FAD__SIGNSTORED_IMPLICITVALUE_MAX;
//    public static final int FAD_CONVEX = FAD__BYTEPOS_STORED_MIN;
//    public static final int FAD__BYTEPOS_STORED_MAX = FAD_CONVEX + 1;
//    public static final int EAD__MAX = FAD__BYTEPOS_STORED_MAX;
//
//    //    class IFaceAngleStorageControl
//    //    {
//    //        public:
//    //        virtual void disposeStorage() = 0;
//    //
//    //        virtual bool areNegativeAnglesStored() const = 0;
//    //
//    //        // This is to store angles between neighbor triangle normals as positive value for convex and negative for concave edges
//    //        virtual void assignFacesAngleIntoStorage(unsigned triangleIndex, dMeshTriangleVertex vertexIndex, dReal dAngleValue) = 0;
//    //    };
//    //
//    //    class IFaceAngleStorageView
//    //    {
//    //        public:
//    //        virtual FaceAngleDomain retrieveFacesAngleFromStorage(dReal &out_AngleValue, unsigned triangleIndex, dMeshTriangleVertex vertexIndex) = 0;
//    //    };
//
//
//    //typedef dBase dxTriDataBase_Parent;
//    //class dxTriDataBase extends dxTriDataBase_Parent
//    class DxTriDataBase extends DBase {
//        //public:
//        DxTriDataBase() {
//            super();
//            m_vertices = null;
//            m_vertexStride = 0;
//            m_vertexCount = 0;
//            m_indices = null;
//            m_triangleCount = 0;
//            m_triStride = 0;
//            m_single = false;
//            m_normals = null;
//            m_faceAngles = null;
//            m_faceAngleView = null;
//
//            if (!dTRIMESH_ENABLED) {
//                dUASSERT(false, "dTRIMESH_ENABLED is not defined. Trimesh geoms will not work");
//            }
//        }
//
//        //~dxTriDataBase();
//        protected void DESTRUCTOR() {
//            super.DESTRUCTOR();
//        }
//
//        //        void buildData(const void *Vertices, int VertexStide, unsigned VertexCount,
//        //        const void *Indices, unsigned IndexCount, int TriStride,
//        //        const void *Normals,
//        //            bool Single);
//
//        public int retrieveVertexCount() {
//            return m_vertexCount;
//        }
//
//        public int retrieveVertexStride() {
//            return m_vertexStride;
//        }
//
//        public int retrieveTriangleCount() {
//            return m_triangleCount;
//        }
//
//        public int retrieveTriangleStride() {
//            return m_triStride;
//        }
//
//        //protected:
//        //    const void *retrieveVertexInstances() const { return m_vertices; }
//        //    const void *retrieveTriangleVertexIndices() const { return m_indices; }
//        @Deprecated
//        private class Void {
//        }
//
//        ;
//
//        protected Void retrieveVertexInstances() {
//            return m_vertices;
//        }
//
//        protected Void retrieveTriangleVertexIndices() {
//            return m_indices;
//        }
//
//        protected boolean isSingle() {
//            return m_single;
//        }
//
//        //        public:
//        //        template<typename tcoordfloat, typename
//        //        tindexint>
//        //        static void retrieveTriangleVertexPoints(dVector3 out_Points[dMTV__MAX], unsigned triangleIndex,
//        //        const tcoordfloat *vertexInstances, int vertexStride, const tindexint *triangleVertexIndices, int triangleStride);
//
//        //const void assignNormals(const void *normals) { m_normals = normals; }
//        //const void *retrieveNormals() const { return m_normals; }
//        public void assignNormals(Void[] normals) {
//            m_normals = normals;
//        }
//
//        public Void[] retrieveNormals() {
//            return m_normals;
//        }
//
//        //        IFaceAngleStorageControl *retrieveFaceAngles() const { return m_faceAngles; }
//        //        IFaceAngleStorageView *retrieveFaceAngleView() const { return m_faceAngleView; }
//        public IFaceAngleStorageControl[] retrieveFaceAngles() {
//            return m_faceAngles;
//        }
//
//        public IFaceAngleStorageView[] retrieveFaceAngleView() {
//            return m_faceAngleView;
//        }
//
//        // protected:
//        // bool allocateFaceAngles(FaceAngleStorageMethod storageMethod);
//        // void freeFaceAngles();
//
//        protected boolean haveFaceAnglesBeenBuilt() {
//            return m_faceAngles != null;
//        }
//
//        //public enum MeshComponentUseFlags {
//        public static final int CUF__USE_EDGES_MIN = 0x01;
//        public static final int CUF_USE_FIRST_EDGE = CUF__USE_EDGES_MIN << dMTV_FIRST;
//        public static final int CUF_USE_SECOND_EDGE = CUF__USE_EDGES_MIN << dMTV_SECOND;
//        public static final int CUF_USE_THIRD_EDGE = CUF__USE_EDGES_MIN << dMTV_THIRD;
//        public static final int CUF__USE_EDGES_MAX = CUF__USE_EDGES_MIN << dMTV__MAX;
//        public static final int CUF__USE_ALL_EDGES = CUF_USE_FIRST_EDGE | CUF_USE_SECOND_EDGE | CUF_USE_THIRD_EDGE;
//
//        public static final int CUF__USE_VERTICES_MIN = CUF__USE_EDGES_MAX;
//        public static final int CUF_USE_FIRST_VERTEX = CUF__USE_VERTICES_MIN << dMTV_FIRST;
//        public static final int CUF_USE_SECOND_VERTEX = CUF__USE_VERTICES_MIN << dMTV_SECOND;
//        public static final int CUF_USE_THIRD_VERTEX = CUF__USE_VERTICES_MIN << dMTV_THIRD;
//        public static final int CUF__USE_VERTICES_LAST = CUF__USE_VERTICES_MIN << (dMTV__MAX - 1);
//        public static final int CUF__USE_VERTICES_MAX = CUF__USE_VERTICES_MIN << dMTV__MAX;
//        public static final int CUF__USE_ALL_VERTICES =
//                CUF_USE_FIRST_VERTEX | CUF_USE_SECOND_VERTEX | CUF_USE_THIRD_VERTEX;
//
//        public static final int CUF__USE_ALL_COMPONENTS = CUF__USE_ALL_VERTICES | CUF__USE_ALL_EDGES;
//
//        static {
//            // Make sure that the flags match the values declared in public interface
//            dSASSERT(CUF_USE_FIRST_EDGE == dMESHDATAUSE_EDGE1);
//            dSASSERT(CUF_USE_SECOND_EDGE == dMESHDATAUSE_EDGE2);
//            dSASSERT(CUF_USE_THIRD_EDGE == dMESHDATAUSE_EDGE3);
//            dSASSERT(CUF_USE_FIRST_VERTEX == dMESHDATAUSE_VERTEX1);
//            dSASSERT(CUF_USE_SECOND_VERTEX == dMESHDATAUSE_VERTEX2);
//            dSASSERT(CUF_USE_THIRD_VERTEX == dMESHDATAUSE_VERTEX3);
//        }
//
//        //protected:
//        static class EdgeRecord {
//            // public:
//            // void setupEdge(dMeshTriangleVertex edgeIdx, int triIdx, const unsigned vertexIndices[dMTV__MAX]);
//
//            // Get the vertex opposite this edge in the triangle
//            //DMeshTriangleVertex
//            int getOppositeVertexIndex() {
//                extern const
//                CEnumUnsortedElementArray < unsigned, dxTriDataBase::CUF__USE_VERTICES_LAST / dxTriDataBase::CUF__USE_VERTICES_MIN, dMeshTriangleVertex, 0x161116DC > g_VertFlagOppositeIndices;
//
//                //dMeshTriangleVertex
//                int oppositeIndex = g_VertFlagOppositeIndices.Encode(((m_vert1Flags | m_vert2Flags) ^ CUF__USE_ALL_VERTICES) / CUF__USE_VERTICES_MIN - 1);
//                dIASSERT(dIN_RANGE(oppositeIndex, dMTV__MIN, dMTV__MAX));
//
//                return oppositeIndex;
//            }
//
//            //DMeshTriangleVertex
//            int getEdgeStartVertexIndex() {
//                extern const
//                CEnumUnsortedElementArray < unsigned, dxTriDataBase::CUF__USE_VERTICES_LAST / dxTriDataBase::CUF__USE_VERTICES_MIN, dMeshTriangleVertex, 0x161225E9 > g_VertFlagEdgeStartIndices;
//
//                //dMeshTriangleVertex
//                int startIndex = g_VertFlagEdgeStartIndices.Encode(((m_vert1Flags | m_vert2Flags) ^ CUF__USE_ALL_VERTICES) / CUF__USE_VERTICES_MIN - 1);
//                dIASSERT(dIN_RANGE(startIndex, dMTV__MIN, dMTV__MAX));
//
//                return startIndex;
//            }
//
//            //  public
//            //  bool operator <(const EdgeRecord &anotherEdge)const
//            //  {
//            //      return m_vertIdx1 < anotherEdge.m_vertIdx1 || (m_vertIdx1 == anotherEdge.m_vertIdx1 && m_vertIdx2 < anotherEdge.m_vertIdx2);
//            //  }
//            public boolean isLessThan(EdgeRecord anotherEdge) {
//                return m_vertIdx1 < anotherEdge.m_vertIdx1 ||
//                        (m_vertIdx1 == anotherEdge.m_vertIdx1 && m_vertIdx2 < anotherEdge.m_vertIdx2);
//            }
//
//
//            //public enum
//            //{
//            public static final int AVF_VERTEX_USED = 0x01;
//            public static final int AVF_VERTEX_HAS_CONCAVE_EDGE = 0x02;
//            //}
//
//            //public:
//            int m_vertIdx1;    // Index into vertex array for this edges vertices
//            int m_vertIdx2;
//            int m_triIdx;        // Index into triangle array for triangle this edge belongs to
//
//            //uint8
//            byte m_edgeFlags;
//            byte m_vert1Flags;
//            byte m_vert2Flags;
//            byte m_absVertexFlags;
//        }
//
//        ;
//
//        class VertexRecord {
//            int m_UsedFromEdgeIndex;
//        }
//
//        ;
//
//        //        template<class TMeshDataAccessor>
//        //        static void meaningfulPreprocess_SetupEdgeRecords(EdgeRecord *edges, size_t numEdges, const TMeshDataAccessor &dataAccessor);
//        //        template<class TMeshDataAccessor>
//        //        static void meaningfulPreprocess_buildEdgeFlags(uint8 *useFlags/*=NULL*/, IFaceAngleStorageControl *faceAngles/*=NULL*/,
//        //            EdgeRecord *edges, size_t numEdges, VertexRecord *vertices,
//        //        const dReal *externalNormals, const TMeshDataAccessor &dataAccessor);
//        //        static void buildBoundaryEdgeAngle(IFaceAngleStorageControl *faceAngles, EdgeRecord *currEdge);
//        //        template<class TMeshDataAccessor>
//        //        static void buildConcaveEdgeAngle(IFaceAngleStorageControl *faceAngles, bool negativeAnglesStored,
//        //            EdgeRecord *currEdge, const dReal &normalSegmentDot, const dReal &lengthSquareProduct,
//        //        const dVector3 &triangleNormal, const dVector3 &secondOppositeVertexSegment,
//        //        const dVector3 *pSecondTriangleMatchingEdge/*=NULL*/, const dVector3 *pFirstTriangle/*=NULL*/,
//        //        const TMeshDataAccessor &dataAccessor);
//        //        template<class TMeshDataAccessor>
//        //        static
//        //        void buildConvexEdgeAngle(IFaceAngleStorageControl *faceAngles,
//        //            EdgeRecord *currEdge, const dReal &normalSegmentDot, const dReal &lengthSquareProduct,
//        //        const dVector3 &triangleNormal, const dVector3 &secondOppositeVertexSegment,
//        //        const dVector3 *pSecondTriangleMatchingEdge/*=NULL*/, const dVector3 *pFirstTriangle/*=NULL*/,
//        //        const TMeshDataAccessor &dataAccessor);
//        //        template<class TMeshDataAccessor>
//        //        static dReal calculateEdgeAngleValidated(unsigned firstVertexStartIndex,
//        //            EdgeRecord *currEdge, const dReal &normalSegmentDot, const dReal &lengthSquareProduct,
//        //        const dVector3 &triangleNormal, const dVector3 &secondOppositeVertexSegment,
//        //        const dVector3 *pSecondTriangleMatchingEdge/*=NULL*/, const dVector3 *pFirstTriangle/*=NULL*/,
//        //        const TMeshDataAccessor &dataAccessor);
//
//        //        private:
//        //    const void *m_vertices;
//        private final Void[] m_vertices;
//        private int m_vertexStride;
//        private int m_vertexCount;
//        //const void *m_indices;
//        private final Void[] m_indices;
//        private int m_triangleCount;
//        private int m_triStride;
//        private boolean m_single;
//
//        //        private:
//        //        const void *m_normals;
//        //        IFaceAngleStorageControl *m_faceAngles;
//        //        IFaceAngleStorageView *m_faceAngleView;
//        private final Void[] m_normals;
//        private IFaceAngleStorageControl[] m_faceAngles;
//        private IFaceAngleStorageView[] m_faceAngleView;
//    }
//
//    ;
//
//
//    //    typedef dxGeom dxMeshBase_Parent;
//    //    struct dxMeshBase:
//    //    public dxMeshBase_Parent
//    class DxMeshBase extends DxGeom {
//        //        public:
//        //        dxMeshBase(dxSpace *Space, dxTriDataBase *Data,
//        //                dTriCallback *Callback, dTriArrayCallback *ArrayCallback, dTriRayCallback *RayCallback,
//        //                bool doTCs=false):
//        public DxMeshBase(DxSpace Space, DxTriDataBase Data, DTriMesh.DTriCallback Callback, DTriArrayCallback ArrayCallback, dTriRayCallback RayCallback, boolean doTCs=false) {
//            super(Space, true);
//            m_Callback = Callback;
//            m_ArrayCallback = ArrayCallback;
//            m_RayCallback = RayCallback;
//            m_TriMergeCallback = null;
//            m_Data = Data;
//            //std::fill(m_DoTCs, m_DoTCs + dARRAY_SIZE(m_DoTCs), doTCs);
//            Arrays.fill(m_DoTCs, doTCs);
//            type = dTriMeshClass;
//        }
//
//        boolean invokeCallback(DxGeom Object, int TriIndex) {
//            return m_Callback == null || m_Callback(this, Object, TriIndex) != 0;
//        }
//
//        public enum TRIMESHTC {
//            TTC__MIN,
//
//            TTC_SPHERE =TTC__MIN,
//            TTC_BOX,
//            TTC_CAPSULE,
//
//            TTC__MAX,
//        }
//
//        ;
//
//        //public:
//        public void assignCallback(DTriMesh.DTriCallback value) {
//            m_Callback = value;
//        }
//
//        public DTriMesh.DTriCallback retrieveCallback() {
//            return m_Callback;
//        }
//
//        public void assignArrayCallback(DTriMesh.DTriArrayCallback value) {
//            m_ArrayCallback = value;
//        }
//
//        public DTriMesh.DTriArrayCallback retrieveArrayCallback() {
//            return m_ArrayCallback;
//        }
//
//        public void assignRayCallback(DTriMesh.DTriRayCallback value) {
//            m_RayCallback = value;
//        }
//
//        public DTriMesh.DTriRayCallback retrieveRayCallback() {
//            return m_RayCallback;
//        }
//
//        public void assignTriMergeCallback(DTriMesh.DTriTriMergeCallback value) {
//            m_TriMergeCallback = value;
//        }
//
//        public DTriMesh.DTriTriMergeCallback retrieveTriMergeCallback() {
//            return m_TriMergeCallback;
//        }
//
//        public void assignMeshData(DxTriDataBase instance) {
//            setMeshData(instance);
//            // I changed my data -- I know nothing about my own AABB anymore.
//            markAABBBad();
//        }
//
//        //dxTriDataBase *retrieveMeshData() const { return getMeshData(); }
//        public DxTriDataBase retrieveMeshData() {
//            return getMeshData();
//        }
//
//        //        IFaceAngleStorageControl *retrieveFaceAngleStorage() { return m_Data.retrieveFaceAngles(); }
//        //        IFaceAngleStorageView *retrieveFaceAngleView() { return m_Data.retrieveFaceAngleView(); }
//        public IFaceAngleStorageControl[] retrieveFaceAngleStorage() {
//            return m_Data.retrieveFaceAngles();
//        }
//
//        public IFaceAngleStorageView[] retrieveFaceAngleView() {
//            return m_Data.retrieveFaceAngleView();
//        }
//
//
//        public void assignDoTC(TRIMESHTC tc, boolean value) {
//            setDoTC(tc, value);
//        }
//
//        public boolean retrieveDoTC(TRIMESHTC tc) {
//            return getDoTC(tc);
//        }
//
//        //public:
//        //  void setDoTC(TRIMESHTC tc, bool value) { dIASSERT(dIN_RANGE(tc, TTC__MIN, TTC__MAX)); m_DoTCs[tc] = value; }
//        //  bool getDoTC(TRIMESHTC tc) const { dIASSERT(dIN_RANGE(tc, TTC__MIN, TTC__MAX)); return m_DoTCs[tc]; }
//        public void setDoTC(TRIMESHTC tc, boolean value) {
//            dIASSERT(dIN_RANGE(tc, TTC__MIN, TTC__MAX));
//            m_DoTCs[tc] = value;
//        }
//
//        public boolean getDoTC(TRIMESHTC tc) {
//            dIASSERT(dIN_RANGE(tc, TTC__MIN, TTC__MAX));
//            return m_DoTCs[tc];
//        }
//
//
//        //private:
//        private void setMeshData(DxTriDataBase Data) {
//            m_Data = Data;
//        }
//
//        //protected:
//        protected DxTriDataBase getMeshData() {
//            return m_Data;
//        }
//
//        //public:
//        // Callbacks
//        public DTriCallback m_Callback;
//        public DTriArrayCallback m_ArrayCallback;
//        public DTriRayCallback m_RayCallback;
//        public DTriTriMergeCallback m_TriMergeCallback;
//
//        // Data types
//        private DxTriDataBase m_Data;
//
//        public boolean[] m_DoTCs = new boolean[TTC__MAX];
//    }
//
//    ;
//
//
//    IFaceAngleStorageView[] dxGeomTriMeshGetFaceAngleView(DxGeom triMeshGeom);
//
//    //  #endif	//_ODE_COLLISION_TRIMESH_INTERNAL_H_
//
//
//    // **************************************************
//    //  collision_trimesh_internal_impl.h
//    // **************************************************
//
//    //#ifndef _ODE_COLLISION_TRIMESH_INTERNAL_IMPL_H_
//    //#define _ODE_COLLISION_TRIMESH_INTERNAL_IMPL_H_
//
//    //        #if dTRIMESH_ENABLED
//
//
//    //    template<typename tcoordfloat, typename tindexint>
//    //    /*static */
//    //    void dxTriDataBase::retrieveTriangleVertexPoints(dVector3 out_Points[dMTV__MAX], unsigned triangleIndex,
//    //    const tcoordfloat *vertexInstances, int vertexStride, const tindexint *triangleVertexIndices, int triangleStride)
//    <tcoordfloat, tindexint>
//    /*static */
//    void retrieveTriangleVertexPoints(DVector3[] out_Points, int triangleIndex, final tcoordfloat[] vertexInstances, int vertexStride, final tindexint[] triangleVertexIndices, int triangleStride) {
//        //final tindexint[] triangleIndicesOfInterest = (const tindexint *)((uint8 *)triangleVertexIndices + (size_t)triangleIndex * triangleStride);
//        final tindexint[] triangleIndicesOfInterest = (const tindexint *)((uint8 *) triangleVertexIndices + triangleIndex * triangleStride);
//        for (int trianglePoint = dMTV__MIN; trianglePoint != dMTV__MAX; ++trianglePoint) {
//            int vertexIndex = triangleIndicesOfInterest[trianglePoint];
//            //tcoordfloat * pointVertex = (tcoordfloat *) ((uint8 *) vertexInstances + (size_t) vertexIndex * vertexStride)
//            tcoordfloat[] pointVertex = (tcoordfloat *) ((uint8 *) vertexInstances + vertexIndex * vertexStride)
//
//            dAssignVector3(out_Points[trianglePoint], (dReal) pointVertex[dSA_X], (dReal) pointVertex[dSA_Y], (dReal) pointVertex[dSA_Z]);
//            dSASSERT(dSA_X == 0);
//            dSASSERT(dSA_Y == 1);
//            dSASSERT(dSA_Z == 2);
//        }
//    }
//
//
//    //    template<class TMeshDataAccessor>
//    //    /*static */
//    //    void dxTriDataBase::meaningfulPreprocess_SetupEdgeRecords(EdgeRecord *edges, size_t numEdges, const TMeshDataAccessor &dataAccessor)
//    <TMeshDataAccessor>
//    /*static */
//    void meaningfulPreprocess_SetupEdgeRecords(EdgeRecord[] edges, int numEdges, final TMeshDataAccessor dataAccessor) {
//        int[] vertexIndices = new int[dMTV__MAX];
//        // Make a list of every edge in the mesh
//        int triangleIdx = 0;
//        for (int edgeIdx = 0; edgeIdx != numEdges; ++triangleIdx, edgeIdx += dMTV__MAX) {
//            dataAccessor.getTriangleVertexIndices(vertexIndices, triangleIdx);
//            edges[edgeIdx + dMTV_FIRST].setupEdge(dMTV_FIRST, triangleIdx, vertexIndices);
//            edges[edgeIdx + dMTV_SECOND].setupEdge(dMTV_SECOND, triangleIdx, vertexIndices);
//            edges[edgeIdx + dMTV_THIRD].setupEdge(dMTV_THIRD, triangleIdx, vertexIndices);
//        }
//    }
//
//    //    template<class TMeshDataAccessor>
//    //    /*static */
//    //    void dxTriDataBase::meaningfulPreprocess_buildEdgeFlags(uint8 *useFlags/*=NULL*/, IFaceAngleStorageControl *faceAngles/*=NULL*/,
//    //                                                            EdgeRecord *edges, size_t numEdges, VertexRecord *vertices,
//    //    const dReal *externalNormals/*=NULL*/, const TMeshDataAccessor &dataAccessor)
//    <TMeshDataAccessor>
//    /*static */
//    void meaningfulPreprocess_buildEdgeFlags(byte[] useFlags/*=NULL*/, IFaceAngleStorageControl[] faceAngles/*=NULL*/, EdgeRecord[] edges, int numEdges, VertexRecord[] vertices, final double[] externalNormals/*=NULL*/, final TMeshDataAccessor dataAccessor) {
//        dIASSERT(useFlags != null || faceAngles != null);
//        dIASSERT(numEdges != 0);
//
//        final boolean negativeAnglesStored = faceAngles != null && faceAngles.areNegativeAnglesStored();
//
//        // Go through the sorted list of edges and flag all the edges and vertices that we need to use
//        //EdgeRecord *const lastEdge = edges + (numEdges - 1);
//        final int lastEdgeOfs = (numEdges - 1);
//        for (EdgeRecord * currEdge = edges; ; ++currEdge) {
//            // Handle the last edge separately to have an optimizer friendly loop
//            if (currEdge >= lastEdge) {
//                // This is a boundary edge
//                if (currEdge == lastEdge) {
//                    if (faceAngles != NULL) {
//                        buildBoundaryEdgeAngle(faceAngles, currEdge);
//                    }
//
//                    if (useFlags != null) {
//                        // For the last element EdgeRecord::kAbsVertexUsed assignment can be skipped as noone is going to need it any more
//                        useFlags[currEdge[0].m_triIdx] |= ((edges[currEdge[0].m_vertIdx1].m_absVertexFlags & EdgeRecord::AVF_VERTEX_USED) == 0 ? currEdge[0].m_vert1Flags : 0) | ((edges[currEdge[0].m_vertIdx2].m_absVertexFlags & EdgeRecord::AVF_VERTEX_USED) == 0 ? currEdge[0].m_vert2Flags : 0) | currEdge[0].m_edgeFlags;
//                    }
//                }
//
//                break;
//            }
//
//            int vertIdx1 = currEdge[0].m_vertIdx1;
//            int vertIdx2 = currEdge[0].m_vertIdx2;
//
//            if (vertIdx2 == currEdge[1].m_vertIdx2 // Check second vertex first as it is more likely to change taking the sorting rules into account
//                    && vertIdx1 == currEdge[1].m_vertIdx1) {
//                // We let the dot threshold for concavity get slightly negative to allow for rounding errors
//                final float kConcaveThreshold = 0.000001f;
//
//                //const dVector3 *pSecondTriangleEdgeToUse = NULL, *pFirstTriangleToUse = NULL;
//                DVector3C pSecondTriangleEdgeToUse = null, pFirstTriangleToUse = null;
//                DVector3 secondTriangleMatchingEdge;
//                DVector3 firstTriangle[ dMTV__MAX];
//                DVector3 secondOppositeVertexSegment, triangleNormal;
//                double lengthSquareProduct, secondOppositeSegmentLengthSquare;
//
//                // Calculate orthogonal vector from the matching edge of the second triangle to its opposite point
//                {
//                    DVector3[] secondTriangle[ dMTV__MAX];
//                    dataAccessor.getTriangleVertexPoints(secondTriangle, currEdge[1].m_triIdx);
//
//                    // Get the vertex opposite this edge in the second triangle
//                    //DMeshTriangleVertex * 3
//                    int secondOppositeVertex = currEdge[1].getOppositeVertexIndex();
//                    int secondEdgeStart = secondOppositeVertex + 1 != dMTV__MAX ? (secondOppositeVertex + 1) : dMTV__MIN;
//                    int secondEdgeEnd = (dMTV_FIRST + dMTV_SECOND + dMTV_THIRD - secondEdgeStart - secondOppositeVertex);
//
//                    dSubtractVectors3(secondTriangleMatchingEdge, secondTriangle[secondEdgeEnd], secondTriangle[secondEdgeStart]);
//
//                    if (dSafeNormalize3(secondTriangleMatchingEdge)) {
//                        pSecondTriangleEdgeToUse = &secondTriangleMatchingEdge;
//
//                        DVector3 secondTriangleOppositeEdge = new DVector3();
//                        dSubtractVectors3(secondTriangleOppositeEdge, secondTriangle[secondOppositeVertex], secondTriangle[secondEdgeStart]);
//                        double dProjectionLength = dCalcVectorDot3(secondTriangleOppositeEdge, secondTriangleMatchingEdge);
//                        dAddVectorScaledVector3(secondOppositeVertexSegment, secondTriangleOppositeEdge, secondTriangleMatchingEdge, -dProjectionLength);
//                    } else {
//                        dSubtractVectors3(secondOppositeVertexSegment, secondTriangle[secondOppositeVertex], secondTriangle[secondEdgeStart]);
//                    }
//
//                    secondOppositeSegmentLengthSquare = dCalcVectorLengthSquare3(secondOppositeVertexSegment);
//                }
//
//                // Either calculate the normal from triangle vertices...
//                if (externalNormals == null) {
//                    // Get the normal of the first triangle
//                    dataAccessor.getTriangleVertexPoints(firstTriangle, currEdge[0].m_triIdx);
//                    pFirstTriangleToUse = &firstTriangle[dMTV__MIN];
//
//                    DVector3 firstEdge = new DVector3(), secondEdge = new DVector3();
//                    dSubtractVectors3(secondEdge, firstTriangle[dMTV_THIRD], firstTriangle[dMTV_SECOND]);
//                    dSubtractVectors3(firstEdge, firstTriangle[dMTV_FIRST], firstTriangle[dMTV_SECOND]);
//                    dCalcVectorCross3(triangleNormal, secondEdge, firstEdge);
//                    double normalLengthSuqare = dCalcVectorLengthSquare3(triangleNormal);
//                    lengthSquareProduct = secondOppositeSegmentLengthSquare * normalLengthSuqare;
//                }
//                // ...or use the externally supplied normals
//                else {
//                const dReal * pTriangleExternalNormal = externalNormals + currEdge[0].m_triIdx * dSA__MAX;
//                    dAssignVector3(triangleNormal, pTriangleExternalNormal[dSA_X], pTriangleExternalNormal[dSA_Y], pTriangleExternalNormal[dSA_Z]);
//                    // normalLengthSuqare = REAL(1.0);
//                    dUASSERT(dFabs(dCalcVectorLengthSquare3(triangleNormal) - (1.0)) < (0.25) * kConcaveThreshold * kConcaveThreshold, "Mesh triangle normals must be normalized");
//
//                    lengthSquareProduct = secondOppositeSegmentLengthSquare/* * normalLengthSuqare*/;
//                }
//
//                double normalSegmentDot = dCalcVectorDot3(triangleNormal, secondOppositeVertexSegment);
//
//                // This is a concave edge, leave it for the next pass
//                // OD: This is the "dot >= kConcaveThresh" check, but since the vectros were not normalized to save on roots and divisions,
//                // the check against zero is performed first and then the dot product is squared and compared against the threshold multiplied by lengths' squares
//                // OD: Originally, there was dot > -kConcaveThresh check, but this does not seem to be a good idea
//                // as it can mark all edges on potentially large (nearly) flat surfaces concave.
//                if (normalSegmentDot > 0.0 && normalSegmentDot * normalSegmentDot >= kConcaveThreshold * kConcaveThreshold * lengthSquareProduct) {
//                    if (faceAngles != null) {
//                        buildConcaveEdgeAngle(faceAngles, negativeAnglesStored, currEdge, normalSegmentDot, lengthSquareProduct, triangleNormal, secondOppositeVertexSegment, pSecondTriangleEdgeToUse, pFirstTriangleToUse, dataAccessor);
//                    }
//
//                    if (useFlags != null) {
//                        // Mark the vertices of a concave edge to prevent their use
//                        int absVertexFlags1 = edges[vertIdx1].m_absVertexFlags;
//                        edges[vertIdx1].m_absVertexFlags |= absVertexFlags1 | EdgeRecord::AVF_VERTEX_HAS_CONCAVE_EDGE | EdgeRecord::AVF_VERTEX_USED;
//
//                        if ((absVertexFlags1 & (EdgeRecord::AVF_VERTEX_HAS_CONCAVE_EDGE | EdgeRecord::AVF_VERTEX_USED)) == EdgeRecord::AVF_VERTEX_USED) {
//                            // If the vertex was already used from other triangles but then discovered
//                            // to have a concave edge, unmark the previous use
//                            int usedFromEdgeIndex = vertices[vertIdx1].m_UsedFromEdgeIndex;
//                        const EdgeRecord * usedFromEdge = edges + usedFromEdgeIndex;
//                            int usedInTriangleIndex = usedFromEdge.m_triIdx;
//                            byte usedVertFlags = usedFromEdge.m_vertIdx1 == vertIdx1 ? usedFromEdge.m_vert1Flags : usedFromEdge.m_vert2Flags;
//                            useFlags[usedInTriangleIndex] ^= usedVertFlags;
//                            dIASSERT((useFlags[usedInTriangleIndex] & usedVertFlags) == 0);
//                        }
//
//                        unsigned absVertexFlags2 = edges[vertIdx2].m_absVertexFlags;
//                        edges[vertIdx2].m_absVertexFlags = absVertexFlags2 | EdgeRecord::AVF_VERTEX_HAS_CONCAVE_EDGE | EdgeRecord::AVF_VERTEX_USED;
//
//                        if ((absVertexFlags2 & (EdgeRecord::AVF_VERTEX_HAS_CONCAVE_EDGE | EdgeRecord::AVF_VERTEX_USED)) == EdgeRecord::AVF_VERTEX_USED) {
//                            // Similarly unmark the possible previous use of the edge's second vertex
//                            int usedFromEdgeIndex = vertices[vertIdx2].m_UsedFromEdgeIndex;
//                        const EdgeRecord * usedFromEdge = edges + usedFromEdgeIndex;
//                            int usedInTriangleIndex = usedFromEdge -> m_triIdx;
//                            byte usedVertFlags = usedFromEdge -> m_vertIdx1 == vertIdx2 ? usedFromEdge.m_vert1Flags : usedFromEdge.m_vert2Flags;
//                            useFlags[usedInTriangleIndex] ^= usedVertFlags;
//                            dIASSERT((useFlags[usedInTriangleIndex] & usedVertFlags) == 0);
//                        }
//                    }
//                }
//                // If this is a convex edge, mark its vertices and edge as used
//                else {
//                    if (faceAngles != null) {
//                        buildConvexEdgeAngle(faceAngles, currEdge, normalSegmentDot, lengthSquareProduct, triangleNormal, secondOppositeVertexSegment, pSecondTriangleEdgeToUse, pFirstTriangleToUse, dataAccessor);
//                    }
//
//                    if (useFlags != null) {
//                        EdgeRecord * edgeToUse = currEdge;
//                        int triIdx = edgeToUse[0].m_triIdx;
//                        int triIdx1 = edgeToUse[1].m_triIdx;
//
//                        int triUseFlags = useFlags[triIdx];
//                        int triUseFlags1 = useFlags[triIdx1];
//
//                        // Choose to add flags to the bitmask that already has more edges
//                        // (to group flags in selected triangles rather than scattering them evenly)
//                        if ((triUseFlags1 & CUF__USE_ALL_EDGES) > (triUseFlags & CUF__USE_ALL_EDGES)) {
//                            triIdx = triIdx1;
//                            triUseFlags = triUseFlags1;
//                            edgeToUse = edgeToUse + 1;
//                        }
//
//                        if ((edges[vertIdx1].m_absVertexFlags & EdgeRecord::AVF_VERTEX_USED) == 0) {
//                            // Only add each vertex once and set a mark to prevent further additions
//                            edges[vertIdx1].m_absVertexFlags |= EdgeRecord::AVF_VERTEX_USED;
//                            // Also remember the index the vertex flags are going to be applied to
//                            // to allow easily clear the vertex from the use flags if any concave edges are found to connect to it
//                            vertices[vertIdx1].m_UsedFromEdgeIndex = (unsigned) (edgeToUse - edges);
//                            triUseFlags |= edgeToUse[0].m_vert1Flags;
//                        }
//
//                        // Same processing for the second vertex...
//                        if ((edges[vertIdx2].m_absVertexFlags & EdgeRecord::AVF_VERTEX_USED) == 0) {
//                            edges[vertIdx2].m_absVertexFlags |= EdgeRecord::AVF_VERTEX_USED;
//                            vertices[vertIdx2].m_UsedFromEdgeIndex = (unsigned) (edgeToUse - edges);
//                            triUseFlags |= edgeToUse[0].m_vert2Flags;
//                        }
//
//                        // And finally store the use flags adding the edge flags in
//                        useFlags[triIdx] = triUseFlags | edgeToUse[0].m_edgeFlags;
//                    }
//                }
//
//                // Skip the second edge
//                ++currEdge;
//            }
//            // This is a boundary edge
//            else {
//                if (faceAngles != null) {
//                    buildBoundaryEdgeAngle(faceAngles, currEdge);
//                }
//
//                if (useFlags != null) {
//                    int triIdx = currEdge[0].m_triIdx;
//                    int triUseExtraFlags = 0;
//
//                    if ((edges[vertIdx1].m_absVertexFlags & EdgeRecord::AVF_VERTEX_USED) == 0) {
//                        edges[vertIdx1].m_absVertexFlags |= EdgeRecord::AVF_VERTEX_USED;
//                        vertices[vertIdx1].m_UsedFromEdgeIndex = (unsigned) (currEdge - edges);
//                        triUseExtraFlags |= currEdge[0].m_vert1Flags;
//                    }
//
//                    if ((edges[vertIdx2].m_absVertexFlags & EdgeRecord::AVF_VERTEX_USED) == 0) {
//                        edges[vertIdx2].m_absVertexFlags |= EdgeRecord::AVF_VERTEX_USED;
//                        vertices[vertIdx2].m_UsedFromEdgeIndex = (unsigned) (currEdge - edges);
//                        triUseExtraFlags |= currEdge[0].m_vert2Flags;
//                    }
//
//                    useFlags[triIdx] |= triUseExtraFlags | currEdge[0].m_edgeFlags;
//                }
//            }
//        }
//    }
//
//    /*static */
//    //    void dxTriDataBase::buildBoundaryEdgeAngle(IFaceAngleStorageControl *faceAngles,
//    //                                               EdgeRecord *currEdge)
//    void buildBoundaryEdgeAngle(IFaceAngleStorageControl[] faceAngles, EdgeRecord[] currEdge) {
//        final double faceAngle = (0.0);
//
//        DMeshTriangleVertex firstVertexStartIndex = currEdge[0].getEdgeStartVertexIndex();
//        faceAngles.assignFacesAngleIntoStorage(currEdge[0].m_triIdx, firstVertexStartIndex, faceAngle);
//        // -- For boundary edges, only the first element is valid
//        // dMeshTriangleVertex secondVertexStartIndex = currEdge[1].getEdgeStartVertexIndex();
//        // faceAngles->assignFacesAngleIntoStorage(currEdge[1].m_TriIdx, secondVertexStartIndex, faceAngle);
//    }
//
//    //    template<class TMeshDataAccessor>
//    //    /*static */
//    //    void dxTriDataBase::buildConcaveEdgeAngle(IFaceAngleStorageControl *faceAngles, bool negativeAnglesStored,
//    //                                              EdgeRecord *currEdge, const dReal &normalSegmentDot, const dReal &lengthSquareProduct,
//    //    const dVector3 &triangleNormal, const dVector3 &secondOppositeVertexSegment,
//    //    const dVector3 *pSecondTriangleMatchingEdge/*=NULL*/, const dVector3 *pFirstTriangle/*=NULL*/,
//    //    const TMeshDataAccessor &dataAccessor)
//    <TMeshDataAccessor>
//    /*static */
//    void buildConcaveEdgeAngle(IFaceAngleStorageControl[] faceAngles, boolean negativeAnglesStored, EdgeRecord[] currEdge, final RefDouble normalSegmentDot, final RefDouble lengthSquareProduct, DVector3C triangleNormal, DVector3C secondOppositeVertexSegment, DVector3C pSecondTriangleMatchingEdge/*=NULL*/, DVector3C pFirstTriangle/*=NULL*/, final TMeshDataAccessor dataAccessor) {
//        double faceAngle;
//        //DMeshTriangleVertex
//        int firstVertexStartIndex = currEdge[0].getEdgeStartVertexIndex();
//
//        // Check if concave angles are stored at all
//        if (negativeAnglesStored) {
//            // The length square product can become zero due to precision loss
//            // when both the normal and the opposite edge vectors are very small.
//            if (lengthSquareProduct != (0.0)) {
//                faceAngle = -calculateEdgeAngleValidated(firstVertexStartIndex, currEdge, normalSegmentDot, lengthSquareProduct, triangleNormal, secondOppositeVertexSegment, pSecondTriangleMatchingEdge, pFirstTriangle, dataAccessor);
//            } else {
//                faceAngle = (0.0);
//            }
//        } else {
//            // If concave angles ate not stored, set an arbitrary negative value
//            faceAngle = -(double) M_PI;
//        }
//
//        faceAngles.assignFacesAngleIntoStorage(currEdge[0].m_triIdx, firstVertexStartIndex, faceAngle);
//        //dMeshTriangleVertex
//        int secondVertexStartIndex = currEdge[1].getEdgeStartVertexIndex();
//        faceAngles.assignFacesAngleIntoStorage(currEdge[1].m_triIdx, secondVertexStartIndex, faceAngle);
//    }
//
//    //    template<class TMeshDataAccessor>
//    //    /*static */
//    //    void dxTriDataBase::buildConvexEdgeAngle(IFaceAngleStorageControl *faceAngles,
//    //                                             EdgeRecord *currEdge, const dReal &normalSegmentDot, const dReal &lengthSquareProduct,
//    //    const dVector3 &triangleNormal, const dVector3 &secondOppositeVertexSegment,
//    //    const dVector3 *pSecondTriangleMatchingEdge/*=NULL*/, const dVector3 *pFirstTriangle/*=NULL*/,
//    //    const TMeshDataAccessor &dataAccessor)
//    <TMeshDataAccessor>
//    /*static */
//    void buildConvexEdgeAngle(IFaceAngleStorageControl[] faceAngles, EdgeRecord[] currEdge, final RefDouble normalSegmentDot, final RefDouble lengthSquareProduct, DVector3C triangleNormal, DVector3 secondOppositeVertexSegment, DVector3C[] pSecondTriangleMatchingEdge/*=NULL*/, final DVector3C[] pFirstTriangle
//            /*=NULL*/, final TMeshDataAccessor dataAccessor) {
//        double faceAngle;
//        //dMeshTriangleVertex
//        int firstVertexStartIndex = currEdge[0].getEdgeStartVertexIndex();
//
//        // The length square product can become zero due to precision loss
//        // when both the normal and the opposite edge vectors are very small.
//        if (normalSegmentDot < (0.0) && lengthSquareProduct != (0.0)) {
//            faceAngle = calculateEdgeAngleValidated(firstVertexStartIndex, currEdge, -normalSegmentDot, lengthSquareProduct, triangleNormal, secondOppositeVertexSegment, pSecondTriangleMatchingEdge, pFirstTriangle, dataAccessor);
//        } else {
//            faceAngle = (0.0);
//        }
//
//        faceAngles.assignFacesAngleIntoStorage(currEdge[0].m_triIdx, firstVertexStartIndex, faceAngle);
//        //dMeshTriangleVertex
//        int secondVertexStartIndex = currEdge[1].getEdgeStartVertexIndex();
//        faceAngles.assignFacesAngleIntoStorage(currEdge[1].m_triIdx, secondVertexStartIndex, faceAngle);
//    }
//
//    //    template<class TMeshDataAccessor>
//    //    /*static */
//    //    dReal dxTriDataBase::calculateEdgeAngleValidated(unsigned firstVertexStartIndex,
//    //                                                     EdgeRecord *currEdge, const dReal &normalSegmentDot, const dReal &lengthSquareProduct,
//    //    const dVector3 &triangleNormal, const dVector3 &secondOppositeVertexSegment,
//    //    const dVector3 *pSecondTriangleMatchingEdge/*=NULL*/, const dVector3 *pFirstTriangle/*=NULL*/,
//    //    const TMeshDataAccessor &dataAccessor)
//    <TMeshDataAccessor>
//    /*static */
//    double calculateEdgeAngleValidated(int firstVertexStartIndex, EdgeRecord[] currEdge, final RefDouble normalSegmentDot, final RefDouble lengthSquareProduct, DVector3C triangleNormal, DVector3C secondOppositeVertexSegment, DVector3C[] pSecondTriangleMatchingEdge/*=NULL*/, DVector3C[] pFirstTriangle
//            /*=NULL*/, final TMeshDataAccessor dataAccessor) {
//        dIASSERT(lengthSquareProduct >= (0.0));
//
//        double result;
//        double angleCosine = normalSegmentDot / dSqrt(lengthSquareProduct);
//
//        if (angleCosine < (1.0)) {
//            DVector3 normalSecondOppositeSegmentCross = new DVector3();
//            dCalcVectorCross3(normalSecondOppositeSegmentCross, triangleNormal, secondOppositeVertexSegment);
//
//            double secondTriangleEdgeDirectionCheck;
//
//            if (pSecondTriangleMatchingEdge != null) {
//                // Check the cross product against the second triangle edge, if possible...
//                secondTriangleEdgeDirectionCheck = dCalcVectorDot3(normalSecondOppositeSegmentCross, * pSecondTriangleMatchingEdge)
//                ;
//            } else {
//                // ...if not, calculate the supposed direction of the second triangle's edge
//                // as negative of first triangle edge. For that cross-multiply the precomputed
//                // first triangle normal by vector from the degenerate edge to its opposite vertex.
//
//                // Retrieve the first triangle points if necessary
//                DVector3[] firstTriangleStorage[ dMTV__MAX];
//                //const dVector3 *pFirstTriangleToUse = pFirstTriangle;
//                int pFirstTriangleToUse = pFirstTriangle;
//
//                if (pFirstTriangle == null) {
//                    dataAccessor.getTriangleVertexPoints(firstTriangleStorage, currEdge[0].m_triIdx);
//                    pFirstTriangleToUse = &firstTriangleStorage[dMTV__MIN];
//                }
//
//                // Calculate the opposite vector
//                int firstTriangleOppositeIndex = firstVertexStartIndex != dMTV__MIN ? firstVertexStartIndex - 1 : dMTV__MAX - 1;
//
//                DVector3 firstOppositeVertexSegment = new DVector3();
//                dSubtractVectors3(firstOppositeVertexSegment, pFirstTriangleToUse[firstTriangleOppositeIndex], pFirstTriangleToUse[firstVertexStartIndex]);
//
//                DVector3 normalFirstOppositeSegmentCross = new DVector3();
//                dCalcVectorCross3(normalFirstOppositeSegmentCross, triangleNormal, firstOppositeVertexSegment);
//
//                // And finally calculate the dot product to compare vector directions
//                secondTriangleEdgeDirectionCheck = dCalcVectorDot3(normalSecondOppositeSegmentCross, normalFirstOppositeSegmentCross);
//            }
//
//            // Negative product means the angle absolute value is less than M_PI_2, positive - greater.
//            result = secondTriangleEdgeDirectionCheck < (0.0) ? dAsin(angleCosine) : M_PI_2 + dAcos(angleCosine);
//        } else {
//            result = (double) M_PI_2;
//            dIASSERT(angleCosine - (1.0) < 1e-4); // The computational error can not be too high because the dot product had been verified to be greater than the concave threshold above
//        }
//
//        return result;
//    }
//
//    // #endif // #if dTRIMESH_ENABLED
//
//
//    // #endif // #ifndef _ODE_COLLISION_TRIMESH_INTERNAL_IMPL_H_
//
//
//    // **************************************************
//    //  collision_trimesh_internal.cpp
//    // **************************************************
//
//    // TriMesh storage classes refactoring and face angle computation code by Oleh Derevenko (C) 2016-2017
//
//    //////////////////////////////////////////////////////////////////////////
//
//    //enum EdgeStorageSignInclusion {
//    public static final int SSI__MIN = 0;
//    public static final int SSI_SIGNED_STORED = SSI__MIN;
//    public static final int SSI_POSITIVE_STORED = SSI_SIGNED_STORED + 1;
//    public static final int SSI__MAX = SSI_POSITIVE_STORED + 1;
//
//
//    //    template<typename TStorageType, EdgeStorageSignInclusion t_SignInclusion>
//    //    class FaceAngleStorageCodec;
//
//    //template<typename TStorageType>
//    <TStorageType extends Number>
//    class FaceAngleStorageCodec<TStorageType, SSI_SIGNED_STORED>
//    {
//        //public:
//        //typedef typename _make_signed<TStorageType>::type storage_type;
//        //enum
//        //{
//            //STORAGE_TYPE_MAX = (typename _make_unsigned<TStorageType>::type)(~(typename _make_unsigned<TStorageType>::type)0) >> 1,
//        final int STORAGE_TYPE_MAX = TStorageType.MAX_VALUE;
//        //};
//
//        static boolean areNegativeAnglesCoded()
//        {
//            return true;
//        }
//
//        static storage_type encodeForStorage(dReal angleValue)
//        {
//            unsigned angleAsInt = (unsigned)dFloor(dFabs(angleValue) * (dReal)(STORAGE_TYPE_MAX / M_PI));
//            unsigned limitedAngleAsInt = dMACRO_MIN(angleAsInt, STORAGE_TYPE_MAX);
//            storage_type result = angleValue < REAL(0.0) ? -(storage_type)limitedAngleAsInt : (storage_type)limitedAngleAsInt;
//            return  result;
//        }
//
//        static FaceAngleDomain classifyStorageValue(storage_type storedValue)
//        {
//            dSASSERT(EAD__MAX == 3);
//
//            return storedValue < 0 ? FAD_CONCAVE : (storedValue == 0 ? FAD_FLAT : FAD_CONVEX);
//        }
//
//        static boolean isAngleDomainStored(FaceAngleDomain domainValue)
//        {
//            return !dTMPL_IN_RANGE(domainValue, FAD__SIGNSTORED_IMPLICITVALUE_MIN, FAD__SIGNSTORED_IMPLICITVALUE_MAX);
//        }
//
//        static double decodeStorageValue(storage_type storedValue)
//        {
//            return storedValue * (double)(M_PI / STORAGE_TYPE_MAX);
//        }
//    };
//
//    template<typename TStorageType>
//    class FaceAngleStorageCodec<TStorageType, SSI_POSITIVE_STORED>
//    {
//        public:
//        typedef typename _make_unsigned<TStorageType>::type storage_type;
//        enum
//        {
//            STORAGE_TYPE_MIN = 0,
//                    STORAGE_TYPE_MAX = (storage_type)(~(storage_type)0),
//        };
//
//        static bool areNegativeAnglesCoded()
//        {
//            return false;
//        }
//
//        static storage_type encodeForStorage(dReal angleValue)
//        {
//            storage_type result = STORAGE_TYPE_MIN;
//
//            if (angleValue >= REAL(0.0))
//            {
//                unsigned angleAsInt = (unsigned)dFloor(angleValue * (dReal)(((STORAGE_TYPE_MAX - STORAGE_TYPE_MIN - 1) / M_PI)));
//                result = (STORAGE_TYPE_MIN + 1) + dMACRO_MIN(angleAsInt, STORAGE_TYPE_MAX - STORAGE_TYPE_MIN - 1);
//            }
//
//            return  result;
//        }
//
//        static FaceAngleDomain classifyStorageValue(storage_type storedValue)
//        {
//            dSASSERT(EAD__MAX == 3);
//
//            return storedValue < STORAGE_TYPE_MIN + 1 ? FAD_CONCAVE : (storedValue == STORAGE_TYPE_MIN + 1 ? FAD_FLAT : FAD_CONVEX);
//        }
//
//        static bool isAngleDomainStored(FaceAngleDomain domainValue)
//        {
//            return dTMPL_IN_RANGE(domainValue, FAD__BYTEPOS_STORED_MIN, FAD__BYTEPOS_STORED_MAX);
//        }
//
//        static dReal decodeStorageValue(storage_type storedValue)
//        {
//            dIASSERT(storedValue >= (STORAGE_TYPE_MIN + 1));
//
//            return (storedValue - (STORAGE_TYPE_MIN + 1)) * (dReal)(M_PI / (STORAGE_TYPE_MAX - STORAGE_TYPE_MIN - 1));
//        }
//    };
//
//    template<class TStorageCodec>
//    class FaceAnglesWrapper:
//    public IFaceAngleStorageControl,
//    public IFaceAngleStorageView
//    {
//        protected:
//        FaceAnglesWrapper(unsigned triangleCount) { setAllocatedTriangleCount(triangleCount); }
//
//        public:
//        virtual ~FaceAnglesWrapper();
//
//        static IFaceAngleStorageControl *allocateInstance(unsigned triangleCount, IFaceAngleStorageView *&out_storageView);
//
//        static bool calculateInstanceSizeRequired(size_t &out_sizeRequired, unsigned triangleCount);
//
//        private:
//        void freeInstance();
//
//        private:
//        typedef typename TStorageCodec::storage_type storage_type;
//        typedef storage_type TriangleFaceAngles[dMTV__MAX];
//
//        struct StorageRecord
//        {
//            StorageRecord(): m_triangleCount(0) {}
//
//            unsigned        m_triangleCount;
//            TriangleFaceAngles  m_triangleFaceAngles[1];
//        };
//
//        static size_t calculateStorageSizeForTriangleCount(unsigned triangleCount)
//        {
//        const unsigned baseIncludedTriangleCount = dSTATIC_ARRAY_SIZE(FaceAnglesWrapper<TStorageCodec>::StorageRecord, m_triangleFaceAngles);
//        const size_t singleTriangleSize = membersize(FaceAnglesWrapper<TStorageCodec>::StorageRecord, m_triangleFaceAngles[0]);
//            return sizeof(FaceAnglesWrapper<TStorageCodec>) + (triangleCount > baseIncludedTriangleCount ? (triangleCount - baseIncludedTriangleCount) * singleTriangleSize : 0U);
//        }
//
//        static size_t calculateTriangleCountForStorageSize(size_t storageSize)
//        {
//            dIASSERT(storageSize >= sizeof(FaceAnglesWrapper<TStorageCodec>));
//
//        const unsigned baseIncludedTriangleCount = dSTATIC_ARRAY_SIZE(FaceAnglesWrapper<TStorageCodec>::StorageRecord, m_triangleFaceAngles);
//        const size_t singleTriangleSize = membersize(FaceAnglesWrapper<TStorageCodec>::StorageRecord, m_triangleFaceAngles[0]);
//            return (storageSize - sizeof(FaceAnglesWrapper<TStorageCodec>)) / singleTriangleSize + baseIncludedTriangleCount;
//        }
//
//        private: // IFaceAngleStorageControl
//        virtual void disposeStorage();
//
//        virtual bool areNegativeAnglesStored() const;
//
//        virtual void assignFacesAngleIntoStorage(unsigned triangleIndex, dMeshTriangleVertex vertexIndex, dReal dAngleValue);
//
//        private: // IFaceAngleStorageView
//        virtual FaceAngleDomain retrieveFacesAngleFromStorage(dReal &out_angleValue, unsigned triangleIndex, dMeshTriangleVertex vertexIndex);
//
//        public:
//        void setFaceAngle(unsigned triangleIndex, dMeshTriangleVertex vertexIndex, dReal dAngleValue)
//        {
//            dIASSERT(dTMPL_IN_RANGE(triangleIndex, 0, getAllocatedTriangleCount()));
//            dIASSERT(dTMPL_IN_RANGE(vertexIndex, dMTV__MIN, dMTV__MAX));
//
//            m_record.m_triangleFaceAngles[triangleIndex][vertexIndex] = TStorageCodec::encodeForStorage(dAngleValue);
//        }
//
//        FaceAngleDomain getFaceAngle(dReal &out_angleValue, unsigned triangleIndex, dMeshTriangleVertex vertexIndex) const
//        {
//            dIASSERT(dTMPL_IN_RANGE(triangleIndex, 0, getAllocatedTriangleCount()));
//            dIASSERT(dTMPL_IN_RANGE(vertexIndex, dMTV__MIN, dMTV__MAX));
//
//            storage_type storedValue = m_record.m_triangleFaceAngles[triangleIndex][vertexIndex];
//            FaceAngleDomain resultDomain = TStorageCodec::classifyStorageValue(storedValue);
//
//            out_angleValue = TStorageCodec::isAngleDomainStored(resultDomain) ? TStorageCodec::decodeStorageValue(storedValue) : REAL(0.0);
//            return resultDomain;
//        }
//
//        private:
//        unsigned getAllocatedTriangleCount() const { return m_record.m_triangleCount; }
//        void setAllocatedTriangleCount(unsigned triangleCount) { m_record.m_triangleCount = triangleCount; }
//
//        private:
//        StorageRecord       m_record;
//    };
//
//
//    template<class TStorageCodec>
//    FaceAnglesWrapper<TStorageCodec>::~FaceAnglesWrapper()
//    {
//    }
//
//
//    template<class TStorageCodec>
//    /*static */
//    IFaceAngleStorageControl *FaceAnglesWrapper<TStorageCodec>::allocateInstance(unsigned triangleCount, IFaceAngleStorageView *&out_storageView)
//    {
//        FaceAnglesWrapper<TStorageCodec> *result = NULL;
//
//        do
//        {
//            size_t sizeRequired;
//            if (!FaceAnglesWrapper<TStorageCodec>::calculateInstanceSizeRequired(sizeRequired, triangleCount))
//            {
//                break;
//            }
//
//            void *bufferPointer = dAlloc(sizeRequired);
//            if (bufferPointer == NULL)
//            {
//                break;
//            }
//
//            result = (FaceAnglesWrapper<TStorageCodec> *)bufferPointer;
//            new(result) FaceAnglesWrapper<TStorageCodec>(triangleCount);
//
//            out_storageView = result;
//        }
//        while (false);
//
//        return result;
//    }
//
//    template<class TStorageCodec>
//    /*static */
//    boolean FaceAnglesWrapper<TStorageCodec>::calculateInstanceSizeRequired(size_t &out_sizeRequired, unsigned triangleCount)
//    {
//        boolean result = false;
//
//        do
//        {
//            size_t triangleMaximumCount = calculateTriangleCountForStorageSize(SIZE_MAX);
//            dIASSERT(triangleCount <= triangleMaximumCount);
//
//            if (triangleCount > triangleMaximumCount) // Check for overflow
//            {
//                break;
//            }
//
//            out_sizeRequired = calculateStorageSizeForTriangleCount(triangleCount); // Trailing alignment is going to be added by memory manager automatically
//            result = true;
//        }
//        while (false);
//
//        return result;
//    }
//
//    template<class TStorageCodec>
//    void FaceAnglesWrapper<TStorageCodec>::freeInstance()
//    {
//        unsigned triangleCount = getAllocatedTriangleCount();
//
//        this->FaceAnglesWrapper<TStorageCodec>::~FaceAnglesWrapper();
//
//        size_t memoryBlockSize = calculateStorageSizeForTriangleCount(triangleCount);
//        dFree(this, memoryBlockSize);
//    }
//
//
//    template<class TStorageCodec>
//    /*virtual */
//    void FaceAnglesWrapper<TStorageCodec>::disposeStorage()
//    {
//        freeInstance();
//    }
//
//    template<class TStorageCodec>
//    /*virtual */
//    bool FaceAnglesWrapper<TStorageCodec>::areNegativeAnglesStored() const
//    {
//        return TStorageCodec::areNegativeAnglesCoded();
//    }
//
//    template<class TStorageCodec>
//    /*virtual */
//    void FaceAnglesWrapper<TStorageCodec>::assignFacesAngleIntoStorage(unsigned triangleIndex, dMeshTriangleVertex vertexIndex, dReal dAngleValue)
//    {
//        setFaceAngle(triangleIndex, vertexIndex, dAngleValue);
//    }
//
//    template<class TStorageCodec>
//    /*virtual */
//    FaceAngleDomain FaceAnglesWrapper<TStorageCodec>::retrieveFacesAngleFromStorage(dReal &out_angleValue, unsigned triangleIndex, dMeshTriangleVertex vertexIndex)
//    {
//        return getFaceAngle(out_angleValue, triangleIndex, vertexIndex);
//    }
//
//
//    typedef IFaceAngleStorageControl *(FAngleStorageAllocProc)(unsigned triangleCount, IFaceAngleStorageView *&out_storageView);
//
//    BEGIN_NAMESPACE_OU();
//    template<>
//    FAngleStorageAllocProc *const CEnumUnsortedElementArray<FaceAngleStorageMethod, ASM__MAX, FAngleStorageAllocProc *, 0x161211AD>::m_aetElementArray[] =
//    {
//    &FaceAnglesWrapper<FaceAngleStorageCodec<uint8, SSI_SIGNED_STORED> >::allocateInstance, // ASM_BYTE_SIGNED,
//    &FaceAnglesWrapper<FaceAngleStorageCodec<uint8, SSI_POSITIVE_STORED> >::allocateInstance, // ASM_BYTE_POSITIVE,
//    &FaceAnglesWrapper<FaceAngleStorageCodec<uint16, SSI_SIGNED_STORED> >::allocateInstance, // ASM_WORD_SIGNED,
//    };
//    END_NAMESPACE_OU();
//    static const CEnumUnsortedElementArray<FaceAngleStorageMethod, ASM__MAX, FAngleStorageAllocProc *, 0x161211AD> g_AngleStorageAllocProcs;
//
//
//    //////////////////////////////////////////////////////////////////////////
//
//    // dxTriDataBase::~dxTriDataBase()
//    void DESTRUCTOR()
//    {
//        freeFaceAngles();
//    }
//
//
//    //    void dxTriDataBase::buildData(const void *vertices, int vertexStride, unsigned vertexCount,
//    //    const void *indices, unsigned indexCount, int triStride,
//    //    const void *normals,
//    //                                  bool single)
//    void buildData(final Void[] vertices, int vertexStride, int vertexCount,
//    final Void[] indices, int indexCount, int triStride,
//    final Void[] normals,
//                              boolean single)
//    {
//        dIASSERT(vertices != null);
//        dIASSERT(indices != null);
//        dIASSERT(vertexStride != 0);
//        dIASSERT(triStride != 0);
//        dIASSERT(indexCount != 0);
//        dIASSERT(indexCount % dMTV__MAX == 0);
//
//        m_vertices = vertices;
//        m_vertexStride = vertexStride;
//        m_vertexCount = vertexCount;
//        m_indices = indices;
//        m_triangleCount = indexCount / dMTV__MAX;
//        m_triStride = triStride;
//        m_single = single;
//
//        m_normals = normals;
//    }
//
//
//    //bool dxTriDataBase::allocateFaceAngles(FaceAngleStorageMethod storageMethod)
//    boolean allocateFaceAngles(FaceAngleStorageMethod storageMethod)
//    {
//        boolean result = false;
//
//        dIASSERT(m_faceAngles == null);
//
//        IFaceAngleStorageView[] storageView;
//
//        int triangleCount = m_triangleCount;
//
//        FAngleStorageAllocProc *allocProc = g_AngleStorageAllocProcs.Encode(storageMethod);
//        IFaceAngleStorageControl *storageInstance = allocProc(triangleCount, storageView);
//
//        if (storageInstance != null)
//        {
//            m_faceAngles = storageInstance;
//            m_faceAngleView = storageView;
//            result = true;
//        }
//
//        return result;
//    }
//
//    //void dxTriDataBase::freeFaceAngles()
//    void freeFaceAngles()
//    {
//        if (m_faceAngles != null)
//        {
//            m_faceAngles.disposeStorage();
//            m_faceAngles = null;
//            m_faceAngleView = null;
//        }
//    }
//
//
//    //void dxTriDataBase::EdgeRecord::setupEdge(dMeshTriangleVertex edgeIdx, int triIdx, const unsigned vertexIndices[dMTV__MAX])
//    void dxTriDataBase::EdgeRecord::setupEdge(int edgeIdx, int triIdx, final int[] vertexIndices[dMTV__MAX])
//    {
//        if (edgeIdx < dMTV_SECOND)
//        {
//            dIASSERT(edgeIdx == dMTV_FIRST);
//
//            m_edgeFlags  = DxTriMeshData.CUF_USE_FIRST_EDGE;
//            m_vert1Flags = DxTriMeshData::CUF_USE_FIRST_VERTEX;
//            m_vert2Flags = DxTriMeshData::CUF_USE_SECOND_VERTEX;
//            m_vertIdx1 = vertexIndices[dMTV_FIRST];
//            m_vertIdx2 = vertexIndices[dMTV_SECOND];
//        }
//        else if (edgeIdx == dMTV_SECOND)
//        {
//            m_edgeFlags  = DxTriMeshData::CUF_USE_SECOND_EDGE;
//            m_vert1Flags = DxTriMeshData::CUF_USE_SECOND_VERTEX;
//            m_vert2Flags = DxTriMeshData::CUF_USE_THIRD_VERTEX;
//            m_vertIdx1 = vertexIndices[dMTV_SECOND];
//            m_vertIdx2 = vertexIndices[dMTV_THIRD];
//        }
//        else
//        {
//            dIASSERT(edgeIdx == dMTV_THIRD);
//
//            m_edgeFlags  = DxTriMeshData::CUF_USE_THIRD_EDGE;
//            m_vert1Flags = DxTriMeshData::CUF_USE_THIRD_VERTEX;
//            m_vert2Flags = DxTriMeshData::CUF_USE_FIRST_VERTEX;
//            m_vertIdx1 = vertexIndices[dMTV_THIRD];
//            m_vertIdx2 = vertexIndices[dMTV_FIRST];
//        }
//
//        // Make sure vertex index 1 is less than index 2 (for easier sorting)
//        if (m_vertIdx1 > m_vertIdx2)
//        {
//            dxSwap(m_vertIdx1, m_vertIdx2);
//            dxSwap(m_vert1Flags, m_vert2Flags);
//        }
//
//        m_triIdx = triIdx;
//        m_absVertexFlags = 0;
//    }
//
//
//    BEGIN_NAMESPACE_OU();
//    template<>
//const dMeshTriangleVertex CEnumUnsortedElementArray<unsigned, dxTriDataBase::CUF__USE_VERTICES_LAST / dxTriDataBase::CUF__USE_VERTICES_MIN, dMeshTriangleVertex, 0x161116DC>::m_aetElementArray[] =
//    {
//        dMTV_FIRST, // kVert0 / kVert_Base
//                dMTV_SECOND, // kVert1 / kVert_Base
//                dMTV__MAX,
//                dMTV_THIRD, // kVert2 / kVert_Base
//    };
//    END_NAMESPACE_OU();
//    /*extern */const CEnumUnsortedElementArray<unsigned, dxTriDataBase::CUF__USE_VERTICES_LAST / dxTriDataBase::CUF__USE_VERTICES_MIN, dMeshTriangleVertex, 0x161116DC> g_VertFlagOppositeIndices;
//
//    BEGIN_NAMESPACE_OU();
//    template<>
//const dMeshTriangleVertex CEnumUnsortedElementArray<unsigned, dxTriDataBase::CUF__USE_VERTICES_LAST / dxTriDataBase::CUF__USE_VERTICES_MIN, dMeshTriangleVertex, 0x161225E9>::m_aetElementArray[] =
//    {
//        dMTV_SECOND, // kVert0 / kVert_Base
//                dMTV_THIRD, // kVert1 / kVert_Base
//                dMTV__MAX,
//                dMTV_FIRST, // kVert2 / kVert_Base
//    };
//    END_NAMESPACE_OU();
//    /*extern */const CEnumUnsortedElementArray<unsigned, dxTriDataBase::CUF__USE_VERTICES_LAST / dxTriDataBase::CUF__USE_VERTICES_MIN, dMeshTriangleVertex, 0x161225E9> g_VertFlagEdgeStartIndices;
//
//
//    //////////////////////////////////////////////////////////////////////////
//
//    /*extern ODE_API */
//    void dGeomTriMeshDataBuildSimple1(dTriMeshDataID g,
//    const dReal* Vertices, int VertexCount,
//    const dTriIndex* Indices, int IndexCount,
//    const int *Normals)
//    {
//#ifdef dSINGLE
//        dGeomTriMeshDataBuildSingle1(g,
//                Vertices, 4 * sizeof(dReal), VertexCount,
//                Indices, IndexCount, 3 * sizeof(dTriIndex),
//                Normals);
//#else
//        dGeomTriMeshDataBuildDouble1(g, Vertices, 4 * sizeof(dReal), VertexCount,
//                Indices, IndexCount, 3 * sizeof(dTriIndex),
//                Normals);
//#endif
//    }
//
//
//    /*extern ODE_API */
//    void dGeomTriMeshDataBuildSingle(dTriMeshDataID g,
//    const void* Vertices, int VertexStride, int VertexCount,
//    const void* Indices, int IndexCount, int TriStride)
//    {
//        dGeomTriMeshDataBuildSingle1(g, Vertices, VertexStride, VertexCount,
//                Indices, IndexCount, TriStride, (const void *)NULL);
//    }
//
//    /*extern ODE_API */
//    void dGeomTriMeshDataBuildDouble(dTriMeshDataID g,
//    const void* Vertices, int VertexStride, int VertexCount,
//    const void* Indices, int IndexCount, int TriStride)
//    {
//        dGeomTriMeshDataBuildDouble1(g, Vertices, VertexStride, VertexCount,
//                Indices, IndexCount, TriStride, NULL);
//    }
//
//    /*extern ODE_API */
//    void dGeomTriMeshDataBuildSimple(dTriMeshDataID g,
//    const dReal* Vertices, int VertexCount,
//    const dTriIndex* Indices, int IndexCount)
//    {
//        dGeomTriMeshDataBuildSimple1(g,
//                Vertices, VertexCount, Indices, IndexCount,
//                (int *)NULL);
//    }
//
//
//    /*extern ODE_API */
//    int dGeomTriMeshDataPreprocess(dTriMeshDataID g)
//    {
//        unsigned buildRequestFlags = (1U << dTRIDATAPREPROCESS_BUILD_CONCAVE_EDGES);
//        return dGeomTriMeshDataPreprocess2(g, buildRequestFlags, NULL);
//    }
//
//
//    BEGIN_NAMESPACE_OU();
//    template<>
//const FaceAngleStorageMethod CEnumUnsortedElementArray<unsigned, dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MAX, FaceAngleStorageMethod, 0x17010902>::m_aetElementArray[] =
//    {
//        ASM_BYTE_POSITIVE, // dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA_BYTE_POSITIVE,
//                ASM_BYTE_SIGNED, // dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA_BYTE_ALL,
//                ASM_WORD_SIGNED, // dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA_WORD_ALL,
//    };
//    END_NAMESPACE_OU();
//    static const CEnumUnsortedElementArray<unsigned, dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MAX, FaceAngleStorageMethod, 0x17010902> g_TriMeshDataPreprocess_FaceAndlesExtraDataAngleStorageMethods;
//
//    /*extern ODE_API */
//    int dGeomTriMeshDataPreprocess2(dTriMeshDataID g, unsigned int buildRequestFlags, const dintptr *requestExtraData/*=NULL | const dintptr (*)[dTRIDATAPREPROCESS_BUILD__MAX]*/)
//    {
//        dUASSERT(g, "The argument is not a trimesh data");
//        dAASSERT((buildRequestFlags & (1U << dTRIDATAPREPROCESS_BUILD_FACE_ANGLES)) == 0 || requestExtraData == NULL || dIN_RANGE(requestExtraData[dTRIDATAPREPROCESS_BUILD_FACE_ANGLES], dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MIN, dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MAX));
//
//        dxTriMeshData *data = g;
//
//        bool buildUseFlags = (buildRequestFlags & (1U << dTRIDATAPREPROCESS_BUILD_CONCAVE_EDGES)) != 0;
//        FaceAngleStorageMethod faceAnglesRequirement = (buildRequestFlags & (1U << dTRIDATAPREPROCESS_BUILD_FACE_ANGLES)) != 0
//            ? g_TriMeshDataPreprocess_FaceAndlesExtraDataAngleStorageMethods.Encode(requestExtraData != NULL && dIN_RANGE(requestExtraData[dTRIDATAPREPROCESS_BUILD_FACE_ANGLES], dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MIN, dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MAX) ? (unsigned)requestExtraData[dTRIDATAPREPROCESS_BUILD_FACE_ANGLES] : dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__DEFAULT)
//            : ASM__INVALID;
//        return data->preprocessData(buildUseFlags, faceAnglesRequirement);
//    }
//
//    /*extern ODE_API */
//    void dGeomTriMeshDataUpdate(dTriMeshDataID g)
//    {
//        dUASSERT(g, "The argument is not a trimesh data");
//
//        dxTriMeshData *data = g;
//        data->updateData();
//    }
//
//
//    //////////////////////////////////////////////////////////////////////////
//
//    /*extern ODE_API */
//    void dGeomTriMeshSetCallback(dGeomID g, dTriCallback* Callback)
//    {
//        dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
//
//        dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
//        mesh->assignCallback(Callback);
//    }
//
//    /*extern ODE_API */
//    dTriCallback* dGeomTriMeshGetCallback(dGeomID g)
//    {
//        dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
//
//    const dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
//        return mesh->retrieveCallback();
//    }
//
//    /*extern ODE_API */
//    void dGeomTriMeshSetArrayCallback(dGeomID g, dTriArrayCallback* ArrayCallback)
//    {
//        dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
//
//        dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
//        mesh->assignArrayCallback(ArrayCallback);
//    }
//
//    /*extern ODE_API */
//    dTriArrayCallback *dGeomTriMeshGetArrayCallback(dGeomID g)
//    {
//        dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
//
//    const dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
//        return mesh->retrieveArrayCallback();
//    }
//
//    /*extern ODE_API */
//    void dGeomTriMeshSetRayCallback(dGeomID g, dTriRayCallback* Callback)
//    {
//        dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
//
//        dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
//        mesh->assignRayCallback(Callback);
//    }
//
//    /*extern ODE_API */
//    dTriRayCallback* dGeomTriMeshGetRayCallback(dGeomID g)
//    {
//        dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
//
//    const dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
//        return mesh->retrieveRayCallback();
//    }
//
//    /*extern ODE_API */
//    void dGeomTriMeshSetTriMergeCallback(dGeomID g, dTriTriMergeCallback* Callback)
//    {
//        dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
//
//        dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
//        mesh->assignTriMergeCallback(Callback);
//    }
//
//    /*extern ODE_API */
//    dTriTriMergeCallback *dGeomTriMeshGetTriMergeCallback(dGeomID g)
//    {
//        dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
//
//    const dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
//        return mesh->retrieveTriMergeCallback();
//    }
//
//    /*extern ODE_API */
//    void dGeomTriMeshSetData(dGeomID g, dTriMeshDataID Data)
//    {
//        dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
//
//        dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
//        mesh->assignMeshData(Data);
//    }
//
//    /*extern ODE_API */
//    dTriMeshDataID dGeomTriMeshGetData(dGeomID g)
//    {
//        dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
//
//    const dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
//        return mesh->retrieveMeshData();
//    }
//
//
//    BEGIN_NAMESPACE_OU();
//    template<>
//const int CEnumSortedElementArray<dxTriMesh::TRIMESHTC, dxTriMesh::TTC__MAX, int, 0x161003D5>::m_aetElementArray[] =
//    {
//        dSphereClass, // TTC_SPHERE,
//                dBoxClass, // TTC_BOX,
//                dCapsuleClass, // TTC_CAPSULE,
//    };
//    END_NAMESPACE_OU();
//    static const CEnumSortedElementArray<dxTriMesh::TRIMESHTC, dxTriMesh::TTC__MAX, int, 0x161003D5> g_asiMeshTCGeomClasses;
//
//    /*extern ODE_API */
//    void dGeomTriMeshEnableTC(dGeomID g, int geomClass, int enable)
//    {
//        dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
//
//        dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
//
//        dxTriMesh::TRIMESHTC tc = g_asiMeshTCGeomClasses.Decode(geomClass);
//
//        if (g_asiMeshTCGeomClasses.IsValidDecode(tc))
//        {
//            mesh->assignDoTC(tc, enable != 0);
//        }
//    }
//
//    /*extern ODE_API */
//    int dGeomTriMeshIsTCEnabled(dGeomID g, int geomClass)
//    {
//        dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
//
//    const dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
//
//        dxTriMesh::TRIMESHTC tc = g_asiMeshTCGeomClasses.Decode(geomClass);
//
//        bool result = g_asiMeshTCGeomClasses.IsValidDecode(tc)
//                && mesh->retrieveDoTC(tc);
//        return result;
//    }
//
//
//    /*extern ODE_API */
//    dTriMeshDataID dGeomTriMeshGetTriMeshDataID(dGeomID g)
//    {
//        dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
//
//    const dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
//        return mesh->retrieveMeshData();
//    }
//
//
//    /*extern ODE_API */
//    void dGeomTriMeshClearTCCache(dGeomID g)
//    {
//        dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
//
//        dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
//        mesh->clearTCCache();
//    }
//
//
//    /*extern ODE_API */
//    int dGeomTriMeshGetTriangleCount(dGeomID g)
//    {
//        dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
//
//    const dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
//        unsigned result = mesh->getMeshTriangleCount();
//        return result;
//    }
//
//
//    /*extern ODE_API */
//    void dGeomTriMeshGetTriangle(dGeomID g, int index, dVector3 *v0/*=NULL*/, dVector3 *v1/*=NULL*/, dVector3 *v2/*=NULL*/)
//    {
//        dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
//        dUASSERT(v0 != NULL || v1 != NULL || v2 != NULL, "A meaningless call");
//
//        dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
//
//        dVector3 *pv[3] = { v0, v1, v2 };
//        mesh->fetchMeshTransformedTriangle(pv, index);
//    }
//
//    /*extern ODE_API */
//    void dGeomTriMeshGetPoint(dGeomID g, int index, dReal u, dReal v, dVector3 Out)
//    {
//        dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
//
//        dxTriMesh *mesh = static_cast<dxTriMesh *>(g);
//
//        dVector3 dv[3];
//        mesh->fetchMeshTransformedTriangle(dv, index);
//
//        GetPointFromBarycentric(dv, u, v, Out);
//    }
//
//
//    /*extern */
//    IFaceAngleStorageView *dxGeomTriMeshGetFaceAngleView(dxGeom *triMeshGeom)
//    {
//        dUASSERT(triMeshGeom && triMeshGeom->type == dTriMeshClass, "The argument is not a trimesh");
//
//        dxTriMesh *mesh = static_cast<dxTriMesh *>(triMeshGeom);
//        return mesh->retrieveFaceAngleView();
//    }
//
//
//    // #endif // #if dTRIMESH_ENABLED
//
//
//    //////////////////////////////////////////////////////////////////////////
//    // Deprecated functions
//
//    /*extern */
//    void dGeomTriMeshDataGetBuffer(dTriMeshDataID g, unsigned char **buf, int *bufLen)
//    {
//        size_t dataSizeStorage;
//        void *dataPointer = dGeomTriMeshDataGet2(g, dTRIMESHDATA_USE_FLAGS, (bufLen != NULL ? &dataSizeStorage : NULL));
//
//        if (bufLen != NULL)
//        {
//        *bufLen = (int)dataSizeStorage;
//        }
//
//        if (buf != NULL)
//        {
//        *buf = (unsigned char *)dataPointer;
//        }
//    }
//
//    /*extern */
//    void dGeomTriMeshDataSetBuffer(dTriMeshDataID g, unsigned char* buf)
//    {
//        dGeomTriMeshDataSet(g, dTRIMESHDATA_USE_FLAGS, (void *)buf);
//    }
//
//
//
//
}
