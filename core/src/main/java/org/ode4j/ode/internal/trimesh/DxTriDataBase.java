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

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DTriMeshData;
import org.ode4j.ode.internal.*;
import org.ode4j.ode.internal.cpp4j.java.Ref;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;
import org.ode4j.ode.ou.CEnumUnsortedElementArray;

import static org.ode4j.ode.DTriMesh.*;
import static org.ode4j.ode.DTriMesh.dMESHDATAUSE.*;
import static org.ode4j.ode.DTriMeshData.dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MAX;
import static org.ode4j.ode.DTriMeshData.dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MIN;
import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.Common.M_PI;
import static org.ode4j.ode.internal.Common.M_PI_2;
import static org.ode4j.ode.internal.CommonEnums.*;

public class DxTriDataBase extends DBase {

    // TZ: FAke Interface to represent templates
    interface TMeshDataAccessorP {
        void getTriangleVertexPoints(DVector3[] firstTriangleStorage, int m_triIdx);
    }
    interface TMeshDataAccessorI {
        void getTriangleVertexIndices(int[] vertexIndices, int triangleIdx);
    }

    // **************************************************
    //  collision_trimesh_trimesh.cpp
    // **************************************************

    //#if !dTLS_ENABLED
    // Have collider cache instance unconditionally of OPCODE or GIMPACT selection
    // public static final TrimeshCollidersCache g_ccTrimeshCollidersCache;
    //#endif


    // **************************************************
    //  collision_trimesh_internal.h
    // **************************************************

    // TriMesh code by Erwin de Vries.
    // Modified for FreeSOLID Compatibility by Rodrigo Hernandez
    // TriMesh caches separation by Oleh Derevenko
    // TriMesh storage classes refactoring and face angle computation code by Oleh Derevenko (C) 2016-2017


    //  #ifndef _ODE_COLLISION_TRIMESH_INTERNAL_H_
    //  #define _ODE_COLLISION_TRIMESH_INTERNAL_H_


    //****************************************************************************
    // dxTriMesh class

    // struct TrimeshCollidersCache;
    // struct dxTriMeshData;


    //    static inline
    //    TrimeshCollidersCache *GetTrimeshCollidersCache(unsigned uiTLSKind)
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


    //enum FaceAngleStorageMethod {
    //    public static final int ASM__MIN = 0;
    //    public static final int ASM_BYTE_SIGNED = ASM__MIN;
    //    public static final int ASM_BYTE_POSITIVE = ASM_BYTE_SIGNED + 1;
    public static final int ASM_WORD_SIGNED = 2;//ASM_BYTE_POSITIVE + 1;
    //    public static final int ASM__MAX = ASM_WORD_SIGNED + 1;
    public static final int ASM__INVALID = 3;//ASM__MAX;

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
    public enum FaceAngleDomain {
        //FAD__MIN = 0;
        FAD_CONCAVE, // = FAD__MIN;
        // FAD__SIGNSTORED_IMPLICITVALUE_MIN = FAD_CONCAVE + 1;
        FAD_FLAT, // = FAD__SIGNSTORED_IMPLICITVALUE_MIN;
        // FAD__SIGNSTORED_IMPLICITVALUE_MAX = FAD_FLAT + 1;
        // FAD__BYTEPOS_STORED_MIN = FAD__SIGNSTORED_IMPLICITVALUE_MAX;
        FAD_CONVEX // = FAD__BYTEPOS_STORED_MIN;
        // FAD__BYTEPOS_STORED_MAX = FAD_CONVEX + 1;
        // EAD__MAX = FAD__BYTEPOS_STORED_MAX;
    }
    //public static final int EAD__MAX = FaceAngleDomain.values().length;

    //typedef dBase dxTriDataBase_Parent;
    //class dxTriDataBase extends dxTriDataBase_Parent
    //class DxTriDataBase extends DBase {
        //public:
        public DxTriDataBase() {
            super();
            m_vertices = null;
            // m_vertexStride = 0;
            m_vertexCount = 0;
            m_indices = null;
            m_triangleCount = 0;
            // m_triStride = 0;
            // m_single = false;
            m_normals = null;
            m_faceAngles = null;
            m_faceAngleView = null;

            if (!dTRIMESH_ENABLED) {
                dUASSERT(false, "dTRIMESH_ENABLED is not defined. Trimesh geoms will not work");
            }
        }

        //~dxTriDataBase();
        //protected void DESTRUCTOR() {
        //    super.DESTRUCTOR();
        //}

        //        void buildData(const void *Vertices, int VertexStide, unsigned VertexCount,
        //        const void *Indices, unsigned IndexCount, int TriStride,
        //        const void *Normals,
        //            bool Single);

        public int retrieveVertexCount() {
            return m_vertexCount;
        }

        //        public int retrieveVertexStride() {
        //            return m_vertexStride; // TZ: This is always 3!
        //        }

        public int retrieveTriangleCount() {
            return m_triangleCount;
        }

        //        public int retrieveTriangleStride() {
        //            //return m_triStride; // TZ: This is always 3!
        //        }

        //protected:
        //    const void *retrieveVertexInstances() const { return m_vertices; }
        //    const void *retrieveTriangleVertexIndices() const { return m_indices; }

        protected float[] retrieveVertexInstances() {
            return m_vertices;
        }

        protected int[] retrieveTriangleVertexIndices() {
            return m_indices;
        }

        //        protected boolean isSingle() {
        //            return m_single;
        //        }

        //        public:
        //        template<typename tcoordfloat, typename
        //        tindexint>
        //        static void retrieveTriangleVertexPoints(dVector3 out_Points[dMTV__MAX], unsigned triangleIndex,
        //        const tcoordfloat *vertexInstances, int vertexStride, const tindexint *triangleVertexIndices, int triangleStride);

        //const void assignNormals(const void *normals) { m_normals = normals; }
        //const void *retrieveNormals() const { return m_normals; }
        public void assignNormals(float[] normals) {
            m_normals = normals;
        }

        public float[] retrieveNormals() {
            return m_normals;
        }

        //        IFaceAngleStorageControl *retrieveFaceAngles() const { return m_faceAngles; }
        //        IFaceAngleStorageView *retrieveFaceAngleView() const { return m_faceAngleView; }
        IFaceAngleStorageControl retrieveFaceAngles() {
            return m_faceAngles;
        }
        public double retrieveFaceAngle(int triangleIndex, int vertexIndex) {
            RefDouble out_angleValue = new RefDouble();
            ((FaceAnglesWrapper)m_faceAngles).getFaceAngle(out_angleValue, triangleIndex, vertexIndex);
            return out_angleValue.get();
        }

        public IFaceAngleStorageView retrieveFaceAngleView() {
            return m_faceAngleView;
        }

        // protected:
        // bool allocateFaceAngles(FaceAngleStorageMethod storageMethod);
        // void freeFaceAngles();

        protected boolean haveFaceAnglesBeenBuilt() {
            return m_faceAngles != null;
        }

        //public enum MeshComponentUseFlags {
        public static final int CUF__USE_EDGES_MIN = 0x01;
        public static final int CUF_USE_FIRST_EDGE = CUF__USE_EDGES_MIN << dMTV_FIRST;
        public static final int CUF_USE_SECOND_EDGE = CUF__USE_EDGES_MIN << dMTV_SECOND;
        public static final int CUF_USE_THIRD_EDGE = CUF__USE_EDGES_MIN << dMTV_THIRD;
        public static final int CUF__USE_EDGES_MAX = CUF__USE_EDGES_MIN << dMTV__MAX;
        public static final int CUF__USE_ALL_EDGES = CUF_USE_FIRST_EDGE | CUF_USE_SECOND_EDGE | CUF_USE_THIRD_EDGE;

        public static final int CUF__USE_VERTICES_MIN = CUF__USE_EDGES_MAX;
        public static final int CUF_USE_FIRST_VERTEX = CUF__USE_VERTICES_MIN << dMTV_FIRST;
        public static final int CUF_USE_SECOND_VERTEX = CUF__USE_VERTICES_MIN << dMTV_SECOND;
        public static final int CUF_USE_THIRD_VERTEX = CUF__USE_VERTICES_MIN << dMTV_THIRD;
        public static final int CUF__USE_VERTICES_LAST = CUF__USE_VERTICES_MIN << (dMTV__MAX - 1);
        // public static final int CUF__USE_VERTICES_MAX = CUF__USE_VERTICES_MIN << dMTV__MAX;
        public static final int CUF__USE_ALL_VERTICES =
                CUF_USE_FIRST_VERTEX | CUF_USE_SECOND_VERTEX | CUF_USE_THIRD_VERTEX;

        // public static final int CUF__USE_ALL_COMPONENTS = CUF__USE_ALL_VERTICES | CUF__USE_ALL_EDGES;

        static {
            // Make sure that the flags match the values declared in public interface
            dSASSERT(CUF_USE_FIRST_EDGE == dMESHDATAUSE_EDGE1);
            dSASSERT(CUF_USE_SECOND_EDGE == dMESHDATAUSE_EDGE2);
            dSASSERT(CUF_USE_THIRD_EDGE == dMESHDATAUSE_EDGE3);
            dSASSERT(CUF_USE_FIRST_VERTEX == dMESHDATAUSE_VERTEX1);
            dSASSERT(CUF_USE_SECOND_VERTEX == dMESHDATAUSE_VERTEX2);
            dSASSERT(CUF_USE_THIRD_VERTEX == dMESHDATAUSE_VERTEX3);
        }

        //protected:


        //        template<class TMeshDataAccessor>
        //        static void meaningfulPreprocess_SetupEdgeRecords(EdgeRecord *edges, size_t numEdges, const TMeshDataAccessor &dataAccessor);
        //        template<class TMeshDataAccessor>
        //        static void meaningfulPreprocess_buildEdgeFlags(uint8 *useFlags/*=NULL*/, IFaceAngleStorageControl *faceAngles/*=NULL*/,
        //            EdgeRecord *edges, size_t numEdges, VertexRecord *vertices,
        //        const dReal *externalNormals, const TMeshDataAccessor &dataAccessor);
        //        static void buildBoundaryEdgeAngle(IFaceAngleStorageControl *faceAngles, EdgeRecord *currEdge);
        //        template<class TMeshDataAccessor>
        //        static void buildConcaveEdgeAngle(IFaceAngleStorageControl *faceAngles, bool negativeAnglesStored,
        //            EdgeRecord *currEdge, const dReal &normalSegmentDot, const dReal &lengthSquareProduct,
        //        const dVector3 &triangleNormal, const dVector3 &secondOppositeVertexSegment,
        //        const dVector3 *pSecondTriangleMatchingEdge/*=NULL*/, const dVector3 *pFirstTriangle/*=NULL*/,
        //        const TMeshDataAccessor &dataAccessor);
        //        template<class TMeshDataAccessor>
        //        static
        //        void buildConvexEdgeAngle(IFaceAngleStorageControl *faceAngles,
        //            EdgeRecord *currEdge, const dReal &normalSegmentDot, const dReal &lengthSquareProduct,
        //        const dVector3 &triangleNormal, const dVector3 &secondOppositeVertexSegment,
        //        const dVector3 *pSecondTriangleMatchingEdge/*=NULL*/, const dVector3 *pFirstTriangle/*=NULL*/,
        //        const TMeshDataAccessor &dataAccessor);
        //        template<class TMeshDataAccessor>
        //        static dReal calculateEdgeAngleValidated(unsigned firstVertexStartIndex,
        //            EdgeRecord *currEdge, const dReal &normalSegmentDot, const dReal &lengthSquareProduct,
        //        const dVector3 &triangleNormal, const dVector3 &secondOppositeVertexSegment,
        //        const dVector3 *pSecondTriangleMatchingEdge/*=NULL*/, const dVector3 *pFirstTriangle/*=NULL*/,
        //        const TMeshDataAccessor &dataAccessor);

        //        private:
        //    const void *m_vertices;
        private float[] m_vertices;
        // private int m_vertexStride;
        private int m_vertexCount;
        //const void *m_indices;
        private int[] m_indices;
        private int m_triangleCount;
        // private int m_triStride;
        // private boolean m_single;

        //        private:
        //        const void *m_normals;
        //        IFaceAngleStorageControl *m_faceAngles;
        //        IFaceAngleStorageView *m_faceAngleView;
        private float[] m_normals;
        private IFaceAngleStorageControl m_faceAngles;
        private IFaceAngleStorageView m_faceAngleView;
    //}


    //IFaceAngleStorageView[] dxGeomTriMeshGetFaceAngleView(DxGeom triMeshGeom);

    //  #endif	//_ODE_COLLISION_TRIMESH_INTERNAL_H_


    // **************************************************
    //  collision_trimesh_internal_impl.h
    // **************************************************

    //#ifndef _ODE_COLLISION_TRIMESH_INTERNAL_IMPL_H_
    //#define _ODE_COLLISION_TRIMESH_INTERNAL_IMPL_H_

    //        #if dTRIMESH_ENABLED


    //    template<typename tcoordfloat, typename tindexint>
    //    /*static */
    //    void dxTriDataBase::retrieveTriangleVertexPoints(dVector3 out_Points[dMTV__MAX], unsigned triangleIndex,
    //    const tcoordfloat *vertexInstances, int vertexStride, const tindexint *triangleVertexIndices, int triangleStride)
    //<tcoordfloat, tindexint>
    /*static */
    //    static void retrieveTriangleVertexPoints(DVector3[] out_Points, int triangleIndex, final float[] vertexInstances, int vertexStride, final int[] triangleVertexIndices, int triangleStride) {
    //        //final tindexint[] triangleIndicesOfInterest = (const tindexint *)((uint8 *)triangleVertexIndices + (size_t)triangleIndex * triangleStride);
    //        final int triangleIndicesOfInterestPos = triangleIndex * triangleStride;
    //        for (int trianglePoint = dMTV__MIN; trianglePoint != dMTV__MAX; ++trianglePoint) {
    //            int vertexIndex = triangleVertexIndices[triangleIndicesOfInterestPos + trianglePoint];
    //            //tcoordfloat * pointVertex = (tcoordfloat *) ((uint8 *) vertexInstances + (size_t) vertexIndex * vertexStride)
    //            int pointVertexPos = /*vertexInstances +*/ vertexIndex * vertexStride;
    //
    //            //dAssignVector3(out_Points[trianglePoint], (double) pointVertex[dSA_X], (double) pointVertex[dSA_Y], (double) pointVertex[dSA_Z]);
    //            out_Points[trianglePoint].set(
    //                    vertexInstances[pointVertexPos + dSA_X],
    //                    vertexInstances[pointVertexPos + dSA_Y],
    //                    vertexInstances[pointVertexPos + dSA_Z]);
    //            dSASSERT(dSA_X == 0);
    //            dSASSERT(dSA_Y == 1);
    //            dSASSERT(dSA_Z == 2);
    //        }
    //    }
    static void retrieveTriangleVertexPoints(DVector3[] out_Points, int triangleIndex, final float[] vertexInstances, final int[] triangleVertexIndices) {
        //final tindexint[] triangleIndicesOfInterest = (const tindexint *)((uint8 *)triangleVertexIndices + (size_t)triangleIndex * triangleStride);
        final int triangleIndicesOfInterestPos = triangleIndex * DxTriMesh.TRIANGLEINDEX_STRIDE;
        for (int trianglePoint = dMTV__MIN; trianglePoint != dMTV__MAX; ++trianglePoint) {
            int vertexIndex = triangleVertexIndices[triangleIndicesOfInterestPos + trianglePoint];
            //tcoordfloat * pointVertex = (tcoordfloat *) ((uint8 *) vertexInstances + (size_t) vertexIndex * vertexStride)
            int pointVertexPos = /*vertexInstances +*/ vertexIndex * DxTriMesh.VERTEXINSTANCE_STRIDE;

            //dAssignVector3(out_Points[trianglePoint], (double) pointVertex[dSA_X], (double) pointVertex[dSA_Y], (double) pointVertex[dSA_Z]);
            out_Points[trianglePoint].set(
                    vertexInstances[pointVertexPos + dSA_X],
                    vertexInstances[pointVertexPos + dSA_Y],
                    vertexInstances[pointVertexPos + dSA_Z]);
            dSASSERT(dSA_X == 0);
            dSASSERT(dSA_Y == 1);
            dSASSERT(dSA_Z == 2);
        }
    }


    //    template<class TMeshDataAccessor>
    //    /*static */
    //    void dxTriDataBase::meaningfulPreprocess_SetupEdgeRecords(EdgeRecord *edges, size_t numEdges, const TMeshDataAccessor &dataAccessor)
    /*static */
    void meaningfulPreprocess_SetupEdgeRecords(EdgeRecord[] edges, int numEdges, final TMeshDataAccessorI dataAccessor) {
        int[] vertexIndices = new int[dMTV__MAX];
        // Make a list of every edge in the mesh
        int triangleIdx = 0;
        for (int edgeIdx = 0; edgeIdx != numEdges; ++triangleIdx, edgeIdx += dMTV__MAX) {
            dataAccessor.getTriangleVertexIndices(vertexIndices, triangleIdx);
            edges[edgeIdx + dMTV_FIRST].setupEdge(dMTV_FIRST, triangleIdx, vertexIndices);
            edges[edgeIdx + dMTV_SECOND].setupEdge(dMTV_SECOND, triangleIdx, vertexIndices);
            edges[edgeIdx + dMTV_THIRD].setupEdge(dMTV_THIRD, triangleIdx, vertexIndices);
        }
    }

    //    template<class TMeshDataAccessor>
    //    /*static */
    //    void dxTriDataBase::meaningfulPreprocess_buildEdgeFlags(uint8 *useFlags/*=NULL*/, IFaceAngleStorageControl *faceAngles/*=NULL*/,
    //                                                            EdgeRecord *edges, size_t numEdges, VertexRecord *vertices,
    //    const dReal *externalNormals/*=NULL*/, const TMeshDataAccessor &dataAccessor)
    /*static */
    void meaningfulPreprocess_buildEdgeFlags(byte[] useFlags/*=NULL*/, IFaceAngleStorageControl faceAngles/*=NULL*/
            , EdgeRecord[] edges, int numEdges, VertexRecord[] vertices, final float[] externalNormals/*=NULL*/,
                                             final TMeshDataAccessorP dataAccessor) {
        dIASSERT(useFlags != null || faceAngles != null);
        dIASSERT(numEdges != 0);

        final boolean negativeAnglesStored = faceAngles != null && faceAngles.areNegativeAnglesStored();

        // Go through the sorted list of edges and flag all the edges and vertices that we need to use
        //EdgeRecord *const lastEdge = edges + (numEdges - 1);
        final int lastEdgeOfs = (numEdges - 1);
        //for (EdgeRecord * currEdge = edges; ; ++currEdge) {
        for (int currEdgeOfs = 0; ; ++currEdgeOfs) {
//            final EdgeRecord currEdge0 = edges[currEdgeOfs];
//            final EdgeRecord currEdge1 = edges[currEdgeOfs + 1];
            // Handle the last edge separately to have an optimizer friendly loop
            if (currEdgeOfs >= lastEdgeOfs) {
                // This is a boundary edge
                if (currEdgeOfs == lastEdgeOfs) {
                    if (faceAngles != null) {
                        buildBoundaryEdgeAngle(faceAngles, edges[currEdgeOfs]);
                    }

                    if (useFlags != null) {
                        // For the last element EdgeRecord::kAbsVertexUsed assignment can be skipped as noone is going to need it any more
                        final EdgeRecord currEdge0 = edges[currEdgeOfs];
                        useFlags[currEdge0.m_triIdx] |= ((edges[currEdge0.m_vertIdx1].m_absVertexFlags & EdgeRecord.AVF_VERTEX_USED) == 0 ? currEdge0.m_vert1Flags : 0) | ((edges[currEdge0.m_vertIdx2].m_absVertexFlags & EdgeRecord.AVF_VERTEX_USED) == 0 ? currEdge0.m_vert2Flags : 0) | currEdge0.m_edgeFlags;
                    }
                }

                break;
            }

            final EdgeRecord currEdge0 = edges[currEdgeOfs];
            final EdgeRecord currEdge1 = edges[currEdgeOfs + 1];
            int vertIdx1 = currEdge0.m_vertIdx1;
            int vertIdx2 = currEdge0.m_vertIdx2;

            if (vertIdx2 == currEdge1.m_vertIdx2 // Check second vertex first as it is more likely to change taking the sorting rules into account
                    && vertIdx1 == currEdge1.m_vertIdx1) {
                // We let the dot threshold for concavity get slightly negative to allow for rounding errors
                final float kConcaveThreshold = 0.000001f;

                //const dVector3 *pSecondTriangleEdgeToUse = NULL, *pFirstTriangleToUse = NULL;
                DVector3C pSecondTriangleEdgeToUse = null;
                int pFirstTriangleToUse = -1;
                DVector3 secondTriangleMatchingEdge = new DVector3();
                DVector3[] firstTriangle = DVector3.newArray( dMTV__MAX);
                DVector3 secondOppositeVertexSegment = new DVector3(), triangleNormal = new DVector3();
                double lengthSquareProduct, secondOppositeSegmentLengthSquare;

                // Calculate orthogonal vector from the matching edge of the second triangle to its opposite point
                {
                    DVector3[] secondTriangle = DVector3.newArray(dMTV__MAX);
                    dataAccessor.getTriangleVertexPoints(secondTriangle, currEdge1.m_triIdx);

                    // Get the vertex opposite this edge in the second triangle
                    //DMeshTriangleVertex * 3
                    int secondOppositeVertex = currEdge1.getOppositeVertexIndex();
                    int secondEdgeStart = secondOppositeVertex + 1 != dMTV__MAX ? (secondOppositeVertex + 1) : dMTV__MIN;
                    int secondEdgeEnd = (dMTV_FIRST + dMTV_SECOND + dMTV_THIRD - secondEdgeStart - secondOppositeVertex);

                    dSubtractVectors3(secondTriangleMatchingEdge, secondTriangle[secondEdgeEnd], secondTriangle[secondEdgeStart]);

                    if (dSafeNormalize3(secondTriangleMatchingEdge)) {
                        pSecondTriangleEdgeToUse = /*&*/secondTriangleMatchingEdge;

                        DVector3 secondTriangleOppositeEdge = new DVector3();
                        dSubtractVectors3(secondTriangleOppositeEdge, secondTriangle[secondOppositeVertex], secondTriangle[secondEdgeStart]);
                        double dProjectionLength = dCalcVectorDot3(secondTriangleOppositeEdge, secondTriangleMatchingEdge);
                        dAddVectorScaledVector3(secondOppositeVertexSegment, secondTriangleOppositeEdge, secondTriangleMatchingEdge, -dProjectionLength);
                    } else {
                        dSubtractVectors3(secondOppositeVertexSegment, secondTriangle[secondOppositeVertex], secondTriangle[secondEdgeStart]);
                    }

                    secondOppositeSegmentLengthSquare = dCalcVectorLengthSquare3(secondOppositeVertexSegment);
                }

                // Either calculate the normal from triangle vertices...
                if (externalNormals == null) {
                    // Get the normal of the first triangle
                    dataAccessor.getTriangleVertexPoints(firstTriangle, currEdge0.m_triIdx);
                    pFirstTriangleToUse = dMTV__MIN;//firstTriangle[dMTV__MIN];

                    DVector3 firstEdge = new DVector3(), secondEdge = new DVector3();
                    dSubtractVectors3(secondEdge, firstTriangle[dMTV_THIRD], firstTriangle[dMTV_SECOND]);
                    dSubtractVectors3(firstEdge, firstTriangle[dMTV_FIRST], firstTriangle[dMTV_SECOND]);
                    dCalcVectorCross3(triangleNormal, secondEdge, firstEdge);
                    double normalLengthSuqare = dCalcVectorLengthSquare3(triangleNormal);
                    lengthSquareProduct = secondOppositeSegmentLengthSquare * normalLengthSuqare;
                }
                // ...or use the externally supplied normals
                else {
                    //const dReal * pTriangleExternalNormal = externalNormals + currEdge0.m_triIdx * dSA__MAX;
                    final int pTriangleExternalNormalPos = currEdge0.m_triIdx * dSA__MAX;
                    //dAssignVector3(triangleNormal, pTriangleExternalNormal[dSA_X], pTriangleExternalNormal[dSA_Y], pTriangleExternalNormal[dSA_Z]);
                    triangleNormal.set(
                            externalNormals[pTriangleExternalNormalPos + dSA_X],
                            externalNormals[pTriangleExternalNormalPos + dSA_Y],
                            externalNormals[pTriangleExternalNormalPos + dSA_Z]);
                    // normalLengthSuqare = REAL(1.0);
                    dUASSERT(dFabs(dCalcVectorLengthSquare3(triangleNormal) - (1.0)) < (0.25) * kConcaveThreshold * kConcaveThreshold, "Mesh triangle normals must be normalized");

                    lengthSquareProduct = secondOppositeSegmentLengthSquare/* * normalLengthSuqare*/;
                }

                double normalSegmentDot = dCalcVectorDot3(triangleNormal, secondOppositeVertexSegment);

                // This is a concave edge, leave it for the next pass
                // OD: This is the "dot >= kConcaveThresh" check, but since the vectros were not normalized to save on roots and divisions,
                // the check against zero is performed first and then the dot product is squared and compared against the threshold multiplied by lengths' squares
                // OD: Originally, there was dot > -kConcaveThresh check, but this does not seem to be a good idea
                // as it can mark all edges on potentially large (nearly) flat surfaces concave.
                if (normalSegmentDot > 0.0 && normalSegmentDot * normalSegmentDot >= kConcaveThreshold * kConcaveThreshold * lengthSquareProduct) {
                    if (faceAngles != null) {
                        buildConcaveEdgeAngle(faceAngles, negativeAnglesStored, currEdge0, currEdge1, normalSegmentDot,
                                lengthSquareProduct, triangleNormal, secondOppositeVertexSegment,
                                pSecondTriangleEdgeToUse, firstTriangle, pFirstTriangleToUse, dataAccessor);
                    }

                    if (useFlags != null) {
                        // Mark the vertices of a concave edge to prevent their use
                        int absVertexFlags1 = edges[vertIdx1].m_absVertexFlags;
                        edges[vertIdx1].m_absVertexFlags |= absVertexFlags1 | EdgeRecord.AVF_VERTEX_HAS_CONCAVE_EDGE |
                                EdgeRecord.AVF_VERTEX_USED;

                        if ((absVertexFlags1 & (EdgeRecord.AVF_VERTEX_HAS_CONCAVE_EDGE | EdgeRecord.AVF_VERTEX_USED)) ==
                                EdgeRecord.AVF_VERTEX_USED) {
                            // If the vertex was already used from other triangles but then discovered
                            // to have a concave edge, unmark the previous use
                            int usedFromEdgeIndex = vertices[vertIdx1].m_UsedFromEdgeIndex;
                            //const EdgeRecord * usedFromEdge = edges + usedFromEdgeIndex;
                            final EdgeRecord usedFromEdge = edges[usedFromEdgeIndex];
                            int usedInTriangleIndex = usedFromEdge.m_triIdx;
                            byte usedVertFlags = usedFromEdge.m_vertIdx1 == vertIdx1 ?
                                    usedFromEdge.m_vert1Flags : usedFromEdge.m_vert2Flags;
                            useFlags[usedInTriangleIndex] ^= usedVertFlags;
                            dIASSERT((useFlags[usedInTriangleIndex] & usedVertFlags) == 0);
                        }

                        int absVertexFlags2 = edges[vertIdx2].m_absVertexFlags;
                        edges[vertIdx2].m_absVertexFlags = (byte) (absVertexFlags2 | EdgeRecord.AVF_VERTEX_HAS_CONCAVE_EDGE |
                                                        EdgeRecord.AVF_VERTEX_USED);

                        if ((absVertexFlags2 & (EdgeRecord.AVF_VERTEX_HAS_CONCAVE_EDGE | EdgeRecord.AVF_VERTEX_USED)) ==
                                EdgeRecord.AVF_VERTEX_USED) {
                            // Similarly unmark the possible previous use of the edge's second vertex
                            int usedFromEdgeIndex = vertices[vertIdx2].m_UsedFromEdgeIndex;
                            //const EdgeRecord * usedFromEdge = edges + usedFromEdgeIndex;
                            final EdgeRecord usedFromEdge = edges[usedFromEdgeIndex];
                            int usedInTriangleIndex = usedFromEdge.m_triIdx;
                            byte usedVertFlags = usedFromEdge.m_vertIdx1 == vertIdx2 ?
                                    usedFromEdge.m_vert1Flags : usedFromEdge.m_vert2Flags;
                            useFlags[usedInTriangleIndex] ^= usedVertFlags;
                            dIASSERT((useFlags[usedInTriangleIndex] & usedVertFlags) == 0);
                        }
                    }
                }
                // If this is a convex edge, mark its vertices and edge as used
                else {
                    if (faceAngles != null) {
                        buildConvexEdgeAngle(faceAngles, currEdge0, currEdge1, normalSegmentDot, lengthSquareProduct,
                                triangleNormal, secondOppositeVertexSegment, pSecondTriangleEdgeToUse,
                                firstTriangle, pFirstTriangleToUse, dataAccessor);
                    }

                    if (useFlags != null) {
                        //EdgeRecord * edgeToUse = currEdge;
                        int edgeToUse = currEdgeOfs;
                        int triIdx = edges[edgeToUse].m_triIdx;
                        int triIdx1 = edges[edgeToUse + 1].m_triIdx;

                        int triUseFlags = useFlags[triIdx];
                        int triUseFlags1 = useFlags[triIdx1];

                        // Choose to add flags to the bitmask that already has more edges
                        // (to group flags in selected triangles rather than scattering them evenly)
                        if ((triUseFlags1 & CUF__USE_ALL_EDGES) > (triUseFlags & CUF__USE_ALL_EDGES)) {
                            triIdx = triIdx1;
                            triUseFlags = triUseFlags1;
                            edgeToUse = edgeToUse + 1;
                        }

                        if ((edges[vertIdx1].m_absVertexFlags & EdgeRecord.AVF_VERTEX_USED) == 0) {
                            // Only add each vertex once and set a mark to prevent further additions
                            edges[vertIdx1].m_absVertexFlags |= EdgeRecord.AVF_VERTEX_USED;
                            // Also remember the index the vertex flags are going to be applied to
                            // to allow easily clear the vertex from the use flags if any concave edges are found to connect to it
                            vertices[vertIdx1].m_UsedFromEdgeIndex = (edgeToUse);// - edges);
                            triUseFlags |= edges[edgeToUse].m_vert1Flags;
                        }

                        // Same processing for the second vertex...
                        if ((edges[vertIdx2].m_absVertexFlags & EdgeRecord.AVF_VERTEX_USED) == 0) {
                            edges[vertIdx2].m_absVertexFlags |= EdgeRecord.AVF_VERTEX_USED;
                            vertices[vertIdx2].m_UsedFromEdgeIndex = (edgeToUse);// - edges);
                            triUseFlags |= edges[edgeToUse].m_vert2Flags;
                        }

                        // And finally store the use flags adding the edge flags in
                        useFlags[triIdx] = (byte) (triUseFlags | edges[edgeToUse].m_edgeFlags);
                    }
                }

                // Skip the second edge
                ++currEdgeOfs;
            }
            // This is a boundary edge
            else {
                if (faceAngles != null) {
                    buildBoundaryEdgeAngle(faceAngles, currEdge0);
                }

                if (useFlags != null) {
                    int triIdx = currEdge0.m_triIdx;
                    int triUseExtraFlags = 0;

                    if ((edges[vertIdx1].m_absVertexFlags & EdgeRecord.AVF_VERTEX_USED) == 0) {
                        edges[vertIdx1].m_absVertexFlags |= EdgeRecord.AVF_VERTEX_USED;
                        vertices[vertIdx1].m_UsedFromEdgeIndex = currEdgeOfs; //(int) (currEdge0 - edges);
                        triUseExtraFlags |= currEdge0.m_vert1Flags;
                    }

                    if ((edges[vertIdx2].m_absVertexFlags & EdgeRecord.AVF_VERTEX_USED) == 0) {
                        edges[vertIdx2].m_absVertexFlags |= EdgeRecord.AVF_VERTEX_USED;
                        vertices[vertIdx2].m_UsedFromEdgeIndex = currEdgeOfs; //(int) (currEdge - edges);
                        triUseExtraFlags |= currEdge0.m_vert2Flags;
                    }

                    useFlags[triIdx] |= triUseExtraFlags | currEdge0.m_edgeFlags;
                }
            }
        }
    }

    /*static */
    //    void dxTriDataBase::buildBoundaryEdgeAngle(IFaceAngleStorageControl *faceAngles,
    //                                               EdgeRecord *currEdge)
    void buildBoundaryEdgeAngle(IFaceAngleStorageControl faceAngles, EdgeRecord currEdge0) {
        final double faceAngle = (0.0);

        // DMeshTriangleVertex
        int firstVertexStartIndex = currEdge0.getEdgeStartVertexIndex();
        faceAngles.assignFacesAngleIntoStorage(currEdge0.m_triIdx, firstVertexStartIndex, faceAngle);
        // -- For boundary edges, only the first element is valid
        // dMeshTriangleVertex secondVertexStartIndex = currEdge[1].getEdgeStartVertexIndex();
        // faceAngles->assignFacesAngleIntoStorage(currEdge[1].m_TriIdx, secondVertexStartIndex, faceAngle);
    }

    //    template<class TMeshDataAccessor>
    //    /*static */
    //    void dxTriDataBase::buildConcaveEdgeAngle(IFaceAngleStorageControl *faceAngles, bool negativeAnglesStored,
    //                                              EdgeRecord *currEdge, const dReal &normalSegmentDot, const dReal &lengthSquareProduct,
    //    const dVector3 &triangleNormal, const dVector3 &secondOppositeVertexSegment,
    //    const dVector3 *pSecondTriangleMatchingEdge/*=NULL*/, const dVector3 *pFirstTriangle/*=NULL*/,
    //    const TMeshDataAccessor &dataAccessor)
    /*static */
    void buildConcaveEdgeAngle(IFaceAngleStorageControl faceAngles,
                               boolean negativeAnglesStored,
                               EdgeRecord currEdge0,
                               EdgeRecord currEdge1,
                               final double normalSegmentDot,
                               final double lengthSquareProduct,
                               DVector3C triangleNormal,
                               DVector3C secondOppositeVertexSegment,
                               DVector3C pSecondTriangleMatchingEdge/*=NULL*/,
                               DVector3C[] firstTriangle,
                               int pFirstTriangle/*=NULL*/,
                               final TMeshDataAccessorP dataAccessor) {
        double faceAngle;
        //DMeshTriangleVertex
        int firstVertexStartIndex = currEdge0.getEdgeStartVertexIndex();

        // Check if concave angles are stored at all
        if (negativeAnglesStored) {
            // The length square product can become zero due to precision loss
            // when both the normal and the opposite edge vectors are very small.
            if (lengthSquareProduct != (0.0)) {
                faceAngle = -calculateEdgeAngleValidated(firstVertexStartIndex, currEdge0, normalSegmentDot,
                        lengthSquareProduct, triangleNormal, secondOppositeVertexSegment, pSecondTriangleMatchingEdge,
                        firstTriangle, pFirstTriangle, dataAccessor);
            } else {
                faceAngle = (0.0);
            }
        } else {
            // If concave angles ate not stored, set an arbitrary negative value
            faceAngle = -M_PI;
        }

        faceAngles.assignFacesAngleIntoStorage(currEdge0.m_triIdx, firstVertexStartIndex, faceAngle);
        //dMeshTriangleVertex
        int secondVertexStartIndex = currEdge1.getEdgeStartVertexIndex();
        faceAngles.assignFacesAngleIntoStorage(currEdge1.m_triIdx, secondVertexStartIndex, faceAngle);
    }

    //    template<class TMeshDataAccessor>
    //    /*static */
    //    void dxTriDataBase::buildConvexEdgeAngle(IFaceAngleStorageControl *faceAngles,
    //                                             EdgeRecord *currEdge, const dReal &normalSegmentDot, const dReal &lengthSquareProduct,
    //    const dVector3 &triangleNormal, const dVector3 &secondOppositeVertexSegment,
    //    const dVector3 *pSecondTriangleMatchingEdge/*=NULL*/, const dVector3 *pFirstTriangle/*=NULL*/,
    //    const TMeshDataAccessor &dataAccessor)
    /*static */
    void buildConvexEdgeAngle(IFaceAngleStorageControl faceAngles, EdgeRecord currEdge0, EdgeRecord currEdge1,
                              final double normalSegmentDot,
                              final double lengthSquareProduct, DVector3C triangleNormal,
                              DVector3 secondOppositeVertexSegment, DVector3C pSecondTriangleMatchingEdge/*=NULL*/,
                              final DVector3C[] firstTriangle, final int pFirstTriangle
            /*=NULL*/, final TMeshDataAccessorP dataAccessor) {
        double faceAngle;
        //dMeshTriangleVertex
        int firstVertexStartIndex = currEdge0.getEdgeStartVertexIndex();

        // The length square product can become zero due to precision loss
        // when both the normal and the opposite edge vectors are very small.
        if (normalSegmentDot < (0.0) && lengthSquareProduct != (0.0)) {
            faceAngle = calculateEdgeAngleValidated(firstVertexStartIndex, currEdge0, -normalSegmentDot,
                    lengthSquareProduct, triangleNormal, secondOppositeVertexSegment, pSecondTriangleMatchingEdge,
                    firstTriangle, pFirstTriangle, dataAccessor);
        } else {
            faceAngle = (0.0);
        }

        faceAngles.assignFacesAngleIntoStorage(currEdge0.m_triIdx, firstVertexStartIndex, faceAngle);
        //dMeshTriangleVertex
        int secondVertexStartIndex = currEdge1.getEdgeStartVertexIndex();
        faceAngles.assignFacesAngleIntoStorage(currEdge1.m_triIdx, secondVertexStartIndex, faceAngle);
    }

    //    template<class TMeshDataAccessor>
    //    /*static */
    //    dReal dxTriDataBase::calculateEdgeAngleValidated(unsigned firstVertexStartIndex,
    //                                                     EdgeRecord *currEdge, const dReal &normalSegmentDot, const dReal &lengthSquareProduct,
    //    const dVector3 &triangleNormal, const dVector3 &secondOppositeVertexSegment,
    //    const dVector3 *pSecondTriangleMatchingEdge/*=NULL*/, const dVector3 *pFirstTriangle/*=NULL*/,
    //    const TMeshDataAccessor &dataAccessor)
    /*static */
    double calculateEdgeAngleValidated(int firstVertexStartIndex, EdgeRecord currEdge0,
                                       final double normalSegmentDot, final double lengthSquareProduct,
                                       DVector3C triangleNormal, DVector3C secondOppositeVertexSegment,
                                       DVector3C pSecondTriangleMatchingEdge/*=NULL*/, DVector3C[] firstTriangle,
                                       int pFirstTriangle
            /*=NULL*/, final TMeshDataAccessorP dataAccessor) {
        dIASSERT(lengthSquareProduct >= (0.0));

        double result;
        double angleCosine = normalSegmentDot / dSqrt(lengthSquareProduct);

        if (angleCosine < (1.0)) {
            DVector3 normalSecondOppositeSegmentCross = new DVector3();
            dCalcVectorCross3(normalSecondOppositeSegmentCross, triangleNormal, secondOppositeVertexSegment);

            double secondTriangleEdgeDirectionCheck;

            if (pSecondTriangleMatchingEdge != null) {
                // Check the cross product against the second triangle edge, if possible...
                secondTriangleEdgeDirectionCheck = dCalcVectorDot3(normalSecondOppositeSegmentCross, pSecondTriangleMatchingEdge);
            } else {
                // ...if not, calculate the supposed direction of the second triangle's edge
                // as negative of first triangle edge. For that cross-multiply the precomputed
                // first triangle normal by vector from the degenerate edge to its opposite vertex.
                // Retrieve the first triangle points if necessary

                // TZ:
                DVector3C[] firstTriangleStorage = firstTriangle;
                //const dVector3 *pFirstTriangleToUse = pFirstTriangle;
                int pFirstTriangleToUse = pFirstTriangle;

                //if (pFirstTriangle == null)
                if (firstTriangle == null)
                {
                    //firstTriangleStorage = new DVector3[dMTV__MAX];
                    DVector3[] temp = new DVector3[dMTV__MAX];
                    dataAccessor.getTriangleVertexPoints(temp, currEdge0.m_triIdx);
                    firstTriangleStorage = temp;
                    pFirstTriangleToUse = dMTV__MIN;//&firstTriangleStorage[dMTV__MIN];
                }

                // Calculate the opposite vector
                int firstTriangleOppositeIndex = firstVertexStartIndex != dMTV__MIN ? firstVertexStartIndex - 1 : dMTV__MAX - 1;

                DVector3 firstOppositeVertexSegment = new DVector3();
                //dSubtractVectors3(firstOppositeVertexSegment,
                //        pFirstTriangleToUse[firstTriangleOppositeIndex],
                //        pFirstTriangleToUse[firstVertexStartIndex]);
                dSubtractVectors3(firstOppositeVertexSegment,
                        firstTriangleStorage[pFirstTriangleToUse + firstTriangleOppositeIndex],
                        firstTriangleStorage[pFirstTriangleToUse + firstVertexStartIndex]);

                DVector3 normalFirstOppositeSegmentCross = new DVector3();
                dCalcVectorCross3(normalFirstOppositeSegmentCross, triangleNormal, firstOppositeVertexSegment);

                // And finally calculate the dot product to compare vector directions
                secondTriangleEdgeDirectionCheck = dCalcVectorDot3(normalSecondOppositeSegmentCross, normalFirstOppositeSegmentCross);
            }

            // Negative product means the angle absolute value is less than M_PI_2, positive - greater.
            result = secondTriangleEdgeDirectionCheck < (0.0) ? dAsin(angleCosine) : M_PI_2 + dAcos(angleCosine);
        } else {
            result = M_PI_2;
            dIASSERT(angleCosine - (1.0) < 1e-4); // The computational error can not be too high because the dot product had been verified to be greater than the concave threshold above
        }

        return result;
    }

    // #endif // #if dTRIMESH_ENABLED


    // #endif // #ifndef _ODE_COLLISION_TRIMESH_INTERNAL_IMPL_H_


    // **************************************************
    //  collision_trimesh_internal.cpp
    // **************************************************

    // TriMesh storage classes refactoring and face angle computation code by Oleh Derevenko (C) 2016-2017

    //////////////////////////////////////////////////////////////////////////

    //enum EdgeStorageSignInclusion {
    //    public static final int SSI__MIN = 0;
    //    public static final int SSI_SIGNED_STORED = SSI__MIN;
    //    public static final int SSI_POSITIVE_STORED = SSI_SIGNED_STORED + 1;
    //    public static final int SSI__MAX = SSI_POSITIVE_STORED + 1;





    //typedef IFaceAngleStorageControl *(FAngleStorageAllocProc)(unsigned triangleCount, IFaceAngleStorageView *&out_storageView);
    //    interface FAngleStorageAllocProc {
    //        IFaceAngleStorageControl allocateInstance(int triangleCount, Ref<IFaceAngleStorageView> out_storageView);
    //    }

    //    BEGIN_NAMESPACE_OU();
    //    template<>
    //    FAngleStorageAllocProc *const CEnumUnsortedElementArray<FaceAngleStorageMethod, ASM__MAX, FAngleStorageAllocProc *, 0x161211AD>::m_aetElementArray[] =
    //    {
    //    &FaceAnglesWrapper<org.ode4j.ode.internal.trimesh.FaceAngleStorageCodec<uint8, SSI_SIGNED_STORED>>::allocateInstance, // ASM_BYTE_SIGNED,
    //    &FaceAnglesWrapper<org.ode4j.ode.internal.trimesh.FaceAngleStorageCodec<uint8, SSI_POSITIVE_STORED>>::allocateInstance, // ASM_BYTE_POSITIVE,
    //    &FaceAnglesWrapper<org.ode4j.ode.internal.trimesh.FaceAngleStorageCodec<uint16, SSI_SIGNED_STORED>>::allocateInstance, // ASM_WORD_SIGNED,
    //    };
    //    END_NAMESPACE_OU();
    //    static const CEnumUnsortedElementArray<FaceAngleStorageMethod, ASM__MAX, FAngleStorageAllocProc *, 0x161211AD> g_AngleStorageAllocProcs;
    // TX we have only one FaceAngleStorageCodec in Java, so we can return it directly.
    //    private static class CEnumUnsortedElementArrayASAP {
    //        public FaceAnglesWrapper Encode(int i) {
    //            if (i != ASM_WORD_SIGNED) {
    //                throw new UnsupportedOperationException("We only support ASM_WORD_SIGNED");
    //            }
    //            return FaceAngleStorageCodec.allocateInstance();
    //        }
    //    }
    //    static final CEnumUnsortedElementArrayASAP g_AngleStorageAllocProcs = new CEnumUnsortedElementArrayASAP();



    //////////////////////////////////////////////////////////////////////////

    // dxTriDataBase::~dxTriDataBase()
    protected void DESTRUCTOR()
    {
        super.DESTRUCTOR();
        freeFaceAngles();
    }


    //    void dxTriDataBase::buildData(const void *vertices, int vertexStride, unsigned vertexCount,
    //    const void *indices, unsigned indexCount, int triStride,
    //    const void *normals,
    //                                  bool single)
    //    private void buildData(final float[] vertices, int vertexStride, int vertexCount, final int[] indices,
    //      int indexCount, int triStride, final float[] normals) //, boolean single) {
    private void buildData(final float[] vertices, int vertexCount, final int[] indices, int indexCount,
                           final float[] normals) {
        dIASSERT(vertices != null);
        dIASSERT(indices != null);
        //dIASSERT(vertexStride != 0); // == 3!
        //dIASSERT(triStride != 0); // == 3!
        dIASSERT(indexCount != 0);
        dIASSERT(indexCount % dMTV__MAX == 0);

        m_vertices = vertices;
        // m_vertexStride = vertexStride;
        m_vertexCount = vertexCount;
        m_indices = indices;
        m_triangleCount = indexCount / dMTV__MAX;
        // m_triStride = triStride;
        // m_single = single;

        m_normals = normals;
    }

    protected void buildData(final float[] vertices, final int[] indices, final float[] normals) {
        buildData(vertices, vertices.length/3, indices, indices.length, normals); //, true);
    }


    //bool dxTriDataBase::allocateFaceAngles(FaceAngleStorageMethod storageMethod)
    boolean allocateFaceAngles(int storageMethod)
    {
        boolean result = false;

        dIASSERT(m_faceAngles == null);

        Ref<IFaceAngleStorageView> storageView = new Ref<>();

        int triangleCount = m_triangleCount;

        //FAngleStorageAllocProc allocProc = g_AngleStorageAllocProcs.Encode(storageMethod);
        //IFaceAngleStorageControl storageInstance = allocProc.allocateInstance(triangleCount, storageView);
        //FaceAnglesWrapper allocProc = new FaceAnglesWrapper(triangleCount);
        //FaceAngleStorageCodec proc = new FaceAngleStorageCodec();
        //IFaceAngleStorageControl storageInstance = allocProc.allocateInstance(triangleCount, storageView);
        FaceAnglesWrapper storage = FaceAnglesWrapper.allocateInstance(triangleCount);

        //if (storageInstance != null)
        //{
            m_faceAngles = storage;//storageInstance;
            m_faceAngleView = storage;//storageView.get();
            result = true;
        //}

        return result;
    }

    //void dxTriDataBase::freeFaceAngles()
    void freeFaceAngles()
    {
        if (m_faceAngles != null)
        {
            m_faceAngles.disposeStorage();
            m_faceAngles = null;
            m_faceAngleView = null;
        }
    }


    //    BEGIN_NAMESPACE_OU();
    //    template<>
    //    const dMeshTriangleVertex CEnumUnsortedElementArray<unsigned, dxTriDataBase::CUF__USE_VERTICES_LAST / dxTriDataBase::CUF__USE_VERTICES_MIN, dMeshTriangleVertex, 0x161116DC>::m_aetElementArray[] =
    //    {
    //        dMTV_FIRST, // kVert0 / kVert_Base
    //                dMTV_SECOND, // kVert1 / kVert_Base
    //                dMTV__MAX,
    //                dMTV_THIRD, // kVert2 / kVert_Base
    //    };
    //    END_NAMESPACE_OU();
    //    /*extern */const CEnumUnsortedElementArray<unsigned, dxTriDataBase::CUF__USE_VERTICES_LAST / dxTriDataBase::CUF__USE_VERTICES_MIN, dMeshTriangleVertex, 0x161116DC> g_VertFlagOppositeIndices;
    static final CEnumUnsortedElementArray g_VertFlagOppositeIndices = new CEnumUnsortedElementArray(new int[]{dMTV_FIRST,
            // kVert0 / kVert_Base
            dMTV_SECOND, // kVert1 / kVert_Base
            dMTV__MAX, dMTV_THIRD, // kVert2 / kVert_Base
    }, CUF__USE_VERTICES_LAST / CUF__USE_VERTICES_MIN);

    //    BEGIN_NAMESPACE_OU();
    //    template<>
    //    const dMeshTriangleVertex CEnumUnsortedElementArray<unsigned, dxTriDataBase::CUF__USE_VERTICES_LAST / dxTriDataBase::CUF__USE_VERTICES_MIN, dMeshTriangleVertex, 0x161225E9>::m_aetElementArray[] =
    //    {
    //        dMTV_SECOND, // kVert0 / kVert_Base
    //                dMTV_THIRD, // kVert1 / kVert_Base
    //                dMTV__MAX,
    //                dMTV_FIRST, // kVert2 / kVert_Base
    //    };
    //    END_NAMESPACE_OU();
    //    /*extern */const CEnumUnsortedElementArray<unsigned, dxTriDataBase::CUF__USE_VERTICES_LAST / dxTriDataBase::CUF__USE_VERTICES_MIN, dMeshTriangleVertex, 0x161225E9> g_VertFlagEdgeStartIndices;
    static final CEnumUnsortedElementArray g_VertFlagEdgeStartIndices =
            new CEnumUnsortedElementArray(new int[]{dMTV_SECOND, // kVert0 / kVert_Base
            dMTV_THIRD, // kVert1 / kVert_Base
            dMTV__MAX, dMTV_FIRST, // kVert2 / kVert_Base
    }, CUF__USE_VERTICES_LAST / CUF__USE_VERTICES_MIN);


    //////////////////////////////////////////////////////////////////////////

    // TZ(2021-04-11) These methods don't appear to have ever been available.
    //    /*extern ODE_API */
    ////    void dGeomTriMeshDataBuildSimple1(dTriMeshDataID g,
    ////    const dReal* Vertices, int VertexCount,
    ////    const dTriIndex* Indices, int IndexCount,
    ////    const int *Normals)
    //    void dGeomTriMeshDataBuildSimple1(DTriMeshData g,
    //    final double[] Vertices, int VertexCount,
    //    final int[] Indices, int IndexCount,
    //    final int[] Normals)
    //    {
    ////#ifdef dSINGLE
    ////        dGeomTriMeshDataBuildSingle1(g,
    ////                Vertices, 4 * sizeof(dReal), VertexCount,
    ////                Indices, IndexCount, 3 * sizeof(dTriIndex),
    ////                Normals);
    ////#else
    ////        dGeomTriMeshDataBuildDouble1(g, Vertices, 4 * sizeof(dReal), VertexCount,
    ////                Indices, IndexCount, 3 * sizeof(dTriIndex),
    ////                Normals);
    //        dGeomTriMeshDataBuildDouble1(g, Vertices, 4 * 1, VertexCount,
    //                Indices, IndexCount, 3 * 1,
    //                Normals);
    ////#endif
    //    }


    // TZ(2021-04-11) These methods don't appear to have ever been available.
    /*extern ODE_API */
    //    void dGeomTriMeshDataBuildSingle(dTriMeshDataID g,
    //    const void* Vertices, int VertexStride, int VertexCount,
    //    const void* Indices, int IndexCount, int TriStride)
    //    {
    //        dGeomTriMeshDataBuildSingle1(g, Vertices, VertexStride, VertexCount,
    //                Indices, IndexCount, TriStride, (const void *)NULL);
    //    }

    // TZ(2021-04-11) These methods don't appear to have ever been available.
    //    /*extern ODE_API */
    //    //    void dGeomTriMeshDataBuildDouble(dTriMeshDataID g,
    //    //    const void* Vertices, int VertexStride, int VertexCount,
    //    //    const void* Indices, int IndexCount, int TriStride)
    //    void dGeomTriMeshDataBuildDouble(DTriMeshData g,
    //    final double[] Vertices, int VertexStride, int VertexCount,
    //    final int[] Indices, int IndexCount, int TriStride)
    //    {
    //        dGeomTriMeshDataBuildDouble1(g, Vertices, VertexStride, VertexCount,
    //                Indices, IndexCount, TriStride, null);
    //    }
    //
    //    /*extern ODE_API */
    //    //    void dGeomTriMeshDataBuildSimple(dTriMeshDataID g,
    //    //    const dReal* Vertices, int VertexCount,
    //    //    const dTriIndex* Indices, int IndexCount)
    //    void dGeomTriMeshDataBuildSimple(DTriMeshData g,
    //    final double[] Vertices, int VertexCount,
    //    final int[] Indices, int IndexCount)
    //    {
    //        dGeomTriMeshDataBuildSimple1(g,
    //                Vertices, VertexCount, Indices, IndexCount,
    //                (int[]) null);
    //    }


    /*extern ODE_API */
    //int
    //boolean dGeomTriMeshDataPreprocess(DTriMeshData g)
    public boolean preprocess()
    {
        //unsigned buildRequestFlags = (1U << dTRIDATAPREPROCESS_BUILD_CONCAVE_EDGES);
        int buildRequestFlags = (1 << DTriMeshData.dTRIDATAPREPROCESS_BUILD.CONCAVE_EDGES);
        return preprocess2(buildRequestFlags, null);
    }


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
    //    static final CEnumUnsortedElementArray g_TriMeshDataPreprocess_FaceAndlesExtraDataAngleStorageMethods =
    //            new CEnumUnsortedElementArray(new int[]{
    //            ASM_BYTE_POSITIVE, // dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA_BYTE_POSITIVE,
    //            ASM_BYTE_SIGNED, // dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA_BYTE_ALL,
    //            ASM_WORD_SIGNED, // dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA_WORD_ALL,
    //    }, dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MAX);


    /*extern ODE_API */
    //int dGeomTriMeshDataPreprocess2(dTriMeshDataID g, unsigned int buildRequestFlags, const dintptr *requestExtraData/*=NULL | const dintptr (*)[dTRIDATAPREPROCESS_BUILD__MAX]*/)
    public boolean preprocess2(int buildRequestFlags, final long[] requestExtraData/*=NULL | const dintptr (*)[dTRIDATAPREPROCESS_BUILD__MAX]*/)
    {
        //dUASSERT(g != null, "The argument is not a trimesh data");
        //dAASSERT((buildRequestFlags & (1U << dTRIDATAPREPROCESS_BUILD_FACE_ANGLES)) == 0 || requestExtraData == NULL || dIN_RANGE(requestExtraData[dTRIDATAPREPROCESS_BUILD_FACE_ANGLES], dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MIN, dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MAX));
        dAASSERT((buildRequestFlags & (1 << DTriMeshData.dTRIDATAPREPROCESS_BUILD.FACE_ANGLES)) == 0
                || requestExtraData == null
                || dIN_RANGE(requestExtraData[DTriMeshData.dTRIDATAPREPROCESS_BUILD.FACE_ANGLES],
                    dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MIN, dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MAX));

        DxTriMeshData data = (DxTriMeshData) this;

        boolean buildUseFlags = (buildRequestFlags & (1 << DTriMeshData.dTRIDATAPREPROCESS_BUILD.CONCAVE_EDGES)) != 0;
        //        FaceAngleStorageMethod faceAnglesRequirement = (buildRequestFlags & (1U << dTRIDATAPREPROCESS_BUILD_FACE_ANGLES)) != 0
        //            ? g_TriMeshDataPreprocess_FaceAndlesExtraDataAngleStorageMethods.Encode(requestExtraData != NULL && dIN_RANGE(requestExtraData[dTRIDATAPREPROCESS_BUILD_FACE_ANGLES], dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MIN, dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MAX) ? (unsigned)requestExtraData[dTRIDATAPREPROCESS_BUILD_FACE_ANGLES] : dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__DEFAULT)
        //            : ASM__INVALID;
        // TZ: we only support signed, anyway, so it is easiest to only support 'word'.
        int faceAnglesRequirement = ASM_WORD_SIGNED;

        return data.preprocessData(buildUseFlags, faceAnglesRequirement);
    }

    // #endif // #if dTRIMESH_ENABLED


    //////////////////////////////////////////////////////////////////////////
    // Deprecated functions

//    /*extern */
//    //void dGeomTriMeshDataGetBuffer(dTriMeshDataID g, unsigned char **buf, int *bufLen)
//    @Deprecated // Deprecated in ODE
//    void dGeomTriMeshDataGetBuffer(DTriMeshData g, Ref<byte[]> buf, RefInt bufLen)
//    {
//        RefInt dataSizeStorage = new RefInt();
//        //void *dataPointer = dGeomTriMeshDataGet2(g, dTRIMESHDATA_USE_FLAGS, (bufLen != NULL ? &dataSizeStorage : NULL));
//        byte[] dataPointer = dGeomTriMeshDataGet2(g, dTRIMESHDATA.USE_FLAGS, (bufLen != null ? dataSizeStorage : null));
//
//        if (bufLen != null)
//        {
//            //*bufLen = (int)dataSizeStorage;
//            bufLen.set( dataSizeStorage.get() );
//        }
//
//        if (buf != null)
//        {
//            buf.set( dataPointer );
//        }
//    }
//
//    /*extern */
//    @Deprecated // Deprecated in ODE
//    //void dGeomTriMeshDataSetBuffer(dTriMeshDataID g, unsigned char* buf)
//    void dGeomTriMeshDataSetBuffer(DTriMeshData g, byte[] buf)
//    {
//        //dGeomTriMeshDataSet(g, dTRIMESHDATA_USE_FLAGS, (void *)buf);
//        dGeomTriMeshDataSet(g, dTRIMESHDATA.USE_FLAGS, buf);
//    }

}
