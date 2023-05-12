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
import org.ode4j.ode.DTriMeshData;
import org.ode4j.ode.OdeConfig;
import org.ode4j.ode.internal.DxGimpactData;
import org.ode4j.ode.internal.DxTriMeshDisabled;

import java.util.Arrays;

import static org.ode4j.ode.DTriMesh.dMTV__MAX;
import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.Common.dUASSERT;

//typedef dxTriDataBase dxTriMeshData_Parent;
//struct dxTriMeshData:public dxTriMeshData_Parent {
public abstract class DxTriMeshData extends DxTriDataBase implements DTriMeshData {

    public static DTriMeshData dGeomTriMeshDataCreate() {
        switch (OdeConfig.dTRIMESH_TYPE) {
            case DISABLED: return new DxTriMeshDisabled.dxTriMeshDisabledData();
            case GIMPACT: return new DxGimpactData();
            default: throw new IllegalArgumentException(OdeConfig.dTRIMESH_TYPE.name());
        }
    }

    //public:
    //dxTriMeshData():
    //dxTriMeshData_Parent() {
    //}
    public DxTriMeshData() {
        super();
    }

    //~dxTriMeshData() { /* Do nothing */ }
    public void DESTRUCTOR() {
        /* Do nothing */
    }

    //using dxTriMeshData_Parent::buildData;

    /* Setup the UseFlags array and/or build face angles*/
    //bool preprocessData (bool buildUseFlags/*=false*/, FaceAngleStorageMethod faceAndgesRequirement/*=ASM__INVALID*/);

    //private:
    //bool meaningfulPreprocessData (FaceAngleStorageMethod faceAndgesRequirement/*=ASM__INVALID*/);

    //public:
    /* For when app changes the vertices */
    public void updateData () { /* Do nothing */ }

    //public:
    //        const vec3f * retrieveVertexInstances() const{
    //            return (const vec3f *)dxTriMeshData_Parent::retrieveVertexInstances ();
    //        }
    @Override
    public float[] retrieveVertexInstances() {
        return super.retrieveVertexInstances ();
    }
    //        const GUINT32 * retrieveTriangleVertexIndices() const{
    //            return (const GUINT32 *)dxTriMeshData_Parent::retrieveTriangleVertexIndices ();
    //        }
    @Override
    public int[] retrieveTriangleVertexIndices(){
        return super.retrieveTriangleVertexIndices ();
    }


    //public:
    //        void assignNormals (const dReal * normals){
    //            dxTriMeshData_Parent::assignNormals (normals);
    //        }
    public void assignNormals (final float[] normals){
        super.assignNormals (normals);
    }

    //        const dReal * retrieveNormals() const{
    //            return (const dReal *)dxTriMeshData_Parent::retrieveNormals ();
    //        }
    public float[] retrieveNormals() {
        return super.retrieveNormals ();
    }
    //        size_t calculateNormalsMemoryRequirement () const{
    //            return retrieveTriangleCount() * (sizeof(dReal) * dSA__MAX);
    //        }
    public int calculateNormalsMemoryRequirement () {
        throw new UnsupportedOperationException();
        //return retrieveTriangleCount() * (Double.BYTES * dSA__MAX);
    }

    //////////////////////////////////////////////////////////////////////////
    // dxTriMeshData

    //boolean
    //preprocessData(boolean x/*buildUseFlags*//*=false*/, FaceAngleStorageMethod faceAndgesRequirement/*=ASM__INVALID*/) {
    boolean preprocessData(boolean x/*buildUseFlags*//*=false*/, int faceAndgesRequirement/*=ASM__INVALID*/) {
        //FaceAngleStorageMethod faceAndgesRequirementToUse = faceAndgesRequirement;
        int faceAndgesRequirementToUse = faceAndgesRequirement;

        if (faceAndgesRequirement != ASM__INVALID && haveFaceAnglesBeenBuilt()) {
            dUASSERT(false, "Another request to build face angles after they had already been built");

            faceAndgesRequirementToUse = ASM__INVALID;
        }

        // If this mesh has already been preprocessed, exit
        boolean result = faceAndgesRequirementToUse == ASM__INVALID ||
                retrieveTriangleCount() == 0 ||
                meaningfulPreprocessData(faceAndgesRequirementToUse);
        return result;
    }

    static class TrimeshDataVertexIndexAccessor_GIMPACT implements TMeshDataAccessorI {
        //enum {
        //TRIANGLEINDEX_STRIDE = dxTriMesh::TRIANGLEINDEX_STRIDE,
        //} ;
        // static final int TRIANGLEINDEX_STRIDE = DxTriMesh.TRIANGLEINDEX_STRIDE;

        //    TrimeshDataVertexIndexAccessor_GIMPACT (dxTriMeshData * meshData):
        //    m_TriangleVertexIndices(meshData.retrieveTriangleVertexIndices()) {
        //        dIASSERT(meshData.retrieveTriangleStride() == TRIANGLEINDEX_STRIDE);
        //    }
        TrimeshDataVertexIndexAccessor_GIMPACT(DxTriMeshData meshData) {
            m_TriangleVertexIndices = meshData.retrieveTriangleVertexIndices();
            // dIASSERT(meshData.retrieveTriangleStride() == TRIANGLEINDEX_STRIDE);
        }

        //void getTriangleVertexIndices(unsigned out_VertexIndices[dMTV__MAX], unsigned triangleIdx) const
        @Override
        public void getTriangleVertexIndices(int[] out_VertexIndices, int triangleIdx) {
            //const GUINT32 * triIndicesBegin = m_TriangleVertexIndices;
            //final int triStride = TRIANGLEINDEX_STRIDE;

            //const GUINT32 * triIndicesOfInterest = (const GUINT32 *)((const uint8 *)
            //    triIndicesBegin + (size_t) triangleIdx * triStride);
            //std::copy (triIndicesOfInterest, triIndicesOfInterest + dMTV__MAX, out_VertexIndices);
            System.arraycopy(m_TriangleVertexIndices, triangleIdx * 3, out_VertexIndices, 0, dMTV__MAX);
        }

        //const GUINT32 * m_TriangleVertexIndices;
        final int[] m_TriangleVertexIndices;
    }

    static class TrimeshDataTrianglePointAccessor_GIMPACT implements TMeshDataAccessorP
    {
        //enum {
        //VERTEXINSTANCE_STRIDE = dxTriMesh::VERTEXINSTANCE_STRIDE, TRIANGLEINDEX_STRIDE = dxTriMesh::TRIANGLEINDEX_STRIDE,
        //} ;
        // final int VERTEXINSTANCE_STRIDE = DxTriMesh.VERTEXINSTANCE_STRIDE;
        // final int TRIANGLEINDEX_STRIDE = DxTriMesh.TRIANGLEINDEX_STRIDE;

        //        TrimeshDataTrianglePointAccessor_GIMPACT(dxTriMeshData * meshData):
        //        m_VertexInstances(meshData -> retrieveVertexInstances()), m_TriangleVertexIndices(meshData -> retrieveTriangleVertexIndices())
        TrimeshDataTrianglePointAccessor_GIMPACT(DxTriMeshData meshData) {
            m_VertexInstances = meshData.retrieveVertexInstances();
            m_TriangleVertexIndices = meshData.retrieveTriangleVertexIndices();
            // dIASSERT(meshData.retrieveVertexStride() == VERTEXINSTANCE_STRIDE);
            // dIASSERT(meshData.retrieveTriangleStride() == TRIANGLEINDEX_STRIDE);
        }

        //void getTriangleVertexPoints (dVector3 out_Points[dMTV__MAX], unsigned triangleIndex) const
        @Override
        public void getTriangleVertexPoints (DVector3[] out_Points, int triangleIndex) {
            //            DxTriMeshData.retrieveTriangleVertexPoints (out_Points, triangleIndex,
            //                m_VertexInstances, VERTEXINSTANCE_STRIDE,
            //                    m_TriangleVertexIndices, TRIANGLEINDEX_STRIDE);
            DxTriMeshData.retrieveTriangleVertexPoints (out_Points, triangleIndex,
                    m_VertexInstances,
                    m_TriangleVertexIndices);

        }

        //            const vec3f * m_VertexInstances;
        //            const GUINT32 * m_TriangleVertexIndices;
        final float[] m_VertexInstances;
        final int[] m_TriangleVertexIndices;

    }

    boolean //dxTriMeshData::
    //meaningfulPreprocessData(FaceAngleStorageMethod faceAndgesRequirement/*=ASM__INVALID*/) {
    meaningfulPreprocessData(int faceAndgesRequirement/*=ASM__INVALID*/) {
            final boolean buildFaceAngles = true;
        dIASSERT(faceAndgesRequirement != ASM__INVALID);
        // dIASSERT(buildFaceAngles);
        dIASSERT(/*!buildFaceAngles || */!haveFaceAnglesBeenBuilt());

        boolean result = false;

        boolean anglesAllocated = false;

        do {
            if (buildFaceAngles) {
                if (!allocateFaceAngles(faceAndgesRequirement)) {
                    break;
                }
            }

            anglesAllocated = true;

            final int numTris = retrieveTriangleCount();
            final int numVertices = retrieveVertexCount();
            int numEdges = numTris * dMTV__MAX;
            dIASSERT(numVertices <= numEdges); // Edge records are going to be used for vertex data as well

            //            final int recordsMemoryRequired = dEFFICIENT_SIZE(numEdges * sizeof(EdgeRecord));
            //            final int verticesMemoryRequired = /*dEFFICIENT_SIZE*/(numVertices * sizeof(VertexRecord)); // Skip alignment for the last chunk
            //            final int totalTempMemoryRequired = recordsMemoryRequired + verticesMemoryRequired;
            //            Void[] tempBuffer = dAlloc(totalTempMemoryRequired);
            //
            //            if (tempBuffer == null) {
            //                break;
            //            }

            EdgeRecord[] edges = new EdgeRecord[numEdges]; //(EdgeRecord *) tempBuffer;
            for (int i = 0; i < edges.length; i++) {
                edges[i] = new EdgeRecord();
            }
            VertexRecord[] vertices = new VertexRecord[numVertices]; //(VertexRecord *) ((uint8 *) tempBuffer + recordsMemoryRequired);
            for (int i = 0; i < vertices.length; i++) {
                vertices[i] = new VertexRecord();
            }

            TrimeshDataVertexIndexAccessor_GIMPACT indexAccessor = new TrimeshDataVertexIndexAccessor_GIMPACT(this);
            meaningfulPreprocess_SetupEdgeRecords(edges, numEdges, indexAccessor);

            // Sort the edges, so the ones sharing the same verts are beside each other
            //std::sort (edges, edges + numEdges);
            Arrays.sort(edges);

            TrimeshDataTrianglePointAccessor_GIMPACT pointAccessor  = new TrimeshDataTrianglePointAccessor_GIMPACT(this);
            final float[] externalNormals = retrieveNormals();
            IFaceAngleStorageControl faceAngles = retrieveFaceAngles();
            meaningfulPreprocess_buildEdgeFlags(null, faceAngles, edges, numEdges, vertices, externalNormals, pointAccessor);

            //dFree(tempBuffer, totalTempMemoryRequired);

            result = true;
        } while (false);

        if (!result) {
            if (anglesAllocated) {
                if (buildFaceAngles) {
                    freeFaceAngles();
                }
            }
        }

        return result;
    }


}
