/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zäschke      *
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
package org.ode4j.ode.internal;

import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.math.DVector4C;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DTriMesh;
import org.ode4j.ode.DTriMeshData;
import org.ode4j.ode.internal.gimpact.GimDynArray;
import org.ode4j.ode.internal.gimpact.GimTriCollision;
import org.ode4j.ode.internal.gimpact.GimTrimesh;
import org.ode4j.ode.internal.trimesh.DxMeshBase;

import static org.ode4j.ode.DTriMesh.dMTV__MAX;
import static org.ode4j.ode.internal.Common.dAASSERT;
import static org.ode4j.ode.internal.Common.dUASSERT;
import static org.ode4j.ode.internal.CommonEnums.dSA__MAX;
import static org.ode4j.ode.internal.gimpact.GimGeometry.*;

// TriMesh code by Erwin de Vries.
// Modified for FreeSOLID Compatibility by Rodrigo Hernandez
// Trimesh caches separation by Oleh Derevenko
// TriMesh storage classes refactoring and face angle computation code by Oleh Derevenko (C) 2016-2017

//****************************************************************************
// dxTriMesh class
public class CollisionTrimeshGimpact {
    // #ifndef _ODE_COLLISION_TRIMESH_GIMPACT_H_
    // #define _ODE_COLLISION_TRIMESH_GIMPACT_H_

    // #if dTRIMESH_ENABLED && dTRIMESH_GIMPACT

    // struct TrimeshCollidersCache // Required for compatibility with OPCODE
    //    {
    //    };


    //typedef dxTriDataBase dxTriMeshData_Parent;
    //struct dxTriMeshData:public dxTriMeshData_Parent {
    public static class DxTrimeshData extends org.ode4j.ode.internal.trimesh.DxTriDataBase {
        //public:
        //dxTriMeshData():
        //dxTriMeshData_Parent() {
        //}
        public DxTrimeshData() {
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
        vec3f retrieveVertexInstances() {
            return (vec3f) super.retrieveVertexInstances ();
        }
        //        const GUINT32 * retrieveTriangleVertexIndices() const{
        //            return (const GUINT32 *)dxTriMeshData_Parent::retrieveTriangleVertexIndices ();
        //        }
        int[] retrieveTriangleVertexIndices(){
            return super.retrieveTriangleVertexIndices ();
        }


        //public:
        //        void assignNormals (const dReal * normals){
        //            dxTriMeshData_Parent::assignNormals (normals);
        //        }
        public void assignNormals (final double[] normals){
            super.assignNormals (normals);
        }

        //        const dReal * retrieveNormals() const{
        //            return (const dReal *)dxTriMeshData_Parent::retrieveNormals ();
        //        }
        public double[] retrieveNormals() {
            return (double[])super.retrieveNormals ();
        }
        //        size_t calculateNormalsMemoryRequirement () const{
        //            return retrieveTriangleCount() * (sizeof(dReal) * dSA__MAX);
        //        }
        int calculateNormalsMemoryRequirement () {
            return retrieveTriangleCount() * (sizeof(dReal) * dSA__MAX);
        }
    }



    // #ifdef dDOUBLE
    // To use GIMPACT with doubles, we need to patch a couple of the GIMPACT functions to
    // convert arguments to floats before sending them in

    /// Convert an gimpact vec3f to a ODE dVector3d:   dVector3[i] = vec3f[i]
    //        #define dVECTOR3_VEC3F_COPY(b, a) { \
    //        (b)[0] = (a)[0];              \
    //        (b)[1] = (a)[1];              \
    //        (b)[2] = (a)[2];              \
    //        (b)[3] = 0;                   \
    //    }
    static void dVECTOR3_VEC3F_COPY(DVector3 b, vec3f a) {
        //        (b)[0] = (a)[0];
        //        (b)[1] = (a)[1];
        //        (b)[2] = (a)[2];
        //        (b)[3] = 0;
        b.set(a.f[0], a.f[1], a.f[2]);
    }

    private static void copy(vec3f b, DVector3C a) {
        b.f[0] = (float) a.get0();
        b.f[1] = (float) a.get1();
        b.f[2] = (float) a.get2();
    }

    private static void copy(vec4f b, DVector4C a) {
        b.f[0] = (float) a.get0();
        b.f[1] = (float) a.get1();
        b.f[2] = (float) a.get2();
        b.f[3] = (float) a.get3();
    }

    //    static inline
    //    void gim_trimesh_get_triangle_verticesODE(GIM_TRIMESH *trimesh, GUINT32 triangle_index, dVector3 v1, dVector3 v2, dVector3 v3) {
    static void gim_trimesh_get_triangle_verticesODE(GimTrimesh trimesh, int triangle_index, DVector3 v1, DVector3 v2, DVector3 v3) {

        //vec3f src1, src2, src3;
        //        GREAL * psrc1 = v1 != null ? src1 : null;
        //        GREAL * psrc2 = v2 != null ? src2 : null;
        //        GREAL * psrc3 = v3 != null ? src3 : null;
        vec3f psrc1 = v1 != null ? new vec3f() : null;
        vec3f psrc2 = v2 != null ? new vec3f() : null;
        vec3f psrc3 = v3 != null ? new vec3f() : null;

        gim_trimesh_get_triangle_vertices(trimesh, triangle_index, psrc1, psrc2, psrc3);

        if (v1 != null) {
            dVECTOR3_VEC3F_COPY(v1, psrc1);
        }

        if (v2 != null) {
            dVECTOR3_VEC3F_COPY(v2, psrc2);
        }

        if (v3 != null) {
            dVECTOR3_VEC3F_COPY(v3, psrc3);
        }
    }

    // Anything calling gim_trimesh_get_triangle_vertices from within ODE
    // should be patched through to the dDOUBLE version above

    //#define gim_trimesh_get_triangle_vertices gim_trimesh_get_triangle_verticesODE

    //    static inline
    //    int gim_trimesh_ray_closest_collisionODE(GIM_TRIMESH *mesh, dVector3 origin, dVector3 dir, dReal tmax, GIM_TRIANGLE_RAY_CONTACT_DATA *contact) {
    static
    int gim_trimesh_ray_closest_collisionODE(GimTrimesh mesh, DVector3 origin, DVector3 dir, double tmax, GimTriCollision.GIM_TRIANGLE_RAY_CONTACT_DATA contact) {
        vec3f dir_vec3f = new vec3f();
        copy(dir_vec3f, dir);//.f[0] = (float) dir.get0(); dir_vec3f.f[1] = (float) dir.get1(); dir_vec3f.f[2] = (float) dir.get2();
        //vec3f origin_vec3f = {(float) origin[0], (float) origin[1], (float) origin[2]};
        vec3f origin_vec3f = new vec3f();
        copy(origin_vec3f, origin);
        return gim_trimesh_ray_closest_collision(mesh, origin_vec3f, dir_vec3f, (float) tmax, contact);
    }

//    static inline
//    int gim_trimesh_ray_collisionODE(GIM_TRIMESH *mesh, const dVector3 origin, const dVector3 dir, dReal tmax, GIM_TRIANGLE_RAY_CONTACT_DATA *contact) {
    static
    int gim_trimesh_ray_collisionODE(GimTrimesh mesh, DVector3C origin, DVector3C dir, double tmax, GimTriCollision.GIM_TRIANGLE_RAY_CONTACT_DATA contact) {
        //vec3f dir_vec3f = {(GREAL) dir[0], (GREAL) dir[1], (GREAL) dir[2]};
        vec3f dir_vec3f = new vec3f();
        copy(dir_vec3f, dir);
        //vec3f origin_vec3f = {(GREAL) origin[0], (GREAL) origin[1], (GREAL) origin[2]};
        vec3f origin_vec3f = new vec3f();
        copy(origin_vec3f, origin);

        return gim_trimesh_ray_collision(mesh, origin_vec3f, dir_vec3f, (float) tmax, contact);
    }

        //    static inline
        //    void gim_trimesh_sphere_collisionODE(GIM_TRIMESH *mesh, const dVector3 Position, dReal Radius, GDYNAMIC_ARRAY *contact) {
    static
    void gim_trimesh_sphere_collisionODE(GimTrimesh mesh, DVector3C Position, double Radius, GimDynArray contact) {
        //vec3f pos_vec3f = {(GREAL) Position[0], (GREAL) Position[1], (GREAL) Position[2]};
        vec3f pos_vec3f = new vec3f();
        copy(pos_vec3f, Position);
        gim_trimesh_sphere_collision(mesh, pos_vec3f, (float) Radius, contact);
    }

    //    static inline
    //    void gim_trimesh_plane_collisionODE(GIM_TRIMESH *mesh, const dVector4 plane, GDYNAMIC_ARRAY *contact) {
    static
    void gim_trimesh_plane_collisionODE(GimTrimesh mesh, DVector4C plane, GimDynArray contact) {
        //vec4f plane_vec4f = {(GREAL) plane[0], (GREAL) plane[1], (GREAL) plane[2], (GREAL) plane[3]};
        vec4f plane_vec4f = new vec4f();
        copy(plane_vec4f, plane);
        gim_trimesh_plane_collision(mesh, plane_vec4f, contact);
    }

//        #define GIM_AABB_COPY(src, dst) {		\
//        (dst)[0] = (src) -> minX;			\
//        (dst)[1] = (src) -> maxX;			\
//        (dst)[2] = (src) -> minY;			\
//        (dst)[3] = (src) -> maxY;			\
//        (dst)[4] = (src) -> minZ;			\
//        (dst)[5] = (src) -> maxZ;			\
//    }
    private static void GIM_AABB_COPY(src, dst) {
        (dst)[0] = (src) -> minX;
        (dst)[1] = (src) -> maxX;
        (dst)[2] = (src) -> minY;
        (dst)[3] = (src) -> maxY;
        (dst)[4] = (src) -> minZ;
        (dst)[5] = (src) -> maxZ;
    }


    //        #else // #ifdef !dDOUBLE
    //
    //                // With single precision, we can pass native ODE vectors directly to GIMPACT
    //
    //                #
    //    define gim_trimesh_ray_closest_collisionODE
    //    gim_trimesh_ray_closest_collision
    //        #
    //    define gim_trimesh_ray_collisionODE
    //    gim_trimesh_ray_collision
    //        #
    //    define gim_trimesh_sphere_collisionODE
    //    gim_trimesh_sphere_collision
    //        #
    //    define gim_trimesh_plane_collisionODE
    //    gim_trimesh_plane_collision
    //
    //        #
    //
    //    define GIM_AABB_COPY(src, dst) 	memcpy(dst,src,6*
    //
    //    sizeof(GREAL) )
    //
    //
    //            #endif // #ifdef !dDOUBLE


//    typedef dxMeshBase dxTriMesh_Parent;
//    struct dxTriMesh: public dxTriMesh_Parent {
    static class DxTriMesh extends DxMeshBase {

        //public:
        // Functions
        //dxTriMesh(dxSpace * Space, dxTriMeshData * Data, dTriCallback * Callback, dTriArrayCallback * ArrayCallback, dTriRayCallback * RayCallback):
        //dxTriMesh_Parent(Space, NULL, Callback, ArrayCallback, RayCallback, true) // TC has speed/space 'issues' that don't make it a clear win by default on spheres/boxes.
        DxTriMesh(DxSpace Space, DxTriMeshData Data, DTriMesh.DTriCallback Callback,
                  DTriMesh.DTriArrayCallback ArrayCallback, DTriMesh.DTriRayCallback RayCallback) {
            // TC has speed/space 'issues' that don't make it a clear win by default on spheres/boxes.
            super(Space, null, Callback, ArrayCallback, RayCallback, true);
            gim_init_buffer_managers(m_buffer_managers);
            assignMeshData(Data);
        }

        //~dxTriMesh();
        public void Destructor() {
            //
        }

        public void clearTCCache () { /* do nothing */ }

        //virtual void computeAABB ();
        public abstract void computeAABB ();

        //public:
        //dxTriMeshData * retrieveMeshData() const{
        public DxTriMeshData retrieveMeshData() {
            return getMeshData();
        }

        //unsigned getMeshTriangleCount () const{
        public int getMeshTriangleCount () {
            //return gim_trimesh_get_triangle_count(const_cast < GIM_TRIMESH * > ( & m_collision_trimesh));
            return gim_trimesh_get_triangle_count(m_collision_trimesh);
        }

        //void fetchMeshTransformedTriangle (dVector3 *const pout_triangle[3], unsigned index)
        public void fetchMeshTransformedTriangle (DVector3C[] pout_triangle, int index)
        {
            gim_trimesh_locks_work_data( m_collision_trimesh);
            //            gim_trimesh_get_triangle_vertices( m_collision_trimesh, (GUINT32) index, *pout_triangle[0], *
            //            pout_triangle[1], *pout_triangle[2]);
            gim_trimesh_get_triangle_vertices( m_collision_trimesh, index, *pout_triangle[0], *
            pout_triangle[1], *pout_triangle[2]);
            gim_trimesh_unlocks_work_data( m_collision_trimesh);
        }

        //void fetchMeshTransformedTriangle (dVector3 out_triangle[3], unsigned index)
        public void fetchMeshTransformedTriangle (DVector3[] out_triangle, int index)
        {
            gim_trimesh_locks_work_data( m_collision_trimesh);
            gim_trimesh_get_triangle_vertices( m_collision_trimesh, (GUINT32) index, out_triangle[0], out_triangle[1], out_triangle[2]);
            gim_trimesh_unlocks_work_data( m_collision_trimesh);
        }

        //private:
        //dxTriMeshData * getMeshData() const{
        private DxTriMeshData getMeshData() {
            //return static_cast < dxTriMeshData * > (dxTriMesh_Parent::getMeshData ());
            return (DxTriMeshData) (super.getMeshData());
        }

        //        public:
        //        enum {
        //            VERTEXINSTANCE_STRIDE = sizeof(vec3f), TRIANGLEINDEX_STRIDE = sizeof(GUINT32) * dMTV__MAX,
        //        } ;
        @Deprecated // TODO CHECK-TZ these should not be used!
        public static final int VERTEXINSTANCE_STRIDE = 1;//sizeof(vec3f);
        @Deprecated // TODO CHECK-TZ these should not be used!
        public static final int TRIANGLEINDEX_STRIDE = 1 /*sizeof(GUINT32)*/ * dMTV__MAX;

        //void assignMeshData (dxTriMeshData * Data);

        //public:
        //GIM_TRIMESH m_collision_trimesh;
        //GBUFFER_MANAGER_DATA m_buffer_managers[ G_BUFFER_MANAGER__MAX];
        GimTrimesh m_collision_trimesh;
        GBUFFER_MANAGER_DATA m_buffer_managers[ G_BUFFER_MANAGER__MAX];
    }


//    static inline
//    void MakeMatrix(const dVector3 position, const dMatrix3 rotation, mat4f m) {
//        m[0][0] = (GREAL) rotation[dM3E_XX];
//        m[0][1] = (GREAL) rotation[dM3E_XY];
//        m[0][2] = (GREAL) rotation[dM3E_XZ];
//
//        m[1][0] = (GREAL) rotation[dM3E_YX];
//        m[1][1] = (GREAL) rotation[dM3E_YY];
//        m[1][2] = (GREAL) rotation[dM3E_YZ];
//
//        m[2][0] = (GREAL) rotation[dM3E_ZX];
//        m[2][1] = (GREAL) rotation[dM3E_ZY];
//        m[2][2] = (GREAL) rotation[dM3E_ZZ];
//
//        m[0][3] = (GREAL) position[dV3E_X];
//        m[1][3] = (GREAL) position[dV3E_Y];
//        m[2][3] = (GREAL) position[dV3E_Z];
//    }
        private static void MakeMatrix(DVector3C position, DMatrix3C rotation, mat4f m) {
        m[0][0] = (float) rotation[dM3E_XX];
        m[0][1] = (float) rotation[dM3E_XY];
        m[0][2] = (float) rotation[dM3E_XZ];

        m[1][0] = (float) rotation[dM3E_YX];
        m[1][1] = (float) rotation[dM3E_YY];
        m[1][2] = (float) rotation[dM3E_YZ];

        m[2][0] = (float) rotation[dM3E_ZX];
        m[2][1] = (float) rotation[dM3E_ZY];
        m[2][2] = (float) rotation[dM3E_ZZ];

        m[0][3] = (float) position[dV3E_X];
        m[1][3] = (float) position[dV3E_Y];
        m[2][3] = (float) position[dV3E_Z];
        }


    //    static inline
    //    void MakeMatrix(dxGeom *g, mat4f m) {
    //        const dVector3 & position = g -> buildUpdatedPosition();
    //        const dMatrix3 & rotation = g -> buildUpdatedRotation();
    //        MakeMatrix(position, rotation, m);
    //    }
    static void MakeMatrix(DxGeom g, mat4f m) {
        DVector3C position = g.buildUpdatedPosition();
        DMatrix3C rotation = g.buildUpdatedRotation();
        MakeMatrix(position, rotation, m);
    }



        //#endif // #if dTRIMESH_ENABLED && dTRIMESH_GIMPACT

        //#endif    //_ODE_COLLISION_TRIMESH_GIMPACT_H_


    /*************************************************************************
     *                                                                       *
     * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
     * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
     *                                                                       *
     * This library is free software; you can redistribute it and/or         *
     * modify it under the terms of EITHER:                                  *
     *   (1) The GNU Lesser General Public License as published by the Free  *
     *       Software Foundation; either version 2.1 of the License, or (at  *
     *       your option) any later version. The text of the GNU Lesser      *
     *       General Public License is included with this library in the     *
     *       file LICENSE.TXT.                                               *
     *   (2) The BSD-style license that is included with this library in     *
     *       the file LICENSE-BSD.TXT.                                       *
     *                                                                       *
     * This library is distributed in the hope that it will be useful,       *
     * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
     * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
     * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
     *                                                                       *
     *************************************************************************/

    // TriMesh storage classes refactoring and face angle computation code by Oleh Derevenko (C) 2016-2017

    //        #include<ode/collision.h>
    //            #include<ode/rotation.h>
    //            #include "config.h"
    //            #include "matrix.h"
    //            #include "odemath.h"
    //            #include "util.h"
    //
    //
    //            #if dTRIMESH_ENABLED &&dTRIMESH_GIMPACT
    //
    //        #include "collision_util.h"
    //            #include "collision_trimesh_gimpact.h"
    //            #include "collision_trimesh_internal_impl.h"


    //////////////////////////////////////////////////////////////////////////
    // dxTriMeshData

    bool dxTriMeshData::
    preprocessData(bool /*buildUseFlags*//*=false*/, FaceAngleStorageMethod faceAndgesRequirement/*=ASM__INVALID*/) {
        FaceAngleStorageMethod faceAndgesRequirementToUse = faceAndgesRequirement;

        if (faceAndgesRequirement != ASM__INVALID && haveFaceAnglesBeenBuilt()) {
            dUASSERT(false, "Another request to build face angles after they had already been built");

            faceAndgesRequirementToUse = ASM__INVALID;
        }

        // If this mesh has already been preprocessed, exit
        bool result = faceAndgesRequirementToUse == ASM__INVALID || retrieveTriangleCount() == 0 || meaningfulPreprocessData(faceAndgesRequirementToUse);
        return result;
    }

    struct TrimeshDataVertexIndexAccessor_GIMPACT
    {
        enum {
        TRIANGLEINDEX_STRIDE = dxTriMesh::TRIANGLEINDEX_STRIDE,
    } ;

        explicit TrimeshDataVertexIndexAccessor_GIMPACT (dxTriMeshData * meshData):
        m_TriangleVertexIndices(meshData -> retrieveTriangleVertexIndices()) {
        dIASSERT(meshData -> retrieveTriangleStride() == TRIANGLEINDEX_STRIDE);
    }

        void getTriangleVertexIndices (unsigned out_VertexIndices[dMTV__MAX], unsigned triangleIdx) const
        {
            const GUINT32 * triIndicesBegin = m_TriangleVertexIndices;
            const unsigned triStride = TRIANGLEINDEX_STRIDE;

            const GUINT32 * triIndicesOfInterest = (const GUINT32 *)((const uint8 *)
            triIndicesBegin + (size_t) triangleIdx * triStride);
            std::copy (triIndicesOfInterest, triIndicesOfInterest + dMTV__MAX, out_VertexIndices);
        }

            const GUINT32 * m_TriangleVertexIndices;
    }

    ;

    struct TrimeshDataTrianglePointAccessor_GIMPACT
    {
        enum {
        VERTEXINSTANCE_STRIDE = dxTriMesh::VERTEXINSTANCE_STRIDE, TRIANGLEINDEX_STRIDE = dxTriMesh::TRIANGLEINDEX_STRIDE,
    } ;

        TrimeshDataTrianglePointAccessor_GIMPACT(dxTriMeshData * meshData):
        m_VertexInstances(meshData -> retrieveVertexInstances()), m_TriangleVertexIndices(meshData -> retrieveTriangleVertexIndices())
        {
            dIASSERT((unsigned) meshData -> retrieveVertexStride() == (unsigned) VERTEXINSTANCE_STRIDE);
            dIASSERT((unsigned) meshData -> retrieveTriangleStride() == (unsigned) TRIANGLEINDEX_STRIDE);
        }

        void getTriangleVertexPoints (dVector3 out_Points[dMTV__MAX], unsigned triangleIndex) const
        {
            dxTriMeshData::retrieveTriangleVertexPoints (out_Points, triangleIndex,
            &m_VertexInstances[0][0], VERTEXINSTANCE_STRIDE, m_TriangleVertexIndices, TRIANGLEINDEX_STRIDE);
        }

            const vec3f * m_VertexInstances;
            const GUINT32 * m_TriangleVertexIndices;
    }

    ;

    bool dxTriMeshData::
    meaningfulPreprocessData(FaceAngleStorageMethod faceAndgesRequirement/*=ASM__INVALID*/) {
            const bool buildFaceAngles = true;
        dIASSERT(faceAndgesRequirement != ASM__INVALID);
        // dIASSERT(buildFaceAngles);
        dIASSERT(/*!buildFaceAngles || */!haveFaceAnglesBeenBuilt());

        bool result = false;

        bool anglesAllocated = false;

        do {
            if (buildFaceAngles) {
                if (!allocateFaceAngles(faceAndgesRequirement)) {
                    break;
                }
            }

            anglesAllocated = true;

            const unsigned int numTris = retrieveTriangleCount();
            const unsigned int numVertices = retrieveVertexCount();
            size_t numEdges = (size_t) numTris * dMTV__MAX;
            dIASSERT(numVertices <= numEdges); // Edge records are going to be used for vertex data as well

            const size_t recordsMemoryRequired = dEFFICIENT_SIZE(numEdges * sizeof(EdgeRecord));
            const size_t verticesMemoryRequired = /*dEFFICIENT_SIZE*/(numVertices * sizeof(VertexRecord)); // Skip alignment for the last chunk
            const size_t totalTempMemoryRequired = recordsMemoryRequired + verticesMemoryRequired;
            void *tempBuffer = dAlloc(totalTempMemoryRequired);

            if (tempBuffer == NULL) {
                break;
            }

            EdgeRecord * edges = (EdgeRecord *) tempBuffer;
            VertexRecord * vertices = (VertexRecord *) ((uint8 *) tempBuffer + recordsMemoryRequired);

            TrimeshDataVertexIndexAccessor_GIMPACT indexAccessor (this);
            meaningfulPreprocess_SetupEdgeRecords(edges, numEdges, indexAccessor);

            // Sort the edges, so the ones sharing the same verts are beside each other
            std::sort (edges, edges + numEdges);

            TrimeshDataTrianglePointAccessor_GIMPACT pointAccessor (this);
            const dReal *const externalNormals = retrieveNormals();
            IFaceAngleStorageControl * faceAngles = retrieveFaceAngles();
            meaningfulPreprocess_buildEdgeFlags(NULL, faceAngles, edges, numEdges, vertices, externalNormals, pointAccessor);

            dFree(tempBuffer, totalTempMemoryRequired);

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


    //////////////////////////////////////////////////////////////////////////
    // Trimesh

    dxTriMesh::~

    dxTriMesh() {
        //Terminate Trimesh
        gim_trimesh_destroy( & m_collision_trimesh);
        gim_terminate_buffer_managers(m_buffer_managers);
    }


    /*virtual */ void dxTriMesh::

    computeAABB() {
        //update trimesh transform
        mat4f transform;
        IDENTIFY_MATRIX_4X4(transform);
        MakeMatrix(this, transform);
        gim_trimesh_set_tranform( & m_collision_trimesh, transform);

        //Update trimesh boxes
        gim_trimesh_update( & m_collision_trimesh);

        GIM_AABB_COPY( & m_collision_trimesh.m_aabbset.m_global_bound, aabb );
    }


    void dxTriMesh::

    assignMeshData(dxTriMeshData *Data) {
        // GIMPACT only supports stride 12, so we need to catch the error early.
        dUASSERT((unsigned int)Data -> retrieveVertexStride() == (unsigned) VERTEXINSTANCE_STRIDE && (unsigned int)
        Data -> retrieveTriangleStride() == (unsigned) TRIANGLEINDEX_STRIDE, "Gimpact trimesh only supports a stride of 3 float/int\n"
        "This means that you cannot use dGeomTriMeshDataBuildSimple() with Gimpact.\n"
        "Change the stride, or use Opcode trimeshes instead.\n"
            );

        dxTriMesh_Parent::assignMeshData (Data);

        //Create trimesh
            const vec3f * vertexInstances = Data -> retrieveVertexInstances();
        if (vertexInstances != NULL) {
            const GUINT32 * triangleVertexIndices = Data -> retrieveTriangleVertexIndices();

            size_t vertexInstanceCount = Data -> retrieveVertexCount();
            size_t triangleVertexCount = (size_t) Data -> retrieveTriangleCount() * dMTV__MAX;

            gim_trimesh_create_from_data(m_buffer_managers, & m_collision_trimesh,                           // gimpact mesh
                    const_cast < vec3f * > (vertexInstances),           // vertices
                    dCAST_TO_SMALLER(GUINT32, vertexInstanceCount), // nr of verts
                    0,                                              // copy verts?
                    const_cast < GUINT32 * > (triangleVertexIndices),   // indices
                    dCAST_TO_SMALLER(GUINT32, triangleVertexCount), // nr of indices
                    0,                                              // copy indices?
                    1                                               // transformed reply
        );
        }
    }


    //////////////////////////////////////////////////////////////////////////

    /*extern */
    dTriMeshDataID dGeomTriMeshDataCreate() {
        return new dxTriMeshData();
    }

    /*extern */
    void dGeomTriMeshDataDestroy(dTriMeshDataID g) {
        dxTriMeshData * data = g;
        delete data;
    }

    /*extern */
    void dGeomTriMeshDataSet(dTriMeshDataID g, int dataId, void *pDataLocation) {
        dUASSERT(g, "The argument is not a trimesh data");

        dxTriMeshData * data = g;

        switch (dataId) {
            case dTRIMESHDATA_FACE_NORMALS: {
                data -> assignNormals((const dReal *)pDataLocation);
                break;
            }

            case dTRIMESHDATA_USE_FLAGS: // Not used for GIMPACT
            {
                break;
            }

            // case dTRIMESHDATA__MAX: -- To be located by Find in Files
            default: {
                dUASSERT(dataId, "invalid data type");
                break;
            }
        }
    }

    static void *

    geomTriMeshDataGet(dTriMeshDataID g, int dataId, size_t *pOutDataSize);

    /*extern */
    void *

    dGeomTriMeshDataGet(dTriMeshDataID g, int dataId) {
        return geomTriMeshDataGet(g, dataId, NULL);
    }

    /*extern */
    void *

    dGeomTriMeshDataGet2(dTriMeshDataID g, int dataId, size_t *pOutDataSize) {
        return geomTriMeshDataGet(g, dataId, pOutDataSize);
    }

    static void *

    geomTriMeshDataGet(dTriMeshDataID g, int dataId, size_t *pOutDataSize) {
        dUASSERT(g, "The argument is not a trimesh data");

        const dxTriMeshData * data = g;

        void *result = NULL;

        switch (dataId) {
            case dTRIMESHDATA_FACE_NORMALS: {
                if (pOutDataSize != NULL) {
        *pOutDataSize = data -> calculateNormalsMemoryRequirement();
                }

                result = ( void *)data -> retrieveNormals();
                break;
            }

            case dTRIMESHDATA_USE_FLAGS: // Not not used for GIMPACT
            {
                if (pOutDataSize != NULL) {
        *pOutDataSize = 0;
                }

                break;
            }

            // case dTRIMESHDATA__MAX: -- To be located by Find in Files
            default: {
                if (pOutDataSize != NULL) {
        *pOutDataSize = 0;
                }

                dUASSERT(dataId, "invalid data type");
                break;
            }
        }

        return result;
    }

    /*extern */
    void dGeomTriMeshDataBuildSingle1(dTriMeshDataID g,
        const void*Vertices, int VertexStride, int VertexCount,
        const void*Indices, int IndexCount, int TriStride,
        const void*Normals) {
        dUASSERT(g, "The argument is not a trimesh data");
        dAASSERT(Vertices);
        dAASSERT(Indices);

        dxTriMeshData * data = g;

        data -> buildData(Vertices, VertexStride, VertexCount, Indices, IndexCount, TriStride, Normals, true);
    }

    /*extern */
    void dGeomTriMeshDataBuildDouble1(dTriMeshDataID g,
        const void*Vertices, int VertexStride, int VertexCount,
        const void*Indices, int IndexCount, int TriStride,
        const void*Normals) {
        dUASSERT(g, "The argument is not a trimesh data");
        dAASSERT(Vertices);
        dAASSERT(Indices);

        dxTriMeshData * data = g;

        data -> buildData(Vertices, VertexStride, VertexCount, Indices, IndexCount, TriStride, Normals, false);
    }


    //////////////////////////////////////////////////////////////////////////

    /*extern */
    //dGeomID dCreateTriMesh(dSpaceID space, dTriMeshDataID Data, dTriCallback*Callback, dTriArrayCallback*ArrayCallback, dTriRayCallback*RayCallback) {
    DGeom dCreateTriMesh(DSpace space, DTriMeshData Data, DTriMesh.DTriCallback Callback,
                         DTriMesh.DTriArrayCallback ArrayCallback, DTriMesh.DTriRayCallback RayCallback) {
        DxTriMesh mesh = new DxTriMesh(space, Data, Callback, ArrayCallback, RayCallback);
        return mesh;
    }


    /*extern */
    //void dGeomTriMeshSetLastTransform(dGeomID g, const dMatrix4 last_trans) {
    void dGeomTriMeshSetLastTransform(DGeom g, DMatrix4C last_trans) {
        dAASSERT(g);
        dUASSERT(g instanceof DTriMesh, "The geom is not a trimesh");

        //stub
    }

    /*extern */
    //    const dReal *
    //dGeomTriMeshGetLastTransform(dGeomID g) {
    double[] dGeomTriMeshGetLastTransform(DGeom g) {
        dAASSERT(g);
        dUASSERT((g instanceof DTriMesh), "The geom is not a trimesh");

        return null; // stub
    }


    //    #endif // #if dTRIMESH_ENABLED && dTRIMESH_GIMPACT

}