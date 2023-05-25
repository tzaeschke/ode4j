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

import org.ode4j.math.*;
import org.ode4j.ode.*;
import org.ode4j.ode.internal.gimpact.GimContact;
import org.ode4j.ode.internal.gimpact.GimDynArray;
import org.ode4j.ode.internal.gimpact.GimTriCollision;
import org.ode4j.ode.internal.gimpact.GimTrimesh;
import org.ode4j.ode.internal.trimesh.DxTriMeshData;

import static org.ode4j.ode.internal.Common.dAASSERT;
import static org.ode4j.ode.internal.Common.dUASSERT;
import static org.ode4j.ode.internal.gimpact.GimGeometry.*;

// TriMesh code by Erwin de Vries.
// Modified for FreeSOLID Compatibility by Rodrigo Hernandez
// Trimesh caches separation by Oleh Derevenko
// TriMesh storage classes refactoring and face angle computation code by Oleh Derevenko (C) 2016-2017

//****************************************************************************
// dxTriMesh class
class CollisionTrimeshGimpact {
    // #ifndef _ODE_COLLISION_TRIMESH_GIMPACT_H_
    // #define _ODE_COLLISION_TRIMESH_GIMPACT_H_

    // #if dTRIMESH_ENABLED && dTRIMESH_GIMPACT

    // struct TrimeshCollidersCache // Required for compatibility with OPCODE
    //    {
    //    };





    //	#ifdef dDOUBLE
    // To use GIMPACT with doubles, we need to patch a couple of the GIMPACT functions to
    // convert arguments to floats before sending them in


    // Convert an gimpact vec3f to a ODE dVector3d:   dVector3[i] = vec3f[i]
//			private void dVECTOR3_VEC3F_COPY(b,a) {
//				(b)[0] = (a)[0];
//				(b)[1] = (a)[1];
//				(b)[2] = (a)[2];
//				(b)[3] = 0;
//			}

    //private void gim_trimesh_get_triangle_verticesODE(GimTrimesh trimesh, GUINT triangle_index, dVector3 v1, dVector3 v2, dVector3 v3) {
//			private void gim_trimesh_get_triangle_verticesODE(GimTrimesh trimesh, int triangle_index,
//					DVector3 v1, DVector3 v2, DVector3 v3) {
//				vec3f src1 = new vec3f(), src2 = new vec3f(), src3 = new vec3f();
//				trimesh.gim_trimesh_get_triangle_vertices(triangle_index, src1, src2, src3);
//
//				v1.set(src1.f);//dVECTOR3_VEC3F_COPY(v1, src1);
//				v2.set(src2.f);//dVECTOR3_VEC3F_COPY(v2, src2);
//				v3.set(src3.f);//dVECTOR3_VEC3F_COPY(v3, src3);
//			}

    private static vec3f DVector3Tovec3f(DVector3C v) {
        vec3f vf = new vec3f();
        vf.f[0] = (float) v.get0();
        vf.f[1] = (float) v.get1();
        vf.f[2] = (float) v.get2();
        return vf;
    }

    private static vec4f DVector4Tovec4f(DVector4C v) {
        vec4f vf = new vec4f();
        vf.f[0] = (float) v.get0();
        vf.f[1] = (float) v.get1();
        vf.f[2] = (float) v.get2();
        vf.f[3] = (float) v.get3();
        return vf;
    }

    // Anything calling gim_trimesh_get_triangle_vertices from within ODE
    // should be patched through to the dDOUBLE version above

    //#define gim_trimesh_get_triangle_vertices gim_trimesh_get_triangle_verticesODE
//			static void gim_trimesh_get_triangle_verticesODE( GimTrimesh mesh ) {
//				mesh.gim_trimesh_get_triangle_vertices(0, null, null, null);
//			}

    //			inline int gim_trimesh_ray_closest_collisionODE( GIM_TRIMESH *mesh, dVector3 origin,
//					dVector3 dir, GREAL tmax, GIM_TRIANGLE_RAY_CONTACT_DATA *contact ) {
    static int gim_trimesh_ray_closest_collisionODE( GimTrimesh mesh, DVector3C origin, DVector3C dir,
                                                     double tmax, GimTriCollision.GIM_TRIANGLE_RAY_CONTACT_DATA contact ) {
        vec3f dir_vec3f    = DVector3Tovec3f( dir );
        vec3f origin_vec3f = DVector3Tovec3f( origin );

        return mesh.gim_trimesh_ray_closest_collision( origin_vec3f, dir_vec3f, (float) tmax, contact );
    }

    //inline int gim_trimesh_ray_collisionODE( GIM_TRIMESH *mesh, dVector3 origin, dVector3 dir,
    //GREAL tmax, GIM_TRIANGLE_RAY_CONTACT_DATA *contact ) {
    static int gim_trimesh_ray_collisionODE( GimTrimesh mesh, DVector3C origin, DVector3C dir,
                                             double tmax, GimTriCollision.GIM_TRIANGLE_RAY_CONTACT_DATA contact ) {
        vec3f dir_vec3f    = DVector3Tovec3f( dir );
        vec3f origin_vec3f = DVector3Tovec3f( origin );

        return mesh.gim_trimesh_ray_collision( origin_vec3f, dir_vec3f, (float) tmax, contact );
    }

    //#define gim_trimesh_sphere_collisionODE( mesh, Position, Radius, contact ) {	\
    static void gim_trimesh_sphere_collisionODE( GimTrimesh mesh, DVector3C Position,
                                                 double Radius, GimDynArray<GimContact> contact ) {
        vec3f pos_vec3f = DVector3Tovec3f( Position );
        mesh.gim_trimesh_sphere_collision( pos_vec3f, (float) Radius, contact );
    }

    //#define gim_trimesh_plane_collisionODE( mesh, plane, contact ) { 			\
    static void gim_trimesh_plane_collisionODE( GimTrimesh mesh, DVector4 plane,
                                                GimDynArray<vec4f> contact ) {
        vec4f plane_vec4f = DVector4Tovec4f( plane );
        mesh.gim_trimesh_plane_collision( plane_vec4f, contact );
    }

    //#define GIM_AABB_COPY( src, dst ) {		\
    static void GIM_AABB_COPY( aabb3f src, DAABB dst ) {
        dst.set( src.minX, src.maxX, src.minY, src.maxY, src.minZ, src.maxZ );
    }


    //	#else
    //		// With single precision, we can pass native ODE vectors directly to GIMPACT
    //
    //		#define gim_trimesh_ray_closest_collisionODE 	gim_trimesh_ray_closest_collision
    //		#define gim_trimesh_ray_collisionODE 			gim_trimesh_ray_collision
    //		#define gim_trimesh_sphere_collisionODE 		gim_trimesh_sphere_collision
    //		#define gim_trimesh_plane_collisionODE 			gim_trimesh_plane_collision
    //
    //		#define GIM_AABB_COPY( src, dst ) 	memcpy( dst, src, 6 * sizeof( GREAL ) )
    //
    //	#endif // dDouble




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

        private static int M(int y, int x) { return y*4 + x; }

    private static void MakeMatrix(DVector3C position, DMatrix3C rotation, mat4f m) {

        m.f[M(0, 0)] = (float) rotation.get00(); //[dM3E_XX];
        m.f[M(0, 1)] = (float) rotation.get01(); //[dM3E_XY];
        m.f[M(0, 2)] = (float) rotation.get02(); //[dM3E_XZ];

        m.f[M(1, 0)] = (float) rotation.get10(); //[dM3E_YX];
        m.f[M(1, 1)] = (float) rotation.get11(); //[dM3E_YY];
        m.f[M(1, 2)] = (float) rotation.get12(); //[dM3E_YZ];

        m.f[M(2, 0)] = (float) rotation.get20(); //[dM3E_ZX];
        m.f[M(2, 1)] = (float) rotation.get21(); //[dM3E_ZY];
        m.f[M(2, 2)] = (float) rotation.get22(); //[dM3E_ZZ];

        m.f[M(0, 3)] = (float) position.get0(); //[dV3E_X];
        m.f[M(1, 3)] = (float) position.get1(); //[dV3E_Y];
        m.f[M(2, 3)] = (float) position.get2(); //[dV3E_Z];
    }


    //    static inline
    //    void MakeMatrix(dxGeom *g, mat4f m) {
    //        const dVector3 & position = g -> buildUpdatedPosition();
    //        const dMatrix3 & rotation = g -> buildUpdatedRotation();
    //        MakeMatrix(position, rotation, m);
    //    }
    public static void MakeMatrix(DxGeom g, mat4f m) {
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

    /*extern */
    DTriMeshData dGeomTriMeshDataCreate() {
        //return new DxTriMeshData();
        return new DxGimpactData();
    }

    /*extern */
    void dGeomTriMeshDataDestroy(DTriMeshData g) {
        DxTriMeshData data = (DxTriMeshData) g;
        //delete data;
        data.DESTRUCTOR();
    }

    //    /*extern */
    //    //void dGeomTriMeshDataSet(DTriMeshData g, int dataId, void *pDataLocation) {
    //    void dGeomTriMeshDataSet(DTriMeshData g, DTriMesh.dTRIMESHDATA dataId, Void[] pDataLocation) {
    //        //dUASSERT(g, "The argument is not a trimesh data");
    //
    //        DxTriMeshData data = (DxTriMeshData) g;
    //
    //        switch (dataId) {
    //            case FACE_NORMALS: {
    //                data.assignNormals((const dReal *)pDataLocation);
    //                break;
    //            }
    //
    //            case USE_FLAGS: // Not used for GIMPACT
    //            {
    //                break;
    //            }
    //
    //            // case dTRIMESHDATA__MAX: -- To be located by Find in Files
    //            default: {
    //                dUASSERT(dataId.ordinal(), "invalid data type");
    //                break;
    //            }
    //        }
    //    }

    //static void *
    //geomTriMeshDataGet(dTriMeshDataID g, int dataId, size_t *pOutDataSize);

    // TZ the following methods are disabled, they don't appear to be used (or have ever been available in ode4j)
//    /*extern */
//    //void *
//    RefInt
//    dGeomTriMeshDataGet(DTriMeshData g, DTriMesh.dTRIMESHDATA dataId) {
//        return geomTriMeshDataGet(g, dataId, new RefInt()); //null);
//    }
//
//    /*extern */
//    //void *
//    //dGeomTriMeshDataGet2(DTriMeshData g, int dataId, size_t *pOutDataSize) {
//    RefInt
//    dGeomTriMeshDataGet2(DTriMeshData g, DTriMesh.dTRIMESHDATA dataId, RefInt pOutDataSize) {
//        return geomTriMeshDataGet(g, dataId, pOutDataSize);
//    }
//
//    //    static void *
//    //    geomTriMeshDataGet(dTriMeshDataID g, int dataId, size_t *pOutDataSize) {
//    static RefInt
//    geomTriMeshDataGet(DTriMeshData g, DTriMesh.dTRIMESHDATA dataId, RefInt pOutDataSize) {
//
//        //dUASSERT(g, "The argument is not a trimesh data");
//
//        final DxTriMeshData data = (DxTriMeshData) g;
//
//        //void *result = NULL;
//        RefInt result = null;
//
//        switch (dataId) {
//            case FACE_NORMALS: {
//                if (pOutDataSize != null) {
//                    pOutDataSize.set( data.calculateNormalsMemoryRequirement() );
//                }
//
//                //result = ( void *)data.retrieveNormals();
//                result = data.retrieveNormals();
//                break;
//            }
//
//            case USE_FLAGS: // Not not used for GIMPACT
//            {
//                if (pOutDataSize != null) {
//                    pOutDataSize.set(0);
//                }
//
//                break;
//            }
//
//            // case dTRIMESHDATA__MAX: -- To be located by Find in Files
//            default: {
//                if (pOutDataSize != null) {
//                    pOutDataSize.set(0);
//                }
//
//                dUASSERT(dataId.ordinal(), "invalid data type");
//                break;
//            }
//        }
//
//        return result;
//    }

    // TZ (2021-04-11) THis doesn't appear to be used (or have been available in earlier ode4j)
//    /*extern */
//    void dGeomTriMeshDataBuildSingle1(DTriMeshData g,
//        final float[] Vertices, int VertexStride, int VertexCount,
//        final int[] Indices, int IndexCount, int TriStride,
//        final Void[] Normals) {
//        //dUASSERT(g, "The argument is not a trimesh data");
//        dAASSERT(Vertices);
//        dAASSERT(Indices);
//
//        DxTriMeshData data = (DxTriMeshData) g;
//
//        data.buildData(Vertices, VertexStride, VertexCount, Indices, IndexCount, TriStride, Normals, true);
//    }
//
//    /*extern */
////    void dGeomTriMeshDataBuildDouble1(DTriMeshData g,
////        const void*Vertices, int VertexStride, int VertexCount,
////        const void*Indices, int IndexCount, int TriStride,
////        const void*Normals) {
//    void dGeomTriMeshDataBuildDouble1(DTriMeshData g,
//        final double[] Vertices, int VertexStride, int VertexCount,
//        final int[]Indices, int IndexCount, int TriStride,
//        final Void[] Normals) {
//        //dUASSERT(g, "The argument is not a trimesh data");
//        dAASSERT(Vertices);
//        dAASSERT(Indices);
//
//        DxTriMeshData data = (DxTriMeshData) g;
//
//        data.buildData(Vertices, VertexStride, VertexCount, Indices, IndexCount, TriStride, Normals, false);
//    }


    //////////////////////////////////////////////////////////////////////////

    /*extern */
    //dGeomID dCreateTriMesh(dSpaceID space, dTriMeshDataID Data, dTriCallback*Callback, dTriArrayCallback*ArrayCallback, dTriRayCallback*RayCallback) {
    //    DGeom dCreateTriMesh(DSpace space, DTriMeshData Data, DTriMesh.DTriCallback Callback,
    //                         DTriMesh.DTriArrayCallback ArrayCallback, DTriMesh.DTriRayCallback RayCallback) {
    //        //DxTriMesh mesh = new DxTriMesh(space, Data, Callback, ArrayCallback, RayCallback);
    //        DxTriMesh Geom;
    //        switch (OdeConfig.dTRIMESH_TYPE) {
    //            // TZ: is it correct to check for TriMeshDisabled here in the GimPact collider?
    //            //     This is new and wasn't in the original ode4j code or ODE code.
    //            case DISABLED: Geom = new DxTriMeshDisabled((DxSpace)space, (DxTriMeshData) Data); break;
    //            case GIMPACT: Geom = new DxGimpact((DxSpace)space, (DxGimpactData) Data, Callback, ArrayCallback, RayCallback); break;
    //            default: throw new IllegalArgumentException(OdeConfig.dTRIMESH_TYPE.name());
    //        }
    //        return Geom;
    //    }


    /*extern */
    //void dGeomTriMeshSetLastTransform(dGeomID g, const dMatrix4 last_trans) {
    void dGeomTriMeshSetLastTransform(DGeom g, Object last_trans) {
        //dAASSERT(g);
        //dUASSERT(g instanceof DTriMesh, "The geom is not a trimesh");
        throw new UnsupportedOperationException();
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