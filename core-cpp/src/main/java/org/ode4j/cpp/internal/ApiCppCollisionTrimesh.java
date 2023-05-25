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
package org.ode4j.cpp.internal;

import org.ode4j.math.DVector3;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DTriMesh;
import org.ode4j.ode.DTriMeshData;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.DTriMesh.DTriCallback;
import org.ode4j.ode.DTriMesh.DTriRayCallback;
import org.ode4j.ode.internal.cpp4j.java.DoubleArray;
import org.ode4j.ode.internal.cpp4j.java.RefInt;


/**
 * TriMesh code by Erwin de Vries.
 * <p>
 * Trimesh data.
 * This is where the actual vertexdata (pointers), and BV tree is stored.
 * Vertices should be single precision!
 * This should be more sophisticated, so that the user can easyly implement
 * another collision library, but this is a lot of work, and also costs some
 * performance because some data has to be copied.
 */
public class ApiCppCollisionTrimesh extends ApiCppTimer {

    ///**
    // * Data storage for triangle meshes.
    // */
    //struct dxTriMeshData;
    //typedef struct dxTriMeshData* dTriMeshDataID;

    /**
     * These don't make much sense now, but they will later when we add more
     * features.
     *
     * @return trimesh data
     */
    //ODE_API
    public static DTriMeshData dGeomTriMeshDataCreate() {
        return OdeHelper.createTriMeshData();
    }

    //ODE_API
    public static void dGeomTriMeshDataDestroy(DTriMeshData g) {
        g.destroy();
    }


    /**
     * @deprecated TZ: find a better name.
     */
    @Deprecated
    enum TRIMESH1 {TRIMESH_FACE_NORMALS}

    //ODE_API
    void dGeomTriMeshDataSet(DTriMeshData g, int data_id, Object in_data) {
        throw new UnsupportedOperationException();
    }

    //ODE_API
    Object dGeomTriMeshDataGet(DTriMeshData g, int data_id) {
        throw new UnsupportedOperationException();
    }


    /**
     * We need to set the last transform after each time step for
     * accurate collision response. These functions get and set that transform.
     * It is stored per geom instance, rather than per dTriMeshData.
     *
     * @param g          trimesh
     * @param last_trans last transform
     */
    //ODE_API
    //void dGeomTriMeshSetLastTransform( DGeom g, DMatrix4 last_trans ) {
    //	throw new UnsupportedOperationException();
    //}
    public static void dGeomTriMeshSetLastTransform(DTriMesh g, DoubleArray last_trans) {
        throw new UnsupportedOperationException();
    }

    //ODE_API
    double[] dGeomTriMeshGetLastTransform(DGeom g) {
        throw new UnsupportedOperationException();
    }

    /**
     * Build a TriMesh data object with single precision vertex data.
     *
     * @param g            g
     * @param Vertices     vertices
     * @param VertexStride stride
     * @param VertexCount  count
     * @param Indices      indices
     * @param IndexCount   count
     * @param TriStride    tri stride
     */
    //ODE_API
    //	 void dGeomTriMeshDataBuildSingle(dTriMeshData g,
    //             final void* Vertices, int VertexStride, int VertexCount,
    //             final void* Indices, int IndexCount, int TriStride);
    public static void dGeomTriMeshDataBuildSingle(DTriMeshData g, final float[] Vertices, int VertexStride,
                                                   int VertexCount, final int[] Indices, int IndexCount,
                                                   int TriStride) {
        g.build(Vertices, //VertexStride, VertexCount,
                Indices);//, IndexCount, TriStride);
    }

    /**
     * same again with a normals array (used as trimesh-trimesh optimization)
     */
    //ODE_API
    //	 void dGeomTriMeshDataBuildSingle1(dTriMeshData g,
    //                                  final void* Vertices, int VertexStride, int VertexCount,
    //                                  final void* Indices, int IndexCount, int TriStride,
    //                                  final void* Normals) {
    void dGeomTriMeshDataBuildSingle1(DTriMeshData g, final double[] Vertices, int VertexStride, int VertexCount,
                                      final int[] Indices, int IndexCount, int TriStride, final int[] Normals) {
        throw new UnsupportedOperationException();
    }

    /**
     * Build a TriMesh data object with double precision vertex data.
     */
    //ODE_API
    void dGeomTriMeshDataBuildDouble(DTriMeshData g,
                                     //                                 final void* Vertices,  int VertexStride, int
                                     //                                 VertexCount,
                                     //                                 final void* Indices, int IndexCount, int
                                     //                                 TriStride) {
                                     final double[] Vertices, int VertexStride, int VertexCount, final int[] Indices,
                                     int IndexCount, int TriStride) {
        throw new UnsupportedOperationException();
    }

    /**
     * same again with a normals array (used as trimesh-trimesh optimization)
     */
    //ODE_API
    //	 void dGeomTriMeshDataBuildDouble1(dTriMeshData g,
    //                                  final void* Vertices,  int VertexStride, int VertexCount,
    //                                  final void* Indices, int IndexCount, int TriStride,
    //                                  final void* Normals) {
    void dGeomTriMeshDataBuildDouble1(DTriMeshData g, final double[] Vertices, int VertexStride, int VertexCount,
                                      final int[] Indices, int IndexCount, int TriStride, final int[] Normals) {
        throw new UnsupportedOperationException();
    }

    /**
     * Simple build. Single/double precision based on dSINGLE/dDOUBLE!
     */
    //ODE_API
    //	 void dGeomTriMeshDataBuildSimple(dTriMeshData g,
    //                                 final double* Vertices, int VertexCount,
    //                                 final dTriIndex* Indices, int IndexCount) {
    void dGeomTriMeshDataBuildSimple(DTriMeshData g, final double[] Vertices, int VertexCount, final int[] Indices,
                                     int IndexCount) {
        throw new UnsupportedOperationException();
    }

    /**
     * same again with a normals array (used as trimesh-trimesh optimization)
     */
    //ODE_API
    //	 void dGeomTriMeshDataBuildSimple1(dTriMeshData g,
    //                                  final double* Vertices, int VertexCount,
    //                                  final dTriIndex* Indices, int IndexCount,
    //                                  final int* Normals) {
    void dGeomTriMeshDataBuildSimple1(DTriMeshData g, final double[] Vertices, int VertexCount, final int[] Indices,
                                      int IndexCount, final int[] Normals) {
        throw new UnsupportedOperationException();
    }

    /**
     * Preprocess the trimesh data to remove mark unnecessary edges and vertices
     */
    //ODE_API
    void dGeomTriMeshDataPreprocess(DTriMeshData g) {
        throw new UnsupportedOperationException();
    }

    /**
     * Get and set the internal preprocessed trimesh data buffer, for loading and saving
     */
    //ODE_API
    //void dGeomTriMeshDataGetBuffer(dTriMeshData g, unsigned char** buf, int* bufLen) {
    void dGeomTriMeshDataGetBuffer(DTriMeshData g, byte[][] buf, RefInt bufLen) {
        throw new UnsupportedOperationException();
    }

    //ODE_API
    //void dGeomTriMeshDataSetBuffer(dTriMeshData g, unsigned char* buf) {
    void dGeomTriMeshDataSetBuffer(DTriMeshData g, byte[] buf) {
        throw new UnsupportedOperationException();
    }


    ///**
    // * Per triangle callback. Allows the user to say if he wants a collision with
    // * a particular triangle.
    // */
    ////typedef int dTriCallback(dGeom TriMesh, dGeom RefObject, int TriangleIndex);
    //	 interface dTriCallback {
    //		 int call(dGeom TriMesh, dGeom RefObject, int TriangleIndex);
    //	 }
    //ODE_API
    void dGeomTriMeshSetCallback(DGeom g, DTriCallback Callback) {
        throw new UnsupportedOperationException();
    }

    //ODE_API
    DTriCallback dGeomTriMeshGetCallback(DGeom g) {
        throw new UnsupportedOperationException();
    }

    ///**
    // * Per object callback. Allows the user to get the list of triangles in 1
    // * shot. Maybe we should remove this one.
    // */
    ////typedef void dTriArrayCallback(dGeom TriMesh, dGeom RefObject, final int* TriIndices, int TriCount);
    //interface dTriArrayCallback {
    //	void call(dGeom TriMesh, dGeom RefObject, final int[] TriIndices, int TriCount);
    //}
    //ODE_API
    @Deprecated
    void dGeomTriMeshSetArrayCallback(DGeom g, org.ode4j.ode.DTriMesh.DTriArrayCallback ArrayCallback) {
        throw new UnsupportedOperationException();
    }

    //ODE_API
    @Deprecated
    org.ode4j.ode.DTriMesh.DTriArrayCallback dGeomTriMeshGetArrayCallback(DGeom g) {
        throw new UnsupportedOperationException();
    }

    ///**
    // * Ray callback.
    // * Allows the user to say if a ray collides with a triangle on barycentric
    // * coords. The user can for example sample a texture with alpha transparency
    // * to determine if a collision should occur.
    // */
    ////typedef int dTriRayCallback(dGeom TriMesh, dGeom Ray, int TriangleIndex, double u, double v);
    //interface dTriRayCallback {
    //	int call(dGeom TriMesh, dGeom Ray, int TriangleIndex, double u, double v);
    //}
    //ODE_API
    void dGeomTriMeshSetRayCallback(DTriMesh g, DTriRayCallback Callback) {
        throw new UnsupportedOperationException();
    }

    //ODE_API
    DTriRayCallback dGeomTriMeshGetRayCallback(DTriMesh g) {
        throw new UnsupportedOperationException();
    }

    /**
     * Triangle merging callback.
     * Allows the user to generate a fake triangle index for a new contact generated
     * from merging of two other contacts. That index could later be used by the
     * user to determine attributes of original triangles used as sources for a
     * merged contact.
     */
    //typedef int dTriTriMergeCallback(dGeomID TriMesh, int FirstTriangleIndex, int SecondTriangleIndex);
    private interface DTriTriMergeCallback {
        int callback(DTriMesh TriMesh, int FirstTriangleIndex, int SecondTriangleIndex);
    }

    //ODE_API
    void dGeomTriMeshSetTriMergeCallback(DTriMesh g, DTriTriMergeCallback Callback) {
        throw new UnsupportedOperationException();
    }

    //ODE_API
    DTriTriMergeCallback dGeomTriMeshGetTriMergeCallback(DTriMesh g) {
        throw new UnsupportedOperationException();
    }

    /**
     * Trimesh class
     * Construction. Callbacks are optional.
     *
     * @param space         space
     * @param Data          data
     * @param Callback      callback
     * @param ArrayCallback callback
     * @param RayCallback   callback
     * @return trimesh
     */
    //ODE_API
    @SuppressWarnings("deprecation")
    public static DTriMesh dCreateTriMesh(DSpace space, DTriMeshData Data, DTriCallback Callback,
                                          org.ode4j.ode.DTriMesh.DTriArrayCallback ArrayCallback,
                                          DTriRayCallback RayCallback) {
        return OdeHelper.createTriMesh(space, Data, Callback, ArrayCallback, RayCallback);
    }

    //ODE_API
    void dGeomTriMeshSetData(DTriMesh g, DTriMeshData Data) {
        g.setData(Data);
    }

    //ODE_API
    DTriMeshData dGeomTriMeshGetData(DTriMesh g) {
        return g.getTrimeshData();
    }


    // enable/disable/check temporal coherence
    //ODE_API
    void dGeomTriMeshEnableTC(DTriMesh g, Class<? extends DGeom> geomClass, boolean enable) {
        g.enableTC(geomClass, enable);
    }

    //ODE_API
    boolean dGeomTriMeshIsTCEnabled(DTriMesh g, Class<? extends DGeom> geomClass) {
        return g.isTCEnabled(geomClass);
    }

    /*
     * Clears the internal temporal coherence caches. When a geom has its
     * collision checked with a trimesh once, data is stored inside the trimesh.
     * With large worlds with lots of seperate objects this list could get huge.
     * We should be able to do this automagically.
     */
    //ODE_API
    void dGeomTriMeshClearTCCache(DTriMesh g) {
        throw new UnsupportedOperationException();
    }


    /*
     * returns the TriMeshDataID
     */
    //ODE_API
    DTriMeshData dGeomTriMeshGetTriMeshDataID(DTriMesh g) {
        throw new UnsupportedOperationException();
    }

    /*
     * Gets a triangle.
     */
    //ODE_API
    //void dGeomTriMeshGetTriangle(dGeom g, int Index, dVector3* v0, dVector3* v1, dVector3* v2) {
    void dGeomTriMeshGetTriangle(DTriMesh g, int Index, DVector3 v0, DVector3 v1, DVector3 v2) {
        throw new UnsupportedOperationException();
    }

    /*
     * Gets the point on the requested triangle and the given barycentric
     * coordinates.
     */
    //ODE_API
    void dGeomTriMeshGetPoint(DTriMesh g, int Index, double u, double v, DVector3 Out) {
        g.getPoint(Index, u, v, Out);
    }

	/*

This is how the strided data works:

struct StridedVertex{
	dVector3 Vertex;
	// Userdata
};
int VertexStride = sizeof(StridedVertex);

struct StridedTri{
	int Indices[3];
	// Userdata
};
int TriStride = sizeof(StridedTri);

	 */


    //ODE_API
    int dGeomTriMeshGetTriangleCount(DTriMesh g) {
        return g.getTriangleCount();
    }

    //ODE_API
    void dGeomTriMeshDataUpdate(DTriMeshData g) {
        g.update();
    }

    protected ApiCppCollisionTrimesh() {}
}

