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
package org.ode4j.ode;


/**
 * TriMesh code by Erwin de Vries.
 *
 * Trimesh data.
 * This is where the actual vertexdata (pointers), and BV tree is stored.
 * Vertices should be single precision!
 * This should be more sophisticated, so that the user can easily implement
 * another collision library, but this is a lot of work, and also costs some
 * performance because some data has to be copied.
 */
public interface DTriMesh extends DGeom {
	
	/**
	 * Ray callback.
	 * Allows the user to say if a ray collides with a triangle on barycentric
	 * coords. The user can for example sample a texture with alpha transparency
	 * to determine if a collision should occur.
	 */
	//typedef int dTriRayCallback(dGeom TriMesh, dGeom Ray, int TriangleIndex, double u, double v);
	interface DTriRayCallback {
		int call(DGeom TriMesh, DGeom Ray, int TriangleIndex, double u, double v);
	}
	
	
	/**
	 * Per object callback. Allows the user to get the list of triangles in 1
	 * shot. Maybe we should remove this one.
	 */
	//typedef void dTriArrayCallback(dGeom TriMesh, dGeom RefObject, final int* TriIndices, int TriCount);
	interface DTriArrayCallback {
		void call(DGeom TriMesh, DGeom RefObject, final int[] TriIndices, int TriCount);
	}
	
	
	/**
	 * Per triangle callback. Allows the user to say if he wants a collision with
	 * a particular triangle.
	 */
	//typedef int dTriCallback(dGeom TriMesh, dGeom RefObject, int TriangleIndex);
	interface DTriCallback {
		int call(DGeom TriMesh, DGeom RefObject, int TriangleIndex);
	}
	
	
	
	
	
//	void dGeomTriMeshDataSet(DTriMeshData g, int data_id, Object in_data);
//	Object dGeomTriMeshDataGet(DTriMeshData g, int data_id);


//The following is not ported to Java because it is not supported by GIMPACT (TZ).
//	/**
//	 * We need to set the last transform after each time step for 
//	 * accurate collision response. These functions get and set that transform.
//	 * It is stored per geom instance, rather than per dTriMeshData.
//	 */
//	void setLastTransform( DMatrix4 last_trans );
//	void setLastTransform( double[] last_trans );
//	double[] dGeomTriMeshGetLastTransform();



//	///**
//	// * Per triangle callback. Allows the user to say if he wants a collision with
//	// * a particular triangle.
//	// */
//	////typedef int dTriCallback(dGeom TriMesh, dGeom RefObject, int TriangleIndex);
//	//	 interface dTriCallback {
//	//		 int call(dGeom TriMesh, dGeom RefObject, int TriangleIndex);
//	//	 }
//	//ODE_API 
//	void dGeomTriMeshSetCallback(DGeom g, dTriCallback Callback) {
//		throw new UnsupportedOperationException();
//	}
//	//ODE_API 
//	dTriCallback dGeomTriMeshGetCallback(DGeom g) {
//		throw new UnsupportedOperationException();
//	}
//
//	///**
//	// * Per object callback. Allows the user to get the list of triangles in 1
//	// * shot. Maybe we should remove this one.
//	// */
//	////typedef void dTriArrayCallback(dGeom TriMesh, dGeom RefObject, final int* TriIndices, int TriCount);
//	//interface dTriArrayCallback {
//	//	void call(dGeom TriMesh, dGeom RefObject, final int[] TriIndices, int TriCount);
//	//}
//	//ODE_API 
//	void dGeomTriMeshSetArrayCallback(DGeom g, dTriArrayCallback ArrayCallback) {
//		throw new UnsupportedOperationException();
//	}
//	//ODE_API 
//	dTriArrayCallback dGeomTriMeshGetArrayCallback(DGeom g) {
//		throw new UnsupportedOperationException();
//	}
//
//	///**
//	// * Ray callback.
//	// * Allows the user to say if a ray collides with a triangle on barycentric
//	// * coords. The user can for example sample a texture with alpha transparency
//	// * to determine if a collision should occur.
//	// */
//	////typedef int dTriRayCallback(dGeom TriMesh, dGeom Ray, int TriangleIndex, double u, double v);
//	//interface dTriRayCallback {
//	//	int call(dGeom TriMesh, dGeom Ray, int TriangleIndex, double u, double v);
//	//}
//	//ODE_API 
//	void dGeomTriMeshSetRayCallback(DGeom g, dTriRayCallback Callback) {
//		throw new UnsupportedOperationException();
//	}
//	//ODE_API 
//	dTriRayCallback dGeomTriMeshGetRayCallback(DGeom g) {
//		throw new UnsupportedOperationException();
//	}

	/**
	 * Triangle merging callback.
	 * Allows the user to generate a fake triangle index for a new contact generated
	 * from merging of two other contacts. That index could later be used by the 
	 * user to determine attributes of original triangles used as sources for a 
	 * merged contact.
	 */
	//typedef int dTriTriMergeCallback(dGeomID TriMesh, int FirstTriangleIndex, int SecondTriangleIndex);
	interface DTriTriMergeCallback {
		int call(DGeom TriMesh, int FirstTriangleIndex, int SecondTriangleIndex);
	}
	
	//ODE_API 
	void setTriMergeCallback(DTriTriMergeCallback Callback);
	
	//ODE_API 
	DTriTriMergeCallback getTriMergeCallback();

//	/**
//	 * Trimesh class
//	 * Construction. Callbacks are optional.
//	 */
//	//ODE_API 
//	public static DTriMesh dCreateTriMesh(DSpace space, DTriMeshData Data, dTriCallback Callback, 
//			dTriArrayCallback ArrayCallback, dTriRayCallback RayCallback) {
//		return OdeHelper.createTriMesh(space, Data, 
//				Callback, ArrayCallback, RayCallback);
//	}
//
//	//ODE_API 
//	void dGeomTriMeshSetData(DGeom g, DTriMeshData Data) {
//		throw new UnsupportedOperationException();
//	}
//	//ODE_API 
//	DTriMeshData dGeomTriMeshGetData(DGeom g) {
//		throw new UnsupportedOperationException();
//	}
//
//
	/** Enable/disable temporal coherence. */
	//ODE_API 
	//void dGeomTriMeshEnableTC(DGeom g, int geomClass, int enable) {
	void enableTC(Class<? extends DGeom> cls, boolean b);


	/** Check temporal coherence. */
	//ODE_API 
	//	int dGeomTriMeshIsTCEnabled(DGeom g, int geomClass) {
	boolean isTCEnabled(Class<? extends DGeom> cls);
	

	/**
	 * Clears the internal temporal coherence caches. When a geom has its
	 * collision checked with a trimesh once, data is stored inside the trimesh.
	 * With large worlds with lots of seperate objects this list could get huge.
	 * We should be able to do this automagically.
	 */
	//ODE_API 
	void clearTCCache(DTriMesh g);


//	/**
//	 * returns the TriMeshDataID
//	 */
//	//ODE_API 
//	DTriMeshData dGeomTriMeshGetTriMeshDataID(DTriMesh g);
//
//	/**
//	 * Gets a triangle.
//	 */
//	//ODE_API 
//	//void dGeomTriMeshGetTriangle(dGeom g, int Index, dVector3* v0, dVector3* v1, dVector3* v2) {
//	void dGeomTriMeshGetTriangle(DTriMesh g, int Index, DVector3 v0, DVector3 v1, DVector3 v2);
//
//	/**
//	 * Gets the point on the requested triangle and the given barycentric
//	 * coordinates.
//	 */
//	//ODE_API 
//	void getPoint(DTriMesh g, int Index, double u, double v, DVector3 Out);
//
//	/*
//
//This is how the strided data works:
//
//struct StridedVertex{
//	dVector3 Vertex;
//	// Userdata
//};
//int VertexStride = sizeof(StridedVertex);
//
//struct StridedTri{
//	int Indices[3];
//	// Userdata
//};
//int TriStride = sizeof(StridedTri);
//
//	 */
//
//
//	//ODE_API 
//	int getTriangleCount (DGeom g);
//
//	//ODE_API 
//	void dGeomTriMeshDataUpdate(DTriMeshData g) {
//		throw new UnsupportedOperationException();
//	}

}
