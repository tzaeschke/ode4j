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
package org.ode4j.ode.internal;

import static org.ode4j.ode.internal.DxGimpactCollision.GIM_AABB_COPY;
import static org.ode4j.ode.internal.DxGimpactCollision.MakeMatrix;

import java.nio.channels.UnsupportedAddressTypeException;

import org.ode4j.math.DVector3;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DCapsule;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.DTriMesh;
import org.ode4j.ode.DTriMeshData;
import org.ode4j.ode.internal.gimpact.GimGeometry;
import org.ode4j.ode.internal.gimpact.GimTrimesh;
import org.ode4j.ode.internal.gimpact.GimGeometry.mat4f;
import org.ode4j.ode.internal.gimpact.GimGeometry.vec3f;

/**
 * 
 *
 * @author Tilmann Zaeschke
 */
public class DxGimpact extends DxTriMesh {

	DxGimpactData _Data;

	GimTrimesh m_collision_trimesh;


	//void dGeomTriMeshSetLastTransform( DMatrix4 last_trans ) { //stub
	void dGeomTriMeshSetLastTransform( Object last_trans ) { //stub
		throw new UnsupportedAddressTypeException();
	}

	//	DMatrix4 dGeomTriMeshGetLastTransform() {
	Object dGeomTriMeshGetLastTransform() {
		throw new UnsupportedAddressTypeException();
		//		return null; // stub
	}


	//Not implemented in ODE 0.11.1
	//	//	void*  dGeomTriMeshDataGet(dTriMeshDataID g, int data_id) {
	////	    dUASSERT(g, "argument not trimesh data");
	////		return NULL; // stub
	////	}
	//	@Override
	//	public Object dGeomTriMeshDataGet(DTriMeshData g, int dataId) {
	//		throw new UnsupportedAddressTypeException();
	////		return null;
	//	}
	//
	////	void dGeomTriMeshDataSet(dTriMeshDataID g, int data_id, void* in_data) { //stub
	////	}
	//	@Override
	//	public void dGeomTriMeshDataSet(DTriMeshData g, int dataId, Object inData) {
	//		throw new UnsupportedAddressTypeException();
	//	}

	@Override
	public int FetchTriangleCount() {
		return DxGimpactCollision.FetchTriangleCount(this);
	}

	@Override
	public void FetchTransformedTriangle(int i, DVector3[] v) {
		DxGimpactCollision.FetchTransformedTriangle(this, i, v);
	}



	// Trimesh

	//dxTriMesh::dxTriMesh(dSpaceID Space, dTriMeshDataID Data) : dxGeom(Space, 1){
	DxGimpact(DxSpace Space, DxGimpactData Data) { 
		super(Space);
		_Data = Data;
		type = dTriMeshClass;

		Callback = null;
		ArrayCallback = null;
		RayCallback = null;
		TriMergeCallback = null; // Not initialized in dCreateTriMesh

//		for (int i=0; i < m_buffer_managers.length; i++) { 
//			m_buffer_managers[i] = new GBUFFER_MANAGER_DATA(); 
//		}
//		GimBufferArray.gim_init_buffer_managers(m_buffer_managers);

		dGeomTriMeshSetData(Data);

		/* TC has speed/space 'issues' that don't make it a clear
		   win by default on spheres/boxes. */
		this.doSphereTC = true;
		this.doBoxTC = true;
		this.doCapsuleTC = true;
	}

	@Override
	//dxTriMesh::~dxTriMesh(){
	public void DESTRUCTOR(){

		//Terminate Trimesh
		m_collision_trimesh.gim_trimesh_destroy();

//		GimBufferArray.gim_terminate_buffer_managers(m_buffer_managers);
		super.DESTRUCTOR();
	}


	@Override
	//void dxTriMesh::ClearTCCache(){
	void ClearTCCache(){

	}


//	boolean dxTriMesh::controlGeometry(int controlClass, int controlCode, void *dataValue, int *dataSize)
//	{
//	    return dxGeom::controlGeometry(controlClass, controlCode, dataValue, dataSize);
//	}


	@Override
	//void dxTriMesh::computeAABB()
	void computeAABB()
	{
		//update trimesh transform
		mat4f transform = new mat4f();
		GimGeometry.IDENTIFY_MATRIX_4X4(transform);
		MakeMatrix(this, transform);
		m_collision_trimesh.gim_trimesh_set_tranform(transform);

		//Update trimesh boxes
		m_collision_trimesh.gim_trimesh_update();

		GIM_AABB_COPY( m_collision_trimesh.getAabbSet().getGlobalBound(), _aabb );
	}



	//	dGeomID dCreateTriMesh(dSpaceID space,
	//			       dTriMeshDataID Data,
	//			       dTriCallback* Callback,
	//			       dTriArrayCallback* ArrayCallback,
	//			       dTriRayCallback* RayCallback)
	//	DGeom dCreateTriMesh(DSpace space,
	//		       DTriMeshData Data,
	//		       DTriCallback Callback,
	//		       DTriArrayCallback ArrayCallback,
	//		       DTriRayCallback RayCallback)
	//	{
	//	    DxTriMesh Geom = new DxGimpact(space, Data);
	//	    Geom.Callback = Callback;
	//	    Geom.ArrayCallback = ArrayCallback;
	//	    Geom.RayCallback = RayCallback;
	//
	//	    return Geom;
	//	}

	//	void dGeomTriMeshSetCallback(dGeomID g, dTriCallback* Callback)
	void dGeomTriMeshSetCallback(DTriCallback Callback)
	{
		//dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
		this.Callback = Callback;
	}

	//dTriCallback* dGeomTriMeshGetCallback(dGeomID g)
	DTriCallback dGeomTriMeshGetCallback()
	{
		//dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
		return Callback;
	}

	//void dGeomTriMeshSetArrayCallback(dGeomID g, dTriArrayCallback* ArrayCallback)
	void dGeomTriMeshSetArrayCallback(DTriArrayCallback ArrayCallback)
	{
		//dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
		this.ArrayCallback = ArrayCallback;
	}

	//dTriArrayCallback* dGeomTriMeshGetArrayCallback(dGeomID g)
	DTriArrayCallback dGeomTriMeshGetArrayCallback()
	{
		//dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
		return ArrayCallback;
	}

	//void dGeomTriMeshSetRayCallback(dGeomID g, dTriRayCallback* Callback)
	void dGeomTriMeshSetRayCallback(DTriRayCallback Callback)
	{
		//dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
		this.RayCallback = Callback;
	}

	//dTriRayCallback* dGeomTriMeshGetRayCallback(dGeomID g)
	DTriRayCallback dGeomTriMeshGetRayCallback()
	{
		//dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
		return RayCallback;
	}

	//void dGeomTriMeshSetTriMergeCallback(dGeomID g, dTriTriMergeCallback* Callback)
	void dGeomTriMeshSetTriMergeCallback(DTriTriMergeCallback Callback)
	{
		//dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
		this.TriMergeCallback = Callback;
	}

	//dTriTriMergeCallback* dGeomTriMeshGetTriMergeCallback(dGeomID g)
	DTriTriMergeCallback dGeomTriMeshGetTriMergeCallback()
	{
		//dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");	
		return TriMergeCallback;
	}

	//void dGeomTriMeshSetData(dGeomID g, dTriMeshDataID Data)
	void dGeomTriMeshSetData(DTriMeshData Data)
	{
		this._Data = (DxGimpactData) Data;
		// I changed my data -- I know nothing about my own AABB anymore.
		//this._gflags |= (GEOM_DIRTY|GEOM_AABB_BAD);
		setFlagDirtyAndBad();

		// ******************************************************************************************
		// GIMPACT only supports stride 12, so we need to catch the error early.
		//		Common.dUASSERT
		//		(
		//		  //Data.m_VertexStride == 3*sizeof(float) && Data->m_TriStride == 3*sizeof(int),
		//				_Data.m_VertexStride == 3 && _Data.m_TriStride == 3,
		//	          "Gimpact trimesh only supports a stride of 3 float/int\n" +
		//		  "This means that you cannot use dGeomTriMeshDataBuildSimple() with Gimpact.\n" +
		//		  "Change the stride, or use Opcode trimeshes instead.\n"
		//		);
		// ******************************************************************************************

		//Create trimesh
		if ( _Data.getDataRef() != null ) {
			this.m_collision_trimesh = GimTrimesh.gim_trimesh_create_from_data
			(
					//this.m_buffer_managers,
					//this.m_collision_trimesh,		// gimpact mesh
					//( vec3f *)(&Data.m_Vertices[0]),	// vertices
					_Data.getDataRef(),//[0],	// vertices
					//_Data.m_VertexCount,		// nr of verts
					false,					// copy verts?
					//( GUINT32 *)(&Data.m_Indices[0]),	// indices
					_Data.getIndexRef(),//[0],	// indices
					//_Data.m_TriangleCount*3,		// nr of indices
					false,					// copy indices?
					true				// transformed reply
			);
		}
	}

	DTriMeshData dGeomTriMeshGetData()
	{
		//dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
		return _Data;
	}



	void dGeomTriMeshEnableTC(Class<? extends DGeom> geomClass, boolean enable)
	{
		if (geomClass.equals(DSphere.class)) {
			doSphereTC = enable;
		} else if (geomClass.equals(DBox.class)) {
			doBoxTC = enable;
		} else if (geomClass.equals(DCapsule.class)) {
			doCapsuleTC = enable;
		} else {
			throw new UnsupportedOperationException();
		}
	}

	boolean dGeomTriMeshIsTCEnabled(Class<? extends DGeom> geomClass)
	{
		if (geomClass.equals(DSphere.class)) {
			return doSphereTC;
		} else if (geomClass.equals(DBox.class)) {
			return doBoxTC;
		} else if (geomClass.equals(DCapsule.class)) {
			return doCapsuleTC;
		}
		//return 0;
		throw new UnsupportedOperationException();
	}

	void dGeomTriMeshClearTCCache(){
		ClearTCCache();
	}

	/**
	 * returns the TriMeshDataID
	 */
	DTriMeshData
	dGeomTriMeshGetTriMeshDataID()
	{
		return _Data;
	}

	// Getting data
	//void dGeomTriMeshGetTriangle(dGeomID g, int Index, dVector3* v0, dVector3* v1, dVector3* v2)
	void dGeomTriMeshGetTriangle(int Index, DVector3 v0, DVector3 v1, DVector3 v2)
	{
		//TZ: simplified version ? Does not allow null-arguments
		DVector3[] v = { v0, v1, v2 };
		FetchTransformedTriangle(Index, v);

		//		// Redirect null vectors to dummy storage
		//		DVector3[] v = { new DVector3(), new DVector3(), new DVector3() };
		//
		//		FetchTransformedTriangle(Index, v);
		//
		//		if (v0!=null){
		//			v0.set(v[0]);
		//		}
		//		if (v1!=null){
		//			v1.set(v[1]);
		//		}
		//		if (v2!=null){
		//			v2.set(v[2]);
		//		}
	}

	//void dGeomTriMeshGetPoint(dGeomID g, int Index, dReal u, dReal v, dVector3 Out){
	void dGeomTriMeshGetPoint(int Index, double u, double v, DVector3 Out){
		vec3f[] dv = { new vec3f(), new vec3f(), new vec3f() };
		m_collision_trimesh.gim_trimesh_locks_work_data();	
		m_collision_trimesh.gim_trimesh_get_triangle_vertices(Index, dv[0],dv[1],dv[2]);
		DxGimpactCollision.GetPointFromBarycentric(dv, u, v, Out);
		m_collision_trimesh.gim_trimesh_unlocks_work_data();
	}

	int dGeomTriMeshGetTriangleCount ()
	{
		return FetchTriangleCount();
	}


	@Override
	public void enableTC(Class<? extends DGeom> cls, boolean enable) {
		dGeomTriMeshEnableTC(cls, enable);
	}


	@Override
	public boolean isTCEnabled(Class<? extends DGeom> cls) {
		return dGeomTriMeshIsTCEnabled(cls);
	}


	@Override
	public void clearTCCache(DTriMesh g) {
		dGeomTriMeshClearTCCache();
	}


	@Override
	public DTriTriMergeCallback getTriMergeCallback() {
		return dGeomTriMeshGetTriMergeCallback();
	}


	@Override
	public void setTriMergeCallback(DTriTriMergeCallback Callback) {
		dGeomTriMeshSetTriMergeCallback(Callback);
	}
}
