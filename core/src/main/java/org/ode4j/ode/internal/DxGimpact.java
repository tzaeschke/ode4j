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

import static org.ode4j.ode.internal.Common.dUASSERT;
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
import org.ode4j.ode.internal.trimesh.DxTriMesh;
import org.ode4j.ode.internal.trimesh.DxTriMeshData;

/**
 * 
 *
 * @author Tilmann Zaeschke
 */
public class DxGimpact extends DxTriMesh {

	DxGimpactData _Data;

	//GimTrimesh m_collision_trimesh;

	//void dGeomTriMeshSetLastTransform( DMatrix4 last_trans ) { //stub
	void dGeomTriMeshSetLastTransform( Object last_trans ) { //stub
		throw new UnsupportedAddressTypeException();
	}

	//	DMatrix4 dGeomTriMeshGetLastTransform() {
	Object dGeomTriMeshGetLastTransform() {
		throw new UnsupportedAddressTypeException();
		//		return null; // stub
	}


	// TODO TZ remove unused
	@Override
	public int FetchTriangleCount() {
		return DxGimpactCollision.FetchTriangleCount(this);
	}

	@Override
	public void FetchTransformedTriangle(int i, DVector3 out0, DVector3 out1, DVector3 out2) {
		DxGimpactCollision.FetchTransformedTriangle(this, i, out0, out1, out2);
	}



	// Trimesh

	// TODO TZ_CHECK
	// TODO TZ_CHECK
	// TODO TZ_CHECK
	// TODO TZ_CHECK
	// TODO TZ_CHECK
	// TODO TZ_CHECK
	// TODO TZ_CHECK The following are DxTrimesh:: methods, they should be move to DxTrimesh!!!
	// TODO TZ_CHECK This also helpw with the callbacks and booleans in the constructor +getters + setters
	// TODO TZ_CHECK
	// TODO TZ_CHECK

		//dxTriMesh::dxTriMesh(dSpaceID Space, dTriMeshDataID Data) : dxGeom(Space, 1){
        public DxGimpact(DxSpace Space, DxGimpactData Data) {
		// TODO TZ-CHECK I inserted 'null' here, are we using Data?
		// TODO TZ-CHECK I inserted 'null' here, are we using the Callbacks?
		super(Space, Data, null, null, null);
		_Data = Data;
		type = dTriMeshClass;

		setCallback(null);
		setArrayCallback(null);
		setRayCallback(null);
		setTriMergeCallback(null); // Not initialized in dCreateTriMesh

//		for (int i=0; i < m_buffer_managers.length; i++) { 
//			m_buffer_managers[i] = new GBUFFER_MANAGER_DATA(); 
//		}
//		GimBufferArray.gim_init_buffer_managers(m_buffer_managers);

		dGeomTriMeshSetData(Data);
	}

	@Override
	//dxTriMesh::~dxTriMesh(){
	public void DESTRUCTOR(){

		//Terminate Trimesh
		m_collision_trimesh().gim_trimesh_destroy();

//		GimBufferArray.gim_terminate_buffer_managers(m_buffer_managers);
		super.DESTRUCTOR();
	}


//	boolean dxTriMesh::controlGeometry(int controlClass, int controlCode, void *dataValue, int *dataSize)
//	{
//	    return dxGeom::controlGeometry(controlClass, controlCode, dataValue, dataSize);
//	}


	@Override
    protected
        //void dxTriMesh::computeAABB()
	void computeAABB()
	{
		//update trimesh transform
		mat4f transform = new mat4f();
		GimGeometry.IDENTIFY_MATRIX_4X4(transform);
		MakeMatrix(this, transform);
		m_collision_trimesh().gim_trimesh_set_tranform(transform);

		//Update trimesh boxes
		m_collision_trimesh().gim_trimesh_update();

		GIM_AABB_COPY( m_collision_trimesh().getAabbSet().getGlobalBound(), _aabb );
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

	/*extern ODE_API */
	//void dGeomTriMeshEnableTC(dGeomID g, int geomClass, int enable)
	void dGeomTriMeshEnableTC(Class<? extends DGeom> geomClass, boolean enable)
	{
		//dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

		//DxTriMesh mesh = (DxTriMesh) g;

		//dxTriMesh::TRIMESHTC tc = g_asiMeshTCGeomClasses.Decode(geomClass);
		//int tc = g_asiMeshTCGeomClasses.Decode(geomClass);

		//if (g_asiMeshTCGeomClasses.IsValidDecode(tc))
		//{
		//	assignDoTC(tc, enable);
		//}
		if (geomClass.equals(DSphere.class)) {
			assignDoTC(TRIMESHTC.TTC_SPHERE, enable);
		} else if (geomClass.equals(DBox.class)) {
			assignDoTC(TRIMESHTC.TTC_BOX, enable);
		} else if (geomClass.equals(DCapsule.class)) {
			assignDoTC(TRIMESHTC.TTC_CAPSULE, enable);
		} else {
			throw new UnsupportedOperationException(geomClass.getName());
		}
	}

	/*extern ODE_API */
	//int dGeomTriMeshIsTCEnabled(dGeomID g, int geomClass)
	boolean dGeomTriMeshIsTCEnabled(Class<? extends DGeom> geomClass)
	{
		//dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

		//DxTriMesh mesh = (DxTriMesh) g;

		//dxTriMesh::TRIMESHTC tc = g_asiMeshTCGeomClasses.Decode(geomClass);
		//int tc = g_asiMeshTCGeomClasses.Decode(geomClass);

		//boolean result = g_asiMeshTCGeomClasses.IsValidDecode(tc)
		//		&& mesh.retrieveDoTC(tc);
		if (geomClass.equals(DSphere.class)) {
			return retrieveDoTC(TRIMESHTC.TTC_SPHERE);
		} else if (geomClass.equals(DBox.class)) {
			return retrieveDoTC(TRIMESHTC.TTC_BOX);
		} else if (geomClass.equals(DCapsule.class)) {
			return retrieveDoTC(TRIMESHTC.TTC_CAPSULE);
		}
		throw new UnsupportedOperationException(geomClass.getName());
	}

	/**
	 * returns the TriMeshDataID
	 */
	/*extern ODE_API */
	//dTriMeshDataID dGeomTriMeshGetTriMeshDataID(dGeomID g)
	DTriMeshData dGeomTriMeshGetTriMeshData()
	{
		//dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

		//DxTriMesh mesh = (DxTriMesh) g;
		return retrieveMeshData();
	}

	/*extern ODE_API */
	//void dGeomTriMeshClearTCCache(dGeomID g)
	void dGeomTriMeshClearTCCache()
	{
		//dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

		//DxTriMesh mesh = (DxTriMesh) g;
		// TZ: This does nothing -> we also need to avoid infinite loop here, the name is the same as the Java API
		//clearTCCache();
	}

	/*extern ODE_API */
	//int dGeomTriMeshGetTriangleCount(dGeomID g)
	public int dGeomTriMeshGetTriangleCount()
	{
		//dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

		//DxTriMesh mesh = (DxTriMesh) g;
		int result = getMeshTriangleCount();
		return result;
	}

	/*extern ODE_API */
	//void dGeomTriMeshGetTriangle(dGeomID g, int index, dVector3 *v0/*=NULL*/, dVector3 *v1/*=NULL*/, dVector3 *v2/*=NULL*/)
	public void dGeomTriMeshGetTriangle(int index, DVector3 v0/*=NULL*/, DVector3 v1/*=NULL*/, DVector3 v2/*=NULL*/)
	{
		//dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
		dUASSERT(v0 != null || v1 != null || v2 != null, "A meaningless call");

		//DxTriMesh mesh = (DxTriMesh) g;
		fetchMeshTransformedTriangle(v0, v1, v2, index);
	}

	/*extern ODE_API */
	//void dGeomTriMeshSetData(dGeomID g, dTriMeshDataID Data)
	void dGeomTriMeshSetData(DTriMeshData Data)
	{
		//dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

		//DxTriMesh mesh = (DxTriMesh) g;
		assignMeshData((DxTriMeshData) Data);
	}

	/*extern ODE_API */
	//dTriMeshDataID dGeomTriMeshGetData(dGeomID g)
	public DTriMeshData dGeomTriMeshGetData()
	{
		//dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

		//DxTriMesh mesh = (DxTriMesh) g;
		return retrieveMeshData();
	}


	//void dGeomTriMeshGetPoint(dGeomID g, int Index, dReal u, dReal v, dVector3 Out){
	void dGeomTriMeshGetPoint(int Index, double u, double v, DVector3 Out){
		//vec3f[] dv = { new vec3f(), new vec3f(), new vec3f() };
		DVector3[] dv = { new DVector3(), new DVector3(), new DVector3() };
		m_collision_trimesh().gim_trimesh_locks_work_data();
		m_collision_trimesh().gim_trimesh_get_triangle_vertices(Index, dv[0], dv[1], dv[2]);
		DxGimpactCollision.GetPointFromBarycentric(dv, u, v, Out);
		m_collision_trimesh().gim_trimesh_unlocks_work_data();
	}


	@Override
	public void enableTC(Class<? extends DGeom> cls, boolean enable) {
		dGeomTriMeshEnableTC(cls, enable);
	}


	@Override
	public boolean isTCEnabled(Class<? extends DGeom> cls) {
		return dGeomTriMeshIsTCEnabled(cls);
	}


//	@Override
//	public DTriTriMergeCallback getTriMergeCallback() {
//		return dGeomTriMeshGetTriMergeCallback();
//	}
//
//
//	@Override
//	public void setTriMergeCallback(DTriTriMergeCallback Callback) {
//		dGeomTriMeshSetTriMergeCallback(Callback);
//	}


	@Override
	/*extern ODE_API */
	//dTriMeshDataID dGeomTriMeshGetTriMeshDataID(dGeomID g)
	public DTriMeshData getTriMeshData() {
		return dGeomTriMeshGetTriMeshData();
	}

	@Override
	/*extern ODE_API */
	//void dGeomTriMeshClearTCCache(dGeomID g)
	public void clearTCCache() {
		dGeomTriMeshClearTCCache();
	}

	@Override
	/*extern ODE_API */
	//int dGeomTriMeshGetTriangleCount(dGeomID g)
	public int getTriangleCount() {
		return dGeomTriMeshGetTriangleCount();
	}

	@Override
	/*extern ODE_API */
	//void dGeomTriMeshGetTriangle(dGeomID g, int index, dVector3 *v0/*=NULL*/, dVector3 *v1/*=NULL*/, dVector3 *v2/*=NULL*/)
	public void getTriangle(int index, DVector3 v0/*=NULL*/, DVector3 v1/*=NULL*/, DVector3 v2/*=NULL*/) {
		dGeomTriMeshGetTriangle(index, v0, v1, v2);
	}

	@Override
	public void setTrimeshData(DTriMeshData Data) {
		dGeomTriMeshSetData(Data);
	}

	@Override
	/*extern ODE_API */
	//dTriMeshDataID dGeomTriMeshGetData(dGeomID g)
	public DTriMeshData getTrimeshData() {
		return dGeomTriMeshGetData();
	}


	@Override
	public float getEdgeAngle(int triangle, int edge) {
		return _Data.getEdgeAngle(triangle, edge);
	}
}
