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

import java.nio.channels.UnsupportedAddressTypeException;

import org.ode4j.math.DVector3;
import org.ode4j.ode.*;
import org.ode4j.ode.internal.gimpact.GimGeometry;
import org.ode4j.ode.internal.trimesh.DxTriMesh;
import org.ode4j.ode.internal.trimesh.DxTriMeshData;

/**
 * 
 *
 * @author Tilmann Zaeschke
 */
public class DxGimpact extends DxTriMesh {

	// see super-class
	// DxGimpactData _Data;

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


	// Trimesh

	//dxTriMesh::dxTriMesh(dSpaceID Space, dTriMeshDataID Data) : dxGeom(Space, 1){
	@SuppressWarnings("deprecation")
	public DxGimpact(DxSpace Space, DxGimpactData Data, DTriMesh.DTriCallback Callback,
					 DTriMesh.DTriArrayCallback ArrayCallback, DTriMesh.DTriRayCallback RayCallback) {
		super(Space, Data, Callback, ArrayCallback, RayCallback);
		type = dTriMeshClass;
	}

	@Override
	//dxTriMesh::~dxTriMesh(){
	public void DESTRUCTOR(){
		super.DESTRUCTOR();
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
		return ((DxGimpactData)getMeshData()).getEdgeAngle(triangle, edge);
	}

	@Override
	protected void MakeMatrix(GimGeometry.mat4f transform) {
		CollisionTrimeshGimpact.MakeMatrix(this, transform);
	}
}
