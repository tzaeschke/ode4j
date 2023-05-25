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

import org.ode4j.math.DVector3;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DTriMeshData;
import org.ode4j.ode.internal.gimpact.GimGeometry;
import org.ode4j.ode.internal.trimesh.DxTriMesh;
import org.ode4j.ode.internal.trimesh.DxTriMeshData;

public class DxTriMeshDisabled extends DxTriMesh {

	public static class dxTriMeshDisabledData extends DxTriMeshData {
		public dxTriMeshDisabledData() {}
		@Override
		public boolean preprocess() {
			return true;
		}

		@Override
		public void update() {

		}

		@Override
		public void updateData() {
			throw new UnsupportedOperationException();
		}

//		@Override
//		public void buildSingle(double[] Vertices,
//				int VertexStride, int VertexCount, int[] Indices,
//				int IndexCount, int TriStride) {
//			dGeomTriMeshDataBuildSingle(Vertices, VertexStride, VertexCount,
//					Indices, IndexCount, TriStride);
//		}
//
//		@Override
////		public void buildSingle(float[] Vertices,
////				int VertexStride, int VertexCount, int[] Indices,
////				int IndexCount, int TriStride) {
////			dGeomTriMeshDataBuildSingle(Vertices, VertexStride, VertexCount,
////					Indices, IndexCount, TriStride);
////		}
//		public void buildSingle(float[] Vertices,
//				int[] Indices
//				) {
//			dGeomTriMeshDataBuildSingle(Vertices, Indices);
//		}

		@Override
		public void destroy() {
			//
		}

		@Override
		public boolean preprocess2(int buildRequestFlags, long[] requestExtraData) {
			return true;
		}

		@Override
		public void build(float[] Vertices, int[] Indices) {
			//dGeomTriMeshDataBuildSingle(Vertices, Indices);
		}
	}
	
	// #if !dTRIMESH_ENABLED

	//#include "collision_util.h"
	//#include "collision_trimesh_internal.h"

	//dxTriMesh::dxTriMesh(dSpaceID Space, dTriMeshDataID Data) : dxGeom(Space, 1){ type = dTriMeshClass; }
	public DxTriMeshDisabled(DxSpace space, DxTriMeshData data) //: dxGeom(Space, 1)
	{ 
		super(space, data, null, null, null);
		type = dTriMeshClass;
	}
	//dxTriMesh::~dxTriMesh(){}

//	boolean controlGeometry(int controlClass, int controlCode, Object[][] dataValue, RefInt dataSize)
//	{
//	    return DxGeom.controlGeometry(controlClass, controlCode, dataValue, dataSize);
//	}

	//void dxTriMesh::computeAABB() { dSetZero (aabb,6); }
	@Override
    protected void computeAABB() {
		_aabb.setZero();
	}

	//TODO TZ report: identity IS NOT an indentity matrix!
//	private static final DMatrix4 identity = new DMatrix4 (
//			( 0.0 ), ( 0.0 ), ( 0.0 ), ( 0.0 ),
//			( 0.0 ), ( 0.0 ), ( 0.0 ), ( 0.0 ),
//			( 0.0 ), ( 0.0 ), ( 0.0 ), ( 0.0 ),
//			( 0.0 ), ( 0.0 ), ( 0.0 ), ( 0.0 ) );

	// Stub functions for trimesh calls

	DTriMeshData dGeomTriMeshDataCreate() { return null; }
	void dGeomTriMeshDataDestroy() {}

	//	public void dGeomTriMeshDataSet(DTriMeshData g, int data_id, Object in_data) {}
	//	public Object dGeomTriMeshDataGet(DTriMeshData g, int data_id) { return null; }

	//ODE_API 
	//void dGeomTriMeshSetLastTransform( DGeom g, DMatrix4 last_trans ) {}
	//ODE_API 
	//dReal* dGeomTriMeshGetLastTransform( dGeom g ) { return identity; }
	//DMatrix4 dGeomTriMeshGetLastTransform( DGeom g ) { return null; } //TZ TODO?: return identity; }

	@Override
	//void dGeomTriMeshSetData(DGeom g, DTriMeshData Data) {}
	public void setTrimeshData(DTriMeshData Data) {}
	@Override
	//DTriMeshData dGeomTriMeshGetData(DGeom g) { return null; }
	public DTriMeshData getTrimeshData() { return null; }


	void dGeomTriMeshSetCallback(DGeom g, DTriCallback Callback) { }
	DTriCallback dGeomTriMeshGetCallback(DGeom g) { return null; }

	@Deprecated
	void dGeomTriMeshSetArrayCallback(DGeom g, DTriArrayCallback ArrayCallback) { }
	@Deprecated
	DTriArrayCallback dGeomTriMeshGetArrayCallback(DGeom g) { return null; }

	void dGeomTriMeshSetRayCallback(DGeom g, DTriRayCallback Callback) { }
	DTriRayCallback dGeomTriMeshGetRayCallback(DGeom g) { return null; }


	void dGeomTriMeshEnableTC(DGeom g, int geomClass, int enable) {}
	int dGeomTriMeshIsTCEnabled(DGeom g, int geomClass) { return 0; }
	void dGeomTriMeshClearTCCache(DGeom g) {}

	DTriMeshData dGeomTriMeshGetTriMeshDataID(DGeom g) { return null; }

	public void dGeomTriMeshGetTriangle(int Index, DVector3 v0, DVector3 v1, DVector3 v2) {}

	int dGeomTriMeshGetTriangleCount (DGeom g) { return 0; }
	void dGeomTriMeshDataUpdate(DTriMeshData g) {}

	@Override
	public void enableTC(Class<? extends DGeom> cls, boolean b) {
		// TODO Auto-generated method stub
	}

	@Override
	public boolean isTCEnabled(Class<? extends DGeom> cls) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void clearTCCache() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public DTriMeshData getTriMeshData() {
		return null;
	}

	@Override
	public void getTriangle(int Index, DVector3 v0, DVector3 v1, DVector3 v2) {

	}

	@Override
	public int getTriangleCount() {
		return 0;
	}

	@Override
	public DTriTriMergeCallback getTriMergeCallback() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void setTriMergeCallback(DTriTriMergeCallback Callback) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public float getEdgeAngle(int triangle, int edge) {
		return (float) (Math.PI * 2);
	}

	//#endif // !dTRIMESH_ENABLED

	@Override
	protected void MakeMatrix(GimGeometry.mat4f transform) {
		CollisionTrimeshGimpact.MakeMatrix(this, transform);
	}
}

