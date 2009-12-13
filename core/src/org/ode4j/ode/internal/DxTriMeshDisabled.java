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
package org.ode4j.ode.internal;

import org.ode4j.math.DVector3;
import org.ode4j.ode.DAABB;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DTriMeshData;
import org.ode4j.ode.internal.Common.DMatrix4;

class DxTriMeshDisabled extends DxTriMesh {

	static class dxTriMeshDisabledData extends DxTriMeshData {
		@Override
		void Preprocess() {
			throw new UnsupportedOperationException();
		}

		@Override
		void UpdateData() {
			throw new UnsupportedOperationException();
		}

		@Override
		public void buildSingle(double[] Vertices,
				int VertexStride, int VertexCount, int[] Indices,
				int IndexCount, int TriStride) {
			dGeomTriMeshDataBuildSingle(Vertices, VertexStride, VertexCount,
					Indices, IndexCount, TriStride);
		}

		@Override
		public void buildSingle(float[] Vertices,
				int VertexStride, int VertexCount, int[] Indices,
				int IndexCount, int TriStride) {
			dGeomTriMeshDataBuildSingle(Vertices, VertexStride, VertexCount,
					Indices, IndexCount, TriStride);
		}

		@Override
		public void destroy() {
			//
		}
		
	}
	
	//TODO #if !dTRIMESH_ENABLED

	//#include "collision_util.h"
	//#include "collision_trimesh_internal.h"

	//dxTriMesh::dxTriMesh(dSpaceID Space, dTriMeshDataID Data) : dxGeom(Space, 1){ type = dTriMeshClass; }
	public DxTriMeshDisabled(DxSpace space, DxTriMeshData data) //: dxGeom(Space, 1)
	{ 
		super(space, data);
		type = dTriMeshClass;
	}
	//dxTriMesh::~dxTriMesh(){}

	//int dxTriMesh::AABBTest(dxGeom* g, dReal aabb[6]) { return 0; }
	@Override
	boolean AABBTest(DAABB aabb) {
		return false;
	}
	//void dxTriMesh::computeAABB() { dSetZero (aabb,6); }
	@Override
	void computeAABB() {
		_aabb.setZero();
	}

	//TODO TZ report: identity IS NOT an indentity matrix!
	private static final DMatrix4 identity = new DMatrix4 (
			( 0.0 ), ( 0.0 ), ( 0.0 ), ( 0.0 ),
			( 0.0 ), ( 0.0 ), ( 0.0 ), ( 0.0 ),
			( 0.0 ), ( 0.0 ), ( 0.0 ), ( 0.0 ),
			( 0.0 ), ( 0.0 ), ( 0.0 ), ( 0.0 ) );

	// Stub functions for trimesh calls

	DTriMeshData dGeomTriMeshDataCreate() { return null; }
	void dGeomTriMeshDataDestroy() {}

	public void dGeomTriMeshDataSet(DTriMeshData g, int data_id, Object in_data) {}
	public Object dGeomTriMeshDataGet(DTriMeshData g, int data_id) { return null; }

	//ODE_API 
	void dGeomTriMeshSetLastTransform( DGeom g, DMatrix4 last_trans ) {}
	//ODE_API 
	//dReal* dGeomTriMeshGetLastTransform( dGeom g ) { return identity; }
	DMatrix4 dGeomTriMeshGetLastTransform( DGeom g ) { return identity; }

	void dGeomTriMeshSetData(DGeom g, DTriMeshData Data) {}
	DTriMeshData dGeomTriMeshGetData(DGeom g) { return null; }


	void dGeomTriMeshSetCallback(DGeom g, dTriCallback Callback) { }
	dTriCallback dGeomTriMeshGetCallback(DGeom g) { return null; }

	void dGeomTriMeshSetArrayCallback(DGeom g, dTriArrayCallback ArrayCallback) { }
	dTriArrayCallback dGeomTriMeshGetArrayCallback(DGeom g) { return null; }

	void dGeomTriMeshSetRayCallback(DGeom g, dTriRayCallback Callback) { }
	dTriRayCallback dGeomTriMeshGetRayCallback(DGeom g) { return null; }


	void dGeomTriMeshEnableTC(DGeom g, int geomClass, int enable) {}
	int dGeomTriMeshIsTCEnabled(DGeom g, int geomClass) { return 0; }
	void dGeomTriMeshClearTCCache(DGeom g) {}

	DTriMeshData dGeomTriMeshGetTriMeshDataID(DGeom g) { return null; }

	int dGeomTriMeshGetTriangleCount (DGeom g) { return 0; }
	void dGeomTriMeshDataUpdate(DTriMeshData g) {}

	@Override
	void ClearTCCache() {
		// TODO Auto-generated method stub

	}

	@Override
	public void FetchTransformedTriangle(int i, DVector3[] v) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public int FetchTriangleCount() {
		// TODO Auto-generated method stub
		return 0;
	}

	//#endif // !dTRIMESH_ENABLED
}

