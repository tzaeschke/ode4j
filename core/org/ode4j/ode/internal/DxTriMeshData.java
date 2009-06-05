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

import java.nio.ByteBuffer;

import org.cpp4j.java.RefInt;
import org.ode4j.ode.OdeConfig;
import org.ode4j.ode.DTriMeshData;

public abstract class DxTriMeshData implements DTriMeshData {
	//TZ from "collision_trimesh_internal.h"
	
    /** Array of flags for which edges and verts should be used on each triangle */
	//    enum UseFlags
	//    {
        protected static final int kEdge0 = 0x1;
        protected static final int kEdge1 = 0x2;
        protected static final int kEdge2 = 0x4;
        protected static final int kVert0 = 0x8;
        protected static final int kVert1 = 0x10;
        protected static final int kVert2 = 0x20;

        protected static final int kUseAll = 0xFF;
//    };

    /* Setup the UseFlags array */
    abstract void Preprocess();
    /* For when app changes the vertices */
    abstract void UpdateData();

    public static DTriMeshData dGeomTriMeshDataCreate() {
		switch (OdeConfig.dTRIMESH_TYPE) {
		case DISABLED: return new DxTriMeshDisabled.dxTriMeshDisabledData();
		default: throw new IllegalArgumentException(OdeConfig.dTRIMESH_TYPE.name());
		}
    }
    
	public void dGeomTriMeshDataBuildSingle(
			final double[] Vertices, int VertexStride, int VertexCount, 
			final int[] Indices, int IndexCount, int TriStride) { }
	public void dGeomTriMeshDataBuildSingle(
			final float[] Vertices, int VertexStride, int VertexCount, 
			final int[] Indices, int IndexCount, int TriStride) { }

	void dGeomTriMeshDataBuildSingle1(DTriMeshData g,
			final double[] Vertices, int VertexStride, int VertexCount, 
			final int[] Indices, int IndexCount, int TriStride,
			final int[] Normals) { }

	void dGeomTriMeshDataBuildDouble(DTriMeshData g, 
			final double[] Vertices,  int VertexStride, int VertexCount, 
			final int[] Indices, int IndexCount, int TriStride) { }

	void dGeomTriMeshDataBuildDouble1(DTriMeshData g, 
			final double[] Vertices,  int VertexStride, int VertexCount, 
			final int[] Indices, int IndexCount, int TriStride,
			final int[] Normals) { }

	void dGeomTriMeshDataBuildSimple(DTriMeshData g,
			final double[] Vertices, int VertexCount,
			final int[] Indices, int IndexCount) { }

	void dGeomTriMeshDataBuildSimple1(DTriMeshData g,
			final double[] Vertices, int VertexCount,
			final int[] Indices, int IndexCount,
			final int[] Normals) { }

	void dGeomTriMeshDataPreprocess(DTriMeshData g) { }

	//void dGeomTriMeshDataGetBuffer(dTriMeshData g, unsigned char** buf, int* bufLen) { *buf = NULL; *bufLen=0; }
	//void dGeomTriMeshDataSetBuffer(dTriMeshData g, unsigned char* buf) {}
	void dGeomTriMeshDataGetBuffer(DTriMeshData g, ByteBuffer buf, RefInt bufLen) { buf.clear(); bufLen.set(0); }
	void dGeomTriMeshDataSetBuffer(DTriMeshData g, ByteBuffer buf) {}

}
