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
		case GIMPACT: return new DxGimpactData();
		default: throw new IllegalArgumentException(OdeConfig.dTRIMESH_TYPE.name());
		}
    }
//	public abstract void dGeomTriMeshDataDestroy();
//   
//	public abstract void dGeomTriMeshDataBuildSingle(
//			final double[] Vertices, int VertexStride, int VertexCount, 
//			final int[] Indices, int IndexCount, int TriStride);
////	public void dGeomTriMeshDataBuildSingle(
////			final float[] Vertices, int VertexStride, int VertexCount, 
////			final int[] Indices, int IndexCount, int TriStride) { }
//	public abstract void dGeomTriMeshDataBuildSingle(
//			final float[] Vertices, final int[] Indices);
//
//	abstract void dGeomTriMeshDataBuildSingle1(
//			final double[] Vertices, int VertexStride, int VertexCount, 
//			final int[] Indices, int IndexCount, int TriStride,
//			final int[] Normals);
//
//	abstract void dGeomTriMeshDataBuildDouble(DTriMeshData g, 
//			final double[] Vertices,  int VertexStride, int VertexCount, 
//			final int[] Indices, int IndexCount, int TriStride);
//
//	abstract void dGeomTriMeshDataBuildDouble1( 
//			final double[] Vertices,  int VertexStride, int VertexCount, 
//			final int[] Indices, int IndexCount, int TriStride,
//			final int[] Normals);
//
//	abstract void dGeomTriMeshDataBuildSimple(DTriMeshData g,
//			final double[] Vertices, int VertexCount,
//			final int[] Indices, int IndexCount);
//
//	abstract void dGeomTriMeshDataBuildSimple1(DTriMeshData g,
//			final double[] Vertices, int VertexCount,
//			final int[] Indices, int IndexCount,
//			final int[] Normals);
//
//	abstract void dGeomTriMeshDataPreprocess(DTriMeshData g);
//
//	//void dGeomTriMeshDataGetBuffer(dTriMeshData g, unsigned char** buf, int* bufLen) { *buf = NULL; *bufLen=0; }
//	//void dGeomTriMeshDataSetBuffer(dTriMeshData g, unsigned char* buf) {}
//	abstract void dGeomTriMeshDataGetBuffer(DTriMeshData g, ByteBuffer buf, RefInt bufLen);
//	abstract void dGeomTriMeshDataSetBuffer(DTriMeshData g, ByteBuffer buf);

}
