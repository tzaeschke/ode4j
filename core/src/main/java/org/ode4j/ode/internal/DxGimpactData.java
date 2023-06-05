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

import static org.ode4j.ode.internal.Common.dIASSERT;

import org.ode4j.ode.internal.trimesh.DxTriMeshData;

import java.util.ArrayList;

/**
 *
 * Data for Gimpact trimeshes.
 */
public class DxGimpactData extends DxTriMeshData {

//	private float[] m_Vertices;//const char* m_Vertices;
////	int m_VertexStride;   //see docs below, GIMPACT does not support strides other than 3 (TZ)
////	int m_VertexCount;
//	private int[] m_Indices;//const char* m_Indices;
////	int m_TriangleCount;
////	int m_TriStride;
////	boolean m_single;
//	private float[] m_Angles;

    public DxGimpactData()//dxTriMeshData()
	{
//		m_Vertices=null;
////		m_VertexStride = 12;
////		m_VertexCount = 0;
//		m_Indices = null;
////		m_TriangleCount = 0;
////		m_TriStride = 12;
////		m_single = true;
	}
    
    
    float[] getDataRef() {
		return super.retrieveVertexInstances();
    }

    int[] getIndexRef() {
		return super.retrieveTriangleVertexIndices();
    }

//    void Build(const void* Vertices, int VertexStride, int VertexCount,
//	       const void* Indices, int IndexCount, int TriStride,
//	       const void* Normals,
//	      bool Single)
//    void Build(final float[] Vertices, int VertexStride, int VertexCount,
// 	       final int[] Indices, int IndexCount, int TriStride,
// 	       final float[] Normals,
// 	      boolean Single)
//	{
//		dIASSERT(Vertices!=null);
//		dIASSERT(Indices!=null);
// 		dIASSERT(VertexStride!=0);
// 		dIASSERT(TriStride!=0);
// 		dIASSERT(IndexCount!=0);
//		m_Vertices = Vertices;
//		m_VertexStride = VertexStride;
//		m_VertexCount = VertexCount;
//		m_Indices = Indices;
//		m_TriangleCount = IndexCount/3;
//		m_TriStride = TriStride;
//		m_single = Single;
//	}
    
//    @Override
//    void build(final float[] Vertices, //int VertexStride,
//  	       final int[] Indices, //int TriStride,
//  	       final float[] Normals)// ,  	      boolean Single)
// 	{
// 		dIASSERT(Vertices!=null);
// 		dIASSERT(Indices!=null);
////  		dIASSERT(VertexStride!=0);
////  		dIASSERT(TriStride!=0);
//  		//dIASSERT(IndexCount!=0);
// 		m_Vertices = Vertices;
//// 		m_VertexStride = VertexStride;
//// 		m_VertexCount = Vertices.length;
// 		m_Indices = Indices;
//// 		m_TriangleCount = Indices.length/3;
//// 		m_TriStride = TriStride;
//// 		m_single = Single;
// 	}

    @Override
    public void build(final float[] Vertices,
					  final int[] Indices) {
		dIASSERT(Vertices!=null);
		dIASSERT(Indices!=null);
		super.buildData(Vertices, Indices, null);
		//m_Vertices = Vertices;
		//m_Indices = Indices;
		// check();
	}
//	void GetVertex(int i, DVector4 Out)
//	{
//		//TZ commented out, special treatment not required (?)
////		if(m_single)
////		{
//			//const float * fverts = (const float * )(m_Vertices + m_VertexStride*i);
//			int p = i*3;//m_VertexStride;
////			Out[0] = fverts[0];
////			Out[1] = fverts[1];
////			Out[2] = fverts[2];
////			Out[3] = 1.0f;
//			Out.set(m_Vertices[p], m_Vertices[p+1], m_Vertices[p+2], 1.0f);
////		}
////		else
////		{
////			const double * dverts = (const double * )(m_Vertices + m_VertexStride*i);
////			Out[0] = (float)dverts[0];
////			Out[1] = (float)dverts[1];
////			Out[2] = (float)dverts[2];
////			Out[3] = 1.0f;
////
////		}
//	}

	//void GetTriIndices(unsigned int itriangle, unsigned int triindices[3])
//	void GetTriIndices(int itriangle, int[] triindices)
//	{
//		//const unsigned int * ind = (const unsigned int * )(m_Indices + m_TriStride*itriangle);
//		int p = itriangle*3;//m_TriStride;
//		triindices[0] = m_Indices[p+0];
//		triindices[1] = m_Indices[p+1];
//		triindices[2] = m_Indices[p+2];
//	}
//#endif  // dTRIMESH_GIMPACT


//	void dGeomTriMeshDataBuildSingle1(dTriMeshDataID g,
//            const void* Vertices, int VertexStride, int VertexCount,
//            const void* Indices, int IndexCount, int TriStride,
//            const void* Normals)
//	void dGeomTriMeshDataBuildSingle1(final float[] Vertices, int VertexStride, int VertexCount,
//			final int[] Indices, int IndexCount, int TriStride,
//			final float[] Normals)
//	{	
//		//dIASSERT(Vertices);
//		//dIASSERT(Indices);
//
//		Build(Vertices, VertexStride, VertexCount,
//				Indices, IndexCount, TriStride,
//				Normals,
//				true);
//	}
//	void dGeomTriMeshDataBuildSingle1(final float[] Vertices,
//			final int[] Indices,
//			final float[] Normals)
//	{	
//		//dIASSERT(Vertices);
//		//dIASSERT(Indices);
//
//		build(Vertices, //VertexStride,
//				Indices, //TriStride,
//				Normals,
//				true);
//	}


//	void dGeomTriMeshDataBuildSingle(dTriMeshDataID g,
//			const void* Vertices, int VertexStride, int VertexCount,
//			const void* Indices, int IndexCount, int TriStride)
//	@Override
//	public void dGeomTriMeshDataBuildSingle(
//			final float[] Vertices, int VertexStride, int VertexCount,
//			final int[] Indices, int IndexCount, int TriStride)
//	{
//		dGeomTriMeshDataBuildSingle1(Vertices, VertexStride, VertexCount,
//				Indices, IndexCount, TriStride, null);
//	}
//	@Override
//	public void dGeomTriMeshDataBuildSingle(
//			final float[] Vertices, final int[] Indices)
//	{
//		dGeomTriMeshDataBuildSingle1(Vertices, Indices, null);
//	}


//	void dGeomTriMeshDataBuildDouble1(dTriMeshDataID g,
//			const void* Vertices, int VertexStride, int VertexCount,
//			const void* Indices, int IndexCount, int TriStride,
//			const void* Normals)
//	void dGeomTriMeshDataBuildDouble1(
//			final double[] Vertices, int VertexStride, int VertexCount,
//			final int[] Indices, int IndexCount, int TriStride,
//			final double[] Normals)
//	void dGeomTriMeshDataBuildDouble1(
//			final double[] Vertices, //int VertexStride, int VertexCount,
//			final int[] Indices, //int IndexCount, int TriStride,
//			final double[] Normals)
//	{
//		Build(Vertices, //VertexStride, VertexCount,
//				Indices, //IndexCount, TriStride,
//				Normals,
//				false);
//	}


//	void dGeomTriMeshDataBuildDouble(dTriMeshDataID g,
//			const void* Vertices, int VertexStride, int VertexCount,
//			const void* Indices, int IndexCount, int TriStride) {
//	void dGeomTriMeshDataBuildDouble(
//			final double[] Vertices, int VertexStride, int VertexCount,
//			final int[] Indices, int IndexCount, int TriStride) {
//		dGeomTriMeshDataBuildDouble1(Vertices, VertexStride, VertexCount,
//				Indices, IndexCount, TriStride, null);
//	}


//	void dGeomTriMeshDataBuildSimple1(dTriMeshDataID g,
//			const dReal* Vertices, int VertexCount,
//			const dTriIndex* Indices, int IndexCount,
//			const int* Normals){
//	void dGeomTriMeshDataBuildSimple1(
//			final float[] Vertices, int VertexCount,
//			final int[] Indices, int IndexCount,
//			final int[] Normals){
//		if (single) {//#ifdef dSINGLE
//		dGeomTriMeshDataBuildSingle1(  //TODO why 4? TZ
//				Vertices, 4,// * sizeof(dReal), 
//				VertexCount,
//				Indices, IndexCount, 3,// * sizeof(dTriIndex),
//				Normals);
//		} else { //#else
//			dGeomTriMeshDataBuildDouble1( Vertices, 4,// * sizeof(dReal),   TODO why 4? TZ 
//					VertexCount,
//					Indices, IndexCount, 3,// * sizeof(unsigned int),
//					Normals);
//		}//#endif
//	}


//	void dGeomTriMeshDataBuildSimple(dTriMeshDataID g,
//			const dReal* Vertices, int VertexCount,
//			const dTriIndex* Indices, int IndexCount) {
//	void dGeomTriMeshDataBuildSimple(
//			final float[] Vertices, int VertexCount,
//			final int[] Indices, int IndexCount) {
//		dGeomTriMeshDataBuildSimple1(
//				Vertices, VertexCount, Indices, IndexCount,
//				null);//(const int*)NULL);
//	}


	//	//void dGeomTriMeshDataGetBuffer(dTriMeshDataID g, unsigned char** buf, int* bufLen)
	//	void dGeomTriMeshDataGetBuffer(Ref<Object> buf, RefInt bufLen)
	//	{
	//		buf.r = null;
	//		bufLen.i = 0;
	//		throw new UnsupportedOperationException();
	//	}
	//
	//	//void dGeomTriMeshDataSetBuffer(dTriMeshDataID g, unsigned char* buf)
	//	void dGeomTriMeshDataSetBuffer(Ref<Object> buf)
	//	{
	//		//g->UseFlags = buf;
	//		throw new UnsupportedOperationException();
	//	}


	/*extern ODE_API */
	void dGeomTriMeshDataUpdate()
	{
		//dUASSERT(g, "The argument is not a trimesh data");

		//DxTriMeshData *data = g;
		//data.updateData();
		//((DxTriMeshData)g).updateData();
		updateData();
	}

	/*extern ODE_API */
	// void dGeomTriMeshDataUpdate()
	public void update() {
		dGeomTriMeshDataUpdate();
	}


//	@Override
//	public void buildSingle(double[] Vertices, int VertexStride,
//			int VertexCount, int[] Indices, int IndexCount, int TriStride) {
//		dGeomTriMeshDataBuildSingle(Vertices, VertexStride, VertexCount, Indices, IndexCount, TriStride);
//	}

//	@Override
//	public void buildSingle(float[] Vertices, int VertexStride,
//			int VertexCount, int[] Indices, int IndexCount, int TriStride) {
//		dGeomTriMeshDataBuildSingle(Vertices, VertexStride, VertexCount, Indices, IndexCount, TriStride);
//	}
//	@Override
//	public void buildSingle(float[] Vertices, int[] Indices) {
//		dGeomTriMeshDataBuildSingle(Vertices, Indices);
//	}

	@Override
	public void destroy() {
		dGeomTriMeshDataDestroy();
	}

	private void dGeomTriMeshDataDestroy() {
		//Nothing to do
	}

//	public void dGeomTriMeshDataBuildSingle(
//			final double[] Vertices, int VertexStride, int VertexCount, 
//			final int[] Indices, int IndexCount, int TriStride) { }
////	public void dGeomTriMeshDataBuildSingle(
////			final float[] Vertices, int VertexStride, int VertexCount, 
////			final int[] Indices, int IndexCount, int TriStride) { }
//	public void dGeomTriMeshDataBuildSingle(
//			final float[] Vertices, final int[] Indices) { }
//
//	void dGeomTriMeshDataBuildSingle1(
//			final double[] Vertices, int VertexStride, int VertexCount, 
//			final int[] Indices, int IndexCount, int TriStride,
//			final int[] Normals) { }
//
//	void dGeomTriMeshDataBuildDouble(DTriMeshData g, 
//			final double[] Vertices,  int VertexStride, int VertexCount, 
//			final int[] Indices, int IndexCount, int TriStride) { }
//
//	void dGeomTriMeshDataBuildDouble1( 
//			final double[] Vertices,  int VertexStride, int VertexCount, 
//			final int[] Indices, int IndexCount, int TriStride,
//			final int[] Normals) { }
//
//	void dGeomTriMeshDataBuildSimple(DTriMeshData g,
//			final double[] Vertices, int VertexCount,
//			final int[] Indices, int IndexCount) { }
//
//	void dGeomTriMeshDataBuildSimple1(DTriMeshData g,
//			final double[] Vertices, int VertexCount,
//			final int[] Indices, int IndexCount,
//			final int[] Normals) { }
//
//	void dGeomTriMeshDataPreprocess(DTriMeshData g) { }
//
//	//void dGeomTriMeshDataGetBuffer(dTriMeshData g, unsigned char** buf, int* bufLen) { *buf = NULL; *bufLen=0; }
//	//void dGeomTriMeshDataSetBuffer(dTriMeshData g, unsigned char* buf) {}
//	void dGeomTriMeshDataGetBuffer(DTriMeshData g, ByteBuffer buf, RefInt bufLen) { buf.clear(); bufLen.set(0); }
//	void dGeomTriMeshDataSetBuffer(DTriMeshData g, ByteBuffer buf) {}

	/**
	 * Debugging method to check trimesh.
	 * This may e.g. be called from build().
	 */
	public void check() {
		@SuppressWarnings({"unchecked"})
		ArrayList<Integer>[] edges = (ArrayList<Integer>[]) new ArrayList<?>[getDataRef().length/3];  // n = number of vertices
		System.out.print("Checking Trimesh (size " + edges.length + " ) ...");
		for (int i = 0; i < edges.length; i++) edges[i] = new ArrayList<>();
		int nE = 0;
		int[] m_Indices = getIndexRef();
		for (int i = 0; i < m_Indices.length; i+=3) {
			int[] ia = new int[4];
			ia[0] = m_Indices[i];
			ia[1] = m_Indices[i+1];
			ia[2] = m_Indices[i+2];
			ia[3] = ia[0];
			for (int j = 0; j < 3; j++) {
				nE++;
				ArrayList<Integer> l = edges[ia[j]];
				if (l.contains(ia[j+1])) {
					System.out.println("WARNING: Reversed edge: " + ia[j] + " / " + ia[j+1]);
				} else {
					l.add(ia[j+1]);
				}
			}

		}
		System.out.println(nE);
	}

	/**
	 * For testing only.
	 * <p>
	 * Note: In ode4j 0.4.0 this returned 2*Pi for boundary edges.
	 * This behavior has changed and is now in alignment with ODE.
	 * @param edge edge
	 * @param triangle triangle
	 * @return angle
	 */
	public float getEdgeAngle(int triangle, int edge) {
		return (float) retrieveFaceAngle(triangle, edge);
		//return (float) (m_Angles != null ? m_Angles[triangle * 3 + edge] : Math.PI * 2);
	}
}
