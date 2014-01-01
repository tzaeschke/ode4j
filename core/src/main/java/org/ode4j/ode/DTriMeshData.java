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
 * This should be more sophisticated, so that the user can easyly implement
 * another collision library, but this is a lot of work, and also costs some
 * performance because some data has to be copied.
 */
public interface DTriMeshData {

//	/**
//	 * Build a TriMesh data object with single precision vertex data.
//	 */
//	void buildSingle(
//			final double[] Vertices, int VertexStride, int VertexCount, 
//			final int[] Indices, int IndexCount, int TriStride);
	/**
	 * Build a TriMesh data object with single precision vertex data.
	 * In Java, the number of vertices and indices are derived from
	 * the length of the arrays. Strides can not be set because
	 * GIMPACT assumes strides to be 3.
	 */
	//void buildSingle(
	void build(
//			final float[] Vertices, int VertexStride, int VertexCount, 
//			final int[] Indices, int IndexCount, int TriStride);
		final float[] vertices,  
		final int[] indices);

	//GIMPACT does not support normals.
//	/**
//	 * Build a TriMesh data object with single precision vertex data.
//	 * In Java, the number of vertices and indices are derived from
//	 * the length of the arrays. Strides can not be set because
//	 * GIMPACT assumes strides to be 3.
//	 * 
//	 * Same again with a normals array (used as trimesh-trimesh optimization).
//	 */
//	void build(
////			final float[] Vertices, int VertexStride, int VertexCount, 
////			final int[] Indices, int IndexCount, int TriStride);
//		final float[] vertices,  
//		final int[] indices,
//		final int[] normals);

	
	void destroy();

	//Not available in GIMPACT
//	void set(int data_id, Object in_data);
//	Object get(int data_id);
//	void dGeomTriMeshDataSet(DTriMeshData g, int data_id, Object in_data);
//	Object dGeomTriMeshDataGet(DTriMeshData g, int data_id);
	
	
	
//	/** same again with a normals array (used as trimesh-trimesh optimization) */
//	//ODE_API 
//	//	 void dGeomTriMeshDataBuildSingle1(dTriMeshData g,
//	//                                  final void* Vertices, int VertexStride, int VertexCount, 
//	//                                  final void* Indices, int IndexCount, int TriStride,
//	//                                  final void* Normals) {
//	void buildSingle1(DTriMeshData g,
//			final double[] Vertices, int VertexStride, int VertexCount, 
//			final int[] Indices, int IndexCount, int TriStride,
//			final int[] Normals);
//	/**
//	 * Build a TriMesh data object with double precision vertex data.
//	 */
//	//ODE_API 
//	void buildDouble(DTriMeshData g, 
//			//                                 final void* Vertices,  int VertexStride, int VertexCount, 
//			//                                 final void* Indices, int IndexCount, int TriStride) {
//			final double[] Vertices,  int VertexStride, int VertexCount, 
//			final int[] Indices, int IndexCount, int TriStride);
//	/** same again with a normals array (used as trimesh-trimesh optimization) */
//	//ODE_API 
//	//	 void dGeomTriMeshDataBuildDouble1(dTriMeshData g, 
//	//                                  final void* Vertices,  int VertexStride, int VertexCount, 
//	//                                  final void* Indices, int IndexCount, int TriStride,
//	//                                  final void* Normals) {
//	void buildDouble1(DTriMeshData g, 
//			final double[] Vertices,  int VertexStride, int VertexCount, 
//			final int[] Indices, int IndexCount, int TriStride,
//			final int[] Normals);
//
//	/**
//	 * Simple build. Single/double precision based on dSINGLE/dDOUBLE!
//	 */
//	//ODE_API 
//	//	 void dGeomTriMeshDataBuildSimple(dTriMeshData g,
//	//                                 final double* Vertices, int VertexCount,
//	//                                 final dTriIndex* Indices, int IndexCount) {
//	void buildSimple(DTriMeshData g,
//			final double[] Vertices, int VertexCount,
//			final int[] Indices, int IndexCount);
//	/** same again with a normals array (used as trimesh-trimesh optimization) */
//	//ODE_API 
//	//	 void dGeomTriMeshDataBuildSimple1(dTriMeshData g,
//	//                                  final double* Vertices, int VertexCount,
//	//                                  final dTriIndex* Indices, int IndexCount,
//	//                                  final int* Normals) {
//	void buildSimple1(DTriMeshData g,
//			final double[] Vertices, int VertexCount,
//			final int[] Indices, int IndexCount,
//			final int[] Normals);
//
//	/** Preprocess the trimesh data to remove mark unnecessary edges and vertices */
//	//ODE_API 
//	void preprocess(DTriMeshData g);
//	/** Get and set the internal preprocessed trimesh data buffer, for loading and saving */
//	//ODE_API 
//	//void dGeomTriMeshDataGetBuffer(dTriMeshData g, unsigned char** buf, int* bufLen) {
//	void getBuffer(DTriMeshData g, byte[][] buf, RefInt bufLen);
//	//ODE_API 
//	//void dGeomTriMeshDataSetBuffer(dTriMeshData g, unsigned char* buf) {
//	void setBuffer(DTriMeshData g, byte[] buf);

}
