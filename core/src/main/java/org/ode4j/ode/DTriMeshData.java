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
	 * @param vertices vertices
	 * @param indices indices
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




	/*
	 * Data preprocessing build request flags.
	 */
	class dTRIDATAPREPROCESS_BUILD {
		public static final int CONCAVE_EDGES = 0; // Used to optimize OPCODE trimesh-capsule collisions; allocates 1 byte per triangle; no extra data associated
		public static final int FACE_ANGLES = 1;   // Used to aid trimesh-convex collisions; memory requirements depend on extra data

		private dTRIDATAPREPROCESS_BUILD() {}
	}
//	enum
//	{
//		dTRIDATAPREPROCESS_BUILD__MIN,
//
//				dTRIDATAPREPROCESS_BUILD_CONCAVE_EDGES = dTRIDATAPREPROCESS_BUILD__MIN, // Used to optimize OPCODE trimesh-capsule collisions; allocates 1 byte per triangle; no extra data associated
//				dTRIDATAPREPROCESS_BUILD_FACE_ANGLES,   // Used to aid trimesh-convex collisions; memory requirements depend on extra data
//
//				dTRIDATAPREPROCESS_BUILD__MAX,
//	};

	/*
	 * Data preprocessing extra values for dTRIDATAPREPROCESS_BUILD_FACE_ANGLES.
	 */
	enum dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA {
		BYTE_POSITIVE, // Build angles for convex edges only and store as bytes; allocates 3 bytes per triangle; stores angles (0..180] in 1/254 fractions leaving two values for the flat and all the concaves
		BYTE_ALL, // Build angles for all the edges and store in bytes; allocates 3 bytes per triangle; stores angles [-180..0) and (0..180] in 1/127 fractions plus a value for the flat angle
		WORD_ALL; // Build angles for all the edges and store in words; allocates 6 bytes per triangle; stores angles [-180..0) and (0..180] in 1/32767 fractions plus a value for the flat angle

		public static final dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA DEFAULT = BYTE_POSITIVE; // The default value assumed if the extra data is not provided
	}
//	enum
//	{
	public static final int dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MIN = 0;
	// Build angles for convex edges only and store as bytes; allocates 3 bytes per triangle; stores angles (0..180] in 1/254 fractions leaving two values for the flat and all the concaves
	public static final int dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA_BYTE_POSITIVE = dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MIN;
	// Build angles for all the edges and store in bytes; allocates 3 bytes per triangle; stores angles [-180..0) and (0..180] in 1/127 fractions plus a value for the flat angle
	public static final int dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA_BYTE_ALL = dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA_BYTE_POSITIVE + 1;
	// Build angles for all the edges and store in words; allocates 6 bytes per triangle; stores angles [-180..0) and (0..180] in 1/32767 fractions plus a value for the flat angle
	public static final int dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA_WORD_ALL = dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA_BYTE_ALL + 1;
	public static final int dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__MAX = dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA_WORD_ALL + 1;
	// The default value assumed if the extra data is not provided
	public static final int dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA__DEFAULT = dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA_BYTE_POSITIVE;



	/*
	 * Pre-process the trimesh data according to the request flags.
	 *
	 * buildRequestFlags is a bitmask of 1U << dTRIDATAPREPROCESS_BUILD_...
	 * It is allowed to call the function multiple times provided the bitmasks are different each time.
	 *
	 * requestExtraData is an optional pointer to array of extra parameters per bitmask bits
	 * (only the elements indexed by positions of raised bits are examined;
	 * defaults are assumed if the pointer is NULL)
	 *
	 * The function returns a boolean status the only failure reason being insufficient memory.
	 */
	//ODE_API
	//int dGeomTriMeshDataPreprocess2(dTriMeshDataID g, unsigned int buildRequestFlags, const dintptr *requestExtraData/*=NULL | const dintptr (*)[dTRIDATAPREPROCESS_BUILD__MAX]*/);
	boolean preprocess2(int buildRequestFlags, final long[] requestExtraData/*=NULL | const dintptr (*)[dTRIDATAPREPROCESS_BUILD__MAX]*/);

	/*
	 * Obsolete. Equivalent to calling dGeomTriMeshDataPreprocess2(g, (1U << dTRIDATAPREPROCESS_BUILD_CONCAVE_EDGES), NULL)
	 */
	//ODE_API
	boolean preprocess();

	/*
	 * Get and set the internal preprocessed trimesh data buffer (see the enumerated type above), for loading and saving
	 * These functions are deprecated. Use dGeomTriMeshDataSet/dGeomTriMeshDataGet2 with dTRIMESHDATA_USE_FLAGS instead.
	 */
//	//ODE_API
//	//void dGeomTriMeshDataGetBuffer(dTriMeshData g, unsigned char** buf, int* bufLen) {
//	void getBuffer(DTriMeshData g, byte[][] buf, RefInt bufLen);
//	//ODE_API 
//	//void dGeomTriMeshDataSetBuffer(dTriMeshData g, unsigned char* buf) {
//	void setBuffer(DTriMeshData g, byte[] buf);

	//ODE_API
	//void dGeomTriMeshDataUpdate(DTriMeshData g) {
	void update();

}
