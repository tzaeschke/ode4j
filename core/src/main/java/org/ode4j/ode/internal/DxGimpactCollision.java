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

import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.math.DVector4;
import org.ode4j.math.DVector4C;
import org.ode4j.ode.DAABB;
import org.ode4j.ode.OdeMath;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;
import org.ode4j.ode.internal.gimpact.GimContact;
import org.ode4j.ode.internal.gimpact.GimDynArray;
import org.ode4j.ode.internal.gimpact.GimTrimesh;
import org.ode4j.ode.internal.gimpact.GimGeometry.aabb3f;
import org.ode4j.ode.internal.gimpact.GimGeometry.mat4f;
import org.ode4j.ode.internal.gimpact.GimGeometry.vec3f;
import org.ode4j.ode.internal.gimpact.GimGeometry.vec4f;
import org.ode4j.ode.internal.gimpact.GimTriCollision.GIM_TRIANGLE_RAY_CONTACT_DATA;

public class DxGimpactCollision {
	
	
	//	#if dTRIMESH_GIMPACT


	//TZ TODO
	//	#ifdef dDOUBLE
	// To use GIMPACT with doubles, we need to patch a couple of the GIMPACT functions to 
	// convert arguments to floats before sending them in
	
	
	// Convert an gimpact vec3f to a ODE dVector3d:   dVector3[i] = vec3f[i]
//			private void dVECTOR3_VEC3F_COPY(b,a) { 			
//				(b)[0] = (a)[0];              
//				(b)[1] = (a)[1];              
//				(b)[2] = (a)[2];              
//				(b)[3] = 0;                   
//			}
	
			//private void gim_trimesh_get_triangle_verticesODE(GimTrimesh trimesh, GUINT triangle_index, dVector3 v1, dVector3 v2, dVector3 v3) {   
//			private void gim_trimesh_get_triangle_verticesODE(GimTrimesh trimesh, int triangle_index, 
//					DVector3 v1, DVector3 v2, DVector3 v3) {   
//				vec3f src1 = new vec3f(), src2 = new vec3f(), src3 = new vec3f();
//				trimesh.gim_trimesh_get_triangle_vertices(triangle_index, src1, src2, src3);
//	
//				v1.set(src1.f);//dVECTOR3_VEC3F_COPY(v1, src1);
//				v2.set(src2.f);//dVECTOR3_VEC3F_COPY(v2, src2);
//				v3.set(src3.f);//dVECTOR3_VEC3F_COPY(v3, src3);
//			}
	
			private static vec3f DVector3Tovec3f(DVector3C v) {
				vec3f vf = new vec3f();
				vf.f[0] = (float) v.get0();
				vf.f[1] = (float) v.get1();
				vf.f[2] = (float) v.get2();
				return vf;
			}

			private static vec4f DVector4Tovec4f(DVector4C v) {
				vec4f vf = new vec4f();
				vf.f[0] = (float) v.get0();
				vf.f[1] = (float) v.get1();
				vf.f[2] = (float) v.get2();
				vf.f[3] = (float) v.get3();
				return vf;
			}

			// Anything calling gim_trimesh_get_triangle_vertices from within ODE 
			// should be patched through to the dDOUBLE version above
	
			//#define gim_trimesh_get_triangle_vertices gim_trimesh_get_triangle_verticesODE
//			static void gim_trimesh_get_triangle_verticesODE( GimTrimesh mesh ) {
//				mesh.gim_trimesh_get_triangle_vertices(0, null, null, null);
//			}
	
//			inline int gim_trimesh_ray_closest_collisionODE( GIM_TRIMESH *mesh, dVector3 origin, 
//					dVector3 dir, GREAL tmax, GIM_TRIANGLE_RAY_CONTACT_DATA *contact ) {
			static int gim_trimesh_ray_closest_collisionODE( GimTrimesh mesh, DVector3C origin, DVector3C dir, 
					double tmax, GIM_TRIANGLE_RAY_CONTACT_DATA contact ) {
				vec3f dir_vec3f    = DVector3Tovec3f( dir );
				vec3f origin_vec3f = DVector3Tovec3f( origin );
	
				return mesh.gim_trimesh_ray_closest_collision( origin_vec3f, dir_vec3f, (float) tmax, contact );
			}
	
			//inline int gim_trimesh_ray_collisionODE( GIM_TRIMESH *mesh, dVector3 origin, dVector3 dir, 
			//GREAL tmax, GIM_TRIANGLE_RAY_CONTACT_DATA *contact ) {
			static int gim_trimesh_ray_collisionODE( GimTrimesh mesh, DVector3C origin, DVector3C dir, 
					double tmax, GIM_TRIANGLE_RAY_CONTACT_DATA contact ) {
				vec3f dir_vec3f    = DVector3Tovec3f( dir );
				vec3f origin_vec3f = DVector3Tovec3f( origin );
	
				return mesh.gim_trimesh_ray_collision( origin_vec3f, dir_vec3f, (float) tmax, contact );
			}
	
			//#define gim_trimesh_sphere_collisionODE( mesh, Position, Radius, contact ) {	\
			static void gim_trimesh_sphere_collisionODE( GimTrimesh mesh, DVector3C Position, 
					double Radius, GimDynArray<GimContact> contact ) {	
				vec3f pos_vec3f = DVector3Tovec3f( Position );			
				mesh.gim_trimesh_sphere_collision( pos_vec3f, (float) Radius, contact );			
			}
	
			//#define gim_trimesh_plane_collisionODE( mesh, plane, contact ) { 			\
			static void gim_trimesh_plane_collisionODE( GimTrimesh mesh, DVector4 plane, 
					GimDynArray<vec4f> contact ) { 			
				vec4f plane_vec4f = DVector4Tovec4f( plane ); 
				mesh.gim_trimesh_plane_collision( plane_vec4f, contact );				
			}
	
			//#define GIM_AABB_COPY( src, dst ) {		\
			static void GIM_AABB_COPY( aabb3f src, DAABB dst ) {
				dst.set( src.minX, src.maxX, src.minY, src.maxY, src.minZ, src.maxZ );			
//				dst[ 0 ]= (src).minX;			
//				dst[ 1 ]= (src).maxX;			
//				dst[ 2 ]= (src).minY;			
//				dst[ 3 ]= (src).maxY;			
//				dst[ 4 ]= (src).minZ;			
//				dst[ 5 ]= (src).maxZ;			
			}
	
	//	#else 
	//		// With single precision, we can pass native ODE vectors directly to GIMPACT
	//
	//		#define gim_trimesh_ray_closest_collisionODE 	gim_trimesh_ray_closest_collision
	//		#define gim_trimesh_ray_collisionODE 			gim_trimesh_ray_collision
	//		#define gim_trimesh_sphere_collisionODE 		gim_trimesh_sphere_collision
	//		#define gim_trimesh_plane_collisionODE 			gim_trimesh_plane_collision
	//
	//		#define GIM_AABB_COPY( src, dst ) 	memcpy( dst, src, 6 * sizeof( GREAL ) )
	//
	//	#endif // dDouble

	//	inline unsigned FetchTriangleCount(dxTriMesh* TriMesh)
	static int FetchTriangleCount(DxGimpact TriMesh)
	{
		return TriMesh.m_collision_trimesh.gim_trimesh_get_triangle_count();
	}

	//inline void FetchTransformedTriangle(dxTriMesh* TriMesh, int Index, dVector3 Out[3]){
	static void FetchTransformedTriangle(DxGimpact TriMesh, int Index, DVector3 Out[]){
		TriMesh.m_collision_trimesh.gim_trimesh_locks_work_data();
		vec3f[] vOut = { new vec3f(), new vec3f(), new vec3f() };
		TriMesh.m_collision_trimesh.gim_trimesh_get_triangle_vertices(Index, vOut[0], vOut[1], vOut[2]);
		Out[0].set( vOut[0].f ); Out[1].set( vOut[1].f ); Out[2].set( vOut[2].f );
		TriMesh.m_collision_trimesh.gim_trimesh_unlocks_work_data();
	}

	//		inline void MakeMatrix(const dVector3 Position, const dMatrix3 Rotation, mat4f m)
	static void MakeMatrix(DVector3C Position, DMatrix3C Rotation, mat4f m)
	{
		m.f[0] = (float) Rotation.get00();
		m.f[1] = (float) Rotation.get01();
		m.f[2] = (float) Rotation.get02();

		m.f[4] = (float) Rotation.get10();
		m.f[5] = (float) Rotation.get11();
		m.f[6] = (float) Rotation.get12();

		m.f[8] = (float) Rotation.get20();//[8];
		m.f[9] = (float) Rotation.get21();//[9];
		m.f[10] = (float) Rotation.get22();//[10];

		m.f[3] = (float) Position.get0();//[0];
		m.f[7] = (float) Position.get1();//[1];
		m.f[11] = (float) Position.get2();//[2];

	}

	//		inline void MakeMatrix(dxGeom* g, mat4f Out){
	static void MakeMatrix(DxGeom g, mat4f Out){
		DVector3C Position = g.dGeomGetPosition();
		DMatrix3C Rotation = g.dGeomGetRotation();
		MakeMatrix(Position, Rotation, Out);
	}

	//***  TODO TZ END OF GIMPACT SPECIFIC PART
	//#endif // dTRIMESH_GIMPACT

	// Outputs a matrix to 3 vectors
	//inline void Decompose(const dMatrix3 Matrix, dVector3 Right, dVector3 Up, dVector3 Direction){
	void Decompose(DMatrix3C Matrix, DVector3 Right, DVector3 Up, DVector3 Direction){
//		Right[0] = Matrix[0 * 4 + 0];
//		Right[1] = Matrix[1 * 4 + 0];
//		Right[2] = Matrix[2 * 4 + 0];
//		Right[3] = REAL(0.0);
		Right.set(Matrix.get00(), Matrix.get10(), Matrix.get20());
//		Up[0] = Matrix[0 * 4 + 1];
//		Up[1] = Matrix[1 * 4 + 1];
//		Up[2] = Matrix[2 * 4 + 1];
//		Up[3] = REAL(0.0);
		Up.set(Matrix.get01(), Matrix.get11(), Matrix.get21());
//		Direction[0] = Matrix[0 * 4 + 2];
//		Direction[1] = Matrix[1 * 4 + 2];
//		Direction[2] = Matrix[2 * 4 + 2];
//		Direction[3] = REAL(0.0);
		Direction.set(Matrix.get02(), Matrix.get12(), Matrix.get22());
	}

	// Outputs a matrix to 3 vectors
	//inline void Decompose(const dMatrix3 Matrix, dVector3 Vectors[3]){
	void Decompose(DMatrix3C Matrix, DVector3 Vectors[]){
		Decompose(Matrix, Vectors[0], Vectors[1], Vectors[2]);
	}

	// Finds barycentric
	//inline void GetPointFromBarycentric(const dVector3 dv[3], dReal u, dReal v, dVector3 Out){
	static void GetPointFromBarycentric(vec3f dv[], double u, double v, DVector3 Out){
		double w = 1.0 - u - v;
		Out.set0( dv[0].f[0]*w + dv[1].f[0]*u + dv[2].f[0]*v );
		Out.set1( dv[0].f[1]*w + dv[1].f[1]*u + dv[2].f[1]*v );
		Out.set2( dv[0].f[2]*w + dv[1].f[2]*u + dv[2].f[2]*v );
//		Out.set0( dv[0].get3()*w + dv[1].get3()*u + dv[2].get3()*v );  //TODO ?
//		Out[0] = (dv[0][0] * w) + (dv[1][0] * u) + (dv[2][0] * v);
//		Out[1] = (dv[0][1] * w) + (dv[1][1] * u) + (dv[2][1] * v);
//		Out[2] = (dv[0][2] * w) + (dv[1][2] * u) + (dv[2][2] * v);
//		Out[3] = (dv[0][3] * w) + (dv[1][3] * u) + (dv[2][3] * v);
	}

	// Performs a callback
	//inline bool Callback(dxTriMesh* TriMesh, dxGeom* Object, int TriIndex){
	boolean Callback(DxTriMesh TriMesh, DxGeom Object, int TriIndex){
		if (TriMesh.Callback != null){
			return (TriMesh.Callback.call(TriMesh, Object, TriIndex)!=0);
		}
		else return true;
	}

//	// Some utilities
////	template<class T> const T& dcMAX(const T& x, const T& y){
////		return x > y ? x : y;
////	}
//	private Object dcMAX(double x, double y) {
//		return x > y ? x : y; //TODO return ref? TZ
//	}
//
////	template<class T> const T& dcMIN(const T& x, const T& y){
////		return x < y ? x : y;
////	}
//	private Object dcMIN(double x, double y) {
//		return x > y ? x : y; //TODO return ref? TZ
//	}

	//TZ TODO
//	dReal SqrDistancePointTri( DVector3C p, DVector3C triOrigin, 
//			DVector3C triEdge1, DVector3C triEdge2,
//			dReal* pfSParam = 0, dReal* pfTParam = 0 );
//
//	dReal SqrDistanceSegments( DVector3C seg1Origin, DVector3C seg1Direction, 
//			DVector3C seg2Origin,DVector3C seg2Direction,
//			dReal* pfSegP0 = 0, dReal* pfSegP1 = 0 );
//
//	dReal SqrDistanceSegTri( DVector3C segOrigin, DVector3C segEnd, 
//			DVector3C triOrigin, 
//			DVector3C triEdge1, DVector3C triEdge2,
//			dReal* t = 0, dReal* u = 0, dReal* v = 0 );

	//inline
	//void Vector3Subtract( const dVector3 left, const dVector3 right, dVector3 result )
	void Vector3Subtract( DVector3C left, DVector3C right, DVector3 result )
	{
//		result[0] = left[0] - right[0];
//		result[1] = left[1] - right[1];
//		result[2] = left[2] - right[2];
//		result[3] = REAL(0.0);
		result.eqDiff(left, right);
	}

	void Vector3Add( DVector3C left, DVector3C right, DVector3 result )
	{
//		result[0] = left[0] + right[0];
//		result[1] = left[1] + right[1];
//		result[2] = left[2] + right[2];
//		result[3] = REAL(0.0);
		result.eqSum(left, right);
	}

	void Vector3Negate( DVector3C in, DVector3 out )
	{
//		out[0] = -in[0];
//		out[1] = -in[1];
//		out[2] = -in[2];
//		out[3] = REAL(0.0);
		out.set(in).scale(-1);
	}

	void Vector3Copy( DVector3C in, DVector3 out )
	{
//		out[0] = in[0];
//		out[1] = in[1];
//		out[2] = in[2];
//		out[3] = REAL(0.0);
		out.set(in);
	}

	void Vector3Multiply( DVector3C in, double scalar, DVector3 out )
	{
//		out[0] = in[0] * scalar;
//		out[1] = in[1] * scalar;
//		out[2] = in[2] * scalar;
//		out[3] = REAL(0.0);
		out.set(in).scale(scalar);
	}

	void TransformVector3( DVector3C in, 
			DMatrix3C orientation, DVector3C position, 
			DVector3 out )
	{
		OdeMath.dMultiply0_331( out, orientation, in );
//		out[0] += position[0];
//		out[1] += position[1];
//		out[2] += position[2];
		out.add(position);
	}

	//------------------------------------------------------------------------------
	/**
	 * Check for intersection between triangle and capsule.
	 * 
	 * @param dist [out] Shortest distance squared between the triangle and 
	 *                   the capsule segment (central axis).
	 * @param t    [out] t value of point on segment that's the shortest distance 
	 *                   away from the triangle, the coordinates of this point 
	 *                   can be found by (cap.seg.end - cap.seg.start) * t,
	 *                   or cap.seg.ipol(t).
	 * @param u    [out] Barycentric coord on triangle.
	 * @param v    [out] Barycentric coord on triangle.
	 * @return True if intersection exists.
	 * 
	 * The third Barycentric coord is implicit, ie. w = 1.0 - u - v
	 * The Barycentric coords give the location of the point on the triangle
	 * closest to the capsule (where the distance between the two shapes
	 * is the shortest).
	 */
	//inline
	//bool IntersectCapsuleTri( const dVector3 segOrigin, const dVector3 segEnd, 
	//                          const dReal radius, const dVector3 triOrigin, 
	//                          const dVector3 triEdge0, const dVector3 triEdge1,
	//                          dReal* dist, dReal* t, dReal* u, dReal* v )
	boolean IntersectCapsuleTri( DVector3C segOrigin, DVector3C segEnd, 
			final double radius, DVector3C triOrigin, 
			DVector3C triEdge0, DVector3C triEdge1,
			RefDouble dist, double[] t, double[] u, double[] v )
	{
		throw new UnsupportedOperationException(); //TZ TODO
//		double sqrDist = SqrDistanceSegTri( segOrigin, segEnd, triOrigin, triEdge0, triEdge1, 
//				t, u, v );
//
////		if ( dist )
////			*dist = sqrDist;
//		if ( dist != null )
//			dist.d = sqrDist;
//
//		return ( sqrDist <= (radius * radius) );
	}


}
