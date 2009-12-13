package org.ode4j.ode.internal;

import org.cpp4j.java.RefDouble;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.math.DVector4;
import org.ode4j.ode.OdeMath;

public class GimpactCollision {
	
	//TODO REMOVE !!
	//TODO REMOVE !!
	//TODO REMOVE !!
	//TODO REMOVE !!
	private static class mat4f {
		float[][] f;
	};
	
	//	#if dTRIMESH_GIMPACT


	//TZ TODO
	//	#ifdef dDOUBLE
	//		// To use GIMPACT with doubles, we need to patch a couple of the GIMPACT functions to 
	//		// convert arguments to floats before sending them in
	//
	//
	//		/// Convert an gimpact vec3f to a ODE dVector3d:   dVector3[i] = vec3f[i]
	//		private void dVECTOR3_VEC3F_COPY(b,a) { 			
	//			(b)[0] = (a)[0];              
	//			(b)[1] = (a)[1];              
	//			(b)[2] = (a)[2];              
	//			(b)[3] = 0;                   
	//		}
	//
	//		private void gim_trimesh_get_triangle_verticesODE(GIM_TRIMESH * trimesh, GUINT triangle_index, dVector3 v1, dVector3 v2, dVector3 v3) {   
	//			vec3f src1, src2, src3;
	//			gim_trimesh_get_triangle_vertices(trimesh, triangle_index, src1, src2, src3);
	//
	//			dVECTOR3_VEC3F_COPY(v1, src1);
	//			dVECTOR3_VEC3F_COPY(v2, src2);
	//			dVECTOR3_VEC3F_COPY(v3, src3);
	//		}
	//
	//		// Anything calling gim_trimesh_get_triangle_vertices from within ODE 
	//		// should be patched through to the dDOUBLE version above
	//
	//		#define gim_trimesh_get_triangle_vertices gim_trimesh_get_triangle_verticesODE
	//
	//		inline int gim_trimesh_ray_closest_collisionODE( GIM_TRIMESH *mesh, dVector3 origin, dVector3 dir, GREAL tmax, GIM_TRIANGLE_RAY_CONTACT_DATA *contact ) {
	//			vec3f dir_vec3f    = { dir[ 0 ],       dir[ 1 ],    dir[ 2 ] };
	//			vec3f origin_vec3f = { origin[ 0 ], origin[ 1 ], origin[ 2 ] };
	//
	//			return gim_trimesh_ray_closest_collision( mesh, origin_vec3f, dir_vec3f, tmax, contact );
	//		}
	//
	//		inline int gim_trimesh_ray_collisionODE( GIM_TRIMESH *mesh, dVector3 origin, dVector3 dir, GREAL tmax, GIM_TRIANGLE_RAY_CONTACT_DATA *contact ) {
	//			vec3f dir_vec3f    = { dir[ 0 ],       dir[ 1 ],    dir[ 2 ] };
	//			vec3f origin_vec3f = { origin[ 0 ], origin[ 1 ], origin[ 2 ] };
	//
	//			return gim_trimesh_ray_collision( mesh, origin_vec3f, dir_vec3f, tmax, contact );
	//		}
	//
	//		#define gim_trimesh_sphere_collisionODE( mesh, Position, Radius, contact ) {	\
	//			vec3f pos_vec3f = { Position[ 0 ], Position[ 1 ], Position[ 2 ] };			\
	//			gim_trimesh_sphere_collision( mesh, pos_vec3f, Radius, contact );			\
	//		}
	//
	//		#define gim_trimesh_plane_collisionODE( mesh, plane, contact ) { 			\
	//			vec4f plane_vec4f = { plane[ 0 ], plane[ 1 ], plane[ 2 ], plane[ 3 ] }; \
	//			gim_trimesh_plane_collision( mesh, plane_vec4f, contact );				\
	//		}
	//
	//		#define GIM_AABB_COPY( src, dst ) {		\
	//			dst[ 0 ]= (src) -> minX;			\
	//			dst[ 1 ]= (src) -> maxX;			\
	//			dst[ 2 ]= (src) -> minY;			\
	//			dst[ 3 ]= (src) -> maxY;			\
	//			dst[ 4 ]= (src) -> minZ;			\
	//			dst[ 5 ]= (src) -> maxZ;			\
	//		}
	//
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
	static int FetchTriangleCount(Gimpact TriMesh)
	{
		throw new UnsupportedOperationException(); //TZ TODO
//		return gim_trimesh_get_triangle_count(TriMesh.m_collision_trimesh);
	}

	//inline void FetchTransformedTriangle(dxTriMesh* TriMesh, int Index, dVector3 Out[3]){
	static void FetchTransformedTriangle(DxTriMesh TriMesh, int Index, DVector3 Out[]){
		throw new UnsupportedOperationException(); //TZ TODO
//		gim_trimesh_locks_work_data(TriMesh.m_collision_trimesh);	
//		gim_trimesh_get_triangle_vertices(TriMesh.m_collision_trimesh, Index, Out[0], Out[1], Out[2]);
//		gim_trimesh_unlocks_work_data(TriMesh.m_collision_trimesh);
	}

	//		inline void MakeMatrix(const dVector3 Position, const dMatrix3 Rotation, mat4f m)
	void MakeMatrix(DVector3C Position, DMatrix3C Rotation, mat4f m)
	{
		m.f[0][0] = (float) Rotation.get00();
		m.f[0][1] = (float) Rotation.get01();
		m.f[0][2] = (float) Rotation.get02();

		m.f[1][0] = (float) Rotation.get10();
		m.f[1][1] = (float) Rotation.get11();
		m.f[1][2] = (float) Rotation.get12();

		m.f[2][0] = (float) Rotation.get20();//[8];
		m.f[2][1] = (float) Rotation.get21();//[9];
		m.f[2][2] = (float) Rotation.get22();//[10];

		m.f[0][3] = (float) Position.get0();//[0];
		m.f[1][3] = (float) Position.get1();//[1];
		m.f[2][3] = (float) Position.get2();//[2];

	}

	//		inline void MakeMatrix(dxGeom* g, mat4f Out){
	void MakeMatrix(DxGeom g, mat4f Out){
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
	void GetPointFromBarycentric(DVector3C dv[], double u, double v, DVector4 Out){
		double w = 1.0 - u - v;
		throw new UnsupportedOperationException(); //TZ TODO
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

	// Some utilities
//	template<class T> const T& dcMAX(const T& x, const T& y){
//		return x > y ? x : y;
//	}
	private Object dcMAX(double x, double y) {
		return x > y ? x : y; //TODO return ref? TZ
	}

//	template<class T> const T& dcMIN(const T& x, const T& y){
//		return x < y ? x : y;
//	}
	private Object dcMIN(double x, double y) {
		return x > y ? x : y; //TODO return ref? TZ
	}

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
		OdeMath.dMULTIPLY0_331( out, orientation, in );
//		out[0] += position[0];
//		out[1] += position[1];
//		out[2] += position[2];
		out.add(position);
	}

	//------------------------------------------------------------------------------
	/**
  @brief Check for intersection between triangle and capsule.

  @param dist [out] Shortest distance squared between the triangle and 
                    the capsule segment (central axis).
  @param t    [out] t value of point on segment that's the shortest distance 
                    away from the triangle, the coordinates of this point 
                    can be found by (cap.seg.end - cap.seg.start) * t,
                    or cap.seg.ipol(t).
  @param u    [out] Barycentric coord on triangle.
  @param v    [out] Barycentric coord on triangle.
  @return True if intersection exists.

  The third Barycentric coord is implicit, ie. w = 1.0 - u - v
  The Barycentric coords give the location of the point on the triangle
  closest to the capsule (where the distance between the two shapes
  is the shortest).
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
