/***
 * libccd
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 * Java-port: Copyright (c) 2009-2014 Tilmann Zaeschke<ode4j@gmx.de>  
 *
 *
 *  This file is part of libccd.
 *
 *  Distributed under the OSI-approved BSD License (the "License");
 *  see accompanying file BDS-LICENSE for details or see
 *  <http://www.opensource.org/licenses/bsd-license.php>.
 *
 *  This software is distributed WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the License for more information.
 */
package org.ode4j.ode.internal.libccd;

import java.util.Arrays;

import org.ode4j.ode.internal.cpp4j.java.Ref;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;
import org.ode4j.ode.internal.libccd.CCDPolyTope.ccd_pt_edge_t;
import org.ode4j.ode.internal.libccd.CCDPolyTope.ccd_pt_face_t;
import org.ode4j.ode.internal.libccd.CCDSimplex.ccd_simplex_t;
import org.ode4j.ode.internal.libccd.CCDVec3.ccd_vec3_t;

import static org.ode4j.ode.internal.libccd.CCDCustomVec3.ccdVec3SafeNormalize;
import static org.ode4j.ode.internal.libccd.CCDPolyTope.*;
import static org.ode4j.ode.internal.libccd.CCDSimplex.*;
import static org.ode4j.ode.internal.libccd.CCDSupport.*;
import static org.ode4j.ode.internal.libccd.CCDVec3.*;

public class CCD {

	/**
	 * Type of *support* function that takes pointer to 3D object and direction
	 * and returns (via vec argument) furthest point from object in specified
	 * direction.
	 */
	public interface ccd_support_fn {
		void run(final Object obj, final ccd_vec3_t dir, ccd_vec3_t vec);
	}

	/**
	 * Returns (via dir argument) first direction vector that will be used in
	 * initialization of algorithm.
	 */
	interface ccd_first_dir_fn {
		void run(final Object obj1, final Object obj2, ccd_vec3_t dir);
	}


	/**
	 * Returns (via center argument) geometric center (some point near center)
	 * of given object.
	 */
	public interface ccd_center_fn {
		void run(final Object obj1, ccd_vec3_t center);
	}

	/**
	 * Main structure of CCD algorithm.
	 */
	public static final class ccd_t {
		ccd_first_dir_fn first_dir; //!< Returns initial direction where first
		//!< support point will be searched
		public ccd_support_fn support1; //!< Function that returns support point of
		//!< first object
		public ccd_support_fn support2; //!< Function that returns support point of
		//!< second object

		public ccd_center_fn center1; //!< Function that returns geometric center of
		//!< first object
		public ccd_center_fn center2; //!< Function that returns geometric center of
		//!< second object

		public long max_iterations; //!< Maximal number of iterations
		double epa_tolerance;
		public double mpr_tolerance; //!< Boundary tolerance for MPR algorithm

		public ccd_t() {}
	}
	//	typedef struct _ccd_t ccd_t;

	//	#define CCD_INIT(ccd) \
	public static void CCD_INIT(ccd_t ccd) {
		(ccd).first_dir = new ccd_first_dir_fn() {
			
			@Override
			public void run(Object obj1, Object obj2, ccd_vec3_t dir) {
				ccdFirstDirDefault(obj1, obj2, dir);
			}
		}; 
		(ccd).support1 = null; 
		(ccd).support2 = null; 
		(ccd).center1  = null; 
		(ccd).center2  = null; 

		(ccd).max_iterations = Long.MAX_VALUE;//(long)-1; 
		(ccd).epa_tolerance = (0.0001); 
		(ccd).mpr_tolerance = (0.0001); 
	}


	/**
	 * Default first direction.
	 */
	static void ccdFirstDirDefault(final Object o1, final Object o2, ccd_vec3_t dir)
	{
		ccdVec3Set(dir, CCD_ONE, CCD_ZERO, CCD_ZERO);
	}


	/**
	 * Returns true if two given objects interest.
	 * @param obj1 object 1
	 * @param obj2 object 2
	 * @param ccd ccd
	 * @return true or false
	 */
	public static boolean ccdGJKIntersect(final Object obj1, final Object obj2, final ccd_t ccd)
	{
		ccd_simplex_t simplex = new ccd_simplex_t();
		return __ccdGJK(obj1, obj2, ccd, simplex) == 0;
	}


	/**
	 * This function computes separation vector of two objects. Separation
	 * vector is minimal translation of obj2 to get obj1 and obj2 speparated
	 * (without intersection).
	 * Returns 0 if obj1 and obj2 intersect and sep is filled with translation
	 * vector. If obj1 and obj2 don't intersect -1 is returned.
	 * @param obj1 object 1
	 * @param obj2 object 2
	 * @param ccd ccd
	 * @param sep sep vector
	 * @return 0 if obj1 and obj2 intersect and sep is filled with translation
	 * vector. If obj1 and obj2 don't intersect -1 is returned.
	 */
	public static int ccdGJKSeparate(final Object obj1, final Object obj2, final ccd_t ccd,
			ccd_vec3_t sep)
	{
		ccd_pt_t polytope = new ccd_pt_t();
		Ref<ccd_pt_el_t<?>> nearestRef = new Ref<CCDPolyTope.ccd_pt_el_t<?>>();
		int ret;

		ccdPtInit(polytope);

		ret = __ccdGJKEPA(obj1, obj2, ccd, polytope, nearestRef);

		// set separation vector
		if (nearestRef.get()!=null)
			ccdVec3Copy(sep, nearestRef.get().witness);

		ccdPtDestroy(polytope);

		return ret;
	}


	static int penEPAPosCmp(final ccd_pt_vertex_t a, final ccd_pt_vertex_t b)
	{
		ccd_pt_vertex_t v1, v2;
		v1 = a;//*(ccd_pt_vertex_t **)a;
		v2 = b;//*(ccd_pt_vertex_t **)b;

		if (ccdEq(v1.dist, v2.dist)){
			return 0;
		}else if (v1.dist < v2.dist){
			return -1;
		}else{
			return 1;
		}
	}

	static void penEPAPos(final ccd_pt_t pt, final ccd_pt_el_t<?> nearest,
			ccd_vec3_t pos)
	{
		//ccd_pt_vertex_t v;
		ccd_pt_vertex_t[] vs;
		int i, len;
		double scale;

		// compute median
		len = 0;
		for (@SuppressWarnings("unused") ccd_pt_vertex_t v: pt.vertices) {
			len++;
		}
//		ccdListForEachEntry(pt.vertices, v, ccd_pt_vertex_t, list){
//			len++;
//		}


		vs = new ccd_pt_vertex_t[len];// CCD_ALLOC_ARR(ccd_pt_vertex_t *, len);
		i = 0;
//		ccdListForEachEntry(pt.vertices, v, ccd_pt_vertex_t, list){
		for (ccd_pt_vertex_t v: pt.vertices) {
			vs[i++] = v;
		}

		//qsort(vs, len, sizeof(ccd_pt_vertex_t *), penEPAPosCmp);
		Arrays.sort(vs); //TODO correct ordering?
		
		ccdVec3Set(pos, CCD_ZERO, CCD_ZERO, CCD_ZERO);
		scale = CCD_ZERO;
		if ((len & 1) == 1)
			len++;

		for (i = 0; i < len / 2; i++){
			ccdVec3Add(pos, vs[i].v.v1);
			ccdVec3Add(pos, vs[i].v.v2);
			scale += (2.);
		}
		ccdVec3Scale(pos, CCD_ONE / scale);

		//free(vs);
	}

	/**
	 * Computes penetration of obj2 into obj1.
	 * Depth of penetration, direction and position is returned. It means that
	 * if obj2 is translated by distance depth in direction dir objects will
	 * have touching contact, pos should be position in global coordinates
	 * where force should take a place.
	 *
	 * CCD+EPA algorithm is used.
	 *
	 * Returns 0 if obj1 and obj2 intersect and depth, dir and pos are filled
	 * if given non-NULL pointers.
	 * If obj1 and obj2 don't intersect -1 is returned.
	 * @param obj1 object 1
	 * @param obj2 object 2
	 * @param ccd ccd
	 * @param depth depth
	 * @param dir direction
	 * @param pos pos
	 * @return 0 if obj1 and obj2 intersect and depth, dir and pos are filled
	 * if given non-NULL pointers.
	 * If obj1 and obj2 don't intersect -1 is returned.
	 */
	public static int ccdGJKPenetration(final Object obj1, final Object obj2, final ccd_t ccd,
			RefDouble depth, ccd_vec3_t dir, ccd_vec3_t pos)
	{
		ccd_pt_t polytope = new ccd_pt_t();
		final Ref<ccd_pt_el_t<?>> nearestRef = new Ref<>();
		int ret;

		ccdPtInit(polytope);

		ret = __ccdGJKEPA(obj1, obj2, ccd, polytope, nearestRef);

		// set separation vector
		if (ret == 0 && nearestRef.get() != null){
			// store normalized direction vector
			ccdVec3Copy(dir, nearestRef.get().witness);
			ret = ccdVec3SafeNormalize(dir);

			if (ret == 0) {
				// compute depth of penetration
            	depth.d = CCD_SQRT(nearestRef.get().dist);
				// compute position
				penEPAPos(polytope, nearestRef.get(), pos);
			}
		}

		ccdPtDestroy(polytope);

		return ret;
	}




	/** Performs GJK algorithm. Returns 0 if intersection was found and simplex
	 *  is filled with resulting polytope. */
	private static int __ccdGJK(final Object obj1, final Object obj2,
			final ccd_t ccd, ccd_simplex_t simplex)
	{
		long iterations;
		ccd_vec3_t dir = new ccd_vec3_t(); // direction vector
		ccd_support_t last = new ccd_support_t(); // last support point
		int do_simplex_res;

		// initialize simplex struct
		ccdSimplexInit(simplex);

		// get first direction
		ccd.first_dir.run(obj1, obj2, dir);
		// get first support point
		__ccdSupport(obj1, obj2, dir, ccd, last);
		// and add this point to simplex as last one
		ccdSimplexAdd(simplex, last);

		// set up direction vector to as (O - last) which is exactly -last
		ccdVec3Copy(dir, last.v);
		ccdVec3Scale(dir, -CCD_ONE);

		// start iterations
		for (iterations = 0L; iterations < ccd.max_iterations; ++iterations) {
			// obtain support point
			__ccdSupport(obj1, obj2, dir, ccd, last);

			// check if farthest point in Minkowski difference in direction dir
			// isn't somewhere before origin (the test on negative dot product)
			// - because if it is, objects are not intersecting at all.
			if (ccdVec3Dot(last.v, dir) < CCD_ZERO){
				return -1; // intersection not found
			}

			// add last support vector to simplex
			ccdSimplexAdd(simplex, last);

			// if doSimplex returns 1 if objects intersect, -1 if objects don't
			// intersect and 0 if algorithm should continue
			do_simplex_res = doSimplex(simplex, dir);
			if (do_simplex_res == 1){
				return 0; // intersection found
			}else if (do_simplex_res == -1){
				return -1; // intersection not found
			}

			if (ccdIsZero(ccdVec3Len2(dir))){
				return -1; // intersection not found
			}
		}

		// intersection wasn't found
		return -1;
	}

	/** Performs GJK+EPA algorithm. Returns 0 if intersection was found and
	 *  pt is filled with resulting polytope and nearest with pointer to
	 *  nearest element (vertex, edge, face) of polytope to origin. */
	private static int __ccdGJKEPA(final Object obj1, final Object obj2,
			final ccd_t ccd,
			ccd_pt_t polytope, Ref<ccd_pt_el_t<?>> nearest) // **nearest)
	{
		ccd_simplex_t simplex = new ccd_simplex_t();
		ccd_support_t supp = new ccd_support_t(); // support point
		int ret, size;

		nearest.set(null);// = NULL;

		// run GJK and obtain terminal simplex
		ret = __ccdGJK(obj1, obj2, ccd, simplex);
		if (ret != 0)
			return -1;

		// transform simplex to polytope - simplex won't be used anymore
		size = ccdSimplexSize(simplex);
		if (size == 4){
			if (simplexToPolytope4(obj1, obj2, ccd, simplex, polytope, nearest) != 0){
				return 0;// touch contact
			}
		}else if (size == 3){
			if (simplexToPolytope3(obj1, obj2, ccd, simplex, polytope, nearest) != 0){
				return 0; // touch contact
			}
		}else{ // size == 2
			if (simplexToPolytope2(obj1, obj2, ccd, simplex, polytope, nearest) != 0){
				return 0; // touch contact
			}
		}

		while (true){
			// get triangle nearest to origin
			nearest.set( ccdPtNearest(polytope) );

			// get next support point
			if (nextSupport(obj1, obj2, ccd, nearest.get(), supp) != 0)
				break;

			// expand nearest triangle using new point - supp
			expandPolytope(polytope, nearest.get(), supp);
		}

		return 0;
	}



	/** Returns true if simplex contains origin.
	 *  This function also alteres simplex and dir according to further
	 *  processing of GJK algorithm. */
	private static int doSimplex2(ccd_simplex_t simplex, ccd_vec3_t dir)
	{
		final ccd_support_t A, B;
		ccd_vec3_t AB = new ccd_vec3_t(), AO = new ccd_vec3_t(), tmp = new ccd_vec3_t();
		double dot;

		// get last added as A
		A = ccdSimplexLast(simplex);
		// get the other point
		B = ccdSimplexPoint0(simplex);
		// compute AB oriented segment
		ccdVec3Sub2(AB, B.v, A.v);
		// compute AO vector
		ccdVec3Copy(AO, A.v);
		ccdVec3Scale(AO, -CCD_ONE);

		// dot product AB . AO
		dot = ccdVec3Dot(AB, AO);

		// check if origin doesn't lie on AB segment
		ccdVec3Cross(tmp, AB, AO);
		if (ccdIsZero(ccdVec3Len2(tmp)) && dot > CCD_ZERO){
			return 1;
		}

		// check if origin is in area where AB segment is
		if (ccdIsZero(dot) || dot < CCD_ZERO){
			// origin is in outside are of A
			ccdSimplexSet0(simplex, A);
			ccdSimplexSetSize(simplex, 1);
			ccdVec3Copy(dir, AO);
		}else{
			// origin is in area where AB segment is

			// keep simplex untouched and set direction to
			// AB x AO x AB
			tripleCross(AB, AO, AB, dir);
		}

		return 0;
	}

	/** Returns true if simplex contains origin.
	 *  This function also alteres simplex and dir according to further
	 *  processing of GJK algorithm. */
	private static int doSimplex3(ccd_simplex_t simplex, ccd_vec3_t dir)
	{
		final ccd_support_t A, B, C;
		final ccd_vec3_t AO = new ccd_vec3_t(), AB = new ccd_vec3_t(), AC = new ccd_vec3_t();
		final ccd_vec3_t ABC = new ccd_vec3_t(), tmp = new ccd_vec3_t();
		double dot, dist;

		// get last added as A
		A = ccdSimplexLast(simplex);
		// get the other points
		B = ccdSimplexPoint1(simplex);
		C = ccdSimplexPoint0(simplex);

		// check touching contact
		dist = ccdVec3PointTriDist2(ccd_vec3_origin, A.v, B.v, C.v, null);
		if (ccdIsZero(dist)){
			return 1;
		}

		// check if triangle is really triangle (has area > 0)
		// if not simplex can't be expanded and thus no itersection is found
		if (ccdVec3Eq(A.v, B.v) || ccdVec3Eq(A.v, C.v)){
			return -1;
		}

		// compute AO vector
		ccdVec3Copy(AO, A.v);
		ccdVec3Scale(AO, -CCD_ONE);

		// compute AB and AC segments and ABC vector (perpendircular to triangle)
		ccdVec3Sub2(AB, B.v, A.v);
		ccdVec3Sub2(AC, C.v, A.v);
		ccdVec3Cross(ABC, AB, AC);

		ccdVec3Cross(tmp, ABC, AC);
		dot = ccdVec3Dot(tmp, AO);
		if (ccdIsZero(dot) || dot > CCD_ZERO){
			dot = ccdVec3Dot(AC, AO);
			if (ccdIsZero(dot) || dot > CCD_ZERO){
				// C is already in place
				ccdSimplexSet1(simplex, A);
				ccdSimplexSetSize(simplex, 2);
				tripleCross(AC, AO, AC, dir);
			}else{
				//ccd_do_simplex3_45:
				dot = ccdVec3Dot(AB, AO);
				if (ccdIsZero(dot) || dot > CCD_ZERO){
					ccdSimplexSet0(simplex, B);
					ccdSimplexSet1(simplex, A);
					ccdSimplexSetSize(simplex, 2);
					tripleCross(AB, AO, AB, dir);
				}else{
					ccdSimplexSet0(simplex, A);
					ccdSimplexSetSize(simplex, 1);
					ccdVec3Copy(dir, AO);
				}
			}
		}else{
			ccdVec3Cross(tmp, AB, ABC);
			dot = ccdVec3Dot(tmp, AO);
			if (ccdIsZero(dot) || dot > CCD_ZERO){
				//goto ccd_do_simplex3_45;
				//TZ copied to avoid goto
				dot = ccdVec3Dot(AB, AO);
				if (ccdIsZero(dot) || dot > CCD_ZERO){
					ccdSimplexSet0(simplex, B);
					ccdSimplexSet1(simplex, A);
					ccdSimplexSetSize(simplex, 2);
					tripleCross(AB, AO, AB, dir);
				}else{
					ccdSimplexSet0(simplex, A);
					ccdSimplexSetSize(simplex, 1);
					ccdVec3Copy(dir, AO);
				}
			}else{
				dot = ccdVec3Dot(ABC, AO);
				if (ccdIsZero(dot) || dot > CCD_ZERO){
					ccdVec3Copy(dir, ABC);
				}else{
					ccd_support_t Ctmp = new ccd_support_t();
					ccdSupportCopy(Ctmp, C);
					ccdSimplexSet0(simplex, B);
					ccdSimplexSet1(simplex, Ctmp);

					ccdVec3Copy(dir, ABC);
					ccdVec3Scale(dir, -CCD_ONE);
				}
			}
		}

		return 0;
	}

	/** Returns true if simplex contains origin.
	 *  This function also alteres simplex and dir according to further
	 *  processing of GJK algorithm. */
	private static int doSimplex4(ccd_simplex_t simplex, ccd_vec3_t dir)
	{
		final ccd_support_t A, B, C, D;
		ccd_vec3_t AO = new ccd_vec3_t(), AB = new ccd_vec3_t(), AC = new ccd_vec3_t();
		ccd_vec3_t AD = new ccd_vec3_t();
		ccd_vec3_t ABC = new ccd_vec3_t(), ACD = new ccd_vec3_t(), ADB = new ccd_vec3_t();
		int B_on_ACD, C_on_ADB, D_on_ABC;
		boolean AB_O, AC_O, AD_O;
		double dist;

		// get last added as A
		A = ccdSimplexLast(simplex);
		// get the other points
		B = ccdSimplexPoint2(simplex);
		C = ccdSimplexPoint1(simplex);
		D = ccdSimplexPoint0(simplex);

		// check if tetrahedron is really tetrahedron (has volume > 0)
		// if it is not simplex can't be expanded and thus no intersection is
		// found
		dist = ccdVec3PointTriDist2(A.v, B.v, C.v, D.v, null);
		if (ccdIsZero(dist)){
			return -1;
		}

		// check if origin lies on some of tetrahedron's face - if so objects
		// intersect
		dist = ccdVec3PointTriDist2(ccd_vec3_origin, A.v, B.v, C.v, null);
		if (ccdIsZero(dist))
			return 1;
		dist = ccdVec3PointTriDist2(ccd_vec3_origin, A.v, C.v, D.v, null);
		if (ccdIsZero(dist))
			return 1;
		dist = ccdVec3PointTriDist2(ccd_vec3_origin, A.v, B.v, D.v, null);
		if (ccdIsZero(dist))
			return 1;
		dist = ccdVec3PointTriDist2(ccd_vec3_origin, B.v, C.v, D.v, null);
		if (ccdIsZero(dist))
			return 1;

		// compute AO, AB, AC, AD segments and ABC, ACD, ADB normal vectors
		ccdVec3Copy(AO, A.v);
		ccdVec3Scale(AO, -CCD_ONE);
		ccdVec3Sub2(AB, B.v, A.v);
		ccdVec3Sub2(AC, C.v, A.v);
		ccdVec3Sub2(AD, D.v, A.v);
		ccdVec3Cross(ABC, AB, AC);
		ccdVec3Cross(ACD, AC, AD);
		ccdVec3Cross(ADB, AD, AB);

		// side (positive or negative) of B, C, D relative to planes ACD, ADB
		// and ABC respectively
		B_on_ACD = ccdSign(ccdVec3Dot(ACD, AB));
		C_on_ADB = ccdSign(ccdVec3Dot(ADB, AC));
		D_on_ABC = ccdSign(ccdVec3Dot(ABC, AD));

		// whether origin is on same side of ACD, ADB, ABC as B, C, D
		// respectively
		AB_O = ccdSign(ccdVec3Dot(ACD, AO)) == B_on_ACD;
		AC_O = ccdSign(ccdVec3Dot(ADB, AO)) == C_on_ADB;
		AD_O = ccdSign(ccdVec3Dot(ABC, AO)) == D_on_ABC;

		if (AB_O && AC_O && AD_O){
			// origin is in tetrahedron
			return 1;

			// rearrange simplex to triangle and call doSimplex3()
		}else if (!AB_O){
			// B is farthest from the origin among all of the tetrahedron's
			// points, so remove it from the list and go on with the triangle
			// case

			// D and C are in place
			ccdSimplexSet2(simplex, A);
			ccdSimplexSetSize(simplex, 3);
		}else if (!AC_O){
			// C is farthest
			ccdSimplexSet1(simplex, D);
			ccdSimplexSet0(simplex, B);
			ccdSimplexSet2(simplex, A);
			ccdSimplexSetSize(simplex, 3);
		}else{ // (!AD_O)
			ccdSimplexSet0(simplex, C);
			ccdSimplexSet1(simplex, B);
			ccdSimplexSet2(simplex, A);
			ccdSimplexSetSize(simplex, 3);
		}

		return doSimplex3(simplex, dir);
	}

	/** Returns true if simplex contains origin.
	 *  This function also alteres simplex and dir according to further
	 *  processing of GJK algorithm. */
	private static int doSimplex(ccd_simplex_t simplex, ccd_vec3_t dir)
	{
		if (ccdSimplexSize(simplex) == 2){
			// simplex contains segment only one segment
			return doSimplex2(simplex, dir);
		}else if (ccdSimplexSize(simplex) == 3){
			// simplex contains triangle
			return doSimplex3(simplex, dir);
		}else{ // ccdSimplexSize(simplex) == 4
			// tetrahedron - this is the only shape which can encapsule origin
			// so doSimplex4() also contains test on it
			return doSimplex4(simplex, dir);
		}
	}

	/** d = a x b x c */
	private static void tripleCross(final ccd_vec3_t a, final ccd_vec3_t b,
			final ccd_vec3_t c, ccd_vec3_t d)
	{
		ccd_vec3_t e = new ccd_vec3_t();
		ccdVec3Cross(e, a, b);
		ccdVec3Cross(d, e, c);
	}



	/** Transforms simplex to polytope. It is assumed that simplex has 4
	 *  vertices! */
	static int simplexToPolytope4(final Object obj1, final Object obj2,
			final ccd_t ccd,
			final ccd_simplex_t simplex,
			final ccd_pt_t pt, final Ref<ccd_pt_el_t<?>> nearest) // **nearest)
	{
		final ccd_support_t a, b, c, d;
		int use_polytope3;
		double dist;
		//final ccd_pt_vertex_t[] v = new ccd_pt_vertex_t[4];
		ccd_pt_vertex_t v0, v1, v2, v3;
		final ccd_pt_edge_t[] e = new ccd_pt_edge_t[6];
		//int i;

		a = ccdSimplexPoint0(simplex);
		b = ccdSimplexPoint1(simplex);
		c = ccdSimplexPoint2(simplex);
		d = ccdSimplexPoint3(simplex);

		// check if origin lies on some of tetrahedron's face - if so use
		// simplexToPolytope3()
		use_polytope3 = 0;
		dist = ccdVec3PointTriDist2(ccd_vec3_origin, a.v, b.v, c.v, null);
		if (ccdIsZero(dist)){
			use_polytope3 = 1;
		}
		dist = ccdVec3PointTriDist2(ccd_vec3_origin, a.v, c.v, d.v, null);
		if (ccdIsZero(dist)){
			use_polytope3 = 1;
			ccdSimplexSet1(simplex, c);
			ccdSimplexSet2(simplex, d);
		}
		dist = ccdVec3PointTriDist2(ccd_vec3_origin, a.v, b.v, d.v, null);
		if (ccdIsZero(dist)){
			use_polytope3 = 1;
			ccdSimplexSet2(simplex, d);
		}
		dist = ccdVec3PointTriDist2(ccd_vec3_origin, b.v, c.v, d.v, null);
		if (ccdIsZero(dist)){
			use_polytope3 = 1;
			ccdSimplexSet0(simplex, b);
			ccdSimplexSet1(simplex, c);
			ccdSimplexSet2(simplex, d);
		}

		if (use_polytope3!=0){
			ccdSimplexSetSize(simplex, 3);
			return simplexToPolytope3(obj1, obj2, ccd, simplex, pt, nearest);
		}

		// no touching contact - simply create tetrahedron
//		for (i = 0; i < 4; i++){
//			v[i] = ccdPtAddVertex(pt, ccdSimplexPoint(simplex, i));
//		}
		v0 = ccdPtAddVertex(pt, ccdSimplexPoint0(simplex));
		v1 = ccdPtAddVertex(pt, ccdSimplexPoint1(simplex));
		v2 = ccdPtAddVertex(pt, ccdSimplexPoint2(simplex));
		v3 = ccdPtAddVertex(pt, ccdSimplexPoint3(simplex));

		e[0] = ccdPtAddEdge(pt, v0, v1);
		e[1] = ccdPtAddEdge(pt, v1, v2);
		e[2] = ccdPtAddEdge(pt, v2, v0);
		e[3] = ccdPtAddEdge(pt, v3, v0);
		e[4] = ccdPtAddEdge(pt, v3, v1);
		e[5] = ccdPtAddEdge(pt, v3, v2);

		ccdPtAddFace(pt, e[0], e[1], e[2]);
		ccdPtAddFace(pt, e[3], e[4], e[0]);
		ccdPtAddFace(pt, e[4], e[5], e[1]);
		ccdPtAddFace(pt, e[5], e[3], e[2]);

		return 0;
	}

	/** Transforms simplex to polytope, three vertices required */
	static int simplexToPolytope3(final Object obj1, final Object obj2,
			final ccd_t ccd,
			final ccd_simplex_t simplex,
			final ccd_pt_t pt, final Ref<ccd_pt_el_t<?>> nearest)// **nearest)
	{
		final ccd_support_t a, b, c;
		final ccd_support_t d = new ccd_support_t(), d2 = new ccd_support_t();
		final ccd_vec3_t ab = new ccd_vec3_t(), ac = new ccd_vec3_t(), dir = new ccd_vec3_t();
		final ccd_pt_vertex_t[] v = new ccd_pt_vertex_t[5];
		final ccd_pt_edge_t[] e = new ccd_pt_edge_t[9];
		double dist, dist2;

		nearest.set(null);// = NULL;

		a = ccdSimplexPoint0(simplex);
		b = ccdSimplexPoint1(simplex);
		c = ccdSimplexPoint2(simplex);

		// If only one triangle left from previous GJK run origin lies on this
		// triangle. So it is necessary to expand triangle into two
		// tetrahedrons connected with base (which is exactly abc triangle).

		// get next support point in direction of normal of triangle
		ccdVec3Sub2(ab, b.v, a.v);
		ccdVec3Sub2(ac, c.v, a.v);
		ccdVec3Cross(dir, ab, ac);
		__ccdSupport(obj1, obj2, dir, ccd, d);
		dist = ccdVec3PointTriDist2(d.v, a.v, b.v, c.v, null);

		// and second one take in opposite direction
		ccdVec3Scale(dir, -CCD_ONE);
		__ccdSupport(obj1, obj2, dir, ccd, d2);
		dist2 = ccdVec3PointTriDist2(d2.v, a.v, b.v, c.v, null);

		// check if face isn't already on edge of minkowski sum and thus we
		// have touching contact
		if (ccdIsZero(dist) || ccdIsZero(dist2)){
			v[0] = ccdPtAddVertex(pt, a);
			v[1] = ccdPtAddVertex(pt, b);
			v[2] = ccdPtAddVertex(pt, c);
			e[0] = ccdPtAddEdge(pt, v[0], v[1]);
			e[1] = ccdPtAddEdge(pt, v[1], v[2]);
			e[2] = ccdPtAddEdge(pt, v[2], v[0]);
			nearest.set( ccdPtAddFace(pt, e[0], e[1], e[2]) );

			return -1;
		}

		// form polyhedron
		v[0] = ccdPtAddVertex(pt, a);
		v[1] = ccdPtAddVertex(pt, b);
		v[2] = ccdPtAddVertex(pt, c);
		v[3] = ccdPtAddVertex(pt, d);
		v[4] = ccdPtAddVertex(pt, d2);

		e[0] = ccdPtAddEdge(pt, v[0], v[1]);
		e[1] = ccdPtAddEdge(pt, v[1], v[2]);
		e[2] = ccdPtAddEdge(pt, v[2], v[0]);

		e[3] = ccdPtAddEdge(pt, v[3], v[0]);
		e[4] = ccdPtAddEdge(pt, v[3], v[1]);
		e[5] = ccdPtAddEdge(pt, v[3], v[2]);

		e[6] = ccdPtAddEdge(pt, v[4], v[0]);
		e[7] = ccdPtAddEdge(pt, v[4], v[1]);
		e[8] = ccdPtAddEdge(pt, v[4], v[2]);

		ccdPtAddFace(pt, e[3], e[4], e[0]);
		ccdPtAddFace(pt, e[4], e[5], e[1]);
		ccdPtAddFace(pt, e[5], e[3], e[2]);

		ccdPtAddFace(pt, e[6], e[7], e[0]);
		ccdPtAddFace(pt, e[7], e[8], e[1]);
		ccdPtAddFace(pt, e[8], e[6], e[2]);

		return 0;
	}

	/** Transforms simplex to polytope, two vertices required */
	static int simplexToPolytope2(final Object obj1, final Object obj2,
			final ccd_t ccd,
			final ccd_simplex_t simplex,
			final ccd_pt_t pt, final Ref<ccd_pt_el_t<?>> nearest)// **nearest)
	{
		final ccd_support_t a, b;
		ccd_vec3_t ab = new ccd_vec3_t(), ac = new ccd_vec3_t(), dir = new ccd_vec3_t();
		ccd_support_t[] supp = new ccd_support_t[4];
		for (int i = 0; i < supp.length; i++) supp[i] = new ccd_support_t();
		ccd_pt_vertex_t[] v=new ccd_pt_vertex_t[6];
		ccd_pt_edge_t[] e = new ccd_pt_edge_t[12];
		int i;
		boolean found;

		a = ccdSimplexPoint0(simplex);
		b = ccdSimplexPoint1(simplex);

		// This situation is a bit tricky. If only one segment comes from
		// previous run of GJK - it means that either this segment is on
		// minkowski edge (and thus we have touch contact) or it it isn't and
		// therefore segment is somewhere *inside* minkowski sum and it *must*
		// be possible to fully enclose this segment with polyhedron formed by
		// at least 8 triangle faces.

		// get first support point (any)
		found = false;
		for (i = 0; i < ccd_points_on_sphere_len; i++){
			__ccdSupport(obj1, obj2, ccd_points_on_sphere[i], ccd, supp[0]);
			if (!ccdVec3Eq(a.v, supp[0].v) && !ccdVec3Eq(b.v, supp[0].v)){
				found = true;
				break;
			}
		}
		boolean touching_contact = true; //TZ
		while (true) { //TZ: to simulate goto
			if (!found) {
				//goto simplexToPolytope2_touching_contact;
				break;
			}

			// get second support point in opposite direction than supp[0]
			ccdVec3Copy(dir, supp[0].v);
			ccdVec3Scale(dir, -CCD_ONE);
			__ccdSupport(obj1, obj2, dir, ccd, supp[1]);
			if (ccdVec3Eq(a.v, supp[1].v) || ccdVec3Eq(b.v, supp[1].v)) {
				//goto simplexToPolytope2_touching_contact;
				break;
			}

			// next will be in direction of normal of triangle a,supp[0],supp[1]
			ccdVec3Sub2(ab, supp[0].v, a.v);
			ccdVec3Sub2(ac, supp[1].v, a.v);
			ccdVec3Cross(dir, ab, ac);
			__ccdSupport(obj1, obj2, dir, ccd, supp[2]);
			if (ccdVec3Eq(a.v, supp[2].v) || ccdVec3Eq(b.v, supp[2].v)) {
				//goto simplexToPolytope2_touching_contact;
				break;
			}

			// and last one will be in opposite direction
			ccdVec3Scale(dir, -CCD_ONE);
			__ccdSupport(obj1, obj2, dir, ccd, supp[3]);
			if (ccdVec3Eq(a.v, supp[3].v) || ccdVec3Eq(b.v, supp[3].v)) {
				//goto simplexToPolytope2_touching_contact;
				break;
			}

			//goto simplexToPolytope2_not_touching_contact;
			touching_contact = false;
			break;
		}
		//simplexToPolytope2_touching_contact:
		if (touching_contact) {
			v[0] = ccdPtAddVertex(pt, a);
			v[1] = ccdPtAddVertex(pt, b);
			nearest.set( ccdPtAddEdge(pt, v[0], v[1]) );
			return -1;
		}

		//simplexToPolytope2_not_touching_contact:
		// form polyhedron
		v[0] = ccdPtAddVertex(pt, a);
		v[1] = ccdPtAddVertex(pt, supp[0]);
		v[2] = ccdPtAddVertex(pt, b);
		v[3] = ccdPtAddVertex(pt, supp[1]);
		v[4] = ccdPtAddVertex(pt, supp[2]);
		v[5] = ccdPtAddVertex(pt, supp[3]);

		e[0] = ccdPtAddEdge(pt, v[0], v[1]);
		e[1] = ccdPtAddEdge(pt, v[1], v[2]);
		e[2] = ccdPtAddEdge(pt, v[2], v[3]);
		e[3] = ccdPtAddEdge(pt, v[3], v[0]);

		e[4] = ccdPtAddEdge(pt, v[4], v[0]);
		e[5] = ccdPtAddEdge(pt, v[4], v[1]);
		e[6] = ccdPtAddEdge(pt, v[4], v[2]);
		e[7] = ccdPtAddEdge(pt, v[4], v[3]);

		e[8]  = ccdPtAddEdge(pt, v[5], v[0]);
		e[9]  = ccdPtAddEdge(pt, v[5], v[1]);
		e[10] = ccdPtAddEdge(pt, v[5], v[2]);
		e[11] = ccdPtAddEdge(pt, v[5], v[3]);

		ccdPtAddFace(pt, e[4], e[5], e[0]);
		ccdPtAddFace(pt, e[5], e[6], e[1]);
		ccdPtAddFace(pt, e[6], e[7], e[2]);
		ccdPtAddFace(pt, e[7], e[4], e[3]);

		ccdPtAddFace(pt, e[8],  e[9],  e[0]);
		ccdPtAddFace(pt, e[9],  e[10], e[1]);
		ccdPtAddFace(pt, e[10], e[11], e[2]);
		ccdPtAddFace(pt, e[11], e[8],  e[3]);

		return 0;
	}

	/** Expands polytope's tri by new vertex v. Triangle tri is replaced by
	 *  three triangles each with one vertex in v. */
	static void expandPolytope(ccd_pt_t pt, final ccd_pt_el_t<?> el,
			final ccd_support_t newv)
	{
		ccd_pt_vertex_t[] v = new ccd_pt_vertex_t[5];
		ccd_pt_edge_t[] e = new ccd_pt_edge_t[8];
		ccd_pt_face_t[] f = new ccd_pt_face_t[2];


		// element can be either segment or triangle
		if (el.type == CCD_PT_EDGE){
			// In this case, segment should be replaced by new point.
			// Simpliest case is when segment stands alone and in this case
			// this segment is replaced by two other segments both connected to
			// newv.
			// Segment can be also connected to max two faces and in that case
			// each face must be replaced by two other faces. To do this
			// correctly it is necessary to have correctly ordered edges and
			// vertices which is exactly what is done in following code.
			//

			ccdPtEdgeVertices((ccd_pt_edge_t )el, v, 0, 2);//v[0], v[2]);

			ccdPtEdgeFaces((ccd_pt_edge_t )el, f, 0, 1);//[0], f[1]);

			if (f[0]!=null){
				ccdPtFaceEdges(f[0], e, 0, 1, 2);//[0], e[1], e[2]);
				if (e[0] == (ccd_pt_edge_t )el){
					e[0] = e[2];
				}else if (e[1] == (ccd_pt_edge_t )el){
					e[1] = e[2];
				}
				ccdPtEdgeVertices(e[0], v, 1, 3);//[1], v[3]);
				if (v[1] != v[0] && v[3] != v[0]){
					e[2] = e[0];
					e[0] = e[1];
					e[1] = e[2];
					if (v[1] == v[2])
						v[1] = v[3];
				}else{
					if (v[1] == v[0])
						v[1] = v[3];
				}

				if (f[1]!=null){
					ccdPtFaceEdges(f[1], e, 2, 3, 4);//[2], e[3], e[4]);
					if (e[2] == (ccd_pt_edge_t )el){
						e[2] = e[4];
					}else if (e[3] == (ccd_pt_edge_t )el){
						e[3] = e[4];
					}
					ccdPtEdgeVertices(e[2], v, 3, 4);//[3], v[4]);
					if (v[3] != v[2] && v[4] != v[2]){
						e[4] = e[2];
						e[2] = e[3];
						e[3] = e[4];
						if (v[3] == v[0])
							v[3] = v[4];
					}else{
						if (v[3] == v[2])
							v[3] = v[4];
					}
				}


				v[4] = ccdPtAddVertex(pt, newv);

				ccdPtDelFace(pt, f[0]);
				if (f[1]!=null){
					ccdPtDelFace(pt, f[1]);
					ccdPtDelEdge(pt, (ccd_pt_edge_t )el);
				}

				e[4] = ccdPtAddEdge(pt, v[4], v[2]);
				e[5] = ccdPtAddEdge(pt, v[4], v[0]);
				e[6] = ccdPtAddEdge(pt, v[4], v[1]);
				if (f[1]!=null)
					e[7] = ccdPtAddEdge(pt, v[4], v[3]);

				ccdPtAddFace(pt, e[1], e[4], e[6]);
				ccdPtAddFace(pt, e[0], e[6], e[5]);
				if (f[1]!=null){
					ccdPtAddFace(pt, e[3], e[5], e[7]);
					ccdPtAddFace(pt, e[4], e[7], e[2]);
				}else{
					ccdPtAddFace(pt, e[4], e[5], (ccd_pt_edge_t )el);
				}
			}
		}else{ // el.type == CCD_PT_FACE
			// replace triangle by tetrahedron without base (base would be the
			// triangle that will be removed)

			// get triplet of surrounding edges and vertices of triangle face
			ccdPtFaceEdges((ccd_pt_face_t )el, e, 0, 1, 2);//[0], e[1], e[2]);
			ccdPtEdgeVertices(e[0], v, 0, 1);//[0], v[1]);
			ccdPtEdgeVertices(e[1], v, 2, 3);//[2], v[3]);

			// following code sorts edges to have e[0] between vertices 0-1,
			// e[1] between 1-2 and e[2] between 2-0
			if (v[2] != v[1] && v[3] != v[1]){
				// swap e[1] and e[2] 
				e[3] = e[1];
				e[1] = e[2];
				e[2] = e[3];
			}
			if (v[3] != v[0] && v[3] != v[1])
				v[2] = v[3];

			// remove triangle face
			ccdPtDelFace(pt, (ccd_pt_face_t )el);

			// expand triangle to tetrahedron
			v[3] = ccdPtAddVertex(pt, newv);
			e[3] = ccdPtAddEdge(pt, v[3], v[0]);
			e[4] = ccdPtAddEdge(pt, v[3], v[1]);
			e[5] = ccdPtAddEdge(pt, v[3], v[2]);

			ccdPtAddFace(pt, e[3], e[4], e[0]);
			ccdPtAddFace(pt, e[4], e[5], e[1]);
			ccdPtAddFace(pt, e[5], e[3], e[2]);
		}
	}

	/** Finds next support point (at stores it in out argument).
	 *  Returns 0 on success, -1 otherwise */
	static int nextSupport(final Object obj1, final Object obj2, final ccd_t ccd,
			final ccd_pt_el_t<?> el,
			final ccd_support_t out)
	{
		ccd_vec3_t a, b, c;
		double dist;

		if (el.type == CCD_PT_VERTEX)
			return -1;

		// touch contact
		if (ccdIsZero(el.dist))
			return -1;

		__ccdSupport(obj1, obj2, el.witness, ccd, out);

		if (el.type == CCD_PT_EDGE){
			// fetch end points of edge
			//ccdPtEdgeVec3((ccd_pt_edge_t )el, a, b);
			ccd_pt_edge_t e = (ccd_pt_edge_t) el;
			a = e.vertex0.v.v;
			b = e.vertex1.v.v;

			// get distance from segment
			dist = ccdVec3PointSegmentDist2(out.v, a, b, null);
		}else{ // el.type == CCD_PT_FACE
			// fetch vertices of triangle face
			//ccdPtFaceVec3((ccd_pt_face_t )el, a, b, c);
			ccd_pt_face_t face = (ccd_pt_face_t) el;
			a = face.edge0.vertex0.v.v;
			b = face.edge0.vertex1.v.v;

			if (face.edge1.vertex0 != face.edge0.vertex0
					&& face.edge1.vertex0 != face.edge0.vertex1){
				c = face.edge1.vertex0.v.v;
			}else{
				c = face.edge1.vertex1.v.v;
			}

			// check if new point can significantly expand polytope
			dist = ccdVec3PointTriDist2(out.v, a, b, c, null);
		}

		if (dist < ccd.epa_tolerance)
			return -1;

		return 0;
	}

	private CCD() {}
}
