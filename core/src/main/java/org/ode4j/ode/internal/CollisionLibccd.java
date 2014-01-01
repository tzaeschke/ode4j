/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Z�schke      *
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

import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DColliderFn;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;
import org.ode4j.ode.internal.libccd.CCD.ccd_center_fn;
import org.ode4j.ode.internal.libccd.CCD.ccd_support_fn;
import org.ode4j.ode.internal.libccd.CCDVec3.ccd_vec3_t;

import static org.ode4j.ode.internal.libccd.CCD.*;
import static org.ode4j.ode.internal.libccd.CCDMPR.*;
import static org.ode4j.ode.internal.libccd.CCDQuat.*;
import static org.ode4j.ode.internal.libccd.CCDVec3.*;

/**
 * Lib ccd.
 *
 * @author Tilmann Zaeschke
 */
public class CollisionLibccd {

	private static class ccd_obj_t {
		final ccd_vec3_t pos = new ccd_vec3_t();
		final ccd_quat_t rot = new ccd_quat_t(), rot_inv = new ccd_quat_t();
	};
	//typedef struct _ccd_obj_t ccd_obj_t;

	private static class ccd_box_t extends ccd_obj_t {
		//ccd_obj_t o;
		double[] dim = new double[3];
	};
	//typedef struct _ccd_box_t ccd_box_t;

	private static class ccd_cap_t extends ccd_obj_t {
		//ccd_obj_t o;
		double radius, height;
	};
	//typedef struct _ccd_cap_t ccd_cap_t;

	private static class ccd_cyl_t extends ccd_obj_t {
		//ccd_obj_t o;
		double radius, height;
	};
	//typedef struct _ccd_cyl_t ccd_cyl_t;

	private static class ccd_sphere_t extends ccd_obj_t {
		//ccd_obj_t o;
		double radius;
	};
	//typedef struct _ccd_sphere_t ccd_sphere_t;

	private static class ccd_convex_t extends ccd_obj_t {
		//ccd_obj_t o;
		DxConvex convex;
	};
	//typedef struct _ccd_convex_t ccd_convex_t;

	/** Transforms geom to ccd struct */
	//    static void ccdGeomToObj(const dGeomID g, ccd_obj_t *);
	//    static void ccdGeomToBox(const dGeomID g, ccd_box_t *);
	//    static void ccdGeomToCap(const dGeomID g, ccd_cap_t *);
	//    static void ccdGeomToCyl(const dGeomID g, ccd_cyl_t *);
	//    static void ccdGeomToSphere(const dGeomID g, ccd_sphere_t *);
	//    static void ccdGeomToConvex(const dGeomID g, ccd_convex_t *);

	/** Support functions */
	//    static void ccdSupportBox(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v);
	//    static void ccdSupportCap(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v);
	//    static void ccdSupportCyl(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v);
	//    static void ccdSupportSphere(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v);
	//    static void ccdSupportConvex(const void *obj, const ccd_vec3_t *_dir, ccd_vec3_t *v);

	/** Center function */
	//    static void ccdCenter(const void *obj, ccd_vec3_t *c);

	/** General collide function */
	//    static int ccdCollide(dGeomID o1, dGeomID o2, int flags,
	//                          dContactGeom *contact, int skip,
	//                          void *obj1, ccd_support_fn supp1, ccd_center_fn cen1,
	//                          void *obj2, ccd_support_fn supp2, ccd_center_fn cen2);



	static void ccdGeomToObj(final DGeom g, ccd_obj_t o)
	{
		DVector3C ode_pos;
		DQuaternionC ode_rot;

		ode_pos = g.getPosition();
		ode_rot = g.getQuaternion();

		ccdVec3Set(o.pos, ode_pos);
		ccdQuatSet(o.rot, ode_rot.get1(), ode_rot.get2(), ode_rot.get3(), ode_rot.get0());

		ccdQuatInvert2(o.rot_inv, o.rot);
	}

	static void ccdGeomToBox(final DxBox g, ccd_box_t box)
	{
		DVector3 dim = new DVector3();

		ccdGeomToObj(g, box);

		g.getLengths(dim);
		box.dim[0] = dim.get0() / 2.;
		box.dim[1] = dim.get1() / 2.;
		box.dim[2] = dim.get2() / 2.;
	}

	static void ccdGeomToCap(final DxCapsule g, ccd_cap_t cap)
	{
		//RefDouble r, h;
		ccdGeomToObj(g, cap);

		//dGeomCapsuleGetParams(g, r, h);
		cap.radius = g.getRadius();
		cap.height = g.getLength() / 2.;
	}

	static void ccdGeomToCyl(final DxCylinder g, ccd_cyl_t cyl)
	{
		//double r, h;
		ccdGeomToObj(g, cyl);

		//dGeomCylinderGetParams(g, &r, &h);
		cyl.radius = g.getRadius();
		cyl.height = g.getLength() / 2.;
	}

	static void ccdGeomToSphere(final DxSphere g, ccd_sphere_t s)
	{
		ccdGeomToObj(g, s);
		s.radius = g.getRadius();
	}

	static void ccdGeomToConvex(final DxConvex g, ccd_convex_t c)
	{
		ccdGeomToObj(g, c);
		c.convex = g;
	}


	private static final ccd_support_fn ccdSupportBox = new ccd_support_fn() {
		@Override
		public void run(Object obj, ccd_vec3_t _dir, ccd_vec3_t v) {
			final ccd_box_t o = (ccd_box_t)obj;
			final ccd_vec3_t dir = new ccd_vec3_t();

			ccdVec3Copy(dir, _dir);
			ccdQuatRotVec(dir, o.rot_inv);

			ccdVec3Set(v, ccdSign(ccdVec3X(dir)) * o.dim[0],
					ccdSign(ccdVec3Y(dir)) * o.dim[1],
					ccdSign(ccdVec3Z(dir)) * o.dim[2]);

			// transform support vertex
			ccdQuatRotVec(v, o.rot);
			ccdVec3Add(v, o.pos);
		}
	};

	private static final ccd_support_fn ccdSupportCap = new ccd_support_fn() {
		@Override
		public void run(Object obj, ccd_vec3_t _dir, ccd_vec3_t v) {
			final ccd_cap_t o = (ccd_cap_t)obj;
			final ccd_vec3_t dir = new ccd_vec3_t(), pos1 = new ccd_vec3_t(), pos2 = new ccd_vec3_t();

			ccdVec3Copy(dir, _dir);
			ccdQuatRotVec(dir, o.rot_inv);

			ccdVec3Set(pos1, CCD_ZERO, CCD_ZERO, o.height);
			ccdVec3Set(pos2, CCD_ZERO, CCD_ZERO, -o.height);

			ccdVec3Copy(v, dir);
			ccdVec3Scale(v, o.radius);
			ccdVec3Add(pos1, v);
			ccdVec3Add(pos2, v);

			if (ccdVec3Dot(dir, pos1) > ccdVec3Dot(dir, pos2)){
				ccdVec3Copy(v, pos1);
			}else{
				ccdVec3Copy(v, pos2);
			}

			// transform support vertex
			ccdQuatRotVec(v, o.rot);
			ccdVec3Add(v, o.pos);
		}
	};

	private static final ccd_support_fn ccdSupportCyl = new ccd_support_fn() {
		@Override
		public void run(Object obj, ccd_vec3_t _dir, ccd_vec3_t v) {
			final ccd_cyl_t cyl = (ccd_cyl_t)obj;
			final ccd_vec3_t dir = new ccd_vec3_t();
			double zdist, rad;

			ccdVec3Copy(dir, _dir);
			ccdQuatRotVec(dir, cyl.rot_inv);

			zdist = dir.get0() * dir.get0() + dir.get1() * dir.get1();
			zdist = Math.sqrt(zdist);
			if (ccdIsZero(zdist)){
				ccdVec3Set(v, 0., 0., ccdSign(ccdVec3Z(dir)) * cyl.height);
			}else{
				rad = cyl.radius / zdist;

				ccdVec3Set(v, rad * ccdVec3X(dir),
						rad * ccdVec3Y(dir),
						ccdSign(ccdVec3Z(dir)) * cyl.height);
			}

			// transform support vertex
			ccdQuatRotVec(v, cyl.rot);
			ccdVec3Add(v, cyl.pos);
		}
	};

	private static final ccd_support_fn ccdSupportSphere = new ccd_support_fn() {
		@Override
		public void run(Object obj, ccd_vec3_t _dir, ccd_vec3_t v) {
			final ccd_sphere_t s = (ccd_sphere_t )obj;
			final ccd_vec3_t dir = new ccd_vec3_t();

			ccdVec3Copy(dir, _dir);
			ccdQuatRotVec(dir, s.rot_inv);

			ccdVec3Copy(v, dir);
			ccdVec3Scale(v, s.radius);
			ccdVec3Scale(v, CCD_ONE / CCD_SQRT(ccdVec3Len2(dir)));

			// transform support vertex
			ccdQuatRotVec(v, s.rot);
			ccdVec3Add(v, s.pos);
		}
	};

	private static final ccd_support_fn ccdSupportConvex = new ccd_support_fn() {
		@Override
		public void run(Object obj, ccd_vec3_t _dir, ccd_vec3_t v) {
			final ccd_convex_t c = (ccd_convex_t )obj;
			final ccd_vec3_t dir = new ccd_vec3_t(), p = new ccd_vec3_t();
			double maxdot, dot;
			int i;
			double []curp;

			ccdVec3Copy(dir, _dir);
			ccdQuatRotVec(dir, c.rot_inv);

			maxdot = -CCD_REAL_MAX;
			curp = c.convex.getPoints();
			int curpI = 0;
			for (i = 0; i < c.convex.getPointcount(); i++, curpI += 3){
				ccdVec3Set(p, curp[curpI+0], curp[curpI+1], curp[curpI+2]);
				dot = ccdVec3Dot(dir, p);
				if (dot > maxdot){
					ccdVec3Copy(v, p);
					maxdot = dot;
				}
			}


			// transform support vertex
			ccdQuatRotVec(v, c.rot);
			ccdVec3Add(v, c.pos);
		}
	}; 

	private static final ccd_center_fn ccdCenter = new ccd_center_fn() {
		@Override
		public void run(Object obj1, ccd_vec3_t c) {
			final ccd_obj_t o = (ccd_obj_t)obj1;
			ccdVec3Copy(c, o.pos);
		}
	};


	static int ccdCollide(DGeom o1, DGeom o2, int flags,
			DContactGeomBuffer contacts,
			ccd_obj_t obj1, ccd_support_fn supp1, ccd_center_fn cen1,
			ccd_obj_t obj2, ccd_support_fn supp2, ccd_center_fn cen2)
	{
		ccd_t ccd = new ccd_t();
		int res;
		final RefDouble depth = new RefDouble();
		final ccd_vec3_t dir = new ccd_vec3_t(), pos = new ccd_vec3_t();
		int max_contacts = (flags & 0xffff);

		if (max_contacts < 1)
			return 0;

		CCD_INIT(ccd);
		ccd.support1 = supp1;
		ccd.support2 = supp2;
		ccd.center1  = cen1;
		ccd.center2  = cen2;
		ccd.max_iterations = 500;
		ccd.mpr_tolerance = 1E-6;


		if ((flags & OdeConstants.CONTACTS_UNIMPORTANT)!=0){
			if (ccdMPRIntersect(obj1, obj2, ccd)!=0){
				return 1;
			}else{
				return 0;
			}
		}

		res = ccdMPRPenetration(obj1, obj2, ccd, depth, dir, pos);
		if (res == 0) {
			DContactGeom contact = contacts.get();
			contact.g1 = o1;
			contact.g2 = o2;

			contact.side1 = contact.side2 = -1;

			contact.depth = depth.get();

			contact.pos.set( ccdVec3X(pos), ccdVec3Y(pos), ccdVec3Z(pos) );

			ccdVec3Scale(dir, -1.);
			contact.normal.set( ccdVec3X(dir), ccdVec3Y(dir), ccdVec3Z(dir));

			return 1;
		}

		return 0;
	}

	public static class CollideCylinderCylinder implements DColliderFn {
		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			final ccd_cyl_t cyl1 = new ccd_cyl_t(), cyl2 = new ccd_cyl_t();

			ccdGeomToCyl((DxCylinder) o1, cyl1);
			ccdGeomToCyl((DxCylinder) o2, cyl2);

			return ccdCollide(o1, o2, flags, contacts,
					cyl1, ccdSupportCyl, ccdCenter,
					cyl2, ccdSupportCyl, ccdCenter);
		}
	}; 

	public static class CollideBoxCylinderCCD implements DColliderFn {
		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			ccd_cyl_t cyl = new ccd_cyl_t();
			ccd_box_t box = new ccd_box_t();
			ccdGeomToBox((DxBox) o1, box);
			ccdGeomToCyl((DxCylinder) o2, cyl);

			return ccdCollide(o1, o2, flags, contacts,
					box, ccdSupportBox, ccdCenter,
					cyl, ccdSupportCyl, ccdCenter);
		}
	};

	public static class CollideCapsuleCylinder implements DColliderFn {
		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			ccd_cap_t cap = new ccd_cap_t();
			ccd_cyl_t cyl = new ccd_cyl_t();

			ccdGeomToCap((DxCapsule) o1, cap);
			ccdGeomToCyl((DxCylinder) o2, cyl);

			return ccdCollide(o1, o2, flags, contacts,
					cap, ccdSupportCap, ccdCenter,
					cyl, ccdSupportCyl, ccdCenter);
		}
	}; 

	public static class CollideConvexBoxCCD implements DColliderFn {
		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			ccd_box_t box = new ccd_box_t();
			ccd_convex_t conv = new ccd_convex_t();
	
			ccdGeomToConvex((DxConvex) o1, conv);
			ccdGeomToBox((DxBox) o2, box);
	
			return ccdCollide(o1, o2, flags, contacts,
					conv, ccdSupportConvex, ccdCenter,
					box, ccdSupportBox, ccdCenter);
		}
	};

	public static class CollideConvexCapsuleCCD implements DColliderFn {
		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			ccd_cap_t cap = new ccd_cap_t();
			ccd_convex_t conv = new ccd_convex_t();

			ccdGeomToConvex((DxConvex) o1, conv);
			ccdGeomToCap((DxCapsule) o2, cap);

			return ccdCollide(o1, o2, flags, contacts,
					conv, ccdSupportConvex, ccdCenter,
					cap, ccdSupportCap, ccdCenter);
		}
	};

	public static class CollideConvexSphereCCD implements DColliderFn {
		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			ccd_sphere_t sphere = new ccd_sphere_t();
			ccd_convex_t conv = new ccd_convex_t();

			ccdGeomToConvex((DxConvex) o1, conv);
			ccdGeomToSphere((DxSphere) o2, sphere);

			return ccdCollide(o1, o2, flags, contacts,
					conv, ccdSupportConvex, ccdCenter,
					sphere, ccdSupportSphere, ccdCenter);
		}
	};

	public static class CollideConvexCylinderCCD implements DColliderFn {
		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			ccd_cyl_t cyl = new ccd_cyl_t();
			ccd_convex_t conv = new ccd_convex_t();

			ccdGeomToConvex((DxConvex) o1, conv);
			ccdGeomToCyl((DxCylinder) o2, cyl);

			return ccdCollide(o1, o2, flags, contacts,
					conv, ccdSupportConvex, ccdCenter,
					cyl, ccdSupportCyl, ccdCenter);
		}
	};

	public static class CollideConvexConvexCCD implements DColliderFn {
		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			ccd_convex_t c1 = new ccd_convex_t(), c2 = new ccd_convex_t();

			ccdGeomToConvex((DxConvex) o1, c1);
			ccdGeomToConvex((DxConvex) o2, c2);

			return ccdCollide(o1, o2, flags, contacts,
					c1, ccdSupportConvex, ccdCenter,
					c2, ccdSupportConvex, ccdCenter);
		}
	};

}
