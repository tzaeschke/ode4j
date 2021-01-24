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

import org.ode4j.ode.DColliderFn;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSphere;

import static org.ode4j.ode.OdeMath.*;


/**
 * standard ODE geometry primitives: public API and pairwise collision functions.
 * 
 * the rule is that only the low level primitive collision functions should set
 * dContactGeom::g1 and dContactGeom::g2.
 */
public class DxSphere extends DxGeom implements DSphere {

	private double _radius;		// sphere radius


	//****************************************************************************
	// sphere public API

	//	dxSphere (dSpace space, double _radius)// : dxGeom (space,1)
	DxSphere (DxSpace space, double radius)
	{
		super(space, true);
		dAASSERT (radius >= 0);
		type = dSphereClass;
		_radius = radius;
		updateZeroSizedFlag(radius == 0.0);
	}


	@Override
    protected void computeAABB()
	{
//		_aabb.v[0] = _final_posr.pos.v[0] - _radius;
//		_aabb.v[1] = _final_posr.pos.v[0] + _radius;
//		_aabb.v[2] = _final_posr.pos.v[1] - _radius;
//		_aabb.v[3] = _final_posr.pos.v[1] + _radius;
//		_aabb.v[4] = _final_posr.pos.v[2] - _radius;
//		_aabb.v[5] = _final_posr.pos.v[2] + _radius;
		_aabb.set(final_posr().pos().get0() - _radius,
				final_posr().pos().get0() + _radius,
				final_posr().pos().get1() - _radius,
				final_posr().pos().get1() + _radius,
				final_posr().pos().get2() - _radius,
				final_posr().pos().get2() + _radius);

	}


	//dGeom dCreateSphere (dSpace space, double radius)
	public static DxSphere dCreateSphere (DxSpace space, double radius)
	{
		return new DxSphere (space,radius);
	}


	//void dGeomSphereSetRadius (dGeom g, double radius)
	public void dGeomSphereSetRadius (double radius)
	{
		//  dUASSERT (g && g.type == dSphereClass,"argument not a sphere");
		dAASSERT (radius >= 0);
		//  dxSphere s = (dxSphere) g;
		_radius = radius;
		updateZeroSizedFlag(radius == 0);
		dGeomMoved ();
	}


	//double dGeomSphereGetRadius (dGeom g)
	public double dGeomSphereGetRadius ()
	{
		//dUASSERT (g && g.type == dSphereClass,"argument not a sphere");
		//dxSphere s = (dxSphere) g;
		return _radius;
	}


	//double dGeomSpherePointDepth (dGeom g, double x, double y, double z)
	public double dGeomSpherePointDepth (double x, double y, double z)
	{
		//dUASSERT (g && g.type == dSphereClass,"argument not a sphere");
		recomputePosr();

		//dxSphere s = (dxSphere) g;
		//  double [] pos = final_posr.pos.v;
		//  return _radius - dSqrt ((x-pos[0])*(x-pos[0]) +
		//			    (y-pos[1])*(y-pos[1]) +
		//			    (z-pos[2])*(z-pos[2]));
		DVector3 pos = new DVector3(x, y, z);
		pos.sub(final_posr().pos());
		return _radius - pos.length();
	}
	public double dGeomSpherePointDepth (DVector3C xyz)
	{
		recomputePosr();
		DVector3 pos = new DVector3();
		pos.eqDiff(xyz, final_posr().pos());
		return _radius - pos.length();
	}

	//****************************************************************************
	// pairwise collision functions for standard geom types

	static class CollideSphereSphere implements DColliderFn {
		//int dCollideSphereSphere (dxGeom *o1, dxGeom *o2, int flags,
		//			  dContactGeom *contact, int skip)
		int dCollideSphereSphere (DxSphere sphere1, DxSphere sphere2, int flags,
				DContactGeomBuffer contacts, int skip)
		{
			//dIASSERT (skip >= (int)sizeof(dContactGeom));
			//TZ
			dIASSERT(skip ==1);
			//  dIASSERT (o1.type == dSphereClass);
			//  dIASSERT (o2.type == dSphereClass);
			dIASSERT ((flags & NUMC_MASK) >= 1);

			//  dxSphere sphere1 = (dxSphere) o1;
			//  dxSphere sphere2 = (dxSphere) o2;

			DContactGeom contact = contacts.get(0);
			contact.g1 = sphere1;
			contact.g2 = sphere2;
			contact.side1 = -1;
			contact.side2 = -1;

			return DxCollisionUtil.dCollideSpheres (sphere1.final_posr().pos(), sphere1._radius,
					sphere2.final_posr().pos(), sphere2._radius, contacts);
		}

		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return dCollideSphereSphere((DxSphere)o1, (DxSphere)o2, flags, contacts, 1);
		}
	}

	static class CollideSphereBox implements DColliderFn {
		//int dCollideSphereBox (dxGeom *o1, dxGeom *o2, int flags,
		//		       dContactGeom *contact, int skip)
		int dCollideSphereBox (DxSphere o1, DxBox o2, int flags,
				DContactGeomBuffer contacts, int skip)
		{
			//dIASSERT (skip >= (int)sizeof(dContactGeom));
			//TZ
			dIASSERT(skip ==1);
			//  dIASSERT (o1.type == dSphereClass);
			//  dIASSERT (o2.type == dBoxClass);
			dIASSERT ((flags & NUMC_MASK) >= 1);

			// this is easy. get the sphere center `p' relative to the box, and then clip
			// that to the boundary of the box (call that point `q'). if q is on the
			// boundary of the box and |p-q| is <= sphere radius, they touch.
			// if q is inside the box, the sphere is inside the box, so set a contact
			// normal to push the sphere to the closest box face.

			double[] l = new double[3],t = new double[3];
			DVector3 p = new DVector3(), q = new DVector3(),r = new DVector3();
			double depth;
			boolean onborder = false;

			DxSphere sphere = o1;
			DxBox box = o2;

			DContactGeom contact = contacts.get(0);
			contact.g1 = o1;
			contact.g2 = o2;
			contact.side1 = -1;
			contact.side2 = -1;

			//p.set(o1._final_posr.pos).sub(o2._final_posr.pos);
			p.eqDiff(o1.final_posr().pos(), o2.final_posr().pos());

			l[0] = box.side.get0()*(0.5);
			t[0] = dCalcVectorDot3_14(p,o2.final_posr().R(),0);
			if (t[0] < -l[0]) { t[0] = -l[0]; onborder = true; }
			if (t[0] >  l[0]) { t[0] =  l[0]; onborder = true; }

			l[1] = box.side.get1()*(0.5);
			t[1] = dCalcVectorDot3_14(p,o2.final_posr().R(),1);
			if (t[1] < -l[1]) { t[1] = -l[1]; onborder = true; }
			if (t[1] >  l[1]) { t[1] =  l[1]; onborder = true; }

			t[2] = dCalcVectorDot3_14(p,o2.final_posr().R(),2);
			l[2] = box.side.get2()*(0.5);
			if (t[2] < -l[2]) { t[2] = -l[2]; onborder = true; }
			if (t[2] >  l[2]) { t[2] =  l[2]; onborder = true; }

			if (!onborder) {
				// sphere center inside box. find closest face to `t'
				double min_distance = l[0] - dFabs(t[0]);
				int mini = 0;
				for (int i=1; i<3; i++) {
					double face_distance = l[i] - dFabs(t[i]);
					if (face_distance < min_distance) {
						min_distance = face_distance;
						mini = i;
					}
				}
				// contact position = sphere center
				contacts.get(0).pos.set(o1.final_posr().pos());

				// contact normal points to closest face
				DVector3 tmp = new DVector3();
				//    tmp[0] = 0;
				//    tmp[1] = 0;
				//    tmp[2] = 0;
				tmp.set(mini, (t[mini] > 0) ? (1.0) : (-1.0) );
				dMultiply0_331 (contacts.get(0).normal,o2.final_posr().R(),tmp);
				// contact depth = distance to wall along normal plus radius
				contacts.get(0).depth = min_distance + sphere._radius;
				return 1;
			}

			//t[3] = 0;			//@@@ hmmm  TODO (TZ) report
			dMultiply0_331 (q,o2.final_posr().R(),new DVector3(t[0], t[1], t[2]));
			//  r[0] = p[0] - q[0];
			//  r[1] = p[1] - q[1];
			//  r[2] = p[2] - q[2];
			r.eqDiff(p, q);
			depth = sphere._radius - dSqrt(r.dot(r));
			if (depth < 0) return 0;
			//  contact.pos[0] = q[0] + o2.final_posr.pos[0];
			//  contact.pos[1] = q[1] + o2.final_posr.pos[1];
			//  contact.pos[2] = q[2] + o2.final_posr.pos[2];
			contacts.get(0).pos.eqSum(q, o2.final_posr().pos());
			//  contact.normal[0] = r[0];
			//  contact.normal[1] = r[1];
			//  contact.normal[2] = r[2];
			contacts.get(0).normal.set(r);
			dNormalize3 (contacts.get(0).normal);
			contacts.get(0).depth = depth;
			return 1;
		}

		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return dCollideSphereBox((DxSphere)o1, (DxBox)o2, flags, contacts, 1);
		}
	}

	static class CollideSpherePlane implements DColliderFn {
		//int dCollideSpherePlane (dxGeom *o1, dxGeom *o2, int flags,
		//			 dContactGeom *contact, int skip)
		int dCollideSpherePlane (DxSphere o1, DxPlane o2, int flags,
				DContactGeomBuffer contacts, int skip)
		{
			//dIASSERT (skip >= (int)sizeof(dContactGeom));
			//TZ
			dIASSERT(skip == 1);
			//  dIASSERT (o1.type == dSphereClass);
			//  dIASSERT (o2.type == dPlaneClass);
			dIASSERT ((flags & NUMC_MASK) >= 1);

			DxSphere sphere = o1;
			DxPlane plane = o2;

			DContactGeom contact = contacts.get(0);
			contact.g1 = o1;
			contact.g2 = o2;
			contact.side1 = -1;
			contact.side2 = -1;

			double k = o1.final_posr().pos().dot( plane.getNormal() );//dDOT (o1._final_posr.pos,plane.getPlanePos());
			double depth = plane.getDepth() - k + sphere._radius;
			if (depth >= 0) {
				//    contact.normal[0] = plane.p[0];
				//    contact.normal[1] = plane.p[1];
				//    contact.normal[2] = plane.p[2];
				contact.normal.set(plane.getNormal());
				//			contact.pos[0] = o1.final_posr.pos[0] - plane.p[0] * sphere._radius;
				//			contact.pos[1] = o1.final_posr.pos[1] - plane.p[1] * sphere._radius;
				//			contact.pos[2] = o1.final_posr.pos[2] - plane.p[2] * sphere._radius;
				//contact.pos.set(plane._p).scale(-sphere._radius).add(o1._final_posr.pos);
				contact.pos.eqSum(o1.final_posr().pos(), contact.normal /* == plane._p */, -sphere._radius);
				contact.depth = depth;
				return 1;
			}
			else return 0;
		}

		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return dCollideSpherePlane((DxSphere)o1, (DxPlane)o2, flags, contacts, 1);
		}
	}

	// ******************************************
	// API dSphere
	// ******************************************


	@Override
	public void setRadius (double radius)
	{ dGeomSphereSetRadius (radius); }
	@Override
	public double getRadius()
	{ return dGeomSphereGetRadius (); }


	@Override
	public double getPointDepth(DVector3C p) {
		return dGeomSpherePointDepth(p);
	}

}
