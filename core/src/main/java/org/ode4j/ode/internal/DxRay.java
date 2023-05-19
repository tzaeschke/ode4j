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
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DRay;

import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.OdeMath.*;


/**
 * standard ODE geometry primitives: public API and pairwise collision functions.
 * 
 * the rule is that only the low level primitive collision functions should set
 * dContactGeom::g1 and dContactGeom::g2.
 */
public class DxRay extends DxGeom implements DRay {

	// Ray specific
	private final int RAY_FIRSTCONTACT = 0x10000;
	private final int RAY_BACKFACECULL = 0x20000;
	private final int RAY_CLOSEST_HIT  = 0x40000;



	private double _length;


	//****************************************************************************
	// ray public API

	//	dxRay::dxRay (dSpace space, dReal _length) : dxGeom (space,1)
	DxRay (DxSpace space, double length) //: dxGeom (space,1)
	{
		super(space, true);
		type = dRayClass;
		_length = length;
	}


	@Override
	protected void computeAABB()
	{
		DVector3 e = new DVector3();
		//  e[0] = final_posr.pos[0] + final_posr.R[0*4+2]*length;
		//  e[1] = final_posr.pos[1] + final_posr.R[1*4+2]*length;
		//  e[2] = final_posr.pos[2] + final_posr.R[2*4+2]*length;
		e.eqSum(final_posr().R().viewCol(2), _length, final_posr().pos(), 1);

		_aabb.setMinMax( final_posr().pos(),  e);
//		if (_final_posr.pos.get0() < e.get0()){
//			_aabb.v[0] = _final_posr.pos.get0();
//			_aabb.v[1] = e.get0();
//		}
//		else{
//			_aabb.v[0] = e.get0();
//			_aabb.v[1] = _final_posr.pos.get0();
//		}
//
//		if (_final_posr.pos.get1() < e.get1()){
//			_aabb.v[2] = _final_posr.pos.get1();
//			_aabb.v[3] = e.get1();
//		}
//		else{
//			_aabb.v[2] = e.get1();
//			_aabb.v[3] = _final_posr.pos.get1();
//		}
//
//		if (_final_posr.pos.get2() < e.get2()){
//			_aabb.v[4] = _final_posr.pos.get2();
//			_aabb.v[5] = e.get2();
//		}
//		else{
//			_aabb.v[4] = e.get2();
//			_aabb.v[5] = _final_posr.pos.get2();
//		}
	}


	public static DxRay dCreateRay (DxSpace space, double length)
	{
		return new DxRay (space,length);
	}


	//void dGeomRaySetLength (dxGeom g, double length)
	private void dGeomRaySetLength (double length)
	{
		//	dUASSERT (g!=null && g.type == dRayClass,"argument not a ray");
		//  dxRay *r = (dxRay*) g;
		_length = length;
		dGeomMoved ();
	}


	//double dGeomRayGetLength (dxGeom g)
//	private double dGeomRayGetLength ()
//	{
//		//dUASSERT (type == dRayClass,"argument not a ray");
//		return _length;
//	}


	//void dGeomRaySet (dxGeom g, double px, double py, double pz,
	//		  double dx, double dy, double dz)
	private void dGeomRaySet (double px, double py, double pz,
			double dx, double dy, double dz)
	{
		//	dUASSERT (g!=null && g.type == dRayClass,"argument not a ray");
		recomputePosr();
		DMatrix3C rot = final_posr().R();
		//  dVector3 pos = final_posr().pos();
		DVector3 n;
		//  pos[0] = px;
		//  pos[1] = py;
		//  pos[2] = pz;
		_final_posr.pos.set(px, py, pz);

		//  n[0] = dx;
		//  n[1] = dy;
		//  n[2] = dz;
		n = new DVector3(dx, dy, dz);
		//dNormalize3(n);
		n.normalize();
		//  rot[0*4+2] = n[0];
		//  rot[1*4+2] = n[1];
		//  rot[2*4+2] = n[2];
		rot.viewCol(2).set(n);
		dGeomMoved ();
	}
	private void dGeomRaySet (DVector3C p, DVector3C d)
	{
		recomputePosr();
		DMatrix3C rot = final_posr().R();
		_final_posr.pos.set(p);

		DVector3 n = new DVector3(d);
		n.normalize();
		rot.viewCol(2).set(n);
		dGeomMoved ();
	}


	//void dGeomRayGet (dxGeom g, dVector3 start, dVector3 dir)
	private void dGeomRayGet (DVector3 start, DVector3 dir)
	{
		//	dUASSERT (type == dRayClass,"argument not a ray");
		recomputePosr();
		//  start[0] = g.final_posr.pos[0];
		//  start[1] = g.final_posr.pos[1];
		//  start[2] = g.final_posr.pos[2];
		start.set(final_posr().pos());
		//  dir[0] = g.final_posr.R[0*4+2];
		//  dir[1] = g.final_posr.R[1*4+2];
		//  dir[2] = g.final_posr.R[2*4+2];
		dir.set(final_posr().R().viewCol(2));
	}


	//void dGeomRayGetParams (dxGeom *g, int *FirstContact, int *BackfaceCull)
//	void dGeomRayGetParams (RefBoolean FirstContact, RefBoolean BackfaceCull)
//	{
//		//dUASSERT (g!=null && g.type == dRayClass,"argument not a ray");
//
//		//  (*FirstContact) = ((g->gflags & RAY_FIRSTCONTACT) != 0);
//		//  (*BackfaceCull) = ((g->gflags & RAY_BACKFACECULL) != 0);
//		FirstContact.set((_gflags & RAY_FIRSTCONTACT) != 0);
//		BackfaceCull.set((_gflags & RAY_BACKFACECULL) != 0);
//	}
	
//	void dGeomRayGetParams (RefBoolean FirstContact, RefBoolean BackfaceCull)
//	{
//	    //dUASSERT (g && g->type == dRayClass,"argument not a ray");
//
//	    FirstContact = ((gflags & RAY_FIRSTCONTACT) != 0);
//	    BackfaceCull = ((gflags & RAY_BACKFACECULL) != 0);
//	}


	// set/get backface culling flag
	void dGeomRaySetBackfaceCull (boolean backfaceCull) 
	{
	    
	    //dUASSERT (g && g->type == dRayClass,"argument not a ray");
	    if (backfaceCull) {
	        setFlagCustom(RAY_BACKFACECULL);//gflags |= RAY_BACKFACECULL;
	    } else {
	        unsetFlagCustom(RAY_BACKFACECULL);//gflags &= ~RAY_BACKFACECULL;
	    }
	}


	boolean dGeomRayGetBackfaceCull ()
	{
	    //dUASSERT (g && g->type == dRayClass,"argument not a ray");
	    return ((getFlags() & RAY_BACKFACECULL) != 0);
	}


	// set/get first contact flag
	void dGeomRaySetFirstContact (boolean firstContact)
	{
	    //dUASSERT (g && g->type == dRayClass,"argument not a ray");
	    if (firstContact) {
	        setFlagCustom(RAY_FIRSTCONTACT);//gflags |= RAY_FIRSTCONTACT;
	    } else {
	        unsetFlagCustom(RAY_FIRSTCONTACT);//gflags &= ~RAY_FIRSTCONTACT;
	    }
	}

	@Override
	public boolean getFirstContact() {
		return (getFlags() & RAY_FIRSTCONTACT) != 0;
	}
	@Override
	public boolean getBackfaceCull() {
		return (getFlags() & RAY_BACKFACECULL) != 0;
	}


	//void dGeomRaySetClosestHit (dxGeom *g, int closestHit)
	void dGeomRaySetClosestHit (boolean closestHit)
	{
		//dUASSERT (g!=null && g.type == dRayClass,"argument not a ray");
		if (closestHit){
			setFlagCustom(RAY_CLOSEST_HIT);//_gflags |= RAY_CLOSEST_HIT;
		} else {
			unsetFlagCustom(RAY_CLOSEST_HIT);//_gflags &= ~RAY_CLOSEST_HIT;
		}
	}


	//int dGeomRayGetClosestHit (dxGeom *g)
	boolean dGeomRayGetClosestHit ()
	{
		//dUASSERT (g!=null && g.type == dRayClass,"argument not a ray");
		return ((getFlags() & RAY_CLOSEST_HIT) != 0);
	}



	// if mode==1 then use the sphere exit contact, not the entry contact

	//static int ray_sphere_helper (dxRay *ray, dVector3 sphere_pos, dReal radius,
	//	      dContactGeom *contact, int mode)
	private static int ray_sphere_helper (DxRay ray, DVector3C sphere_pos, double radius,
			DContactGeomBuffer contacts, boolean mode)
	{
		DVector3 q = new DVector3();
		//  q[0] = ray.final_posr.pos[0] - sphere_pos[0];
		//  q[1] = ray.final_posr.pos[1] - sphere_pos[1];
		//  q[2] = ray.final_posr.pos[2] - sphere_pos[2];
		q.eqDiff(ray.final_posr().pos(), sphere_pos);
		double B = dCalcVectorDot3_14(q,ray.final_posr().R(),2);
		double C = q.dot(q) - radius*radius;
		// note: if C <= 0 then the start of the ray is inside the sphere
		double k = B*B - C;
		if (k < 0) return 0;
		k = dSqrt(k);
		double alpha;
		if (mode && C >= 0) {
			alpha = -B + k;
			if (alpha < 0) return 0;
		}
		else {
			alpha = -B - k;
			if (alpha < 0) {
				alpha = -B + k;
				if (alpha < 0) return 0;
			}
		}
		if (alpha > ray._length) return 0;
		DContactGeom contact = contacts.get(0);
		//  contact.pos[0] = ray.final_posr.pos[0] + alpha*ray.final_posr.R[0*4+2];
		//  contact.pos[1] = ray.final_posr.pos[1] + alpha*ray.final_posr.R[1*4+2];
		//  contact.pos[2] = ray.final_posr.pos[2] + alpha*ray.final_posr.R[2*4+2];
		contact.pos.eqSum(ray.final_posr().pos(), ray.final_posr().R().viewCol(2), alpha);
		double nsign = (C < 0 || mode) ? (-1.0) : (1.0);
		//  contact.normal[0] = nsign*(contact.pos[0] - sphere_pos[0]);
		//  contact.normal[1] = nsign*(contact.pos[1] - sphere_pos[1]);
		//  contact.normal[2] = nsign*(contact.pos[2] - sphere_pos[2]);
		contact.normal.eqDiff(contact.pos, sphere_pos).scale(nsign);
		//dNormalize3 (contact.normal);
		contact.normal.normalize();
		contact.depth = alpha;
		return 1;
	}

	static class CollideRaySphere implements DColliderFn {
		//int dCollideRaySphere (dxGeom *o1, dxGeom *o2, int flags,
		//	       dContactGeom *contact, int skip)
		int dCollideRaySphere (DxRay ray, DxSphere sphere, int flags,
				DContactGeomBuffer contacts, int skip)
		{
			dIASSERT (skip >= 1);//(int)sizeof(dContactGeom));
			//  dIASSERT (o1.type == dRayClass);
			//  dIASSERT (o2.type == dSphereClass);
			dIASSERT ((flags & NUMC_MASK) >= 1);

			//  dxRay *ray = (dxRay*) o1;
			//  dxSphere *sphere = (dxSphere*) o2;
			DContactGeom contact = contacts.get(0);
			contact.g1 = ray;
			contact.g2 = sphere;
			contact.side1 = -1;
			contact.side2 = -1;
			return ray_sphere_helper (ray,sphere.final_posr().pos(),sphere.getRadius(),contacts,false);
		}

		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return dCollideRaySphere((DxRay)o1, (DxSphere)o2, flags, contacts, 1);
		}
	}

	static class CollideRayBox implements DColliderFn {
		//int dCollideRayBox (dxGeom *o1, dxGeom *o2, int flags,
		//	    dContactGeom *contact, int skip)
		int dCollideRayBox (DxRay ray, DxBox box, int flags,
				DContactGeomBuffer contacts, int skip)
		{
			dIASSERT (skip >= 1);//(int)sizeof(dContactGeom));
			//  dIASSERT (o1.type == dRayClass);
			//  dIASSERT (o2.type == dBoxClass);
			dIASSERT ((flags & NUMC_MASK) >= 1);

			//  dxRay *ray = (dxRay*) o1;
			//  dxBox *box = (dxBox*) o2;

			DContactGeom contact = contacts.get(0);
			contact.g1 = ray;
			contact.g2 = box;
			contact.side1 = -1;
			contact.side2 = -1;

			int i;

			// compute the start and delta of the ray relative to the box.
			// we will do all subsequent computations in this box-relative coordinate
			// system. we have to do a translation and rotation for each point.
			DVector3 tmp=new DVector3(),s=new DVector3(),v=new DVector3();
			//		tmp[0] = ray.final_posr.pos[0] - box.final_posr.pos[0];
			//		tmp[1] = ray.final_posr.pos[1] - box.final_posr.pos[1];
			//		tmp[2] = ray.final_posr.pos[2] - box.final_posr.pos[2];
			tmp.eqDiff(ray.final_posr().pos(), box.final_posr().pos());
			dMultiply1_331 (s,box.final_posr().R(),tmp);
			//		tmp[0] = ray.final_posr.R[0*4+2];
			//		tmp[1] = ray.final_posr.R[1*4+2];
			//		tmp[2] = ray.final_posr.R[2*4+2];
			tmp.set(ray.final_posr().R().viewCol(2));
			dMultiply1_331 (v,box.final_posr().R(),tmp);

			// mirror the line so that v has all components >= 0
			DVector3 sign = new DVector3();
			for (i=0; i<3; i++) {
				if (v.get(i) < 0) {
					//				s.v[i] = -s.v[i];
					//				v.v[i] = -v.v[i];
					s.scale(i, -1);
					v.scale(i, -1);
					sign.set(i, 1);
				}
				else sign.set(i, -1);
			}

			// compute the half-sides of the box
			double[] h=new double[3];
			h[0] = (0.5) * box.side.get0();
			h[1] = (0.5) * box.side.get1();
			h[2] = (0.5) * box.side.get2();

			// do a few early exit tests
			if ((s.get0() < -h[0] && v.get0() <= 0) || s.get0() >  h[0] ||
					(s.get1() < -h[1] && v.get1() <= 0) || s.get1() >  h[1] ||
					(s.get2() < -h[2] && v.get2() <= 0) || s.get2() >  h[2] ||
					(v.get0() == 0 && v.get1() == 0 && v.get2() == 0)) {
				return 0;
			}

			// compute the t=[lo..hi] range for where s+v*t intersects the box
			double lo = -dInfinity;
			double hi = dInfinity;
			int nlo = 0, nhi = 0;
			for (i=0; i<3; i++) {
				if (v.get(i) != 0) {
					double k = (-h[i] - s.get(i))/v.get(i);
					if (k > lo) {
						lo = k;
						nlo = i;
					}
					k = (h[i] - s.get(i))/v.get(i);
					if (k < hi) {
						hi = k;
						nhi = i;
					}
				}
			}

			// check if the ray intersects
			if (lo > hi) return 0;
			double alpha;
			int n;
			if (lo >= 0) {
				alpha = lo;
				n = nlo;
			}
			else {
				alpha = hi;
				n = nhi;
			}
			if (alpha < 0 || alpha > ray._length) return 0;
			//		contact.pos[0] = ray.final_posr.pos[0] + alpha*ray.final_posr.R[0*4+2];
			//		contact.pos[1] = ray.final_posr.pos[1] + alpha*ray.final_posr.R[1*4+2];
			//		contact.pos[2] = ray.final_posr.pos[2] + alpha*ray.final_posr.R[2*4+2];
			contact.pos.eqSum(ray.final_posr().pos(), ray.final_posr().R().viewCol(2), alpha);
			//		contact.normal[0] = box.final_posr.R[0*4+n] * sign[n];
			//		contact.normal[1] = box.final_posr.R[1*4+n] * sign[n];
			//		contact.normal[2] = box.final_posr.R[2*4+n] * sign[n];
			contact.normal.set(box.final_posr().R().viewCol(n)).scale(sign.get(n));
			contact.depth = alpha;
			return 1;
		}

		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return dCollideRayBox((DxRay)o1, (DxBox)o2, flags, contacts, 1);
		}
	}

	static class CollideRayCapsule implements DColliderFn {
		//int dCollideRayCapsule (dxGeom *o1, dxGeom *o2,
		//		  int flags, dContactGeom *contact, int skip)
		int dCollideRayCapsule (DxRay ray, DxCapsule ccyl,
				int flags, DContactGeomBuffer contacts, int skip)
		{
			dIASSERT (skip >= 1);//(int)sizeof(dContactGeom));
			//  dIASSERT (o1.type == dRayClass);
			//  dIASSERT (o2.type == dCapsuleClass);
			dIASSERT ((flags & NUMC_MASK) >= 1);

			//  dxRay *ray = (dxRay*) o1;
			//  dxCapsule *ccyl = (dxCapsule*) o2;

			DContactGeom contact = contacts.get(0);
			contact.g1 = ray;
			contact.g2 = ccyl;
			contact.side1 = -1;
			contact.side2 = -1;

			double lz2 = ccyl.getLength() * 0.5;//_lz * (0.5);

			// compute some useful info
			DVector3 cs=new DVector3(),q=new DVector3(),r=new DVector3();
			double C,k;
			//		cs[0] = ray.final_posr.pos[0] - ccyl.final_posr.pos[0];
			//		cs[1] = ray.final_posr.pos[1] - ccyl.final_posr.pos[1];
			//		cs[2] = ray.final_posr.pos[2] - ccyl.final_posr.pos[2];
			cs.eqDiff(ray.final_posr().pos(), ccyl.final_posr().pos());
			k = dCalcVectorDot3_41(ccyl.final_posr().R(),2,cs);	// position of ray start along ccyl axis
			//		q[0] = k*ccyl.final_posr.R[0*4+2] - cs[0];
			//		q[1] = k*ccyl.final_posr.R[1*4+2] - cs[1];
			//		q[2] = k*ccyl.final_posr.R[2*4+2] - cs[2];
			q.eqSum( ccyl.final_posr().R().viewCol(2), k, 
					cs, -1);
			C = dCalcVectorDot3(q,q) - ccyl.getRadius()*ccyl.getRadius();
			// if C < 0 then ray start position within infinite extension of cylinder

			// see if ray start position is inside the capped cylinder
			boolean inside_ccyl = false;
			if (C < 0) {
				if (k < -lz2) k = -lz2;
				else if (k > lz2) k = lz2;
				//				r[0] = ccyl.final_posr.pos[0] + k*ccyl.final_posr.R[0*4+2];
				//				r[1] = ccyl.final_posr.pos[1] + k*ccyl.final_posr.R[1*4+2];
				//				r[2] = ccyl.final_posr.pos[2] + k*ccyl.final_posr.R[2*4+2];
				r.eqSum(ccyl.final_posr().R().viewCol(2), k, ccyl.final_posr().pos(), 1.0);
				if ((ray.final_posr().pos().get0()-r.get0())*(ray.final_posr().pos().get0()-r.get0()) +
						(ray.final_posr().pos().get1()-r.get1())*(ray.final_posr().pos().get1()-r.get1()) +
						(ray.final_posr().pos().get2()-r.get2())*(ray.final_posr().pos().get2()-r.get2()) < 
						ccyl.getRadius()*ccyl.getRadius()) {
					inside_ccyl = true;
				}
			}

			// compute ray collision with infinite cylinder, except for the case where
			// the ray is outside the capped cylinder but within the infinite cylinder
			// (it that case the ray can only hit endcaps)
			if (!inside_ccyl && C < 0) {
				// set k to cap position to check
				if (k < 0) k = -lz2; else k = lz2;
			}
			else {
				double uv = dCalcVectorDot3_44(ccyl.final_posr().R(),2,ray.final_posr().R(),2);
				//				r[0] = uv*ccyl.final_posr.R[0*4+2] - ray.final_posr.R[0*4+2];
				//				r[1] = uv*ccyl.final_posr.R[1*4+2] - ray.final_posr.R[1*4+2];
				//				r[2] = uv*ccyl.final_posr.R[2*4+2] - ray.final_posr.R[2*4+2];
				r.eqSum(ccyl.final_posr().R().viewCol(2), uv, ray.final_posr().R().viewCol(2), -1);
				double A = dCalcVectorDot3(r,r);
				// A == 0 means that the ray and ccylinder axes are parallel
				if (A == 0) { // There is a division by A below...
					// set k to cap position to check
					if (uv < 0) k = -lz2; else k = lz2;
				}
				else {
					double B = 2*dCalcVectorDot3(q,r);
					k = B*B-4*A*C;
					if (k < 0) {
						// the ray does not intersect the infinite cylinder, but if the ray is
						// inside and parallel to the cylinder axis it may intersect the end
						// caps. set k to cap position to check.
						if (!inside_ccyl) return 0;
						if (uv < 0) k = -lz2; else k = lz2;
					}
					else {
						k = dSqrt(k);
						A = dRecip(2 * A);
						double alpha = (-B - k) * A;
						if (alpha < 0) {
							alpha = (-B + k) * A;
							if (alpha < 0) return 0;
						}
						if (alpha > ray._length) return 0;

						// the ray intersects the infinite cylinder. check to see if the
						// intersection point is between the caps
						//					contact.pos[0] = ray.final_posr.pos[0] + alpha*ray.final_posr.R[0*4+2];
						//					contact.pos[1] = ray.final_posr.pos[1] + alpha*ray.final_posr.R[1*4+2];
						//					contact.pos[2] = ray.final_posr.pos[2] + alpha*ray.final_posr.R[2*4+2];
						contact.pos.eqSum(ray.final_posr().R().viewCol(2), alpha, ray.final_posr().pos(), 1);
						//					q[0] = contact.pos[0] - ccyl.final_posr.pos[0];
						//					q[1] = contact.pos[1] - ccyl.final_posr.pos[1];
						//					q[2] = contact.pos[2] - ccyl.final_posr.pos[2];
						q.eqDiff(contact.pos, ccyl.final_posr().pos());
						k = dCalcVectorDot3_14(q, ccyl.final_posr().R(), 2);
						double nsign = inside_ccyl ? (-1.0) : (1.0);
						if (k >= -lz2 && k <= lz2) {
							//						contact.normal[0] = nsign * (contact.pos[0] -
							//								(ccyl.final_posr.pos[0] + k*ccyl.final_posr.R[0*4+2]));
							//						contact.normal[1] = nsign * (contact.pos[1] -
							//								(ccyl.final_posr.pos[1] + k*ccyl.final_posr.R[1*4+2]));
							//						contact.normal[2] = nsign * (contact.pos[2] -
							//								(ccyl.final_posr.pos[2] + k*ccyl.final_posr.R[2*4+2]));
							contact.normal.eqSum(ccyl.final_posr().R().viewCol(2), -k, ccyl.final_posr().pos(), -1);
							//TODO while scale() if normalized afterwards?
							contact.normal.add(contact.pos).scale(nsign);
							//dNormalize3 (contact.normal);
							contact.normal.normalize();
							contact.depth = alpha;
							return 1;
						}

						// the infinite cylinder intersection point is not between the caps.
						// set k to cap position to check.
						if (k < 0) k = -lz2;
						else k = lz2;
					}
				}
			}

			// check for ray intersection with the caps. k must indicate the cap
			// position to check
			//			q[0] = ccyl.final_posr.pos[0] + k*ccyl.final_posr.R[0*4+2];
			//			q[1] = ccyl.final_posr.pos[1] + k*ccyl.final_posr.R[1*4+2];
			//			q[2] = ccyl.final_posr.pos[2] + k*ccyl.final_posr.R[2*4+2];
			q.eqSum( ccyl.final_posr().R().viewCol(2), k, ccyl.final_posr().pos(), 1.0 );
			return ray_sphere_helper (ray,q,ccyl.getRadius(),contacts, inside_ccyl);
		}

		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return dCollideRayCapsule((DxRay)o1, (DxCapsule)o2, flags, contacts, 1);
		}
	}

	static class CollideRayPlane implements DColliderFn {

		//int dCollideRayPlane (dxGeom *o1, dxGeom *o2, int flags,
		//	      dContactGeom *contact, int skip)
		int dCollideRayPlane (DxRay ray, DxPlane plane, int flags,
				DContactGeomBuffer contacts, int skip)
		{
			dIASSERT (skip >= 1);//(int)sizeof(dContactGeom));
			//  dIASSERT (o1.type == dRayClass);
			//  dIASSERT (o2.type == dPlaneClass);
			dIASSERT ((flags & NUMC_MASK) >= 1);

			//  dxRay *ray = (dxRay*) o1;
			//  dxPlane *plane = (dxPlane*) o2;

			//double alpha = plane.getD_p[3] - dDOT (plane._p,ray._final_posr.pos.v);
			double alpha = plane.getDepth() - plane.getNormal().dot (ray.final_posr().pos());
			// note: if alpha > 0 the starting point is below the plane
			double nsign = (alpha > 0) ? (-1.0) : (1.0);
			//double k = dDOT14(plane._p, 0,ray._final_posr.R.v,2);
			double k = plane.getNormal().dot( ray.final_posr().R().viewCol(2) );
			if (k==0) return 0;		// ray parallel to plane
			alpha /= k;
			if (alpha < 0 || alpha > ray._length) return 0;
			DContactGeom contact = contacts.get(0);
			//  contact.pos[0] = ray.final_posr.pos[0] + alpha*ray.final_posr.R[0*4+2];
			//  contact.pos[1] = ray.final_posr.pos[1] + alpha*ray.final_posr.R[1*4+2];
			//  contact.pos[2] = ray.final_posr.pos[2] + alpha*ray.final_posr.R[2*4+2];
			contact.pos.eqSum(ray.final_posr().R().viewCol(2), alpha, ray.final_posr().pos(), 1);
			//  contact.normal[0] = nsign*plane.p[0];
			//  contact.normal[1] = nsign*plane.p[1];
			//  contact.normal[2] = nsign*plane.p[2];
			contact.normal.set(plane.getNormal());
			contact.normal.scale(nsign);
			contact.depth = alpha;
			contact.g1 = ray;
			contact.g2 = plane;
			contact.side1 = -1;
			contact.side2 = -1;
			return 1;
		}

		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return dCollideRayPlane((DxRay)o1, (DxPlane)o2, flags, contacts, 1);
		}
	}

	static class CollideRayCylinder implements DColliderFn {
		// Ray-Cylinder collider by Joseph Cooper (2011)
		//int dCollideRayCylinder( dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip )
		int dCollideRayCylinder( DxRay ray, DxCylinder cyl, int flags, DContactGeomBuffer contacts, int skip )
		{
			dIASSERT( skip >= 1);//(int)sizeof( dContactGeom ) );
			//	dIASSERT( o1.type == dRayClass );
			//	dIASSERT( o2.type == dCylinderClass );
			dIASSERT( (flags & NUMC_MASK) >= 1 );

			//	dxRay* ray = (dxRay*)( o1 );
			//	dxCylinder* cyl = (dxCylinder*)( o2 );

			// Fill in contact information.
			DContactGeom contact = contacts.get(0);
			contact.g1 = ray;
			contact.g2 = cyl;
			contact.side1 = -1;
			contact.side2 = -1;

			final double half_length = cyl.getLength() * ( 0.5 );//_lz * REAL( 0.5 );

		    /* Possible collision cases:
		     *  Ray origin between/outside caps
		     *  Ray origin within/outside radius
		     *  Ray direction left/right/perpendicular
		     *  Ray direction parallel/perpendicular/other
		     * 
		     *  Ray origin cases (ignoring origin on surface)
		     *
		     *  A          B
		     *     /-\-----------\
		     *  C (   )    D      )
		     *     \_/___________/
		     *
		     *  Cases A and D can collide with caps or cylinder
		     *  Case C can only collide with the caps
		     *  Case B can only collide with the cylinder
		     *  Case D will produce inverted normals
		     *  If the ray is perpendicular, only check the cylinder
		     *  If the ray is parallel to cylinder axis,
		     *  we can only check caps
		     *  If the ray points right,
		     *    Case A,C Check left cap
		     *    Case  D  Check right cap
		     *  If the ray points left
		     *    Case A,C Check right cap
		     *    Case  D  Check left cap
		     *  Case B, check only first possible cylinder collision
		     *  Case D, check only second possible cylinder collision
		     */
		    // Find the ray in the cylinder coordinate frame:
		    DVector3 tmp = new DVector3();
		    DVector3 pos = new DVector3();  // Ray origin in cylinder frame
		    DVector3 dir = new DVector3();  // Ray direction in cylinder frame
		    // Translate ray start by inverse cyl
		    //dSubtractVectors3(tmp,ray.final_posr().pos(),cyl.final_posr().pos());
		    tmp.eqDiff( ray.final_posr().pos(), cyl.final_posr().pos() );
		    // Rotate ray start by inverse cyl
		    dMultiply1_331(pos,cyl.final_posr().R(),tmp);

		    // Get the ray's direction
		    //tmp[0] = ray.final_posr().R[2];
		    //tmp[1] = ray.final_posr().R[6];
		    //tmp[2] = ray.final_posr().R[10];
		    ray.final_posr().R().getColumn2(tmp);
		    // Rotate the ray direction by inverse cyl
		    dMultiply1_331(dir,cyl.final_posr().R(),tmp); 

		    // Is the ray origin inside of the (extended) cylinder?
		    double r2 = cyl.getRadius()*cyl.getRadius();
		    double C = pos.get0()*pos.get0() + pos.get1()*pos.get1() - r2;

		    // Find the different cases
		    // Is ray parallel to the cylinder length?
		    boolean parallel = (dir.get0()==0 && dir.get1()==0);
		    // Is ray perpendicular to the cylinder length?
		    boolean perpendicular = (dir.get2()==0);
		    // Is ray origin within the radius of the caps?
		    boolean inRadius = (C<=0);
		    // Is ray origin between the top and bottom caps?
		    boolean inCaps   = (dFabs(pos.get2())<=half_length);

		    boolean checkCaps = (!perpendicular && (!inCaps || inRadius));
		    boolean checkCyl  = (!parallel && (!inRadius || inCaps));
		    boolean flipNormals = (inCaps&&inRadius);

		    double tt=-dInfinity; // Depth to intersection
		    DVector3 tmpNorm = new DVector3(dNaN, dNaN, dNaN); // ensure we don't leak garbage

		    if (checkCaps) {
		        // Make it so we only need to check one cap
		        boolean flipDir = false;
		        // Wish c had logical xor...
		        if ((dir.get2()<0 && flipNormals) || (dir.get2()>0 && !flipNormals)) {
		            flipDir = true;
		            dir.set2( -dir.get2() );
		            pos.set2( -pos.get2() );
		        }
		        // The cap is half the cylinder's length
		        // from the cylinder's origin
		        // We only checkCaps if dir[2]!=0
		        tt = (half_length-pos.get2())/dir.get2();
		        if (tt>=0 && tt<=ray.getLength()) {
		            tmp.set0( pos.get0() + tt*dir.get0() );
		            tmp.set1( pos.get1() + tt*dir.get1() );
		            // Ensure collision point is within cap circle
		            if (tmp.get0()*tmp.get0() + tmp.get1()*tmp.get1() <= r2) {
		                // Successful collision
		                tmp.set2( (flipDir)?-half_length:half_length );
		                tmpNorm.set0( 0 );
		                tmpNorm.set1( 0 );
		                tmpNorm.set2( (flipDir!=flipNormals)?-1:1 );
		                checkCyl = false;  // Short circuit cylinder check
		            } else {
		                // Ray hits cap plane outside of cap circle
		                tt=-dInfinity; // No collision yet
		            }
		        } else {
		            // The cap plane is beyond (or behind) the ray length
		            tt=-dInfinity; // No collision yet
		        }
		        if (flipDir) {
		            // Flip back
		            dir.set2( -dir.get2() );
		            pos.set2( -pos.get2() );
		        }
		    }
		    if (checkCyl) {
		        // Compute quadratic formula for parametric ray equation
		        double A =    dir.get0()*dir.get0() + dir.get1()*dir.get1();
		        double B = 2*(pos.get0()*dir.get0() + pos.get1()*dir.get1());
		        // Already computed C

		        double k = B*B - 4*A*C;
		        // Check collision with infinite cylinder
		        // k<0 means the ray passes outside the cylinder
		        // k==0 means ray is tangent to cylinder (or parallel)
		        //
		        //  Our quadratic formula: tt = (-B +- sqrt(k))/(2*A)   
		        // 
		        // A must be positive (otherwise we wouldn't be checking
		        // cylinder because ray is parallel)
		        //    if (k<0) ray doesn't collide with sphere
		        //    if (B > sqrt(k)) then both times are negative
		        //         -- don't calculate
		        //    if (B<-sqrt(k)) then both times are positive (Case A or B)
		        //         -- only calculate first, if first isn't valid
		        //         -- second can't be without first going through a cap
		        //    otherwise (fabs(B)<=sqrt(k)) then C<=0 (ray-origin inside/on cylinder)
		        //         -- only calculate second collision
		        if (k>=0 && (B<0 || B*B<=k)) {
		            k = dSqrt(k); 
		            A = dRecip(2*A);
		            if (dFabs(B)<=k) {
		                tt = (-B + k)*A; // Second solution
		                // If ray origin is on surface and pointed out, we
		                // can get a tt=0 solution...
		            } else {
		                tt = (-B - k)*A; // First solution
		            }
		            if (tt<=ray.getLength()) {
		                tmp.set2( pos.get2() + tt*dir.get2() );
		                if (dFabs(tmp.get2())<=half_length) {
		                    // Valid solution
		                    tmp.set0( pos.get0() + tt*dir.get0() );
		                    tmp.set1( pos.get1() + tt*dir.get1() );
		                    tmpNorm.set0( tmp.get0()/cyl.getRadius() );
		                    tmpNorm.set1( tmp.get1()/cyl.getRadius() );
		                    tmpNorm.set2( 0 );
		                    if (flipNormals) {
		                        // Ray origin was inside cylinder
		                        tmpNorm.set0( -tmpNorm.get0() );
		                        tmpNorm.set1( -tmpNorm.get1() );
		                    }
		                } else {
		                    // Ray hits cylinder outside of caps
		                    tt=-dInfinity;
		                }
		            } else {
		                // Ray doesn't reach the cylinder
		                tt=-dInfinity;
		            }
		        }
		    }

		    if (tt>0) {
		        contact.depth = tt;
		        // Transform the point back to world coordinates
		        //tmpNorm[3]=0;
		        //tmp[3] = 0;
		        dMultiply0_331(contact.normal,cyl.final_posr().R(),tmpNorm);
		        dMultiply0_331(contact.pos,cyl.final_posr().R(),tmp);
//		        contact.pos[0]+=cyl.final_posr.pos[0];
//		        contact.pos[1]+=cyl.final_posr.pos[1];
//		        contact.pos[2]+=cyl.final_posr.pos[2];
		        contact.pos.add( cyl.final_posr().pos() );
		        return 1;
		    }
		    // No contact with anything.
		    return 0;
		}

		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return dCollideRayCylinder((DxRay)o1, (DxCylinder)o2, flags, contacts, 1);
		}
	}
	
	static class CollideRayCylinder_ODE_0_12 implements DColliderFn {
		// Ray - Cylinder collider by David Walters (June 2006)
		//int dCollideRayCylinder( dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip )
		int dCollideRayCylinder( DxRay ray, DxCylinder cyl, int flags, DContactGeomBuffer contacts, int skip )
		{
			dIASSERT( skip >= 1);//(int)sizeof( dContactGeom ) );
			//	dIASSERT( o1.type == dRayClass );
			//	dIASSERT( o2.type == dCylinderClass );
			dIASSERT( (flags & NUMC_MASK) >= 1 );

			//	dxRay* ray = (dxRay*)( o1 );
			//	dxCylinder* cyl = (dxCylinder*)( o2 );

			// Fill in contact information.
			DContactGeom contact = contacts.get(0);
			contact.g1 = ray;
			contact.g2 = cyl;
			contact.side1 = -1;
			contact.side2 = -1;

			final double half_length = cyl.getLength() * ( 0.5 );//_lz * REAL( 0.5 );

			//
			// Compute some useful info
			//

			DVector3 q = new DVector3(), r = new DVector3();
			double d, C, k;

			// Vector 'r', line segment from C to R (ray start) ( r = R - C )
			//		r[ 0 ] = ray.final_posr.pos[0] - cyl.final_posr.pos[0];
			//		r[ 1 ] = ray.final_posr.pos[1] - cyl.final_posr.pos[1];
			//		r[ 2 ] = ray.final_posr.pos[2] - cyl.final_posr.pos[2];
			r.eqDiff(ray.final_posr().pos(), cyl.final_posr().pos());

			// Distance that ray start is along cyl axis ( Z-axis direction )
			d = dCalcVectorDot3_41( cyl.final_posr().R() , 2, r );

			//
			// Compute vector 'q' representing the shortest line from R to the cylinder z-axis (Cz).
			//
			// Point on axis ( in world space ):	cp = ( d * Cz ) + C
			//
			// Line 'q' from R to cp:				q = cp - R
			//										q = ( d * Cz ) + C - R
			//										q = ( d * Cz ) - ( R - C )

			//		q[ 0 ] = ( d * cyl.final_posr.R[0*4+2] ) - r[ 0 ];
			//		q[ 1 ] = ( d * cyl.final_posr.R[1*4+2] ) - r[ 1 ];
			//		q[ 2 ] = ( d * cyl.final_posr.R[2*4+2] ) - r[ 2 ];
			q.eqSum(cyl.final_posr().R().viewCol(2), d, r, -1);


			// Compute square length of 'q'. Subtract from radius squared to
			// get square distance 'C' between the line q and the radius.

			// if C < 0 then ray start position is within infinite extension of cylinder

			C = dCalcVectorDot3( q, q ) - ( cyl.getRadius() * cyl.getRadius() );

			// Compute the projection of ray direction normal onto cylinder direction normal.
			double uv = dCalcVectorDot3_44( cyl.final_posr().R(),2, ray.final_posr().R(),2 );



			//
			// Find ray collision with infinite cylinder
			//

			// Compute vector from end of ray direction normal to projection on cylinder direction normal.
			//		r[ 0 ] = ( uv * cyl.final_posr.R[0*4+2] ) - ray.final_posr.R[0*4+2];
			//		r[ 1 ] = ( uv * cyl.final_posr.R[1*4+2] ) - ray.final_posr.R[1*4+2];
			//		r[ 2 ] = ( uv * cyl.final_posr.R[2*4+2] ) - ray.final_posr.R[2*4+2];
			r.eqSum(cyl.final_posr().R().viewCol(2), uv, ray.final_posr().R().viewCol(2), -1);


			// Quadratic Formula Magic
			// Compute discriminant 'k':

			// k < 0 : No intersection
			// k = 0 : Tangent
			// k > 0 : Intersection

			double A = dCalcVectorDot3( r, r );
			double B = 2 * dCalcVectorDot3( q, r );

			k = B*B - 4*A*C;




			//
			// Collision with Flat Caps ?
			//

			// No collision with cylinder edge. ( Use epsilon here or we miss some obvious cases )
			if ( k < dEpsilon && C <= 0 )
			{
				// The ray does not intersect the edge of the infinite cylinder,
				// but the ray start is inside and so must run parallel to the axis.
				// It may yet intersect an end cap. The following cases are valid:

				//        -ve-cap , -half              centre               +half , +ve-cap
				//  <<================|-------------------|------------->>>---|================>>
				//                    |                                       |
				//                    |                              d------------------->    1.
				//   2.    d------------------>                               |
				//   3.    <------------------d                               |
				//                    |                              <-------------------d    4.
				//                    |                                       |
				//  <<================|-------------------|------------->>>---|===============>>

				// Negative if the ray and cylinder axes point in opposite directions.
				final double uvsign = ( uv < 0 ) ? ( -1.0 ) : ( 1.0 );

				// Negative if the ray start is inside the cylinder
				final double internal = ( d >= -half_length && d <= +half_length ) ? ( -1.0 ) : ( 1.0 );

				// Ray and Cylinder axes run in the same direction ( cases 1, 2 )
				// Ray and Cylinder axes run in opposite directions ( cases 3, 4 )
				if ( ( ( uv > 0 ) && ( d + ( uvsign * ray._length ) < half_length * internal ) ) ||
						( ( uv < 0 ) && ( d + ( uvsign * ray._length ) > half_length * internal ) ) )
				{
					return 0; // No intersection with caps or curved surface.
				}

				// Compute depth (distance from ray to cylinder)
				contact.depth = ( ( -uvsign * d ) - ( internal * half_length ) );

				// Compute contact point.
				//			contact.pos[0] = ray.final_posr.pos[0] + ( contact.depth * ray.final_posr.R[0*4+2] );
				//			contact.pos[1] = ray.final_posr.pos[1] + ( contact.depth * ray.final_posr.R[1*4+2] );
				//			contact.pos[2] = ray.final_posr.pos[2] + ( contact.depth * ray.final_posr.R[2*4+2] );
				contact.pos.eqSum(ray.final_posr().R().viewCol(2), contact.depth, ray.final_posr().pos(), 1.0);

				// Compute reflected contact normal.
				//			contact.normal[0] = uvsign * ( cyl.final_posr.R[0*4+2] );
				//			contact.normal[1] = uvsign * ( cyl.final_posr.R[1*4+2] );
				//			contact.normal[2] = uvsign * ( cyl.final_posr.R[2*4+2] );
				contact.normal.set(cyl.final_posr().R().viewCol(2)).scale(uvsign);

				// Contact!
				return 1;
			}


			//
			// Collision with Curved Edge ?
			//

			if ( k > 0 )
			{
				// Finish off quadratic formula to get intersection co-efficient
				k = dSqrt( k );
				A = dRecip( 2 * A );

				// Compute distance along line to contact point.
				double alpha = ( -B - k ) * A;
				if ( alpha < 0 )
				{
					// Flip in the other direction.
					alpha = ( -B + k ) * A;
				}

				// Intersection point is within ray length?
				if ( alpha >= 0 && alpha <= ray._length )
				{
					// The ray intersects the infinite cylinder!

					// Compute contact point.
					//				contact.pos[0] = ray.final_posr.pos[0] + ( alpha * ray.final_posr.R[0*4+2] );
					//				contact.pos[1] = ray.final_posr.pos[1] + ( alpha * ray.final_posr.R[1*4+2] );
					//				contact.pos[2] = ray.final_posr.pos[2] + ( alpha * ray.final_posr.R[2*4+2] );
					contact.pos.eqSum( ray.final_posr().R().viewCol(2), alpha, 
							ray.final_posr().pos(), 1.0) ;

					// q is the vector from the cylinder centre to the contact point.
					//				q[0] = contact.pos[0] - cyl.final_posr.pos[0];
					//				q[1] = contact.pos[1] - cyl.final_posr.pos[1];
					//				q[2] = contact.pos[2] - cyl.final_posr.pos[2];
					q.eqDiff(contact.pos, cyl.final_posr().pos());

					// Compute the distance along the cylinder axis of this contact point.
					d = dCalcVectorDot3_14( q, cyl.final_posr().R(),2 );

					// Check to see if the intersection point is between the flat end caps
					if ( d >= -half_length && d <= +half_length )
					{
						// Flip the normal if the start point is inside the cylinder.
						final double nsign = ( C < 0 ) ? ( -1.0 ) : ( 1.0 );

						// Compute contact normal.
						//					contact.normal[0] = nsign * (contact.pos[0] - (cyl.final_posr.pos[0] + d*cyl.final_posr.R[0*4+2]));
						//					contact.normal[1] = nsign * (contact.pos[1] - (cyl.final_posr.pos[1] + d*cyl.final_posr.R[1*4+2]));
						//					contact.normal[2] = nsign * (contact.pos[2] - (cyl.final_posr.pos[2] + d*cyl.final_posr.R[2*4+2]));
						contact.normal.eqSum( cyl.final_posr().R().viewCol(2), -d, 
								cyl.final_posr().pos(), -1 );
						contact.normal.add( contact.pos ).scale(nsign);
						//dNormalize3( contact.normal );
						contact.normal.normalize();

						// Store depth.
						contact.depth = alpha;

						// Contact!
						return 1;
					}
				}
			}


			// No contact with anything.
			return 0;
		}
		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return dCollideRayCylinder((DxRay)o1, (DxCylinder)o2, flags, contacts, 1);
		}
	}

	// *****************************
	// dRay API
	// *****************************

	@Override
	public void setLength (double length)
	{ dGeomRaySetLength (length); }
	@Override
	public double getLength()
	{ return _length; }

	@Override
	public void set (double px, double py, double pz, double dx, double dy, double dz)
	{ dGeomRaySet (px, py, pz, dx, dy, dz); }
	@Override
	public void set(DVector3C p, DVector3C d) {
		dGeomRaySet(p, d);
	}
	@Override
	public void get (DVector3 start, DVector3 dir)
	{ dGeomRayGet (start, dir); }

	@Override
	public void setClosestHit (boolean closestHit)
	{ dGeomRaySetClosestHit (closestHit); }

	@Override
	public boolean getClosestHit()
	{ return dGeomRayGetClosestHit (); }


	@Override
	public DVector3C getDirection() {
		return final_posr().R().columnAsNewVector(2);
	}


	@Override
	public void setFirstContact(boolean firstContact) {
		dGeomRaySetFirstContact(firstContact);
	}


	@Override
	public void setBackfaceCull(boolean backfaceCull) {
		dGeomRaySetBackfaceCull(backfaceCull);
	}


}
