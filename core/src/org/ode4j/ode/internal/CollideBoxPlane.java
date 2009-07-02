/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode.internal;

import org.cpp4j.java.RefDouble;
import org.cpp4j.java.RefInt;
import org.ode4j.ode.DColliderFn;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;

import static org.ode4j.ode.OdeMath.*;

public class CollideBoxPlane implements DColliderFn {
	//int dCollideBoxPlane (dxGeom *o1, dxGeom *o2,
	//    int flags, dContactGeom *contact, int skip)
	int dCollideBoxPlane (DxBox o1, DxPlane o2,
			int flags, DContactGeomBuffer contacts, int skip)
	{
		//dIASSERT (skip >= (int)sizeof(dContactGeom));
		dIASSERT (skip == 1);
//		dIASSERT (o1.type == dBoxClass);
//		dIASSERT (o2.type == dPlaneClass);
		dIASSERT ((flags & DxGeom.NUMC_MASK) >= 1);

		DxBox box = (DxBox) o1;
		DxPlane plane = (DxPlane) o2;

		DContactGeom contact = contacts.get(0);
		contact.g1 = o1;
		contact.g2 = o2;
		contact.side1 = -1;
		contact.side2 = -1;
		  
		//int ret = 0;
		RefInt ret = new RefInt();

		//@@@ problem: using 4-vector (plane.p) as 3-vector (normal).
		//final double *R = o1.final_posr.R;		// rotation of box
		//final double *n = plane.p;		// normal vector
		final DMatrix3 R = o1._final_posr.R;		// rotation of box
		//final double []n = plane._p;		// normal vector
		DVector3C n = plane.getNormal();
		
		// project sides lengths along normal vector, get absolute values
		double Q1 = dDOT14(n,R,0);
		double Q2 = dDOT14(n,R,1);
		double Q3 = dDOT14(n,R,2);
//		double A1 = box.side.v[0] * Q1;
//		double A2 = box.side.v[1] * Q2;
//		double A3 = box.side.v[2] * Q3;
		double[] A = {box.side.get0() * Q1, box.side.get1() * Q2, box.side.get2() * Q3};
//		double B1 = dFabs(A1);
//		double B2 = dFabs(A2);
//		double B3 = dFabs(A3);
		double[] B = {dFabs(A[0]), dFabs(A[1]), dFabs(A[2])};

		// early exit test
//		double depth = plane._p[3] + (0.5)*(B1+B2+B3) - dDOT(n,o1._final_posr.pos);
		RefDouble depth = new RefDouble( plane.getDepth() + (0.5)*(B[0]+B[1]+B[2]) - n.dot(o1._final_posr.pos) );
		if (depth.get() < 0) return 0;

		// find number of contacts requested
		int maxc = flags & DxGeom.NUMC_MASK;
		// if (maxc < 1) maxc = 1; // an assertion is made on entry
		if (maxc > 3) maxc = 3;	// not more than 3 contacts per box allowed

		// find deepest point
		DVector3 p = new DVector3();
//		p.v[0] = o1._final_posr.pos.v[0];
//		p.v[1] = o1._final_posr.pos.v[1];
//		p.v[2] = o1._final_posr.pos.v[2];
		p.set(o1._final_posr.pos);
//		#define FOO1(i,op) \
//		p.v[0] op (0.5)*box.side[i] * R[0+i]; \
//		p.v[1] op (0.5)*box.side[i] * R[4+i]; \
//		p.v[2] op (0.5)*box.side[i] * R[8+i];
//		#define BAR1(i,iinc) if (A ## iinc > 0) { FOO1(i,-=) } else { FOO1(i,+=) }
//		BAR1(0,1);
//		BAR1(1,2);
//		BAR1(2,3);
//		#undef FOO
//		#undef BAR
//		if (A[0] > 0) p.sum(p, R.columnAsNewVector(0), -0.5*box.side.get(0));
//		else p.sum(p, R.columnAsNewVector(0), +0.5*box.side.get(0));
//		if (A[1] > 0) p.sum(p, R.columnAsNewVector(1), -0.5*box.side.get(1));
//		else p.sum(p, R.columnAsNewVector(1), +0.5*box.side.get(1));
//		if (A[2] > 0) p.sum(p, R.columnAsNewVector(2), -0.5*box.side.get(2));
//		else p.sum(p, R.columnAsNewVector(2), +0.5*box.side.get(2));
		for (int i = 0; i < 3; i++) {
			if (A[i] > 0) 	p.eqSum(p, R.columnAsNewVector(i), -0.5*box.side.get(i));
			else 			p.eqSum(p, R.columnAsNewVector(i), +0.5*box.side.get(i));
		}

		// the deepest point is the first contact point
//		contact.pos[0] = p[0];
//		contact.pos[1] = p[1];
//		contact.pos[2] = p[2];
		contact.pos.set(p);
//		contact.normal[0] = n[0];
//		contact.normal[1] = n[1];
//		contact.normal[2] = n[2];
		contact.normal.set(n);
		contact.depth = depth.get();
		ret.set(1);//ret = 1;		// ret is number of contact points found so far
		//TZ 
		do {
			if (maxc == 1) {
				//goto done;
				done(ret.get(), contacts, skip, o1, o2);
				return ret.get();
			}

			// get the second and third contact points by starting from `p' and going
			// along the two sides with the smallest projected length.

//			#define FOO2(i,j,op) \
//			CONTACT(contact,i*skip).pos[0] = p[0] op box.side[j] * R[0+j]; \
//			CONTACT(contact,i*skip).pos[1] = p[1] op box.side[j] * R[4+j]; \
//			CONTACT(contact,i*skip).pos[2] = p[2] op box.side[j] * R[8+j];
			//contacts.get(i*skip).pos.sum(p, R.columnAsNewVector(j), op*box.side(j));
//			#define BAR2(ctact,side,sideinc) \
//			depth -= B ## sideinc; \
//			if (depth < 0) goto done; \
//			if (A ## sideinc > 0) { FOO(ctact,side,+); } else { FOO2(ctact,side,-); } \
//			CONTACT(contact,ctact*skip).depth = depth; \
//			ret++;

//			CONTACT(contact,skip).normal[0] = n[0];
//			CONTACT(contact,skip).normal[1] = n[1];
//			CONTACT(contact,skip).normal[2] = n[2];
			contacts.get(skip).normal.set(n);
			if (maxc == 3) {
//				CONTACT(contact,2*skip).normal[0] = n[0];
//				CONTACT(contact,2*skip).normal[1] = n[1];
//				CONTACT(contact,2*skip).normal[2] = n[2];
				contacts.get(2*skip).normal.set(n);
			}

			int p1, p2, p3, go;
//			if (B[0] < B[1]) {
//				if (B[2] < B[0]) goto use_side_3; 
//				else {
//					BAR2(1,0,1);	// use side 1
//					if (maxc == 2) { //goto done;
//						done(ret, contacts, skip, o1, o2);
//						return ret;
//					}
//					if (B[1] < B[2]) goto contact2_2; else goto contact2_3;
//				}
//			} else {
//				if (B[2] < B[1]) {
//					use_side_3:	// use side 3
//					BAR2(1,2,3);
//					if (maxc == 2) { //goto done;
//						done(ret, contacts, skip, o1, o2);
//						return ret;
//					}
//					if (B[0] < B[1]) goto contact2_1; else goto contact2_2;
//				} else {
//					BAR2(1,1,2);	// use side 2
//					if (maxc == 2) { //goto done;
//						done(ret, contacts, skip, o1, o2);
//						return ret;
//					}
//					if (B[0] < B[2]) goto contact2_1; else goto contact2_3;
//				}
//			}
			if (B[0] < B[1]) {
				if (B[2] < B[0]) {
					//TODO
					p1=1; p2=2; p3=3; //1 2 3
					if (B[0] < B[1]) go = 21; else go = 22;
				} else {
					p1=1; p2=0; p3=1; //1 0 1
					if (B[1] < B[2]) go = 22; else go = 23;
				}
			} else {
				if (B[2] < B[1]) {
					p1=1; p2=2; p3=3; //1 2 3
					if (B[0] < B[1]) go = 21; else go = 22;
				} else {
					p1=1; p2=1; p3=2; //1 1 2
					if (B[0] < B[2]) go = 21; else go = 23;
				}
			}
			
			BAR2(p1, p2, p3, depth, ret, contacts, skip, A, B, o1, o2, p, R, box.side);
			//TODO improve  (e.g. make depth a primitive int, reduce arguments, ...
//			BAR2_(p1, p2, depth, ret, contacts, skip, A[p2], B[p2], o1, o2, p, R, box.side);
			
			if (maxc ==2) { //goto done;
				break;
			} else {
				if (go == 21) {
					if (!BAR2(2, 0, 1, depth, ret, contacts, skip, A, B, o1, o2, p, R, box.side)) return ret.get();
					break;
				} else if (go == 22) {
					if (!BAR2(2, 1, 2, depth, ret, contacts, skip, A, B, o1, o2, p, R, box.side)) return ret.get();
					break;
				} else if (go == 23) {
					if (!BAR2(2, 2, 3, depth, ret, contacts, skip, A, B, o1, o2, p, R, box.side)) return ret.get();
					break;
				} else {
					throw new IllegalStateException("go=" + go);
				}
			}
			//TZ
//			throw new IllegalStateException();
//			contact2_1: BAR2(2,0,1); goto done;
//			contact2_2: BAR2(2,1,2); goto done;
//			contact2_3: BAR2(2,2,3); goto done;
//			#undef FOO
//			#undef BAR
		} while (false);
		//done:
//			for (int i=0; i<ret; i++) {
//				CONTACT(contact,i*skip).g1 = o1;
//				CONTACT(contact,i*skip).g2 = o2;
//			}
		done(ret.get(), contacts, skip, o1, o2);
		return ret.get();
	}

//	#define FOO2(i,j,op) \
//	CONTACT(contact,i*skip).pos[0] = p[0] op box.side[j] * R[0+j]; \
//	CONTACT(contact,i*skip).pos[1] = p[1] op box.side[j] * R[4+j]; \
//	CONTACT(contact,i*skip).pos[2] = p[2] op box.side[j] * R[8+j];
	private final void FOO2(int i, int j, int op, DContactGeomBuffer contacts, 
			int skip, DVector3 p, DMatrix3 R, DVector3 side) {
		contacts.get(i*skip).pos.eqSum(p, R.columnAsNewVector(j), op*side.get(j));
	}
	
//	#define BAR2(ctact,side,sideinc) \
//	depth -= B ## sideinc; \
//	if (depth < 0) goto done; \
//	if (A ## sideinc > 0) { FOO2(ctact,side,+); } else { FOO2(ctact,side,-); } \
//	CONTACT(contact,ctact*skip).depth = depth; \
//	ret++;
	private final boolean BAR2(int ctact, int side, int sideinc, 
			RefDouble depth, RefInt ret, DContactGeomBuffer contacts, int skip,
			double[] A, double[] B, DxGeom o1, DxGeom o2, 
			DVector3 p, DMatrix3 R, DVector3 boxSide) {
//		depth -= B ## sideinc; \
		depth.sub(B[sideinc-1]);
//		if (depth < 0) goto done; \
		if (depth.get() < 0) {
			done(ret.get(), contacts, skip, o1, o2);
			return false;
		}
//		if (A ## sideinc > 0) { FOO(ctact,side,+); } else { FOO2(ctact,side,-); } \
		if (A[sideinc-1] > 0) {
			FOO2(ctact,side, +1, contacts, skip, p, R, boxSide);
		} else {
			FOO2(ctact,side, -1, contacts, skip, p, R, boxSide);
		}
//		CONTACT(contact,ctact*skip).depth = depth; \
		contacts.get(ctact*skip).depth = depth.get();
//		ret++;
		ret.inc();
		return true;
	}
	
	private final void done(int ret, DContactGeomBuffer contacts, int skip, DxGeom o1, DxGeom o2) {
		//done:
		for (int i=0; i<ret; i++) {
//			CONTACT(contact,i*skip).g1 = o1;
//			CONTACT(contact,i*skip).g2 = o2;
			DContactGeom contact = contacts.get(i*skip);
			contact.g1 = o1;
			contact.g2 = o2;
			contact.side1 = -1;
			contact.side2 = -1;
		}
	}
	
	@Override
	public int dColliderFn (DGeom o1, DGeom o2, int flags, 
			DContactGeomBuffer contacts) {
		return dCollideBoxPlane((DxBox)o1, (DxPlane)o2, flags, contacts, 1);
	}

}
