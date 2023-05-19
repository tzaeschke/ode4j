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
import org.ode4j.ode.internal.cpp4j.java.RefDouble;
import org.ode4j.ode.internal.cpp4j.java.RefInt;

import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.Common.dFabs;
import static org.ode4j.ode.internal.Common.dIASSERT;

class CollideBoxPlane implements DColliderFn {
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

		DxBox box = o1;
		DxPlane plane = o2;

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
		final DMatrix3C R = o1.final_posr().R();		// rotation of box
		//final double []n = plane._p;		// normal vector
		DVector3C n = plane.getNormal();
		
		// project sides lengths along normal vector, get absolute values
		double Q1 = dCalcVectorDot3_14(n,R,0);
		double Q2 = dCalcVectorDot3_14(n,R,1);
		double Q3 = dCalcVectorDot3_14(n,R,2);
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
		RefDouble depth = new RefDouble( plane.getDepth() + (0.5)*(B[0]+B[1]+B[2]) - n.dot(o1.final_posr().pos()) );
		if (depth.get() < 0) return 0;

		// find number of contacts requested
		int maxc = flags & DxGeom.NUMC_MASK;
		// if (maxc < 1) maxc = 1; // an assertion is made on entry
		if (maxc > 4) maxc = 4;	// not more than 4 contacts per box allowed

		// find deepest point
		DVector3 p = new DVector3();
//		p.v[0] = o1._final_posr.pos.v[0];
//		p.v[1] = o1._final_posr.pos.v[1];
//		p.v[2] = o1._final_posr.pos.v[2];
		p.set(o1.final_posr().pos());
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
		//substitute for FOO and BAR above (TZ)
		for (int i = 0; i < 3; i++) {
			if (A[i] > 0) 	p.eqSum(p, R.columnAsNewVector(i), -0.5*box.side.get(i));
			else 			p.eqSum(p, R.columnAsNewVector(i), +0.5*box.side.get(i));
		}

		// the deepest point is the first contact point
//		contact.pos[0] = p[0];
//		contact.pos[1] = p[1];
//		contact.pos[2] = p[2];
		contact.pos.set(p);
		contact.depth = depth.get();
		ret.set(1);//ret = 1;		// ret is number of contact points found so far
		//TZ 
		do {
			if (maxc == 1) {
				//goto done;
				done(ret, contacts, skip, o1, o2, maxc, depth.get(), p, n);
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
			
			BAR2(p1, p2, p3, depth, ret, contacts, skip, A, B, o1, o2, p, R, box.side, maxc,n);
			//TODO improve  (e.g. make depth a primitive int, reduce arguments, ...
//			BAR2_(p1, p2, depth, ret, contacts, skip, A[p2], B[p2], o1, o2, p, R, box.side);
			
			if (maxc ==2) { //goto done;
				break;
			} else {
				if (go == 21) {
					if (!BAR2(2, 0, 1, depth, ret, contacts, skip, A, B, o1, o2, p, R, box.side, maxc, n)) return ret.get();
					break;
				} else if (go == 22) {
					if (!BAR2(2, 1, 2, depth, ret, contacts, skip, A, B, o1, o2, p, R, box.side, maxc, n)) return ret.get();
					break;
				} else if (go == 23) {
					if (!BAR2(2, 2, 3, depth, ret, contacts, skip, A, B, o1, o2, p, R, box.side, maxc, n)) return ret.get();
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
		done(ret, contacts, skip, o1, o2, maxc, depth.get(), p, n);
		return ret.get();
	}

//	#define FOO2(i,j,op) \
//	CONTACT(contact,i*skip).pos[0] = p[0] op box.side[j] * R[0+j]; \
//	CONTACT(contact,i*skip).pos[1] = p[1] op box.side[j] * R[4+j]; \
//	CONTACT(contact,i*skip).pos[2] = p[2] op box.side[j] * R[8+j];
	private void FOO2(int i, int j, int op, DContactGeomBuffer contacts,
			int skip, DVector3 p, DMatrix3C R, DVector3 side) {
		contacts.get(i*skip).pos.eqSum(p, R.viewCol(j), op*side.get(j));
	}
	
//  #define BAR2(ctact,side,sideinc) \
//  if (depth - B ## sideinc < 0) goto done; \
//  if (A ## sideinc > 0) { FOO2(ctact,side,+); } else { FOO2(ctact,side,-); } \
//  CONTACT(contact,ctact*skip).depth = depth; \
//  ret++;
    private boolean BAR2(final int ctact, final int side, final int sideinc,
            RefDouble depth, RefInt ret, DContactGeomBuffer contacts, final int skip,
            double[] A, double[] B, DxGeom o1, DxGeom o2, 
            DVector3 p, DMatrix3C R, DVector3 boxSide, final int maxc, 
            DVector3C n) {
    //  if (depth - B ## sideinc < 0) goto done; \
        if (depth.get()-B[sideinc-1] < 0) {
            done(ret, contacts, skip, o1, o2, maxc, depth.get(), p, n);
            return false;
        }
//      if (A ## sideinc > 0) { FOO(ctact,side,+); } else { FOO2(ctact,side,-); } \
        if (A[sideinc-1] > 0) {
            FOO2(ctact,side, +1, contacts, skip, p, R, boxSide);
        } else {
            FOO2(ctact,side, -1, contacts, skip, p, R, boxSide);
        }
//      CONTACT(contact,ctact*skip).depth = depth; \
        contacts.get(ctact*skip).depth = depth.get() - B[sideinc-1];
//      ret++;
        ret.inc();
        return true;
    }
    
//  #define BAR2(ctact,side,sideinc) \
//  depth -= B ## sideinc; \
//  if (depth < 0) goto done; \
//  if (A ## sideinc > 0) { FOO2(ctact,side,+); } else { FOO2(ctact,side,-); } \
//  CONTACT(contact,ctact*skip).depth = depth; \
//  ret++;
//    private final boolean BAR2_OLD(int ctact, int side, int sideinc, 
//            RefDouble depth, RefInt ret, DContactGeomBuffer contacts, int skip,
//            double[] A, double[] B, DxGeom o1, DxGeom o2, 
//            DVector3 p, DMatrix3C R, DVector3 boxSide) {
////      depth -= B ## sideinc; \
//        depth.sub(B[sideinc-1]);
////      if (depth < 0) goto done; \
//        if (depth.get() < 0) {
//            done(ret.get(), contacts, skip, o1, o2, maxc);
//            return false;
//        }
////      if (A ## sideinc > 0) { FOO(ctact,side,+); } else { FOO2(ctact,side,-); } \
//        if (A[sideinc-1] > 0) {
//            FOO2(ctact,side, +1, contacts, skip, p, R, boxSide);
//        } else {
//            FOO2(ctact,side, -1, contacts, skip, p, R, boxSide);
//        }
////      CONTACT(contact,ctact*skip).depth = depth; \
//        contacts.get(ctact*skip).depth = depth.get();
////      ret++;
//        ret.inc();
//        return true;
//    }
    
	private void done(RefInt ret, DContactGeomBuffer contacts, int skip, DxGeom o1, DxGeom o2,
	        final int maxc, final double depth, DVector3C p, DVector3C n) {
		//done:
	    if (maxc == 4 && ret.get() == 3) { // If user requested 4 contacts, and the first 3 were created...
	        // Combine contacts 2 and 3 (vectorial sum) and get the fourth one
	        // Result: if a box face is completely inside a plane, contacts are created for all the 4 vertices
	        //double d4 = CONTACT(contact,1*skip).depth + CONTACT(contact,2*skip).depth - depth;  // depth is the depth for first contact
	        double d4 = contacts.get(1*skip).depth + contacts.get(2*skip).depth - depth;
	        if (d4 > 0) {
//	            CONTACT(contact,3*skip)->pos[0] = CONTACT(contact,1*skip).pos[0] + CONTACT(contact,2*skip).pos[0] - p[0]; // p is the position of first contact
//	            CONTACT(contact,3*skip)->pos[1] = CONTACT(contact,1*skip).pos[1] + CONTACT(contact,2*skip).pos[1] - p[1];
//	            CONTACT(contact,3*skip)->pos[2] = CONTACT(contact,1*skip).pos[2] + CONTACT(contact,2*skip).pos[2] - p[2];
//	            CONTACT(contact,3*skip)->depth  = d4;
	            DContactGeom c3 = contacts.get(3*skip);
	            c3.pos.set( contacts.get(1*skip).pos ).add( contacts.get(2*skip).pos).sub(p);
	            c3.depth = d4;
	            ret.inc();
	        }
	    }

		for (int i=0; i<ret.get(); i++) {
//			CONTACT(contact,i*skip).g1 = o1;
//			CONTACT(contact,i*skip).g2 = o2;
			DContactGeom contact = contacts.get(i*skip);
			contact.g1 = o1;
			contact.g2 = o2;
			contact.side1 = -1;
			contact.side2 = -1;
			
			contact.normal.set(n);
		}
	}
	
	@Override
	public int dColliderFn (DGeom o1, DGeom o2, int flags, 
			DContactGeomBuffer contacts) {
		return dCollideBoxPlane((DxBox)o1, (DxPlane)o2, flags, contacts, 1);
	}

}
