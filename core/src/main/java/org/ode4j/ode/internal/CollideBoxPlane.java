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
import static org.ode4j.ode.internal.DxGeom.NUMC_MASK;

class CollideBoxPlane implements DColliderFn {
	//int dCollideBoxPlane (dxGeom *o1, dxGeom *o2,
	//    int flags, dContactGeom *contact, int skip)
	int dCollideBoxPlaneOld (DxBox o1, DxPlane o2,
			int flags, DContactGeomBuffer contacts, int skip)
	{
		//dIASSERT (skip >= (int)sizeof(dContactGeom));
		dIASSERT (skip == 1);
//		dIASSERT (o1.type == dBoxClass);
//		dIASSERT (o2.type == dPlaneClass);
		dIASSERT ((flags & NUMC_MASK) >= 1);

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
		int maxc = flags & NUMC_MASK;
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

	// Fill in the the non-varying contact info for the first contactCount contacts.
	// static inline void finishBoxPlaneContacts(unsigned contactCount, dContactGeom *&contact, int skip, dxGeom *o1, dxGeom *o2, const dReal *normal);
	// TZ: -> See below

	//	int dCollideBoxPlane(dxGeom *o1, dxGeom *o2,
	//						 int flags, dContactGeom *contact, int skip)
	private int dCollideBoxPlane (DxBox o1, DxPlane o2,
							 int flags, DContactGeomBuffer contacts, int skip)
	{
		// dIASSERT(skip >= (int)sizeof(dContactGeom));
		dIASSERT(skip == 1);
		//		dIASSERT(o1->type == dBoxClass);
		//		dIASSERT(o2->type == dPlaneClass);
		dIASSERT((flags & NUMC_MASK) >= 1);

		DxBox box = o1;
		DxPlane plane = o2;

		//@@@ problem: using 4-vector (plane->p) as 3-vector (normal).
    	DVector3C normal = plane.getNormal();          // normal vector

		// The number of contacts found.
		int nContactsFound = 0;

		do
		{
         DMatrix3C R = box.final_posr().R(); // rotation of box

			// project sides lengths along normal vector, get absolute values
			// A[i] will be the reduction in depth resulting from moving in direction i by an amount of side[i].
        	final double Q0 = dCalcVectorDot3_14(normal, R, 0);
			final double Q1 = dCalcVectorDot3_14(normal, R, 1);
			final double Q2 = dCalcVectorDot3_14(normal, R, 2);
        	final DVector3C side = box.side;
        	DVector3C A = new DVector3(side.get0() * Q0, side.get1() * Q1, side.get2() * Q2);

			// Find centerDepth
        	final double centerDepth = plane.getDepth() - dCalcVectorDot3(normal, box.final_posr().pos());

			// The depths (positive = deeper) of the 8 vertices will be:
			//
			// centerDepth - 0.5*(+/- A[0] +/- A[1] +/- A[2])
			//
			// for the 8 combinations of +/-.

			// What we'd *really* like is to get the vertices in depth order (deepest first), so that we can
			// easily return the maxc deepest vertices and also return early once we get to a vertex with a
			// negative depth. To achieve that efficiently, it will turn out to be useful to use |A[s]|
			// instead of A[s] when computing the depths. We store that in B.
			double[] B = {dFabs(A.get0()), dFabs(A.get1()), dFabs(A.get2())};

			// Note that A[s]=sgn(A[s])*B[s]. That means that the depths are
			// given by:
			//
			// centerDepth - 0.5*(+/- sgn(A[0])*B[0] +/- sgn(A[1])*B[1] +/- sgn(A[2])*B[2])
			//
			// The depth of the deepest vertex is achieved when the choice of + or - is opposite the
			// corresponding sgn() function. That will lead to the depth of the deepest vertex being:
			double firstContactDepth = centerDepth + 0.5 * (B[0] + B[1] + B[2]);

			// If that isn't below the plane, there is no collision and we can return immediately.
			if (firstContactDepth < 0)
			{
				break;
			}

			// The number of contacts requested. We can't return more than this many.
        	final int maxc = flags & NUMC_MASK;
			dIASSERT(maxc != 0);

			// Otherwise, we need to figure out the position of the vertex, and potentially other vertices.

			// Let p be the center point of the box.
        	DVector3C p = new DVector3(box.final_posr().pos());//.get0->pos[0], box->final_posr->pos[1], box->final_posr->pos[2]);

			// To compute the positions of each of the 8 vertices, note that side[s]*R[4*c+s] is the change
			// in coordinate c associated with moving along the full length of side s. As a result, the cth
			// coord of the vertices are given by:
			//
			// p[c] +/- 0.5*side[0]*R[4*c+0] +/- 0.5*side[1]*R[4*c+1] +/- 0.5*side[2]*R[4*c+2]
			//
			// for c in 0, 1, 2, and all 8 combinations of +/- where the choices for +/- for a vertex match
			// the choices when computing the depth of the vertex:
			//
			// centerDepth - 0.5*(+/- sgn(A[0])*B[0] +/- sgn(A[1])*B[1] +/- sgn(A[2])*B[2])
			//
			// Since the sgn() function will just flip or not flip the +/- to a -/+, and we just need to
			// keep the choices consistent between the computations of the vertices' depths and the
			// positions, we can move the sgn() to the computation of the coords to get:
			//
			// centerDepth - 0.5*(+/- B[0] +/- B[1] +/- B[2])
			//
			// and:
			//
			// p[c] +/- 0.5*sgn(A[0])*side[0]*R[4*c+0] +/- 0.5*sgn(A[1])*side[1]*R[4*c+1] +/-
			// 0.5*sgn(A[2])*side[2]*R[4*c+2]
			//
			// We can precompute signedHalfSides[s]=0.5*sgn(A[s])*side[s] and
			// signedHalfSideVectors[s][c]=signedHalfSides[s]*R[4*s+c]
			DVector3C signedHalfSides = new DVector3(
					dCopySign(0.5, A.get0()) * side.get0(),
					dCopySign(0.5, A.get1()) * side.get1(),
					dCopySign(0.5, A.get2()) * side.get2()
					);

			DVector3C[] signedHalfSideVectors = new DVector3C[3] = {
				{signedHalfSides[0] * R[0 + 0], signedHalfSides[0] * R[4 + 0], signedHalfSides[0] * R[8 + 0]},
				{signedHalfSides[1] * R[0 + 1], signedHalfSides[1] * R[4 + 1], signedHalfSides[1] * R[8 + 1]},
				{signedHalfSides[2] * R[0 + 2], signedHalfSides[2] * R[4 + 2], signedHalfSides[2] * R[8 + 2]},
			};

			// Now the vertices are given by:
			//
			// p[c] +/- signedHalfSides[0]*R[4*c+0] +/- signedHalfSides[1]*R[4*c+1] +/-
			// signedHalfSides[2]*R[4*c+2]
			//
			// The deepest vertex corresponds to always choosing "-". So we can fill it in right away.
			{
				DContactGeom currContact = contact;
				dIASSERT(contact == CONTACT(contact, nContactsFound * skip));
				currContact.pos.set0( p.get0() - signedHalfSideVectors[2].get0() - signedHalfSideVectors[1].get0() - signedHalfSideVectors[0].get0() );
				currContact.pos.set1( p.get1() - signedHalfSideVectors[2].get1() - signedHalfSideVectors[1].get1() - signedHalfSideVectors[0].get1() );
				currContact.pos.set2( p.get2() - signedHalfSideVectors[2].get2() - signedHalfSideVectors[1].get2() - signedHalfSideVectors[0].get2() );
				currContact.depth = firstContactDepth;
				if (++nContactsFound == maxc)
				{
					break;
				}
			}

			// The second deepest will correspond to choosing the - option for all but the smallest B[s].
			// That corresponds to moving along the edge that causes the least change in depth. The third
			// deepest will correspond to choosing the - option for all but the second smallest B[s]. The
			// least, second least, and third *least* deep can be found similarly. So the ranks of 6 of the
			// 8 vertices can be determined in this way. The only remaining ambiguity is which of the 2
			// other vertices is deeper, which we will handle later.
			//
			// To compute the vertices and depths efficiently in the desired order, we will presort the
			// sides and their corresponding Bs in ascending order of the Bs and we will precompute each of
			// the signedHalfSide[s]*R[4*c+s] terms as follows:
			//
			// Let minSide, midSide, maxSide be the indices of the sides such that
			// B[minSide]<=B[midSide]<=B[maxSide]
			//
			// Let orderedB[0]=B[minSide], orderedB[1]=B[midSide], and orderedB[2]=B[maxSide]
			//
			// and:
			//
			// orderedSignedHalfSideVectors[0][c] = signedHalfSideVectors[minSide]
			// orderedSignedHalfSideVectors[1][c] = signedHalfSideVectors[midSide]
			// orderedSignedHalfSideVectors[2][c] = signedHalfSideVectors[maxSide]
			//
			// The positions of the vertices will then be:
			//
			// p[c] +/- orderedHalfSideVector[0][c] +/- orderedHalfSideVector[1][c] +/-
			// orderedHalfSideVector[2][c]
			//
			// and the corresponding depths:
			//
			// centerDepth - 0.5*(+/- orderedB[0] +/- orderedB[1] +/- orderedB[2])
			//
			// We will call the following macro with the various +/- combos to compute the depth of a
			// vertex, add it to the contacts if it is inside and stop early if it isn't or we've found
			// the requested number of contacts.
//			#define ADD_VERTEX_IF_INSIDE(op1, op2, op3)                                                                                                              \
//			{                                                                                                                                                        \
//				double contactDepth = (centerDepth - 0.5 * (op3 orderedB[2] op2 orderedB[1] op1 orderedB[0]));                                                  \
//				if (contactDepth < 0.0)                                                                                                                              \
//				{                                                                                                                                                    \
//					break;                                                                                                                                           \
//				}                                                                                                                                                    \
//				dContactGeom *currContact = CONTACT(contact, nContactsFound * skip);                                                                                 \
//				currContact->pos[0] = p[0] op3 orderedSignedHalfSideVectors[2][0] op2 orderedSignedHalfSideVectors[1][0] op1 orderedSignedHalfSideVectors[0][0];     \
//				currContact->pos[1] = p[1] op3 orderedSignedHalfSideVectors[2][1] op2 orderedSignedHalfSideVectors[1][1] op1 orderedSignedHalfSideVectors[0][1];     \
//				currContact->pos[2] = p[2] op3 orderedSignedHalfSideVectors[2][2] op2 orderedSignedHalfSideVectors[1][2] op1 orderedSignedHalfSideVectors[0][2];     \
//				currContact->depth = contactDepth;                                                                                                                   \
//				if (++nContactsFound == maxc)                                                                                                                        \
//				{                                                                                                                                                    \
//					break;                                                                                                                                           \
//				}                                                                                                                                                    \
//			} // END OF MACRO DEFINITION


			// But first we need to sort the sides and Bs. We'll start with them in their original order...
        	DVector3C[] orderedSignedHalfSideVectors = new DVector3C[]{
				&(signedHalfSideVectors[0][0]),
            &(signedHalfSideVectors[1][0]),
            &(signedHalfSideVectors[2][0]),
        	};
        	DVector3C orderedB = B; // TODO copy???

			TriFunction ADD_VERTEX_IF_INSIDE = (op1, op2, op3) ->
			{
				double contactDepth = (centerDepth - 0.5 * (op3 * orderedB.get2() + op2 * orderedB.get1() + op1 * orderedB.get0()));
				if (contactDepth < 0.0)
				{
					//break;
					return true;
				}
				DContactGeom currContact = contacts.get(nContactsFound * skip); //CONTACT(contact, nContactsFound * skip);
				currContact.pos.set0( p.get0() + op3 * orderedSignedHalfSideVectors[2].get0() + op2 * orderedSignedHalfSideVectors[1].get0() + op1 * orderedSignedHalfSideVectors[0].get0() );
				currContact.pos.set1( p.get1() + op3 * orderedSignedHalfSideVectors[2].get1() + op2 * orderedSignedHalfSideVectors[1].get1() + op1 * orderedSignedHalfSideVectors[0].get1() );
				currContact.pos.set2( p.get2() + op3 * orderedSignedHalfSideVectors[2].get2() + op2 * orderedSignedHalfSideVectors[1].get2() + op1 * orderedSignedHalfSideVectors[0].get2() );
				currContact.depth = contactDepth;
				if (++nContactsFound == maxc)
				{
					//break;
					return true;
				}
				return false;
			};

			// Swap the one that corresponds to the lowest B into the 0th position.
			if (B[1] < B[0])
			{
				dxSwap(B, 1, B, 0);
				dxSwap(orderedSignedHalfSideVectors, 1, orderedSignedHalfSideVectors, 0);
			}

			if (B[2] < B[0])
			{
				dxSwap(B, 2, B, 0);
				dxSwap(orderedSignedHalfSideVectors, 2, orderedSignedHalfSideVectors, 0);
			}

			// For the second deepest vertex, we don't actually care about the order of other 2 sides, so we
			// can go ahead and process that one. Maybe we'll get lucky and exit early.
			if (ADD_VERTEX_IF_INSIDE.isBreak(+1, -1, -1)) break;

			// If we didn't get lucky, we need to sort the remaining 2 sides.
			if (B[2] < B[1])
			{
				dxSwap(B, 2, B, 1);
				dxSwap(orderedSignedHalfSideVectors, 2, orderedSignedHalfSideVectors, 1);
			}

			// Now we can process the third deepest vertex.
			if (ADD_VERTEX_IF_INSIDE.isBreak(-1, +1, -1)) break;

			// Determine which of the 2 middle vertices is deeper and process them in the correct order.
			if (B[0] + B[1] < B[2])
			{
				if (ADD_VERTEX_IF_INSIDE.isBreak(+1, +1, -1)) break;
				if (ADD_VERTEX_IF_INSIDE.isBreak(-1, -1, +1)) break;
			}
			else
			{
				if (ADD_VERTEX_IF_INSIDE.isBreak(-1, -1, +1)) break;
				if (ADD_VERTEX_IF_INSIDE.isBreak(+1, +1, -1)) break;
			}

			// Process the 3 remaining vertices in order.
			if (ADD_VERTEX_IF_INSIDE.isBreak(+1, -1, +1)) break;
			if (ADD_VERTEX_IF_INSIDE.isBreak(-1, +1, +1)) break;
			if (ADD_VERTEX_IF_INSIDE.isBreak(+1, +1, +1)) break;
		}
		while (false);

		if (nContactsFound != 0)
		{
			finishBoxPlaneContacts(nContactsFound, contacts, skip, box, plane, normal);
		}

		int retVal = nContactsFound;
		return retVal;
	}

	private interface TriFunction {
		boolean isBreak(int op1, int op2, int op3);
	}


	// Fill in the the non-varying contact info for the first n contacts.
	static
	// void finishBoxPlaneContacts(unsigned contactCount, dContactGeom *&contact, int skip, dxGeom *o1, dxGeom *o2, const dReal *normal)
	void finishBoxPlaneContacts(int contactCount, DContactGeomBuffer contacts, int skip, DxGeom o1, DxGeom o2, DVector3C normal)
	{
		for (int contactIndex = 0; contactIndex != contactCount; ++contactIndex)
		{
			DContactGeom currContact = contacts.get(contactIndex * skip); //CONTACT(contact, contactIndex * skip);
			currContact.g1 = o1;
			currContact.g2 = o2;
			currContact.side1 = -1;
			currContact.side2 = -1;

			//			currContact.normal[0] = normal[0];
			//			currContact.normal[1] = normal[1];
			//			currContact.normal[2] = normal[2];
			currContact.normal.set(normal);
		}

	}



	@Override
	public int dColliderFn (DGeom o1, DGeom o2, int flags, 
			DContactGeomBuffer contacts) {
		return dCollideBoxPlane((DxBox)o1, (DxPlane)o2, flags, contacts, 1);
	}

}
