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

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DColliderFn;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;

import static org.ode4j.ode.internal.Common.dDOUBLE;
import static org.ode4j.ode.internal.Common.dIASSERT;


/**
 * Cylinder-Plane collider by Christoph Beyer ( boernerb@web.de )
 *
 * This testing basically comes down to testing the intersection
 * of the cylinder caps (discs) with the plane.
 * 
 */
class CollideCylinderPlane extends DxCollisionUtil implements DColliderFn {

	//TODO calculate toleranz properly
	private static final double toleranz;
	static {
		if (dDOUBLE) {
			toleranz = 0.0001;
		} else {
			toleranz = 0.0000001;
		}
	}

	int dCollideCylinderPlane(DxCylinder cylinder, DxPlane plane, int flags, DContactGeomBuffer contacts, int skip)
	{
		dIASSERT (skip == 1 );//>= 1,(int)sizeof(dContactGeom));
		//	dIASSERT (Cylinder.type == dCylinderClass);
		//	dIASSERT (Plane.type == dPlaneClass);
		dIASSERT ((flags & DxGeom.NUMC_MASK) >= 1);

		int GeomCount = 0; // count of used contactgeoms

		//#ifdef dSINGLE
		//	const dReal toleranz = (0.0001);
		//#endif
		//#ifdef dDOUBLE
		//	const dReal toleranz = (0.0000001);
		//#endif

		// Get the properties of the cylinder (length+radius)
		double radius, length;
		//dGeomCylinderGetParams(cylinder, &radius, &length);
		radius = cylinder.getRadius();
		length = cylinder.getLength();
		DVector3C cylpos = cylinder.final_posr().pos();
		// and the plane
		//	dVector4 planevec;
		//	dGeomPlaneGetParams(plane, planevec);
		//	dVector3 PlaneNormal = {planevec[0],planevec[1],planevec[2]};
		//	//dVector3 PlanePos = {planevec[0] * planevec[3],planevec[1] * planevec[3],planevec[2] * planevec[3]};
		DVector3C planevec = plane.getNormal();
		double planeDepth = plane.getDepth(); 
		DVector3 PlaneNormal = new DVector3(planevec);

		DContactGeom contact = contacts.get(); 
		int contactPos = 0;

		DVector3 G1Pos1 = new DVector3(), G1Pos2 = new DVector3();
		DVector3C vDir1;
		//	vDir1[0] = cylinder.final_posr.R[2];
		//	vDir1[1] = cylinder.final_posr.R[6];
		//	vDir1[2] = cylinder.final_posr.R[10];
		vDir1 = cylinder.final_posr().R().columnAsNewVector(2);

		double s;
		s = length * (0.5);
		//	G1Pos2[0] = vDir1[0] * s + cylpos[0];
		//	G1Pos2[1] = vDir1[1] * s + cylpos[1];
		//	G1Pos2[2] = vDir1[2] * s + cylpos[2];
		G1Pos2.eqSum( cylpos, vDir1, s );

		//	G1Pos1[0] = vDir1[0] * -s + cylpos[0];
		//	G1Pos1[1] = vDir1[1] * -s + cylpos[1];
		//	G1Pos1[2] = vDir1[2] * -s + cylpos[2];
		G1Pos1.eqSum(cylpos, vDir1, -s);

		// parallel-check
		//s = vDir1[0] * PlaneNormal[0] + vDir1[1] * PlaneNormal[1] + vDir1[2] * PlaneNormal[2];
		s = vDir1.dot(PlaneNormal);
		if(s < 0)
			s += (1.0); // is ca. 0, if vDir1 and PlaneNormal are parallel
		else
			s -= (1.0); // is ca. 0, if vDir1 and PlaneNormal are parallel
		if(s < toleranz && s > (-toleranz))
		{
			// discs are parallel to the plane

			// 1.compute if, and where contacts are
			DVector3 P = new DVector3();
			s = planeDepth - planevec.dot(G1Pos1);
			double t;
			t = planeDepth - planevec.dot(G1Pos2);
			if(s >= t) // s == t does never happen, 
			{
				if(s >= 0)
				{
					// 1. Disc
					P.set(G1Pos1);//dVector3Copy(G1Pos1, P);
				}
				else
					return GeomCount; // no contacts
			}
			else
			{
				if(t >= 0)
				{
					// 2. Disc
					P.set(G1Pos2);//dVector3Copy(G1Pos2, P);
				}
				else
					return GeomCount; // no contacts
			}

			// 2. generate a coordinate-system on the disc
			DVector3 V1 = new DVector3(), V2 = new DVector3();
			if(vDir1.get0() < toleranz && vDir1.get0() > (-toleranz))
			{
				// not x-axis
				//			V1[0] = vDir1[0] + (1.0); // random value
				//			V1[1] = vDir1[1];
				//			V1[2] = vDir1[2];
				V1.set(vDir1).add(0, 1.0);
			}
			else
			{
				// maybe x-axis
				//			V1[0] = vDir1[0];
				//			V1[1] = vDir1[1] + (1.0); // random value
				//			V1[2] = vDir1[2];
				V1.set(vDir1).add(1, 1.0);
			}
			// V1 is now another direction than vDir1
			// Cross-product
			dVector3Cross(V1, vDir1, V2);
			// make unit V2
			t = dVector3Length(V2);
			t = radius / t;
			dVector3Scale(V2, t);
			// cross again
			dVector3Cross(V2, vDir1, V1);
			// |V2| is 'radius' and vDir1 unit, so |V1| is 'radius'
			// V1 = first axis
			// V2 = second axis

			// 3. generate contactpoints

			// Potential contact 1
			dVector3Add(P, V1, contact.pos);
			contact.depth = planeDepth - planevec.dot(contact.pos);
			if(contact.depth > 0)
			{
				dVector3Copy(PlaneNormal, contact.normal);
				contact.g1 = cylinder;
				contact.g2 = plane;
				contact.side1 = -1;
				contact.side2 = -1;
				GeomCount++;
				if( GeomCount >= (flags & DxGeom.NUMC_MASK))
					return GeomCount; // enough contactgeoms
				contactPos += skip;
				contact = contacts.get(contactPos);//(dContactGeom *)((char *)contact + skip);
			}

			// Potential contact 2
			dVector3Subtract(P, V1, contact.pos);
			contact.depth = planeDepth - dVector3Dot(planevec, contact.pos);
			if(contact.depth > 0)
			{
				dVector3Copy(PlaneNormal, contact.normal);
				contact.g1 = cylinder;
				contact.g2 = plane;
				contact.side1 = -1;
				contact.side2 = -1;
				GeomCount++;
				if( GeomCount >= (flags & DxGeom.NUMC_MASK))
					return GeomCount; // enough contactgeoms
				contactPos += skip;
				contact = contacts.get(contactPos);//(dContactGeom *)((char *)contact + skip);
			}

			// Potential contact 3
			dVector3Add(P, V2, contact.pos);
			contact.depth = planeDepth - dVector3Dot(planevec, contact.pos);
			if(contact.depth > 0)
			{
				dVector3Copy(PlaneNormal, contact.normal);
				contact.g1 = cylinder;
				contact.g2 = plane;
				contact.side1 = -1;
				contact.side2 = -1;
				GeomCount++;
				if( GeomCount >= (flags & DxGeom.NUMC_MASK))
					return GeomCount; // enough contactgeoms
				contactPos += skip;
				contact = contacts.get(contactPos);//(dContactGeom *)((char *)contact + skip);
			}

			// Potential contact 4
			dVector3Subtract(P, V2, contact.pos);
			contact.depth = planeDepth - dVector3Dot(planevec, contact.pos);
			if(contact.depth > 0)
			{
				dVector3Copy(PlaneNormal, contact.normal);
				contact.g1 = cylinder;
				contact.g2 = plane;
				contact.side1 = -1;
				contact.side2 = -1;
				GeomCount++;
				if( GeomCount >= (flags & DxGeom.NUMC_MASK))
					return GeomCount; // enough contactgeoms
				contact = null;//contacts.inc(skip).get();//(dContactGeom *)((char *)contact + skip);
			}
		}
		else
		{
			double t = dVector3Dot(PlaneNormal, vDir1);
			DVector3 C = new DVector3();
			//		C[0] = vDir1[0] * t - PlaneNormal[0];
			//		C[1] = vDir1[1] * t - PlaneNormal[1];
			//		C[2] = vDir1[2] * t - PlaneNormal[2];
			C.eqSum(vDir1, t, PlaneNormal, -1);
			s = dVector3Length(C);
			// move C onto the circle
			s = radius / s;
			dVector3Scale(C, s);

			// deepest point of disc 1
			dVector3Add(C, G1Pos1, contact.pos);

			// depth of the deepest point
			contact.depth = planeDepth - dVector3Dot(planevec, contact.pos);
			if(contact.depth >= 0)
			{
				dVector3Copy(PlaneNormal, contact.normal);
				contact.g1 = cylinder;
				contact.g2 = plane;
				contact.side1 = -1;
				contact.side2 = -1;
				GeomCount++;
				if( GeomCount >= (flags & DxGeom.NUMC_MASK))
					return GeomCount; // enough contactgeoms
				contactPos += skip;
				contact = contacts.get(contactPos);//(dContactGeom *)((char *)contact + skip);
			}

			// C is still computed

			// deepest point of disc 2
			dVector3Add(C, G1Pos2, contact.pos);

			// depth of the deepest point
			//contact.depth = planeDepth - planevec[0] * contact.pos[0] - planevec[1] * contact.pos[1] - planevec[2] * contact.pos[2];
			contact.depth = planeDepth - planevec.dot(contact.pos);
			if(contact.depth >= 0)
			{
				dVector3Copy(PlaneNormal, contact.normal);
				contact.g1 = cylinder;
				contact.g2 = plane;
				contact.side1 = -1;
				contact.side2 = -1;
				GeomCount++;
				if( GeomCount >= (flags & DxGeom.NUMC_MASK))
					return GeomCount; // enough contactgeoms
				contact = null;//contacts.inc(skip).get();//(dContactGeom *)((char *)contact + skip);
			}
		}
		return GeomCount;
	}

	@Override
	public int dColliderFn(DGeom o1, DGeom o2, int flags,
			DContactGeomBuffer contacts) {
		return dCollideCylinderPlane((DxCylinder)o1, (DxPlane)o2, flags, contacts, 1);
	}
}