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
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DColliderFn;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;
import org.ode4j.ode.internal.cpp4j.java.RefInt;

import static org.ode4j.ode.internal.Common.*;

/**
 * Do not use directly, this is an internal class!
 */
class CollideBoxBox implements DColliderFn {
	//int dCollideBoxBox (dxGeom *o1, dxGeom *o2, int flags,
	//    dContactGeom *contact, int skip)
	public int dCollideBoxBox (DxBox o1, DxBox o2, int flags,
			DContactGeomBuffer contacts, int skip)
	{
		//TZ dIASSERT (skip >= (int)sizeof(dContactGeom));
		dIASSERT(skip ==1);
//		dIASSERT (o1.type == dBoxClass);
//		dIASSERT (o2.type == dBoxClass);
		dIASSERT ((flags & DxGeom.NUMC_MASK) >= 1);

		DVector3 normal = new DVector3();
		RefDouble depth = new RefDouble(0); //double depth;
		RefInt code = new RefInt();//int code;
		//int num = dBoxBox (o1.final_posr.pos,o1.final_posr.R,b1.side, 
		//		 o2.final_posr.pos,o2.final_posr.R,b2.side,
		//	     normal,&depth,&code,flags,contact,skip);
		int num = DxBox.dBoxBox (
				o1.final_posr().pos(), o1.final_posr().R(),o1.side, 
				o2.final_posr().pos(), o2.final_posr().R(),o2.side,
				normal,depth,code,flags,contacts,skip);
//		for (int i=0; i<num; i++) {
//			CONTACT(contact,i*skip).normal[0] = -normal.v[0];
//			CONTACT(contact,i*skip).normal[1] = -normal.v[1];
//			CONTACT(contact,i*skip).normal[2] = -normal.v[2];
//			CONTACT(contact,i*skip).g1 = o1;
//			CONTACT(contact,i*skip).g2 = o2;
//			currContact->side1 = -1;
//		    currContact->side2 = -1;
//		}
		for (int i=0; i<num; i++) {
			DContactGeom c = contacts.get(i*skip);
			c.normal.set(normal).scale(-1);
			c.g1 = o1;
			c.g2 = o2;
			c.side1 = -1;
			c.side2 = -1;
		}
		return num;
	}

	@Override
	public int dColliderFn(DGeom o1, DGeom o2, int flags,
			DContactGeomBuffer contacts) {
		return dCollideBoxBox((DxBox)o1, (DxBox)o2, flags, contacts, 1);
	}
}
