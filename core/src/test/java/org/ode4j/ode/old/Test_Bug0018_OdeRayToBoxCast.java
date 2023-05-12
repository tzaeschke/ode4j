/*************************************************************************
 *                                                                       *
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
 *       the file ODE4J-LICENSE-BSD.TXT.                                 *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and ODE4J-LICENSE-BSD.TXT for more details.               *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode.old;

import org.junit.Test;
import org.ode4j.ode.*;

import static org.junit.Assert.*;

/**
 *  Shows an incorrect calculation of the contact normal collision beam and boxing
 */
public class Test_Bug0018_OdeRayToBoxCast
{

    public static void main(String[] args) {
		new Test_Bug0018_OdeRayToBoxCast().test();
    }
	
	
	@Test
	public void test() {
		OdeHelper.initODE2(0);

        DSpace rootSpace = OdeHelper.createHashSpace();

        final int maxContacts = 5;

        OdeHelper.createBox(rootSpace, 1, 1, 1);   //result contact - invalid normal  ([ -0.0, -0.0, -1.0 ])
        //OdeHelper.createSphere(rootSpace, 1);    //result contact - OK  normal  DVector3[ -1.0, 0.0, 0.0 ]

        DRay rayCastGeom = OdeHelper.createRay(rootSpace, 1);
        rayCastGeom.set(-10, 0, 0, 1, 0, 0);
        rayCastGeom.setLength(30);

        OdeHelper.spaceCollide(rootSpace, null, new DGeom.DNearCallback() {

        	@Override
            public void call(Object data, DGeom o1, DGeom o2) {
//                boolean isSpace0 = (o1 instanceof DSpace);
//                boolean isSpace1 = (o2 instanceof DSpace);
//                if(isSpace0 || isSpace1)
//                {
//                    OdeHelper.spaceCollide2(o1, o2, data, this);
//                }
//                else
//                {
//                    if (o1 == o2)
//                        return;

                    DContactBuffer contacts = new DContactBuffer(maxContacts);
                    int numContacts = OdeHelper.collide(o1, o2, maxContacts, contacts.getGeomBuffer());
                    if (numContacts == 0)
                        return;

                    DContact contact = contacts.get(0);
                    assertTrue(Math.abs(contact.geom.normal.get0()) > 0.99);
                    assertTrue(Math.abs(contact.geom.normal.get1()) < 0.01);
                    assertTrue(Math.abs(contact.geom.normal.get2()) < 0.01);
//                    for (DContact dcon: contacts) {
//                    	System.out.println("DCon: " + dcon.geom.normal);
//                    }
//
//                    //Results !!!!
//                    System.out.println(contact.geom.normal);
//                    System.out.println(contact.geom.normal.isEq(new DVector3(-1, 0, 0)));

//                }
            }

        });

        OdeHelper.closeODE();
    }

}
