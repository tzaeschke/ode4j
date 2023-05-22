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
 *       the file ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.         *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT, ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT for more   *
 * details.                                                              *
 *                                                                       *
 *************************************************************************/
package org.ode4j.tests;

import org.junit.Test;
import org.ode4j.ode.*;

import java.util.ArrayList;

import static org.ode4j.ode.OdeConstants.*;

public class TestIssue00XX_HashSpace_EndlessLoop {

    private DWorld world;
    private DSpace space;
    private DJointGroup contactgroup;
    private final ArrayList<DJoint> contactJoints = new ArrayList<DJoint>();

    /**
     * Issue #xx: HashSpace enters an endless loop resulting in OutOfMemoryException.
     * TODO TZ: This needs to be fixed!!!!
     */
    @Test
    public void testTrimesh() {
		float[] size = new float[]{ 5.0f, 5.0f, 2.5f };
		float[] vertices = new float[]{
				-size[0], -size[1], size[2],
				 size[0], -size[1], size[2],
				 size[0],  size[1], size[2],
				-size[0],  size[1], size[2],
				0f, 0f, 0f};
		int[] indices = new int[] {
				0, 1, 4,
				1, 2, 4,
				2, 3, 4,
				3, 0, 4
		};

        OdeHelper.initODE2(0);
        world = OdeHelper.createWorld();
        //space = OdeHelper.createHashSpace();  // TODO TZ this fails with OOM Exceptions
        space = OdeHelper.createSapSpace(DSapSpace.AXES.XYZ);
        contactgroup = OdeHelper.createJointGroup();

        DTriMeshData Data = OdeHelper.createTriMeshData();
		Data.build(vertices, indices);
		OdeHelper.createTriMesh(space, Data, null, null, null);

		DSphere sphere = OdeHelper.createSphere(space, 10);
	    DBody sBody = OdeHelper.createBody(world);
	    //sphere = OdeHelper.createSphere(1);
	    sphere.setBody(sBody);
	    sBody.setPosition(10, 10, 10);
	    space.add(sphere);
		
	    for (int i = 0; i < 100; i++) {
	    	OdeHelper.spaceCollide(space, 0, nearCallback);
	    	world.quickStep(10.0 / 60);
	    }

        OdeHelper.closeODE();
    }

    
    private final DGeom.DNearCallback nearCallback = new DGeom.DNearCallback() {
        @Override
        public void call(Object data, DGeom o1, DGeom o2) {
            nearCallback(data, o1, o2);
        }
    };

    
    private void nearCallback(Object data, DGeom o1, DGeom o2) {
        assert (o1 != null);
        assert (o2 != null);

        if (o1 instanceof DSpace || o2 instanceof DSpace) {
            OdeHelper.spaceCollide2(o1, o2, data, nearCallback);
            return;
        }

        //  fprintf(stderr,"testing geoms %p %p", o1, o2);
        final int N = 32;
        DContactBuffer contacts = new DContactBuffer(N);
        int n = OdeHelper.collide(o1, o2, N, contacts.getGeomBuffer());
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                DContact contact = contacts.get(i);
                contact.surface.slip1 = 0.1;
                contact.surface.slip2 = 0.1;
                contact.surface.mode = dContactSoftERP | dContactSoftCFM | dContactSlip1 | dContactSlip2;
                contact.surface.mu = OdeConstants.dInfinity;
                contact.surface.soft_erp = 0.99;
                contact.surface.soft_cfm = 0.10;
                contact.surface.bounce = 0;
                DJoint c = OdeHelper.createContactJoint(world, contactgroup, contact);
                c.attach(contact.geom.g1.getBody(),
                        contact.geom.g2.getBody());
                contactJoints.add(c);
            }
        }
    }

}
