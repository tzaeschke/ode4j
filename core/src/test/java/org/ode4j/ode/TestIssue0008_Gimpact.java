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
package org.ode4j.ode;

import static org.ode4j.ode.OdeConstants.dContactSlip1;
import static org.ode4j.ode.OdeConstants.dContactSlip2;
import static org.ode4j.ode.OdeConstants.dContactSoftCFM;
import static org.ode4j.ode.OdeConstants.dContactSoftERP;

import java.util.ArrayList;

import org.junit.Test;
import org.ode4j.math.DVector3;

public class TestIssue0008_Gimpact {

    private DWorld world;
    private DSpace space;
    private DJointGroup contactgroup;
    private final ArrayList<DJoint> contactJoints = new ArrayList<DJoint>();

    /**
     * Issue #8: Gimpact Radix sort failed for large trimeshes.
     */
    @Test
    public void testLargeTrimesh() {
    	//TODO Test does not fail yet, even if the last part (0x3FF) is removed from line 76 in RadixSort.
    	
//		float[] size = new float[]{ 5.0f, 5.0f, 2.5f };
//
//		float[] verticesTemplate = new float[]{ 
//				-size[0], -size[1], size[2],
//				 size[0], -size[1], size[2],
//				 size[0],  size[1], size[2],
//				-size[0],  size[1], size[2],
//				0f, 0f, 0f};
//
//		int[] indicesTemplate = new int[] {
//				0, 1, 4,
//				1, 2, 4,
//				2, 3, 4,
//				3, 0, 4
//		};

    	float dX = -10;
    	float dY = -10;
    	float dZ = -10;
    	
		int nVx = 1000;
		
		float [] vertices = new float[nVx*nVx*3];
		for (int y = 0; y < nVx; y++) {
			for (int x = 0; x < nVx; x++) {
				int pos = (x+y*nVx) * 3; 
				vertices[pos] = x + dX; 
				vertices[pos+1] = y + dY; 
				vertices[pos+2] = ((x+y) % 10) + dZ; 
			}
		}
		
		int[] indices = new int[(nVx-1)*(nVx-1)*2*3];
		int iI = 0;
		for (int y = 0; y < nVx-1; y++) {
			for (int x = 0; x < nVx-1; x++) {
				int pos = (x+y*(nVx-1)) * 6;
				indices[pos] = iI; 
				indices[pos+1] = iI+1; 
				indices[pos+2] = iI+1+nVx; 

				indices[pos+3] = iI; 
				indices[pos+4] = iI+1+nVx; 
				indices[pos+5] = iI+nVx;
				
				iI++;
			}
		}
		
        OdeHelper.initODE2(0);
        world = OdeHelper.createWorld();
        space = OdeHelper.createHashSpace();
        DSpace tSpace = OdeHelper.createQuadTreeSpace(
        		new DVector3(nVx/2+dX,nVx/2+dY,1+dZ), 
        		new DVector3(nVx, nVx, 100), 
        		4);//HashSpace();
        contactgroup = OdeHelper.createJointGroup();

        
        DTriMeshData Data = OdeHelper.createTriMeshData();
		Data.build(vertices, indices);
		OdeHelper.createTriMesh(space, Data, null, null, null);
		space.add(tSpace);
		
		
		DSphere sphere = OdeHelper.createSphere(space, 10);
		world.setGravity(0, -1, -1);
	    DBody sBody = OdeHelper.createBody(world);
	    DMass sMass = OdeHelper.createMass();
	    sMass.setSphereTotal(10, 2);
	    sBody.setMass(sMass);
	    sphere = OdeHelper.createSphere(1);
	    sphere.setBody(sBody);
	    sBody.setPosition(10, 10, 10);
	    //sphere.setData(obs);
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
