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

/**
 * Test harness for bugs specific ode4j.
 *
 * @author Tilmann Zaeschke
 */
public class BugsTest {

    /**
     */
	@Test
    public void testBodyDampening() {
    	int NUM = 10;			/* number of boxes */
    	double SIDE = (0.2);		/* side length of a box */
    	double MASS = (1.0);		/* mass of a box */
    	double RADIUS = (0.1732f);	/* sphere radius */


    	/* dynamics and collision objects */

    	DBody[] body = new DBody[NUM];
    	DBallJoint[] joint = new DBallJoint[NUM-1];
    	DSphere[] sphere=new DSphere[NUM];

    	int i;
		double k;
		DMass m;

		/* create world */
//		OdeHelper.initODE2(0);
		DWorld world = OdeHelper.createWorld();
		DSpace space = OdeHelper.createHashSpace(null);
		DJointGroup contactgroup = OdeHelper.createJointGroup ();
		world.setGravity (0, 0, -0.5);
		OdeHelper.createPlane (space,0,0,1,0);

		//TZ
		m = OdeHelper.createMass();
		for (i=0; i<NUM; i++) {
			body[i] = OdeHelper.createBody(world);
			k = i*SIDE;
			body[i].setPosition(k,k,k+0.4);
			m.setBox(1,SIDE,SIDE,SIDE);
			m.adjust (MASS);
			body[i].setMass (m);
			sphere[i] = OdeHelper.createSphere (space,RADIUS);
			sphere[i].setBody(body[i]);
			body[i].setAngularDamping(0.3);
			body[i].setAngularDampingThreshold(0.9);
			body[i].getAngularDamping();
			body[i].getAngularDampingThreshold();
			body[i].setLinearDamping(0.3);
			body[i].setLinearDampingThreshold(0.9);
			body[i].getLinearDamping();
			body[i].getLinearDampingThreshold();
			body[i].setDampingDefaults();
		}
		for (i=0; i<(NUM-1); i++) {
			joint[i] = OdeHelper.createBallJoint(world,null);
			joint[i].attach(body[i],body[i+1]);
			k = (i+0.5)*SIDE;
			joint[i].setAnchor(k,k,k+0.4);
		}

		/* run simulation */
//		dsSimulationLoop (args,640,480,this);

		contactgroup.destroy();
		space.destroy();
		world.destroy();
//		OdeHelper.closeODE();
    }
}
