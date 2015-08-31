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
package org.ode4j.demo;

import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsDrawLine;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DDoubleHingeJoint;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;

public class DemoDHinge extends dsFunctions {

	private DWorld world;
	private DSpace space;
	private DBody body1;
	private DBody body2;
	private DDoubleHingeJoint joint1; 
	private DDoubleHingeJoint joint2;
	private boolean applyForce = false;

	private static double[] xyz = {3.8966, -2.0614, 4.0300};
	private static double[] hpr = {153.5, -16.5, 0};

	@Override
	public void start() {
		world = OdeHelper.createWorld();
		world.setGravity(0,0,-9.8);

		world.setDamping(1e-4, 1e-5);
		//	    dWorldSetERP(world, 1);

		space = OdeHelper.createSimpleSpace();

		body1 = OdeHelper.createBody(world);
		body2 = OdeHelper.createBody(world);

		body1.setPosition(0, 0, 3);
		body2.setPosition(0, 0, 1);


		DGeom g;
		DMass mass = OdeHelper.createMass();

		g = OdeHelper.createBox(space, 0.2, 0.2, 1);
		g.setBody(body1);
		mass.setBox(1, 0.2, 0.2, 1);
		body1.setMass(mass);

		g = OdeHelper.createBox(space, 0.2, 0.2, 1);
		g.setBody(body2);
		mass.setBox(1, 0.2, 0.2, 1);
		body2.setMass(mass);

		if (true) {
			joint1 = OdeHelper.createDHingeJoint(world);
			joint1.attach(body1, null);
			joint1.setAxis(0, 1, 0);
			joint1.setAnchor1(0, 0, 3.5);
			joint1.setAnchor2(0, 0, 4.5);
		}

		if (true) {
			joint2 = OdeHelper.createDHingeJoint(world);
			joint2.attach(body1, body2);
			joint2.setAxis(1, 0, 0);
			joint2.setAnchor1(0, 0, 2.5);
			joint2.setAnchor2(0, 0, 1.5);
			//	} else {
				//	    joint2 = dJointCreateDBall(world, 0);
			//	    dJointAttach(joint2, body1, body2);
			//	    dJointSetDBallAnchor1(joint2, 0, 0, 2.5);
			//	    dJointSetDBallAnchor2(joint2, 0, 0, 1.5);
		}

		//body1.addForce(20, 0, 0);


		// initial camera position
		dsSetViewpoint (xyz,hpr);
	}

	@Override
	public void stop() {
		space.destroy();
		world.destroy();
	}


	private void drawGeom(DGeom g) {
		DVector3C pos = g.getPosition();
		DMatrix3C rot = g.getRotation();

		if (g instanceof DBox) {
			if (applyForce) {
				dsSetColor(1., .5, 0.);
			} else {
				dsSetColor(1, 1, 0);
			}
			dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
			DVector3C lengths = ((DBox)g).getLengths();
			dsDrawBox(pos, rot, lengths);
		}
	}


	private static double t = 0;

	@Override
	public void step(boolean pause) {
		if (!pause) {

			final double step = 0.005;
			final int nsteps = 2;

			for (int i=0; i<nsteps; ++i) {

				applyForce = ( t % 3.) > 2.;

				if (applyForce) {
					double f = 0.3 * Math.sin(t*1.2);
					body1.addForceAtRelPos(f, 0, 0, 
							0, 0, -0.5); // at the lower end

					double g = 0.3 * Math.sin(t*0.7);
					body2.addForceAtRelPos(0, g, 0, 
							0, 0, -0.5); // at the lower end
				}

				t += step;
				if (t > 20.) {
					t = 0.;
				}

				world.quickStep(step);
			}
		}

		// now we draw everything
		int ngeoms = space.getNumGeoms();
		for (int i=0; i<ngeoms; ++i) {
			DGeom g = space.getGeom(i);
			drawGeom(g);
		}

		if (true) {
			DVector3 a11 = new DVector3(), a12 = new DVector3();
			joint1.getAnchor1(a11);
			joint1.getAnchor2(a12);
			dsSetColor(1, 0, 0);
			dsDrawLine(a11, a12);
		}

		if (true) {
			DVector3 a21 = new DVector3(), a22 = new DVector3();
			joint2.getAnchor1(a21);
			joint2.getAnchor2(a22);
			dsSetColor(0, 1, 0);
			dsDrawLine(a21, a22);
		}
	}

	public static void main(String[] args) {
		new DemoDHinge().demo(args);
	}

	private void demo(String[] args) {
		// create world
		OdeHelper.initODE();

		// run demo
		dsSimulationLoop (args, 800, 600, this);

		OdeHelper.closeODE();
	}

	@Override
	public void command(char cmd) {
		// Auto-generated method stub
	}

}