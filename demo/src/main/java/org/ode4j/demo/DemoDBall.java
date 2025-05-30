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

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DDoubleBallJoint;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;

import static org.ode4j.drawstuff.DrawStuff.*;
import static org.ode4j.drawstuff.DrawStuff.DS_SIMULATION_DEFAULT_HEIGHT;

public class DemoDBall extends dsFunctions {

	private DWorld world;
	private DSpace space;
	private DBody body1;
	private DBody body2;
	private DDoubleBallJoint joint1, joint2;

	private static final double[] xyz = {5.8966, -4.0614, 4.0300};
	private static final double[] hpr = {153.5, 1, 0};

	@Override
	public void start()
	{
		world = OdeHelper.createWorld();
		world.setGravity(0,0,-9.8);

		world.setDamping(1e-4, 1e-5);
		//    dWorldSetERP(world, 1);

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

		joint1 = OdeHelper.createDBallJoint(world);
		joint1.attach(body1, null);
		joint1.setAnchor1(0, 0, 3.5);
		joint1.setAnchor2(0, 0, 4.5);

		joint2 = OdeHelper.createDBallJoint(world);
		joint2.attach(body1, body2);
		joint2.setAnchor1(0, 0, 2.5);
		joint2.setAnchor2(0, 0, 1.5);


		// initial camera position
		dsSetViewpoint (xyz,hpr);
	}

	@Override
	public void stop()
	{
		space.destroy();

		world.destroy();
	}


	private void drawGeom(DGeom g)
	{
		//int gclass = dGeomGetClass(g);
		DVector3C pos = g.getPosition();
		DMatrix3C rot = g.getRotation();

		if (g instanceof DSphere) {
			dsSetColorAlpha(0, 0.75, 0.5, 1);
			dsSetTexture (DS_TEXTURE_NUMBER.DS_CHECKERED);
			dsDrawSphere(pos, rot, ((DSphere)g).getRadius());
		} else if (g instanceof DBox) {
			dsSetColorAlpha(1, 1, 0, 1);
			dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
			DVector3C lengths = ((DBox)g).getLengths();
			dsDrawBox(pos, rot, lengths);
		}
	}

	private static double t = 0;


	@Override
	public void step(boolean pause)
	{
		if (!pause) {


			final double step = 0.005;
			final int nsteps = 4;

			for (int i=0; i<nsteps; ++i) {

				double f = Math.sin(t*1.2)*0.8;
				body1.addForceAtRelPos(
						f, 0, 0, 
						0, 0, -0.5); // at the lower end

						double g = Math.sin(t*0.7)*0.8;
						body2.addForceAtRelPos(
								0, g, 0, 
								0, 0, -0.5); // at the lower end
								t += step;

								world.quickStep(step);
			}
		}

		// now we draw everything
		for (DGeom g : space.getGeoms()) {

			drawGeom(g);
		}

		DVector3 a11 = new DVector3(), a12 = new DVector3();
		joint1.getAnchor1(a11);
		joint1.getAnchor2(a12);
		dsSetColor(1, 0, 0);
		dsDrawLine(a11, a12);

		//printf("Error 1: %f\n", fabs(dJointGetDBallDistance(joint1) - dCalcPointsDistance3(a11, a12)));

		DVector3 a21 = new DVector3(), a22 = new DVector3();
		joint2.getAnchor1(a21);
		joint2.getAnchor2(a22);
		dsSetColor(0, 1, 0);
		dsDrawLine(a21, a22);

		//printf("Error 2: %f\n", fabs(dJointGetDBallDistance(joint2) - dCalcPointsDistance3(a21, a22)));
	}


	public static void main(String[] args) {
		new DemoDBall().demo(args);
	}

	private void demo(String[] args) {
		// create world
		OdeHelper.initODE();

		// run demo
		dsSimulationLoop(args, DS_SIMULATION_DEFAULT_WIDTH, DS_SIMULATION_DEFAULT_HEIGHT, this);

		OdeHelper.closeODE();
	}

	@Override
	public void command(char cmd) {
		// Auto-generated method stub
	}

}