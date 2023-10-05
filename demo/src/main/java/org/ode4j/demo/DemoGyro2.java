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
package org.ode4j.demo;

import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DMassC;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.OdeMath;


/**
 *
 * Gyroscopic demo.
 */
public class DemoGyro2 extends dsFunctions {

	// dynamics and collision objects
	private DWorld world = null;

	private final double dt = 1/60.0; // 60 fps
	// Water density if units are meters and kg
	private final double density = 1000;  

	// A long skinny thing
	private DVector3 sides = new DVector3(2,.5,.25); 
	// Initial angular velocity
	private DVector3 omega = new DVector3(5,1,2);
	private DVector3 torque = new DVector3(0,10,0);
	private DBody noGyroBody;
	private DBody expGyroBody;
	private DBody impGyroBody;


	private static double[] xyz = {0,-4.0f,3.0f};
	private static double[] hpr = {90.0000,-15.0000,0.0000};

	// start simulation - set viewpoint

	@Override
	public void start()	{
		//dAllocateODEDataForThread(dAllocateMaskAll);

		dsSetViewpoint (xyz,hpr);
		System.out.println("Press:");
		System.out.println("\t'a' to apply a torque");
		System.out.println("\t'r' to reset simulation.");
	}

	/**
	 * Delete the bodies, etc.
	 */
	private void clear() {
		if (world != null) {
			world.destroy();
		}
		world = null;
	}



	/**
	 * Cleanup if necessary and rebuild the world.
	 */
	private void reset() {
		clear();

		// create world
		world = OdeHelper.createWorld();

		// Calculate mass for a box;
		DMass boxMass = OdeHelper.createMass();
		boxMass.setBox(density, sides);

		noGyroBody  = OdeHelper.createBody(world);// Conservation of ang-velocity
		expGyroBody = OdeHelper.createBody(world);// Explicit conservation of ang-momentum
		impGyroBody = OdeHelper.createBody(world);// Implicit conservation of ang-momentum

		noGyroBody.setMass( boxMass );
		expGyroBody.setMass( boxMass );
		impGyroBody.setMass( boxMass );

		// Try to avoid collisions.
		double sep = sides.length();
		noGyroBody.setPosition( -sep, 0, sep);
		expGyroBody.setPosition(   0, 0, sep);
		impGyroBody.setPosition( sep, 0, sep);

		// Set the initial angular velocity
		noGyroBody.setAngularVel(omega);
		expGyroBody.setAngularVel(omega);
		impGyroBody.setAngularVel(omega);

		noGyroBody.setGyroscopicMode(false);
		// We compute this ourselves using the math
		// that was in the old stepper.
		expGyroBody.setGyroscopicMode(false); 
		// Keep things from crashing by limiting
		// the angular speed of the explicit body.
		// Note that this isn't necessary for
		// the other two bodies.
		expGyroBody.setMaxAngularSpeed( 40 );
	}

	@Override
	public void command (char cmd) {
		switch (cmd) {
		case 'a': case 'A':
			noGyroBody.addTorque( torque );
			expGyroBody.addTorque( torque );
			impGyroBody.addTorque( torque );
			break;
		case 'r': case 'R':
			reset();
			break;
		}
	}

	/**
	 * This is the explicit computation of
	 * gyroscopic forces.
	 */
	private static void expStep(DBody body) {
		// Explicit computation
		DMatrix3 I = new DMatrix3(), tmp = new DMatrix3();
		DMassC m = body.getMass();
		DMatrix3C R = body.getRotation();
		// compute inertia tensor in global frame
		OdeMath.dMultiply2_333 (tmp,m.getI(),R);
		OdeMath.dMultiply0_333 (I,R,tmp);
		// compute explicit rotational force
		// we treat 'tmp'like a vector, but that's okay.
		DVector3C w = body.getAngularVel();
		OdeMath.dMultiply0_331 (tmp,I,w);
		DVector3 tau = new DVector3();
		DVector3 tmp2 = new DVector3(tmp.get00(), tmp.get01(), tmp.get02());
		OdeMath.dCalcVectorCross3(tau,tmp2,w);
		body.addTorque(tau);
	}


	// simulation loop
	@Override
	public void step(boolean pause) {
		if (!pause) {
			expStep(expGyroBody);
			world.step(dt); 
		}

		dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
		dsSetColor(1,0,0);
		dsDrawBox(noGyroBody.getPosition(), noGyroBody.getRotation(), sides);
		dsSetColor(1,1,0);
		dsDrawBox(expGyroBody.getPosition(), expGyroBody.getRotation(), sides);
		dsSetColor(0,1,0);
		dsDrawBox(impGyroBody.getPosition(), impGyroBody.getRotation(), sides);
	}



	/**
	 * @param args args
	 */
	public static void main(String[] args) {
		new DemoGyro2().demo(args);
	}

	private void demo(String[] args) {
		OdeHelper.initODE2(0);
		reset();

		// run simulation
		dsSimulationLoop (args,640,480,this);

		clear();
		OdeHelper.closeODE();
	}


	@Override
	public void stop() {
		// Nothing
	}

}
