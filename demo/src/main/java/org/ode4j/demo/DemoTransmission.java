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

import static org.ode4j.drawstuff.DrawStuff.dsDrawCylinder;
import static org.ode4j.drawstuff.DrawStuff.dsDrawLine;
import static org.ode4j.drawstuff.DrawStuff.dsDrawSphere;
import static org.ode4j.drawstuff.DrawStuff.dsSetColorAlpha;
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
import org.ode4j.ode.DCylinder;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DHingeJoint;
import org.ode4j.ode.DJoint.DJointFeedback;
import org.ode4j.ode.DJoint.PARAM_N;
import org.ode4j.ode.DJoint.TRANSMISSION;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DTransmissionJoint;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.OdeMath;


/**
 *
 * Gyroscopic demo.
 */
public class DemoTransmission extends dsFunctions {

	private double theta = OdeMath.M_PI / 4;
	private double ratio = 1, speed = 5, rho_1 = 1, rho_2 = 1, backlash = 0.1;
	private TRANSMISSION mode = TRANSMISSION.dTransmissionParallelAxes;

	private DWorld world;
	private DSpace space;
	private DBody body1, body2;
	private DGeom geom1, geom2;
	private DHingeJoint hinge1, hinge2;
	private DTransmissionJoint transmission;
	private DJointFeedback feedback;

	private void setup() {
		DMatrix3 R = new DMatrix3();

		switch (mode) {
		case dTransmissionParallelAxes:
			// Parallel axes.

			body1.setPosition(1, 0, 1);
			body2.setPosition(-1, 0, 1);

			R.setIdentity();
			body1.setRotation(R);
			body2.setRotation(R);

			hinge2.setAnchor(-1, 0, 1);
			hinge2.setAxis(0, 0, 1);

			hinge1.setAnchor(1, 0, 1);
			hinge1.setAxis(0, 0, 1);

			transmission.setMode(TRANSMISSION.dTransmissionParallelAxes);
			transmission.setRatio(ratio);
			transmission.setAnchor1(1, 0, 1);
			transmission.setAnchor2(-1, 0, 1);
			transmission.setAxis(0, 0, 1);

			break;
		case dTransmissionIntersectingAxes:
			// Intersecting axes.

			body1.setPosition(1, 0, 1);
			body2.setPosition(-1, 0, 2);

			R.setIdentity();
			body1.setRotation(R);

			OdeMath.dRFromZAxis(R, new DVector3(Math.cos(theta), 0, Math.sin(theta)));
			body2.setRotation(R);

			hinge2.setAnchor(-1, 0, 2);
			hinge2.setAxis(Math.cos(theta), 0, Math.sin(theta));

			hinge1.setAnchor(1, 0, 1);
			hinge1.setAxis(0, 0, 1);

			transmission.setMode(TRANSMISSION.dTransmissionIntersectingAxes);
			transmission.setAnchor1(1, 0, 1);
			transmission.setAnchor2(-1, 0, 2);
			transmission.setAxis1(0, 0, -1);
			transmission.setAxis2(Math.cos(theta), 0, Math.sin(theta));

			break;
		case dTransmissionChainDrive:
			// Chain.

			body1.setPosition(2, 0, 1);
			body2.setPosition(-2, 0, 1);

			R.setIdentity();
			body1.setRotation(R);
			body2.setRotation(R);

			hinge2.setAnchor(-2, 0, 1);
			hinge2.setAxis(0, 0, 1);

			hinge1.setAnchor(2, 0, 1);
			hinge1.setAxis(0, 0, 1);

			transmission.setMode(TRANSMISSION.dTransmissionChainDrive);
			transmission.setAnchor1(2, 0, 1);
			transmission.setAnchor2(-2, 0, 1);
			transmission.setRadius1(rho_1);
			transmission.setRadius2(rho_2);
			transmission.setAxis(0, 0, 1);

			break;
		}

		transmission.setBacklash(backlash);

		hinge2.setParam(PARAM_N.dParamVel1, speed);
		hinge2.setParam(PARAM_N.dParamFMax1, 50);

		hinge1.setParam(PARAM_N.dParamVel1, 0);
		hinge1.setParam(PARAM_N.dParamFMax1, 2);

		body1.setLinearVel(0, 0, 0);
		body2.setLinearVel(0, 0, 0);
		body1.setAngularVel(0, 0, 0);
		body2.setAngularVel(0, 0, 0);
	}

	private static double[] xyz = {1.15,-2.78,4.1};
	private static double[] hpr = {105,-45.5,0};

	@Override
	public void start() {
		DMass mass = OdeHelper.createMass();

		world = OdeHelper.createWorld();
		world.setGravity(0,0,-9.8);

		world.setERP(0.2);

		space = OdeHelper.createSimpleSpace();

		body1 = OdeHelper.createBody(world);
		body2 = OdeHelper.createBody(world);

		body1.setFiniteRotationMode(true);
		body2.setFiniteRotationMode(true);

		geom1 = OdeHelper.createCylinder(space, 0.2, 0.5);
		geom1.setBody(body1);
		mass.setCylinder(100, 3, 0.2, 0.5);
		body1.setMass(mass);

		geom2 = OdeHelper.createCylinder(space, 0.2, 0.5);
		geom2.setBody(body2);
		mass.setCylinder(100, 3, 0.2, 0.5);
		body2.setMass(mass);

		hinge1 = OdeHelper.createHingeJoint(world);
		hinge1.attach(body1, null);

		hinge2 = OdeHelper.createHingeJoint(world);
		hinge2.attach(body2, null);

		transmission = OdeHelper.createTransmissionJoint(world);
		transmission.attach(body1, body2);
		transmission.setFeedback(feedback);

		setup();

		// initial camera position
		dsSetViewpoint (xyz,hpr);

		System.out.println("The green wheel is driving the red one. To control it use the following:");
		System.out.println("   '[' : decrease wheel ratio");
		System.out.println("   ']' : increase wheel ratio");
		System.out.println("   ',' : decrease driving wheel speed");
		System.out.println("   '.' : increase driving wheel speed");
		System.out.println("   '-' : decrease backlash");
		System.out.println("   '=' : increase backlash");
		System.out.println("   '1' : switch to parallel axes gears mode");
		System.out.println("   '2' : switch to intersecting axes gears mode");
		System.out.println("   '3' : switch to chain (or belt) mode");
	}

	@Override
	public void stop()	{
		space.destroy();
		world.destroy();
	}

	private void drawGeom(DGeom g) {
		DVector3C pos = g.getPosition();
		DMatrix3C rot = g.getRotation();

		if (g instanceof DCylinder) {
			DCylinder cyl = (DCylinder) g;

			if (g == geom1) {
				dsSetColorAlpha(1, 0, 0, 1);
			} else {
				dsSetColorAlpha(0, 1, 0, 1);
			}

			dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);
			dsDrawCylinder(pos, rot, cyl.getLength(), cyl.getRadius());
		} else {
			System.exit(0);
		}
	}

	@Override
	public void step(boolean pause)	{
		if (!pause) {

			final double step = 0.003;
			final int nsteps = 4;

			for (int i=0; i<nsteps; ++i) {
				world.quickStep(step);
			}
		}

//		#if 0
//		{
//			const dReal *omega_1, *omega_2;
//
//			omega_1 = dBodyGetAngularVel(body1);
//			omega_2 = dBodyGetAngularVel(body2);
//
//			printf ("T1: %f, %f, %f\n",
//					feedback.t1[0], feedback.t1[1], feedback.t1[2]);
//
//			printf ("T2: %f, %f, %f\n",
//					feedback.t2[0], feedback.t2[1], feedback.t2[2]);
//
//			printf ("F1: %f, %f, %f\n",
//					feedback.f1[0], feedback.f1[1], feedback.f1[2]);
//
//			printf ("F2: %f, %f, %f\n",
//					feedback.f2[0], feedback.f2[1], feedback.f2[2]);
//		}
//		#endif

		// now we draw everything
		int ngeoms = space.getNumGeoms();
		for (int i=0; i<ngeoms; ++i) {
			DGeom g = space.getGeom(i);

			drawGeom(g);
		}

		DMatrix3C R_1 = geom1.getRotation();
		DMatrix3C R_2 = geom2.getRotation();
		DVector3 c_1 = new DVector3(), c_2 = new DVector3();
		DVector3 a_1 = new DVector3(), a_2 = new DVector3();

		transmission.getContactPoint1(c_1);
		transmission.getContactPoint2(c_2);
		transmission.getAnchor1(a_1);
		transmission.getAnchor2(a_2);

		dsSetColorAlpha(1, 0, 0, 0.5);
		dsDrawCylinder(a_1, R_1, 0.05, c_1.distance(a_1));
		dsSetColorAlpha(0, 1, 0, 0.5);
		dsDrawCylinder(a_2, R_2, 0.05, c_2.distance(a_2));

		dsSetColorAlpha(1, 0, 0, 0.5);
		dsDrawSphere (c_1, R_1, 0.05);
		dsDrawSphere (c_2, R_1, 0.05);

		dsSetColorAlpha(1, 1, 0, 0.5);
		if (mode == TRANSMISSION.dTransmissionChainDrive) {
			dsDrawLine(c_1, c_2);
		}
	}

	@Override
	public void command (char cmd) {
		if (cmd == '[') {
			switch(mode) {
			case dTransmissionParallelAxes:
				if (ratio > 0.125) {
					ratio *= 0.5;

					System.out.println("Gear ratio set to " + ratio);
				}
				break;
			case dTransmissionIntersectingAxes:
				if (theta > 0.1) {
					theta -= 0.1;

					System.out.println("Gear angle set to " + 
							(theta / OdeMath.M_PI * 180) + " deg");
				}
				break;
			case dTransmissionChainDrive:
				if (rho_2 > 0.125) {
					rho_2 /= 2;

					System.out.println("Sprocket ratio set to " + rho_2 / rho_1);
				}
				break;
			}

			setup();
		} else if (cmd == ']') {
			switch(mode) {
			case dTransmissionParallelAxes:
				if (ratio < 8) {
					ratio *= 2;

					System.out.println("Gear ratio set to " + ratio);
				}
				break;
			case dTransmissionIntersectingAxes:
				if (theta < 0.9) {
					theta += 0.1;

					System.out.println("Gear angle set to %.3f .\n" +
							(theta / OdeMath.M_PI * 180) + " deg");
				}
				break;
			case dTransmissionChainDrive:
				if (rho_2 < 2) {
					rho_2 *= 2;

					System.out.println("Sprocket ratio set to " + rho_2 / rho_1);
				}
				break;
			}

			setup();
		} else if (cmd == '.') {
			speed += 5;

			System.out.println("Driving wheel speed set to " + speed + " rad/s");

			hinge2.setParam(PARAM_N.dParamVel1, speed);
		} else if (cmd == ',') {
			speed -= 5;

			System.out.println("Driving wheel speed set to " + speed + " rad/s");

			hinge2.setParam(PARAM_N.dParamVel1, speed);
		} else if (cmd == '/') {
			if (hinge2.getParam(PARAM_N.dParamFMax1) > 0) {
				hinge2.setParam(PARAM_N.dParamFMax1, 0);
			} else {
				hinge2.setParam(PARAM_N.dParamFMax1, 50);
			}

		} else if (cmd == '-') {
			backlash -= 0.1;

			System.out.println("Backlash set to " + backlash + " m");

			transmission.setBacklash(backlash);
		} else if (cmd == '=') {
			backlash += 0.1;

			System.out.println("Backlash set to " + backlash + " m");

			transmission.setBacklash(backlash);
		} else if (cmd == '1') {
			mode = TRANSMISSION.dTransmissionParallelAxes;
			setup();
		} else if (cmd == '2') {
			mode = TRANSMISSION.dTransmissionIntersectingAxes;
			setup();
		} else if (cmd == '3') {
			mode = TRANSMISSION.dTransmissionChainDrive;
			setup();
		}
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		new DemoTransmission().demo(args);
	}

	private void demo(String[] args) {
		OdeHelper.initODE2(0);

		// run simulation
		dsSimulationLoop (args,800,600,this);

		OdeHelper.closeODE();
	}

}
