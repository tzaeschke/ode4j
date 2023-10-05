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
package org.ode4j.democpp;

import static org.ode4j.cpp.internal.ApiCppBody.dBodyAddTorque;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyCreate;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetAngularVel;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetQuaternion;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetRotation;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetMass;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetQuaternion;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateBall;
import static org.ode4j.cpp.internal.ApiCppMass.dMassAdjust;
import static org.ode4j.cpp.internal.ApiCppMass.dMassCreate;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetBox;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetZero;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldStep;
import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.DMisc.dMaxDifference;
import static org.ode4j.ode.DMisc.dRandReal;
import static org.ode4j.ode.DMisc.dRandSetSeed;
import static org.ode4j.ode.DRotation.dRfromQ;
import static org.ode4j.ode.OdeMath.dNormalize4;
import static org.ode4j.ode.internal.cpp4j.Cstdio.printf;

import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.math.DVectorN;
import org.ode4j.ode.DBallJoint;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.internal.Matrix;


/**
 * test that the rotational physics is correct.
 * 
 * an "anchor body" has a number of other randomly positioned bodies
 * ("particles") attached to it by ball-and-socket joints, giving it some
 * random effective inertia tensor. the effective inertia matrix is calculated,
 * and then this inertia is assigned to another "test" body. a random torque is
 * applied to both bodies and the difference in angular velocity and orientation
 * is observed after a number of iterations.
 * 
 * typical errors for each test cycle are about 1e-5 ... 1e-4.
 * 
 */
class DemoI extends dsFunctions {

	// some constants

	//#define NUM 10			// number of particles
	//#define SIDE 0.1		// visual size of the particles
	private static final int NUM = 10;			// number of particles
	private static final float SIDE = 0.1f;		// visual size of the particles


	// dynamics objects an globals

	private static DWorld world=null;
	private static DBody anchor_body,particle[]=new DBody[NUM],test_body;
	private static DBallJoint particle_joint[]=new DBallJoint[NUM];
	private static float torque[]=new float[3];
	private static int iteration;


	private static float[] xyz = {1.5572f,-1.8886f,1.5700f};
	private static float[] hpr = {118.5000f,-17.0000f,0.0000f};

	// start simulation - set viewpoint
	@Override
	public void start()
	{
		//dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);

		dsSetViewpoint (xyz,hpr);
	}


	// compute the mass parameters of a particle set. q = particle positions,
	// pm = particle masses

	//#define _I(i,j) I[(i)*4+(j)]
//	private final void _I(int i, int j) { 
//		I[(i)*4+(j)];
//	}

	//void computeMassParams (dMass *m, dReal q[NUM][3], dReal pm[NUM])
	private static void computeMassParams (DMass m, DVector3[] q, DVectorN pm)
	{
		int i,j;
		double pmi, q0, q1, q2;
		dMassSetZero (m);
		DVector3 C = m.getC().copy();
		DMatrix3 I = m.getI().copy();
		for (i=0; i<NUM; i++) {
			pmi = pm.get(i);
			m.setMass(m.getMass() + pmi);// += pmi;
			for (j=0; j<3; j++) C.add(j, pmi*q[i].get(j));
			q0 = q[i].get0();
			q1 = q[i].get1();
			q2 = q[i].get2();
			I.add(0,0, pmi*(q1*q1 + q2*q2));
			I.add(1,1, pmi*(q0*q0 + q2*q2));
			I.add(2,2, pmi*(q0*q0 + q1*q1));
			I.sub(0,1, pmi*(q0*q1));
			I.sub(0,2, pmi*(q0*q2));
			I.sub(1,2, pmi*(q1*q2));
		}
		//for (j=0; j<3; j++) m.c.v[j] /= m.mass;
		C.scale(1./m.getMass());
		I.set(1,0, I.get(0,1));
		I.set(2,0, I.get(0,2));
		I.set(2,1, I.get(1,2));
		m.setC(C);
		m.setI(I);
	}

	private static void reset_test()
	{
		int i;
		DMass m = dMassCreate(),anchor_m=dMassCreate();
		//float q[NUM][3], pm[NUM];	// particle positions and masses
		DVector3[] q = new DVector3[NUM];
		DVectorN pm = new DVectorN(NUM);
		float[] pos1 = {1,0,1};	// point of reference (POR)
		float[] pos2 = {-1,0,1};	// point of reference (POR)

		// make random particle positions (relative to POR) and masses
		for (i=0; i<NUM; i++) {
			pm.set(i, dRandReal()+0.1);
			q[i] = new DVector3();
			q[i].set(0, dRandReal()-0.5);
			q[i].set(1, dRandReal()-0.5);
			q[i].set(2, dRandReal()-0.5);
		}

		// adjust particle positions so centor of mass = POR
		computeMassParams (m,q,pm);
		for (i=0; i<NUM; i++) {
			q[i].add(0, -m.getC().get(0));
			q[i].add(1, -m.getC().get(1));
			q[i].add(2, -m.getC().get(2));
		}

		if (world!=null) dWorldDestroy (world);
		world = dWorldCreate();

		anchor_body = dBodyCreate (world);
		dBodySetPosition (anchor_body,pos1[0],pos1[1],pos1[2]);
		dMassSetBox (anchor_m,1,SIDE,SIDE,SIDE);
		dMassAdjust (anchor_m,0.1);
		dBodySetMass (anchor_body,anchor_m);

		for (i=0; i<NUM; i++) {
			particle[i] = dBodyCreate (world);
			dBodySetPosition (particle[i],
					pos1[0]+ q[i].get0(),
					pos1[1]+ q[i].get1(), 
					pos1[2]+ q[i].get2());
			dMassSetBox (m,1,SIDE,SIDE,SIDE);
			dMassAdjust (m,pm.get(i));
			dBodySetMass (particle[i],m);
		}

		for (i=0; i < NUM; i++) {
			particle_joint[i] = dJointCreateBall (world,null);
			dJointAttach (particle_joint[i],anchor_body,particle[i]);
			final DVector3C p = dBodyGetPosition (particle[i]);
			//dJointSetBallAnchor (particle_joint[i],p.v[0],p.v[1],p.v[2]);
			particle_joint[i].setAnchor(p);
		}

		// make test_body with the same mass and inertia of the anchor_body plus
		// all the particles
		test_body = dBodyCreate (world);
		dBodySetPosition (test_body,pos2[0],pos2[1],pos2[2]);
		computeMassParams (m,q,pm);
		m.setMass( m.getMass() + anchor_m.getMass()); //+= anchor_m._mass;
		//for (i=0; i<12; i++) m.I.v[i] = m.I.v[i] + anchor_m.I.v[i];
		m.setI( m.getI().copy().add(anchor_m.getI()) );
		dBodySetMass (test_body,m);

		// rotate the test and anchor bodies by a random amount
		DQuaternion qrot = new DQuaternion();
		for (i=0; i<4; i++) qrot.set(i, dRandReal()-0.5);
		dNormalize4 (qrot);
		dBodySetQuaternion (anchor_body,qrot);
		dBodySetQuaternion (test_body,qrot);
		DMatrix3 R = new DMatrix3();
		dRfromQ (R,qrot);
		for (i=0; i<NUM; i++) {
			DVector3 v = new DVector3();
//			dMultiply0 (v,R,q[i][0],3,3,1);
			Matrix.dMultiply0 (v,R,q[i]);
			dBodySetPosition (particle[i],pos1[0]+v.get0(),pos1[1]+v.get1(),pos1[2]+v.get2());
		}

		// set random torque
		for (i=0; i<3; i++) torque[i] = (float) ((dRandReal()-0.5) * 0.1);


		iteration=0;
	}


	// simulation loop

	private void simLoop (boolean pause)
	{
		if (!pause) {
			dBodyAddTorque (anchor_body,torque[0],torque[1],torque[2]);
			dBodyAddTorque (test_body,torque[0],torque[1],torque[2]);
			dWorldStep (world,0.03);

			iteration++;
			if (iteration >= 100) {
				// measure the difference between the anchor and test bodies
				final DVector3C w1 = dBodyGetAngularVel (anchor_body);
				final DVector3C w2 = dBodyGetAngularVel (test_body);
				final DQuaternionC q1 = dBodyGetQuaternion (anchor_body);
				final DQuaternionC q2 = dBodyGetQuaternion (test_body);
				double maxdiff = dMaxDifference (w1,w2);
				printf ("w-error = %.4e  (%.2f,%.2f,%.2f) and (%.2f,%.2f,%.2f)\n",
						maxdiff,w1.get0(),w1.get1(),w1.get2(),w2.get0(),w2.get1(),w2.get2());
				maxdiff = dMaxDifference (q1,q2,1,4);
				printf ("q-error = %.4e\n",maxdiff);
				reset_test();
			}
		}

		DVector3 sides = new DVector3(SIDE,SIDE,SIDE);
		DVector3 sides2 = new DVector3(6*SIDE,6*SIDE,6*SIDE);
		DVector3 sides3 = new DVector3(3*SIDE,3*SIDE,3*SIDE);
		dsSetColor (1,1,1);
		dsDrawBox (dBodyGetPosition(anchor_body), dBodyGetRotation(anchor_body),
				sides3);
		dsSetColor (1,0,0);
		dsDrawBox (dBodyGetPosition(test_body), dBodyGetRotation(test_body), sides2);
		dsSetColor (1,1,0);
		for (int i=0; i<NUM; i++)
			dsDrawBox (dBodyGetPosition (particle[i]),
					dBodyGetRotation (particle[i]), sides);
	}


	public static void main(String[] args) {
		new DemoI().demo(args);
	}
	
	private void demo(String[] args) {
		// setup pointers to drawstuff callback functions
		//dsFunctions fn = new DemoI();
		//fn.version = DrawStuff.DS_VERSION;
		//  fn.start = &start;
		//  fn.step = &simLoop;
		//  fn.command = 0;
		//  fn.stop = 0;
		//  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
		//fn.setPathToTextures(DrawStuff.DRAWSTUFF_TEXTURE_PATH);

		dInitODE2(0);
		dRandSetSeed (0); // System.currentTimeMillis());
		reset_test();

		// run simulation
		dsSimulationLoop (args,640,480,this);

		dWorldDestroy (world);
		dCloseODE();
	}


	@Override
	public void command(char cmd) {
		// Nothing
	}


	@Override
	public void step(boolean pause) {
		simLoop(pause);
	}


	@Override
	public void stop() {
		// Nothing
	}
}