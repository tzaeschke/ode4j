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
package org.ode4j.demo.ragdoll;

import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCapsule;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.OdeConstants.dContactBounce;
import static org.ode4j.ode.OdeConstants.dContactSoftCFM;
import static org.ode4j.ode.OdeHelper.areConnectedExcluding;

import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;
import org.ode4j.ode.ragdoll.DRagdoll;

public class DemoRagdoll extends dsFunctions {

	private static final int  MAX_CONTACTS = 64;		// maximum number of contact points per body
	private DWorld world;
	private DSpace space;
	private DRagdoll ragdoll;
	private static final double[] xyz = {4.3966, -2.0614, 3.4300};
	private static final double[] hpr = {153.5, -14.5, 0};
	private DJointGroup contactgroup;
	private boolean show_contacts = false;	// show contact points?

	@Override
	public void start()
	{
		//dAllocateODEDataForThread(dAllocateMaskAll);
		dsSetViewpoint (xyz,hpr);
		System.out.println ("Press space to apply some force to the ragdoll");

		world = OdeHelper.createWorld();
		world.setGravity(0,0,-9.8);
		world.setDamping(1e-4, 1e-5);
		//    dWorldSetERP(world, 1);
		space = OdeHelper.createSimpleSpace();
		contactgroup = OdeHelper.createJointGroup ();
		OdeHelper.createPlane( space, 0, 0, 1, 0 );

		ragdoll = OdeHelper.createRagdoll(world, space, new DxDefaultHumanRagdollConfig());
		ragdoll.setAngularDamping(0.1);
		DQuaternion q = new DQuaternion(1, 0, 0, 0);
		DRotation.dQFromAxisAndAngle(q, new DVector3(1, 0, 0), -0.5 * Math.PI);
		for (DRagdoll.DRagdollBody bone : ragdoll.getBoneIter()) {
			DGeom g = OdeHelper.createCapsule(space, bone.getRadius(), bone.getLength());
			DBody body = bone.getBody();
			DQuaternion qq = new DQuaternion();
			OdeMath.dQMultiply1(qq, q, body.getQuaternion());
			body.setQuaternion(qq);
			DMatrix3 R = new DMatrix3();
			OdeMath.dRfromQ(R, q);
			DVector3 v = new DVector3();
			OdeMath.dMultiply0_133(v, body.getPosition(), R);
			body.setPosition(v.get0(), v.get1(), v.get2());
			g.setBody(body);
		}
		// initial camera position
		dsSetViewpoint (xyz,hpr);
	}

	@Override
	public void stop()
	{
		ragdoll.destroy();
		contactgroup.destroy();
		space.destroy();
		world.destroy();
	}


	private void drawGeom(DGeom g)
	{
		//int gclass = dGeomGetClass(g);
		if (g instanceof DCapsule) {
			DVector3C pos = g.getPosition();
			DMatrix3C rot = g.getRotation();
			DCapsule cap = (DCapsule) g; 
			dsDrawCapsule (pos, rot, cap.getLength(), cap.getRadius());
		}
	}

	private void nearCallback (Object data, DGeom o1, DGeom o2) {
		int i;
		// if (o1->body && o2->body) return;

		// exit without doing anything if the two bodies are connected by a joint
		DBody b1 = o1.getBody();
		DBody b2 = o2.getBody();
		if (b1!=null && b2!=null && areConnectedExcluding (b1,b2,DContactJoint.class)) {
			return;
		}

		DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);   // up to MAX_CONTACTS contacts per box-box
		for (i=0; i<MAX_CONTACTS; i++) {
			DContact contact = contacts.get(i);
			contact.surface.mode = dContactBounce | dContactSoftCFM;
			contact.surface.mu = 100;
			contact.surface.mu2 = 0;
			contact.surface.bounce = 0.01;
			contact.surface.bounce_vel = 0.01;
			contact.surface.soft_cfm = 0.0001;
		}
		int numc = OdeHelper.collide (o1,o2,MAX_CONTACTS,contacts.getGeomBuffer());
		if (numc!=0) {
			DMatrix3 RI = new DMatrix3();
			RI.setIdentity();
			final DVector3 ss = new DVector3(0.02,0.02,0.02);
			for (i=0; i<numc; i++) {
				DJoint c = OdeHelper.createContactJoint (world,contactgroup,contacts.get(i));
				c.attach (b1,b2);
				if (show_contacts) {
					dsSetColor(0, 0, 1);
					dsDrawBox (contacts.get(i).geom.pos,RI,ss);
				}
			}
		}
	}

	@Override
	public void step(boolean pause)
	{
		space.collide (null, this::nearCallback);

		if (!pause) {


			final double step = 0.005;
			final int nsteps = 4;

			for (int i=0; i<nsteps; ++i) {
				world.quickStep(step);
			}
		}
		contactgroup.empty();

		// now we draw everything
		for (DGeom g : space.getGeoms()) {

			drawGeom(g);
		}

	}


	public static void main(String[] args) {
		new DemoRagdoll().demo(args);
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
		cmd = Character.toLowerCase(cmd);
		if (cmd == ' ') {
			ragdoll.getBone(DxDefaultHumanRagdollConfig.PELVIS).getBody().setLinearVel(0, 0, 80);
		}
	}

}