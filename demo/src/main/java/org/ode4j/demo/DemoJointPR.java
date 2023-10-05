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

import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DContactJoint;
import org.ode4j.ode.DPRJoint;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.DGeom.DNearCallback;

import static org.ode4j.drawstuff.DrawStuff.*;
import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.OdeHelper.*;


/**
 * This file try to demonstrate how the PR joint is working.
 *
 * The axisP is draw in red and the axisR is in green
 */
class DemoJointPR extends dsFunctions {

	// physics parameters
	//#define BOX1_LENGTH 2    // Size along the X axis 
	//#define BOX1_WIDTH 1     // Size along the Y axis
	//#define BOX1_HEIGHT 0.4  // Size along the Z axis (up) since gravity is (0,0,-10)
	//#define BOX2_LENGTH 0.2
	//#define BOX2_WIDTH 0.1
	//#define BOX2_HEIGHT 0.4
	//#define Mass1 10
	//#define Mass2 0.1
	private static int BOX1_LENGTH = 2;    // Size along the X axis 
	private static int BOX1_WIDTH = 1;     // Size along the Y axis
	private static double BOX1_HEIGHT = 0.4;  // Size along the Z axis (up) since gravity is (0,0,-10)
	private static double BOX2_LENGTH = 0.2;
	private static double BOX2_WIDTH = 0.1;
	private static double BOX2_HEIGHT = 0.4;
	private static double Mass1 = 10;
	private static double Mass2 = 0.1;


	//#define PRISMATIC_ONLY 1
	//#define ROTOIDE_ONLY   2
	private static final int PRISMATIC_ONLY = 1;
	private static final int ROTOIDE_ONLY   = 2;
	private static int flag = 0;


	//camera view
	private static float[] xyz = {2.0f,-3.5f,2.0000f};
	private static float[] hpr = {90.000f,-25.5000f,0.0000f};
	//world,space,body & geom
	private static DWorld world;
	private static DSpace space;
	private static DSpace box1_space;
	private static DBody box1_body;
	private static DBody box2_body;
	private static DPRJoint joint;
	private static DJointGroup contactgroup;
	//private static DGeom ground;
	private static DBox box1;
	private static DBox box2;


	//collision detection
	//static void nearCallback (void *data, dGeom o1, dGeom o2)
	private static void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		int i,n;

		DBody b1 = o1.getBody();
		DBody b2 = o2.getBody();
		if (b1!=null && b2!=null &&	areConnectedExcluding (b1,b2,DContactJoint.class)) 
			return;
		final int N = 10;
		//dContact[] contact=new dContact[N];
		DContactBuffer contacts = new DContactBuffer(N);
		n = OdeHelper.collide (o1,o2,N,contacts.getGeomBuffer());//,sizeof(dContact));
		if (n > 0)
		{
			for (i=0; i<n; i++)
			{
				DContact contact = contacts.get(i);
				contact.surface.mode = OdeConstants.dContactSlip1 | 
				OdeConstants.dContactSlip2 |
				OdeConstants.dContactSoftERP | OdeConstants.dContactSoftCFM | 
				OdeConstants.dContactApprox1;
				contact.surface.mu = 0.1;
				contact.surface.slip1 = 0.02;
				contact.surface.slip2 = 0.02;
				contact.surface.soft_erp = 0.1;
				contact.surface.soft_cfm = 0.0001;
				DJoint c = OdeHelper.createContactJoint (world,contactgroup,contact);
				c.attach (contact.geom.g1.getBody(),contact.geom.g2.getBody());
			}
		}
	}


	// start simulation - set viewpoint
	@Override
	public void start()
	{
		dsSetViewpoint (xyz,hpr);
		System.out.println ("Press 'd' to add force along positive x direction.");
		System.out.println ("Press 'a' to add force along negative x direction.");
		System.out.println ("Press 'w' to add force along positive y direction.");
		System.out.println ("Press 's' to add force along negative y direction.");
		System.out.println ("Press 'e' to add torque around positive z direction.");
		System.out.println ("Press 'q' to add torque around negative z direction.");
		System.out.println ("Press 'o' to add force around positive x direction");

		System.out.println ("Press 'v' to give a defined velocity and add a FMax to the rotoide axis");
		System.out.println ("Press 'c' to set the velocity to zero and remove the FMax");

		System.out.println ("Press 'l' to add limits (-0.5 to 0.5rad) on the rotoide axis");
		System.out.println ("Press 'k' to remove the limits on the rotoide axis");

		System.out.println ("Press 'i' to get joint info");
	}

	// function to update camera position at each step.
	private void update()
	{
		// 		const dReal *a =(dBodyGetPosition (box1_body[0]));
		// 		float dx=a[0];
		// 		float dy=a[1];
		// 		float dz=a[2];
		// 		xyz[0]=dx;
		// 		xyz[1]=dy-5;
		// 		xyz[2]=dz+2;
		// 		hpr[1]=-22.5000f;
		// 		dsSetViewpoint (xyz,hpr);
	}


	// called when a key pressed
	@Override
	public void command (char cmd)
	{
		DVector3C pos = box2_body.getPosition();
		switch(Character.toLowerCase(cmd))
		{
		case 'w':
			box2_body.addForce(0,500,0);
			//std::cout<<(dBodyGetPosition(box2_body[0])[1]-dBodyGetPosition(boxody[0])[1])<<'\n';
			System.out.print((pos.get1()-pos.get1()));
			break;
		case 's':
			box2_body.addForce(0,-500,0);
			System.out.print((pos.get1()-pos.get1()));
			break;
		case 'd':
			box2_body.addForce(500,0,0);
			System.out.print((pos.get0()-pos.get0()));
			break;
		case 'a':
			box2_body.addForce(-500,0,0);
			System.out.print((pos.get0()-pos.get0()));
			break;
		case 'e':
			box2_body.addRelTorque(0,0,200);
			break;
		case 'q':
			box2_body.addRelTorque(0,0,-200);
			break;
		case 'o':
			box2_body.addForce(10000,0,0);
			break;
	    case 'v':
	    case 'V':
	        joint.setParamVel2(2);
	        joint.setParamFMax2(500);
	        break;

	    case 'c':
	    case 'C':
	        joint.setParamVel2(0);
	        joint.setParamFMax2(0);
	        break;

	    case 'l':
	    case 'L':
	        joint.setParamLoStop2(-0.5);
	        joint.setParamHiStop2( 0.5);
	        break;

	    case 'k':
	    case 'K':
	        joint.setParamLoStop2( -dInfinity);
	        joint.setParamHiStop2(  dInfinity);
	        break;

	    case 'i':
	    case 'I':
	        DVector3 anchor = new DVector3();
	        joint.getAnchor(anchor);
	        double angle = joint.getAngle();
	        double w = joint.getAngleRate();

	        double l = joint.getPosition();
	        double v = joint.getPositionRate();

	        System.out.println("Anchor: " + anchor);
	        System.out.println("Position: " + l + ", Rate: " + v);
	        System.out.println("Angle: " + angle + ", Rate: " + w);
	        break;
		}
	}


	// simulation loop
	@Override
	public void step (boolean pause)
	{
		if (!pause)
		{
			//draw 2 boxes
			DVector3C ss = new DVector3();
			dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);

			DVector3C posBox2 = box2.getPosition();
			DMatrix3C rotBox2 = box2.getRotation();
			dsSetColor (1,1,0);
			ss = box2.getLengths();
			dsDrawBox (posBox2, rotBox2, ss);

			DVector3C posBox1 = box1.getPosition();
			DMatrix3C rotBox1 = box1.getRotation();
			dsSetColor (1,1,2);
			ss = box1.getLengths();
			dsDrawBox (posBox1, rotBox1, ss);

			DVector3 anchorPos = new DVector3();
			joint.getAnchor(anchorPos);//)dJointGetPRAnchor (joint, anchorPos);

			// Draw the axisP 
			if (ROTOIDE_ONLY != flag )
			{
				dsSetColor (1,0,0);
				DVector3 sizeP = new DVector3(0, 0.1, 0.1);
				for (int i=0; i<3; ++i)
					sizeP.add(0, (anchorPos.get(i) - posBox1.get(i))*(anchorPos.get(i) - posBox1.get(i)) );
				sizeP.set(0, Math.sqrt(sizeP.get(0)));
				DVector3 posAxisP = new DVector3();
				for (int i=0; i<3; ++i)
					posAxisP.set(i, posBox1.get(i) + (anchorPos.get(i) - posBox1.get(i))/2.0 );
				dsDrawBox (posAxisP, rotBox1, sizeP);
			}


			// Draw the axisR 
			if (PRISMATIC_ONLY != flag )
			{
				dsSetColor (0,1,0);
				DVector3 sizeR = new DVector3(0, 0.1, 0.1);
				for (int i=0; i<3; ++i)
					sizeR.add(0, (anchorPos.get(i) - posBox2.get(i))*(anchorPos.get(i) - posBox2.get(i)) );
				sizeR.set(0, Math.sqrt(sizeR.get(0)));
				DVector3 posAxisR = new DVector3();
				for (int i=0; i<3; ++i)
					posAxisR.set(i, posBox2.get(i) + (anchorPos.get(i) - posBox2.get(i))/2.0 );
				dsDrawBox (posAxisR, rotBox2, sizeR);
			}

			//		dSpaceCollide (space,0,&nearCallback);
			OdeHelper.spaceCollide (space,null,new DNearCallback() {
				@Override
				public void call(Object data, DGeom o1, DGeom o2) {
					nearCallback(data, o1, o2);
				}} );
			world.quickStep (0.0001);
			update();
			contactgroup.empty ();
		}
	}

	@Override
	public void dsPrintHelp()
	{
		super.dsPrintHelp();
		System.out.println(" -b | --both :           Display how the complete joint works");
		System.out.println("                         Default behavior");
		System.out.println(" -p | --prismatic-only : Display how the prismatic part works");
		System.out.println("                         The anchor pts is set at the center of body 2");
		System.out.println(" -r | --rotoide-only   : Display how the rotoide part works");
		System.out.println("                         The anchor pts is set at the center of body 1");
		System.out.println("--------------------------------------------------");
		System.out.println("Hit any key to continue:");
		//  getchar();

		System.exit(0);
	}

	public static void main(String[] args) {
		new DemoJointPR().demo(args);
	}
	
	private void demo(String[] args) {
		for (int i=0; i < args.length; ++i)
		{
			if (flag==0 && ("-p".equals(args[i]) || "--prismatic-only".equals(args[i])) ) {
				flag = PRISMATIC_ONLY;
				args[i] = "";
			}

			if (flag==0 && ("-r".equals(args[i]) || "--rotoide-only".equals(args[i])) ) {
				flag = ROTOIDE_ONLY;
				args[i] = "";
			}
		}

		OdeHelper.initODE2(0);

		// create world
		world = OdeHelper.createWorld();
		space = OdeHelper.createHashSpace (null);
		contactgroup = OdeHelper.createJointGroup ();
		world.setGravity (0,0,-10);
		OdeHelper.createPlane (space,0,0,1,0);

		//create two boxes
		DMass m = OdeHelper.createMass();
		box1_body = OdeHelper.createBody (world);
		m.setBox (1,BOX1_LENGTH,BOX1_WIDTH,BOX1_HEIGHT);
		m.adjust (Mass1);
		box1_body.setMass (m);
		box1 = OdeHelper.createBox (null,BOX1_LENGTH,BOX1_WIDTH,BOX1_HEIGHT);
		box1.setBody (box1_body);

		box2_body = OdeHelper.createBody (world);
		m.setBox (10,BOX2_LENGTH,BOX2_WIDTH,BOX2_HEIGHT);
		m.adjust (Mass2);
		box2_body.setMass (m);
		box2 = OdeHelper.createBox (null,BOX2_LENGTH,BOX2_WIDTH,BOX2_HEIGHT);
		box2.setBody (box2_body);

		//set the initial positions of body1 and body2
		DMatrix3 R = new DMatrix3();
		R.setIdentity();
		box1_body.setPosition (0,0,BOX1_HEIGHT/2.0);
		box1_body.setRotation (R);

		box2_body.setPosition (
				2.1, 
				0.0,
				BOX2_HEIGHT/2.0);
		box2_body.setRotation (R);


		//set PR joint
		joint = OdeHelper.createPRJoint(world,null);
		joint.attach (box1_body,box2_body);  
		switch (flag)
		{
		case PRISMATIC_ONLY:
			joint.setAnchor ( 
					2.1, 
					0.0,
					BOX2_HEIGHT/2.0);
			joint.setParamLoStop (-0.5);
			joint.setParamHiStop ( 1.5);
			break;

		case ROTOIDE_ONLY:
			joint.setAnchor ( 
					0.0, 
					0.0,
					BOX2_HEIGHT/2.0);
			joint.setParamLoStop ( 0.0);
			joint.setParamHiStop ( 0.0);
			break;

		default:
			joint.setAnchor ( 
					1.1, 
					0.0,
					BOX2_HEIGHT/2.0);
		joint.setParamLoStop (-0.5);
		joint.setParamHiStop ( 1.5);
		break;
		}

		joint.setAxis1(1,0,0);
		joint.setAxis2(0,0,1);
		// We position  the 2 body
		// The position of the rotoide joint is on the second body so it can rotate on itself
		// and move along the X axis.
		// With this anchor 
		// - A force in X will move only the body 2 inside the low and hi limit
		//   of the prismatic
		// - A force in Y will make the 2 bodies to rotate around on the plane

		box1_space = OdeHelper.createSimpleSpace (space);
		box1_space.setCleanup (false);
		box1_space.add(box1);

		// run simulation
		dsSimulationLoop (args,640,480,this);
		contactgroup.destroy ();
		space.destroy ();
		world.destroy ();
		OdeHelper.closeODE();
	}


	@Override
	public void stop() {
		// Nothing
	}
}
