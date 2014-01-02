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

import static org.ode4j.cpp.internal.ApiCppBody.dBodyAddForce;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyAddRelTorque;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyCreate;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyGetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetMass;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetPosition;
import static org.ode4j.cpp.internal.ApiCppBody.dBodySetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dCollide;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateBox;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreatePlane;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomBoxGetLengths;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dHashSpaceCreate;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSimpleSpaceCreate;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceAdd;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceDestroy;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceSetCleanup;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateContact;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreatePR;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPRAnchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPRAngle;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPRAngleRate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPRPosition;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPRPositionRate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupCreate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupDestroy;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupEmpty;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPRAnchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPRAxis1;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPRAxis2;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPRParam;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamFMax2;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamHiStop;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamHiStop2;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamLoStop;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamLoStop2;
import static org.ode4j.cpp.internal.ApiCppJoint.dParamVel2;
import static org.ode4j.cpp.internal.ApiCppMass.dMassAdjust;
import static org.ode4j.cpp.internal.ApiCppMass.dMassCreate;
import static org.ode4j.cpp.internal.ApiCppMass.dMassSetBox;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.cpp.internal.ApiCppOther.dAreConnectedExcluding;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldCreate;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldQuickStep;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetGravity;
import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.OdeConstants.dContactApprox1;
import static org.ode4j.ode.OdeConstants.dContactSlip1;
import static org.ode4j.ode.OdeConstants.dContactSlip2;
import static org.ode4j.ode.OdeConstants.dContactSoftCFM;
import static org.ode4j.ode.OdeConstants.dContactSoftERP;
import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.internal.cpp4j.Cmath.sqrt;
import static org.ode4j.ode.internal.cpp4j.Cstdio.printf;
import static org.ode4j.ode.internal.cpp4j.Cstdlib.exit;
import static org.ode4j.ode.internal.cpp4j.Cstring.strcmp;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DContactJoint;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DPRJoint;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;


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
	private static DBody[] box1_body = new DBody[1];
	private static DBody[] box2_body = new DBody[1];
	private static DPRJoint[] joint = new DPRJoint[1];
	private static DJointGroup contactgroup;
	//private static DGeom ground;
	private static DBox[] box1 = new DBox[1];
	private static DBox[] box2 = new DBox[1];


	//collision detection
	//static void nearCallback (void *data, dGeom o1, dGeom o2)
	private static void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		int i,n;

		DBody b1 = dGeomGetBody(o1);
		DBody b2 = dGeomGetBody(o2);
		if (b1!=null && b2!=null && 
				dAreConnectedExcluding (b1,b2,DContactJoint.class)) 
			return;
		final int N = 10;
		//dContact[] contact=new dContact[N];
		DContactBuffer contacts = new DContactBuffer(N);
		n = dCollide (o1,o2,N,contacts.getGeomBuffer());//,sizeof(dContact));
		if (n > 0)
		{
			for (i=0; i<n; i++)
			{
				DContact contact = contacts.get(i);
				contact.surface.mode = dContactSlip1 | dContactSlip2 |
				dContactSoftERP | dContactSoftCFM | dContactApprox1;
				contact.surface.mu = 0.1;
				contact.surface.slip1 = 0.02;
				contact.surface.slip2 = 0.02;
				contact.surface.soft_erp = 0.1;
				contact.surface.soft_cfm = 0.0001;
				DJoint c = dJointCreateContact (world,contactgroup,contact);
				dJointAttach (c,dGeomGetBody(contact.geom.g1),dGeomGetBody(contact.geom.g2));
			}
		}
	}


	// start simulation - set viewpoint
	@Override
	public void start()
	{
		//dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);

		dsSetViewpoint (xyz,hpr);
		printf ("Press 'd' to add force along positive x direction.\nPress 'a' to add force along negative x direction.\n");
		printf ("Press 'w' to add force along positive y direction.\nPress 's' to add force along negative y direction.\n");
		printf ("Press 'e' to add torque around positive z direction.\nPress 'q' to add torque around negative z direction.\n");
		printf ("Press 'o' to add force around positive x direction \n");

	    printf("Press 'v' to give a defined velocity and add a FMax to the rotoide axis\n");
	    printf("Press 'c' to set the velocity to zero and remove the FMax\n");

	    printf("Press 'l' to add limits (-0.5 to 0.5rad) on the rotoide axis\n");
	    printf("Press 'k' to remove the limits on the rotoide axis\n");

	    printf("Press 'i' to get joint info\n");
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
		switch(cmd)
		{
		case 'w': 
		case 'W':
			dBodyAddForce(box2_body[0],0,500,0);
			//std::cout<<(dBodyGetPosition(box2_body[0])[1]-dBodyGetPosition(box1_body[0])[1])<<'\n';
			System.out.print((dBodyGetPosition(box2_body[0]).get1()-dBodyGetPosition(box1_body[0]).get1()) +'\n');
			break;
		case 's': 
		case 'S':
			dBodyAddForce(box2_body[0],0,-500,0);
			System.out.print((dBodyGetPosition(box2_body[0]).get1()-dBodyGetPosition(box1_body[0]).get1())+'\n');
			break;
		case 'd': 
		case 'D':
			dBodyAddForce(box2_body[0],500,0,0);
			System.out.print((dBodyGetPosition(box2_body[0]).get0()-dBodyGetPosition(box1_body[0]).get0())+'\n');
			break;
		case 'a': 
		case 'A':
			dBodyAddForce(box2_body[0],-500,0,0);
			System.out.print((dBodyGetPosition(box2_body[0]).get0()-dBodyGetPosition(box1_body[0]).get0())+'\n');
			break;
		case 'e': 
		case 'E':
			dBodyAddRelTorque(box2_body[0],0,0,200);
			break;
		case 'q': 
		case 'Q':
			dBodyAddRelTorque(box2_body[0],0,0,-200);
			break;
		case 'o': 
		case 'O':
			dBodyAddForce(box1_body[0],10000,0,0);
			break;
	    case 'v':
	    case 'V':
	        dJointSetPRParam(joint[0], dParamVel2, 2);
	        dJointSetPRParam(joint[0], dParamFMax2, 500);
	        break;

	    case 'c':
	    case 'C':
	        dJointSetPRParam(joint[0], dParamVel2, 0);
	        dJointSetPRParam(joint[0], dParamFMax2, 0);
	        break;

	    case 'l':
	    case 'L':
	        dJointSetPRParam(joint[0], dParamLoStop2, -0.5);
	        dJointSetPRParam(joint[0], dParamHiStop2,  0.5);
	        break;

	    case 'k':
	    case 'K':
	        dJointSetPRParam(joint[0], dParamLoStop2, -dInfinity);
	        dJointSetPRParam(joint[0], dParamHiStop2,  dInfinity);
	        break;

	    case 'i':
	    case 'I':
	        DVector3 anchor = new DVector3();
	        dJointGetPRAnchor(joint[0], anchor);
	        double angle = dJointGetPRAngle(joint[0]);
	        double w = dJointGetPRAngleRate(joint[0]);

	        double l = dJointGetPRPosition(joint[0]);
	        double v = dJointGetPRPositionRate(joint[0]);

	        printf("Anchor: [%6.4lf, %6.4lf, %6.4lf]\n", anchor.get0(), anchor.get1(), anchor.get2());
	        printf("Position: %7.4lf, Rate: %7.4lf\n", l, v);
	        printf("Angle: %7.4lf, Rate: %7.4lf\n", angle, w);
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
			DVector3 ss = new DVector3();
			dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);

			final DVector3C posBox2 = dGeomGetPosition(box2[0]);
			final DMatrix3C rotBox2 = dGeomGetRotation(box2[0]);
			dsSetColor (1,1,0);
			dGeomBoxGetLengths (box2[0],ss);
			dsDrawBox (posBox2, rotBox2, ss);

			final DVector3C posBox1 = dGeomGetPosition(box1[0]);
			final DMatrix3C rotBox1 = dGeomGetRotation(box1[0]);
			dsSetColor (1,1,2);
			dGeomBoxGetLengths (box1[0], ss);
			dsDrawBox (posBox1, rotBox1, ss);

			DVector3 anchorPos = new DVector3();
			dJointGetPRAnchor (joint[0], anchorPos);

			// Draw the axisP 
			if (ROTOIDE_ONLY != flag )
			{
				dsSetColor (1,0,0);
				DVector3 sizeP = new DVector3(0, 0.1, 0.1);
				for (int i=0; i<3; ++i)
					sizeP.add(0, (anchorPos.get(i) - posBox1.get(i))*(anchorPos.get(i) - posBox1.get(i)) );
				sizeP.set(0, sqrt(sizeP.get(0)));
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
				sizeR.set(0, sqrt(sizeR.get(0)));
				DVector3 posAxisR = new DVector3();
				for (int i=0; i<3; ++i)
					posAxisR.set(i, posBox2.get(i) + (anchorPos.get(i) - posBox2.get(i))/2.0 );
				dsDrawBox (posAxisR, rotBox2, sizeR);
			}

			//		dSpaceCollide (space,0,&nearCallback);
			dSpaceCollide (space,null,new DNearCallback() {
				@Override
				public void call(Object data, DGeom o1, DGeom o2) {
					nearCallback(data, o1, o2);
				}} );
			dWorldQuickStep (world,0.0001);
			update();
			dJointGroupEmpty (contactgroup);
		}
	}


	@Override
	public void dsPrintHelp()
	{
		super.dsPrintHelp();
		printf(" -b | --both : Display how the complete joint works\n");
		printf("               Default behavior\n");
		printf(" -p | --prismatic-only : Display how the prismatic part works\n");
		printf("                         The anchor pts is set at the center of body 2\n");
		printf(" -r | --rotoide-only   : Display how the rotoide part works\n");
		printf("                         The anchor pts is set at the center of body 1\n");
		printf("--------------------------------------------------\n");
		printf("Hit any key to continue:");
		//  getchar();

		exit(0);
	}

	public static void main(String[] args) {
		new DemoJointPR().demo(args);
	}
	
	private void demo(String[] args) {
		for (int i=0; i < args.length; ++i)
		{
//			if(  0 == strcmp("-h", args[i]) || 0 == strcmp("--help", args[i]) )
//				Help(args);

			if(flag==0 && (0 == strcmp("-p", args[i]) ||0 == strcmp("--prismatic-only", args[i])) ) {
				flag = PRISMATIC_ONLY;
				args[i] = "";
			}


			if(flag==0 && (0 == strcmp("-r", args[i]) || 0 == strcmp("--rotoide-only", args[i])) ) {
				flag = ROTOIDE_ONLY;
				args[i] = "";
			}


//			if(0 == strcmp("-t", args[i]) || 0 == strcmp("--texture-path", args[i]))
//			{
//				int j = i+1;
//				if ( j+1 > args.length      ||  // Check if we have enough arguments
//						args[j].equals('\0') ||  // We should have a path here
//						args[j].charAt(0) == '-' ) // We should have a path not a command line
//					Help(args);
//				else
//					dsSetPathToTextures( args[++i] ); // Increase i since we use this argument
//			}
		}

		dInitODE2(0);

		// create world
		world = dWorldCreate();
		space = dHashSpaceCreate (null);
		contactgroup = dJointGroupCreate (0);
		dWorldSetGravity (world,0,0,-10);
		dCreatePlane (space,0,0,1,0);

		//create two boxes
		DMass m = dMassCreate();
		box1_body[0] = dBodyCreate (world);
		dMassSetBox (m,1,BOX1_LENGTH,BOX1_WIDTH,BOX1_HEIGHT);
		dMassAdjust (m,Mass1);
		dBodySetMass (box1_body[0],m);
		box1[0] = dCreateBox (null,BOX1_LENGTH,BOX1_WIDTH,BOX1_HEIGHT);
		dGeomSetBody (box1[0],box1_body[0]);

		box2_body[0] = dBodyCreate (world);
		dMassSetBox (m,10,BOX2_LENGTH,BOX2_WIDTH,BOX2_HEIGHT);
		dMassAdjust (m,Mass2);
		dBodySetMass (box2_body[0],m);
		box2[0] = dCreateBox (null,BOX2_LENGTH,BOX2_WIDTH,BOX2_HEIGHT);
		dGeomSetBody (box2[0],box2_body[0]);

		//set the initial positions of body1 and body2
		DMatrix3 R = new DMatrix3();
		R.setIdentity();
		dBodySetPosition (box1_body[0],0,0,BOX1_HEIGHT/2.0);
		dBodySetRotation (box1_body[0], R);

		dBodySetPosition (box2_body[0],
				2.1, 
				0.0,
				BOX2_HEIGHT/2.0);
		dBodySetRotation (box2_body[0], R);


		//set PR joint
		joint[0] = dJointCreatePR(world,null);
		dJointAttach (joint[0],box1_body[0],box2_body[0]);  
		switch (flag)
		{
		case PRISMATIC_ONLY:
			dJointSetPRAnchor (joint[0], 
					2.1, 
					0.0,
					BOX2_HEIGHT/2.0);
			dJointSetPRParam (joint[0],dParamLoStop, -0.5);
			dJointSetPRParam (joint[0],dParamHiStop, 1.5);
			break;

		case ROTOIDE_ONLY:
			dJointSetPRAnchor (joint[0], 
					0.0, 
					0.0,
					BOX2_HEIGHT/2.0);
			dJointSetPRParam (joint[0],dParamLoStop, 0.0);
			dJointSetPRParam (joint[0],dParamHiStop, 0.0);
			break;

		default:
			dJointSetPRAnchor (joint[0], 
					1.1, 
					0.0,
					BOX2_HEIGHT/2.0);
		dJointSetPRParam (joint[0],dParamLoStop, -0.5);
		dJointSetPRParam (joint[0],dParamHiStop, 1.5);
		break;
		}

		dJointSetPRAxis1(joint[0],1,0,0);
		dJointSetPRAxis2(joint[0],0,0,1);
		// We position  the 2 body
		// The position of the rotoide joint is on the second body so it can rotate on itself
		// and move along the X axis.
		// With this anchor 
		// - A force in X will move only the body 2 inside the low and hi limit
		//   of the prismatic
		// - A force in Y will make the 2 bodies to rotate around on the plane

		box1_space = dSimpleSpaceCreate (space);
		dSpaceSetCleanup (box1_space,false);
		dSpaceAdd(box1_space,box1[0]);

		// run simulation
		dsSimulationLoop (args,400,300,this);
		dJointGroupDestroy (contactgroup);
		dSpaceDestroy (space);
		dWorldDestroy (world);
		dCloseODE();
	}


	@Override
	public void stop() {
		// Nothing
	}
}
