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
 * Created by:    Remi Ricard                                            *
 *                (remi.ricard@simlog.com or papaDoc@videotron.ca)       *
 * Creation date: 2007/05/04                                             *
 *************************************************************************/
package org.ode4j.democpp;

import static org.ode4j.cpp.internal.ApiCppBody.dBodyAddForce;
import static org.ode4j.cpp.internal.ApiCppBody.dBodyAddTorque;
import static org.ode4j.cpp.internal.ApiCppCollision.dCollide;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateBox;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateCapsule;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreatePlane;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomBoxGetLengths;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomCapsuleGetParams;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetCategoryBits;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetCollideBits;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetOffsetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetOffsetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSimpleSpaceCreate;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceDestroy;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAddPistonForce;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAddSliderForce;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateContact;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateFixed;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPistonAnchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupCreate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupDestroy;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupEmpty;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetFixed;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPistonAnchor;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.cpp.internal.ApiCppOther.dAreConnectedExcluding;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldSetGravity;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldStep;
import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCapsule;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCylinder;
import static org.ode4j.drawstuff.DrawStuff.dsDrawSphere;
import static org.ode4j.drawstuff.DrawStuff.dsElapsedTime;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeConstants.dContactApprox1;
import static org.ode4j.ode.OdeConstants.dContactSlip1;
import static org.ode4j.ode.OdeConstants.dContactSlip2;
import static org.ode4j.ode.OdeConstants.dContactSoftCFM;
import static org.ode4j.ode.OdeConstants.dContactSoftERP;
import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.internal.cpp4j.Cmath.ceilf;
import static org.ode4j.ode.internal.cpp4j.Cstdio.getchar;
import static org.ode4j.ode.internal.cpp4j.Cstdio.printf;
import static org.ode4j.ode.internal.cpp4j.Cstdio.std_cout;
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
import org.ode4j.ode.DCapsule;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DContactJoint;
import org.ode4j.ode.DFixedJoint;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJoint.PARAM_N;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DPistonJoint;
import org.ode4j.ode.DSliderJoint;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;


/**
 * This program demonstrates how the Piston joint works.
 * 
 * A Piston joint enables the sliding of a body with respect to another body
 * and the 2 bodies are free to rotate about the sliding axis.
 *  - The yellow body is fixed to the world.
 *  - The yellow body and the blue body are attached by a Piston joint with
 *    the axis along the x direction.
 *  - The purple object is a geometry obstacle.
 *  - The red line is the representation of the prismatic axis
 *  - The orange line is the representation of the rotoide axis
 *  - The light blue ball is the anchor position
 * N.B. Many command options are available type -h to print them.
 */
class DemoPiston extends dsFunctions {

	private final double VEL_INC = 0.01; // Velocity increment

	// physics parameters
	private final double PI = 3.14159265358979323846264338327950288419716939937510;
	private final double BODY1_LENGTH = 1.5;    // Size along the X axis



	private final double RADIUS = 0.2;
	private final float AXIS_RADIUS = 0.01f;


	//#define X 0
	//#define Y 1
	//#define Z 2
	private static final int X = 0;
	private static final int Y = 1;
	private static final int Z = 2;

	//private enum INDEX
	//{
	private static final int BODY1 = 0;
	private static final int BODY2 = 1;
	private static final int RECT = 2;
	//private static final int BOX = 3;
	private static final int OBS = 4;
	private static final int GROUND = 5;
	private static final int NUM_PARTS = 6;
	private static final int ALL = NUM_PARTS;
	//};

	private static final long[] catBits = //[NUM_PARTS+1] =
	{
		0x0001, ///< Ext Cylinder category
		0x0002, ///< Int Cylinder category
		0x0004, ///< Int_Rect Cylinder category
		0x0008, ///< Box category
		0x0010, ///< Obstacle category
		0x0020, ///< Ground category
		~0L    ///< All categories
	};

	//#define Mass1 10
	//#define Mass2 8
	private static final double Mass1 = 10;
	//private static final double Mass2 = 8;

	//camera view
	private static float[] xyz = {2.0f,-3.5f,2.0000f};
	private static float[] hpr = {90.000f,-25.5000f,0.0000f};


	//world,space,body & geom
	private static DWorld           world;
	private static DSpace         space;
	private static DJointGroup    contactgroup;
	private static DBody[]            body = new DBody[NUM_PARTS];
	private static DGeom[]          geom = new DGeom[NUM_PARTS];

	// Default Positions and anchor of the 2 bodies
	private DVector3 pos1 = new DVector3();
	private DVector3 pos2 = new DVector3();
	private DVector3 anchor = new DVector3();

	private static DJoint joint;


	private static final double[] BODY2_SIDES = {0.4, 0.4, 0.4};
	private static final double[] OBS_SIDES  = {1,1,1};
	private static final double[] RECT_SIDES = {0.3, 0.1, 0.2};


	private Class<?> type = DPistonJoint.class;

	//#pragma message("tc to be changed to 0")

	private int tc = 0; // The test case choice;


	private DNearCallback nearCallback = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}
	};

	//collision detection
	private static void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		int i,n;

		DBody b1 = dGeomGetBody (o1);
		DBody b2 = dGeomGetBody (o2);
		if (b1!=null && b2!=null && dAreConnectedExcluding (b1,b2,DContactJoint.class) ) return;
		final int N = 10;
		DContactBuffer contacts = new DContactBuffer(N);
		n = dCollide (o1,o2,N,contacts.getGeomBuffer());//[0].geom,sizeof (dContact) );
		if (n > 0) {
			for  (i=0; i<n; i++) {
				DContact contact = contacts.get(i);
				contact.surface.mode = (dContactSlip1 | dContactSlip2 |
						dContactSoftERP | dContactSoftCFM |
						dContactApprox1);
				contact.surface.mu = 0.1;
				contact.surface.slip1 = 0.02;
				contact.surface.slip2 = 0.02;
				contact.surface.soft_erp = 0.1;
				contact.surface.soft_cfm = 0.0001;
				DJoint c = dJointCreateContact (world,contactgroup,contact);
				dJointAttach (c,dGeomGetBody (contact.geom.g1),dGeomGetBody (contact.geom.g2) );
			}
		}
	}

	private static void printKeyBoardShortCut()
	{
		printf ("Press 'h' for this help.\n");
		printf ("Press 'q' to add force on BLUE body along positive x direction.\n");
		printf ("Press 'w' to add force on BLUE body along negative x direction.\n");

		printf ("Press 'a' to add force on BLUE body along positive y direction.\n");
		printf ("Press 's' to add force on BLUE body along negative y direction.\n");

		printf ("Press 'z' to add force on BLUE body along positive z direction.\n");
		printf ("Press 'x' to add force on BLUE body along negative z direction.\n");

		printf ("Press 'e' to add torque on BLUE body around positive x direction \n");
		printf ("Press 'r' to add torque on BLUE body around negative x direction \n");

		printf ("Press 'd' to add torque on BLUE body around positive y direction \n");
		printf ("Press 'f' to add torque on BLUE body around negative y direction \n");

		printf ("Press 'c' to add torque on BLUE body around positive z direction \n");
		printf ("Press 'v' to add torque on BLUE body around negative z direction \n");

		printf ("Press 't' to add force on prismatic joint in the positive axis direction\n");
		printf ("Press 'y' to add force on prismatic joint in the negative axis direction\n");

	    printf ("Press 'i' to add limits on the prismatic joint (0 to 0) \n");
	    printf ("Press 'o' to add limits on the rotoide joint (0 to 0)\n");
		printf ("Press 'k' to add limits on the rotoide joint (-45 to 45deg) \n");
		printf ("Press 'l' to remove limits on the rotoide joint \n");


		printf ("Press '.' to increase joint velocity along the prismatic direction.\n");
		printf ("Press ',' to decrease joint velocity along the prismatic direction.\n");

		printf ("Press 'p' to print the Position of the joint.\n");

		printf ("Press '+' Go to the next test case.\n");
		printf ("Press '-' Go to the previous test case.\n");

	    printf ("Press '8' To remove one of the body. The blue body and the world will be\n");
	    printf ("          attached to the joint (blue body at position 1)\n");
	    printf ("Press '9' To remove one of the body. The blue body and the world will be\n");
	    printf ("          attached to the joint (body body at position 2)\n");
	}


	// start simulation - set viewpoint
	@Override
	public void start()
	{
		//dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);

		dsSetViewpoint (xyz,hpr);
		printf ("This program demonstrates how the Piston joint works.\n");
		printf ("A Piston joint enables the sliding of a body with respect to another body\n");
		printf ("and the 2 bodies are free to rotate about the sliding axis.\n\n");
		printf ("The yellow body is fixed to the world\n");
		printf ("The yellow body and the blue body are attached by a Piston joint with\n");
		printf ("the axis along the x direction.\n");
		printf ("The purple object is a geometry obstacle.\n");

		printKeyBoardShortCut();
	}


	private void setPositionBodies (int val)
	{
		final DVector3 POS1 = new DVector3(0,0,1.5);
		final DVector3 POS2 = new DVector3(0,0,1.5);
		final DVector3 ANCHOR = new DVector3(0,0,1.5);

		//  for (int i=0; i<3; ++i) {
		//    pos1[i] = POS1[i];
		//    pos2[i] = POS2[i];
		//    anchor[i] = ANCHOR[i];
		//  }
		pos1.set(POS1);
		pos2.set(POS2);
		anchor.set(ANCHOR);

		if (body[BODY1]!=null) {
			body[BODY1].setLinearVel(0,0,0);
			body[BODY1].setAngularVel(0,0,0);
		}

		if (body[BODY2]!=null) {
			body[BODY2].setLinearVel(0,0,0);
			body[BODY2].setAngularVel(0,0,0);
		}

		switch (val) {
		case 3:
			pos1.add(Z, -0.5);//[Z] += -0.5;
			anchor.add(Z, -0.25);//[Z] -= 0.25;
			break;
		case 2:
			pos1.add(Z, -0.5);//[Z] -= 0.5;
			anchor.add(Z, -0.5);//[Z] -= 0.5;
			break;
		case 1:
			pos1.add(Z, -0.5);//[Z] += -0.5;
			break;
		default: // This is also case 0
		// Nothing to be done
			break;
		}

		DMatrix3C R = new DMatrix3(
				1,0,0,
				0,1,0,
				0,0,1
		);

		if (body[BODY1]!=null) {
			body[BODY1].setPosition(pos1.get(X), pos1.get(Y), pos1.get(Z));
			body[BODY1].setRotation(R);
		}

		if (body[BODY2]!=null) {
			body[BODY2].setPosition(pos2.get(X), pos2.get(Y), pos2.get(Z));
			body[BODY2].setRotation(R);
		}



		if (joint!=null) {
			joint.attach (body[BODY1], body[BODY2]);
			if (joint instanceof DPistonJoint)
				dJointSetPistonAnchor((DPistonJoint)joint, anchor.get(X), anchor.get(Y), anchor.get(Z));
		}

	}


	// function to update camera position at each step.
	private void update()
	{
		//   static FILE *file = fopen("x:/sim/src/libode/tstsrcSF/export.dat", "w");

		//   static int cnt = 0;
		//   char str[24];
		//   sprintf(str, "%06d",cnt++);

		//   dWorldExportDIF(world, file, str);
	}


	// called when a key pressed
	@Override
	public void command (char cmd)
	{
		switch (cmd) {
		case 'h' : case 'H' : case '?' :
			printKeyBoardShortCut();
			break;

			// Force
		case 'q' : case 'Q' :
			dBodyAddForce (body[BODY1],4,0,0);
			break;
		case 'w' : case 'W' :
			dBodyAddForce (body[BODY1],-4,0,0);
			break;

		case 'a' : case 'A' :
			dBodyAddForce (body[BODY1],0,40,0);
			break;
		case 's' : case 'S' :
			dBodyAddForce (body[BODY1],0,-40,0);
			break;

		case 'z' : case 'Z' :
			dBodyAddForce (body[BODY1],0,0,4);
			break;
		case 'x' : case 'X' :
			dBodyAddForce (body[BODY1],0,0,-4);
			break;

			// Torque
		case 'e': case 'E':
			dBodyAddTorque (body[BODY1],0.1,0,0);
			break;
		case 'r': case 'R':
			dBodyAddTorque (body[BODY1],-0.1,0,0);
			break;

		case 'd': case 'D':
			dBodyAddTorque (body[BODY1],0, 0.1,0);
			break;
		case 'f': case 'F':
			dBodyAddTorque (body[BODY1],0,-0.1,0);
			break;

		case 'c': case 'C':
			dBodyAddTorque (body[BODY1],0.1,0,0);
			break;
		case 'v': case 'V':
			dBodyAddTorque (body[BODY1],-0.1,0,0);
			break;

		case 't': case 'T':
			if (joint instanceof DPistonJoint)
				dJointAddPistonForce ((DPistonJoint)joint,1);
			else
				dJointAddSliderForce ((DSliderJoint)joint,1);
			break;
		case 'y': case 'Y':
			if (joint instanceof DPistonJoint)
				dJointAddPistonForce ((DPistonJoint)joint,-1);
			else
				dJointAddSliderForce ((DSliderJoint)joint,-1);
			break;
	    case '8' :
	        dJointAttach(joint, body[0], null);
	        break;
	    case '9' :
	        dJointAttach(joint, null, body[0]);
	        break;

	    case 'i':
	    case 'I' :
	        joint.setParam (PARAM_N.dParamLoStop1, 0);
	        joint.setParam (PARAM_N.dParamHiStop1, 0);
	        break;

	    case 'o':
	    case 'O' :
	        joint.setParam (PARAM_N.dParamLoStop2, 0);
	        joint.setParam (PARAM_N.dParamHiStop2, 0);
	        break;

	    case 'k':
	    case 'K':
	        joint.setParam (PARAM_N.dParamLoStop2, -45.0*3.14159267/180.0);
	        joint.setParam (PARAM_N.dParamHiStop2,  45.0*3.14159267/180.0);
			break;
	    case 'l':
	    case 'L':
	        joint.setParam (PARAM_N.dParamLoStop2, -dInfinity);
	        joint.setParam (PARAM_N.dParamHiStop2, dInfinity);
			break;

			// Velocity of joint
		case ',': case '<' : {
			double vel = joint.getParam (PARAM_N.dParamVel1) - VEL_INC;
			joint.setParam (PARAM_N.dParamVel1, vel);
			std_cout("Velocity = ",vel,"  FMax = 2",'\n');
		}
		break;

		case '.': case '>' : {
			double vel = joint.getParam (PARAM_N.dParamVel1) + VEL_INC;
			joint.setParam (PARAM_N.dParamVel1, vel);
			std_cout("Velocity = ",vel,"  FMax = 2",'\n');
		}
		break;

		case 'p' :case 'P' : {
			if ( joint instanceof DSliderJoint) {
				DSliderJoint sj = (DSliderJoint) (joint);
				std_cout("Position =",sj.getPosition() ,"\n");
			} else {
				DPistonJoint rj = (DPistonJoint ) (joint);
				std_cout("Position =",rj.getPosition() ,"\n");
			}
		}
		break;

		case '+' :
			tc = ++tc % 4;//(++tc) %= 4;
			setPositionBodies (tc);
			break;
		case '-' :
			tc = --tc % 4;//(--tc) %= 4;
			setPositionBodies (tc);
			break;


		}
	}

	private static void drawBox (DGeom id, int R, int G, int B)
	{
		if (id==null)
			return;

		final DVector3C pos = dGeomGetPosition (id);
		final DMatrix3C rot = dGeomGetRotation (id);
		dsSetColor (R,G,B);

		DVector3 l = new DVector3();
		dGeomBoxGetLengths ((DBox)id, l);
		dsDrawBox (pos, rot, l);
	}


	// simulation loop
	private void simLoop (boolean pause)
	{
		DMatrix3C rot;
		DVector3 ax = new DVector3();
		double l=0;

		if (joint instanceof DSliderJoint) {
			( (DSliderJoint ) joint).getAxis (ax);
			l = ( (DSliderJoint ) joint).getPosition();
		} else {
			( (DPistonJoint ) joint).getAxis (ax);
			l = ( (DPistonJoint ) joint).getPosition();
		}


		if (!pause) {
			double simstep = 0.01; // 1ms simulation steps
			double dt = dsElapsedTime();

			int nrofsteps = (int) ceilf (dt/simstep);
			if (nrofsteps == 0)
				nrofsteps = 1;

			for (int i=0; i<nrofsteps && !pause; i++) {
				dSpaceCollide (space,0,nearCallback);
				dWorldStep (world, simstep);

				dJointGroupEmpty (contactgroup);
			}

			update();


			RefDouble radius = new RefDouble(), length = new RefDouble();

			dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);

			drawBox (geom[BODY2], 1,1,0);

			drawBox (geom[RECT], 0,0,1);

			if ( geom[BODY1] != null) {
				final DVector3C pos = dGeomGetPosition (geom[BODY1]);
				rot = dGeomGetRotation (geom[BODY1]);
				dsSetColor (0,0,1);

				dGeomCapsuleGetParams ((DCapsule)geom[BODY1], radius, length);
				dsDrawCapsule (pos, rot, length.getF(), radius.getF());
			}


			drawBox (geom[OBS], 1,0,1);


			// Draw the prismatic axis
			if ( geom[BODY1] != null) {
				final DVector3C pos = dGeomGetPosition (geom[BODY1]);
				rot = dGeomGetRotation (geom[BODY2]);
				DVector3 p = new DVector3();
//				p[X] = pos[X] - l*ax[X];
//				p[Y] = pos[Y] - l*ax[Y];
//				p[Z] = pos[Z] - l*ax[Z];
				p.eqSum(pos, ax, -l);
				dsSetColor (1,0,0);
				dsDrawCylinder (p, rot, 3.75f, 1.05f*AXIS_RADIUS);
			}


			if (joint instanceof DPistonJoint ) {
				DVector3 anchor = new DVector3();
				dJointGetPistonAnchor((DPistonJoint)joint, anchor);

				// Draw the rotoide axis
				rot = dGeomGetRotation (geom[BODY2]);
				dsSetColor (1f,0.5f,0);
				dsDrawCylinder (anchor, rot, 4f, AXIS_RADIUS);


				dsSetColor (0,1,1);
				rot = dGeomGetRotation (geom[BODY1]);
				dsDrawSphere (anchor, rot, 1.5*RADIUS);
			}

		}
	}


	@Override
	public void dsPrintHelp ()
	{
		super.dsPrintHelp();
		//printf ("%s ", args[0]);
//		printf (" -h | --help   : print this help\n");
		printf (" -s | --slider : Set the joint as a slider\n");
		printf (" -p | --piston : Set the joint as a Piston. (Default joint)\n");
		printf (" -1 | --offset1 : Create an offset between the 2 bodies\n");
		printf ("                  Offset one of the body by z=-0.5 and keep the anchor\n");
		printf ("                  point in the middle of the fixed body\n");
		printf (" -2 | --offset2 : Create an offset between the 2 bodies\n");
		printf ("                  Offset one of the body by z=-0.5 and set the anchor\n");
		printf ("                  point in the middle of the movable body\n");
		printf (" -3 | --offset3 : Create an offset between the 2 bodies\n");
		printf ("                  Offset one of the body by z=-0.5 and set the anchor\n");
		printf ("                  point in the middle of the 2 bodies\n");
//		printf (" -t | --texture-path path  : Path to the texture.\n");
//		printf ("                             Default = %s\n", DRAWSTUFF_TEXTURE_PATH);
		printf (" -n | --notFixed : In free space with no gravity mode");
//		printf ("-notex          : Don't use texture\n");
//		printf ("-noshadow       : No shadow\n");
//		printf ("-noshadows      : No shadows\n");
//		printf ("-pause          : Initial pause\n");
		printf ("--------------------------------------------------\n");
		printf ("Hit any key to continue:");
		getchar();

		exit (0);
	}

	public static void main(String[] args) {
		new DemoPiston().demo(args);
	}

	private void demo(String[] args) {
		dInitODE2(0);
		boolean fixed  = true;

		// setup pointers to drawstuff callback functions
		//dsFunctions fn = this;
		//  fn.version = DS_VERSION;
		//  fn.start = &start;
		//  fn.step = &simLoop;
		//  fn.command = &command;
		//  fn.stop = 0;
		//fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

		//DVector3 offset = new DVector3();
		//dSetZero (offset, 4);

		// Default test case

//		if (args.length >= 2 ) {
			for (int i=0; i < args.length; ++i) {
				//static int tata = 0;

				if (true) {
//					if ( 0 == strcmp ("-h", args[i]) || 0 == strcmp ("--help", args[i]) )
//						Help (args);

					if ( 0 == strcmp ("-s", args[i]) || 0 == strcmp ("--slider", args[i]) ) {
						type = DSliderJoint.class;
						 args[i] = ""; 
					}

//					if ( 0 == strcmp ("-t", args[i]) || 0 == strcmp ("--texture-path", args[i]) ) {
//						int j = i+1;
//						if ( j+1 > args.length      ||  // Check if we have enough arguments
//								//TZ ? args[j] == '\0' ||  // We should have a path here
//								args[j].charAt(0) == '-' ) // We should have a path not a command line
//							Help (args);
//						else
//							dsSetPathToTextures( args[++i] ); // Increase i since we use this argument
//					}
				}


				if ( 0 == strcmp ("-1", args[i]) || 0 == strcmp ("--offset1", args[i]) ) {
					tc = 1;
					args[i] = ""; 
				}

				if ( 0 == strcmp ("-2", args[i]) || 0 == strcmp ("--offset2", args[i]) ) {
					tc = 2;
					args[i] = ""; 
				}

				if ( 0 == strcmp ("-3", args[i]) || 0 == strcmp ("--offset3", args[i]) ) {
					tc = 3;
					args[i] = ""; 
				}

				if (0 == strcmp ("-n", args[i]) || 0 == strcmp ("--notFixed", args[i]) ) {
					fixed = false;
					args[i] = ""; 
				}
			}
//		}

		world = OdeHelper.createWorld();
		world.setERP (0.8);

		space = dSimpleSpaceCreate (null);
		contactgroup = dJointGroupCreate (0);
		geom[GROUND] = dCreatePlane (space, 0,0,1,0);
		dGeomSetCategoryBits (geom[GROUND], catBits[GROUND]);
		dGeomSetCollideBits (geom[GROUND], catBits[ALL]);

		DMass m = OdeHelper.createMass();
		DMatrix3 R = new DMatrix3();


		// Create the Obstacle
		geom[OBS] = dCreateBox (space, OBS_SIDES[0], OBS_SIDES[1], OBS_SIDES[2]);
		dGeomSetCategoryBits (geom[OBS], catBits[OBS]);
		dGeomSetCollideBits (geom[OBS], catBits[ALL]);
		//Rotation of 45deg around y
		dRFromAxisAndAngle (R, 1,1,0, -0.25*PI);
		dGeomSetRotation (geom[OBS], R);
		dGeomSetPosition (geom[OBS], 1.95, -0.2, 0.5);


		//Rotation of 90deg around y
		// Will orient the Z axis along X
		dRFromAxisAndAngle (R, 0,1,0, -0.5*PI);


		// Create Body2 (Wiil be attached to the world)
		body[BODY2] = OdeHelper.createBody(world);//.create (world);
		// Main axis of cylinder is along X=1
		m.setBox (1, BODY2_SIDES[0], BODY2_SIDES[1], BODY2_SIDES[2]);
		m.adjust (Mass1);
		geom[BODY2] = dCreateBox (space, BODY2_SIDES[0], BODY2_SIDES[1], BODY2_SIDES[2]);
		dGeomSetBody (geom[BODY2], body[BODY2]);
		dGeomSetOffsetRotation (geom[BODY2], R);
		dGeomSetCategoryBits (geom[BODY2], catBits[BODY2]);
		dGeomSetCollideBits (geom[BODY2], catBits[ALL] & (~catBits[BODY1]) );
		body[BODY2].setMass(m);


		// Create Body 1 (Slider on the prismatic axis)
		body[BODY1] = OdeHelper.createBody(world);//.create (world);
		// Main axis of capsule is along X=1
		m.setCapsule (1, 1, RADIUS, BODY1_LENGTH);
		m.adjust(Mass1);
		geom[BODY1] = dCreateCapsule (space, RADIUS, BODY1_LENGTH);
		dGeomSetBody (geom[BODY1], body[BODY1]);
		dGeomSetOffsetRotation (geom[BODY1], R);
		dGeomSetCategoryBits (geom[BODY1], catBits[BODY1]);
		dGeomSetCollideBits (geom[BODY1], catBits[ALL] & ~catBits[BODY2] & ~catBits[RECT]);

		DMass mRect = OdeHelper.createMass();
		mRect.setBox(1, RECT_SIDES[0], RECT_SIDES[1], RECT_SIDES[2]);
		m.add(mRect);
		// TODO: translate m?
		geom[RECT] = dCreateBox (space, RECT_SIDES[0], RECT_SIDES[1], RECT_SIDES[2]);
		dGeomSetBody (geom[RECT], body[BODY1]);
		dGeomSetOffsetPosition (geom[RECT],
				(BODY1_LENGTH-RECT_SIDES[0]) /2.0,
				0.0,
				-RADIUS -RECT_SIDES[2]/2.0);
		dGeomSetCategoryBits (geom[RECT], catBits[RECT]);
		dGeomSetCollideBits (geom[RECT], catBits[ALL] & (~catBits[BODY1]) );

		body[BODY1].setMass(m);



		setPositionBodies (tc);


		if ( fixed ) {
			// Attache external cylinder to the world
			DFixedJoint fixedJ = dJointCreateFixed (world,null);
			dJointAttach (fixedJ , null, body[BODY2]);
			dJointSetFixed (fixedJ );
			dWorldSetGravity (world,0,0,-0.8);
		}
		else {
			dWorldSetGravity (world,0,0,0);
		}




		// The static is here only to help debugging
		if ( type == DSliderJoint.class ) {
			DSliderJoint sj = OdeHelper.createSliderJoint (world, null);
			sj.attach (body[BODY1], body[BODY2]);
			sj.setAxis (1, 0, 0);
			joint = sj;
		} else { // fall through default
			DPistonJoint pj = OdeHelper.createPistonJoint(world, null);
			pj.attach (body[BODY1], body[BODY2]);
			pj.setAxis (1, 0, 0);

			dJointSetPistonAnchor(pj, anchor.get(X), anchor.get(Y), anchor.get(Z));

			joint = pj;
		}


		// run simulation
		dsSimulationLoop (args,640,480,this);

		//delete joint;
		dJointGroupDestroy (contactgroup);
		dSpaceDestroy (space);
		dWorldDestroy (world);
		dCloseODE();
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



