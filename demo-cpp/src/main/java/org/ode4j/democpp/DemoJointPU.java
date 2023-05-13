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
import static org.ode4j.cpp.internal.ApiCppBody.dBodyAddTorque;
import static org.ode4j.cpp.internal.ApiCppCollision.dCollide;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateBox;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreateCylinder;
import static org.ode4j.cpp.internal.ApiCppCollision.dCreatePlane;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomBoxGetLengths;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomCylinderGetParams;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetQuaternion;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomGetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetBody;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetCategoryBits;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetCollideBits;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetPosition;
import static org.ode4j.cpp.internal.ApiCppCollision.dGeomSetRotation;
import static org.ode4j.cpp.internal.ApiCppCollision.dSpaceCollide;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSimpleSpaceCreate;
import static org.ode4j.cpp.internal.ApiCppCollisionSpace.dSpaceDestroy;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointAttach;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateContact;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointCreateFixed;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPRAnchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGetPUAnchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupCreate;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupDestroy;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointGroupEmpty;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetFixed;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPRAnchor;
import static org.ode4j.cpp.internal.ApiCppJoint.dJointSetPUAnchor;
import static org.ode4j.cpp.internal.ApiCppMass.dMassCreate;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dCloseODE;
import static org.ode4j.cpp.internal.ApiCppOdeInit.dInitODE2;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldDestroy;
import static org.ode4j.cpp.internal.ApiCppWorld.dWorldStep;
import static org.ode4j.drawstuff.DrawStuff.dsDrawBox;
import static org.ode4j.drawstuff.DrawStuff.dsDrawCylinder;
import static org.ode4j.drawstuff.DrawStuff.dsElapsedTime;
import static org.ode4j.drawstuff.DrawStuff.dsSetColor;
import static org.ode4j.drawstuff.DrawStuff.dsSetTexture;
import static org.ode4j.drawstuff.DrawStuff.dsSetViewpoint;
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.drawstuff.DrawStuff.dsStop;
import static org.ode4j.ode.DRotation.dQFromAxisAndAngle;
import static org.ode4j.ode.DRotation.dQMultiply1;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.DRotation.dRfromQ;
import static org.ode4j.ode.OdeConstants.dContactApprox1;
import static org.ode4j.ode.OdeConstants.dContactSlip1;
import static org.ode4j.ode.OdeConstants.dContactSlip2;
import static org.ode4j.ode.OdeConstants.dContactSoftCFM;
import static org.ode4j.ode.OdeConstants.dContactSoftERP;
import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.OdeMath.dNormalize3;
import static org.ode4j.ode.internal.cpp4j.Cmath.ceilf;
import static org.ode4j.ode.internal.cpp4j.Cstdio.printf;
import static org.ode4j.ode.internal.cpp4j.Cstdio.std_cout;
import static org.ode4j.ode.internal.cpp4j.Cstdlib.exit;
import static org.ode4j.ode.internal.cpp4j.Cstring.strcmp;

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DCylinder;
import org.ode4j.ode.DFixedJoint;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJoint.PARAM_N;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DPRJoint;
import org.ode4j.ode.DPUJoint;
import org.ode4j.ode.DSliderJoint;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;


/**
 * This program demonstrates how the PU joint works.
 * A PU joint is a combination of a Universal joint and a Slider joint.
 * It is a universal joint with a slider between the anchor point and
 * body 1.
 * 
 * 
 * The upper yellow body is fixed to the world
 * The lower yellow body is attached to the upper body by a PU joint
 * The green object is one aprt of the slider.
 * The purple object is the second part of the slider.
 * The red object represent the axis1 of the universal part.
 * The blue object represent the axis2 of the universal part.
 * The gray object represent the anchor2 of the PU joint.
 */
class DemoJointPU extends dsFunctions {


//	enum IDX_CYL_DIM
//	{
//		RADIUS,
//		LENGTH,
//		NUM_CYL_DIM
//	};
	private static final int RADIUS = 0;
	private static final int LENGTH = 1;
	//private static final int NUM_CYL_DIM = 2;


	private static final DVector3C boxDim = new DVector3(1,1,1);
	private static final DVector3C extDim = new DVector3(0.2,0.2,1.2);
	private static final DVector3C ancDim = new DVector3(0.2,0.2,0.5);
	private static final double[] axDim = {0.1,1.0};//[NUM_CYL_DIM]


	//private int type = dJointTypePU;
	private static Class<?> type = DPUJoint.class;


	private static final double VEL_INC = 0.01; // Velocity increment

	// physics parameters
	private static final double PI = 3.14159265358979323846264338327950288419716939937510;


	private static final double INT_EXT_RATIO = 0.8;

	//#define X 0
	//#define Y 1
	//#define Z 2
	private static final int X = 0;
	private static final int Y = 1;
	private static final int Z = 2;

//	private enum INDEX
//	{
//		W,// = 0,
//		D,
//		EXT,
//		INT,
//		AXIS1,
//		AXIS2,
//		ANCHOR,
//		GROUND,
//		//NUM_PARTS,
//		ALL,// = NUM_PARTS,
//		// INDEX for catBits
//		JOINT,
//		LAST_INDEX_CNT
//	};
	private static final int W = 0;
	private static final int D = 1;
	private static final int EXT = 2;
	private static final int INT = 3;
	private static final int AXIS1 = 4;
	private static final int AXIS2 = 5;
	private static final int ANCHOR = 6;
	private static final int GROUND = 7;
		//NUM_PARTS = ;
	private static final int ALL = 8;// = NUM_PARTS,
		// INDEX for catBits
	private static final int JOINT = 9;
//	private static final int LAST_INDEX_CNT = 10;

	private static final long[] catBits = //[LAST_INDEX_CNT] =
	{
			0x0001, ///< W Cylinder category
			0x0002, ///< D Cylinder category
			0x0004, ///< EXT sliderr category
			0x0008, ///< INT slider category
			0x0010, ///< AXIS1 universal category
			0x0020, ///< AXIS2 universal category
			0x0040, ///< ANCHOR category
			0x0080, ///< Ground category
			~0L,    ///< All categories
			0x0004 | 0x0008 | 0x0010 | 0x0020 ///< JOINT category
	};

	//#define Mass1 10
	//#define Mass2 8
	private static final double Mass1 = 10;
	//private static final double Mass2 = 8;


	//camera view
	private static float[] xyz = {6.0f,0.0f,6.0000f};
	private static float[] hpr = {-180.000f,-25.5000f,0.0000f};


	//world,space,body & geom
	private static DWorld           world;
	private static DSpace         space;
	private static DJointGroup    contactgroup;
	private static DBody[]            body=new DBody[ALL];//[NUM_PARTS];
	private static DGeom[]          geom=new DGeom[ALL];//[NUM_PARTS];

//	private static dJoint *joint;
	private static DJoint joint;



	final double[] BOX_SIDES  = {1.0,1.0,1.0};
	final double[] OBS_SIDES  = {0.4,0.4,0.4};
	final double[] RECT_SIDES = {0.3, 0.1, 0.2};


	private DNearCallback nearCallback = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}
	};
	
	
	//collision detection
	private void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		int i,n;

		final int N = 10;
		//dContact contact[N];
		DContactBuffer contacts = new DContactBuffer(N);
		n = dCollide (o1,o2,N,contacts.getGeomBuffer());//,sizeof (dContact) );
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

		printf ("Press '.' to increase joint velocity along the prismatic direction.\n");
		printf ("Press ',' to decrease joint velocity along the prismatic direction.\n");

		printf ("Press 'l' Toggle ON/OFF the limits on all the axis\n");
		printf ("Press 'g' Toggle ON/OFF the gravity\n");


		printf ("Press 'p' to print the position, angle and rates of the joint.\n");
	}


	// start simulation - set viewpoint
	@Override
	public void start()
	{
		//dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);

		dsSetViewpoint (xyz,hpr);
		printf ("This program demonstrates how the PU joint works.\n");
		printf ("A PU joint is a combination of a Universal joint and a Slider joint.\n");
		printf ("It is a universal joint with a slider between the anchor point and \n");
		printf ("body 1.\n\n");
		printf ("The upper yellow body is fixed to the world\n");
		printf ("The lower yellow body is attached to the upper body by a PU joint\n");
		printf ("The green object is one aprt of the slider.\n");
		printf ("The purple object is the second part of the slider.\n");
		printf ("The red object represent the axis1 of the universal part. \n");
		printf ("The blue object represent the axis2 of the universal part. \n");
		printf ("The gray object represent the anchor2 of the PU joint. \n");
		printKeyBoardShortCut();
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
			dBodyAddForce(body[D],40,0,0);
			break;
		case 'w' : case 'W' :
			dBodyAddForce(body[D],-40,0,0);
			break;

		case 'a' : case 'A' :
			dBodyAddForce(body[D],0,40,0);
			break;
		case 's' : case 'S' :
			dBodyAddForce(body[D],0,-40,0);
			break;

		case 'z' : case 'Z' :
			dBodyAddForce(body[D],0,0,40);
			break;
		case 'x' : case 'X' :
			dBodyAddForce(body[D],0,0,-40);
			break;

			// Torque
		case 'e': case 'E':
			dBodyAddTorque(body[D],0.1,0,0);
			break;
		case 'r': case 'R':
			dBodyAddTorque(body[D],-0.1,0,0);
			break;

		case 'd': case 'D':
			dBodyAddTorque(body[D],0, 0.1,0);
			break;
		case 'f': case 'F':
			dBodyAddTorque(body[D],0,-0.1,0);
			break;

		case 'c': case 'C':
			dBodyAddTorque(body[D],0,0,0.1);
			break;
		case 'v': case 'V':
			dBodyAddTorque(body[D],0,0,0.1);
			break;

			// Velocity of joint
		case ',': case '<' : {
			double vel = joint.getParam (PARAM_N.dParamVel3) - VEL_INC;
			joint.setParam (PARAM_N.dParamVel3, vel);
			joint.setParam (PARAM_N.dParamFMax3, 2);
			std_cout("Velocity = ",vel,"  FMax = 2",'\n');
		}
		break;

		case '.': case '>' : {
			double vel = joint.getParam (PARAM_N.dParamVel3) + VEL_INC;
			joint.setParam (PARAM_N.dParamVel3, vel);
			joint.setParam (PARAM_N.dParamFMax3, 2);
			std_cout("Velocity = ",vel,"  FMax = 2",'\n');
		}
		break;

		case 'l': case 'L' : {
			double aLimit, lLimit, fmax;
			if (  joint.getParam (PARAM_N.dParamFMax1)!=0 ) {
				aLimit = dInfinity;
				lLimit = dInfinity;
				fmax = 0;
			}
			else {
				aLimit = 0.25*PI;
				lLimit = 0.5*axDim[LENGTH];
				fmax = 0.02;
			}

			joint.setParam (PARAM_N.dParamFMax1, fmax);
			joint.setParam (PARAM_N.dParamFMax2, fmax);
			joint.setParam (PARAM_N.dParamFMax3, fmax);

			if (joint instanceof DPRJoint) {
				DPRJoint pr = (DPRJoint) (joint);
				pr.setParam (PARAM_N.dParamLoStop1, -lLimit);
				pr.setParam (PARAM_N.dParamHiStop1, -lLimit);
				pr.setParam (PARAM_N.dParamLoStop2, aLimit);
				pr.setParam (PARAM_N.dParamHiStop2, -aLimit);
			} else if (joint instanceof DPUJoint) {
				DPUJoint pu = (DPUJoint) (joint);
				pu.setParam (PARAM_N.dParamLoStop1, -aLimit);
				pu.setParam (PARAM_N.dParamHiStop1, aLimit);
				pu.setParam (PARAM_N.dParamLoStop2, -aLimit);
				pu.setParam (PARAM_N.dParamHiStop2, aLimit);
				pu.setParam (PARAM_N.dParamLoStop3, -lLimit);
				pu.setParam (PARAM_N.dParamHiStop3, lLimit);
			}
		}

		break;

		case 'g': case 'G' : {
			DVector3 g = new DVector3();
			world.getGravity(g);
			if (  g.get2() < -0.1 )
				world.setGravity(0, 0, 0);
			else
				world.setGravity(0, 0, -0.5);

		}

		break;

		case 'p' : case 'P' : {
			if (joint instanceof DSliderJoint) {
				DSliderJoint sj = (DSliderJoint) (joint);
				std_cout("Position =", sj.getPosition(), "\n");
			} else if (joint instanceof DPUJoint) {
				DPUJoint pu = (DPUJoint) (joint);
				std_cout("Position =",pu.getPosition() ,"\n");
				std_cout("Position Rate=",pu.getPositionRate() ,"\n");
				std_cout("Angle1 =",pu.getAngle1() ,"\n");
				std_cout("Angle1 Rate=",pu.getAngle1Rate() ,"\n");
				std_cout("Angle2 =",pu.getAngle2() ,"\n");
				std_cout("Angle2 Rate=",pu.getAngle2Rate() ,"\n");
			}
		}
		break;
		}
	}

	private static void drawBox (DBox id, int R, int G, int B)
	{
		if (id==null)
			return;

		final DVector3C pos = dGeomGetPosition (id);
		final DMatrix3C rot = dGeomGetRotation (id);
		dsSetColor (R,G,B);

		DVector3 l = new DVector3();
		dGeomBoxGetLengths (id, l);
		dsDrawBox (pos, rot, l);
	}


	private static boolean todo = false;
	private static int cnt = 0;
	// simulation loop
	private void simLoop (boolean pause)
	{
		if ( todo ) { // DEBUG
			//    static int cnt = 0;
			++cnt;

			if (cnt == 5)
				command ( 'q' );
			if (cnt == 10)
				dsStop();
		}


		if (!pause) {
			double simstep = 0.01; // 10ms simulation steps
			double dt = dsElapsedTime();

			int nrofsteps = (int) ceilf (dt/simstep);
			if (nrofsteps==0)
				nrofsteps = 1;

			for (int i=0; i<nrofsteps && !pause; i++) {
				dSpaceCollide (space,0,nearCallback);
				dWorldStep (world, simstep);

				dJointGroupEmpty (contactgroup);
			}

			update();


			RefDouble radius = new RefDouble(), length = new RefDouble();

			dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);

			drawBox ((DBox)geom[W], 1,1,0);


			drawBox ((DBox)geom[EXT], 0,1,0);

			DVector3 anchorPos = new DVector3();



			double ang1 = 0;
			double ang2 = 0;
			DVector3 axisP = new DVector3(), axisR1 = new DVector3(), axisR2 = new DVector3();

			if ( DPUJoint.class == type ) {
				DPUJoint pu = (DPUJoint) (joint);
				ang1 = pu.getAngle1();
				ang2 = pu.getAngle2();
				pu.getAxis1 (axisR1);
				pu.getAxis2 (axisR2);
				pu.getAxisP (axisP);

				dJointGetPUAnchor (pu, anchorPos);
			} else if ( DPRJoint.class == type ) {
				DPRJoint pr = (DPRJoint) (joint);
				pr.getAxis1 (axisP);
				pr.getAxis2 (axisR1);

				dJointGetPRAnchor (pr, anchorPos);
			}


			// Draw the axisR
			if ( geom[INT]!=null ) {
				dsSetColor (1,0,1);
				DVector3 l = new DVector3();
				dGeomBoxGetLengths ((DBox)geom[INT], l);

				final DMatrix3C rotBox = dGeomGetRotation (geom[W]);

				DVector3 pos = new DVector3();
//				for (int i=0; i<3; ++i)
//					pos[i] = anchorPos[i] - 0.5*extDim[Z]*axisP[i];
				pos.eqSum(anchorPos, axisP, -0.5 * extDim.get(Z));
				dsDrawBox (pos, rotBox, l);
			}

			dsSetTexture (DS_TEXTURE_NUMBER.DS_CHECKERED);
			if ( geom[AXIS1]!= null ) {
				DQuaternion q = new DQuaternion(), qAng = new DQuaternion();
				dQFromAxisAndAngle (qAng,axisR1.get(X), axisR1.get(Y), axisR1.get(Z), ang1);
				dGeomGetQuaternion (geom[AXIS1], q);

				DQuaternion qq = new DQuaternion();
				dQMultiply1 (qq, qAng, q);
				DMatrix3 R = new DMatrix3();
				dRfromQ (R,qq);


				dGeomCylinderGetParams ((DCylinder)geom[AXIS1], radius, length);
				dsSetColor (1,0,0);
				dsDrawCylinder (anchorPos, R, length.getF(), radius.getF());
			}

			if ( DPUJoint.class == type && geom[AXIS2]!=null ) {
				//dPUJoint *pu = dynamic_cast<dPUJoint *> (joint);

				DQuaternion q = new DQuaternion(), qAng = new DQuaternion();
				DQuaternion qq = new DQuaternion(), qq1 = new DQuaternion();
				dGeomGetQuaternion (geom[AXIS2], q);

				dQFromAxisAndAngle (qAng, 0, 1, 0, ang2);
				dQMultiply1 (qq, qAng, q);


				dQFromAxisAndAngle (qAng,axisR1.get(X), axisR1.get(Y), axisR1.get(Z), ang1);

				dQMultiply1 (qq1, qAng, qq);


				DMatrix3 R = new DMatrix3();
				dRfromQ (R,qq1);


				dGeomCylinderGetParams ((DCylinder)geom[AXIS2], radius, length);
				dsSetColor (0,0,1);
				dsDrawCylinder (anchorPos, R, length.getF(), radius.getF());
			}

			dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);

			// Draw the anchor
			if ( geom[ANCHOR]!=null ) {
				dsSetColor (1,1,1);
				DVector3 l = new DVector3();
				dGeomBoxGetLengths ((DBox)geom[ANCHOR], l);

				final DMatrix3C rotBox = dGeomGetRotation (geom[D]);
				final DVector3C posBox = dGeomGetPosition (geom[D]);

				DVector3 e = new DVector3();
//				for (int i=0; i<3; ++i)
//					e[i] = posBox[i] - anchorPos[i];
				e.eqDiff(posBox, anchorPos);
				dNormalize3 (e);

				DVector3 pos = new DVector3();
//				for (int i=0; i<3; ++i)
//					pos[i] = anchorPos[i] + 0.5 * l[Z]*e[i];
				pos.eqSum(anchorPos, e, 0.5*l.get(Z));
				dsDrawBox (pos, rotBox, l);
			}

			drawBox ((DBox)geom[D], 1,1,0);
		}
	}


	@Override
	public void dsPrintHelp ()
	{
		super.dsPrintHelp();
		//printf (" -h | --help   : print this help\n");
		printf (" -p | --PRJoint : Use a PR joint instead of PU joint\n");
		//printf (" -t | --texture-path path  : Path to the texture.\n");
		//printf ("                             Default = %s\n", DRAWSTUFF_TEXTURE_PATH);
		printf ("--------------------------------------------------\n");
		printf ("Hit any key to continue:");
		//getchar();

		exit (0);
	}

	public static void main(String[] args) {
		new DemoJointPU().demo(args);
	}
	
	private void demo(String[] args) {
//		if (args.length >= 2 ) {
			for (int i=0; i < args.length; ++i) {
//				if (  0 == strcmp ("-h", args[i]) || 0 == strcmp ("--help", args[i]) )
//					Help (args);

				if (  0 == strcmp ("-p", args[i]) || 0 == strcmp ("--PRJoint", args[i]) ) {
					type = DPRJoint.class;
					args[i] = "";
				}

//				if (0 == strcmp ("-t", args[i]) || 0 == strcmp ("--texture-path", args[i]) ) {
//					int j = i+1;
//					if ( j+1 > args.length      ||  // Check if we have enough arguments
//							args[j].charAt(0) == '\0' ||  // We should have a path here
//							args[j].charAt(0) == '-' ) // We should have a path not a command line
//						Help (args);
//					else
//						dsSetPathToTextures( args[++i] ); // Increase i since we use this argument
//				}
			}
//		}

		dInitODE2(0);

		world = OdeHelper.createWorld();
		world.setERP (0.8);

		space = dSimpleSpaceCreate (null);
		contactgroup = dJointGroupCreate (0);
		geom[GROUND] = dCreatePlane (space, 0,0,1,0);
		dGeomSetCategoryBits (geom[GROUND], catBits[GROUND]);
		dGeomSetCollideBits (geom[GROUND], catBits[ALL]);

		DMass m = dMassCreate();

		// Create the body attached to the World
		body[W] = OdeHelper.createBody (world);
		// Main axis of cylinder is along X=1
		m.setBox (1, boxDim.get(X), boxDim.get(Y), boxDim.get(Z));
		m.adjust (Mass1);
		geom[W] = dCreateBox (space, boxDim.get(X), boxDim.get(Y), boxDim.get(Z));
		dGeomSetBody (geom[W], body[W]);
		dGeomSetCategoryBits (geom[W], catBits[W]);
		dGeomSetCollideBits (geom[W], catBits[ALL] & (~catBits[W]) & (~catBits[JOINT]) );
		body[W].setMass(m);





		// Create the dandling body
		body[D] = OdeHelper.createBody(world);
		// Main axis of capsule is along X=1
		m.setBox (1, boxDim.get(X), boxDim.get(Y), boxDim.get(Z));
		m.adjust (Mass1);
		geom[D] = dCreateBox (space, boxDim.get(X), boxDim.get(Y), boxDim.get(Z));
		dGeomSetBody (geom[D], body[D]);
		dGeomSetCategoryBits (geom[D], catBits[D]);
		dGeomSetCollideBits (geom[D], catBits[ALL] & (~catBits[D]) & (~catBits[JOINT]) );
		body[D].setMass(m);


		// Create the external part of the slider joint
		geom[EXT] = dCreateBox (null, extDim.get(X), extDim.get(Y), extDim.get(Z));
		dGeomSetCategoryBits (geom[EXT], catBits[EXT]);
		dGeomSetCollideBits (geom[EXT],
				catBits[ALL] & (~catBits[JOINT]) & (~catBits[W]) & (~catBits[D]) );

		// Create the internal part of the slider joint
		geom[INT] = dCreateBox (null, INT_EXT_RATIO*extDim.get(X),
				INT_EXT_RATIO*extDim.get(Y),
				INT_EXT_RATIO*extDim.get(Z));
		dGeomSetCategoryBits (geom[INT], catBits[INT]);
		dGeomSetCollideBits (geom[INT],
				catBits[ALL] & (~catBits[JOINT]) & (~catBits[W]) & (~catBits[D]) );


		DMatrix3 R = new DMatrix3();
		// Create the first axis of the universal joi9nt
		//Rotation of 90deg around y
		geom[AXIS1] = dCreateCylinder(null, axDim[RADIUS], axDim[LENGTH]);
		//Rotation of 90deg around y
		dRFromAxisAndAngle (R, 0,1,0, 0.5*PI);
		dGeomSetRotation (geom[AXIS1], R);
		dGeomSetCategoryBits (geom[AXIS1], catBits[AXIS1]);
		dGeomSetCollideBits (geom[AXIS1],
				catBits[ALL]  & ~catBits[JOINT] & ~catBits[W] & ~catBits[D]);


		// Create the second axis of the universal joint
		geom[AXIS2] = dCreateCylinder(null, axDim[RADIUS], axDim[LENGTH]);
		//Rotation of 90deg around y
		dRFromAxisAndAngle (R, 1,0,0, 0.5*PI);
		dGeomSetRotation (geom[AXIS2], R);
		dGeomSetCategoryBits (geom[AXIS2], catBits[AXIS2]);
		dGeomSetCollideBits (geom[AXIS2],
				catBits[ALL]  & ~catBits[JOINT] & ~catBits[W] & ~catBits[D]);


		// Create the anchor
		geom[ANCHOR] = dCreateBox (null, ancDim.get(X), ancDim.get(Y), ancDim.get(Z));
		dGeomSetCategoryBits (geom[ANCHOR], catBits[ANCHOR]);
		dGeomSetCollideBits (geom[ANCHOR],
				catBits[ALL] & (~catBits[JOINT]) & (~catBits[W]) & (~catBits[D]) );


		if (body[W]!=null) {
			body[W].setPosition(0, 0, 5);
		}


		if (geom[EXT]!=null) {
			dGeomSetPosition (geom[EXT], 0,0,3.8);
		}
		if (geom[INT]!=null) {
			dGeomSetPosition (geom[INT], 0,0,2.6);
		}
		if (geom[AXIS1]!=null) {
			dGeomSetPosition (geom[AXIS1], 0,0,2.5);
		}
		if (geom[AXIS2]!=null) {
			dGeomSetPosition (geom[AXIS2], 0,0,2.5);
		}

		if (geom[ANCHOR]!=null) {
			dGeomSetPosition (geom[ANCHOR], 0,0,2.25);
		}

		if (body[D]!=null) {
			body[D].setPosition(0,0,1.5);
		}


		// Attache the upper box to the world
		DFixedJoint fixed = dJointCreateFixed (world,null);
		dJointAttach (fixed , null, body[W]);
		dJointSetFixed (fixed );

		if (type == DPRJoint.class) {
			DPRJoint pr = OdeHelper.createPRJoint(world, null);
			pr.attach (body[W], body[D]);
			pr.setAxis1 (0, 0, -1);
			pr.setAxis2 (1, 0, 0);
			joint = pr;

			dJointSetPRAnchor (pr, 0, 0, 2.5);
		} else {
			DPUJoint pu = OdeHelper.createPUJoint(world, null);
			pu.attach (body[W], body[D]);
			pu.setAxis1 (1, 0, 0);
			pu.setAxis2 (0, 1, 0);
			pu.setAxisP (0, 0, -1);
			joint = pu;

			dJointSetPUAnchor (pu, 0, 0, 2.5);
		}


		// run simulation
		dsSimulationLoop (args,400,300,this);

		//delete joint;
		//joint.DESTRUCTOR();  //TZ, not necessary, is deleted from dWorldDestroy()
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

