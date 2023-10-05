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

import org.ode4j.drawstuff.DrawStuff.DS_TEXTURE_NUMBER;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
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
import org.ode4j.ode.DPlane;
import org.ode4j.ode.DSliderJoint;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;


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
public class DemoJointPU extends dsFunctions {


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
	//private static DBody[]            body=new DBody[ALL];//[NUM_PARTS];
	private static DBody bodyD;
	private static DBody bodyW;
	//private static DGeom[]          geom=new DGeom[ALL];//[NUM_PARTS];
	private static DBox geomD;
	private static DBox geomW;
	private static DBox geomEXT;
	private static DBox geomINT;
	private static DBox geomANCHOR;
	private static DCylinder geomAXIS1;
	private static DCylinder geomAXIS2;
	private static DPlane geomGROUND;
	

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
	private void nearCallback (Object data, DGeom o1, DGeom o2)	{
		int i,n;

		final int N = 10;
		//dContact contact[N];
		DContactBuffer contacts = new DContactBuffer(N);
		n = OdeHelper.collide (o1,o2,N,contacts.getGeomBuffer());//,sizeof (dContact) );
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
				DJoint c = OdeHelper.createContactJoint (world,contactgroup,contact);
				c.attach (contact.geom.g1.getBody(), contact.geom.g2.getBody() );
			}
		}
	}

	private void printKeyBoardShortCut() {
		System.out.println ("Press 'h' for this help.");
		System.out.println ("Press 'q' to add force on BLUE body along positive x direction.");
		System.out.println ("Press 'w' to add force on BLUE body along negative x direction.");

		System.out.println ("Press 'a' to add force on BLUE body along positive y direction.");
		System.out.println ("Press 's' to add force on BLUE body along negative y direction.");

		System.out.println ("Press 'z' to add force on BLUE body along positive z direction.");
		System.out.println ("Press 'x' to add force on BLUE body along negative z direction.");

		System.out.println ("Press 'e' to add torque on BLUE body around positive x direction ");
		System.out.println ("Press 'r' to add torque on BLUE body around negative x direction ");

		System.out.println ("Press 'd' to add torque on BLUE body around positive y direction ");
		System.out.println ("Press 'f' to add torque on BLUE body around negative y direction ");

		System.out.println ("Press 'c' to add torque on BLUE body around positive z direction ");
		System.out.println ("Press 'v' to add torque on BLUE body around negative z direction ");

		System.out.println ("Press '.' to increase joint velocity along the prismatic direction.");
		System.out.println ("Press ',' to decrease joint velocity along the prismatic direction.");

		System.out.println ("Press 'l' Toggle ON/OFF the limits on all the axis");
		System.out.println ("Press 'g' Toggle ON/OFF the gravity");


		System.out.println ("Press 'p' to print the position, angle and rates of the joint.");
	}


	// start simulation - set viewpoint
	@Override
	public void start()	{
		dsSetViewpoint (xyz,hpr);
		System.out.println ("This program demonstrates how the PU joint works.");
		System.out.println ("A PU joint is a combination of a Universal joint and a Slider joint.");
		System.out.println ("It is a universal joint with a slider between the anchor point and ");
		System.out.println ("body 1.");
		System.out.println ();
		System.out.println ("The upper yellow body is fixed to the world.");
		System.out.println ("The lower yellow body is attached to the upper body by a PU joint.");
		System.out.println ("The green object is one aprt of the slider.");
		System.out.println ("The purple object is the second part of the slider.");
		System.out.println ("The red object represent the axis1 of the universal part.");
		System.out.println ("The blue object represent the axis2 of the universal part.");
		System.out.println ("The gray object represent the anchor2 of the PU joint.");
		printKeyBoardShortCut();
	}

	// called when a key pressed
	@Override
	public void command (char cmd) {
		switch (cmd) {
		case 'h' : case 'H' : case '?' :
			printKeyBoardShortCut();
			break;

			// Force
		case 'q' : case 'Q' :
			bodyD.addForce(40,0,0);
			break;
		case 'w' : case 'W' :
			bodyD.addForce(-40,0,0);
			break;

		case 'a' : case 'A' :
			bodyD.addForce(0,40,0);
			break;
		case 's' : case 'S' :
			bodyD.addForce(0,-40,0);
			break;

		case 'z' : case 'Z' :
			bodyD.addForce(0,0,40);
			break;
		case 'x' : case 'X' :
			bodyD.addForce(0,0,-40);
			break;

			// Torque
		case 'e': case 'E':
			bodyD.addTorque(0.1,0,0);
			break;
		case 'r': case 'R':
			bodyD.addTorque(-0.1,0,0);
			break;

		case 'd': case 'D':
			bodyD.addTorque(0, 0.1,0);
			break;
		case 'f': case 'F':
			bodyD.addTorque(0,-0.1,0);
			break;

		case 'c': case 'C':
			bodyD.addTorque(0,0,0.1);
			break;
		case 'v': case 'V':
			bodyD.addTorque(0,0,0.1);
			break;

			// Velocity of joint
		case ',': case '<' : {
			double vel = joint.getParam (PARAM_N.dParamVel3) - VEL_INC;
			joint.setParam (PARAM_N.dParamVel3, vel);
			joint.setParam(PARAM_N.dParamFMax3, 2);
			System.out.println("Velocity = " + vel + "  FMax = 2");
		}
		break;

		case '.': case '>' : {
			double vel = joint.getParam (PARAM_N.dParamVel3) + VEL_INC;
			joint.setParam (PARAM_N.dParamVel3, vel);
			joint.setParam(PARAM_N.dParamFMax3, 2);
			System.out.println("Velocity = " + vel + "  FMax = 2");
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
				System.out.println("Position =" + sj.getPosition());
			} else if (joint instanceof DPUJoint) {
				DPUJoint pu = (DPUJoint) (joint);
				System.out.println("Position =" + pu.getPosition());
				System.out.println("Position Rate=" + pu.getPositionRate());
				System.out.println("Angle1 =" + pu.getAngle1());
				System.out.println("Angle1 Rate=" + pu.getAngle1Rate());
				System.out.println("Angle2 =" + pu.getAngle2());
				System.out.println("Angle2 Rate=" + pu.getAngle2Rate());
			}
		}
		break;
		}
	}

	private void drawBox (DBox id, int R, int G, int B)	{
		if (id==null)
			return;

		DVector3C pos = id.getPosition ();
		DMatrix3C rot = id.getRotation ();
		dsSetColor (R,G,B);

		dsDrawBox (pos, rot, id.getLengths());
	}


	private static boolean todo = false;
	private static int cnt = 0;
	
	// simulation loop
	@Override
	public void step (boolean pause) {
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

			int nrofsteps = (int) Math.ceil(dt/simstep);
			if (nrofsteps==0)
				nrofsteps = 1;

			for (int i=0; i<nrofsteps && !pause; i++) {
				OdeHelper.spaceCollide (space,0,nearCallback);
				world.step (simstep);

				contactgroup.empty ();
			}

			dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);

			drawBox (geomW, 1,1,0);


			drawBox (geomEXT, 0,1,0);

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

				pu.getAnchor (anchorPos);
			} else if ( DPRJoint.class == type ) {
				DPRJoint pr = (DPRJoint) (joint);
				pr.getAxis1 (axisP);
				pr.getAxis2 (axisR1);

				pr.getAnchor (anchorPos);
			}


			// Draw the axisR
			if ( geomINT!=null ) {
				dsSetColor (1,0,1);
				DVector3C l = geomINT.getLengths();

				DMatrix3C rotBox = geomW.getRotation();

				DVector3 pos = new DVector3();
//				for (int i=0; i<3; ++i)
//					pos[i] = anchorPos[i] - 0.5*extDim[Z]*axisP[i];
				pos.eqSum(anchorPos, axisP, -0.5 * extDim.get(Z));
				dsDrawBox (pos, rotBox, l);
			}

			dsSetTexture (DS_TEXTURE_NUMBER.DS_CHECKERED);
			if ( geomAXIS1!= null ) {
				DQuaternion qAng = new DQuaternion();
				dQFromAxisAndAngle (qAng,axisR1.get(X), axisR1.get(Y), axisR1.get(Z), ang1);
				DQuaternionC q = geomAXIS1.getQuaternion ();

				DQuaternion qq = new DQuaternion();
				dQMultiply1 (qq, qAng, q);
				DMatrix3 R = new DMatrix3();
				dRfromQ (R,qq);

				dsSetColor (1,0,0);
				dsDrawCylinder (anchorPos, R, geomAXIS1.getLength(), geomAXIS1.getRadius());
			}

			if ( DPUJoint.class == type && geomAXIS2!=null ) {
				DQuaternion qAng = new DQuaternion();
				DQuaternion qq = new DQuaternion(), qq1 = new DQuaternion();
				DQuaternionC q = geomAXIS2.getQuaternion();

				dQFromAxisAndAngle (qAng, 0, 1, 0, ang2);
				dQMultiply1 (qq, qAng, q);


				dQFromAxisAndAngle (qAng,axisR1.get(X), axisR1.get(Y), axisR1.get(Z), ang1);

				dQMultiply1 (qq1, qAng, qq);


				DMatrix3 R = new DMatrix3();
				dRfromQ (R,qq1);

				dsSetColor (0,0,1);
				dsDrawCylinder (anchorPos, R, geomAXIS2.getLength(), geomAXIS2.getRadius());
			}

			dsSetTexture (DS_TEXTURE_NUMBER.DS_WOOD);

			// Draw the anchor
			if ( geomANCHOR!=null ) {
				dsSetColor (1,1,1);
				DVector3C l = geomANCHOR.getLengths();

				DMatrix3C rotBox = geomD.getRotation();
				DVector3C posBox = geomD.getPosition();

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

			drawBox (geomD, 1,1,0);
		}
	}


	@Override
	public void dsPrintHelp()
	{
		super.dsPrintHelp();
		System.out.println (" -p | --PRJoint : Use a PR joint instead of PU joint");
		System.out.println ("--------------------------------------------------");
		System.out.println ("Hit any key to continue:");
		//getchar();

		System.exit (0);
	}

	public static void main(String[] args) {
		new DemoJointPU().demo(args);
	}
	
	private void demo(String[] args) {
		for (int i=0; i < args.length; ++i) {
			if ( "-p".equals(args[i]) || "--PRJoint".equals(args[i]) ) {
				type = DPRJoint.class;
				args[i] = "";
			}
		}

		OdeHelper.initODE2(0);

		world = OdeHelper.createWorld();
		world.setERP (0.8);

		space = OdeHelper.createSimpleSpace (null);
		contactgroup = OdeHelper.createJointGroup ();
		geomGROUND = OdeHelper.createPlane (space, 0,0,1,0);
		geomGROUND.setCategoryBits (catBits[GROUND]);
		geomGROUND.setCollideBits (catBits[ALL]);

		DMass m = OdeHelper.createMass();

		// Create the body attached to the World
		bodyW = OdeHelper.createBody (world);
		// Main axis of cylinder is along X=1
		m.setBox (1, boxDim.get(X), boxDim.get(Y), boxDim.get(Z));
		m.adjust (Mass1);
		geomW = OdeHelper.createBox (space, boxDim.get(X), boxDim.get(Y), boxDim.get(Z));
		geomW.setBody (bodyW);
		geomW.setCategoryBits (catBits[W]);
		geomW.setCollideBits (catBits[ALL] & (~catBits[W]) & (~catBits[JOINT]) );
		bodyW.setMass(m);





		// Create the dandling body
		bodyD = OdeHelper.createBody(world);
		// Main axis of capsule is along X=1
		m.setBox (1, boxDim.get(X), boxDim.get(Y), boxDim.get(Z));
		m.adjust (Mass1);
		geomD = OdeHelper.createBox (space, boxDim.get(X), boxDim.get(Y), boxDim.get(Z));
		geomD.setBody (bodyD);
		geomD.setCategoryBits (catBits[D]);
		geomD.setCollideBits (catBits[ALL] & (~catBits[D]) & (~catBits[JOINT]) );
		bodyD.setMass(m);


		// Create the external part of the slider joint
		geomEXT = OdeHelper.createBox (null, extDim.get(X), extDim.get(Y), extDim.get(Z));
		geomEXT.setCategoryBits (catBits[EXT]);
		geomEXT.setCollideBits (
				catBits[ALL] & (~catBits[JOINT]) & (~catBits[W]) & (~catBits[D]) );

		// Create the internal part of the slider joint
		geomINT = OdeHelper.createBox (null, INT_EXT_RATIO*extDim.get(X),
				INT_EXT_RATIO*extDim.get(Y),
				INT_EXT_RATIO*extDim.get(Z));
		geomINT.setCategoryBits (catBits[INT]);
		geomINT.setCollideBits (
				catBits[ALL] & (~catBits[JOINT]) & (~catBits[W]) & (~catBits[D]) );


		DMatrix3 R = new DMatrix3();
		// Create the first axis of the universal joint
		geomAXIS1 = OdeHelper.createCylinder(null, axDim[RADIUS], axDim[LENGTH]);
		//Rotation of 90deg around y
		dRFromAxisAndAngle (R, 0,1,0, 0.5*PI);
		geomAXIS1.setRotation (R);
		geomAXIS1.setCategoryBits (catBits[AXIS1]);
		geomAXIS1.setCollideBits (
				catBits[ALL]  & ~catBits[JOINT] & ~catBits[W] & ~catBits[D]);


		// Create the second axis of the universal joint
		geomAXIS2 = OdeHelper.createCylinder(null,  axDim[RADIUS], axDim[LENGTH]);
		//Rotation of 90deg around y
		dRFromAxisAndAngle (R, 1,0,0, 0.5*PI);
		geomAXIS2.setRotation (R);
		geomAXIS2.setCategoryBits (catBits[AXIS2]);
		geomAXIS2.setCollideBits (
				catBits[ALL]  & ~catBits[JOINT] & ~catBits[W] & ~catBits[D]);


		// Create the anchor
		geomANCHOR = OdeHelper.createBox (null, ancDim.get(X), ancDim.get(Y), ancDim.get(Z));
		geomANCHOR.setCategoryBits (catBits[ANCHOR]);
		geomANCHOR.setCollideBits (
				catBits[ALL] & (~catBits[JOINT]) & (~catBits[W]) & (~catBits[D]) );


		if (bodyW!=null) {
			bodyW.setPosition(0, 0, 5);
		}


		if (geomEXT!=null) {
			geomEXT.setPosition (0,0,3.8);
		}
		if (geomINT!=null) {
			geomINT.setPosition (0,0,2.6);
		}
		if (geomAXIS1!=null) {
			geomAXIS1.setPosition (0,0,2.5);
		}
		if (geomAXIS2!=null) {
			geomAXIS2.setPosition (0,0,2.5);
		}

		if (geomANCHOR!=null) {
			geomANCHOR.setPosition (0,0,2.25);
		}

		if (bodyD!=null) {
			bodyD.setPosition(0,0,1.5);
		}


		// Attache the upper box to the world
		DFixedJoint fixed = OdeHelper.createFixedJoint (world,null);
		fixed.attach (null, bodyW);
		fixed.setFixed ();

		if (type == DPRJoint.class) {
			DPRJoint pr = OdeHelper.createPRJoint(world, null);
			pr.attach (bodyW, bodyD);
			pr.setAxis1 (0, 0, -1);
			pr.setAxis2 (1, 0, 0);
			joint = pr;

			pr.setAnchor (0, 0, 2.5);
		} else {
			DPUJoint pu = OdeHelper.createPUJoint(world, null);
			pu.attach (bodyW, bodyD);
			pu.setAxis1 (1, 0, 0);
			pu.setAxis2 (0, 1, 0);
			pu.setAxisP (0, 0, -1);
			joint = pu;

			pu.setAnchor (0, 0, 2.5);
		}


		// run simulation
		dsSimulationLoop (args,640,480,this);

		//delete joint;
		//joint.DESTRUCTOR();  //TZ, not necessary, is deleted from dWorldDestroy()
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

