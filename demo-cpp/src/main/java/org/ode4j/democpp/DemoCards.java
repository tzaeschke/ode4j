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

import java.util.ArrayList;

import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.DGeom.DNearCallback;

import static org.ode4j.cpp.OdeCpp.*;
import static org.ode4j.drawstuff.DrawStuff.*;
import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.cpp4j.C_All.*;

/**
 *
 */
public class DemoCards extends dsFunctions {

	private static int levels = 5;
	private static int ncards = 0;

	private static DSpace space;
	private static DWorld world;
	private static DJointGroup contactgroup;

	private static class Card {
		DBody body;
		DGeom geom;
		//    static const dReal sides[3];

		Card()
		{
			body = dBodyCreate(world);
			geom = dCreateBox(space, sides.get0(), sides.get1(), sides.get2());
			dGeomSetBody(geom, body);
			dGeomSetData(geom, this);
			DMass mass = OdeHelper.createMass();
			mass.setBox(1, sides.get0(), sides.get1(), sides.get2());
			dBodySetMass(body, mass);
		}

		//~Card()
		public void DESTRUCTOR() {
			dBodyDestroy(body);
			dGeomDestroy(geom);
		}

		void draw() //const
		{
			dsDrawBox(dBodyGetPosition(body),
					dBodyGetRotation(body), sides);
		}
	};

	private static final double cwidth=.5, cthikness=.02, clength=1;
	//const dReal Card::sides[3] = { cwidth, cthikness, clength };
	private static final DVector3C sides = new DVector3( cwidth, cthikness, clength ); 


	//std::vector<Card*> cards;
	private final ArrayList<Card> cards = new ArrayList<Card>();

	private int getncards(int levels)
	{
		return (3*levels*levels + levels) / 2;
	}

	private void place_cards()
	{
		ncards = getncards(levels);
		// destroy removed cards (if any)
		int oldcards = cards.size();
		for (int i=ncards; i<oldcards; ++i) {
			//delete cards[i];
			cards.remove(cards.size()-1).DESTRUCTOR();
		}

		//cards.resize(ncards);
		// construct new cards (if any)
		for (int i=oldcards; i<ncards; ++i)
			cards.add(new Card() );

		// for each level
		int c = 0;
		DMatrix3 right = new DMatrix3(), left = new DMatrix3(), hrot = new DMatrix3();
		double angle = 20*M_PI/180.;
		dRFromAxisAndAngle(right, 1, 0, 0, -angle);
		dRFromAxisAndAngle(left, 1, 0, 0, angle);

		dRFromAxisAndAngle(hrot, 1, 0, 0, 91*M_PI/180.);

		double eps = 0.05;
		double vstep = cos(angle)*clength + eps;
		double hstep = sin(angle)*clength + eps;

		for (int lvl=0; lvl<levels; ++lvl) {
			// there are 3*(levels-lvl)-1 cards in each level, except last
			int n = (levels-lvl);
			double height = (lvl)*vstep + vstep/2;
			// inclined cards
			for (int i=0; i<2*n; ++i, ++c) {
				dBodySetPosition(cards.get(c).body, 
						0,
						-n*hstep + hstep*i,
						height
				);
				if (i%2 != 0)
					dBodySetRotation(cards.get(c).body, left);
				else
					dBodySetRotation(cards.get(c).body, right);
			}

			if (n==1) // top of the house
				break;

			// horizontal cards
			for (int i=0; i<n-1; ++i, ++c) {
				dBodySetPosition(cards.get(c).body,
						0,
						-(n-1 - (clength-hstep)/2)*hstep + 2*hstep*i,
						height + vstep/2);
				dBodySetRotation(cards.get(c).body, hrot);
			}
		}

	}


	@Override
	public void start()
	{
		System.out.println("Controls:");
		System.out.println("   SPACE - reposition cards");
		System.out.println("   -     - one less level");
		System.out.println("   =     - one more level");
	}

	private static DNearCallback nearCallback = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}
	};
	
	private static void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		// exit without doing anything if the two bodies are connected by a joint
		DBody b1 = dGeomGetBody(o1);
		DBody b2 = dGeomGetBody(o2);

		final int MAX_CONTACTS = 8;
		//DContact contact[MAX_CONTACTS];
		DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);

		int numc = dCollide (o1, o2, MAX_CONTACTS,
				contacts.getGeomBuffer()
				);//sizeof(dContact));

		for (int i=0; i<numc; i++) {
			contacts.get(i).surface.mode = dContactApprox1;
			contacts.get(i).surface.mu = 5;
			DJoint c = dJointCreateContact (world, contactgroup, contacts.get(i));//contact+i);
			dJointAttach (c, b1, b2);
		}
	}
	
	private void simLoop(boolean pause)
	{
		if (!pause) {
			dSpaceCollide (space, 0, nearCallback);
			dWorldQuickStep(world, 0.01);
			dJointGroupEmpty(contactgroup);
		}

		dsSetColor (1,1,0);
		for (int i=0; i<ncards; ++i) {
			dsSetColor (1, ((float)i)/ncards, 0);
			cards.get(i).draw();
		}

	}

	@Override
	public void command(char c)
	{
		switch (c) {
		case '=':
			levels++;
			place_cards();
			break;
		case '-':
			levels--;
			if (levels <= 0)
				levels++;
			place_cards();
			break;
		case ' ':
			place_cards();
			break;
		}
	}

	/**
	 * @param args
	 */
	public static void main(final String[] args)
	{
		new DemoCards().demo(args);
	}

	private void demo(String[] args) {
		dInitODE2(0);

		// setup pointers to drawstuff callback functions
		//dsFunctions fn = this;
		//fn.version = DS_VERSION;
		//    fn.start = &start;
		//    fn.step = &simLoop;
		//    fn.command = &command;
		//    fn.stop = 0;
		//fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;


		world = dWorldCreate();
		dWorldSetGravity(world, 0, 0, -0.5);
		dWorldSetQuickStepNumIterations(world, 50); // <-- increase for more stability

		space = dSimpleSpaceCreate(null);
		contactgroup = dJointGroupCreate(0);
		DGeom ground = dCreatePlane(space, 0, 0, 1, 0);

		place_cards();

		// run simulation
		dsSimulationLoop (args, 640, 480, this);

		levels = 0;
		place_cards();

		dJointGroupDestroy(contactgroup);
		dWorldDestroy(world);
		dGeomDestroy(ground);
		dSpaceDestroy(space);

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
