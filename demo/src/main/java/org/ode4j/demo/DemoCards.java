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
import static org.ode4j.drawstuff.DrawStuff.dsSimulationLoop;
import static org.ode4j.ode.DRotation.dRFromAxisAndAngle;
import static org.ode4j.ode.OdeConstants.dContactApprox1;

import java.util.ArrayList;

import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;


/**
 *
 * @author Tilmann Zaeschke
 */
public class DemoCards extends dsFunctions {

	private static int levels = 5;
	private static int ncards = 0;

	private static DSpace space;
	private static DWorld world;
	private static DJointGroup contactgroup;

	private static final double cwidth=.5, cthikness=.02, clength=1;
	private static final DVector3C sides = new DVector3( cwidth, cthikness, clength ); 
	
	private static class Card {
		DBody body;
		DGeom geom;
		//    static const dReal sides[3];

		Card()
		{
			body = OdeHelper.createBody(world);
			geom = OdeHelper.createBox(space, sides.get0(), sides.get1(), sides.get2());
			geom.setBody(body);
			geom.setData(this);
			DMass mass = OdeHelper.createMass();
			mass.setBox(1, sides.get0(), sides.get1(), sides.get2());
			body.setMass(mass);
		}

		//~Card()
		public void DESTRUCTOR() {
			body.destroy();
			geom.destroy();
		}

		void draw()
		{
			dsDrawBox(body.getPosition(), body.getRotation(), sides);
		}
	}


	private final ArrayList<Card> cards = new ArrayList<>();

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
		double angle = 20*OdeMath.M_PI/180.;
		dRFromAxisAndAngle(right, 1, 0, 0, -angle);
		dRFromAxisAndAngle(left, 1, 0, 0, angle);

		dRFromAxisAndAngle(hrot, 1, 0, 0, 91* OdeMath.M_PI/180.);

		double eps = 0.05;
		double vstep = Math.cos(angle)*clength + eps;
		double hstep = Math.sin(angle)*clength + eps;

		for (int lvl=0; lvl<levels; ++lvl) {
			// there are 3*(levels-lvl)-1 cards in each level, except last
			int n = (levels-lvl);
			double height = (lvl)*vstep + vstep/2;
			// inclined cards
			for (int i=0; i<2*n; ++i, ++c) {
				cards.get(c).body.setPosition( 
						0,
						-n*hstep + hstep*i,
						height
				);
				if (i%2 != 0)
					cards.get(c).body.setRotation(left);
				else
					cards.get(c).body.setRotation(right);
			}

			if (n==1) // top of the house
				break;

			// horizontal cards
			for (int i=0; i<n-1; ++i, ++c) {
				cards.get(c).body.setPosition(
						0,
						-(n-1 - (clength-hstep)/2)*hstep + 2*hstep*i,
						height + vstep/2);
				cards.get(c).body.setRotation(hrot);
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

	private static void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		// exit without doing anything if the two bodies are connected by a joint
		DBody b1 = o1.getBody();
		DBody b2 = o2.getBody();

		final int MAX_CONTACTS = 8;
		//DContact contact[MAX_CONTACTS];
		DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);

		int numc = OdeHelper.collide (o1, o2, MAX_CONTACTS,
				contacts.getGeomBuffer());

		for (int i=0; i<numc; i++) {
			contacts.get(i).surface.mode = dContactApprox1;
			contacts.get(i).surface.mu = 5;
			DJoint c = OdeHelper.createContactJoint (world, contactgroup, contacts.get(i));//contact+i);
			c.attach (b1, b2);
		}
	}
	
	@Override
	public void step(boolean pause)
	{
		if (!pause) {
			space.collide(null, DemoCards::nearCallback);
			world.quickStep(0.01);
			contactgroup.empty();
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
	 * @param args args
	 */
	public static void main(final String[] args)
	{
		new DemoCards().demo(args);
	}

	private void demo(String[] args) {
		OdeHelper.initODE2(0);

		world = OdeHelper.createWorld();
		world.setGravity(0, 0, -0.5);
		world.setQuickStepNumIterations(50); // <-- increase for more stability

		space = OdeHelper.createSimpleSpace(null);
		//space = OdeHelper.createSapSpace(AXES.XYZ);
		//space = OdeHelper.createBHVSpace(0);
		contactgroup = OdeHelper.createJointGroup();
		DGeom ground = OdeHelper.createPlane(space, 0, 0, 1, 0);

		place_cards();

		// run simulation
		dsSimulationLoop (args, 640, 480, this);

		levels = 0;
		place_cards();

		contactgroup.destroy();
		world.destroy();
		ground.destroy();
		space.destroy();

		OdeHelper.closeODE();
	}


	@Override
	public void stop() {
		// Nothing
	}
}
