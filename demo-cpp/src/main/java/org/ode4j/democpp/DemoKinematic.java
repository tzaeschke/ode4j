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

import java.util.HashSet;

import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DContactJoint;
import org.ode4j.ode.DCylinder;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DHingeJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DPlane;
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
public class DemoKinematic extends dsFunctions {

	private DWorld world;
	private DSpace space;
	private DPlane ground;
	private DBody kbody;
	private DBox kbox;
	private DJointGroup joints;
	private DCylinder kpole;
	private DBody matraca;
	private DBox matraca_geom;
	private DHingeJoint hinge;

	private class Box {
	    DBody body;
	    DBox geom;
	    Box()// :
//	        body(world),
//	        geom(space, 0.2, 0.2, 0.2)
	    {
	    	body = OdeHelper.createBody(world);
	        geom = OdeHelper.createBox(space, 0.2, 0.2, 0.2);
	        DMass mass = OdeHelper.createMass();
	        mass.setBox(10, 0.2, 0.2, 0.2);
	        body.setMass(mass);
	        geom.setData(this);
	        geom.setBody(body);
	    }
	    void draw() //const
	    {
	        DVector3 lengths = new DVector3();
	        geom.getLengths(lengths);
	        dsSetTexture(DS_TEXTURE_NUMBER.DS_WOOD);
	        dsSetColor(0,1,0);
	        dsDrawBox(geom.getPosition(), geom.getRotation(), lengths);
	    }
		public void DESTRUCTOR() {
			// Nothing
		}
	};

//	set<Box*> boxes;
//	set<Box*> to_remove;
	private static final HashSet<Box> boxes = new HashSet<Box>();
	private static final HashSet<Box> to_remove = new HashSet<Box>();

	private void dropBox()
	{
	    Box box = new Box();
	    
	    double px = (rand() / ((float)RAND_MAX)) * 2 - 1;
	    double py = (rand() / ((float)RAND_MAX)) * 2 - 1;
	    double pz = 2.5;
	    box.body.setPosition(px, py, pz);
	    
	    boxes.add(box);
	}

	private void queueRemoval(DGeom g)
	{
	    Box b = (Box)dGeomGetData(g);
	    to_remove.add(b);
	}

	private void removeQueued()
	{
		for (Box b: to_remove) {
			boxes.remove(b);
			b.DESTRUCTOR();
		}
		to_remove.clear();
//	    while (!to_remove.isEmpty()) {
//	        Box b = to_remove.begin();
//	        to_remove.erase(b);
//	        boxes.erase(b);
//	        delete b;
//	    }
	}

	private DNearCallback nearCallback = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}
	};
		
	private void nearCallback(Object data, DGeom g1, DGeom g2)
	{
	    if (g1 == ground) {
	        queueRemoval(g2);
	        return;
	    }
	    if (g2 == ground) {
	        queueRemoval(g1);
	        return;
	    }

	    DBody b1 = dGeomGetBody(g1);
	    DBody b2 = dGeomGetBody(g2);
	    
	    if (b1!=null && b2!=null && dAreConnectedExcluding(b1, b2, DContactJoint.class))
	        return;

	    final int MAX_CONTACTS = 10;
	    //DContact contact[MAX_CONTACTS];
	    DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);
	    int n = dCollide(g1, g2, MAX_CONTACTS, contacts.getGeomBuffer());//, sizeof(dContact));
	    for (int i=0; i<n; ++i) {
	        contacts.get(i).surface.mode = 0;
	        contacts.get(i).surface.mu = 1;
	        DJoint j = dJointCreateContact (world, joints, contacts.get(i));
	        dJointAttach(j, b1, b2);
	    }
	}

    private static float t=0;
	private void simLoop(boolean pause)
	{
	    if (!pause) {
	        final double timestep = 0.04;

	        // this does a hard-coded circular motion animation
//	        static float t=0;
	        t += timestep/4;
	        if (t > 2*M_PI)
	            t = 0;
	        DVector3 next_pos = new DVector3( Math.cos(t), Math.sin(t), 0.5 );
	        DVector3 vel = new DVector3();
	        // vel = (next_pos - cur_pos) / timestep
	        vel.eqDiff(next_pos, kbody.getPosition());
	        vel.scale(1/timestep);
	        kbody.setLinearVel(vel);
	        // end of hard-coded animation
	        
	        space.collide(0, nearCallback);
	        removeQueued();
	        
	        world.quickStep(timestep);
	        joints.clear();
	    }

	    DVector3 lengths = new DVector3();

	    // the moving platform
	    kbox.getLengths(lengths);
	    dsSetTexture(DS_TEXTURE_NUMBER.DS_WOOD);
	    dsSetColor(.3f, .3f, 1f);
	    dsDrawBox(kbox.getPosition(), kbox.getRotation(), lengths);
	    double length, radius;
//	    kpole.getParams(&radius, &length);
	    radius = kpole.getRadius();
	    length = kpole.getLength();
	    dsSetTexture(DS_TEXTURE_NUMBER.DS_CHECKERED);
	    dsSetColor(1, 1, 0);
	    dsDrawCylinder(kpole.getPosition(), kpole.getRotation(), length, radius);
	    
	    // the matraca
	    matraca_geom.getLengths(lengths);
	    dsSetColor(1,0,0);
	    dsSetTexture(DS_TEXTURE_NUMBER.DS_WOOD);
	    dsDrawBox(matraca_geom.getPosition(), matraca_geom.getRotation(), lengths);

	    // and the boxes
	    //for_each(boxes.begin(), boxes.end(), mem_fun(&Box::draw));
	    for (Box b: boxes) b.draw();
	}

	@Override
	public void command(char c)
	{
	    switch (c) {
	        case ' ':
	            dropBox();
	            break;
	    }
	}

	public static void main(String[] args) {
		new DemoKinematic().demo(args);
	}
	
	private void demo(String[] args)
	{
	    dInitODE2(0);

	    // setup pointers to drawstuff callback functions
//	    dsFunctions fn = this;
//	    fn.version = DS_VERSION;
//	    fn.start = 0;
//	    fn.step = &simLoop;
//	    fn.command = &command;
//	    fn.stop = 0;
//	    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
	    
	    System.out.println("*** Press SPACE to drop boxes **");
	    
	    space = OdeHelper.createSimpleSpace();
	    joints = OdeHelper.createJointGroup();
	    ground = OdeHelper.createPlane(space, 0, 0, 1, 0);
	    
	    world = OdeHelper.createWorld();
	    world.setGravity(0, 0, -.5);
	    
	    kbody = OdeHelper.createBody(world);
	    kbody.setKinematic();
	    final double kx = 1, ky = 0, kz = .5;
	    kbody.setPosition(kx, ky, kz);
	    kbox = OdeHelper.createBox(space, 3, 3, .5);
	    kbox.setBody(kbody);
	    kpole = OdeHelper.createCylinder(space, .125, 1.5);
	    kpole.setBody(kbody);
	    dGeomSetOffsetPosition(kpole, 0, 0, 0.8);
	    
	    matraca = OdeHelper.createBody(world);
	    matraca.setPosition(kx+0, ky+1, kz+1);
	    matraca_geom = OdeHelper.createBox(space, 0.5, 2, 0.75);
	    matraca_geom.setBody(matraca);
	    DMass mass = OdeHelper.createMass();
	    mass.setBox(1, 0.5, 2, 0.75);
	    matraca.setMass(mass);
	    
	    hinge = OdeHelper.createHingeJoint(world);
	    hinge.attach(kbody, matraca);
	    hinge.setAnchor(kx, ky, kz+1);
	    hinge.setAxis(0, 0, 1);
	    
	    dsSimulationLoop (args, 640, 480, this);
	    
	    dCloseODE();
	}

	@Override
	public void start() {
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
