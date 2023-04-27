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
import org.ode4j.ode.DAABBC;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DCapsule;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DContactJoint;
import org.ode4j.ode.DConvex;
import org.ode4j.ode.DCylinder;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.DTriMesh;
import org.ode4j.ode.DTriMeshData;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.DGeom.DNearCallback;

import static org.ode4j.ode.OdeHelper.*;
import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.drawstuff.DrawStuff.*;
import static org.ode4j.demo.BunnyGeom.*;
import static org.ode4j.demo.ConvexBunnyGeom.*;


/**
 *
 * Moving convex demo.
 */
public class DemoMovingConvex extends dsFunctions {

	// some constants

	private static final int NUM = 200;			// max number of objects
	private static final double DENSITY = (5.0);		// density of all objects
	private static final int GPB = 3;			// maximum number of geometries per body
	private static final int MAX_CONTACTS = 64;		// maximum number of contact points per body


	// dynamics and collision objects

	private static class MyObject
	{
		DBody body;			// the body
		DGeom[] geom = new DGeom[GPB];		// geometries representing this body
	};

	private static int num=0;		// number of objects in simulation
	private static int nextobj=0;		// next object to recycle if num==NUM
	private static DWorld world;
	private static DSpace space;
	private static MyObject[] obj=new MyObject[NUM];
	private static DJointGroup contactgroup;
	private static int selected = -1;	// selected object
	private static boolean show_aabb = false;	// show geom AABBs?
	private static boolean show_contacts = false;	// show contact points?
	private static boolean random_pos = true;	// drop objects from random position?

	private DNearCallback nearCallback = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}
	};


	// this is called by dSpaceCollide when two objects in space are
	// potentially colliding.

	private void nearCallback( Object data, DGeom o1, DGeom o2 )
	{
		// exit without doing anything if the two bodies are connected by a joint
		DBody b1 = o1.getBody();
		DBody b2 = o2.getBody();
		if ( b1!=null && b2!=null && areConnectedExcluding( b1,b2,DContactJoint.class ) ) return;

		//dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
		DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);

		for ( int i=0; i<MAX_CONTACTS; i++ )
		{
			DContact contact = contacts.get(i);
			contact.surface.mode = dContactBounce | dContactSoftCFM;
			contact.surface.mu = dInfinity;
			contact.surface.mu2 = 0;
			contact.surface.bounce = 0.1;
			contact.surface.bounce_vel = 0.1;
			contact.surface.soft_cfm = 0.01;
		}
		int numc = OdeHelper.collide( o1,o2,MAX_CONTACTS,contacts.getGeomBuffer() );//].geom,
		//sizeof( dContact ) )
		if ( numc != 0 )
		{
			DMatrix3 RI = new DMatrix3();
			RI.setIdentity();
			DVector3C ss = new DVector3(0.02,0.02,0.02);
			for ( int i=0; i<numc; i++ )
			{
				DJoint c = OdeHelper.createContactJoint( world,contactgroup,contacts.get(i) );
				c.attach( b1,b2 );
				if ( show_contacts ) dsDrawBox( contacts.get(i).geom.pos,RI,ss );
			}
		}
	}


	private static float[] xyz = {2.1640f,-1.3079f,1.7600f};
	private static float[] hpr = {125.5000f,-17.0000f,0.0000f};

	// start simulation - set viewpoint

	@Override
	public void start()
	{
		dsSetViewpoint( xyz,hpr );
		System.out.println( "To drop another object, press:" );
		System.out.println( "   b for box." );
		System.out.println( "   s for sphere." );
		System.out.println( "   c for capsule." );
		System.out.println( "   v for a convex." );
		System.out.println( "To select an object, press space." );
		System.out.println( "To disable the selected object, press d." );
		System.out.println( "To enable the selected object, press e." );
		System.out.println( "To toggle showing the geom AABBs, press a." );
		System.out.println( "To toggle showing the contact points, press t." );
		System.out.println( "To toggle dropping from random position/orientation, press r." );
	}


	// called when a key pressed
	@Override
	public void command( char cmd )
	{
		int i,k;
		double[] sides = new double[3];
		DMass m = OdeHelper.createMass();

		cmd = Character.toLowerCase( cmd );
		if ( cmd == 'v' || cmd == 'b' || cmd == 'c' || cmd == 's' || cmd == 'y')
		{
			if ( num < NUM )
			{
				i = num;
				num++;
			}
			else
			{
				i = nextobj;
				nextobj++;
				if ( nextobj >= num ) nextobj = 0;

				// destroy the body and geoms for slot i
				obj[i].body.destroy();
				for ( k=0; k < GPB; k++ )
				{
					if ( obj[i].geom[k] != null ) obj[i].geom[k].destroy();
				}
				obj[i] = new MyObject();
			}

			obj[i].body = OdeHelper.createBody( world );
			for ( k=0; k<3; k++ ) sides[k] = dRandReal()*0.5+0.1;

			DMatrix3 R = new DMatrix3();
			if ( random_pos )
			{
				obj[i].body.setPosition(
						dRandReal()*2-1,dRandReal()*2-1,dRandReal()+3 );
				dRFromAxisAndAngle( R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
						dRandReal()*2.0-1.0,dRandReal()*10.0-5.0 );
			}
			else
			{
				double maxheight = 0;
				for ( k=0; k<num; k++ )
				{
					DVector3C pos = obj[k].body.getPosition();
					if ( pos.get2() > maxheight ) maxheight = pos.get2();
				}
				obj[i].body.setPosition( 0,0,maxheight+1 );
				dRFromAxisAndAngle( R,0,0,1,dRandReal()*10.0-5.0 );
			}
			obj[i].body.setRotation( R );
			obj[i].body.setData( i );

			if ( cmd == 'b' )
			{
				m.setBox( DENSITY,sides[0],sides[1],sides[2] );
				obj[i].geom[0] = OdeHelper.createBox( space,sides[0],sides[1],sides[2] );
			}
			else if ( cmd == 'c' )
			{
				sides[0] *= 0.5;
				m.setCapsule( DENSITY,3,sides[0],sides[1] );
				obj[i].geom[0] = OdeHelper.createCapsule( space,sides[0],sides[1] );
			}
	        else if (cmd == 'y') {
	            m.setCylinder (DENSITY,3,sides[0],sides[1]);
	            obj[i].geom[0] = OdeHelper.createCylinder (space,sides[0],sides[1]);
	        }
			else if ( cmd == 's' )
			{
				sides[0] *= 0.5;
				m.setSphere( DENSITY,sides[0] );
				obj[i].geom[0] = OdeHelper.createSphere( space,sides[0] );
			}
			else  if ( cmd == 'v' )
			{
				obj[i].geom[0] = OdeHelper.createConvex( space,
						convexBunnyPlanes,
						convexBunnyPlaneCount,
						convexBunnyPoints,
						convexBunnyPointCount,
						convexBunnyPolygons );

				/// Use equivalent TriMesh to set mass
				DTriMeshData new_tmdata = OdeHelper.createTriMeshData();
				//				dGeomTriMeshDataBuildSingle( new_tmdata, &Vertices[0], 3 * sizeof( float ), VertexCount,
				//				                             ( dTriIndex* )&Indices[0], IndexCount, 3 * sizeof( dTriIndex ) );
				new_tmdata.build( Vertices, Indices );
				new_tmdata.preprocess2((1 << DTriMeshData.dTRIDATAPREPROCESS_BUILD.FACE_ANGLES), null );

				DTriMesh triMesh = OdeHelper.createTriMesh( null, new_tmdata, null, null, null );

				m.setTrimesh( DENSITY, triMesh );

				triMesh.destroy();
				new_tmdata.destroy();

				DVector3 mc = new DVector3( m.getC() );
				//System.out.println( "mass at " + mc );
				mc.scale( -1 );
				obj[i].geom[0].setPosition( mc );
				m.translate( mc );
			}

			for ( k=0; k < GPB; k++ )
			{
				if ( obj[i].geom[k] != null ) obj[i].geom[k].setBody( obj[i].body );
			}

			obj[i].body.setMass( m );
		}

		if ( cmd == ' ' )
		{
			selected++;
			if ( selected >= num ) selected = 0;
			if ( selected < 0 ) selected = 0;
		}
		else if ( cmd == 'd' && selected >= 0 && selected < num )
		{
			obj[selected].body.disable();
		}
		else if ( cmd == 'e' && selected >= 0 && selected < num )
		{
			obj[selected].body.enable();
		}
		else if ( cmd == 'a' )
		{
			show_aabb ^= true;
		}
		else if ( cmd == 't' )
		{
			show_contacts ^= true;
		}
		else if ( cmd == 'r' )
		{
			random_pos ^= true;
		}
	}

	// draw a geom
	//private void drawGeom( DGeom g, const double *pos, const double *R, int show_aabb )
	private void drawGeom( DGeom g, DVector3C pos, DMatrix3C R, boolean show_aabb )
	{
		if ( g == null ) return;
		if ( pos == null ) pos = g.getPosition();
		if ( R == null ) R = g.getRotation();

		if ( g instanceof DBox )
		{
			DVector3C sides = ((DBox)g).getLengths();
			dsDrawBox( pos,R,sides );
		}
		else if ( g instanceof DSphere )
		{
			dsDrawSphere( pos,R, ((DSphere)g).getRadius() );
		}
	    else if ( g instanceof DCylinder) {
	        double radius = ((DCylinder)g).getRadius();
	        double length = ((DCylinder)g).getLength();
	        dsDrawCylinder (pos,R,length,radius);
	    }
		else if ( g instanceof DCapsule )
		{
			double radius = ((DCapsule)g).getRadius();
			double length = ((DCapsule)g).getLength();
			dsDrawCapsule( pos,R,length,radius );
		}
		else if ( g instanceof DConvex )
		{
			dsDrawConvex( pos,R,
					convexBunnyPlanes,
					convexBunnyPlaneCount,
					convexBunnyPoints,
					convexBunnyPointCount,
					convexBunnyPolygons );
		}

		if ( show_aabb )
		{
			// draw the bounding box for this geom
			DAABBC aabb = g.getAABB();
			DVector3 bbpos = aabb.getCenter();
			DVector3 bbsides = aabb.getLengths();
			DMatrix3 RI = new DMatrix3();
			RI.setIdentity();
			dsSetColorAlpha( 1f, 0f, 0f, 0.5f );
			dsDrawBox( bbpos, RI, bbsides );
		}
	}

	// simulation loop

	private void simLoop( boolean pause )
	{
		dsSetColor( 0,0,2 );
		space.collide( null,nearCallback );

		if ( !pause ) world.quickStep( 0.05 );

		for (DGeom g : space.getGeoms()) {
		}

		// remove all contact joints
		contactgroup.empty();

		dsSetColor( 1,1,0 );
		dsSetTexture( DS_TEXTURE_NUMBER.DS_WOOD );
		for ( int i=0; i<num; i++ )
		{
			for ( int j=0; j < GPB; j++ )
			{
				if ( obj[i].geom[j] != null )
				{
					if ( i==selected )
					{
						dsSetColor( 0f,0.7f,1f );
					}
					else if ( ! obj[i].body.isEnabled() )
					{
						dsSetColor( 1,0,0 );
					}
					else
					{
						dsSetColor( 1,1,0 );
					}

					drawGeom( obj[i].geom[j],null,null,show_aabb );
				}
			}
		}
	}


	/**
	 * @param args args
	 */
	public static void main(String[] args) {
		new DemoMovingConvex().demo(args);
	}


	private void demo(String[] args)
	{

		// create world
		OdeHelper.initODE2( 0 );
		world = OdeHelper.createWorld();

		space = OdeHelper.createSimpleSpace( null );
		contactgroup = OdeHelper.createJointGroup();
		world.setGravity( 0,0,-0.5 );
		world.setCFM( 1e-5 );
		OdeHelper.createPlane( space,0,0,1,0 );
		for (int i = 0; i < NUM; i++) obj[i] = new MyObject();

		// run simulation
		dsSimulationLoop( args,352,288,this );

		contactgroup.destroy();
		space.destroy();
		world.destroy();
		OdeHelper.closeODE();
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
