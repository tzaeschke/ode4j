/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2007-2010 Tilmann Zäschke      *
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

import org.cpp4j.java.RefDouble;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DAABB;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DCapsule;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DContactJoint;
import org.ode4j.ode.DConvex;
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

import static org.cpp4j.C_All.*;
import static org.ode4j.cpp.OdeCpp.*;
import static org.ode4j.drawstuff.DrawStuff.*;
import static org.ode4j.democpp.BunnyGeom.*;
import static org.ode4j.democpp.ConvexBunnyGeom.*;
import static org.ode4j.ode.OdeMath.*;


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
		int i;
		// if (o1->body && o2->body) return;

		// exit without doing anything if the two bodies are connected by a joint
		DBody b1 = dGeomGetBody( o1 );
		DBody b2 = dGeomGetBody( o2 );
		if ( b1!=null && b2!=null && dAreConnectedExcluding( b1,b2,DContactJoint.class ) ) return;

		//dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
		DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);
		
		for ( i=0; i<MAX_CONTACTS; i++ )
		{
			DContact contact = contacts.get(i);
			contact.surface.mode = dContactBounce | dContactSoftCFM;
			contact.surface.mu = dInfinity;
			contact.surface.mu2 = 0;
			contact.surface.bounce = 0.1;
			contact.surface.bounce_vel = 0.1;
			contact.surface.soft_cfm = 0.01;
		}
		int numc = dCollide( o1,o2,MAX_CONTACTS,contacts.getGeomBuffer() );//].geom,
                //sizeof( dContact ) )
		if ( numc != 0 )
		{
			DMatrix3 RI = new DMatrix3();
			dRSetIdentity( RI );
			DVector3C ss = new DVector3(0.02,0.02,0.02);
			for ( i=0; i<numc; i++ )
			{
				DJoint c = dJointCreateContact( world,contactgroup,contacts.get(i) );
				dJointAttach( c,b1,b2 );
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
		dAllocateODEDataForThread( dAllocateMaskAll );

		dsSetViewpoint( xyz,hpr );
		printf( "To drop another object, press:\n" );
		printf( "   b for box.\n" );
		printf( "   s for sphere.\n" );
		printf( "   c for capsule.\n" );
		printf( "   v for a convex.\n" );
		printf( "To select an object, press space.\n" );
		printf( "To disable the selected object, press d.\n" );
		printf( "To enable the selected object, press e.\n" );
		printf( "To toggle showing the geom AABBs, press a.\n" );
		printf( "To toggle showing the contact points, press t.\n" );
		printf( "To toggle dropping from random position/orientation, press r.\n" );
	}


	// called when a key pressed
	@Override
	public void command( char cmd )
	{
		int i,j,k;
		double[] sides = new double[3];
		DMass m = OdeHelper.createMass();

		cmd = Character.toLowerCase( cmd );
		if ( cmd == 'v' || cmd == 'b' || cmd == 'c' || cmd == 's' )
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
				dBodyDestroy( obj[i].body );
				for ( k=0; k < GPB; k++ )
				{
					if ( obj[i].geom[k] != null ) dGeomDestroy( obj[i].geom[k] );
				}
				//memset( &obj[i],0,sizeof( obj[i] ) );
				obj[i] = new MyObject();
			}

			obj[i].body = dBodyCreate( world );
			for ( k=0; k<3; k++ ) sides[k] = dRandReal()*0.5+0.1;

			DMatrix3 R = new DMatrix3();
			if ( random_pos )
			{
				dBodySetPosition( obj[i].body,
				                  dRandReal()*2-1,dRandReal()*2-1,dRandReal()+3 );
				dRFromAxisAndAngle( R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
				                    dRandReal()*2.0-1.0,dRandReal()*10.0-5.0 );
			}
			else
			{
				double maxheight = 0;
				for ( k=0; k<num; k++ )
				{
					DVector3C pos = dBodyGetPosition( obj[k].body );
					if ( pos.get2() > maxheight ) maxheight = pos.get2();
				}
				dBodySetPosition( obj[i].body, 0,0,maxheight+1 );
				dRFromAxisAndAngle( R,0,0,1,dRandReal()*10.0-5.0 );
			}
			dBodySetRotation( obj[i].body,R );
			dBodySetData( obj[i].body, i );

			if ( cmd == 'b' )
			{
				dMassSetBox( m,DENSITY,sides[0],sides[1],sides[2] );
				obj[i].geom[0] = dCreateBox( space,sides[0],sides[1],sides[2] );
			}
			else if ( cmd == 'c' )
			{
				sides[0] *= 0.5;
				dMassSetCapsule( m,DENSITY,3,sides[0],sides[1] );
				obj[i].geom[0] = dCreateCapsule( space,sides[0],sides[1] );
			}
			else if ( cmd == 's' )
			{
				sides[0] *= 0.5;
				dMassSetSphere( m,DENSITY,sides[0] );
				obj[i].geom[0] = dCreateSphere( space,sides[0] );
			}
			else  if ( cmd == 'v' )
			{
				obj[i].geom[0] = dCreateConvex( space,
				                                convexBunnyPlanes,
				                                convexBunnyPlaneCount,
				                                convexBunnyPoints,
				                                convexBunnyPointCount,
				                                convexBunnyPolygons );

				/// Use equivalent TriMesh to set mass
				DTriMeshData new_tmdata = dGeomTriMeshDataCreate();
//				dGeomTriMeshDataBuildSingle( new_tmdata, &Vertices[0], 3 * sizeof( float ), VertexCount,
//				                             ( dTriIndex* )&Indices[0], IndexCount, 3 * sizeof( dTriIndex ) );
				dGeomTriMeshDataBuildSingle( new_tmdata, Vertices, 3, VertexCount,
                        Indices, IndexCount, 3 );

				DTriMesh triMesh = dCreateTriMesh( null, new_tmdata, null, null, null );

				dMassSetTrimesh( m, DENSITY, triMesh );

				dGeomDestroy( triMesh );
				dGeomTriMeshDataDestroy( new_tmdata );

				DVector3C mc = m.getC();
				printf( "mass at %f %f %f\n", mc.get0(), mc.get1(), mc.get2() );
				dGeomSetPosition( obj[i].geom[0], -mc.get0(), -mc.get1(), -mc.get2() );
				dMassTranslate( m, -mc.get0(), -mc.get1(), -mc.get2() );
			}

			for ( k=0; k < GPB; k++ )
			{
				if ( obj[i].geom[k] != null ) dGeomSetBody( obj[i].geom[k],obj[i].body );
			}

			dBodySetMass( obj[i].body, m );
		}

		if ( cmd == ' ' )
		{
			selected++;
			if ( selected >= num ) selected = 0;
			if ( selected < 0 ) selected = 0;
		}
		else if ( cmd == 'd' && selected >= 0 && selected < num )
		{
			dBodyDisable( obj[selected].body );
		}
		else if ( cmd == 'e' && selected >= 0 && selected < num )
		{
			dBodyEnable( obj[selected].body );
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
		if ( pos == null ) pos = dGeomGetPosition( g );
		if ( R == null ) R = dGeomGetRotation( g );

		if ( g instanceof DBox )
		{
			DVector3 sides = new DVector3();
			dGeomBoxGetLengths( (DBox)g,sides );
			dsDrawBox( pos,R,sides );
		}
		else if ( g instanceof DSphere )
		{
			dsDrawSphere( pos,R,dGeomSphereGetRadius( (DSphere)g ) );
		}
		else if ( g instanceof DCapsule )
		{
			RefDouble radius = new RefDouble(),length = new RefDouble();
			dGeomCapsuleGetParams( (DCapsule)g,radius,length );
			dsDrawCapsule( pos,R,length.d,radius.d );
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
			DAABB aabb = new DAABB();
			dGeomGetAABB( g,aabb );
			DVector3 bbpos = aabb.getCenter();
			//for ( int i=0; i<3; i++ ) bbpos[i] = 0.5*( aabb[i*2] + aabb[i*2+1] );
			DVector3 bbsides = aabb.getLengths();
			//for ( int j=0; j<3; j++ ) bbsides[j] = aabb[j*2+1] - aabb[j*2];
			DMatrix3 RI = new DMatrix3();
			dRSetIdentity( RI );
			dsSetColorAlpha( 1f,0f,0f,0.5f );
			dsDrawBox( bbpos,RI,bbsides );
		}
	}

	// simulation loop

	private void simLoop( boolean pause )
	{
		dsSetColor( 0,0,2 );
		dSpaceCollide( space,0,nearCallback );

		if ( !pause ) dWorldQuickStep( world,0.05 );

		for ( int j = 0; j < dSpaceGetNumGeoms( space ); j++ )
		{
			dSpaceGetGeom( space, j );
		}

		// remove all contact joints
		dJointGroupEmpty( contactgroup );

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
					else if ( ! dBodyIsEnabled( obj[i].body ) )
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


	public static void main(String[] args) {
		new DemoMovingConvex().demo(args);
	}
	
	
	private void demo(String[] args)
	{
		// setup pointers to drawstuff callback functions
//		dsFunctions fn = this;
//		fn.version = DS_VERSION;
//		fn.start = &start;
//		fn.step = &simLoop;
//		fn.command = &command;
//		fn.stop = 0;
//		fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

		// create world
		dInitODE2( 0 );
		world = dWorldCreate();

		space = dSimpleSpaceCreate( null );
		contactgroup = dJointGroupCreate( 0 );
		dWorldSetGravity( world,0,0,-0.5 );
		dWorldSetCFM( world,1e-5 );
		dCreatePlane( space,0,0,1,0 );
		//memset( obj,0,sizeof( obj ) );
		for (int i = 0; i < NUM; i++) obj[i] = new MyObject();

		// run simulation
		dsSimulationLoop( args,352,288,this );

		dJointGroupDestroy( contactgroup );
		dSpaceDestroy( space );
		dWorldDestroy( world );
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
