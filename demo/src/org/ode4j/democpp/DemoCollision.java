/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.democpp;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import org.cpp4j.java.RefDouble;
import org.cpp4j.java.RefInt;
import org.ode4j.drawstuff.DrawStuff.dsFunctions;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector4;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DCapsule;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DPlane;
import org.ode4j.ode.DRay;
import org.ode4j.ode.DSimpleSpace;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.OdeMath.OP;
import org.ode4j.ode.DGeom.DNearCallback;

import static org.cpp4j.C_All.*;
import static org.ode4j.cpp.OdeCpp.*;
import static org.ode4j.drawstuff.DrawStuff.*;
import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.DGeom.*;

/**
 * collision tests. if this program is run without any arguments it will
 * perform all the tests multiple times, with different random data for each
 * test. if this program is given a test number it will run that test
 * graphically/interactively, in which case the space bar can be used to
 * change the random test conditions.
 */
class DemoCollision extends dsFunctions {


	//****************************************************************************
	// test infrastructure, including constants and macros

	//#define TEST_REPS1 1000		// run each test this many times (first batch)
	//#define TEST_REPS2 10000	// run each test this many times (second batch)
	//const dReal tol = 1e-8;		// tolerance used for numerical checks
	//#define MAX_TESTS 1000		// maximum number of test slots
	//#define Z_OFFSET 2		// z offset for drawing (to get above ground)
	private static final int TEST_REPS1 = 1000;		// run each test this many times (first batch)
	private static final int TEST_REPS2 = 10000;	// run each test this many times (second batch)
	private static final float tol = 1e-8f;		// tolerance used for numerical checks
	private static final int MAX_TESTS = 1000;		// maximum number of test slots
	private static final float Z_OFFSET = 2;		// z offset for drawing (to get above ground)

	//using namespace ode;

	// test function. returns 1 if the test passed or 0 if it failed
	//TODO
	//typedef int test_function_t();

	private class TestSlot {
		int number;			// number of test
		//const char *name;			// name of test
		String name;			// name of test
		int failcount;
		//TODO ?
		//  test_function_t test_fn;
		int last_failed_line;
		public boolean test_fn() {
			return DemoCollision.this.runtest(name);
		}
	};

	private boolean runtest(String name) {
		try {
			Method m = DemoCollision.class.getDeclaredMethod(name);
			return (Boolean)m.invoke(this);
		} catch (SecurityException e) {
			throw new RuntimeException(e);
		} catch (NoSuchMethodException e) {
			throw new RuntimeException(e);
		} catch (IllegalArgumentException e) {
			throw new RuntimeException(e);
		} catch (IllegalAccessException e) {
			throw new RuntimeException(e);
		} catch (InvocationTargetException e) {
			throw new RuntimeException(e);
		}
	}
	
	
	//TODO initilalize
	private TestSlot[] testslot=new TestSlot[MAX_TESTS];


	// globals used by the test functions
	private int graphical_test=0;		// show graphical results of this test, 0=none
	private int current_test;		// currently execiting test
	private boolean draw_all_objects_called;


	//#define MAKE_TEST(number,function) \
	//  if (testslot[number].name) dDebug (0,"test number already used"); \
	//  if (number <= 0 || number >= MAX_TESTS) dDebug (0,"bad test number"); \
	//  testslot[number].name = # function; \
	//  testslot[number].test_fn = function;
	private void MAKE_TEST(int number,String function) {
		if (testslot[number].name != null) dDebug (0,"test number already used"); 
		if (number <= 0 || number >= MAX_TESTS) dDebug (0,"bad test number"); 
		//  testslot[number].name = # function; 
		//  testslot[number].test_fn = function;
		//TZ TODO?
		testslot[number].name = function; 
	}

	//#define FAILED() { if (graphical_test==0) { \
	//  testslot[current_test].last_failed_line=__LINE__; return 0; } }
	private boolean testFAILED() { 
		if (graphical_test==0) { 
			testslot[current_test].last_failed_line=new RuntimeException().getStackTrace()[0].getLineNumber(); 
			return false; 
		} else return true; 
		
	}
	//#define PASSED() { return 1; }
	private boolean retPASSED() { return true; }

	//****************************************************************************
	// globals

	/* int dBoxBox (const dVector3 p1, const dMatrix3 R1,
	     const dVector3 side1, const dVector3 p2,
	     const dMatrix3 R2, const dVector3 side2,
	     dVector3 normal, dReal *depth, int *code,
	     int maxc, dContactGeom *contact, int skip); */

	//TODO
	//void dLineClosestApproach (final dVector3 pa, final dVector3 ua,
	//			   final dVector3 pb, final dVector3 ub,
	//			   double *alpha, double *beta);

	//****************************************************************************
	// draw all objects in a space, and draw all the collision contact points

	//void nearCallback (void *data, dGeom o1, dGeom o2)
	private void nearCallback (Object data, DGeom o1, DGeom o2)
	{
		int i,n;
		final int N = 100;
		//dContactGeom contact[N];
		DContactGeomBuffer contacts = new DContactGeomBuffer(N);

		if (o2 instanceof DRay) {
			n = dCollide (o2,o1,N,contacts);//,sizeof(dContactGeom));
		}
		else {
			n = dCollide (o1,o2,N,contacts);//,sizeof(dContactGeom));
		}
		if (n > 0) {
			DMatrix3 RI = new DMatrix3();
			dRSetIdentity (RI);
			DVector3 ss = new DVector3(0.01,0.01,0.01);
			for (i=0; i<n; i++) {
				DContactGeom contact = contacts.get(i);
				contact.pos.v[2] += Z_OFFSET;
				dsDrawBox (contact.pos,RI,ss);
				DVector3 n2 = new DVector3();
				//for (j=0; j<3; j++) n[j] = contact[i].pos[j] + 0.1*contact[i].normal[j];
				n2.eqSum(contact.pos, contact.normal, 0.1);
				dsDrawLine (contact.pos,n2);
			}
		}
	}

	private DNearCallback nearCallback = new DNearCallback() {
		@Override
		public void call(Object data, DGeom o1, DGeom o2) {
			nearCallback(data, o1, o2);
		}};
	
	private void draw_all_objects (DSpace space)
	{
		int i, j;

		draw_all_objects_called = true;
		if (0==graphical_test) return;
		int nGeoms = dSpaceGetNumGeoms (space);

		// draw all contact points
		dsSetColor (0,1,1);
		dSpaceCollide (space,0,nearCallback);

		// draw all rays
		for (i=0; i<nGeoms; i++) {
			DGeom g = dSpaceGetGeom (space,i);
			if (g instanceof DRay) {
				dsSetColor (1,1,1);
				DVector3 origin = new DVector3(),dir=new DVector3();
				dGeomRayGet ((DRay)g,origin,dir);
				origin.v[2] += Z_OFFSET;
				double length = dGeomRayGetLength ((DRay)g);
				//for (j=0; j<3; j++) dir[j] = dir[j]*length + origin[j];
				dir.eqSum(origin, dir, length);
				dsDrawLine (origin,dir);
				dsSetColor (0,0,1);
				dsDrawSphere (origin,dGeomGetRotation(g),0.01);
			}
		}

		// draw all other objects
		for (i=0; i<nGeoms; i++) {
			DGeom g = dSpaceGetGeom (space,i);
			DVector3 pos = new DVector3();
			if (!(g instanceof DPlane)) {//dGeomGetClass (g) != dPlaneClass) {
				//memcpy (pos,dGeomGetPosition(g),sizeof(pos));
				pos.set(dGeomGetPosition(g));
				pos.v[2] += Z_OFFSET;
			}

			switch (g.getClassID()) {

			case dSphereClass: {
				dsSetColorAlpha (1f,0f,0f,0.8f);
				double radius = dGeomSphereGetRadius ((DSphere)g);
				dsDrawSphere (pos,dGeomGetRotation(g),radius);
				break;
			}

			case dBoxClass: {
				dsSetColorAlpha (1f,1f,0f,0.8f);
				DVector3 sides = new DVector3();
				dGeomBoxGetLengths ((DBox)g,sides);
				dsDrawBox (pos,dGeomGetRotation(g),sides);
				break;
			}

			case dCapsuleClass: {
				dsSetColorAlpha (0f,1f,0f,0.8f);
				RefDouble radius = new RefDouble(),length=new RefDouble();
				dGeomCapsuleGetParams ((DCapsule)g,radius,length);
				dsDrawCapsule (pos,dGeomGetRotation(g),length.getF(),radius.getF());
				break;
			}

			case dPlaneClass: {
				DVector4 n4=new DVector4();
				DMatrix3 R = new DMatrix3();//,sides;
				DVector3 sides = new DVector3();
				DVector3 pos2 = new DVector3();
				dGeomPlaneGetParams ((DPlane)g,n4);
				dRFromZAxis (R,n4.get0(),n4.get1(),n4.get2());
				for (j=0; j<3; j++) pos.set(j, n4.get(j)*n4.get3());
				pos.v[2] += Z_OFFSET;
//				sides[0] = 2;
//				sides[1] = 2;
//				sides[2] = 0.001;
				sides.set(2, 2, 0.001);
				dsSetColor (1,0,1);
				for (j=0; j<3; j++) pos2.set(j, pos.get(j) + 0.1*n4.get(j));
				dsDrawLine (pos,pos2);
				dsSetColorAlpha (1f,0f,1f,0.8f);
				dsDrawBox (pos,R,sides);
				break;
			}

			}
		}
	}

	//****************************************************************************
	// point depth tests

	private boolean test_sphere_point_depth()
	{
		int j;
		DVector3 p=new DVector3(),q=new DVector3();
		DMatrix3 R=new DMatrix3();
		double r,d;

		//dSimpleSpace space(0);
		DSimpleSpace space = dSimpleSpaceCreate(null);
		DSphere sphere = dCreateSphere (null,1);
		dSpaceAdd (space,sphere);

		// ********** make a random sphere of radius r at position p

		r = dRandReal()+0.1;
		dGeomSphereSetRadius (sphere,r);
		dMakeRandomVector (p,1.0);
		dGeomSetPosition (sphere,p.v[0],p.v[1],p.v[2]);
		dRFromAxisAndAngle (R,dRandReal()*2-1,dRandReal()*2-1,
				dRandReal()*2-1,dRandReal()*10-5);
		dGeomSetRotation (sphere,R);

		// ********** test center point has depth r

		if (dFabs(dGeomSpherePointDepth (sphere,p.v[0],p.v[1],p.v[2]) - r) > tol) if (testFAILED()) return false;

		// ********** test point on surface has depth 0

		for (j=0; j<3; j++) q.v[j] = dRandReal()-0.5;
		dNormalize3 (q);
		for (j=0; j<3; j++) q.v[j] = q.v[j]*r + p.v[j];
		if (dFabs(dGeomSpherePointDepth (sphere,q.v[0],q.v[1],q.v[2])) > tol) if (testFAILED()) return false;

		// ********** test point at random depth

		d = (dRandReal()*2-1) * r;
		for (j=0; j<3; j++) q.v[j] = dRandReal()-0.5;
		dNormalize3 (q);
		for (j=0; j<3; j++) q.v[j] = q.v[j]*(r-d) + p.v[j];
		if (dFabs(dGeomSpherePointDepth (sphere,q.v[0],q.v[1],q.v[2])-d) > tol) if (testFAILED()) return false;

		return retPASSED();
	}


	private boolean test_box_point_depth()
	{
		int i,j;
		DVector3 s=new DVector3(),p=new DVector3(),q=new DVector3(),q2=new DVector3();	// s = box sides
		DMatrix3 R=new DMatrix3();
		double ss,d;		// ss = smallest side

		DSimpleSpace space = dSimpleSpaceCreate(null);
		DBox box = dCreateBox (null,1,1,1);
		dSpaceAdd (space,box);

		// ********** make a random box

		for (j=0; j<3; j++) s.set(j, dRandReal() + 0.1);
		dGeomBoxSetLengths (box,s.v[0],s.v[1],s.v[2]);
		dMakeRandomVector (p,1.0);
		dGeomSetPosition (box,p.v[0],p.v[1],p.v[2]);
		dRFromAxisAndAngle (R,dRandReal()*2-1,dRandReal()*2-1,
				dRandReal()*2-1,dRandReal()*10-5);
		dGeomSetRotation (box,R);

		// ********** test center point has depth of smallest side

		ss = 1e9;
		for (j=0; j<3; j++) if (s.get(j) < ss) ss = s.get(j);
		if (dFabs(dGeomBoxPointDepth (box,p.v[0],p.v[1],p.v[2]) - 0.5*ss) > tol)
			if (testFAILED()) return false;

		// ********** test point on surface has depth 0

		for (j=0; j<3; j++) q.set(j, (dRandReal()-0.5)*s.get(j) );
		i = dRandInt (3);
		if (dRandReal() > 0.5) q.v[i] = 0.5*s.v[i]; else q.v[i] = -0.5*s.v[i];
		dMultiply0 (q2,dGeomGetRotation(box),q,3,3,1);
		//for (j=0; j<3; j++) q2[j] += p[j];
		q2.add(p);
		if (dFabs(dGeomBoxPointDepth (box,q2.v[0],q2.v[1],q2.v[2])) > tol) if (testFAILED()) return false;

		// ********** test points outside box have -ve depth

		for (j=0; j<3; j++) {
			q.v[j] = 0.5*s.v[j] + dRandReal() + 0.01;
			if (dRandReal() > 0.5) q.v[j] = -q.v[j];
		}
		dMultiply0 (q2,dGeomGetRotation(box),q,3,3,1);
		for (j=0; j<3; j++) q2.v[j] += p.v[j];
		if (dGeomBoxPointDepth (box,q2.v[0],q2.v[1],q2.v[2]) >= 0) if (testFAILED()) return false;

		// ********** test points inside box have +ve depth

		for (j=0; j<3; j++) q.v[j] = s.v[j] * 0.99 * (dRandReal()-0.5);
		dMultiply0 (q2,dGeomGetRotation(box),q,3,3,1);
		for (j=0; j<3; j++) q2.v[j] += p.v[j];
		if (dGeomBoxPointDepth (box,q2.v[0],q2.v[1],q2.v[2]) <= 0) if (testFAILED()) return false;

		// ********** test random depth of point aligned along axis (up to ss deep)

		i = dRandInt (3);
		for (j=0; j<3; j++) q.v[j] = 0;
		d = (dRandReal()*(ss*0.5+1)-1);
		q.v[i] = s.v[i]*0.5 - d;
		if (dRandReal() > 0.5) q.v[i] = -q.v[i];
		dMultiply0 (q2,dGeomGetRotation(box),q,3,3,1);
		for (j=0; j<3; j++) q2.v[j] += p.v[j];
		if (dFabs(dGeomBoxPointDepth (box,q2.v[0],q2.v[1],q2.v[2]) - d) >= tol) if (testFAILED()) return false;

		return retPASSED();
	}


	private boolean test_ccylinder_point_depth()
	{
		int j;
		DVector3 p=new DVector3(),a=new DVector3();
		DMatrix3 R=new DMatrix3();
		double r,l,beta,x,y,d;

				DSimpleSpace space = dSimpleSpaceCreate(null);

		DCapsule ccyl = dCreateCapsule (null,1,1);
		dSpaceAdd (space,ccyl);

		// ********** make a random ccyl

		r = dRandReal()*0.5 + 0.01;
		l = dRandReal()*1 + 0.01;
		dGeomCapsuleSetParams (ccyl,r,l);
		dMakeRandomVector (p,1.0);
		dGeomSetPosition (ccyl,p.v[0],p.v[1],p.v[2]);
		dRFromAxisAndAngle (R,dRandReal()*2-1,dRandReal()*2-1,
				dRandReal()*2-1,dRandReal()*10-5);
		dGeomSetRotation (ccyl,R);

		// ********** test point on axis has depth of 'radius'

		beta = dRandReal()-0.5;
		for (j=0; j<3; j++) a.v[j] = p.v[j] + l*beta*R.v[j*4+2];
		if (dFabs(dGeomCapsulePointDepth (ccyl,a.v[0],a.v[1],a.v[2]) - r) >= tol)
			if (testFAILED()) return false;

		// ********** test point on surface (excluding caps) has depth 0

		beta = dRandReal()*2*M_PI;
		x = r*sin(beta);
		y = r*cos(beta);
		beta = dRandReal()-0.5;
		for (j=0; j<3; j++) a.v[j] = p.v[j] + x*R.v[j*4+0] + y*R.v[j*4+1] + l*beta*R.v[j*4+2];
		if (dFabs(dGeomCapsulePointDepth (ccyl,a.v[0],a.v[1],a.v[2])) >= tol) if (testFAILED()) return false;

		// ********** test point on surface of caps has depth 0

		for (j=0; j<3; j++) a.v[j] = dRandReal()-0.5;
		dNormalize3 (a);
		if (dDOT14(a,R,2) > 0) {
			for (j=0; j<3; j++) a.v[j] = p.v[j] + a.v[j]*r + l*0.5*R.v[j*4+2];
		}
		else {
			for (j=0; j<3; j++) a.v[j] = p.v[j] + a.v[j]*r - l*0.5*R.v[j*4+2];
		}
		if (dFabs(dGeomCapsulePointDepth (ccyl,a.v[0],a.v[1],a.v[2])) >= tol) if (testFAILED()) return false;

		// ********** test point inside ccyl has positive depth

		for (j=0; j<3; j++) a.v[j] = dRandReal()-0.5;
		dNormalize3 (a);
		beta = dRandReal()-0.5;
		for (j=0; j<3; j++) a.v[j] = p.v[j] + a.v[j]*r*0.99 + l*beta*R.v[j*4+2];
		if (dGeomCapsulePointDepth (ccyl,a.v[0],a.v[1],a.v[2]) < 0) if (testFAILED()) return false;

		// ********** test point depth (1)

		d = (dRandReal()*2-1) * r;
		beta = dRandReal()*2*M_PI;
		x = (r-d)*sin(beta);
		y = (r-d)*cos(beta);
		beta = dRandReal()-0.5;
		for (j=0; j<3; j++) a.v[j] = p.v[j] + x*R.v[j*4+0] + y*R.v[j*4+1] + l*beta*R.v[j*4+2];
		if (dFabs(dGeomCapsulePointDepth (ccyl,a.v[0],a.v[1],a.v[2]) - d) >= tol)
			if (testFAILED()) return false;

		// ********** test point depth (2)

		d = (dRandReal()*2-1) * r;
		for (j=0; j<3; j++) a.v[j] = dRandReal()-0.5;
		dNormalize3 (a);
		if (dDOT14(a,R,2) > 0) {
			for (j=0; j<3; j++) a.v[j] = p.v[j] + a.v[j]*(r-d) + l*0.5*R.v[j*4+2];
		}
		else {
			for (j=0; j<3; j++) a.v[j] = p.v[j] + a.v[j]*(r-d) - l*0.5*R.v[j*4+2];
		}
		if (dFabs(dGeomCapsulePointDepth (ccyl,a.v[0],a.v[1],a.v[2]) - d) >= tol)
			if (testFAILED()) return false;

		return retPASSED();
	}


	private boolean test_plane_point_depth()
	{
		int j;
		DVector3 n=new DVector3(),p=new DVector3(),q=new DVector3(),
			a=new DVector3(),b=new DVector3();	// n = plane normal
		double d;

				DSimpleSpace space = dSimpleSpaceCreate(null);

		DPlane plane = dCreatePlane (null,0,0,1,0);
		dSpaceAdd (space,plane);

		// ********** make a random plane

		for (j=0; j<3; j++) n.v[j] = dRandReal() - 0.5;
		dNormalize3 (n);
		d = dRandReal() - 0.5;
		dGeomPlaneSetParams (plane,n.v[0],n.v[1],n.v[2],d);
		dPlaneSpace (n,p,q);

		// ********** test point on plane has depth 0

		a.v[0] = dRandReal() - 0.5;
		a.v[1] = dRandReal() - 0.5;
		a.v[2] = 0;
		for (j=0; j<3; j++) b.v[j] = a.v[0]*p.v[j] + a.v[1]*q.v[j] + (a.v[2]+d)*n.v[j];
		if (dFabs(dGeomPlanePointDepth (plane,b.v[0],b.v[1],b.v[2])) >= tol) if (testFAILED()) return false;

		// ********** test arbitrary depth point

		a.v[0] = dRandReal() - 0.5;
		a.v[1] = dRandReal() - 0.5;
		a.v[2] = dRandReal() - 0.5;
		for (j=0; j<3; j++) b.v[j] = a.v[0]*p.v[j] + a.v[1]*q.v[j] + (a.v[2]+d)*n.v[j];
		if (dFabs(dGeomPlanePointDepth (plane,b.v[0],b.v[1],b.v[2]) + a.v[2]) >= tol)
			if (testFAILED()) return false;

		// ********** test depth-1 point

		a.v[0] = dRandReal() - 0.5;
		a.v[1] = dRandReal() - 0.5;
		a.v[2] = -1;
		for (j=0; j<3; j++) b.v[j] = a.v[0]*p.v[j] + a.v[1]*q.v[j] + (a.v[2]+d)*n.v[j];
		if (dFabs(dGeomPlanePointDepth (plane,b.v[0],b.v[1],b.v[2]) - 1) >= tol) if (testFAILED()) return false;

		return retPASSED();
	}

	//****************************************************************************
	// ray tests

	private boolean test_ray_and_sphere()
	{
		int j;
		DContactGeomBuffer contacts = new DContactGeomBuffer(1);
		DVector3 p=new DVector3(),q=new DVector3(),q2=new DVector3(),
			n=new DVector3(),v1=new DVector3();
		DMatrix3 R=new DMatrix3();
		double r,k;

		DSimpleSpace space = dSimpleSpaceCreate(null);

		DRay ray = dCreateRay (null,0);
		DSphere sphere = dCreateSphere (null,1);
		dSpaceAdd (space,ray);
		dSpaceAdd (space,sphere);

		// ********** make a random sphere of radius r at position p

		r = dRandReal()+0.1;
		dGeomSphereSetRadius (sphere,r);
		dMakeRandomVector (p,1.0);
		dGeomSetPosition (sphere,p.v[0],p.v[1],p.v[2]);
		dRFromAxisAndAngle (R,dRandReal()*2-1,dRandReal()*2-1,
				dRandReal()*2-1,dRandReal()*10-5);
		dGeomSetRotation (sphere,R);

		// ********** test zero length ray just inside sphere

		dGeomRaySetLength (ray,0);
		dMakeRandomVector (q,1.0);
		dNormalize3 (q);
		for (j=0; j<3; j++) q.v[j] = 0.99*r * q.v[j] + p.v[j];
		dGeomSetPosition (ray,q.v[0],q.v[1],q.v[2]);
		dRFromAxisAndAngle (R,dRandReal()*2-1,dRandReal()*2-1,
				dRandReal()*2-1,dRandReal()*10-5);
		dGeomSetRotation (ray,R);
		if (dCollide (ray,sphere,1,contacts) != 0) if (testFAILED()) return false;

		// ********** test zero length ray just outside that sphere

		dGeomRaySetLength (ray,0);
		dMakeRandomVector (q,1.0);
		dNormalize3 (q);
		for (j=0; j<3; j++) q.v[j] = 1.01*r * q.v[j] + p.v[j];
		dGeomSetPosition (ray,q.v[0],q.v[1],q.v[2]);
		dRFromAxisAndAngle (R,dRandReal()*2-1,dRandReal()*2-1,
				dRandReal()*2-1,dRandReal()*10-5);
		dGeomSetRotation (ray,R);
		if (dCollide (ray,sphere,1,contacts) != 0) if (testFAILED()) return false;

		// ********** test finite length ray totally contained inside the sphere

		dMakeRandomVector (q,1.0);
		dNormalize3 (q);
		k = dRandReal();
		for (j=0; j<3; j++) q.v[j] = k*r*0.99 * q.v[j] + p.v[j];
		dMakeRandomVector (q2,1.0);
		dNormalize3 (q2);
		k = dRandReal();
		for (j=0; j<3; j++) q2.v[j] = k*r*0.99 * q2.v[j] + p.v[j];
		for (j=0; j<3; j++) n.v[j] = q2.v[j] - q.v[j];
		dNormalize3 (n);
		dGeomRaySet (ray,q.v[0],q.v[1],q.v[2],n.v[0],n.v[1],n.v[2]);
		dGeomRaySetLength (ray,dDISTANCE (q,q2));
		if (dCollide (ray,sphere,1,contacts) != 0) if (testFAILED()) return false;

		// ********** test finite length ray totally outside the sphere

		dMakeRandomVector (q,1.0);
		dNormalize3 (q);
		do {
			dMakeRandomVector (n,1.0);
			dNormalize3 (n);
		}
		while (dDOT(n,q) < 0);	// make sure normal goes away from sphere
		for (j=0; j<3; j++) q.v[j] = 1.01*r * q.v[j] + p.v[j];
		dGeomRaySet (ray,q.v[0],q.v[1],q.v[2],n.v[0],n.v[1],n.v[2]);
		dGeomRaySetLength (ray,100);
		if (dCollide (ray,sphere,1,contacts) != 0) if (testFAILED()) return false;

		// ********** test ray from outside to just above surface

		dMakeRandomVector (q,1.0);
		dNormalize3 (q);
		for (j=0; j<3; j++) n.v[j] = -q.v[j];
		for (j=0; j<3; j++) q2.v[j] = 2*r * q.v[j] + p.v[j];
		dGeomRaySet (ray,q2.v[0],q2.v[1],q2.v[2],n.v[0],n.v[1],n.v[2]);
		dGeomRaySetLength (ray,0.99*r);
		if (dCollide (ray,sphere,1,contacts) != 0) if (testFAILED()) return false;

		// ********** test ray from outside to just below surface

		dGeomRaySetLength (ray,1.01*r);
		if (dCollide (ray,sphere,1,contacts) != 1) if (testFAILED()) return false;
		for (j=0; j<3; j++) q2.v[j] = r * q.v[j] + p.v[j];
		if (dDISTANCE (contacts.get(0).pos,q2) > tol) if (testFAILED()) return false;

		// ********** test contact point distance for random rays

		dMakeRandomVector (q,1.0);
		dNormalize3 (q);
		k = dRandReal()+0.5;
		for (j=0; j<3; j++) q.v[j] = k*r * q.v[j] + p.v[j];
		dMakeRandomVector (n,1.0);
		dNormalize3 (n);
		dGeomRaySet (ray,q.v[0],q.v[1],q.v[2],n.v[0],n.v[1],n.v[2]);
		dGeomRaySetLength (ray,100);
		if (dCollide (ray,sphere,1,contacts)!=0) {
			DContactGeom contact = contacts.get(0);
			k = dDISTANCE (contacts.get(0).pos,dGeomGetPosition(sphere));
			if (dFabs(k - r) > tol) if (testFAILED()) return false;
			// also check normal signs
			if (dDOT (n,contact.normal) > 0) if (testFAILED()) return false;
			// also check depth of contact point
			if (dFabs (dGeomSpherePointDepth
					(sphere,contact.pos.v[0],contact.pos.v[1],contact.pos.v[2])) > tol)
				if (testFAILED()) return false;

			draw_all_objects (space);
		}

		// ********** test tangential grazing - miss

		dMakeRandomVector (q,1.0);
		dNormalize3 (q);
		dPlaneSpace (q,n,v1);
		for (j=0; j<3; j++) q.v[j] = 1.01*r * q.v[j] + p.v[j];
		for (j=0; j<3; j++) q.v[j] -= n.v[j];
		dGeomRaySet (ray,q.v[0],q.v[1],q.v[2],n.v[0],n.v[1],n.v[2]);
		dGeomRaySetLength (ray,2);
		if (dCollide (ray,sphere,1,contacts) != 0) if (testFAILED()) return false;

		// ********** test tangential grazing - hit

		dMakeRandomVector (q,1.0);
		dNormalize3 (q);
		dPlaneSpace (q,n,v1);
		for (j=0; j<3; j++) q.v[j] = 0.99*r * q.v[j] + p.v[j];
		for (j=0; j<3; j++) q.v[j] -= n.v[j];
		dGeomRaySet (ray,q.v[0],q.v[1],q.v[2],n.v[0],n.v[1],n.v[2]);
		dGeomRaySetLength (ray,2);
		if (dCollide (ray,sphere,1,contacts) != 1) if (testFAILED()) return false;

		return retPASSED();
	}


	private boolean test_ray_and_box()
	{
		int i,j;
		//dContactGeom contact;
		DContactGeomBuffer contacts = new DContactGeomBuffer(1);
		DVector3 s=new DVector3(),p=new DVector3(),q=new DVector3(),n=new DVector3(),
			q2=new DVector3(),q3=new DVector3(),q4=new DVector3();		// s = box sides
		DMatrix3 R=new DMatrix3();
		double k;

		DSimpleSpace space = dSimpleSpaceCreate(null);

		DRay ray = dCreateRay (null,0);
		DBox box = dCreateBox (null,1,1,1);
		dSpaceAdd (space,ray);
		dSpaceAdd (space,box);

		// ********** make a random box

		for (j=0; j<3; j++) s.v[j] = dRandReal() + 0.1;
		dGeomBoxSetLengths (box,s.v[0],s.v[1],s.v[2]);
		dMakeRandomVector (p,1.0);
		dGeomSetPosition (box,p.v[0],p.v[1],p.v[2]);
		dRFromAxisAndAngle (R,dRandReal()*2-1,dRandReal()*2-1,
				dRandReal()*2-1,dRandReal()*10-5);
		dGeomSetRotation (box,R);

		// ********** test zero length ray just inside box

		dGeomRaySetLength (ray,0);
		for (j=0; j<3; j++) q.v[j] = (dRandReal()-0.5)*s.v[j];
		i = dRandInt (3);
		if (dRandReal() > 0.5) q.v[i] = 0.99*0.5*s.v[i]; else q.v[i] = -0.99*0.5*s.v[i];
		dMultiply0 (q2,dGeomGetRotation(box),q,3,3,1);
		for (j=0; j<3; j++) q2.v[j] += p.v[j];
		dGeomSetPosition (ray,q2.v[0],q2.v[1],q2.v[2]);
		dRFromAxisAndAngle (R,dRandReal()*2-1,dRandReal()*2-1,
				dRandReal()*2-1,dRandReal()*10-5);
		dGeomSetRotation (ray,R);
		if (dCollide (ray,box,1,contacts) != 0) if (testFAILED()) return false;

		// ********** test zero length ray just outside box

		dGeomRaySetLength (ray,0);
		for (j=0; j<3; j++) q.v[j] = (dRandReal()-0.5)*s.v[j];
		i = dRandInt (3);
		if (dRandReal() > 0.5) q.v[i] = 1.01*0.5*s.v[i]; else q.v[i] = -1.01*0.5*s.v[i];
		dMultiply0 (q2,dGeomGetRotation(box),q,3,3,1);
		for (j=0; j<3; j++) q2.v[j] += p.v[j];
		dGeomSetPosition (ray,q2.v[0],q2.v[1],q2.v[2]);
		dRFromAxisAndAngle (R,dRandReal()*2-1,dRandReal()*2-1,
				dRandReal()*2-1,dRandReal()*10-5);
		dGeomSetRotation (ray,R);
		if (dCollide (ray,box,1,contacts) != 0) if (testFAILED()) return false;

		// ********** test finite length ray totally contained inside the box

		for (j=0; j<3; j++) q.v[j] = (dRandReal()-0.5)*0.99*s.v[j];
		dMultiply0 (q2,dGeomGetRotation(box),q,3,3,1);
		for (j=0; j<3; j++) q2.v[j] += p.v[j];
		for (j=0; j<3; j++) q3.v[j] = (dRandReal()-0.5)*0.99*s.v[j];
		dMultiply0 (q4,dGeomGetRotation(box),q3,3,3,1);
		for (j=0; j<3; j++) q4.v[j] += p.v[j];
		for (j=0; j<3; j++) n.v[j] = q4.v[j] - q2.v[j];
		dNormalize3 (n);
		dGeomRaySet (ray,q2.v[0],q2.v[1],q2.v[2],n.v[0],n.v[1],n.v[2]);
		dGeomRaySetLength (ray,dDISTANCE(q2,q4));
		if (dCollide (ray,box,1,contacts) != 0) if (testFAILED()) return false;

		// ********** test finite length ray totally outside the box

		for (j=0; j<3; j++) q.v[j] = (dRandReal()-0.5)*s.v[j];
		i = dRandInt (3);
		if (dRandReal() > 0.5) q.v[i] = 1.01*0.5*s.v[i]; else q.v[i] = -1.01*0.5*s.v[i];
		dMultiply0 (q2,dGeomGetRotation(box),q,3,3,1);
		for (j=0; j<3; j++) q3.v[j] = q2.v[j] + p.v[j];
		dNormalize3 (q2);
		dGeomRaySet (ray,q3.v[0],q3.v[1],q3.v[2],q2.v[0],q2.v[1],q2.v[2]);
		dGeomRaySetLength (ray,10);
		if (dCollide (ray,box,1,contacts) != 0) if (testFAILED()) return false;

		// ********** test ray from outside to just above surface

		for (j=0; j<3; j++) q.v[j] = (dRandReal()-0.5)*s.v[j];
		i = dRandInt (3);
		if (dRandReal() > 0.5) q.v[i] = 1.01*0.5*s.v[i]; else q.v[i] = -1.01*0.5*s.v[i];
		dMultiply0 (q2,dGeomGetRotation(box),q,3,3,1);
		for (j=0; j<3; j++) q3.v[j] = 2*q2.v[j] + p.v[j];
		k = dSqrt(q2.v[0]*q2.v[0] + q2.v[1]*q2.v[1] + q2.v[2]*q2.v[2]);
		for (j=0; j<3; j++) q2.v[j] = -q2.v[j];
		dGeomRaySet (ray,q3.v[0],q3.v[1],q3.v[2],q2.v[0],q2.v[1],q2.v[2]);
		dGeomRaySetLength (ray,k*0.99);
		if (dCollide (ray,box,1,contacts) != 0) if (testFAILED()) return false;

		// ********** test ray from outside to just below surface

		dGeomRaySetLength (ray,k*1.01);
		if (dCollide (ray,box,1,contacts) != 1) if (testFAILED()) return false;

		// ********** test contact point position for random rays

		for (j=0; j<3; j++) q.v[j] = dRandReal()*s.v[j];
		dMultiply0 (q2,dGeomGetRotation(box),q,3,3,1);
		for (j=0; j<3; j++) q2.v[j] += p.v[j];
		for (j=0; j<3; j++) q3.v[j] = dRandReal()-0.5;
		dNormalize3 (q3);
		dGeomRaySet (ray,q2.v[0],q2.v[1],q2.v[2],q3.v[0],q3.v[1],q3.v[2]);
		dGeomRaySetLength (ray,10);
		if (dCollide (ray,box,1,contacts)!=0) {
			DContactGeom contact = contacts.get(0);
			// check depth of contact point
			if (dFabs (dGeomBoxPointDepth
					(box,contact.pos.v[0],contact.pos.v[1],contact.pos.v[2])) > tol)
				if (testFAILED()) return false;
			// check position of contact point
			for (j=0; j<3; j++) contact.pos.v[j] -= p.v[j];
			dMultiply1 (q,dGeomGetRotation(box),contact.pos);//,3,3,1); TZ TODO check (?)
			if ( dFabs(dFabs (q.v[0]) - 0.5*s.v[0]) > tol &&
					dFabs(dFabs (q.v[1]) - 0.5*s.v[1]) > tol &&
					dFabs(dFabs (q.v[2]) - 0.5*s.v[2]) > tol) {
				if (testFAILED()) return false;
			}
			// also check normal signs
			if (dDOT (q3,contact.normal) > 0) if (testFAILED()) return false;

			draw_all_objects (space);
		}

		return retPASSED();
	}


	private boolean test_ray_and_ccylinder()
	{
		int j;
		DContactGeomBuffer contacts = new DContactGeomBuffer(1);
		DVector3 p=new DVector3(),a=new DVector3(),b=new DVector3(),n=new DVector3();
		DMatrix3 R=new DMatrix3();
		double r,l,k,x,y;

		DSimpleSpace space = dSimpleSpaceCreate(null);

		DRay ray = dCreateRay (null,0);
		DCapsule ccyl = dCreateCapsule (null,1,1);
		dSpaceAdd (space,ray);
		dSpaceAdd (space,ccyl);

		// ********** make a random capped cylinder

		r = dRandReal()*0.5 + 0.01;
		l = dRandReal()*1 + 0.01;
		dGeomCapsuleSetParams (ccyl,r,l);
		dMakeRandomVector (p,1.0);
		dGeomSetPosition (ccyl,p.v[0],p.v[1],p.v[2]);
		dRFromAxisAndAngle (R,dRandReal()*2-1,dRandReal()*2-1,
				dRandReal()*2-1,dRandReal()*10-5);
		dGeomSetRotation (ccyl,R);

		// ********** test ray completely within ccyl

		for (j=0; j<3; j++) a.v[j] = dRandReal()-0.5;
		dNormalize3 (a);
		k = (dRandReal()-0.5)*l;
		for (j=0; j<3; j++) a.v[j] = p.v[j] + r*0.99*a.v[j] + k*0.99*R.v[j*4+2];
		for (j=0; j<3; j++) b.v[j] = dRandReal()-0.5;
		dNormalize3 (b);
		k = (dRandReal()-0.5)*l;
		for (j=0; j<3; j++) b.v[j] = p.v[j] + r*0.99*b.v[j] + k*0.99*R.v[j*4+2];
		dGeomRaySetLength (ray,dDISTANCE(a,b));
		for (j=0; j<3; j++) b.v[j] -= a.v[j];
		dNormalize3 (b);
		dGeomRaySet (ray,a.v[0],a.v[1],a.v[2],b.v[0],b.v[1],b.v[2]);
		if (dCollide (ray,ccyl,1,contacts) != 0) if (testFAILED()) return false;

		// ********** test ray outside ccyl that just misses (between caps)

		k = dRandReal()*2*M_PI;
		x = sin(k);
		y = cos(k);
		for (j=0; j<3; j++) a.v[j] = x*R.v[j*4+0] + y*R.v[j*4+1];
		k = (dRandReal()-0.5)*l;
		for (j=0; j<3; j++) b.v[j] = -a.v[j]*r*2 + k*R.v[j*4+2] + p.v[j];
		dGeomRaySet (ray,b.v[0],b.v[1],b.v[2],a.v[0],a.v[1],a.v[2]);
		dGeomRaySetLength (ray,r*0.99);
		if (dCollide (ray,ccyl,1,contacts) != 0) if (testFAILED()) return false;

		// ********** test ray outside ccyl that just hits (between caps)

		dGeomRaySetLength (ray,r*1.01);
		if (dCollide (ray,ccyl,1,contacts) != 1) if (testFAILED()) return false;
		// check depth of contact point
		if (dFabs (dGeomCapsulePointDepth
				(ccyl,contacts.get(0).pos.v[0],contacts.get(0).pos.v[1],contacts.get(0).pos.v[2])) > tol)
			if (testFAILED()) return false;

		// ********** test ray outside ccyl that just misses (caps)

		for (j=0; j<3; j++) a.v[j] = dRandReal()-0.5;
		dNormalize3 (a);
		if (dDOT14(a,R,2) < 0) {
			for (j=0; j<3; j++) b.v[j] = p.v[j] - a.v[j]*2*r + l*0.5*R.v[j*4+2];
		}
		else {
			for (j=0; j<3; j++) b.v[j] = p.v[j] - a.v[j]*2*r - l*0.5*R.v[j*4+2];
		}
		dGeomRaySet (ray,b.v[0],b.v[1],b.v[2],a.v[0],a.v[1],a.v[2]);
		dGeomRaySetLength (ray,r*0.99);
		if (dCollide (ray,ccyl,1,contacts) != 0) if (testFAILED()) return false;

		// ********** test ray outside ccyl that just hits (caps)

		dGeomRaySetLength (ray,r*1.01);
		if (dCollide (ray,ccyl,1,contacts) != 1) if (testFAILED()) return false;
		// check depth of contact point
		if (dFabs (dGeomCapsulePointDepth
				(ccyl,contacts.get(0).pos.v[0],contacts.get(0).pos.v[1],contacts.get(0).pos.v[2])) > tol)
			if (testFAILED()) return false;

		// ********** test random rays

		for (j=0; j<3; j++) a.v[j] = dRandReal()-0.5;
		for (j=0; j<3; j++) n.v[j] = dRandReal()-0.5;
		dNormalize3 (n);
		dGeomRaySet (ray,a.v[0],a.v[1],a.v[2],n.v[0],n.v[1],n.v[2]);
		dGeomRaySetLength (ray,10);

		if (dCollide (ray,ccyl,1,contacts)!=0) {
			// check depth of contact point
			if (dFabs (dGeomCapsulePointDepth
					(ccyl,contacts.get(0).pos.v[0],contacts.get(0).pos.v[1],contacts.get(0).pos.v[2])) > tol)
				if (testFAILED()) return false;

			// check normal signs
			if (dDOT (n,contacts.get(0).normal) > 0) if (testFAILED()) return false;

			draw_all_objects (space);
		}

		return retPASSED();
	}


	private boolean test_ray_and_plane()
	{
		int j;
		DContactGeomBuffer contacts = new DContactGeomBuffer(1);
		DVector3 n=new DVector3(),p=new DVector3(),q=new DVector3(),a=new DVector3(),
			b=new DVector3(),g=new DVector3(),h=new DVector3();		// n,d = plane parameters
		DMatrix3 R=new DMatrix3();
		double d;
		
		DSimpleSpace space = dSimpleSpaceCreate(null);

		DRay ray = dCreateRay (null,0);
		DPlane plane = dCreatePlane (null,0,0,1,0);
		dSpaceAdd (space,ray);
		dSpaceAdd (space,plane);

		// ********** make a random plane

		for (j=0; j<3; j++) n.v[j] = dRandReal() - 0.5;
		dNormalize3 (n);
		d = dRandReal() - 0.5;
		dGeomPlaneSetParams (plane,n.v[0],n.v[1],n.v[2],d);
		dPlaneSpace (n,p,q);

		// ********** test finite length ray below plane

		dGeomRaySetLength (ray,0.09);
		a.v[0] = dRandReal()-0.5;
		a.v[1] = dRandReal()-0.5;
		a.v[2] = -dRandReal()*0.5 - 0.1;
		for (j=0; j<3; j++) b.v[j] = a.v[0]*p.v[j] + a.v[1]*q.v[j] + (a.v[2]+d)*n.v[j];
		dGeomSetPosition (ray,b.v[0],b.v[1],b.v[2]);
		dRFromAxisAndAngle (R,dRandReal()*2-1,dRandReal()*2-1,
				dRandReal()*2-1,dRandReal()*10-5);
		dGeomSetRotation (ray,R);
		if (dCollide (ray,plane,1,contacts) != 0) if (testFAILED()) return false;

		// ********** test finite length ray above plane

		a.v[0] = dRandReal()-0.5;
		a.v[1] = dRandReal()-0.5;
		a.v[2] = dRandReal()*0.5 + 0.01;
		for (j=0; j<3; j++) b.v[j] = a.v[0]*p.v[j] + a.v[1]*q.v[j] + (a.v[2]+d)*n.v[j];
		g.v[0] = dRandReal()-0.5;
		g.v[1] = dRandReal()-0.5;
		g.v[2] = dRandReal() + 0.01;
		for (j=0; j<3; j++) h.v[j] = g.v[0]*p.v[j] + g.v[1]*q.v[j] + g.v[2]*n.v[j];
		dNormalize3 (h);
		dGeomRaySet (ray,b.v[0],b.v[1],b.v[2],h.v[0],h.v[1],h.v[2]);
		dGeomRaySetLength (ray,10);
		if (dCollide (ray,plane,1,contacts) != 0) if (testFAILED()) return false;

		// ********** test finite length ray that intersects plane

		a.v[0] = dRandReal()-0.5;
		a.v[1] = dRandReal()-0.5;
		a.v[2] = dRandReal()-0.5;
		for (j=0; j<3; j++) b.v[j] = a.v[0]*p.v[j] + a.v[1]*q.v[j] + (a.v[2]+d)*n.v[j];
		g.v[0] = dRandReal()-0.5;
		g.v[1] = dRandReal()-0.5;
		g.v[2] = dRandReal()-0.5;
		for (j=0; j<3; j++) h.v[j] = g.v[0]*p.v[j] + g.v[1]*q.v[j] + g.v[2]*n.v[j];
		dNormalize3 (h);
		dGeomRaySet (ray,b.v[0],b.v[1],b.v[2],h.v[0],h.v[1],h.v[2]);
		dGeomRaySetLength (ray,10);
		if (dCollide (ray,plane,1,contacts)!=0) {
			// test that contact is on plane surface
			if (dFabs (dDOT(contacts.get(0).pos,n) - d) > tol) if (testFAILED()) return false;
			// also check normal signs
			if (dDOT (h,contacts.get(0).normal) > 0) if (testFAILED()) return false;
			// also check contact point depth
			if (dFabs (dGeomPlanePointDepth
					(plane,contacts.get(0).pos.v[0],contacts.get(0).pos.v[1],contacts.get(0).pos.v[2])) > tol)
				if (testFAILED()) return false;

			draw_all_objects (space);
		}

		// ********** test ray that just misses

		for (j=0; j<3; j++) b.v[j] = (1+d)*n.v[j];
		for (j=0; j<3; j++) h.v[j] = -n.v[j];
		dGeomRaySet (ray,b.v[0],b.v[1],b.v[2],h.v[0],h.v[1],h.v[2]);
		dGeomRaySetLength (ray,0.99);
		if (dCollide (ray,plane,1,contacts) != 0) if (testFAILED()) return false;

		// ********** test ray that just hits

		dGeomRaySetLength (ray,1.01);
		if (dCollide (ray,plane,1,contacts) != 1) if (testFAILED()) return false;

		// ********** test polarity with typical ground plane

		dGeomPlaneSetParams (plane,0,0,1,0);
		for (j=0; j<3; j++) a.v[j] = 0.1;
		for (j=0; j<3; j++) b.v[j] = 0;
		a.v[2] = 1;
		b.v[2] = -1;
		dGeomRaySet (ray,a.v[0],a.v[1],a.v[2],b.v[0],b.v[1],b.v[2]);
		dGeomRaySetLength (ray,2);
		if (dCollide (ray,plane,1,contacts) != 1) if (testFAILED()) return false;
		if (dFabs (contacts.get(0).depth - 1) > tol) if (testFAILED()) return false;
		a.v[2] = -1;
		b.v[2] = 1;
		dGeomRaySet (ray,a.v[0],a.v[1],a.v[2],b.v[0],b.v[1],b.v[2]);
		if (dCollide (ray,plane,1,contacts) != 1) if (testFAILED()) return false;
		if (dFabs (contacts.get(0).depth - 1) > tol) if (testFAILED()) return false;

		return retPASSED();
	}

	//****************************************************************************
	// a really inefficient, but hopefully correct implementation of
	// dBoxTouchesBox(), that does 144 edge-face tests.

	// return 1 if edge v1 -> v2 hits the rectangle described by p1,p2,p3

	private static boolean edgeIntersectsRect (DVector3 v1, DVector3 v2,
			DVector3 p1, DVector3 p2, DVector3 p3)
	{
		int k;
		DVector3 u1=new DVector3(),u2=new DVector3(),n=new DVector3(),tmp=new DVector3();
		//for (k=0; k<3; k++) u1[k] = p3[k]-p1[k];
		u1.eqDiff(p3, p1);
		//for (k=0; k<3; k++) u2[k] = p2[k]-p1[k];
		u2.eqDiff(p2, p1);
		double d1 = dSqrt(dDOT(u1,u1));
		double d2 = dSqrt(dDOT(u2,u2));
		dNormalize3 (u1);
		dNormalize3 (u2);
		if (dFabs(dDOT(u1,u2)) > 1e-6) dDebug (0,"bad u1/u2");
		dCROSS (n,OP.EQ,u1,u2);
		//for (k=0; k<3; k++) tmp[k] = v2[k]-v1[k];
		tmp.eqDiff(v2, v1);
		double d = -dDOT(n,p1);
		if (dFabs(dDOT(n,p1)+d) > 1e-8) dDebug (0,"bad n wrt p1");
		if (dFabs(dDOT(n,p2)+d) > 1e-8) dDebug (0,"bad n wrt p2");
		if (dFabs(dDOT(n,p3)+d) > 1e-8) dDebug (0,"bad n wrt p3");
		double alpha = -(d+dDOT(n,v1))/dDOT(n,tmp);
		//for (k=0; k<3; k++) tmp[k] = v1[k]+alpha*(v2[k]-v1[k]);
		tmp.eqDiff(v2, v1);
		tmp.eqSum(v1, tmp.scale(alpha));
		if (dFabs(dDOT(n,tmp)+d) > 1e-6) dDebug (0,"bad tmp");
		if (alpha < 0) return false;
		if (alpha > 1) return false;
		//for (k=0; k<3; k++) tmp[k] -= p1[k];
		tmp.set(p1).scale(-1);
		double a1 = dDOT(u1,tmp);
		double a2 = dDOT(u2,tmp);
		if (a1<0 || a2<0 || a1>d1 || a2>d2) return false;
		return true;
	}


	// return 1 if box 1 is completely inside box 2

	private static boolean box1inside2 (final DVector3 p1, final DMatrix3 R1,
			final DVector3 side1, final DVector3 p2,
			final DMatrix3 R2, final DVector3 side2)
	{
		for (int i=-1; i<=1; i+=2) {
			for (int j=-1; j<=1; j+=2) {
				for (int k=-1; k<=1; k+=2) {
					DVector3 v=new DVector3(),vv=new DVector3();
					v.v[0] = i*0.5*side1.v[0];
					v.v[1] = j*0.5*side1.v[1];
					v.v[2] = k*0.5*side1.v[2];
					dMULTIPLY0_331 (vv,R1,v);
					vv.v[0] += p1.v[0] - p2.v[0];
					vv.v[1] += p1.v[1] - p2.v[1];
					vv.v[2] += p1.v[2] - p2.v[2];
					for (int axis=0; axis < 3; axis++) {
						double z = dDOT14(vv,R2,axis);
						if (z < (-side2.v[axis]*0.5) || z > (side2.v[axis]*0.5)) return false;
					}
				}
			}
		}
		return true;
	}


	// test if any edge from box 1 hits a face from box 2

	private static boolean testBoxesTouch2 (final DVector3 p1, final DMatrix3 R1,
			final DVector3 side1, final DVector3 p2,
			final DMatrix3 R2, final DVector3 side2)
	{
		int j,k,j1,j2;

		// for 6 faces from box 2
		for (int fd=0; fd<3; fd++) {		// direction for face

			for (int fo=0; fo<2; fo++) {	// offset of face
				// get four points on the face. first get 2 indexes that are not fd
				int k1=0,k2=0;
				if (fd==0) { k1 = 1; k2 = 2; }
				if (fd==1) { k1 = 0; k2 = 2; }
				if (fd==2) { k1 = 0; k2 = 1; }
				DVector3 fp[]=new DVector3[4],tmp=new DVector3();
				for (int z=0; z < fp.length; z++) fp[z] = new DVector3();
				k=0;
				for (j1=-1; j1<=1; j1+=2) {
					for (j2=-1; j2<=1; j2+=2) {
						fp[k].v[k1] = j1;
						fp[k].v[k2] = j2;
						fp[k].v[fd] = fo*2-1;
						k++;
					}
				}
				for (j=0; j<4; j++) {
					for (k=0; k<3; k++) fp[j].v[k] *= 0.5*side2.v[k];
					dMULTIPLY0_331 (tmp,R2,fp[j]);
					for (k=0; k<3; k++) fp[j].v[k] = tmp.v[k] + p2.v[k];
				}

				// for 8 vertices
				double[] v1=new double[3];
				for (v1[0]=-1; v1[0] <= 1; v1[0] += 2) {
					for (v1[1]=-1; v1[1] <= 1; v1[1] += 2) {
						for (v1[2]=-1; v1[2] <= 1; v1[2] += 2) {
							// for all possible +ve leading edges from those vertices
							for (int ei=0; ei < 3; ei ++) {
								if (v1[ei] < 0) {
									// get vertex1 -> vertex2 = an edge from box 1
									DVector3 vv1=new DVector3(),vv2=new DVector3();
									for (k=0; k<3; k++) vv1.v[k] = v1[k] * 0.5*side1.v[k];
									for (k=0; k<3; k++) vv2.v[k] = (v1[k] + (k==ei?1:0)*2)*0.5*side1.v[k];
									DVector3 vertex1=new DVector3(),vertex2=new DVector3();
									dMULTIPLY0_331 (vertex1,R1,vv1);
									dMULTIPLY0_331 (vertex2,R1,vv2);
									//for (k=0; k<3; k++) vertex1[k] += p1[k];
									vertex1.add(p1);
									//for (k=0; k<3; k++) vertex2[k] += p1[k];
									vertex2.add(p1);

									// see if vertex1 -> vertex2 interesects face
									if (edgeIntersectsRect (vertex1,vertex2,fp[0],fp[1],fp[2]))
										return true;
								}
							}
						}
					}
				}
			}
		}

		if (box1inside2 (p1,R1,side1,p2,R2,side2)) return true;
		if (box1inside2 (p2,R2,side2,p1,R1,side1)) return true;

		return false;
	}

	//****************************************************************************
	// dBoxTouchesBox() test

	private boolean test_dBoxTouchesBox()
	{
		int k;
		boolean bt1,bt2;
		DVector3 p1=new DVector3(),p2=new DVector3(),side1=new DVector3(),side2=new DVector3();
		DMatrix3 R1=new DMatrix3(),R2=new DMatrix3();
		
		DSimpleSpace space = dSimpleSpaceCreate(null);

		DBox box1 = dCreateBox (null,1,1,1);
		dSpaceAdd (space,box1);
		DBox box2 = dCreateBox (null,1,1,1);
		dSpaceAdd (space,box2);

		dMakeRandomVector (p1,0.5);
		dMakeRandomVector (p2,0.5);
		for (k=0; k<3; k++) side1.v[k] = dRandReal() + 0.01;
		for (k=0; k<3; k++) side2.v[k] = dRandReal() + 0.01;
		dRFromAxisAndAngle (R1,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
				dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
		dRFromAxisAndAngle (R2,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
				dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);

		dGeomBoxSetLengths (box1,side1.v[0],side1.v[1],side1.v[2]);
		dGeomBoxSetLengths (box2,side2.v[0],side2.v[1],side2.v[2]);
		dGeomSetPosition (box1,p1.v[0],p1.v[1],p1.v[2]);
		dGeomSetRotation (box1,R1);
		dGeomSetPosition (box2,p2.v[0],p2.v[1],p2.v[2]);
		dGeomSetRotation (box2,R2);
		draw_all_objects (space);

		boolean t1 = testBoxesTouch2 (p1,R1,side1,p2,R2,side2);
		boolean t2 = testBoxesTouch2 (p2,R2,side2,p1,R1,side1);
		bt1 = t1 || t2;
		bt2 = dBoxTouchesBox (p1,R1,side1,p2,R2,side2);

		if (bt1 != bt2) if (testFAILED()) return false;

		/*
    // some more debugging info if necessary
    if (bt1 && bt2) printf ("agree - boxes touch\n");
    if (!bt1 && !bt2) printf ("agree - boxes don't touch\n");
    if (bt1 && !bt2) printf ("disagree - boxes touch but dBoxTouchesBox "
			     "says no\n");
    if (!bt1 && bt2) printf ("disagree - boxes don't touch but dBoxTouchesBox "
			     "says yes\n");
		 */

		return retPASSED();
	}

	//****************************************************************************
	// test box-box collision

	private boolean test_dBoxBox()
	{
		int k,bt;
		DVector3 p1=new DVector3(),p2=new DVector3(),side1=new DVector3(),side2=new DVector3(),
			normal=new DVector3(),normal2=new DVector3();
		DMatrix3 R1=new DMatrix3(),R2=new DMatrix3();
		RefDouble depth=new RefDouble(),depth2=new RefDouble();
		RefInt code = new RefInt();
		//dContactGeom contact[48];
		DContactGeomBuffer contacts = new DContactGeomBuffer(48);
		
		DSimpleSpace space = dSimpleSpaceCreate(null);

		DBox box1 = dCreateBox (null,1,1,1);
		dSpaceAdd (space,box1);
		DBox box2 = dCreateBox (null,1,1,1);
		dSpaceAdd (space,box2);

		dMakeRandomVector (p1,0.5);
		dMakeRandomVector (p2,0.5);
		for (k=0; k<3; k++) side1.v[k] = dRandReal() + 0.01;
		for (k=0; k<3; k++) side2.v[k] = dRandReal() + 0.01;

		dRFromAxisAndAngle (R1,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
				dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);
		dRFromAxisAndAngle (R2,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
				dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);

		// dRSetIdentity (R1);	// we can also try this
		// dRSetIdentity (R2);

		dGeomBoxSetLengths (box1,side1.v[0],side1.v[1],side1.v[2]);
		dGeomBoxSetLengths (box2,side2.v[0],side2.v[1],side2.v[2]);
		dGeomSetPosition (box1,p1.v[0],p1.v[1],p1.v[2]);
		dGeomSetRotation (box1,R1);
		dGeomSetPosition (box2,p2.v[0],p2.v[1],p2.v[2]);
		dGeomSetRotation (box2,R2);

		code.set(0); //= 0;
		depth.set(0);// = 0;
		bt = dBoxBox (p1,R1,side1,p2,R2,side2,normal,depth,code,8,contacts);
				//sizeof(dContactGeom));
		if (bt==1) {
			p2.v[0] += normal.v[0] * 0.96 * depth.get();
			p2.v[1] += normal.v[1] * 0.96 * depth.get();
			p2.v[2] += normal.v[2] * 0.96 * depth.get();
			bt = dBoxBox (p1,R1,side1,p2,R2,side2,normal2,depth2,code,8,contacts);
					//sizeof(dContactGeom));

			/*
    dGeomSetPosition (box2,p2[0],p2[1],p2[2]);
    draw_all_objects (space);
			 */

			if (bt != 1) {
				if (testFAILED()) return false;
				dGeomSetPosition (box2,p2.v[0],p2.v[1],p2.v[2]);
				draw_all_objects (space);
			}

			p2.v[0] += normal.v[0] * 0.08 * depth.get();
			p2.v[1] += normal.v[1] * 0.08 * depth.get();
			p2.v[2] += normal.v[2] * 0.08 * depth.get();
			bt = dBoxBox (p1,R1,side1,p2,R2,side2,normal2,depth2,code,8,contacts);
					//sizeof(dContactGeom));
			if (bt != 0) if (testFAILED()) return false;

			// dGeomSetPosition (box2,p2[0],p2[1],p2[2]);
			// draw_all_objects (space);
		}

		// printf ("code=%2d  depth=%.4f  ",code,depth);

		return retPASSED();
	}

	//****************************************************************************
	// graphics

	private boolean space_pressed = false;


	private static float[] xyz = {2.4807f,-1.8023f,2.7600f};
	private static float[] hpr = {141.5000f,-18.5000f,0.0000f};
	// start simulation - set viewpoint

	public void start()
	{
		dAllocateODEDataForThread(OdeConstants.dAllocateMaskAll);

		//  static float xyz[3] = {2.4807,-1.8023,2.7600};
		//  static float hpr[3] = {141.5000,-18.5000,0.0000};
		dsSetViewpoint (xyz,hpr);
	}


	// called when a key pressed

	public void command (char cmd)
	{
		if (cmd == ' ') space_pressed = true;
	}


	// simulation loop

	public void simLoop (boolean pause)
	{
		do {
			draw_all_objects_called = false;
			//unsigned TODO fAbs() ?
			long seed = dRandGetSeed();
			testslot[graphical_test].test_fn();
			if (draw_all_objects_called) {
				if (space_pressed) space_pressed = false; else dRandSetSeed (seed);
			}
		}
		while (!draw_all_objects_called);
	}

	//****************************************************************************
	// do all the tests

	private void do_tests (String[] args)
	{
		int i,j;

		// process command line arguments
		if (args.length >= 2) {
			graphical_test = atoi (args[1]);
		}

		if (graphical_test!=0) {
			// do one test gaphically and interactively

			if (graphical_test < 1 || graphical_test >= MAX_TESTS ||
					testslot[graphical_test].name==null) {
				dError (0,"invalid test number");
			}

			printf ("performing test: %s\n",testslot[graphical_test].name);

			// setup pointers to drawstuff callback functions
			dsFunctions fn = this;
			fn.version = DS_VERSION;
			//    fn.start = &start;
			//    fn.step = &simLoop;
			//    fn.command = &command;
			//    fn.stop = 0;
			fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

			dsSetSphereQuality (3);
			dsSetCapsuleQuality (8);
			dsSimulationLoop (args,1280,900,fn);
		}
		else {
			// do all tests noninteractively

			for (i=0; i<MAX_TESTS; i++) testslot[i].number = i;

			// first put the active tests into a separate array
			int n=0;
			for (i=0; i<MAX_TESTS; i++) if (testslot[i].name!=null) n++;
			//TestSlot **ts = (TestSlot**) malloc (n * sizeof(TestSlot*));
			TestSlot[] ts = new TestSlot[n];
			j = 0;
			for (i=0; i<MAX_TESTS; i++) if (testslot[i].name!=null) ts[j++] = testslot[i];//+i;
			if (j != n) dDebug (0,"internal");

			// do two test batches. the first test batch has far fewer reps and will
			// catch problems quickly. if all tests in the first batch passes, the
			// second batch is run.

			for (i=0; i<n; i++) ts[i].failcount = 0;
			int total_reps=0;
			for (int batch=0; batch<2; batch++) {
				int reps = (batch==0) ? TEST_REPS1 : TEST_REPS2;
				total_reps += reps;
				printf ("testing batch %d (%d reps)...\n",batch+1,reps);

				// run tests
				for (j=0; j<reps; j++) {
					for (i=0; i<n; i++) {
						current_test = ts[i].number;
						if (ts[i].test_fn() != true) ts[i].failcount++;
					}
				}

				// check for failures
				int total_fail_count=0;
				for (i=0; i<n; i++) total_fail_count += ts[i].failcount;
				if (total_fail_count!=0) break;
			}

			// print results
			for (i=0; i<n; i++) {
				printf ("%3d: %-30s: ",ts[i].number,ts[i].name);
				if (ts[i].failcount!=0) {
					printf ("FAILED (%.2f%%) at line %d\n",
							((double)ts[i].failcount)/((double)total_reps)*100.0,
							ts[i].last_failed_line);
				}
				else {
					printf ("ok\n");
				}
			}
		}
	}

	//****************************************************************************

	public static void main(String[] args) {
		new DemoCollision().setup(args);
	}
	
	private void setup(String[] args) {
		// setup all tests

		//memset (testslot,0,sizeof(testslot));
		for (int i = 0; i < MAX_TESTS; i++) testslot[i] = new TestSlot();

		dInitODE2(0);

		MAKE_TEST(1,"test_sphere_point_depth");
		MAKE_TEST(2,"test_box_point_depth");
		MAKE_TEST(3,"test_ccylinder_point_depth");
		MAKE_TEST(4,"test_plane_point_depth");

		MAKE_TEST(10,"test_ray_and_sphere");
		MAKE_TEST(11,"test_ray_and_box");
		MAKE_TEST(12,"test_ray_and_ccylinder");
		MAKE_TEST(13,"test_ray_and_plane");

		MAKE_TEST(100,"test_dBoxTouchesBox");
		MAKE_TEST(101,"test_dBoxBox");

		do_tests (args);
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