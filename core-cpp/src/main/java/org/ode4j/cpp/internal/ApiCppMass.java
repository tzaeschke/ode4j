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
package org.ode4j.cpp.internal;

import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DMassC;
import org.ode4j.ode.DTriMesh;
import org.ode4j.ode.OdeHelper;


public abstract class ApiCppMass extends ApiCppOdeInit {

	/** 
	 * Not in the original API. By TZ. 
	 * @return Mass object
	 */
	public static DMass dMassCreate() {
		return OdeHelper.createMass();
	}

	
	/**
	 * Check if a mass structure has valid value.
	 * The function check if the mass and inertia matrix are positive definits
	 *
	 * @param m A mass structure to check
	 *
	 * @return 1 if both condition are met
	 */
	//ODE_API 
	public static boolean dMassCheck(DMassC m) {
		return m.check();
	}

	//ODE_API 
	public static void dMassSetZero (DMass m) {
		m.setZero();
	}

	//ODE_API 
	public static void dMassSetParameters (DMass m, double themass,
			double cgx, double cgy, double cgz,
			double I11, double I22, double I33,
			double I12, double I13, double I23) {
		m.setParameters(themass, cgx, cgy, cgz, I11, I22, I33, I12, I13, I23);
	}

	//ODE_API 
	public static void dMassSetSphere (DMass m, double density, double radius) {
		m.setSphere(density, radius);
	}
	//ODE_API 
	public static void dMassSetSphereTotal (DMass m, double total, double radius) {
		m.setSphereTotal(total, radius);
	}

	//ODE_API 
	public static void dMassSetCapsule (DMass m, double density, int direction,
			double radius, double length) {
		m.setCapsule(density, direction, radius, length);
	}
	//ODE_API 
	public static void dMassSetCapsuleTotal (DMass m, double total, int direction,
			double radius, double length) {
		m.setCapsuleTotal(total, direction, radius, length);
	}

	//ODE_API 
	public static void dMassSetCylinder (DMass m, double density, int direction,
			double radius, double length) {
		m.setCylinder(density, direction, radius, length);
	}
	//ODE_API 
	public static void dMassSetCylinderTotal (DMass m, double total, int direction,
			double radius, double length) {
		m.setCylinderTotal(total, direction, radius, length);
	}

	//ODE_API 
	public static void dMassSetBox (DMass m, double density,
			double lx, double ly, double lz) {
		m.setBox(density, lx, ly, lz);
	}
	//ODE_API 
	public static void dMassSetBoxTotal (DMass m, double total_mass,
			double lx, double ly, double lz) {
		m.setBoxTotal(total_mass, lx, ly, lz);
	}

	//ODE_API 
	public static void dMassSetTrimesh (DMass m, double density, DTriMesh g) {
		m.setTrimesh(density, g);
	}

	//ODE_API 
	public static void dMassSetTrimeshTotal (DMass m, double total_mass, DTriMesh g) {
		m.setTrimeshTotal(total_mass, g);
	}

	//ODE_API 
	public static void dMassAdjust (DMass m, double newmass) {
		m.adjust(newmass);
	}

	//ODE_API 
	public static void dMassTranslate (DMass m, double x, double y, double z) {
		m.translate(new DVector3(x, y, z));
	}

	//ODE_API v
	public static void dMassRotate (DMass m, DMatrix3C R) {
		m.rotate(R);
	}

	//ODE_API 
	public static void dMassAdd (DMass m, DMassC b) {
		m.add(b);
	}

	// Backwards compatible API (deprecated)
	//#define dMassSetCappedCylinder dMassSetCapsule
	//#define dMassSetCappedCylinderTotal dMassSetCapsuleTotal
//
//	
//	void setZero()
//	{ dMassSetZero (this); }
//	
//	void setParameters (double themass, double cgx, double cgy, double cgz,
//			double I11, double I22, double I33,
//			double I12, double I13, double I23)
//	{ dMassSetParameters (this,themass,cgx,cgy,cgz,I11,I22,I33,I12,I13,I23); }
//	
//	void setSphere (double density, double radius)
//	{ dMassSetSphere (this,density,radius); }
//	
//	void setCapsule (double density, int direction, double a, double b)
//	//  { dMassSetCappedCylinder (this,density,direction,a,b); }
//	{ dMassSetCapsule (this,density,direction,a,b); }
//	
//	void setCappedCylinder (double density, int direction, double a, double b)
//	{ setCapsule(density, direction, a, b); }
//	
//	void setBox (double density, double lx, double ly, double lz)
//	{ dMassSetBox (this,density,lx,ly,lz); }
//	
//	void adjust (double newmass)
//	{ dMassAdjust (this,newmass); }
//	
//	void translate (double x, double y, double z)
//	{ dMassTranslate (this,x,y,z); }
//	
//	void rotate (final dMatrix3 R)
//	{ dMassRotate (this,R); }
//	
//	void add (final dMass b)
//	{ dMassAdd (this,b); }
//	//#endif
//	//};
//
//	public void set(dMass m) {
//		mass = m.mass;
//		c = new dVector4(m.c);
//		I = new dMatrix3(m.I);
//	}
//
//	@Override
//	public String toString() {
//		FormattedStringBuilder b = new FormattedStringBuilder();
//		b.appendln("Mass = " + mass);
//		b.appendln("c = ", c.toString());
//		b.appendln("I = ", I.toString());
//		return b.toString();
//	}

	//#ifdef __cplusplus
	//}
	//#endif
	//
	//#endif

	protected ApiCppMass() {}
}