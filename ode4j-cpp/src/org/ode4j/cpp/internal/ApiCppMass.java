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

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DMass;
import org.ode4j.ode.internal.DxMass;


public abstract class ApiCppMass extends ApiCppOdeInit {

	//struct dMass;
	//typedef struct dMass dMass;

	/** Not in the original API.
	 * @author TZ 
	 */
	public static DMass dMassCreate() {
		return new DxMass();
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
	public static int dMassCheck(final DMass m) {
		throw new UnsupportedOperationException();
	}

	//ODE_API 
	public static void dMassSetZero (DMass m) {
		((DxMass)m).dMassSetZero();
	}

	//ODE_API 
	public static void dMassSetParameters (DMass m, double themass,
			double cgx, double cgy, double cgz,
			double I11, double I22, double I33,
			double I12, double I13, double I23) {
		((DxMass)m).dMassSetParameters(themass, cgx, cgy, cgz, I11, I22, I33, I12, I13, I23);
	}

	//ODE_API 
	public static void dMassSetSphere (DMass m, double density, double radius) {
		((DxMass)m).dMassSetSphere(density, radius);
	}
	//ODE_API 
	public static void dMassSetSphereTotal (DMass m, double total_mass, double radius) {
		((DxMass)m).dMassSetSphereTotal(total_mass, radius);
	}

	//ODE_API 
	public static void dMassSetCapsule (DMass m, double density, int direction,
			double radius, double length) {
		((DxMass)m).dMassSetCapsule(density, direction, radius, length);
	}
	//ODE_API 
	public static void dMassSetCapsuleTotal (DMass m, double total_mass, int direction,
			double radius, double length) {
		throw new UnsupportedOperationException();
	}

	//ODE_API 
	public static void dMassSetCylinder (DMass m, double density, int direction,
			double radius, double length) {
		((DxMass)m).dMassSetCylinder(density, direction, radius, length);
	}
	//ODE_API 
	public static void dMassSetCylinderTotal (DMass m, double total_mass, int direction,
			double radius, double length) {
		throw new UnsupportedOperationException();
	}

	//ODE_API 
	public static void dMassSetBox (DMass m, double density,
			double lx, double ly, double lz) {
		((DxMass)m).dMassSetBox(density, lx, ly, lz);
	}
	//ODE_API 
	public static void dMassSetBoxTotal (DMass m, double total_mass,
			double lx, double ly, double lz) {
		throw new UnsupportedOperationException();
	}

	//ODE_API 
	public static void dMassSetTrimesh (DMass m, double density, DGeom g) {
		throw new UnsupportedOperationException();
	}

	//ODE_API 
	public static void dMassSetTrimeshTotal (DMass m, double total_mass, DGeom g) {
		throw new UnsupportedOperationException();
	}

	//ODE_API 
	public static void dMassAdjust (DMass m, double newmass) {
		((DxMass)m).dMassAdjust(newmass);
	}

	//ODE_API 
	public static void dMassTranslate (DMass m, double x, double y, double z) {
		((DxMass)m).dMassTranslate(new DVector3(x, y, z));
	}

	//ODE_API v
	public static void dMassRotate (DMass m, final DMatrix3 R) {
		((DxMass)m).dMassRotate(R);
	}

	//ODE_API 
	public static void dMassAdd (DMass a, final DMass b) {
		((DxMass)a).add(b);
	}

	// Backwards compatible API
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
//		//TODO complete?
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
}