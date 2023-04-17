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
package org.ode4j.ode;

import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3C;


/**
 * 
 */
public interface DMass extends DMassC {

	/**
	 * Set mass, center and inertia matrix to zero.
	 */
	void setZero();

	/**
	 * Set all parameters of the mass.
	 * @param total_mass total mass
	 * @param cgx center of gravity x
	 * @param cgy center of gravity y
	 * @param cgz center of gravity x
	 * @param I11 intertia matrix 11
	 * @param I22 intertia matrix 22
	 * @param I33 intertia matrix 33
	 * @param I12 intertia matrix 12
	 * @param I13 intertia matrix 13
	 * @param I23 intertia matrix 23
	 */
	void setParameters (double total_mass, double cgx, double cgy, double cgz,
			double I11, double I22, double I33,
			double I12, double I13, double I23);
	
	void setSphere (double density, double radius);
	void setSphereTotal (double total_mass, double radius);

 	void setTrimesh(double density, DTriMesh geom);
 	void setTrimeshTotal(double total_mass, DTriMesh geom);
	
	/** 
	 * @param density density 
	 * @param direction 1=x; 2=y; 3=z 
	 * @param radius radius
	 * @param length length 
	 */
	void setCapsule (double density, int direction, double radius, double length);
	/** 
	 * @param total_mass total mass
	 * @param direction 1=x; 2=y; 3=z 
	 * @param radius radius
	 * @param length length
	 */
	void setCapsuleTotal (double total_mass, int direction, double radius, double length);
	
	/** 
	 * @param density density
	 * @param direction 1=x; 2=y; 3=z 
	 * @param radius radius 
	 * @param length length
	 */
	void setCylinder (double density, int direction, double radius, double length);
	/** 
	 * @param total_mass total mass
	 * @param direction 1=x; 2=y; 3=z 
	 * @param radius radius
	 * @param length length
	 */
	void setCylinderTotal (double total_mass, int direction, double radius, double length);

	void setBox (double density, double lx, double ly, double lz);
	void setBox (double density, DVector3C lxyz);
	void setBoxTotal (double total_mass, double lx, double ly, double lz);
	
	void adjust (double new_total_mass);
	
	void translate (double x, double y, double z);
	void translate(DVector3C c);
	
	void rotate (DMatrix3C R);
	
	void add (DMassC b);

	//by TZ
	@Override
	double getMass();
	void setMass(double total_mass);

	/**
	 * Get center of gravity.
	 * @return center of gravity
	 */
	@Override
	DVector3C getC();
	/**
	 * Set inertia matrix.
	 * @return inertia matrix
	 */
	@Override
	DMatrix3C getI();
	/**
	 * @param c mass center
	 */
	void setC(DVector3C c);
	/**
	 * @param I inertia matrix
	 */
	void setI(DMatrix3C I);

	/**
	 * Check if a mass structure has valid value.
	 * The function check if the mass and inertia matrix are positive definits
	 *
	 * @return <tt>true</tt> if both condition are met
	 */
	@Override
	boolean check();


	
	
	//	/**
//	 * Check if a mass structure has valid value.
//	 * The function check if the mass and innertia matrix are positive definits
//	 *
//	 * @param m A mass structure to check
//	 *
//	 * @return 1 if both codition are met
//	 */
//	//ODE_API 
//	public abstract int dMassCheck(final dMass m);
//
//	//ODE_API 
//	public abstract void dMassSetZero (dMass m);
//
//	//ODE_API 
//	public abstract void dMassSetParameters (dMass m, double themass,
//			double cgx, double cgy, double cgz,
//			double I11, double I22, double I33,
//			double I12, double I13, double I23);
//
//	//ODE_API 
//	public abstract void dMassSetSphere (dMass m, double density, double radius);
//	//ODE_API 
//	public abstract void dMassSetSphereTotal (dMass m, double total_mass, double radius);
//
//	//ODE_API 
//	public abstract void dMassSetCapsule (dMass m, double density, int direction,
//			double radius, double length);
//	//ODE_API 
//	public abstract void dMassSetCapsuleTotal (dMass m, double total_mass, int direction,
//			double radius, double length);
//
//	//ODE_API 
//	public abstract void dMassSetCylinder (dMass m, double density, int direction,
//			double radius, double length);
//	//ODE_API 
//	public abstract void dMassSetCylinderTotal (dMass m, double total_mass, int direction,
//			double radius, double length);
//
//	//ODE_API 
//	public abstract void dMassSetBox (dMass m, double density,
//			double lx, double ly, double lz);
//	//ODE_API 
//	public abstract void dMassSetBoxTotal (dMass m, double total_mass,
//			double lx, double ly, double lz);
//
//	//ODE_API 
//	public abstract void dMassSetTrimesh (dMass m, double density, dxGeom g);
//
//	//ODE_API 
//	public abstract void dMassSetTrimeshTotal (dMass m, double total_mass, dxGeom g);
//
//	//ODE_API 
//	public abstract void dMassAdjust (dMass m, double newmass);
//
//	//ODE_API 
//	public abstract void dMassTranslate (dMass m, double x, double y, double z);
//
//	//ODE_API v
//	public abstract void dMassRotate (dMass m, final dMatrix3 R);
//
//	//ODE_API 
//	public abstract void dMassAdd (dMass a, final dMass b);

	// Backwards compatible API
	//#define dMassSetCappedCylinder dMassSetCapsule
	//#define dMassSetCappedCylinderTotal dMassSetCapsuleTotal


//	protected void setZero()
//	{ dMassSetZero (this); }
//	
//	protected void setParameters (double themass, double cgx, double cgy, double cgz,
//			double I11, double I22, double I33,
//			double I12, double I13, double I23)
//	{ dMassSetParameters (this,themass,cgx,cgy,cgz,I11,I22,I33,I12,I13,I23); }
//	
//	protected void setSphere (double density, double radius)
//	{ dMassSetSphere (this,density,radius); }
//	
//	protected void setCapsule (double density, int direction, double a, double b)
//	//  { dMassSetCappedCylinder (this,density,direction,a,b); }
//	{ dMassSetCapsule (this,density,direction,a,b); }
//	
//	protected void setCappedCylinder (double density, int direction, double a, double b)
//	{ setCapsule(density, direction, a, b); }
//	
//	public void setBox (double density, double lx, double ly, double lz)
//	{ dMassSetBox (this,density,lx,ly,lz); }
//	
//	public void adjust (double newmass)
//	{ dMassAdjust (this,newmass); }
//	
//	protected void translate (double x, double y, double z)
//	{ dMassTranslate (this,x,y,z); }
//	
//	protected void rotate (final dMatrix3 R)
//	{ dMassRotate (this,R); }
//	
//	protected void add (final dMass b)
//	{ dMassAdd (this,b); }
//
//	//by TZ
//	public abstract dVector3 getC();
//
//	//by TZ
//	public abstract double getMass();
//
//	//by TZ
//	public abstract void setMass(double d);
//
//	//by TZ
//	public abstract dMatrix3 getI();
//
//	public abstract void setC(dVector3 c);
//
//	public abstract void setI(dMatrix3 I);
}