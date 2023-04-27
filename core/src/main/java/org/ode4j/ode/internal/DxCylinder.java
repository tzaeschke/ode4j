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
package org.ode4j.ode.internal;

import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DCylinder;

import static org.ode4j.ode.OdeMath.*;


/**
 * standard ODE geometry primitives: public API and pairwise collision functions.
 * 
 * the rule is that only the low level primitive collision functions should set
 * dContactGeom::g1 and dContactGeom::g2.
 */
public class DxCylinder extends DxGeom implements DCylinder {

	private double _radius,_lz;        // radius, length along z axis

	//#define dMAX(A,B)  ((A)>(B) ? (A) : (B))
	private final double dMAX(double A, double B) {
		return ((A)>(B) ? (A) : (B));
	}

	// flat cylinder public API

	DxCylinder (DxSpace space, double __radius, double __length)// dxGeom (space,1)
	{
		super(space, true);
		dAASSERT (__radius >= 0 && __length >= 0);
		type = dCylinderClass;
		_radius = __radius;
		_lz = __length;
		//updateZeroSizedFlag(!__radius || !__length);
		updateZeroSizedFlag(_radius == 0.0 || __length == 0.0);
	}


	@Override
    protected void computeAABB()
	{
		final DMatrix3C R = final_posr().R();
		final DVector3C pos = final_posr().pos();

	    double dOneMinusR2Square = (1.0 - R.get02()*R.get02());
	    double xrange = dFabs(R.get02()*_lz*0.5) + _radius * dSqrt(dMAX(0.0, dOneMinusR2Square));
	    double dOneMinusR6Square = (1.0 - R.get12()*R.get12());
	    double yrange = dFabs(R.get12()*_lz*0.5) + _radius * dSqrt(dMAX(0.0, dOneMinusR6Square));
	    double dOneMinusR10Square = (1.0 - R.get22()*R.get22());
	    double zrange = dFabs(R.get22()*_lz*0.5) + _radius * dSqrt(dMAX(0.0, dOneMinusR10Square));

//		_aabb.v[0] = pos.v[0] - xrange;
//		_aabb.v[1] = pos.v[0] + xrange;
//		_aabb.v[2] = pos.v[1] - yrange;
//		_aabb.v[3] = pos.v[1] + yrange;
//		_aabb.v[4] = pos.v[2] - zrange;
//		_aabb.v[5] = pos.v[2] + zrange;
		_aabb.setMinMax(xrange, yrange, zrange);
		_aabb.shiftPos(pos);
	}


	//dGeom dCreateCylinder (dSpace space, double radius, double length)
	public static DxCylinder dCreateCylinder (DxSpace space, double radius, double length)
	{
		return new DxCylinder (space,radius,length);
	}

	//void dGeomCylinderSetParams (dGeom cylinder, double radius, double length)
	void dGeomCylinderSetParams (double radius, double length)
	{
		//	dUASSERT (cylinder && cylinder.type == dCylinderClass,"argument not a ccylinder");
		dAASSERT (radius >= 0 && length >= 0);
		//	dxCylinder *c = (dxCylinder*) cylinder;
		_radius = radius;
		_lz = length;
		//updateZeroSizedFlag(!radius || !length);
		updateZeroSizedFlag(radius==0.0 || length==0.0);
		dGeomMoved ();
	}

//	//void dGeomCylinderGetParams (dGeom cylinder, RefDouble radius, RefDouble length)
//	public void dGeomCylinderGetParams (RefDouble radius, RefDouble length)
//	{
//		//	dUASSERT (cylinder && cylinder.type == dCylinderClass,"argument not a ccylinder");
//		//	dxCylinder *c = (dxCylinder*) cylinder;
//		radius.set(_radius);
//		length.set(_lz);
//	}

	@Override
	public double getRadius() {
		return _radius;
	}

	@Override
	public double getLength() {
		return _lz;
	}

	@Override
	public void setParams (double radius, double length)
	{ dGeomCylinderSetParams (radius, length); }
}

