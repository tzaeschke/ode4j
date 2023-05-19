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
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.math.DVector4;
import org.ode4j.ode.DPlane;

import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.internal.Common.dRecipSqrt;


/**
 * standard ODE geometry primitives: public API and pairwise collision functions.
 *
 * the rule is that only the low level primitive collision functions should set
 * dContactGeom::g1 and dContactGeom::g2.
 */
public class DxPlane extends DxGeom implements DPlane {
	//****************************************************************************
	// plane public API

	//private double[] _p = new double[4];
	private DVector3 _pV = new DVector3();
	private double _pD = 0;

	private void make_sure_plane_normal_has_unit_length ()
	{
//		double l = _p[0]*_p[0] + _p[1]*_p[1] + _p[2]*_p[2];
//		if (l > 0) {
//			l = dRecipSqrt(l);
//			_p[0] *= l;
//			_p[1] *= l;
//			_p[2] *= l;
//			_p[3] *= l;
//		}
//		else {
//			_p[0] = 1;
//			_p[1] = 0;
//			_p[2] = 0;
//			_p[3] = 0;
//		}
		double l = _pV.lengthSquared();
		if (l > 0) {
			l = dRecipSqrt(l);
			_pV.scale( l );
			_pD *= l;
		}
		else {
			_pV.set (1, 0, 0);
			_pD = 0;
		}
	}


	DxPlane (DxSpace space, double a, double b, double c, double d) 
	//dxGeom (space,0)
	{
		super(space, false);
		type = dPlaneClass;
//		_p[0] = a;
//		_p[1] = b;
//		_p[2] = c;
//		_p[3] = d;
		_pV.set( a, b, c);
		_pD = d;
		make_sure_plane_normal_has_unit_length ();
	}


	@Override
    protected void computeAABB()
	{
		_aabb.setToInfinity();

		// Planes that have normal vectors aligned along an axis can use a
		// less comprehensive (half space) bounding box.

		if ( _pV.get1() == 0.0f && _pV.get2() == 0.0f ) {
			// normal aligned with x-axis
			_aabb.setMin0( (_pV.get0() > 0) ? -dInfinity : -_pD );
			_aabb.setMax0( (_pV.get0() > 0) ? _pD : dInfinity ) ;
		} else if ( _pV.get0() == 0.0f && _pV.get2() == 0.0f ) {
			// normal aligned with y-axis
			_aabb.setMin1( (_pV.get1() > 0) ? -dInfinity : -_pD );
			_aabb.setMax1( (_pV.get1() > 0) ? _pD : dInfinity );
		} else if ( _pV.get0() == 0.0f && _pV.get1() == 0.0f ) {
			// normal aligned with z-axis
			_aabb.setMin2( (_pV.get2() > 0) ? -dInfinity : -_pD );
			_aabb.setMax2( (_pV.get2() > 0) ? _pD : dInfinity );
		}
	}


	public static DxPlane dCreatePlane (DxSpace space,
			double a, double b, double c, double d)
	{
		return new DxPlane (space,a,b,c,d);
	}


	//void dGeomPlaneSetParams (dxGeom g, double a, double b, double c, double d)
	public void dGeomPlaneSetParams (double a, double b, double c, double d)
	{
		//		COM.dUASSERT (g!= null && g.type == dPlaneClass,"argument not a plane");
		//		dxPlane p = (dxPlane) g;
//		_p[0] = a;
//		_p[1] = b;
//		_p[2] = c;
//		_p[3] = d;
		_pV.set( a, b, c );
		_pD = d;
		make_sure_plane_normal_has_unit_length ();
		dGeomMoved ();
	}


	//void dGeomPlaneGetParams (dxGeom g, dVector4 result)
	public void dGeomPlaneGetParams (DVector4 result)
	{
		//COM.dUASSERT (g != null&& g.type == dPlaneClass,"argument not a plane");
		//dxPlane p = (dxPlane) g;
		//		result.v[0] = p._p[0];
		//		result.v[1] = p._p[1];
		//		result.v[2] = p._p[2];
		//		result.v[3] = p._p[3];
		result.set(_pV.get0(), _pV.get1(), _pV.get2(), _pD);
	}


	//double dGeomPlanePointDepth (dxGeom g, double x, double y, double z)
	public double dGeomPlanePointDepth (double x, double y, double z)
	{
		//		COM.dUASSERT (g!= null && g.type == dPlaneClass,"argument not a plane");
		//		dxPlane p = (dxPlane) g;
		return _pD - _pV.get0()*x - _pV.get1()*y - _pV.get2()*z;
	}
	@Override
	public double getPointDepth(DVector3C p) {
		return _pD - _pV.dot(p);
	}

	//TZ 
	@Override
	public DVector3C getNormal() {
		return _pV;
	}
	@Override
	public double getDepth() {
		return _pD;
	}
	
	// **********************************
	// API dPlane
	// **********************************

	@Override
	public void setParams (double a, double b, double c, double d)
	{ dGeomPlaneSetParams (a, b, c, d); }
	@Override
	public void setParams (DVector3C abc, double d)
	{ dGeomPlaneSetParams (abc.get0(), abc.get1(), abc.get2(), d); }
	public void getParams (DVector4 result) 
	{ dGeomPlaneGetParams (result); }

	@Override
	public DVector3C getPosition() {
		//TODO
		throw new UnsupportedOperationException(
				"ERROR: getPosition() is not supported for Planes.");
	}
	
	@Override
	public DMatrix3C getRotation() {
		throw new UnsupportedOperationException(
				"ERROR: getRotation() is not supported for Planes.");
	}
}
