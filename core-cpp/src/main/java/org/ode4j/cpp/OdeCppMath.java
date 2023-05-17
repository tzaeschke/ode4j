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
package org.ode4j.cpp;

import org.ode4j.cpp.internal.ApiCppMathMatrix;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector4;
import org.ode4j.ode.OdeMath;

/**
 * From OdeMath.java.
 *
 */
@Deprecated // To be removed in 0.6.0
public abstract class OdeCppMath extends ApiCppMathMatrix {

	//ODE_API 
	public static boolean dSafeNormalize3 (DVector3 a) {
		return OdeMath.dSafeNormalize3(a);
	}

	//ODE_API 
	public static boolean dSafeNormalize4 (DVector4 a) {
		return OdeMath.dSafeNormalize4(a);
	}
	
	/** 
	 * Potentially asserts on zero vec. 
	 * @param a vector
	 */
	//ODE_API 
	public static void dNormalize3 (DVector3 a) {
		OdeMath.dNormalize3(a);
	}
	
	/** 
	 * Potentially asserts on zero vec.
	 * @param a vector 
	 */
	//ODE_API 
	public static void dNormalize4 (DVector4 a) {
		OdeMath.dNormalize4(a);
	}



	/**
	 * given a unit length "normal" vector n, generate vectors p and q vectors
	 * that are an orthonormal basis for the plane space perpendicular to n.
	 * i.e. this makes p,q such that n,p,q are all perpendicular to each other.
	 * q will equal n x p. if n is not unit length then p will be unit length but
	 * q wont be.
	 * @param n n
	 * @param p p
	 * @param q q
	 */
	//ODE_API 
	public static void dPlaneSpace (final DVector3 n, DVector3 p, DVector3 q) {
		OdeMath.dPlaneSpace(n, p, q);
	}

	protected OdeCppMath() {}
}
