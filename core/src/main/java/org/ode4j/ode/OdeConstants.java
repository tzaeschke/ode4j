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

/**
 * This class serves as a container for many constants used throughout ODE.
 *
 * @author Tilmann Zaeschke
 */
public class OdeConstants {
	/** Use axis dependent friction */
	public static final int 	  dContactMu2		= 0x001;	
	/** Same as above */
	public static final int		  dContactAxisDep   = 0x001;      
	/** Use FDir for the first friction value */
	public static final int 	  dContactFDir1		= 0x002;
	/** Restore collision energy anti-parallel to the normal */
	public static final int 	  dContactBounce	= 0x004;
	/** Don't use global erp for penetration reduction */
	public static final int 	  dContactSoftERP	= 0x008;
	/** Don't use global cfm for penetration constraint */
	public static final int 	  dContactSoftCFM	= 0x010;
	/** Use a non-zero target velocity for the constraint */
	public static final int 	  dContactMotion1	= 0x020;
	public static final int 	  dContactMotion2	= 0x040;
	public static final int 	  dContactMotionN	= 0x080;
	/** Force-dependent slip. */
	public static final int 	  dContactSlip1		= 0x100;
	public static final int 	  dContactSlip2		= 0x200;
	/** Rolling/Angular friction */
	public static final int 	  dContactRolling   = 0x400;

	public static final int 	  dContactApprox0	= 0x0000;
	public static final int 	  dContactApprox1_1	= 0x1000;
	public static final int 	  dContactApprox1_2	= 0x2000;
	/** For rolling friction */
	public static final int		  dContactApprox1_N = 0x4000;
	public static final int		  dContactApprox1   = 0x7000;
	
	
	public static final double dInfinity = Double.POSITIVE_INFINITY;
	
	/**
	 *	Just generate any contacts (disables any contact refining).
	 */
	// #define CONTACTS_UNIMPORTANT			0x80000000
	public static final int CONTACTS_UNIMPORTANT		=	0x80000000;

	
	/** 
	 * Allocate all the possible data that is currently defined
	 * or will be defined in the future.
	 * @deprecated TZ: probably not required. 
	 */
	@Deprecated
    public static final int dAllocateMaskAll = 0xFFFFFFFF; //~0U,

	protected OdeConstants() {}
}
