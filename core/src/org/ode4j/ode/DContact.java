/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2007-2013 Tilmann Zaeschke     *
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

import org.ode4j.math.DVector3;

/**
 *  contact info used by contact joint.
 */
public class DContact {

	public final DSurfaceParameters surface = new DSurfaceParameters();
	public final DContactGeom geom = new DContactGeom();
	public final DVector3 fdir1 = new DVector3();

	DContact() {
		// Nothing
	}
	
	public class DSurfaceParameters {
		/* must always be defined */
		public int mode;
		public double mu;

		/* only defined if the corresponding flag is set in mode */
		public double mu2;
		public double bounce;
		public double bounce_vel;
		public double soft_erp;
		public double soft_cfm;
		public double motion1,motion2,motionN;
		public double slip1;
		public double slip2;
	}

	public DContactGeom getContactGeom() {
		return geom;
	}
}
