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

import org.ode4j.math.DVector3;

/**
 *  contact info used by contact joint.
 */
public class DContact {

	public final DSurfaceParameters surface = new DSurfaceParameters();
	public final DContactGeom geom = new DContactGeom();
	public final DVector3 fdir1 = new DVector3();

	public DContact() {
		// Nothing
	}
	
	public static class DSurfaceParameters {
		/* must always be defined */
		public int mode;
		public double mu;

		/* only defined if the corresponding flag is set in mode */
		public double mu2;
		/** Rolling friction */
		public double rho;                    
		public double rho2;
		/** Spinning friction */
		public double rhoN;                   
		/** Coefficient of restitution */
		public double bounce;                 
		/** Bouncing threshold */
		public double bounce_vel;             
		public double soft_erp;
		public double soft_cfm;
		public double motion1,motion2,motionN;
		public double slip1;
		public double slip2;

		public DSurfaceParameters() {}

		void nullify() {
			 mode = 0;
			 mu = 0;
			 mu2 = 0;
			 rho = 0;
			 rho2 = 0;
			 rhoN = 0;
			 bounce = 0;
			 bounce_vel = 0;
			 soft_erp = 0;
			 soft_cfm = 0;
			 motion1 = 0;
			 motion2 = 0;
			 motionN = 0;
			 slip1 = 0;
			 slip2 = 0;
		}

		public void set(DSurfaceParameters other) {
			mode = other.mode;
			mu = other.mu;
			mu2 = other.mu2;
			rho = other.rho;
			rho2 = other.rho2;
			rhoN = other.rhoN;
			bounce = other.bounce;
			bounce_vel = other.bounce_vel;
			soft_erp = other.soft_erp;
			soft_cfm = other.soft_cfm;
			motion1 = other.motion1;
			motion2 = other.motion2;
			motionN = other.motionN;
			slip1 = other.slip1;
			slip2 = other.slip2;
		}
	}

	public DContactGeom getContactGeom() {
		return geom;
	}

	public void nullify() {
		surface.nullify();
		geom.nullify();
		fdir1.setZero();
	}

	public void set(DContact other) {
		surface.set(other.surface);
		geom.set(other.geom);
		fdir1.set(other.fdir1);
	}
}
