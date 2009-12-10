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
package org.ode4j.ode.internal;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;


/**
 *  object, body, and world structs.
 */
class Objects_H {


	/** auto disable parameters. */
	public static class dxAutoDisable implements Cloneable {
		public double idle_time;		// time the body needs to be idle to auto-disable it
		public int idle_steps;		// steps the body needs to be idle to auto-disable it
		public double linear_average_threshold;   // linear (squared) average velocity threshold
		public double angular_average_threshold;  // angular (squared) average velocity threshold
		//TODO? unsigned 
		public int average_samples;     // size of the average_lvel and average_avel buffers
		@Override
		protected dxAutoDisable clone() {
			try {
				return (dxAutoDisable) super.clone();
			} catch (CloneNotSupportedException e) {
				throw new RuntimeException(e);
			}
		}
	}


	/** damping parameters. */
	public static class dxDampingParameters implements Cloneable {
		public double linear_scale;  // multiply the linear velocity by (1 - scale)
		public double angular_scale; // multiply the angular velocity by (1 - scale)
		public double linear_threshold;   // linear (squared) average speed threshold
		public double angular_threshold;  // angular (squared) average speed threshold
		@Override
		protected dxDampingParameters clone() {
			try {
				return (dxDampingParameters) super.clone();
			} catch (CloneNotSupportedException e) {
				throw new RuntimeException(e);
			}
		}
	}


	/** quick-step parameters. */
	public static class dxQuickStepParameters extends CloneableParameter {
		public int num_iterations;		// number of SOR iterations to perform
		public double w;			// the SOR over-relaxation parameter
		@Override
		protected dxQuickStepParameters clone() {
			return clone();
		}
	}


	/** contact generation parameters. */
	public static class dxContactParameters extends CloneableParameter {
		public double max_vel;		// maximum correcting velocity
		public double min_depth;		// thickness of 'surface layer'
		@Override
		protected dxContactParameters clone() {
			return cloneThis();
		}
	}



	/** 
	 * position vector and rotation matrix for geometry objects that are not
	 * connected to bodies.
	 */
	public static class dxPosR {
		public DVector3 pos = new DVector3();
		public DMatrix3 R = new DMatrix3();
		public DMatrix3 R() {
			return R;
		}
		public DVector3 pos() {
			return pos;
		}
		@Override
		protected dxPosR clone() {
			try {
				dxPosR p = (dxPosR) super.clone();
				p.pos = pos.clone();
				p.R = R.clone();
				return p;
			} catch (CloneNotSupportedException e) {
				throw new RuntimeException(e);
			}
		}
	}
	
	private static class CloneableParameter implements Cloneable {
		//@Override
		@SuppressWarnings("unchecked")
		public <T> T cloneThis() {
			try {
				return (T) super.clone();
			} catch (CloneNotSupportedException e) {
				throw new RuntimeException(e);
			}
		}
	}
}