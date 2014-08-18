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

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.threading.DThreadingImplementation;
import org.ode4j.ode.threading.Threading_H.DThreadingFunctionsInfo;


/**
 *  object, body, and world structs.
 *  
 *  @author Tilmann Zaeschke
 */
public class Objects_H {

	static final double dWORLD_DEFAULT_GLOBAL_ERP = 0.2;

//	#if defined(dSINGLE)
//	#define dWORLD_DEFAULT_GLOBAL_CFM REAL(1e-5)
//	#elif defined(dDOUBLE)
	static final double dWORLD_DEFAULT_GLOBAL_CFM = Common.dDOUBLE ? 1e-10 : 1e-5;
//	#else
//	#error dSINGLE or dDOUBLE must be defined
//	#endif

	static DThreadingImplementation g_world_default_threading_impl = null;
	static DThreadingFunctionsInfo g_world_default_threading_functions = null;

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

		//TZ: 'explicit' not required in Java
	    dxAutoDisable() {
	        idle_time = 0.0;
	        idle_steps = 10;
	        average_samples = 1; // Default is 1 sample => Instantaneous velocity
	        linear_average_threshold = 0.01*0.01; // (magnitude squared)
	        angular_average_threshold = 0.01*0.01; // (magnitude squared)
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

		//TZ: 'explicit' not required in Java
	    dxDampingParameters() {
	        linear_scale = 0.0;
	        angular_scale = 0.0;
	        linear_threshold = 0.01 * 0.01;
	        angular_threshold = 0.01 * 0.01;
	    }
	}


	/** quick-step parameters. */
	public static class dxQuickStepParameters extends CloneableParameter {
		public int num_iterations;		// number of SOR iterations to perform
		public double w;			// the SOR over-relaxation parameter
		@Override
		protected dxQuickStepParameters clone() {
			return cloneThis();
		}

		//TZ: 'explicit' not required in Java
	    dxQuickStepParameters() {
	    	num_iterations = 20;
	    	w = 1.3;
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

		//TZ: 'explicit' not required in Java
	    dxContactParameters() {
	        max_vel = Common.dInfinity;
	        min_depth = 0.0;
	    }
	}



	/** 
	 * position vector and rotation matrix for geometry objects that are not
	 * connected to bodies.
	 */
	public static class DxPosR implements DxPosRC {
		public final DVector3 pos = new DVector3();
		private final DMatrix3 R = new DMatrix3();
		@Override
		public DMatrix3C R() {
			return R;
		}
		/**
		 * 
		 * @return Writable R.
		 */
		public DMatrix3 Rw() {
			return R;
		}
		@Override
		public DVector3C pos() {
			return pos;
		}
		@Override
		protected DxPosR clone() {
			try {
				DxPosR p = (DxPosR) super.clone();
				p.pos.set( pos );// = pos.clone();
				p.R.set( R );// = R.clone();
				return p;
			} catch (CloneNotSupportedException e) {
				throw new RuntimeException(e);
			}
		}
	}
	
	public interface DxPosRC {
		DMatrix3C R();
		DVector3C pos();
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