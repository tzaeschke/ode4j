package org.ode4j.ode;

import org.ode4j.math.DVector3;

/**
 *  contact info used by contact joint.
 */
public class DContact {

	public final dSurfaceParameters surface = new dSurfaceParameters();
	public final DContactGeom geom = new DContactGeom();
	public final DVector3 fdir1 = new DVector3();

	DContact() {
		// Nothing
	}
	
	public class dSurfaceParameters {
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
