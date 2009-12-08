package org.ode4j.ode;

import org.ode4j.math.DVector3;
import org.ode4j.ode.internal.DxGeom;


/**
 * Describe the contact point between two geoms.
 *
 * If two bodies touch, or if a body touches a static feature in its 
 * environment, the contact is represented by one or more "contact 
 * points", described by dContactGeom.
 *
 * The convention is that if body 1 is moved along the normal vector by 
 * a distance depth (or equivalently if body 2 is moved the same distance 
 * in the opposite direction) then the contact depth will be reduced to 
 * zero. This means that the normal vector points "in" to body 1.
 *
 * @ingroup collide
 */
public class DContactGeom {
	
	DContactGeom() {
		// Non-public
	}
	
	public DVector3 pos = new DVector3();          ///< contact position
	public DVector3 normal = new DVector3();       ///< normal vector
	public double depth;           ///< penetration depth
	public DxGeom g1;         ///< the colliding geoms
	public DxGeom g2;
	public int side1;       ///< (to be documented)
	public int side2;
}
