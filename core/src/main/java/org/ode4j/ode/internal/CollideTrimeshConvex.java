package org.ode4j.ode.internal;

import org.ode4j.ode.DColliderFn;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;

public class CollideTrimeshConvex implements DColliderFn {

	@Override
	public int dColliderFn(DGeom o1, DGeom o2, int flags, DContactGeomBuffer contacts) {
		return((DxConvex)o2).collideTriMesh(o1, flags, contacts);
	}

}
