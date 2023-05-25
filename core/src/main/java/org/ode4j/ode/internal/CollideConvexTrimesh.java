package org.ode4j.ode.internal;

import java.util.Arrays;

import org.ode4j.ode.DAABBC;
import org.ode4j.ode.DColliderFn;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.internal.CollisionLibccd.CollideConvexTrimeshTrianglesCCD;
import org.ode4j.ode.internal.gimpact.GimDynArrayInt;
import org.ode4j.ode.internal.gimpact.GimGeometry.aabb3f;
import org.ode4j.ode.internal.gimpact.GimTrimesh;

class CollideConvexTrimesh implements DColliderFn {

	@Override
	public int dColliderFn(DGeom o1, DGeom o2, int flags, DContactGeomBuffer contacts) {

		DxGimpact trimesh = (DxGimpact) o2; 
		GimTrimesh ptrimesh = trimesh.m_collision_trimesh();
		aabb3f test_aabb = new aabb3f();

		DAABBC aabb = o1.getAABB();
		test_aabb.set(aabb.getMin0(), aabb.getMax0(), aabb.getMin1(), aabb.getMax1(), aabb.getMin2(), aabb.getMax2());

		GimDynArrayInt collision_result = GimDynArrayInt.GIM_CREATE_BOXQUERY_LIST();
		ptrimesh.getAabbSet().gim_aabbset_box_collision(test_aabb, collision_result);
		int contactcount = 0;
		if (collision_result.size() != 0) {
			int[] boxesresult = Arrays.copyOf(collision_result.GIM_DYNARRAY_POINTER(), collision_result.size());
			ptrimesh.gim_trimesh_locks_work_data();
			CollideConvexTrimeshTrianglesCCD collideFn = new CollideConvexTrimeshTrianglesCCD();
			contactcount = collideFn.collide(o1, o2, boxesresult, flags, contacts);
			ptrimesh.gim_trimesh_unlocks_work_data();
		}
		collision_result.GIM_DYNARRAY_DESTROY();
		return contactcount;
	}
}
