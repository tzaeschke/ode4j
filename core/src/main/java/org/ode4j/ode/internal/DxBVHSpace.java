/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine 4J                                               *
 * Copyright (C) 2017 Piotr Piastucki, Tilmann Zaeschke                  *
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

import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.internal.Common.dAASSERT;
import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.Common.dUASSERT;

import java.util.ArrayList;
import java.util.List;

import org.ode4j.ode.DAABB;
import org.ode4j.ode.DBhvSpace;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.internal.aabbtree.AABBTree;
import org.ode4j.ode.internal.aabbtree.AABBTreeNodeCallback;
import org.ode4j.ode.internal.aabbtree.AABBTreePairCallback;
import org.ode4j.ode.internal.aabbtree.ExternalObjectHandler;

@SuppressWarnings("deprecation")
public class DxBVHSpace extends DxSpace implements DBhvSpace {

	private final AABBTree<DxGeom> bvhTree;
	private boolean dirtyGeoms;
	private List<DxGeom> dirty = new ArrayList<DxGeom>();
	// geoms with infinite AABBs
	private List<DxGeom> infGeomList = new ArrayList<DxGeom>();

	private class GeomSpatialIndexHandler implements ExternalObjectHandler<DxGeom> {

	    private final long staticGeomCategoryMask;
	    
		public GeomSpatialIndexHandler(long staticGeomCategoryMask) {
            this.staticGeomCategoryMask = staticGeomCategoryMask;
        }

        @Override
		public void setSpatialIndex(DxGeom object, int index) {
			object._sapIdxGeomEx = index;
		}

		@Override
		public int getSpatialIndex(DxGeom object) {
			return object._sapIdxGeomEx;
		}

		@Override
		public boolean isEnabled(DxGeom object) {
			return GEOM_ENABLED(object);
		}

        @Override
        public boolean isStatic(DxGeom object) {
            return (object.getCategoryBits() & staticGeomCategoryMask) != 0;
        }

	}

	public static DxBVHSpace bvhSpaceCreate(DxSpace space, int nodesPerLeaf, boolean highQuality, double fatAabbMargin, long staticGeomCategoryMask) {
		return new DxBVHSpace(space, nodesPerLeaf, highQuality, fatAabbMargin, staticGeomCategoryMask);
	}

	private DxBVHSpace(DxSpace space, int nodesPerLeaf, boolean highQuality, double fatAabbMargin, long staticGeomCategoryMask) {
		super(space);
		type = dBVHSpaceClass;
		_aabb.set(-dInfinity, dInfinity, -dInfinity, dInfinity, -dInfinity, dInfinity);
		bvhTree = new AABBTree<>(new GeomSpatialIndexHandler(staticGeomCategoryMask), nodesPerLeaf, highQuality, fatAabbMargin);
	}

	@Override
	void add(DxGeom g) {
		CHECK_NOT_LOCKED(this);
		dAASSERT(g);
		dUASSERT(g.parent_space == null, "geom is already in a space");
		dirty.add(g);
		dirtyGeoms = true;
		g._sapIdxGeomEx = AABBTree.UNDEFINED_INDEX;
		super.add(g);
	}

	@Override
	void remove(DxGeom g) {
		CHECK_NOT_LOCKED(this);
		dAASSERT(g);
		dUASSERT(g.parent_space == this, "object is not in this space");
		infGeomList.remove(g);
		dirty.remove(g);
		bvhTree.remove(g);
		dirtyGeoms = true;
		super.remove(g);
	}

	@Override
	void dirty(DxGeom g) {
		dAASSERT(g);
		dUASSERT(g.parent_space == this, "object is not in this space");
		dirty.add(g);
		dirtyGeoms = true;
	}

	@Override
	public void cleanGeoms() {
		lock_count++;
		if (dirtyGeoms) {
			for (DxGeom g : dirty) {
				if (g instanceof DSpace) {
					((DSpace) g).cleanGeoms();
				}
				g.recomputeAABB();
				g.unsetFlagDirtyAndBad();
				double[] extents = getExtents(g);
				if (g._sapIdxGeomEx != AABBTree.UNDEFINED_INDEX) {
					bvhTree.update(g, extents);
				} else if (!isInfinite(g)) {
					bvhTree.add(g, extents);
				} else {
					infGeomList.add(g);
				}
			}
			dirty.clear();
			dirtyGeoms = false;
			bvhTree.finalizeUpdate();
		}
		lock_count--;
	}

	private double[] getExtents(DxGeom g) {
		double[] extents = new double[6];
		extents[0] = g.getAABB().getMin0();
		extents[1] = g.getAABB().getMin1();
		extents[2] = g.getAABB().getMin2();
		extents[3] = g.getAABB().getMax0();
		extents[4] = g.getAABB().getMax1();
		extents[5] = g.getAABB().getMax2();
		return extents;
	}

	private boolean isInfinite(DxGeom g) {
		return g._aabb.getMax0() == dInfinity && g._aabb.getMax1() == dInfinity && g._aabb.getMax2() == dInfinity;
	}

	@Override
	public void collide(final Object data, final DNearCallback callback) {
		dAASSERT(callback);

		lock_count++;

		cleanGeoms();

		// do BVH on normal AABBs
		bvhTree.getOverlappingPairs(new AABBTreePairCallback<DxGeom>() {
			@Override
			public void overlap(DxGeom o1, DxGeom o2) {
				collideGeomsNoAABBs(o1, o2, data, callback);
			}
		});

		int infSize = infGeomList.size();

		for (int m = 0; m < infSize; ++m) {
			final DxGeom g1 = infGeomList.get(m);
			if (!GEOM_ENABLED(g1))
				continue;

			// collide infinite ones
			for (int n = m + 1; n < infSize; ++n) {
				DxGeom g2 = infGeomList.get(n);
				if (GEOM_ENABLED(g2)) {
					collideGeomsNoAABBs(g1, g2, data, callback);
				}
			}

			// collide infinite ones with normal ones
			bvhTree.getOverlappingNodes(getExtents(g1), new AABBTreeNodeCallback<DxGeom>() {
				@Override
				public void overlap(DxGeom o) {
					collideGeomsNoAABBs(g1, o, data, callback);
				}
			});
		}

		lock_count--;
	}

	void collideGeomsNoAABBs(DxGeom g1, DxGeom g2, Object data, DNearCallback callback) {
		dIASSERT(!g1.hasFlagAabbBad());
		dIASSERT(!g2.hasFlagAabbBad());

		// no contacts if both geoms on the same body, and the body is not 0
		if (g1.body == g2.body && g1.body != null)
			return;

		// test if the category and collide bitfields match
		if (((g1.category_bits & g2.collide_bits) != 0 || (g2.category_bits & g1.collide_bits) != 0) == false) {
			return;
		}

		DAABB bounds1 = g1._aabb;
		DAABB bounds2 = g2._aabb;

		// check if either object is able to prove that it doesn't intersect the
		// AABB of the other
		if (g1.AABBTest(g2, bounds2) == false)
			return;
		if (g2.AABBTest(g1, bounds1) == false)
			return;

		// the objects might actually intersect - call the space callback
		// function
		callback.call(data, g1, g2);
	}

	@Override
	void collide2(final Object data, final DxGeom geom, final DNearCallback callback) {
		dAASSERT(geom != null && callback != null);

		lock_count++;
		cleanGeoms();
		geom.recomputeAABB();

		bvhTree.getOverlappingNodes(getExtents(geom), new AABBTreeNodeCallback<DxGeom>() {
			@Override
			public void overlap(DxGeom o) {
				collideGeomsNoAABBs(geom, o, data, callback);
			}
		});

		for (DxGeom g : infGeomList) {
			if (GEOM_ENABLED(g)) {
				collideAABBs(g, geom, data, callback);
			}
		}
		lock_count--;
	}

}
