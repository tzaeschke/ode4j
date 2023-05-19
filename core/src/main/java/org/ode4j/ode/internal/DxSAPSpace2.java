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

import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.internal.Common.dAASSERT;
import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.Common.dUASSERT;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;

import org.ode4j.ode.DAABB;
import org.ode4j.ode.DAABBC;
import org.ode4j.ode.DSapSpace;
import org.ode4j.ode.DSpace;

/**
 * Original code: OPCODE - Optimized Collision Detection Copyright (C) 2001
 * Pierre Terdiman Homepage: http://www.codercorner.com/Opcode.htm
 *
 * Temporally coherent version of SAPSpace with additional BVH tree for collide2 performance improvement.
 *
 *
 * This is essentially the same as SAP Space but with a crucial improvement: It is possible to specify a mask for
 * marking bodies as 'immobile' (such as houses or walls in a 3D game). These objects are not tested for collision.
 * For large gaming areas this can considerably improve performance because many or even most of the collisions do
 * not need to be calculated even if the objects touch or even intersect.
 *
 * @author Piotr Piastucki
 *
 */
public class DxSAPSpace2 extends DxSpace implements DSapSpace {

    private static class BVHNode {
        DAABB aabb;
        int imin, imax;
        int escape;

        private BVHNode() {
            aabb = new DAABB();
            aabb.set(dInfinity, -dInfinity, dInfinity, -dInfinity, dInfinity, -dInfinity);

        }
    }

    private boolean dirtyGeoms;
    private List<DxGeom> dirty = new ArrayList<DxGeom>();
    // geoms with infinite AABBs
    private List<DxGeom> infGeomList = new ArrayList<DxGeom>();
    // geoms with normal AABBs
    private List<DxGeom> normGeomList = new ArrayList<DxGeom>();
    // temporary storage for enabled geoms with normal AABBs
    private List<DxGeom> tempGeomList = new ArrayList<DxGeom>();
    // temporary storage for tracking overlapping geoms
    private List<BVHNode> bvhNodes = new ArrayList<BVHNode>();
    // Our sorting axes. (X,Z,Y is often best). Stored *2 for minor speedup
    // Axis indices into geom's aabb are: min=idx, max=idx+1
    private int ax0id;
    private int ax1id;
    private int ax2id;
    private final long staticGeomCategoryMask;

    /**
     * Creation.
     * @param space space
     * @param axisorder axis order
     * @param staticGeomCategoryMask mask
     * @return SAPSpace
     */
    public static DxSAPSpace2 dSweepAndPruneSpaceCreate(DxSpace space, int axisorder, long staticGeomCategoryMask) {
        return new DxSAPSpace2(space, axisorder, staticGeomCategoryMask);
    }

    /**
     * A bit of repetitive work - similar to collideAABBs, but doesn't check if
     * AABBs intersect (because SAP returns pairs with overlapping AABBs).
     */
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

    private DxSAPSpace2(DxSpace space, int axisorder, long staticGeomCategoryMask) {
        super(space);
        this.staticGeomCategoryMask = staticGeomCategoryMask;
        type = dSweepAndPruneSpaceClass;
        // Init AABB to infinity
        _aabb.set(-dInfinity, dInfinity, -dInfinity, dInfinity, -dInfinity, dInfinity);
        ax0id = ((axisorder) & 3);
        ax1id = ((axisorder >> 2) & 3);
        ax2id = ((axisorder >> 4) & 3);
    }

    @Override
    void add(DxGeom g) {
        CHECK_NOT_LOCKED(this);
        dAASSERT(g);
        dUASSERT(g.parent_space == null, "geom is already in a space");
        infGeomList.add(g);
        dirty.add(g);
        dirtyGeoms = true;
        super.add(g);
    }

    @Override
    void remove(DxGeom g) {
        CHECK_NOT_LOCKED(this);
        dAASSERT(g);
        dUASSERT(g.parent_space == this, "object is not in this space");
        // remove
        if (!infGeomList.remove(g)) {
            normGeomList.remove(g);
        }
        dirty.remove(g);
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
        // compute the AABBs of all dirty geoms, clear the dirty flags,
        // remove from dirty list
        lock_count++;
        if (dirtyGeoms) {
            for (DxGeom g : dirty) {
                if (g instanceof DSpace) {
                    ((DSpace) g).cleanGeoms();
                }
                g.recomputeAABB();
                g.unsetFlagDirtyAndBad();
            }
            for (Iterator<DxGeom> iter = infGeomList.iterator(); iter.hasNext();) {
                DxGeom g = iter.next();
                if (g._aabb.getMax(ax0id) != dInfinity) {
                    iter.remove();
                    normGeomList.add(g);
                }
            }
            Collections.sort(normGeomList, new GeomComparator());
            dirty.clear();
            dirtyGeoms = false;
            bvhNodes.clear();
        }
        lock_count--;
    }

    @Override
    public void collide(Object data, DNearCallback callback) {
        dAASSERT(callback);

        lock_count++;

        cleanGeoms();
        tempGeomList.clear();
        for (DxGeom g : normGeomList) {
            if (GEOM_ENABLED(g)) {
                tempGeomList.add(g);
            }
        }
        // do SAP on normal AABBs
        boxPruning(tempGeomList, data, callback);

        int infSize = infGeomList.size();

        for (int m = 0; m < infSize; ++m) {
            DxGeom g1 = infGeomList.get(m);
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
            for (DxGeom g2 : tempGeomList) {
                collideGeomsNoAABBs(g1, g2, data, callback);
            }
        }

        lock_count--;
    }

    @Override
    void collide2(Object data, DxGeom geom, DNearCallback callback) {
        dAASSERT(geom != null && callback != null);

        lock_count++;
        cleanGeoms();
        geom.recomputeAABB();
        if (normGeomList.size() > 0) {
            if (bvhNodes.isEmpty()) {
                buildBVH();
            }
            DAABBC aabb = geom.getAABB();
            int i = 0;
            int size = bvhNodes.size();
            while (i < size) {
                BVHNode node = bvhNodes.get(i);
                boolean overlap = !node.aabb.isDisjoint(aabb); 
                boolean isLeafNode = node.escape >= 0;
                if (isLeafNode && overlap) {
                    for (int j = node.imin; j < node.imax; j++) {
                        DxGeom g = normGeomList.get(j);
                        if (GEOM_ENABLED(g)) {
                            collideAABBs(g, geom, data, callback);
                        }
                    }
                }
                if (overlap || isLeafNode) {
                    i++;
                } else {
                    i = -node.escape;
                }
            }
        }
        for (DxGeom g : infGeomList) {
            if (GEOM_ENABLED(g)) {
                collideAABBs(g, geom, data, callback);
            }
        }
        lock_count--;
    }

    private void buildBVH() {
        subdivide(null, 0, normGeomList.size(), 10);
    }

    private void subdivide(BVHNode parent, int imin, int imax, int geomsPerNode) {
        int inum = imax - imin;
        if (inum == 0)
            return;
        BVHNode node = new BVHNode();
        node.imin = imin;
        node.imax = imax;
        bvhNodes.add(node);
        if (inum <= geomsPerNode) {
            // Leaf
            node.escape = bvhNodes.size();
            for (int i = imin; i < imax; i++) {
                node.aabb.expand(normGeomList.get(i)._aabb);
            }
            if (parent != null) {
                parent.aabb.expand(node.aabb);
            }
        } else {
            // Split
            int isplit = imin + inum / 2;
            subdivide(node, imin, isplit, geomsPerNode);
            subdivide(node, isplit, imax, geomsPerNode);
            if (parent != null) {
                parent.aabb.expand(node.aabb);
            }
            node.escape = -bvhNodes.size();
        }
    }

    private class GeomComparator implements Comparator<DxGeom> {
        @Override
        public int compare(DxGeom arg0, DxGeom arg1) {
            double a0 = arg0._aabb.getMin(ax0id);
            double a1 = arg1._aabb.getMin(ax0id);
            return a1 > a0 ? -1 : (a1 < a0 ? 1 : 0);
        }
    }

    /**
     * Complete box pruning. Returns a list of overlapping pairs of boxes, each
     * box of the pair belongs to the same set.
     * 
     * @param geoms
     *            [in] geoms of boxes.
     */
    void boxPruning(final List<DxGeom> geoms, Object data, DNearCallback callback) {
        int size = geoms.size();
        int nonStaticIndex = 0;
        for (int i = 0; i < size; i++) {
            DxGeom g0 = geoms.get(i);
            DAABB aabb0 = g0._aabb;
            final double idx0ax0max = aabb0.getMax(ax0id);
            int next = i + 1;
            // Static geoms can collide with non-static geoms only, hence advance to the next non-static one
            if ((g0.getCategoryBits() & staticGeomCategoryMask) != 0) {
                if (nonStaticIndex < next) {
                    for (nonStaticIndex = next; nonStaticIndex < size; nonStaticIndex++) {
                        DxGeom g1 = geoms.get(nonStaticIndex);
                        if ((g1.getCategoryBits() & staticGeomCategoryMask) == 0) {
                            break;
                        }
                    }
                }
                next = nonStaticIndex;
            }
            for (int j = next; j < size; j++) {
                DxGeom g1 = geoms.get(j);
                if (g1._aabb.getMin(ax0id) > idx0ax0max) {
                    // This and following elements can not intersect with g1.
                    break;
                }
                if (aabb0.getMax(ax1id) >= g1._aabb.getMin(ax1id))
                    if (g1._aabb.getMax(ax1id) >= aabb0.getMin(ax1id))
                        if (aabb0.getMax(ax2id) >= g1._aabb.getMin(ax2id))
                            if (g1._aabb.getMax(ax2id) >= aabb0.getMin(ax2id))
                                collideGeomsNoAABBs(g0, g1, data, callback);
            }
        }
    }
}
