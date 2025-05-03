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

import ch.ethz.globis.phtree.PhTreeMultiMapF2;
import ch.ethz.globis.phtree.PhTreeMultiMapSF2;
import ch.ethz.globis.phtree.PhTreeSolidF;
import org.ode4j.ode.DAABB;
import org.ode4j.ode.DAABBC;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DSimpleSpace;

import java.util.ArrayList;
import java.util.List;

import static org.ode4j.ode.internal.Common.*;


/**
 * ***************************************************************************
 * simple space - reports all n^2 object intersections
 */
public class DxPHTreeSpace extends DxSpace implements DSimpleSpace {

    //	  dxSimpleSpace (dSpace _space);
    //	  void cleanGeoms();
    //	  void collide (void *data, dNearCallback *callback);
    //	  void collide2 (void *data, dxGeom *geom, dNearCallback *callback);

    // TODO MultimapF?
    // TODO use other preprocessor for DAABB!!!
    private final PhTreeMultiMapSF2<DxGeom> tree;
    private final List<DxGeom> DirtyList = new ArrayList<>();
    private DxGeom mFirst = null;
    private int geomCount = 0;


    public static DxPHTreeSpace dPHTreeSpaceCreate(DxSpace space) {
        return new DxPHTreeSpace(space);
    }


    DxPHTreeSpace(DxSpace space) {
        super(space);
        type = dPHTreeSpaceClass;
        tree = PhTreeMultiMapSF2.create(3);
    }

    private static double[] toMin(DAABBC aabb) {
        return new double[]{aabb.getMin0(), aabb.getMin1(), aabb.getMin2()};
    }

    private static double[] toMax(DAABBC aabb) {
        return new double[]{aabb.getMax0(), aabb.getMax1(), aabb.getMax2()};
    }

    @Override
    public void DESTRUCTOR() {
        tree.clear();
        super.DESTRUCTOR();
    }

    @Override
    void add(DxGeom g) {
        CHECK_NOT_LOCKED(this);
        dUASSERT(g._qtIdxEx == null && g.getNextEx() == null, "geom is already in a space");

        DirtyList.add(g);

        // add
        tree.put(toMin(g._aabb), toMax(g._aabb), g);

        super.add(g);



        // Add the geom
        g.setNextEx( mFirst );
        mFirst = g;
        //XXX TZ aObject.tome = (dxGeom**)this;
        //g._qtIdxEx = this;
        // Each geom gets a unique ID. This can be used to prevent duplicate collisions.
        if (g._sapIdxGeomEx == 0) {
            g._sapIdxGeomEx = ++geomCount;
        }
    }

    @Override
    void remove(DxGeom g2) {
        CHECK_NOT_LOCKED(this);
        dUASSERT(g2.parent_space == this, "object is not in this space");

        // remove
        tree.remove(toMin(g2._aabb), toMax(g2._aabb), g2); // DO NOT CALL getAABB() !

        for (int i = 0; i < DirtyList.size(); i++) {
            if (DirtyList.get(i) == g2) {
                DirtyList.remove(i);
                // (mg) there can be multiple instances of a dirty object on stack  be sure to remove ALL and not just first, for this we decrement i
                --i;
            }
        }

        super.remove(g2);




        // Del the geom
        DxGeom prev = null, gF = mFirst;
        boolean Found = false;

        if (gF == g2){
            mFirst = gF.getNextEx();
            Found = true;
        } else {
            prev = gF;
            gF = gF.getNextEx();
        }

        for (; !Found && gF != null; Found = false){
            if (gF == g2){
                prev.setNextEx( gF.getNextEx() );
                break;
            }
            prev = gF;
            gF = gF.getNextEx();
        }

        //XXX TZ aObject.tome = null;
        g2._qtIdxEx = null;
        g2.setNextEx(null);
    }

    //void dxQuadTreeSpace::dirty(dxGeom* g){
    @Override
    void dirty(DxGeom g) {
        DirtyList.add(g);
    }

    @Override
    protected void computeAABB() {
        // // TODO ???? Why not in QuadTree? Others?
    }

    @Override
    public void cleanGeoms() {
        // compute the AABBs of all dirty geoms, and clear the dirty flags
        lock_count++;

        for (int i = 0; i < DirtyList.size(); i++) {
            DxGeom g = DirtyList.get(i);
            if (g instanceof DxSpace) {
                ((DxSpace) g).cleanGeoms();
                // TODO AABB for PHTRee? -> Estimate with 1st level of sub-nodes
            }

            double[] oldAABBMin = toMin(g._aabb); // DO NOT CALL getAABB() !
            double[] oldAABBMax = toMax(g._aabb); // DO NOT CALL getAABB() !
            g.recomputeAABB();
            double[] newAABBMin = toMin(g._aabb);
            double[] newAABBMax = toMax(g._aabb);

            dIASSERT(!g.hasFlagAabbBad());

            //g->gflags &= ~GEOM_DIRTY;
            g.unsetFlagDirty();

            dUASSERT(tree.remove(oldAABBMin, oldAABBMax, g), "Entry not found!");
            tree.put(newAABBMin, newAABBMax, g);
        }
        DirtyList.clear();

        lock_count--;
    }

    /**
     * From <a href="https://www.ode.org/wiki/index.php/Manual#Collision_detection_2">ODE Wiki</a>:
     * "dSpaceCollide determines which pairs of geoms in a space may potentially intersect, and calls a callback
     * function with each candidate pair. This does not generate contact points directly, because the user may want
     * to handle some pairs specially - for example by ignoring them or using different contact generating strategies.
     * Such decisions are made in the callback function, which can choose whether or not to call dCollide for each
     * pair."
     *
     * @param UserData data
     * @param Callback callback
     */
    @Override
    public void collide(Object UserData, DNearCallback Callback) {
        dAASSERT(Callback);

        lock_count++;
        cleanGeoms();

        PhTreeMultiMapSF2.PhQuerySF<DxGeom> q = null;

//        PhTreeMultiMapSF2.PhExtentF<DxGeom> ext = tree.queryExtent();
//        while (ext.hasNext()) {
//        DxGeom g1 = ext.nextValue();
        DxGeom g1 = mFirst;
        while (g1 != null) {
            if (GEOM_ENABLED(g1)) {
                if (q == null) {
                    q = tree.queryIntersect(toMin(g1._aabb), toMax(g1._aabb));
                } else {
                    q.reset(toMin(g1._aabb), toMax(g1._aabb));
                }
                while (q.hasNext()) {
                    DxGeom g2 = q.nextValue();
                    // if (g2 != g1 && GEOM_ENABLED(g2)) {
                    if (g2._sapIdxGeomEx > g1._sapIdxGeomEx && GEOM_ENABLED(g2)) {
                        collideGeomsNoAABBs(g1, g2, UserData, Callback);
                    }
                }
            }
            g1 = g1.getNextEx();
        }
        lock_count--; // TODO ????
    }

    /**
     * From <a href="https://www.ode.org/wiki/index.php/Manual#Collision_detection_2">ODE Wiki</a>:
     * "dSpaceCollide2 determines which geoms from one space may potentially intersect with geoms from another space,
     * and calls a callback function with each candidate pair. It can also test a single non-space geom against a
     * space or treat a space of lower inclusion sublevel as a geom against a space of higher level. This function is
     * useful when there is a collision hierarchy, i.e. when there are spaces that contain other spaces."
     *
     * @param UserData data
     * @param g2       2nd geom
     * @param Callback callback
     */
    @Override
    void collide2(Object UserData, DxGeom g2, DNearCallback Callback) {
        dAASSERT(Callback);

        lock_count++;
        cleanGeoms();
        g2.recomputeAABB();

        PhTreeMultiMapSF2.PhQuerySF<DxGeom> q = tree.queryIntersect(toMin(g2._aabb), toMax(g2._aabb));
        while (q.hasNext()) {
            DxGeom g = q.nextValue();
            if (g2 != g && GEOM_ENABLED(g)) {
                collideGeomsNoAABBs(g, g2, UserData, Callback);
            }
        }

        lock_count--;
    }

    /**
     * Copied from SAP SPace 2.
     * Similar to collideAABBs, but doesn't check if AABBs intersect because we already  know they overlap.
     */
    private void collideGeomsNoAABBs(DxGeom g1, DxGeom g2, Object data, DNearCallback callback) {
        dIASSERT(!g1.hasFlagAabbBad());
        dIASSERT(!g2.hasFlagAabbBad());

        // no contacts if both geoms on the same body, and the body is not 0
        if (g1.body == g2.body && g1.body != null) return;

        // test if the category and collide bitfields match
        if (!((g1.category_bits & g2.collide_bits) != 0 || (g2.category_bits & g1.collide_bits) != 0)) {
            return;
        }

        DAABB bounds1 = g1._aabb;
        DAABB bounds2 = g2._aabb;

        // check if either object is able to prove that it doesn't intersect the AABB of the other
        if (!g1.AABBTest(g2, bounds2)) return;
        if (!g2.AABBTest(g1, bounds1)) return;

        // the objects might actually intersect - call the space callback function
        callback.call(data, g1, g2);
    }
}
