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
package org.ode4j.ode.internal.trimesh;

import org.ode4j.math.DVector3;
import org.ode4j.ode.DTriMesh;
import org.ode4j.ode.internal.DxGeom;
import org.ode4j.ode.internal.DxGimpactCollision;
import org.ode4j.ode.internal.DxSpace;

import java.util.Arrays;

import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.Common.dIN_RANGE;

//    typedef dxGeom dxMeshBase_Parent;
//    struct dxMeshBase:
//    public dxMeshBase_Parent
public abstract class DxMeshBase extends DxGeom {
    //        public:
    //        dxMeshBase(dxSpace *Space, dxTriDataBase *Data,
    //                dTriCallback *Callback, dTriArrayCallback *ArrayCallback, dTriRayCallback *RayCallback,
    //                bool doTCs=false):
    public DxMeshBase(DxSpace Space, DxTriDataBase Data, DTriMesh.DTriCallback Callback,
                      DTriMesh.DTriArrayCallback ArrayCallback, DTriMesh.DTriRayCallback RayCallback) {
        this(Space, Data, Callback, ArrayCallback, RayCallback, false);
    }
    public DxMeshBase(DxSpace Space, DxTriDataBase Data, DTriMesh.DTriCallback Callback,
                      DTriMesh.DTriArrayCallback ArrayCallback, DTriMesh.DTriRayCallback RayCallback, boolean doTCs) {
        super(Space, true);
        m_Callback = Callback;
        m_ArrayCallback = ArrayCallback;
        m_RayCallback = RayCallback;
        m_TriMergeCallback = null;
        m_Data = Data;
        //std::fill(m_DoTCs, m_DoTCs + dARRAY_SIZE(m_DoTCs), doTCs);
        Arrays.fill(m_DoTCs, doTCs);
        type = dTriMeshClass;
    }

    boolean invokeCallback(DxGeom Object, int TriIndex) {
        return m_Callback == null || m_Callback.call(this, Object, TriIndex) != 0;
    }

    public enum TRIMESHTC {
        TTC_SPHERE,
        TTC_BOX,
        TTC_CAPSULE
    }
    // enum TRIMESHTC
    //    public static final int TTC__MIN = 0;
    //    public static final int TTC_SPHERE =TTC__MIN;
    //    public static final int TTC_BOX = TTC_SPHERE + 1;
    //    public static final int TTC_CAPSULE = TTC_BOX + 1;
    //    public static final int TTC__MAX = TTC_CAPSULE + 1;


    //public:
    public void assignCallback(DTriMesh.DTriCallback value) {
        m_Callback = value;
    }

    public DTriMesh.DTriCallback retrieveCallback() {
        return m_Callback;
    }

    public void assignArrayCallback(DTriMesh.DTriArrayCallback value) {
        m_ArrayCallback = value;
    }

    public DTriMesh.DTriArrayCallback retrieveArrayCallback() {
        return m_ArrayCallback;
    }

    public void assignRayCallback(DTriMesh.DTriRayCallback value) {
        m_RayCallback = value;
    }

    public DTriMesh.DTriRayCallback retrieveRayCallback() {
        return m_RayCallback;
    }

    public void assignTriMergeCallback(DTriMesh.DTriTriMergeCallback value) {
        m_TriMergeCallback = value;
    }

    public DTriMesh.DTriTriMergeCallback retrieveTriMergeCallback() {
        return m_TriMergeCallback;
    }

    //public void assignMeshData(DxTriDataBase instance) {
    void assignMeshData(DxTriMeshData instance) {
        setMeshData(instance);
        // I changed my data -- I know nothing about my own AABB anymore.
        markAABBBad();
    }

    //dxTriDataBase *retrieveMeshData() const { return getMeshData(); }
    public DxTriDataBase retrieveMeshData() {
        return getMeshData();
    }

    //        IFaceAngleStorageControl *retrieveFaceAngleStorage() { return m_Data.retrieveFaceAngles(); }
    //        IFaceAngleStorageView *retrieveFaceAngleView() { return m_Data.retrieveFaceAngleView(); }
    public IFaceAngleStorageControl retrieveFaceAngleStorage() {
        return m_Data.retrieveFaceAngles();
    }

    public IFaceAngleStorageView retrieveFaceAngleView() {
        return m_Data.retrieveFaceAngleView();
    }


    public void assignDoTC(TRIMESHTC tc, boolean value) {
        setDoTC(tc, value);
    }

    public boolean retrieveDoTC(TRIMESHTC tc) {
        return getDoTC(tc);
    }

    //public:
    //  void setDoTC(TRIMESHTC tc, bool value) { dIASSERT(dIN_RANGE(tc, TTC__MIN, TTC__MAX)); m_DoTCs[tc] = value; }
    //  bool getDoTC(TRIMESHTC tc) const { dIASSERT(dIN_RANGE(tc, TTC__MIN, TTC__MAX)); return m_DoTCs[tc]; }
    public void setDoTC(TRIMESHTC tc, boolean value) {
        //dIASSERT(dIN_RANGE(tc, TTC__MIN, TTC__MAX));
        m_DoTCs[tc.ordinal()] = value;
    }

    public boolean getDoTC(TRIMESHTC tc) {
        //dIASSERT(dIN_RANGE(tc, TTC__MIN, TTC__MAX));
        return m_DoTCs[tc.ordinal()];
    }


    //private:
    private void setMeshData(DxTriDataBase Data) {
        m_Data = Data;
    }

    //protected:
    protected DxTriDataBase getMeshData() {
        return m_Data;
    }

    //public:
    // Callbacks
    public DTriMesh.DTriCallback m_Callback;
    public DTriMesh.DTriArrayCallback m_ArrayCallback;
    public DTriMesh.DTriRayCallback m_RayCallback;
    public DTriMesh.DTriTriMergeCallback m_TriMergeCallback;

    // TODO TZ 2023 remove!
//    public DTriMesh.DTriCallback Callback() { return m_Callback; }
//    public DTriMesh.DTriArrayCallback ArrayCallback() { return m_ArrayCallback; }
//    public DTriMesh.DTriRayCallback RayCallback() { return m_RayCallback; }
//    public DTriMesh.DTriTriMergeCallback TriMergeCallback() { return m_TriMergeCallback; }

    // Data types
    private DxTriDataBase m_Data;

    //private final boolean[] m_DoTCs = new boolean[TTC__MAX];
    private final boolean[] m_DoTCs = new boolean[TRIMESHTC.values().length];

    /*extern ODE_API */
    //void dGeomTriMeshSetCallback(dGeomID g, dTriCallback* Callback)
    void dGeomTriMeshSetCallback(DTriMesh.DTriCallback Callback)
    {
        //dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

        //DxTriMesh mesh = (DxTriMesh) g;
        assignCallback(Callback);
    }

    /*extern ODE_API */
    //dTriCallback* dGeomTriMeshGetCallback(dGeomID g)
    DTriMesh.DTriCallback dGeomTriMeshGetCallback()
    {
        //dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

        //DxTriMesh mesh = (DxTriMesh) g;
        return retrieveCallback();
    }

    /*extern ODE_API */
    //void dGeomTriMeshSetArrayCallback(dGeomID g, dTriArrayCallback* ArrayCallback)
    void dGeomTriMeshSetArrayCallback(DTriMesh.DTriArrayCallback ArrayCallback)
    {
        //dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

        //DxTriMesh mesh = (DxTriMesh) g;
        assignArrayCallback(ArrayCallback);
    }

    /*extern ODE_API */
    //dTriArrayCallback *dGeomTriMeshGetArrayCallback(dGeomID g)
    DTriMesh.DTriArrayCallback dGeomTriMeshGetArrayCallback()
    {
        //dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");
        //DxTriMesh mesh = (DxTriMesh) g;
        return retrieveArrayCallback();
    }

    /*extern ODE_API */
    //void dGeomTriMeshSetRayCallback(dGeomID g, dTriRayCallback* Callback)
    void dGeomTriMeshSetRayCallback(DTriMesh.DTriRayCallback Callback)
    {
        //dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

        //DxTriMesh mesh = (DxTriMesh) g;
        assignRayCallback(Callback);
    }

    /*extern ODE_API */
    //dTriRayCallback* dGeomTriMeshGetRayCallback(dGeomID g)
    DTriMesh.DTriRayCallback dGeomTriMeshGetRayCallback()
    {
        //dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

        //DxTriMesh mesh = (DxTriMesh) g;
        return retrieveRayCallback();
    }

    /*extern ODE_API */
    //void dGeomTriMeshSetTriMergeCallback(dGeomID g, dTriTriMergeCallback* Callback)
    void dGeomTriMeshSetTriMergeCallback(DTriMesh.DTriTriMergeCallback Callback)
    {
        //dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

        //DxTriMesh mesh = (DxTriMesh) g;
        assignTriMergeCallback(Callback);
    }

    /*extern ODE_API */
    //dTriTriMergeCallback *dGeomTriMeshGetTriMergeCallback(dGeomID g)
    DTriMesh.DTriTriMergeCallback dGeomTriMeshGetTriMergeCallback()
    {
        //dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

        //DxTriMesh mesh = (DxTriMesh) g;
        return retrieveTriMergeCallback();
    }


    // *****************************************
    // Java API
    // *****************************************
    //	void dGeomTriMeshSetCallback(dGeomID g, dTriCallback* Callback)
    public void setCallback(DTriMesh.DTriCallback Callback) {
        dGeomTriMeshSetCallback(Callback);
    }

    //dTriCallback* dGeomTriMeshGetCallback(dGeomID g)
    public DTriMesh.DTriCallback getCallback() {
        return dGeomTriMeshGetCallback();
    }

    //void dGeomTriMeshSetArrayCallback(dGeomID g, dTriArrayCallback* ArrayCallback)
    public void setArrayCallback(DTriMesh.DTriArrayCallback ArrayCallback) {
        dGeomTriMeshSetArrayCallback(ArrayCallback);
    }

    //dTriArrayCallback* dGeomTriMeshGetArrayCallback(dGeomID g)
    public DTriMesh.DTriArrayCallback getArrayCallback() {
        return dGeomTriMeshGetArrayCallback();
    }

    //void dGeomTriMeshSetRayCallback(dGeomID g, dTriRayCallback* Callback)
    public void setRayCallback(DTriMesh.DTriRayCallback Callback) {
        dGeomTriMeshSetRayCallback(Callback);
    }

    //dTriRayCallback* dGeomTriMeshGetRayCallback(dGeomID g)
    public DTriMesh.DTriRayCallback getRayCallback() {
        return dGeomTriMeshGetRayCallback();
    }

    //void dGeomTriMeshSetTriMergeCallback(dGeomID g, dTriTriMergeCallback* Callback)
    public void setTriMergeCallback(DTriMesh.DTriTriMergeCallback Callback) {
        dGeomTriMeshSetTriMergeCallback(Callback);
    }

    //dTriTriMergeCallback* dGeomTriMeshGetTriMergeCallback(dGeomID g)
    public DTriMesh.DTriTriMergeCallback getTriMergeCallback() {
        return dGeomTriMeshGetTriMergeCallback();
    }

}
