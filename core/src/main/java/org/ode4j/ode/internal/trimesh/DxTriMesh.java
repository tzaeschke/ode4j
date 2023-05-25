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
import org.ode4j.ode.DAABB;
import org.ode4j.ode.DTriMesh;
import org.ode4j.ode.OdeConfig;
import org.ode4j.ode.internal.*;
import org.ode4j.ode.internal.gimpact.GimGeometry;
import org.ode4j.ode.internal.gimpact.GimTrimesh;

import static org.ode4j.ode.internal.gimpact.GimGeometry.IDENTIFY_MATRIX_4X4;

//    typedef dxMeshBase dxTriMesh_Parent;
//    struct dxTriMesh: public dxTriMesh_Parent {
public abstract class DxTriMesh extends DxMeshBase implements DTriMesh {

    @SuppressWarnings("deprecation")
    public static DxTriMesh dCreateTriMesh(DxSpace space,
                                            DxTriMeshData Data,
                                            DTriCallback Callback,
                                            DTriArrayCallback ArrayCallback,
                                            DTriRayCallback RayCallback)
    {
        DxTriMesh Geom;
        switch (OdeConfig.dTRIMESH_TYPE) {
            case DISABLED: Geom = new DxTriMeshDisabled(space, Data); break;
            case GIMPACT: Geom = new DxGimpact(space, (DxGimpactData) Data, Callback, ArrayCallback, RayCallback); break;
            default: throw new IllegalArgumentException(OdeConfig.dTRIMESH_TYPE.name());
        }
        //		Geom.Callback = Callback;
        //		Geom.ArrayCallback = ArrayCallback;
        //		Geom.RayCallback = RayCallback;
        //
        return Geom;
    }

    //public:
    // Functions
    //dxTriMesh(dxSpace * Space, dxTriMeshData * Data, dTriCallback * Callback, dTriArrayCallback * ArrayCallback, dTriRayCallback * RayCallback):
    //dxTriMesh_Parent(Space, NULL, Callback, ArrayCallback, RayCallback, true) // TC has speed/space 'issues' that don't make it a clear win by default on spheres/boxes.
    @SuppressWarnings("deprecation")
    protected DxTriMesh(DxSpace Space, DxTriMeshData Data, DTriMesh.DTriCallback Callback,
              DTriMesh.DTriArrayCallback ArrayCallback, DTriMesh.DTriRayCallback RayCallback) {
        // TC has speed/space 'issues' that don't make it a clear win by default on spheres/boxes.
        super(Space, null, Callback, ArrayCallback, RayCallback, true);
        //gim_init_buffer_managers(m_buffer_managers);
        assignMeshData(Data);
    }

    //~dxTriMesh();
    public void Destructor() {
        //
    }

    public void clearTCCache () { /* do nothing */ }

    //virtual void computeAABB ();

    //public:
    //dxTriMeshData * retrieveMeshData() const{
    public DxTriMeshData retrieveMeshData() {
        return getMeshData();
    }

    //unsigned getMeshTriangleCount () const{
    public int getMeshTriangleCount () {
        //return gim_trimesh_get_triangle_count(const_cast < GIM_TRIMESH * > ( & m_collision_trimesh));
        return m_collision_trimesh.gim_trimesh_get_triangle_count();
    }

    //void fetchMeshTransformedTriangle (dVector3 *const pout_triangle[3], unsigned index)
    public void fetchMeshTransformedTriangle (
            DVector3 pout_triangle0, DVector3 pout_triangle1, DVector3 pout_triangle2, int index)
    {
        m_collision_trimesh.gim_trimesh_locks_work_data();
        //            gim_trimesh_get_triangle_vertices( m_collision_trimesh, (GUINT32) index, *pout_triangle[0], *
        //            pout_triangle[1], *pout_triangle[2]);
        m_collision_trimesh.gim_trimesh_get_triangle_vertices(index, pout_triangle0, pout_triangle1, pout_triangle2);
        m_collision_trimesh.gim_trimesh_unlocks_work_data();
    }

    //private:
    //dxTriMeshData * getMeshData() const{
    protected DxTriMeshData getMeshData() {
        //return static_cast < dxTriMeshData * > (dxTriMesh_Parent::getMeshData ());
        return (DxTriMeshData) (super.getMeshData());
    }

    //        public:
    //        enum {
    //            VERTEXINSTANCE_STRIDE = sizeof(vec3f), TRIANGLEINDEX_STRIDE = sizeof(GUINT32) * dMTV__MAX,
    //        } ;
    static final int VERTEXINSTANCE_STRIDE = 3;//sizeof(vec3f);
    static final int TRIANGLEINDEX_STRIDE = 1 /*sizeof(GUINT32)*/ * dMTV__MAX;

    //void assignMeshData (dxTriMeshData * Data);


    static void GIM_AABB_COPY(GimGeometry.aabb3f src, DAABB dst ) {
        dst.set( src.minX, src.maxX, src.minY, src.maxY, src.minZ, src.maxZ );
        //				dst[ 0 ]= (src).minX;
        //				dst[ 1 ]= (src).maxX;
        //				dst[ 2 ]= (src).minY;
        //				dst[ 3 ]= (src).maxY;
        //				dst[ 4 ]= (src).minZ;
        //				dst[ 5 ]= (src).maxZ;
    }


    //////////////////////////////////////////////////////////////////////////
    // Trimesh

    //    dxTriMesh::~
    //    dxTriMesh() {
    public void DESTRUCTOR() {
        //Terminate Trimesh
        if (m_collision_trimesh != null){
            m_collision_trimesh.gim_trimesh_destroy();
        }
        //gim_terminate_buffer_managers(m_buffer_managers);
        super.DESTRUCTOR();
    }

    // TZ: This is not in ODE, but we do it to prevent dependency on DxCollision
    // (and because ODE basically implements it as polymorphic via #ifdef GIMPCAT
    protected abstract void MakeMatrix(GimGeometry.mat4f transform);

    /*virtual */
    //void dxTriMesh::
    protected void computeAABB() {
        //update trimesh transform
        GimGeometry.mat4f transform = new GimGeometry.mat4f();
        IDENTIFY_MATRIX_4X4(transform);
        MakeMatrix(transform);
        m_collision_trimesh.gim_trimesh_set_tranform(transform);

        //Update trimesh boxes
        m_collision_trimesh.gim_trimesh_update();

        GIM_AABB_COPY( m_collision_trimesh.getAabbSet().getGlobalBound(), _aabb );
    }

    @Override
    protected
        //void dxTriMesh::
    void assignMeshData(DxTriMeshData Data) {
        // GIMPACT only supports stride 12, so we need to catch the error early.
        //dUASSERT((unsigned int)Data.retrieveVertexStride() == (unsigned) VERTEXINSTANCE_STRIDE && (unsigned int)
        //dUASSERT((int)Data.retrieveVertexStride() == (int) VERTEXINSTANCE_STRIDE && (int)
        //Data.retrieveTriangleStride() == (int) TRIANGLEINDEX_STRIDE, "Gimpact trimesh only supports a stride of 3 float/int\n" +
        //"This means that you cannot use dGeomTriMeshDataBuildSimple() with Gimpact.\n" +
        //"Change the stride, or use Opcode trimeshes instead.\n");

        //        dxTriMesh_Parent::assignMeshData (Data);
        super.assignMeshData (Data);

        //Create trimesh
//            const vec3f * vertexInstances = Data -> retrieveVertexInstances();
//        if (vertexInstances != NULL) {
//            const GUINT32 * triangleVertexIndices = Data -> retrieveTriangleVertexIndices();
//
//            size_t vertexInstanceCount = Data -> retrieveVertexCount();
//            size_t triangleVertexCount = (size_t) Data -> retrieveTriangleCount() * dMTV__MAX;
//
//            gim_trimesh_create_from_data(m_buffer_managers, & m_collision_trimesh,                           // gimpact mesh
//                    const_cast < vec3f * > (vertexInstances),           // vertices
//                    dCAST_TO_SMALLER(GUINT32, vertexInstanceCount), // nr of verts
//                    0,                                              // copy verts?
//                    const_cast < GUINT32 * > (triangleVertexIndices),   // indices
//                    dCAST_TO_SMALLER(GUINT32, triangleVertexCount), // nr of indices
//                    0,                                              // copy indices?
//                    1                                               // transformed reply
//        );
//        }
          //final GimGeometry.vec3f  vertexInstances = Data.retrieveVertexInstances();
        final float[] vertexInstances = Data.retrieveVertexInstances();
        if (vertexInstances != null) {
            final int[] triangleVertexIndices = Data.retrieveTriangleVertexIndices();

            int vertexInstanceCount = Data.retrieveVertexCount();
            int triangleVertexCount = Data.retrieveTriangleCount() * dMTV__MAX;

            // TODO TZ-CHECK Important, we are reallocating the mesh here, is this useful?
            //    Can we avoid it? Should we avoid it?
            m_collision_trimesh =
                    GimTrimesh.gim_trimesh_create_from_data(
                            vertexInstances, false,
                            triangleVertexIndices, false, true);
            //GimTrimesh.gim_trimesh_create_from_data(
            //        vertexInstances, vertexInstanceCount, false,
            //        triangleVertexIndices, triangleVertexCount, false, true);

            //            gim_trimesh_create_from_data(m_buffer_managers, & m_collision_trimesh,                           // gimpact mesh
            //                    const_cast < vec3f * > (vertexInstances),           // vertices
            //                    dCAST_TO_SMALLER(GUINT32, vertexInstanceCount), // nr of verts
            //                    0,                                              // copy verts?
            //                    const_cast < GUINT32 * > (triangleVertexIndices),   // indices
            //                    dCAST_TO_SMALLER(GUINT32, triangleVertexCount), // nr of indices
            //                    0,                                              // copy indices?
            //                    1                                               // transformed reply
            //        );
        }
    }

    //public:
    //GIM_TRIMESH m_collision_trimesh;
    //GBUFFER_MANAGER_DATA m_buffer_managers[ G_BUFFER_MANAGER__MAX];
    GimTrimesh m_collision_trimesh;
    //GBUFFER_MANAGER_DATA m_buffer_managers[ G_BUFFER_MANAGER__MAX];

    public GimTrimesh m_collision_trimesh() {
        return m_collision_trimesh;
    }

    /*
     * Returns the following values:
     * between -PI and 0 for concave edges
     * 0 for flat edges
     * between 0 and PI for convex edges
     * > PI for boundary edges
     */
    abstract public float getEdgeAngle(int triangle, int edge);

    public DTriRayCallback RayCallback() {
        return m_RayCallback;
    }


    /*extern ODE_API */
    //void dGeomTriMeshGetPoint(dGeomID g, int index, dReal u, dReal v, dVector3 Out)
    void dGeomTriMeshGetPoint(int index, double u, double v, DVector3 Out)
    {
        //dUASSERT(g && g->type == dTriMeshClass, "The argument is not a trimesh");

        //DxTriMesh mesh = (DxTriMesh) g;
        DVector3 dv0 = new DVector3(), dv1 = new DVector3(), dv2 = new DVector3();
        fetchMeshTransformedTriangle(dv0, dv1, dv2, index);

        DxCollisionUtil.GetPointFromBarycentric(dv0, dv1, dv2, u, v, Out);
    }

    /*extern */
    //IFaceAngleStorageView *dxGeomTriMeshGetFaceAngleView(dxGeom *triMeshGeom)
    public IFaceAngleStorageView dxGeomTriMeshGetFaceAngleView()
    {
        //dUASSERT(triMeshGeom && triMeshGeom->type == dTriMeshClass, "The argument is not a trimesh");

        //DxTriMesh mesh = (DxTriMesh) triMeshGeom;
        return retrieveFaceAngleView();
    }

    @Override
    /*extern ODE_API */
    //void dGeomTriMeshGetPoint(dGeomID g, int index, dReal u, dReal v, dVector3 Out)
    public void getPoint(int index, double u, double v, DVector3 Out) {
        dGeomTriMeshGetPoint(index, u, v, Out);
    }


}
