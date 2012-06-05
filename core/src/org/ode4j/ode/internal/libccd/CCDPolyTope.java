/***
 * libccd
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 * Java-port: Copyright (c) 2007-2012 Tilmann Zäschke <ode4j@gmx.de>  
 *
 *
 *  This file is part of libccd.
 *
 *  Distributed under the OSI-approved BSD License (the "License");
 *  see accompanying file BDS-LICENSE for details or see
 *  <http://www.opensource.org/licenses/bsd-license.php>.
 *
 *  This software is distributed WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the License for more information.
 */
package org.ode4j.ode.internal.libccd;

import org.cpp4j.Cstdio;
import org.cpp4j.FILE;
import org.ode4j.ode.internal.libccd.CCDList.ccd_list_t;
import org.ode4j.ode.internal.libccd.CCDSupport.ccd_support_t;
import org.ode4j.ode.internal.libccd.CCDVec3.ccd_vec3_t;
import static org.ode4j.ode.internal.libccd.CCDList.*;
import static org.ode4j.ode.internal.libccd.CCDSupport.*;
import static org.ode4j.ode.internal.libccd.CCDVec3.*;

public class CCDPolyTope {

	public static final int CCD_PT_VERTEX = 1;
	public static final int CCD_PT_EDGE = 2;
	public static final int CCD_PT_FACE = 3;


	//#define __CCD_PT_EL
//	private static class __CCD_PT_EL {
//	    int type;           /*! type of element */ 
//	    double dist;        /*! distance from origin */ 
//	    ccd_vec3_t witness; /*! witness point of projection of origin */ 
//	    ccd_list_t list;    /*! list of elements of same type */
//	}

	/**
	 * General polytope element.
	 * Could be vertex, edge or triangle.
	 */
	public static class ccd_pt_el_t {
//	    __CCD_PT_EL
	    int type;           /*! type of element */ 
	    double dist;        /*! distance from origin */ 
	    final ccd_vec3_t witness = new ccd_vec3_t(); /*! witness point of projection of origin */ 
	    final ccd_list_t list = new ccd_list_t(this);    /*! list of elements of same type */

		public double dist() {
			return dist;
		}
		public int type() {
			return type;
		}
		public ccd_vec3_t witness() {
			return witness;
		}
	};

	/**
	 * Polytope's vertex.
	 */
	public static final class ccd_pt_vertex_t extends ccd_pt_el_t {
//	    __CCD_PT_EL

	    int id;
	    final ccd_support_t v = new ccd_support_t();
	    ccd_list_t<ccd_pt_edge_t> edges = new ccd_list_t<ccd_pt_edge_t>(null); //!< List of edges
		public ccd_support_t v() {
			return v;
		}
	};

	/**
	 * Polytope's edge.
	 */
	public static final class ccd_pt_edge_t extends ccd_pt_el_t {
//	    __CCD_PT_EL

	    final ccd_pt_vertex_t[] vertex = new ccd_pt_vertex_t[2]; //!< Reference to vertices
	    final ccd_pt_face_t[] faces = new ccd_pt_face_t[2]; //!< Reference to faces

	    @SuppressWarnings("unchecked")
		final ccd_list_t<ccd_pt_vertex_t>[] vertex_list = new ccd_list_t[2]; //!< List items in vertices' lists
		{
			vertex_list[0] = new ccd_list_t<ccd_pt_vertex_t>(null);
			vertex_list[1] = new ccd_list_t<ccd_pt_vertex_t>(null);
		}
	};

	/**
	 * Polytope's triangle faces.
	 */
	public static final class ccd_pt_face_t extends ccd_pt_el_t {
//	    __CCD_PT_EL

	    final ccd_pt_edge_t[] edge = new ccd_pt_edge_t[3]; //!< Reference to surrounding edges
	};


	/**
	 * Struct containing polytope.
	 */
	public static final class ccd_pt_t {
	    ccd_list_t<ccd_pt_vertex_t> vertices = new ccd_list_t<ccd_pt_vertex_t>(null); //!< List of vertices
	    ccd_list_t<ccd_pt_edge_t> edges = new ccd_list_t<ccd_pt_edge_t>(null); //!< List of edges
	    ccd_list_t<ccd_pt_face_t> faces = new ccd_list_t<ccd_pt_face_t>(null); //!< List of faces

	    ccd_pt_el_t nearest;
	    double nearest_dist;
	    int nearest_type;
	};


	/**** INLINES ****/
	public static final ccd_pt_vertex_t ccdPtAddVertexCoords(ccd_pt_t pt,
	                                                  double x, double y, double z)
	{
	    ccd_support_t s = new ccd_support_t();
	    ccdVec3Set(s.v, x, y, z);
	    return ccdPtAddVertex(pt, s);
	}

	/**
	 * Deletes vertex from polytope.
	 * Returns 0 on success, -1 otherwise.
	 */
	public static final int ccdPtDelVertex(ccd_pt_t pt, ccd_pt_vertex_t v)
	{
	    // test if any edge is connected to this vertex
	    if (!ccdListEmpty(v.edges))
	        return -1;

	    // delete vertex from main list
	    ccdListDel(v.list);

	    if (pt.nearest == v){
	        pt.nearest = null;
	    }

	    //free(v);
	    return 0;
	}

	public static final int ccdPtDelEdge(ccd_pt_t pt, ccd_pt_edge_t e)
	{
	    // text if any face is connected to this edge (faces[] is always
	    // aligned to lower indices)
	    if (e.faces[0] != null)
	        return -1;

	    // disconnect edge from lists of edges in vertex struct
	    ccdListDel(e.vertex_list[0]);
	    ccdListDel(e.vertex_list[1]);

	    // disconnect edge from main list
	    ccdListDel(e.list);

	    if (pt.nearest == e){
	        pt.nearest = null;
	    }

	    //free(e);
	    return 0;
	}

	public static final int ccdPtDelFace(final ccd_pt_t pt, ccd_pt_face_t f)
	{
	    ccd_pt_edge_t e;
	    int i;

	    // remove face from edges' recerence lists
	    for (i = 0; i < 3; i++){
	        e = f.edge[i];
	        if (e.faces[0] == f){
	            e.faces[0] = e.faces[1];
	        }
	        e.faces[1] = null;
	    }

	    // remove face from list of all faces
	    ccdListDel(f.list);

	    if (pt.nearest == f){
	        pt.nearest = null;
	    }

	    //free(f);
	    return 0;
	}

	/**
	 * Returns vertices surrounding given triangle face.
	 */
	@Deprecated
	static final void ccdPtFaceVec3(final ccd_pt_face_t face,
	                               ccd_vec3_t[][] a,
	                               ccd_vec3_t[][] b,
	                               ccd_vec3_t[][] c)
	{
		throw new UnsupportedOperationException("This should be inlined manually");
//	    *a = face.edge[0].vertex[0].v.v;
//	    *b = face.edge[0].vertex[1].v.v;
//
//	    if (face.edge[1].vertex[0] != face.edge[0].vertex[0]
//	            && face.edge[1].vertex[0] != face.edge[0].vertex[1]){
//	        *c = face.edge[1].vertex[0].v.v;
//	    }else{
//	        *c = face.edge[1].vertex[1].v.v;
//	    }
	}

	@Deprecated
	static final void ccdPtFaceVertices(final ccd_pt_face_t face,
	                                   ccd_pt_vertex_t[][] a,
	                                   ccd_pt_vertex_t[][] b,
	                                   ccd_pt_vertex_t[][] c)
	{
		throw new UnsupportedOperationException(); //TZ implement?
//	    *a = face.edge[0].vertex[0];
//	    *b = face.edge[0].vertex[1];
//
//	    if (face.edge[1].vertex[0] != face.edge[0].vertex[0]
//	            && face.edge[1].vertex[0] != face.edge[0].vertex[1]){
//	        *c = face.edge[1].vertex[0];
//	    }else{
//	        *c = face.edge[1].vertex[1];
//	    }
	}

//	static final void ccdPtFaceEdges(final ccd_pt_face_t f,
//	                                ccd_pt_edge_t[][] a,
//	                                ccd_pt_edge_t[][] b,
//	                                ccd_pt_edge_t[][] c)
//	{
//	    *a = f.edge[0];
//	    *b = f.edge[1];
//	    *c = f.edge[2];
//	}
	static final void ccdPtFaceEdges(final ccd_pt_face_t f,
			ccd_pt_edge_t[] abc, int pos1, int pos2, int pos3)
	{
		abc[pos1] = f.edge[0];
		abc[pos2] = f.edge[1];
		abc[pos3] = f.edge[2];
	}

	@Deprecated
	static final void ccdPtEdgeVec3(final ccd_pt_edge_t e,
	                               ccd_vec3_t[][] a,
	                               ccd_vec3_t[][] b)
	{
		throw new UnsupportedOperationException("This should be inlined manually");
//	    *a = e.vertex[0].v.v;
//	    *b = e.vertex[1].v.v;
	}

//	static final void ccdPtEdgeVertices(final ccd_pt_edge_t e,
//	                                   ccd_pt_vertex_t[][] a,
//	                                   ccd_pt_vertex_t[][] b)
//	{
//	    *a = e.vertex[0];
//	    *b = e.vertex[1];
//	}
	static final void ccdPtEdgeVertices(final ccd_pt_edge_t e,
			ccd_pt_vertex_t[] ab, int a1, int a2)
	{
		ab[a1] = e.vertex[0];
		ab[a2] = e.vertex[1];
	}

//	static final void ccdPtEdgeFaces(final ccd_pt_edge_t e,
//	                                ccd_pt_face_t[][] f1,
//	                                ccd_pt_face_t[][] f2)
//	{
//	    *f1 = e.faces[0];
//	    *f2 = e.faces[1];
//	}
	static final void ccdPtEdgeFaces(final ccd_pt_edge_t e,
			ccd_pt_face_t[] f12, int pos1, int pos2)
	{
		f12[pos1] = e.faces[0];
		f12[pos2] = e.faces[1];
	}


	/***
	 * libccd
	 * ---------------------------------
	 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
	 *
	 *
	 *  This file is part of libccd.
	 *
	 *  Distributed under the OSI-approved BSD License (the "License");
	 *  see accompanying file BDS-LICENSE for details or see
	 *  <http://www.opensource.org/licenses/bsd-license.php>.
	 *
	 *  This software is distributed WITHOUT ANY WARRANTY; without even the
	 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
	 *  See the License for more information.
	 */

	private static final void _ccdPtNearestUpdate(ccd_pt_t pt, ccd_pt_el_t el)
	{
	    if (ccdEq(pt.nearest_dist, el.dist)){
	        if (el.type < pt.nearest_type){
	            pt.nearest = el;
	            pt.nearest_dist = el.dist;
	            pt.nearest_type = el.type;
	        }
	    }else if (el.dist < pt.nearest_dist){
	        pt.nearest = el;
	        pt.nearest_dist = el.dist;
	        pt.nearest_type = el.type;
	    }
	}

	private static void _ccdPtNearestRenew(ccd_pt_t pt)
	{
	    //ccd_pt_vertex_t v;
	    //ccd_pt_edge_t e;
	    //ccd_pt_face_t f;

	    pt.nearest_dist = CCD_REAL_MAX;
	    pt.nearest_type = 3;
	    pt.nearest = null;

	    //ccdListForEachEntry(pt.vertices, v, ccd_pt_vertex_t, list){
    	System.err.println("xxx1");
	    for (ccd_pt_el_t v: pt.vertices) {
	    	System.err.println("xxx2");
	        _ccdPtNearestUpdate(pt, (ccd_pt_el_t)v);
	    }

	    //ccdListForEachEntry(pt.edges, e, ccd_pt_edge_t, list){
	    for (ccd_pt_el_t e: pt.edges) {
	        _ccdPtNearestUpdate(pt, (ccd_pt_el_t)e);
	    }

	    //ccdListForEachEntry(pt.faces, f, ccd_pt_face_t, list){
	    for (ccd_pt_el_t f: pt.faces) {
	        _ccdPtNearestUpdate(pt, (ccd_pt_el_t)f);
	    }
	}



	public static final void ccdPtInit(ccd_pt_t pt)
	{
//	    ccdListInit(pt.vertices);
//	    ccdListInit(pt.edges);
//	    ccdListInit(pt.faces);

	    pt.nearest = null;
	    pt.nearest_dist = CCD_REAL_MAX;
	    pt.nearest_type = 3;
	}

	public static final void ccdPtDestroy(ccd_pt_t pt)
	{
	    //ccd_pt_face_t f, f2;
	    //ccd_pt_edge_t e, e2;
	    //ccd_pt_vertex_t v, v2;

	    // first delete all faces
	    //ccdListForEachEntrySafe(pt.faces, f, ccd_pt_face_t, f2, ccd_pt_face_t, list){
	    for (ccd_pt_face_t f: pt.faces) {
	        ccdPtDelFace(pt, f);
	    }

	    // delete all edges
	    //ccdListForEachEntrySafe(pt.edges, e, ccd_pt_edge_t, e2, ccd_pt_edge_t, list){
	    for (ccd_pt_edge_t e: pt.edges) {
	        ccdPtDelEdge(pt, e);
	    }

	    // delete all vertices
	    //ccdListForEachEntrySafe(pt.vertices, v, ccd_pt_vertex_t, v2, ccd_pt_vertex_t, list){
	    for (ccd_pt_vertex_t v: pt.vertices) {
	        ccdPtDelVertex(pt, v);
	    }
	}


	/**
	 * Adds vertex to polytope and returns pointer to newly created vertex.
	 */
	static final ccd_pt_vertex_t ccdPtAddVertex(ccd_pt_t pt, final ccd_support_t v)
	{
	    ccd_pt_vertex_t vert;

	    vert = new ccd_pt_vertex_t();//CCD_ALLOC(ccd_pt_vertex_t);
	    vert.type = CCD_PT_VERTEX;
	    ccdSupportCopy(vert.v, v);

	    vert.dist = ccdVec3Len2(vert.v.v);
	    ccdVec3Copy(vert.witness, vert.v.v);

	    ccdListInit(vert.edges);

	    // add vertex to list
	    ccdListAppend(pt.vertices, vert.list);

	    // update position in .nearest array
	    _ccdPtNearestUpdate(pt, (ccd_pt_el_t)vert);

	    return vert;
	}

	/**
	 * Adds edge to polytope.
	 */
	public static final ccd_pt_edge_t ccdPtAddEdge(ccd_pt_t pt, ccd_pt_vertex_t v1,
	                                          ccd_pt_vertex_t v2)
	{
	    final ccd_vec3_t a, b;
	    ccd_pt_edge_t edge;

	    edge = new ccd_pt_edge_t();//CCD_ALLOC(ccd_pt_edge_t);
	    edge.type = CCD_PT_EDGE;
	    edge.vertex[0] = v1;
	    edge.vertex[1] = v2;
	    edge.faces[0] = edge.faces[1] = null;

	    a = edge.vertex[0].v.v;
	    b = edge.vertex[1].v.v;
	    edge.dist = ccdVec3PointSegmentDist2(ccd_vec3_origin, a, b, edge.witness);

	    //TZ TODO this is weird, adding vertices to the edge-list????
	    ccdListAppend(edge.vertex[0].edges, edge.vertex_list[0]);
	    ccdListAppend(edge.vertex[1].edges, edge.vertex_list[1]);

	    ccdListAppend(pt.edges, edge.list);

	    // update position in .nearest array
	    _ccdPtNearestUpdate(pt, (ccd_pt_el_t )edge);

	    return edge;
	}

	/**
	 * Adds face to polytope.
	 */
	public static final ccd_pt_face_t ccdPtAddFace(ccd_pt_t pt, ccd_pt_edge_t e1,
	                                          ccd_pt_edge_t e2,
	                                          ccd_pt_edge_t e3)
	{
	    final ccd_vec3_t a, b, c;
	    ccd_pt_face_t face;
	    ccd_pt_edge_t e;
	    int i;

	    face = new ccd_pt_face_t();// CCD_ALLOC(ccd_pt_face_t);
	    face.type = CCD_PT_FACE;
	    face.edge[0] = e1;
	    face.edge[1] = e2;
	    face.edge[2] = e3;

	    // obtain triplet of vertices
	    a = face.edge[0].vertex[0].v.v;
	    b = face.edge[0].vertex[1].v.v;
	    e = face.edge[1];
	    if (e.vertex[0] != face.edge[0].vertex[0]
	            && e.vertex[0] != face.edge[0].vertex[1]){
	        c = e.vertex[0].v.v;
	    }else{
	        c = e.vertex[1].v.v;
	    }
	    face.dist = ccdVec3PointTriDist2(ccd_vec3_origin, a, b, c, face.witness);


	    for (i = 0; i < 3; i++){
	        if (face.edge[i].faces[0] == null){
	            face.edge[i].faces[0] = face;
	        }else{
	            face.edge[i].faces[1] = face;
	        }
	    }

	    ccdListAppend(pt.faces, face.list);

	    // update position in .nearest array
	    _ccdPtNearestUpdate(pt, (ccd_pt_el_t )face);

	    return face;
	}


	/**
	 * Recompute distances from origin for all elements in pt.
	 */
	private static final void ccdPtRecomputeDistances(ccd_pt_t pt)
	{
	    //ccd_pt_vertex_t v;
	    //ccd_pt_edge_t e;
	    //ccd_pt_face_t f;
	    ccd_vec3_t a, b, c;
	    double dist;

	    //ccdListForEachEntry(pt.vertices, v, ccd_pt_vertex_t, list){
	    for (ccd_pt_vertex_t v: pt.vertices) {
	    	dist = ccdVec3Len2(v.v.v);
	        v.dist = dist;
	        ccdVec3Copy(v.witness, v.v.v);
	    }

	    //ccdListForEachEntry(pt.edges, e, ccd_pt_edge_t, list){
	    for (ccd_pt_edge_t e: pt.edges) {
	        a = e.vertex[0].v.v;
	        b = e.vertex[1].v.v;
	        dist = ccdVec3PointSegmentDist2(ccd_vec3_origin, a, b, e.witness);
	        e.dist = dist;
	    }

	    //ccdListForEachEntry(pt.faces, f, ccd_pt_face_t, list){
	    for (ccd_pt_face_t f: pt.faces) {
	        // obtain triplet of vertices
	        a = f.edge[0].vertex[0].v.v;
	        b = f.edge[0].vertex[1].v.v;
	        ccd_pt_edge_t e = f.edge[1];
	        if (e.vertex[0] != f.edge[0].vertex[0]
	                && e.vertex[0] != f.edge[0].vertex[1]){
	            c = e.vertex[0].v.v;
	        }else{
	            c = e.vertex[1].v.v;
	        }

	        dist = ccdVec3PointTriDist2(ccd_vec3_origin, a, b, c, f.witness);
	        f.dist = dist;
	    }
	}

	/**
	 * Returns nearest element to origin.
	 */
	public static final ccd_pt_el_t ccdPtNearest(ccd_pt_t pt)
	{
	    if (pt.nearest==null){
	        _ccdPtNearestRenew(pt);
	    }
	    return pt.nearest;
	}


	private static final void ccdPtDumpSVT(ccd_pt_t pt, String fn)//const char *fn)
	{
	    FILE fout;

	    fout = Cstdio.fopen(fn, "a");
	    if (fout == null)
	        return;

	    ccdPtDumpSVT2(pt, fout);

	    Cstdio.fclose(fout);
	}

	private static final void ccdPtDumpSVT2(ccd_pt_t pt, FILE fout)
	{
	    //ccd_pt_vertex_t v; 
	    ccd_pt_vertex_t a, b, c;
	    //ccd_pt_edge_t e;
	    //ccd_pt_face_t f;
	    int i;

	    Cstdio.fprintf(fout, "-----\n");

	    Cstdio.fprintf(fout, "Points:\n");
	    i = 0;
	    //ccdListForEachEntry(pt.vertices, v, ccd_pt_vertex_t, list){
	    for (ccd_pt_vertex_t v: pt.vertices) {
	        v.id = i++;
	        Cstdio.fprintf(fout, "%lf %lf %lf\n",
	                ccdVec3X(v.v.v), ccdVec3Y(v.v.v), ccdVec3Z(v.v.v));
	    }

	    Cstdio.fprintf(fout, "Edges:\n");
	    //ccdListForEachEntry(pt.edges, e, ccd_pt_edge_t, list){
	    for (ccd_pt_edge_t e: pt.edges) {
	    	Cstdio.fprintf(fout, "%d %d\n", e.vertex[0].id, e.vertex[1].id);
	    }

	    Cstdio.fprintf(fout, "Faces:\n");
	    //ccdListForEachEntry(pt.faces, f, ccd_pt_face_t, list){
	    for (ccd_pt_face_t f: pt.faces) {
	        a = f.edge[0].vertex[0];
	        b = f.edge[0].vertex[1];
	        c = f.edge[1].vertex[0];
	        if (c == a || c == b){
	            c = f.edge[1].vertex[1];
	        }
	        Cstdio.fprintf(fout, "%d %d %d\n", a.id, b.id, c.id);
	    }
	}

}
