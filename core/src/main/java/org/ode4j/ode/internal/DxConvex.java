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

import static org.ode4j.ode.OdeConstants.CONTACTS_UNIMPORTANT;
import static org.ode4j.ode.OdeConstants.dInfinity;
import static org.ode4j.ode.OdeMath.*;
import static org.ode4j.ode.internal.DxCollisionUtil.dVector3Copy;
import static org.ode4j.ode.internal.cpp4j.Cmath.fabs;
import static org.ode4j.ode.internal.cpp4j.Cstdio.fprintf;
import static org.ode4j.ode.internal.cpp4j.Cstdio.stdout;

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DColliderFn;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DConvex;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;
import org.ode4j.ode.internal.cpp4j.java.RefInt;

/**
 * Code for Convex Collision Detection
 * By Rodrigo Hernandez
 */
public class DxConvex extends DxGeom implements DConvex {

	/** An array of planes in the form:
	   normal X, normal Y, normal Z,Distance */
//	private double[] planes;
	private DVector3[] planesV;
	private double[] planesD;

	/** An array of points X,Y,Z. */
	private double[] points;

	/** 
	 * An array of indices to the points of each polygon, it should be the 
	 * number of vertices followed by that amount of indices to "points" in 
	 * counter clockwise order */
	//unsigned
	private int[] polygons;

	/** Amount of planes in planes. */
	//unsigned
	private int planecount;
	
	/** Amount of points in points. */
	//unsigned
	private int pointcount;

	/** Amount of edges in convex. */
	//unsigned
	private int edgecount;
	/**  */
	//private double[] saabb=new double[6];/*!< Static AABB */
	//dxConvex(dSpace space,
	//  dReal *planes,
	//  unsigned int planecount,
	//  dReal *points,
	//  unsigned int pointcount,
	//  unsigned int *polygons);
	//~dxConvex()
	//TZ does nothing important, commented out.
	//	  public void DESTRUCTOR()
	//	  {
	//		  //if((edgecount!=0)&&(edges!=NULL)) delete[] edges;
	////		  if (edgecount != 0 && edges != null) {
	////			  for (Edge e: edges) e.DESTRUCTOR();
	////		  }
	//		  super.DESTRUCTOR();
	//	  }
	//void computeAABB();
	private static class Edge
	{
		//unsigned 
		int first;
		//unsigned 
		int second;
	}
	//edge* edges;
	private Edge[] edges;

	/** 
	 * A Support mapping function for convex shapes.
	 * @param dir [IN] direction to find the Support Point for
	 * @return the index of the support vertex.
	 */
	//inline unsigned int SupportIndex(dVector3 dir)
	private int SupportIndex(DVector3 dir)
	{
		DVector3 rdir = new DVector3();
		//unsigned 
		int index=0;
		dMultiply1_331 (rdir,final_posr().R(),dir);
		double max = dCalcVectorDot3(points,0,rdir);
		double tmp;
		for (int i = 1; i < pointcount; ++i)
		{
			tmp = dCalcVectorDot3(points,(i*3),rdir);
			if (tmp > max)
			{
				index=i;
				max = tmp;
			}
		}
		return index;
	}

	//private:
	// For Internal Use Only
	/*! \brief Fills the edges dynamic array based on points and polygons.
	 */
	//void FillEdges();
	//#if 0
	///*
	//What this does is the same as the Support function by doing some preprocessing
	//for optimization. Not complete yet.
	//*/
	//// Based on Eberly's Game Physics Book page 307
	//struct Arc
	//{
	// // indices of polyhedron normals that form the spherical arc
	// int normals[2];
	// // index of edge shared by polyhedron faces
	// int edge;
	//};
	//struct Polygon
	//{
	// // indices of polyhedron normals that form the spherical polygon
	// std::vector<int> normals;
	// // index of extreme vertex corresponding to this polygon
	// int vertex;
	//};
	//// This is for extrem feature query and not the usual level BSP structure (that comes later)
	//struct BSPNode
	//{
	//// Normal index (interior node), vertex index (leaf node)
	//int normal;
	//// if Dot (E,D)>=0, D gets propagated to this child
	//BSPNode* right;
	//// if Dot (E,D)<0, D gets propagated to this child
	//BSPNode* left;
	//};
	//void CreateTree();
	//BSPNode* CreateNode(std::vector<Arc> Arcs,std::vector<Polygon> Polygons);
	//void GetFacesSharedByVertex(int i, std::vector<int> f);
	//void GetFacesSharedByEdge(int i, int* f);
	//void GetFaceNormal(int i, dVector3 normal);
	//BSPNode* tree;
	//#endif





	//#if 1
	//#define dMIN(A,B)  ((A)>(B) ? (B) : (A))
	//#define dMAX(A,B)  ((A)>(B) ? (A) : (B))
	//#else
	//#define dMIN(A,B)  std::min(A,B)
	//#define dMAX(A,B)  std::max(A,B)
	//#endif
	private static double dMIN(double a, double b) { return Math.min(a, b); }
	private static double dMAX(double a, double b) { return Math.max(a, b); }
	private static int dMIN(int a, int b) { return Math.min(a, b); }
	private static int dMAX(int a, int b) { return Math.max(a, b); }

	//****************************************************************************
	// Convex public API
	//	dxConvex::dxConvex (dSpace space,
	//		    dReal *_planes,
	//		    unsigned int _planecount,
	//		    dReal *_points,
	//		    unsigned int _pointcount,
	//		    unsigned int *_polygons) : 
	//  dxGeom (space,1)
	DxConvex (DxSpace space,
			double[] _planes,
			int _planecount,
			double[] _points,
			int _pointcount,
			int[] _polygons) 
			{
		super(space, true);
		dAASSERT (_planes != null);
		dAASSERT (_points != null);
		dAASSERT (_polygons != null);
		//fprintf(stdout,"dxConvex Constructor planes %X\n",_planes);
		type = dConvexClass;
		//planes = _planes;
		planesV = new DVector3[_planecount];
		planesD = new double[_planecount];
		for (int i = 0; i < _planecount; i++) {
			planesV[i] = new DVector3(_planes[i*4], _planes[i*4+1], _planes[i*4+2]);
			planesD[i] = _planes[i*4+3];
		}
		planecount = _planecount;
		// we need points as well
		points = _points;
		pointcount = _pointcount;
		polygons=_polygons;
		edges = null;
		FillEdges();
		if (!dNODEBUG) {//#ifndef dNODEBUG
			// Check for properly build polygons by calculating the determinant
			// of the 3x3 matrix composed of the first 3 points in the polygon.
			//int[] points_in_poly=polygons;
			int points_in_polyPos = 0;
			//int[] index=polygons+1;
			int indexPos = 1; 

			for(int i=0;i<planecount;++i)
			{
				dAASSERT (polygons[points_in_polyPos] > 2); //(*points_in_poly > 2 );
				int index03 = polygons[indexPos] * 3;
				int index13 = polygons[indexPos+1] * 3;
				int index23 = polygons[indexPos+2] * 3;
				if((
						points[(index03)+0]*points[(index13)+1]*points[(index23)+2] + 
						points[(index03)+1]*points[(index13)+2]*points[(index23)+0] + 
						points[(index03)+2]*points[(index13)+0]*points[(index23)+1] -
						points[(index03)+2]*points[(index13)+1]*points[(index23)+0] - 
						points[(index03)+1]*points[(index13)+0]*points[(index23)+2] - 
						points[(index03)+0]*points[(index13)+2]*points[(index23)+1])<0)
				{
					fprintf(stdout,"WARNING: Polygon %d is not defined counterclockwise\n",i);
				}
				//points_in_poly+=(*points_in_poly+1);
				points_in_polyPos+= polygons[points_in_polyPos]+1;
				indexPos=points_in_polyPos+1;//index=points_in_poly+1;
				//if(planes[(i*4)+3]<0) fprintf(stdout,"WARNING: Plane %d does not contain the origin\n",i);
				if(planesD[i]<0) fprintf(stdout,"WARNING: Plane %d does not contain the origin\n",i);
			}
		} //#endif dNODEBUG

		//CreateTree();
			}


	@Override
    protected void computeAABB()
	{
		// this can, and should be optimized
		DVector3 point = new DVector3();
		dMultiply0_331 (point, final_posr().R(), points,0);
//		_aabb.v[0] = point.v[0]+_final_posr.pos.v[0];
//		_aabb.v[1] = point.v[0]+_final_posr.pos.v[0];
//		_aabb.v[2] = point.v[1]+_final_posr.pos.v[1];
//		_aabb.v[3] = point.v[1]+_final_posr.pos.v[1];
//		_aabb.v[4] = point.v[2]+_final_posr.pos.v[2];
//		_aabb.v[5] = point.v[2]+_final_posr.pos.v[2];
		_aabb.setMin( final_posr().pos() );
		_aabb.setMax( final_posr().pos() );
		_aabb.shiftPos( point );
		for(int i=3;i<(pointcount*3);i+=3)
		{
			dMultiply0_331 (point, final_posr().R(), points,i);
//			_aabb.v[0] = dMIN(_aabb.v[0],point.v[0]+_final_posr.pos.v[0]);
//			_aabb.v[1] = dMAX(_aabb.v[1],point.v[0]+_final_posr.pos.v[0]);
//			_aabb.v[2] = dMIN(_aabb.v[2],point.v[1]+_final_posr.pos.v[1]);
//			_aabb.v[3] = dMAX(_aabb.v[3],point.v[1]+_final_posr.pos.v[1]);
//			_aabb.v[4] = dMIN(_aabb.v[4],point.v[2]+_final_posr.pos.v[2]);
//			_aabb.v[5] = dMAX(_aabb.v[5],point.v[2]+_final_posr.pos.v[2]);
			DVector3 tmp = new DVector3();
			tmp.eqSum( point, final_posr().pos() );
			_aabb.expand( tmp );
		}
	}

	/** 
	 * Populates the edges set, should be called only once whenever
	 * the polygon array gets updated.
	 */
	void FillEdges()
	{
		//int[] points_in_poly=polygons;
		int points_in_polyPos = 0;
		//int[] index=polygons+1;
		int indexPos = 1; 
		if (edges!=null) edges = null;//delete[] edges;
		edgecount = 0;
		Edge e = new Edge();
		boolean isinset;
		for( int i=0;i<planecount;++i)
		{
			for( int j=0;j<polygons[points_in_polyPos];++j)
			{
//				e.first = dMIN(index[j],index[(j+1)%*points_in_poly]);
//				e.second = dMAX(index[j],index[(j+1)%*points_in_poly]);
				e.first = dMIN(polygons[indexPos + j],polygons[indexPos + (j+1)%polygons[points_in_polyPos]]);
				e.second = dMAX(polygons[indexPos + j],polygons[indexPos + (j+1)%polygons[points_in_polyPos]]);
				isinset=false;
				for( int k=0;k<edgecount;++k)
				{
					if((edges[k].first==e.first)&&(edges[k].second==e.second))
					{
						isinset=true;
						break;
					}
				}
				if(!isinset)
				{
					Edge[] tmp = new Edge[edgecount+1];				
					if(edgecount!=0)
					{
//						memcpy(tmp,edges,(edgecount)*sizeof(edge));
//						delete[] edges;
						for (int ii=0; ii<edges.length; ii++) tmp[ii] = edges[ii];
					}
					tmp[edgecount] = new Edge();
					tmp[edgecount].first=e.first;
					tmp[edgecount].second=e.second;
					edges = tmp;
					++edgecount;
				}
			}
			//points_in_poly+=(*points_in_poly+1);
			points_in_polyPos+= polygons[points_in_polyPos]+1;
			indexPos=points_in_polyPos+1;//index=points_in_poly+1;
		}
	}
	//#if 0
	//dxConvex::BSPNode* dxConvex::CreateNode(std::vector<Arc> Arcs,std::vector<Polygon> Polygons)
	//{
	//#if 0	
	//	dVector3 ea,eb,e;
	//	dVector3Copy(points+((edges.begin()+Arcs[0].edge)first*3),ea);
	//      dMULTIPLY0_331(e1b,cvx1.final_posr->R,cvx1.points+(i->second*3));
	//
	//	dVector3Copy(points[edges[Arcs[0].edge]
	//#endif
	//	return NULL;
	//}
	//
	//void dxConvex::CreateTree()
	//{
	//	std::vector<Arc> A;
	//	A.reserve(edgecount);
	//	for(unsigned int i=0;i<edgecount;++i)
	//	{
	//		this->GetFacesSharedByEdge(i,A[i].normals);
	//		A[i].edge = i;
	//	}
	//	std::vector<Polygon> S;
	//	S.reserve(pointcount);
	//	for(unsigned int i=0;i<pointcount;++i)
	//	{
	//		this->GetFacesSharedByVertex(i,S[i].normals);
	//		S[i].vertex=i;
	//	}
	//	this->tree = CreateNode(A,S);
	//}
	//
	//void dxConvex::GetFacesSharedByVertex(int i, std::vector<int> f)
	//{
	//}
	//void dxConvex::GetFacesSharedByEdge(int i, int* f)
	//{
	//}
	//void dxConvex::GetFaceNormal(int i, dVector3 normal)
	//{
	//}
	//#endif

	//dGeom dCreateConvex (dSpace space,dReal *_planes,unsigned int _planecount,
	//		    dReal *_points,
	//		       unsigned int _pointcount,
	//		       unsigned int *_polygons)
	public static DConvex dCreateConvex (DxSpace space, double[] planes, int planecount,
										 double[] points, int pointcount, int[] polygons)
	{
		//fprintf(stdout,"dxConvex dCreateConvex\n");
		return new DxConvex(space, planes, planecount,
				points,
				pointcount,
				polygons);
	}

	// Only in ode4j.
	public static DConvex dCreateConvex (DxSpace space, double[] planes, double[] points, int[] polygons)
	{
		//fprintf(stdout,"dxConvex dCreateConvex\n");
		dUASSERT(planes.length % 4 == 0, "planes are defined by four values: x, y, z, depth");
		dUASSERT(points.length % 3 == 0, "points are defined by three values: x, y, z");
		return new DxConvex(space, planes, planes.length / 4, points, points.length / 3, polygons);
	}

	//void dGeomSetConvex (dGeom g,dReal *_planes,unsigned int _planecount,
	//		     dReal *_points,
	//		     unsigned int _pointcount,
	//		     unsigned int *_polygons)
	void dGeomSetConvex (double[] planes, int planecount,
						 double[] points, int pointcount, int[] polygons)
	{
		//fprintf(stdout,"dxConvex dGeomSetConvex\n");
		//dUASSERT (g && g.type == dConvexClass,"argument not a convex shape");
		//dxConvex *s = (dxConvex*) g;
		//this.planes = planes;
//		this.planesV = planesV;
//		this.planesD = planesD;
		planesV = new DVector3[planecount];
		planesD = new double[planecount];
		for (int i = 0; i < planecount; i++) {
			planesV[i] = new DVector3(planes[i*4], planes[i*4+1], planes[i*4+2]);
			planesD[i] = planes[i*4+3];
		}
		this.planecount = planecount;
		this.points = points;
		this.pointcount = pointcount;
		this.polygons=polygons;
	}


	//****************************************************************************
	// Helper Inlines
	//

	/** 
	 * Returns Whether or not the segment ab intersects plane p.
	 * @param a origin of the segment
	 * @param b segment destination
	 * @param pV plane to test for intersection - vector
	 * @param pD plane to test for intersection - depth
	 * @param t returns the time "t" in the segment ray that gives us the intersecting
	 * point
	 * @param q returns the intersection point
	 * @return true if there is an intersection, otherwise false.
	 */
	//private boolean IntersectSegmentPlane(dVector3 a, 
	//		   dVector3 b, 
	//		   dVector4 p, 
	//		   dReal &t, 
	//		   dVector3 q)
	private static boolean IntersectSegmentPlane(DVector3 a, 
			DVector3 b, 
			DVector3 pV, double pD, 
			RefDouble t, 
			DVector3 q)
	{
		// Compute the t value for the directed line ab intersecting the plane
		DVector3 ab = new DVector3();
		//  ab[0]= b[0] - a[0];
		//  ab[1]= b[1] - a[1];
		//  ab[2]= b[2] - a[2];
		ab.eqDiff(b, a);

//		t.set( (p.v[3] - dDOT(pV,a)) / dDOT(pV,ab)  );
		t.set( (pD - dCalcVectorDot3(pV,a)) / dCalcVectorDot3(pV,ab)  );

		// If t in [0..1] compute and return intersection point
		if (t.get() >= 0.0 && t.get() <= 1.0) 
		{
			//      q[0] = a[0] + t * ab[0];
			//      q[1] = a[1] + t * ab[1];
			//      q[2] = a[2] + t * ab[2];
			q.eqSum(a, ab, t.get());
			return true;
		}
		// Else no intersection
		return false;
	}

	//	/**
	//	 * Returns the Closest Point in Ray 1 to Ray 2.
	//	 * @param Origin1 The origin of Ray 1
	//	 * @param Direction1 The direction of Ray 1
	//	 * @param Origin1 The origin of Ray 2
	//	 * @param Direction1 The direction of Ray 3
	//	 * @param t the time "t" in Ray 1 that gives us the closest point
	//	 * (closest_point=Origin1+(Direction1*t).
	//	 * @return true if there is a closest point, false if the rays are paralell.
	//	 */
	//inline bool ClosestPointInRay(final dVector3 Origin1,
	//	      final dVector3 Direction1,
	//	      final dVector3 Origin2,
	//	      final dVector3 Direction2,
	//	      dReal& t)
//	private boolean ClosestPointInRay(final DVector3 Origin1,
//			final DVector3 Direction1,
//			final DVector3 Origin2,
//			final DVector3 Direction2,
//			RefDouble t)
//	{
//		//  dVector3 w = {Origin1[0]-Origin2[0],
//		//				Origin1[1]-Origin2[1],
//		//				Origin1[2]-Origin2[2]};
//		DVector3 w = new DVector3();
//		w.eqDiff(Origin1, Origin2);
//		double a = dCalcVectorDot3(Direction1 , Direction1);
//		double b = dCalcVectorDot3(Direction1 , Direction2);
//		double c = dCalcVectorDot3(Direction2 , Direction2);
//		double d = dCalcVectorDot3(Direction1 , w);
//		double e = dCalcVectorDot3(Direction2 , w);
//		double denominator = (a*c)-(b*b);
//		if(denominator==0.0f)
//		{
//			return false;
//		}
//		t.set( ((a*e)-(b*d))/denominator );
//		return true;
//	}

	/**
	 * Returns the Closest Points from Segment 1 to Segment 2.
	 * <p>NOTE: Adapted from Christer Ericson's Real Time Collision Detection Book.
	 * @param p1 start of segment 1
	 * @param q1 end of segment 1
	 * @param p2 start of segment 2
	 * @param q2 end of segment 2
	 */
//	private float ClosestPointBetweenSegments(dVector3& p1,
//            dVector3& q1,
//            dVector3& p2,
//            dVector3& q2,
//            dVector3& c1,
//            dVector3& c2)
	private static double ClosestPointBetweenSegments(DVector3 p1, DVector3 q1,
            DVector3 p2, DVector3 q2, DVector3 c1, DVector3 c2)
	{
	    // s & t were originaly part of the output args, but since
	    // we don't really need them, we'll just declare them in here
	    double s;
	    double t; // TZ: the time "t" in Ray 1 that gives us the closest point (closest_point=Origin1+(Direction1*t) (?)
	    DVector3 d1 = q1.reSub(p1);//{q1[0] - p1[0],
	                   //q1[1] - p1[1],
	                   //q1[2] - p1[2]};
	    DVector3 d2 = q2.reSub( p2 ); //{q2[0] - p2[0],
	                   ///q2[1] - p2[1],
	                   //q2[2] - p2[2]};
	    DVector3 r  = p1.reSub( p2 ); //{p1[0] - p2[0],
	                   //p1[1] - p2[1],
	                   //p1[2] - p2[2]};
	    double a = d1.lengthSquared();//dDOT(d1, d1);
	    double e = d2.lengthSquared();//dDOT(d2, d2);
	    double f = d2.dot( r );//dDOT(d2, r);
	    // Check if either or both segments degenerate into points
	    if (a <= dEpsilon && e <= dEpsilon)
	    {
	        // Both segments degenerate into points
	        s = t = 0.0f;
	        c1.set( p1 );//dVector3Copy(p1,c1);
	        c2.set( p2 );//dVector3Copy(p2,c2);
//	        return (c1[0] - c2[0])*(c1[0] - c2[0])+
//	               (c1[1] - c2[1])*(c1[1] - c2[1])+
//	               (c1[2] - c2[2])*(c1[2] - c2[2]);
	        return c1.reSub(c2).lengthSquared();
	    }
	    if (a <= dEpsilon)
	    {
	        // First segment degenerates into a point
	        s = 0.0f;
	        t = f / e; // s = 0 => t = (b*s + f) / e = f / e
	        t = dxClamp(t, 0.0f, 1.0f);
	    }
	    else
	    {
	        double c = dCalcVectorDot3(d1, r);
	        if (e <= dEpsilon)
	        {
	            // Second segment degenerates into a point
	            t = 0.0f;
	            s = dxClamp(-c / a, 0.0f, 1.0f); // t = 0 => s = (b*t - c) / a = -c / a
	        }
	        else
	        {
	            // The general non degenerate case starts here
	            double b = dCalcVectorDot3(d1, d2);
	            double denom = a*e-b*b; // Always nonnegative

	            // If segments not parallel, compute closest point on L1 to L2, and
	            // clamp to segment S1. Else pick arbitrary s (here 0)
	            if (denom != 0.0f)
	            {
	                s = dxClamp((b*f - c*e) / denom, 0.0f, 1.0f);
	            }
	            else s = 0.0f;
//	if (false) {//#if 0
//	            // Compute point on L2 closest to S1(s) using
//	            // t = Dot((P1+D1*s)-P2,D2) / Dot(D2,D2) = (b*s + f) / e
//	            t = (b*s + f) / e;
//
//	            // If t in [0,1] done. Else clamp t, recompute s for the new value
//	            // of t using s = Dot((P2+D2*t)-P1,D1) / Dot(D1,D1)= (t*b - c) / a
//	            // and clamp s to [0, 1]
//	            if (t < 0.0f) {
//	                t = 0.0f;
//	                s = Clamp(-c / a, 0.0f, 1.0f);
//	            } else if (t > 1.0f) {
//	                t = 1.0f;
//	                s = Clamp((b - c) / a, 0.0f, 1.0f);
//	            }
//	} else {//#else
	            double tnom = b*s + f;
	            if (tnom < 0.0f)
	            {
	                t = 0.0f;
	                s = dxClamp(-c / a, 0.0f, 1.0f);
	            }
	            else if (tnom > e)
	            {
	                t = 1.0f;
	                s = dxClamp((b - c) / a, 0.0f, 1.0f);
	            }
	            else
	            {
	                t = tnom / e;
	            }
//	}//#endif
	        }
	    }

//	    c1[0] = p1[0] + d1[0] * s;
//	    c1[1] = p1[1] + d1[1] * s;
//	    c1[2] = p1[2] + d1[2] * s;
	    c1.eqSum( p1, d1, s );
//	    c2[0] = p2[0] + d2[0] * t;
//	    c2[1] = p2[1] + d2[1] * t;
//	    c2[2] = p2[2] + d2[2] * t;
	    c2.eqSum( p2, d2, t );
//	    return (c1[0] - c2[0])*(c1[0] - c2[0])+
//	           (c1[1] - c2[1])*(c1[1] - c2[1])+
//	           (c1[2] - c2[2])*(c1[2] - c2[2]);
	    return (c1.get0() - c2.get0())*(c1.get0() - c2.get0())+
        (c1.get1() - c2.get1())*(c1.get1() - c2.get1())+
        (c1.get2() - c2.get2())*(c1.get2() - c2.get2());
	}

//	#if 0
//	float tnom = b*s + f;
//	if (tnom < 0.0f) {
//	    t = 0.0f;
//	    s = Clamp(-c / a, 0.0f, 1.0f);
//	} else if (tnom > e) {
//	    t = 1.0f;
//	    s = Clamp((b - c) / a, 0.0f, 1.0f);
//	} else {
//	    t = tnom / e;
//	}
//	#endif

//	/** 
//	 * Returns the Ray on which 2 planes intersect if they do.
//	 * @param p1 Plane 1
//	 * @param p2 Plane 2
//	 * @param p Contains the origin of the ray upon returning if planes intersect
//	 * @param d Contains the direction of the ray upon returning if planes intersect
//	 * @return true if the planes intersect, false if paralell.
//	 */
//	private boolean IntersectPlanes(final DVector3 p1, double p13, final DVector3 p2, double p23,DVector3 p, DVector3 d)
//	{
//		// Compute direction of intersection line
//	    dCalcVectorCross3(d,p1,p2);  
//		// If d is (near) zero, the planes are parallel (and separated)
//		// or coincident, so they're not considered intersecting
//		double denom = d.dot(d);
//		if (denom < dEpsilon) return false;
//		DVector3 n=new DVector3();
//		//  n[0]=p1[3]*p2[0] - p2[3]*p1[0];
//		//  n[1]=p1[3]*p2[1] - p2[3]*p1[1];
//		//  n[2]=p1[3]*p2[2] - p2[3]*p1[2];
//		n.eqSum(p2, p13, p1, -p23);
//		// Compute point on intersection line
//		dCalcVectorCross3(p,n,d);
//		//  p[0]/=denom;
//		//  p[1]/=denom;
//		//  p[2]/=denom;
//		p.scale(1/denom);
//		return true;
//	}


	//#if 0
	///*! \brief Finds out if a point lies inside a convex
	//  \param p Point to test
	//  \param convex a pointer to convex to test against
	//  \return true if the point lies inside the convex, false if not.
	//*/
	//inline bool IsPointInConvex(dVector3 p,
	//			    dxConvex *convex)
	//{
	//  dVector3 lp,tmp;
	//  // move point into convex space to avoid plane local to world calculations
	//  tmp[0] = p[0] - convex->final_posr->pos[0];
	//  tmp[1] = p[1] - convex->final_posr->pos[1];
	//  tmp[2] = p[2] - convex->final_posr->pos[2];
	//  dMULTIPLY1_331 (lp,convex->final_posr->R,tmp);
	//  for(unsigned int i=0;i<convex->planecount;++i)
	//  {
	//    if((
	//	  ((convex->planes+(i*4))[0]*lp[0])+
	//	  ((convex->planes+(i*4))[1]*lp[1])+
	//	  ((convex->planes+(i*4))[2]*lp[2])+
	//	  -(convex->planes+(i*4))[3]
	//	  )>0)
	//	  {
	//	    return false;
	//	  }
	//  }
	//  return true;
	//}
	//#endif

	/** 
	 * Finds out if a point lies inside a 2D polygon.
	 * @param p Point to test
	 * @param polygonA a pointer to the start of the convex polygon index buffer
	 * @param out the closest point in the polygon if the point is not inside
	 * @return true if the point lies inside of the polygon, false if not.
	 */
	//inline bool IsPointInPolygon(dVector3 p,
	//			     unsigned int *polygon,
	//			     dxConvex *convex,
	//			     dVector3 out)
	private static boolean IsPointInPolygon(DVector3C p,
			int[] polygonA, int polyPos,
			DVector3C plane,
			DxConvex convex,
			DVector3 out)
	{
		// p is the point we want to check,
		// polygon is a pointer to the polygon we
		// are checking against, remember it goes
		// number of vertices then that many indexes
		// out returns the closest point on the border of the
		// polygon if the point is not inside it.
		//size_t 
		DVector3 a = new DVector3();
		DVector3 b = new DVector3();
		DVector3 ab = new DVector3();
		DVector3 ap = new DVector3();
		DVector3 v = new DVector3();

		int pointcount=polygonA[0+polyPos];
		Common.dIASSERT(pointcount != 0);
		polyPos++;//polygon++; // skip past pointcount

		dMultiply0_331 (b, convex.final_posr().R(),
	            convex.points, (polygonA[polyPos + pointcount-1]*3));
	    //    b[0]=convex->final_posr->pos[0]+b[0];
	    //    b[1]=convex->final_posr->pos[1]+b[1];
	    //    b[2]=convex->final_posr->pos[2]+b[2];
		b.eqSum(convex.final_posr().pos(), b);

		for(int i=0;i<pointcount;++i)
		{
			a.set(b);

			dMultiply0_331 (b,convex.final_posr().R(),
					convex.points,(polygonA[polyPos+i]*3));
			//      b[0]=convex.final_posr.pos[0]+b[0];
			//      b[1]=convex.final_posr.pos[1]+b[1];
			//      b[2]=convex.final_posr.pos[2]+b[2];
			b.eqSum(convex.final_posr().pos(), b);

			//      ab[0] = b[0] - a[0];
			//      ab[1] = b[1] - a[1];
			//      ab[2] = b[2] - a[2];
			ab.eqDiff(b, a);
			//      ap[0] = p[0] - a[0];
			//      ap[1] = p[1] - a[1];
			//      ap[2] = p[2] - a[2];
			ap.eqDiff(p, a);
			
			dCalcVectorCross3(v, ab, plane);

			if (dCalcVectorDot3(ap, v) > 0.0)
			{
				double ab_m2 = dCalcVectorDot3(ab, ab);
				double s = ab_m2 != 0.0 ? dCalcVectorDot3(ab, ap) / ab_m2 : 0.0;

				if (s <= 0.0)
				{
					//	  out[0]=a[0];
					//	  out[1]=a[1];
					//	  out[2]=a[2];
				out.set(a);
			}
	            else if (s >= 1.0) 
	            {
	                //out[0] = b[0];
	                //out[1] = b[1];
	                //out[2] = b[2];
	            	out.set(b);
	            }
	            else
	            {
	                //out[0] = a[0] + ab[0] * s;
	                //out[1] = a[1] + ab[1] * s;
	                //out[2] = a[2] + ab[2] * s;
	            	out.eqSum(a, ab, s);
	            }

	            return false;
	        }
	    }

	    return true;
	}

	static class CollideConvexPlane implements DColliderFn {
		//int dCollideConvexPlane (dxGeom *o1, dxGeom *o2, int flags,
		//		 dContactGeom *contact, int skip)
		int dCollideConvexPlane (DxConvex Convex, DxPlane Plane, int flags,
				DContactGeomBuffer contactBuf, int skip)
		{
			dIASSERT (skip >= 1);//(int)sizeof(dContactGeom));
			//	dIASSERT (o1->type == dConvexClass);
			//	dIASSERT (o2->type == dPlaneClass);
			dIASSERT ((flags & NUMC_MASK) >= 1);
	
			//	dxConvex *Convex = (dxConvex*) o1;
			//	dxPlane *Plane = (dxPlane*) o2;
			//unsigned 
			int contacts=0;
			//unsigned 
			int maxc = flags & NUMC_MASK;
			DVector3 v2 = new DVector3();
	
			//#define LTEQ_ZERO	0x10000000
			//#define GTEQ_ZERO	0x20000000
			//#define BOTH_SIGNS	(LTEQ_ZERO | GTEQ_ZERO)
			final int LTEQ_ZERO = 0x10000000;
			final int GTEQ_ZERO = 0x20000000;
			final int BOTH_SIGNS = (LTEQ_ZERO | GTEQ_ZERO);
			//dIASSERT((BOTH_SIGNS & NUMC_MASK) == 0); // used in conditional operator later
	
			//unsigned 
			int totalsign = 0;
			for(int i=0;i<Convex.pointcount;++i)
			{
				dMultiply0_331 (v2,Convex.final_posr().R(),Convex.points,i*3);//[(i*3)]);
				v2.add(Convex.final_posr().pos());//dVector3Add(Convex._final_posr.pos, v2, v2);
	
				//unsigned 
				int distance2sign = GTEQ_ZERO;
				//double distance2 = dVector3Dot(Plane._p, v2) - Plane._p[3]; // Ax + By + Cz - D
//				double distance2 = Plane._p[0]*v2.get0() + 
//				Plane._p[1]*v2.get1() + Plane._p[2]*v2.get2() - Plane._p[3]; // Ax + By + Cz - D
				double distance2 = Plane.getNormal().dot( v2 ) - Plane.getDepth(); // Ax + By + Cz - D
				if((distance2 <= (0.0)))
				{
					distance2sign = distance2 != (0.0) ? LTEQ_ZERO : BOTH_SIGNS;
	
					if (contacts != maxc)
					{
						//dContactGeom *target = SAFECONTACT(flags, contactBuf, contacts, skip);
						DContactGeom target = contactBuf.getSafe(flags, contacts);
						target.normal.set(Plane.getNormal());//dVector3Copy(Plane.p, target.normal);
						target.pos.set(v2);//dVector3Copy(v2, target.pos);
						target.depth = -distance2;
						target.g1 = Convex;
						target.g2 = Plane;
						target.side1 = -1; // TODO: set plane index?
						target.side2 = -1;
						contacts++;
					}
				}
	
				// Take new sign into account
				totalsign |= distance2sign;
				// Check if contacts are full and both signs have been already found
				if (((contacts ^ maxc) | totalsign) == BOTH_SIGNS) // harder to comprehend but requires one register less
				{
					break; // Nothing can be changed any more
				}
			}
			if (totalsign == BOTH_SIGNS) return contacts;
			return 0;
			//#undef BOTH_SIGNS
			//#undef GTEQ_ZERO
			//#undef LTEQ_ZERO
		}

		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return dCollideConvexPlane((DxConvex)o1, (DxPlane)o2, flags, contacts, 1);
		}
	}

	static class CollideSphereConvex implements DColliderFn {
		//int dCollideSphereConvex (dxGeom *o1, dxGeom *o2, int flags,
		//			  dContactGeom *contact, int skip)
		int dCollideSphereConvex (DxSphere sphere, DxConvex convex, int flags,
				DContactGeomBuffer contacts, int skip)
		{
			dIASSERT (skip >= 1);//(int)sizeof(dContactGeom));
			//  dIASSERT (o1.type == dSphereClass);
			//  dIASSERT (o2.type == dConvexClass);
			dIASSERT ((flags & NUMC_MASK) >= 1);
	
			//  dxSphere *Sphere = (dxSphere*) o1;
			//  dxConvex *Convex = (dxConvex*) o2;
			double dist,closestdist=dInfinity;
			//dVector4 plane;
			DVector3 planeV = new DVector3();
			double planeD;
			// dVector3 contactpoint;
			DVector3 offsetpos=new DVector3(),out=new DVector3(),temp=new DVector3();
			//unsigned int *pPoly=Convex.polygons;
			int[] pPolyV=convex.polygons;
			int pPolyPos=0;
			int closestplane=-1;
			boolean sphereinside=true;
			DContactGeom contact = contacts.get(0);
			/* 
	     Do a good old sphere vs plane check first,
	     if a collision is found then check if the contact point
	     is within the polygon
			 */
			// offset the sphere final_posr->position into the convex space
	//		offsetpos[0]=sphere.final_posr.pos[0]-convex.final_posr.pos[0];
	//		offsetpos[1]=sphere.final_posr.pos[1]-convex.final_posr.pos[1];
	//		offsetpos[2]=sphere.final_posr.pos[2]-convex.final_posr.pos[2];
			offsetpos.eqDiff(sphere.final_posr().pos(), convex.final_posr().pos());
			for(int i=0;i<convex.planecount;++i)
			{
				// apply rotation to the plane
				dMultiply0_331(planeV,convex.final_posr().R(),convex.planesV[i]);//convex.planes[(i*4)]);
				planeD=convex.planesD[i];//(convex.planes[(i*4)])[3];
				// Get the distance from the sphere origin to the plane
				dist = planeV.dot(offsetpos) - planeD; // Ax + By + Cz - D
				if(dist>0)
				{
					// if we get here, we know the center of the sphere is
					// outside of the convex hull.
					if(dist<sphere.getRadius())
					{
						// if we get here we know the sphere surface penetrates
						// the plane
						if(IsPointInPolygon(sphere.final_posr().pos(),pPolyV, pPolyPos,planeV,convex,out))
						{
							// finally if we get here we know that the
							// sphere is directly touching the inside of the polyhedron
	//						contact.normal[0] = plane[0];
	//						contact.normal[1] = plane[1];
	//						contact.normal[2] = plane[2];
							contact.normal.set(planeV);
	//						contact.pos[0] = sphere._final_posr.pos[0]+
	//						(-contact.normal[0]*sphere.radius);
	//						contact.pos[1] = sphere._final_posr.pos[1]+
	//						(-contact.normal[1]*sphere.radius);
	//						contact.pos[2] = sphere._final_posr.pos[2]+
	//						(-contact.normal[2]*sphere.radius);
							contact.pos.eqSum(sphere.final_posr().pos(),
									contact.normal, -sphere.getRadius());
							contact.depth = sphere.getRadius()-dist;
							contact.g1 = sphere;
							contact.g2 = convex;
							contact.side1 = -1;
							contact.side2 = -1; // TODO: set plane index?
							return 1;
						}
						else
						{
							// the sphere may not be directly touching
							// the polyhedron, but it may be touching
							// a point or an edge, if the distance between
							// the closest point on the poly (out) and the 
							// center of the sphere is less than the sphere 
							// radius we have a hit.
	//						temp[0] = (sphere._final_posr.pos[0]-out[0]);
	//						temp[1] = (sphere._final_posr.pos[1]-out[1]);
	//						temp[2] = (sphere._final_posr.pos[2]-out[2]);
							temp.eqDiff(sphere.final_posr().pos(), out);
							dist=temp.lengthSquared();//(temp[0]*temp[0])+(temp[1]*temp[1])+(temp[2]*temp[2]);
							// avoid the sqrt unless really necesary
							if(dist<(sphere.getRadius()*sphere.getRadius()))
							{
								// We got an indirect hit
								dist=dSqrt(dist);
	//							contact.normal[0] = temp[0]/dist;
	//							contact.normal[1] = temp[1]/dist;
	//							contact.normal[2] = temp[2]/dist;
								contact.normal.set(temp).scale(1./dist);
	//							contact.pos[0] = sphere.final_posr.pos[0]+
	//							(-contact.normal[0]*sphere.radius);
	//							contact.pos[1] = sphere.final_posr.pos[1]+
	//							(-contact.normal[1]*sphere.radius);
	//							contact.pos[2] = sphere.final_posr.pos[2]+
	//							(-contact.normal[2]*sphere.radius);
								contact.pos.eqSum(sphere.final_posr().pos(),
										contact.normal, -sphere.getRadius());
								contact.depth = sphere.getRadius()-dist;
								contact.g1 = sphere;
								contact.g2 = convex;
								contact.side1 = -1;
								contact.side2 = -1; // TODO: set plane index?
								return 1;
							}
						}
					}
					sphereinside=false;
				}
				if(sphereinside)
				{
					if(closestdist>dFabs(dist))
					{
						closestdist=dFabs(dist);
						closestplane=i;
					}
				}
				//pPoly+=pPoly[0]+1;
				pPolyPos += pPolyV[pPolyPos]+1;
			}
			if(sphereinside)
			{
				// if the center of the sphere is inside 
				// the Convex, we need to pop it out
				dMultiply0_331(contact.normal,
						convex.final_posr().R(),
						//convex.planes[(closestplane*4)]);
						convex.planesV[closestplane]);
	//			contact.pos[0] = sphere.final_posr.pos[0];
	//			contact.pos[1] = sphere.final_posr.pos[1];
	//			contact.pos[2] = sphere.final_posr.pos[2];
				contact.pos.set(sphere.final_posr().pos());
				contact.depth = closestdist+sphere.getRadius();
				contact.g1 = sphere;
				contact.g2 = convex;
				contact.side1 = -1;
				contact.side2 = -1; // TODO: set plane index?
				return 1;
			}
			return 0;
		}

		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return dCollideSphereConvex((DxSphere)o1, (DxConvex)o2, flags, contacts, 1);
		}
	}

	static class CollideConvexBox implements DColliderFn {
		//int dCollideConvexBox (dxGeom *o1, dxGeom *o2, int flags,
		//		       dContactGeom *contact, int skip)
		int dCollideConvexBox (DxConvex Convex, DxBox box, int flags,
				DContactGeomBuffer contacts, int skip)
		{
			dIASSERT (skip >= 1);//(int)sizeof(dContactGeom));
			//  dIASSERT (o1.type == dConvexClass);
			//  dIASSERT (o2.type == dBoxClass);
			dIASSERT ((flags & NUMC_MASK) >= 1);
	
			//dxConvex *Convex = (dxConvex*) o1;
			//dxBox *Box = (dxBox*) o2;
	
			return 0;
		}

		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return dCollideConvexBox((DxConvex)o1, (DxBox)o2, flags, contacts, 1);
		}
	}

	static class CollideConvexCapsule implements DColliderFn {
		//int dCollideConvexCapsule (dxGeom *o1, dxGeom *o2,
		//			     int flags, dContactGeom *contact, int skip)
		int dCollideConvexCapsule (DxConvex Convex, DxCapsule Capsule,
				int flags, DContactGeomBuffer contacts, int skip)
		{
			dIASSERT (skip >= 1);//(int)sizeof(dContactGeom));
			//  dIASSERT (o1.type == dConvexClass);
			//  dIASSERT (o2.type == dCapsuleClass);
			dIASSERT ((flags & NUMC_MASK) >= 1);

			//dxConvex *Convex = (dxConvex*) o1;
			//dxCapsule *Capsule = (dxCapsule*) o2;

			return 0;
		}

		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return dCollideConvexCapsule((DxConvex)o1, (DxCapsule)o2, flags, contacts, 1);
		}
	}

	//inline void ComputeInterval(dxConvex& cvx,dVector4 axis,dReal& min,dReal& max)
	private static void ComputeInterval(DxConvex cvx,DVector3 axis, double axisD, RefDouble min, RefDouble max)
	{
		/* TODO: Use Support points here */
		DVector3 point=new DVector3();
		double value;
		//fprintf(stdout,"Compute Interval Axis %f,%f,%f\n",axis[0],axis[1],axis[2]);
		dMultiply0_331(point, cvx.final_posr().R(), cvx.points,0);
		//fprintf(stdout,"initial point %f,%f,%f\n",point[0],point[1],point[2]);
		//    point[0]+=cvx.final_posr.pos[0];
		//    point[1]+=cvx.final_posr.pos[1];
		//    point[2]+=cvx.final_posr.pos[2];
		point.add(cvx.final_posr().pos());
		//    max = min = dDOT(point,axis)-axis[3];//(*)
		min.set( dCalcVectorDot3(point,axis)-axisD );//[3];//(*)
		max.set(min.get());// = min;//(*)
		for (int i = 1; i < cvx.pointcount; ++i) 
		{
			dMultiply0_331(point,cvx.final_posr().R(),cvx.points,(i*3));
//			point[0]+=cvx.final_posr.pos[0];
//			point[1]+=cvx.final_posr.pos[1];
//			point[2]+=cvx.final_posr.pos[2];
			point.add( cvx.final_posr().pos() );
			value=dCalcVectorDot3(point,axis)-axisD;//[3];//(*)
			if(value<min.get())
			{
				min.set(value);
			}
			else if(value>max.get())
			{
				max.set(value);
			}
		}
		// *: usually using the distance part of the plane (axis) is
		// not necesary, however, here we need it here in order to know
		// which face to pick when there are 2 parallel sides.
	}

	//bool CheckEdgeIntersection(dxConvex& cvx1,dxConvex& cvx2, int flags,int& curc,
	//			   dContactGeom *contact, int skip)
	boolean CheckEdgeIntersection(DxConvex cvx1,DxConvex cvx2, int flags, RefInt curc,
			DContactGeomBuffer contacts, int skip)
	{
		int maxc = flags & NUMC_MASK;
		dIASSERT(maxc != 0);
		DVector3 e1=new DVector3(),e2=new DVector3(),q=new DVector3();
		//dVector4 plane,depthplane;
		DVector3 planeV=new DVector3(),depthplaneV=new DVector3();
		double planeD,depthplaneD;
		RefDouble t = new RefDouble();  //TZ TODO Why?
		for(int i = 0;i<cvx1.edgecount;++i)
		{
			// Rotate
			dMultiply0_331(e1,cvx1.final_posr().R(),cvx1.points,(cvx1.edges[i].first*3));
			// translate
//			e1[0]+=cvx1.final_posr.pos[0];
//			e1[1]+=cvx1.final_posr.pos[1];
//			e1[2]+=cvx1.final_posr.pos[2];
			e1.add( cvx1.final_posr().pos() );
			// Rotate
			dMultiply0_331(e2,cvx1.final_posr().R(),cvx1.points,(cvx1.edges[i].second*3));
			// translate
//			e2[0]+=cvx1.final_posr.pos[0];
//			e2[1]+=cvx1.final_posr.pos[1];
//			e2[2]+=cvx1.final_posr.pos[2];
			e2.add( cvx1.final_posr().pos() );
			//unsigned int* pPoly=cvx2.polygons;
			int[] pPolyV=cvx2.polygons;
			int pPolyPos=0;
			for(int j=0;j<cvx2.planecount;++j)
			{
				// Rotate
				dMultiply0_331(planeV,cvx2.final_posr().R(),cvx2.planesV[i]);//+(j*4));
				dNormalize3(planeV);
				// Translate
				planeD = //plane[3]=
					cvx2.planesD[j] + //(cvx2.planes[(j*4)+3])+
//					((plane[0] * cvx2._final_posr.pos[0]) + 
//							(plane[1] * cvx2._final_posr.pos[1]) + 
//							(plane[2] * cvx2._final_posr.pos[2]));
					planeV.dot(cvx2.final_posr().pos());
				//dContactGeom *target = SAFECONTACT(flags, contact, curc, skip);
				DContactGeom target = contacts.getSafe(flags, curc.get());
				target.g1=cvx1;//&cvx1; // g1 is the one pushed
				target.g2=cvx2;//&cvx2;
				if(IntersectSegmentPlane(e1,e2,planeV, planeD,t,target.pos))
				{
					if(IsPointInPolygon(target.pos,pPolyV, pPolyPos,planeV,cvx2,q))
					{
						target.depth = dInfinity;
						for(int k=0;k<cvx2.planecount;++k)
						{
							if(k==j) continue; // we're already at 0 depth on this plane
							// Rotate
							dMultiply0_331(depthplaneV,cvx2.final_posr().R(),cvx2.planesV[k]);//*4));
							dNormalize3(depthplaneV);
							// Translate
							depthplaneD = //depthplane[3]=
								cvx2.planesD[k] + //(cvx2.planes[(k*4)+3])+
//								((plane[0] * cvx2.final_posr.pos[0]) + 
//										(plane[1] * cvx2.final_posr.pos[1]) + 
//										(plane[2] * cvx2.final_posr.pos[2]));
								planeV.dot(cvx2.final_posr().pos());
							//double depth = (dVector3Dot(depthplane, target.pos) - depthplane[3]); // Ax + By + Cz - D
							double depth = depthplaneV.dot(target.pos) - depthplaneD; // Ax + By + Cz - D
							if((fabs(depth)<fabs(target.depth))&&((depth<-dEpsilon)||(depth>dEpsilon)))
							{
								target.depth=depth;
								target.normal.set(depthplaneV);//dVector3Copy(depthplane,target.normal);
							}
						}
						curc.inc();//++curc;
						if(curc.get()==maxc)
							return true;
					}
				}
				//pPoly+=pPoly[0]+1;
				pPolyPos += pPolyV[pPolyPos]+1;
			}
		}
		return false;
	}

	/*
Helper struct
	 */

	private static class ConvexConvexSATOutput
	{
		double min_depth;
		int depth_type;
		DVector3 dist = new DVector3(); // distance from center to center, from cvx1 to cvx2
		//int side_index;
		//dxConvex* g1;
		//dxConvex* g2;
		 // e1a to e1b = edge in cvx1,e2a to e2b = edge in cvx2.
		DVector3 e1a = new DVector3(), e1b = new DVector3();
		DVector3 e2a = new DVector3(), e2b = new DVector3();
	}

	/** 
	 * Does an axis separation test using cvx1 planes on cvx1 and cvx2, 
	 * returns true for a collision false for no collision.
	 * @param cvx1 [IN] First Convex object, its planes are used to do the tests
	 * @param cvx2 [IN] Second Convex object
	 */
	//inline bool CheckSATConvexFaces(dxConvex& cvx1,
	//				dxConvex& cvx2,
	//				ConvexConvexSATOutput& ccso)
	private static boolean CheckSATConvexFaces(DxConvex cvx1,
			DxConvex cvx2,
			ConvexConvexSATOutput ccso)
	{
		//double min,max,min1,max1,min2,max2,depth;
		double min,max,depth;
		RefDouble min1=new RefDouble(), max1=new RefDouble(); 
		RefDouble min2=new RefDouble(), max2=new RefDouble();
		//dVector4 plane;
		DVector3 planeV = new DVector3();
		double planeD;
		for(int i=0;i<cvx1.planecount;++i)
		{
			// -- Apply Transforms --
			// Rotate
			dMultiply0_331(planeV, cvx1.final_posr().R(), cvx1.planesV[i]);
			dNormalize3(planeV);
			// Translate
			planeD = //plane[3]=
				cvx1.planesD[i]+//(cvx1.planes[(i*4)+3])+
//				((plane[0] * cvx1.final_posr.pos[0]) + 
//						(plane[1] * cvx1.final_posr.pos[1])  + 
//						(plane[2] * cvx1.final_posr.pos[2]));
				planeV.dot(cvx1.final_posr().pos());
			ComputeInterval(cvx1,planeV,planeD,min1,max1);
			ComputeInterval(cvx2,planeV,planeD,min2,max2);
			if(max2.get()<min1.get() || max1.get()<min2.get()) return false;
			min = dMAX(min1.get(), min2.get());
			max = dMIN(max1.get(), max2.get());
			depth = max-min;
			/* 
        Take only into account the faces that penetrate cvx1 to determine
        minimum depth
        ((max2*min2)<=0) = different sign, or one is zero and thus
        cvx2 barelly touches cvx1
			 */
			if (((max2.get()*min2.get())<=0) && (dFabs(depth)<dFabs(ccso.min_depth)))
			{
				// Flip plane because the contact normal must point INTO g1,
				// plus the integrator seems to like positive depths better than negative ones
				ccso.min_depth=-depth;
				ccso.depth_type = 1; // 1 = face-something
			}
		}
		return true;
	}


	/** 
	 * Does an axis separation test using cvx1 and cvx2 edges, 
	 * returns true for a collision false for no collision.
	 * @param cvx1 [IN] First Convex object
	 * @param cvx2 [IN] Second Convex object
	 */
	//inline bool CheckSATConvexEdges(dxConvex& cvx1,
	//				dxConvex& cvx2,
	//				ConvexConvexSATOutput& ccso)
	private static boolean CheckSATConvexEdges(DxConvex cvx1,
			DxConvex cvx2,
			ConvexConvexSATOutput ccso)
	{
		// Test cross products of pairs of edges
		double depth,min,max;
		RefDouble min1 = new RefDouble(), max1 = new RefDouble();
		RefDouble min2 = new RefDouble(), max2 = new RefDouble();
		//dVector4 plane;
		DVector3 planeV = new DVector3();
		double planeD;
		DVector3 e1 = new DVector3(),e2 = new DVector3(),e1a = new DVector3();
		DVector3 e1b = new DVector3(),e2a = new DVector3(),e2b = new DVector3();
		DVector3 dist = new DVector3(ccso.dist);
		//dVector3Copy(ccso.dist,dist);
		int s1 = cvx1.SupportIndex(dist);
		// invert direction
		dist.scale(-1);//dVector3Inv(dist);  
		int s2 = cvx2.SupportIndex(dist);
		for(int i = 0;i<cvx1.edgecount;++i)
		{
			// Skip edge if it doesn't contain the extremal vertex
			if((cvx1.edges[i].first!=s1)&&(cvx1.edges[i].second!=s1)) continue;
			// we only need to apply rotation here
			dMultiply0_331(e1a,cvx1.final_posr().R(),cvx1.points,(cvx1.edges[i].first*3));
			dMultiply0_331(e1b,cvx1.final_posr().R(),cvx1.points,(cvx1.edges[i].second*3));
//			e1[0]=e1b[0]-e1a[0];
//			e1[1]=e1b[1]-e1a[1];
//			e1[2]=e1b[2]-e1a[2];
			e1.eqDiff(e1b, e1a);
			for(int j = 0;j<cvx2.edgecount;++j)
			{
				// Skip edge if it doesn't contain the extremal vertex
				if((cvx2.edges[j].first!=s2)&&(cvx2.edges[j].second!=s2)) continue;
				// we only need to apply rotation here
				dMultiply0_331 (e2a,cvx2.final_posr().R(),cvx2.points,(cvx2.edges[j].first*3));
				dMultiply0_331 (e2b,cvx2.final_posr().R(),cvx2.points,(cvx2.edges[j].second*3));
//				e2[0]=e2b[0]-e2a[0];
//				e2[1]=e2b[1]-e2a[1];
//				e2[2]=e2b[2]-e2a[2];
				e2.eqDiff(e2b, e2a);
				dCalcVectorCross3(planeV,e1,e2);
				if(planeV.dot(planeV) < dEpsilon) /* edges are parallel */ continue;
				dNormalize3(planeV);
				planeD = 0;//plane[3]=0;
				ComputeInterval(cvx1,planeV,planeD,min1,max1);
				ComputeInterval(cvx2,planeV,planeD,min2,max2);
				if(max2.get() < min1.get() || max1.get() < min2.get()) return false;
				min = dMAX(min1.get(), min2.get());
				max = dMIN(max1.get(), max2.get());
				depth = max-min;
				if (((dFabs(depth)+dEpsilon)<dFabs(ccso.min_depth)))
				{
					ccso.min_depth=depth;
					ccso.depth_type = 2; // 2 = edge-edge
					// use cached values, add position
					ccso.e1a.set(e1a);//(dVector3Copy(e1a,ccso.e1a);
					ccso.e1b.set(e1b);//dVector3Copy(e1b,ccso.e1b);
//					ccso.e1a[0]+=cvx1.final_posr.pos[0];
//					ccso.e1a[1]+=cvx1.final_posr.pos[1];
//					ccso.e1a[2]+=cvx1.final_posr.pos[2];
					ccso.e1a.add(cvx1.final_posr().pos());
//					ccso.e1b[0]+=cvx1.final_posr.pos[0];
//					ccso.e1b[1]+=cvx1.final_posr.pos[1];
//					ccso.e1b[2]+=cvx1.final_posr.pos[2];
					ccso.e1b.add(cvx1.final_posr().pos());
					ccso.e2a.set(e2a);//dVector3Copy(e2a,ccso.e2a);	      
					ccso.e2b.set(e2b);//dVector3Copy(e2b,ccso.e2b);	      
//					ccso.e2a[0]+=cvx2.final_posr.pos[0];
//					ccso.e2a[1]+=cvx2.final_posr.pos[1];
//					ccso.e2a[2]+=cvx2.final_posr.pos[2];
					ccso.e2a.add(cvx2.final_posr().pos());
//					ccso.e2b[0]+=cvx2.final_posr.pos[0];
//					ccso.e2b[1]+=cvx2.final_posr.pos[1];
//					ccso.e2b[2]+=cvx2.final_posr.pos[2];
					ccso.e2b.add(cvx2.final_posr().pos());
				}	  
			}
		}
		return true;
	}

	//#if 0
	///*! \brief Returns the index of the plane/side of the incident convex (ccso.g2) 
	//      which is closer to the reference convex (ccso.g1) side
	//      
	//      This function just looks for the incident face that is facing the reference face
	//      and is the closest to being parallel to it, which sometimes is.
	//*/
	//inline unsigned int GetIncidentSide(ConvexConvexSATOutput& ccso)
	//{
	//  dVector3 nis; // (N)ormal in (I)ncident convex (S)pace
	//  dReal SavedDot;
	//  dReal Dot;
	//  unsigned int incident_side=0;
	//  // Rotate the plane normal into incident convex space 
	//  // (things like this should be done all over this file,
	//  //  will look into that)
	//  dMULTIPLY1_331(nis,ccso.g2.final_posr.R,ccso.plane);
	//  SavedDot = dDOT(nis,ccso.g2.planes);
	//  for(unsigned int i=1;i<ccso.g2.planecount;++i)
	//  {
	//    Dot = dDOT(nis,ccso.g2.planes+(i*4));
	//    if(Dot>SavedDot)
	//    {
	//      SavedDot=Dot;
	//      incident_side=i;
	//    }
	//  }
	//  return incident_side;
	//}
	//#endif

	//inline unsigned int GetSupportSide(dVector3& dir,dxConvex& cvx)
	private static int GetSupportSide(DVector3 dir,DxConvex cvx)
	{
		DVector3 dics = new DVector3(),tmp = new DVector3(); // Direction in convex space
		double SavedDot;
		double Dot;
		//unsigned 
		int side=0;
		//dVector3Copy(dir,tmp);
		tmp.set(dir);
		dNormalize3(tmp);
		dMultiply1_331(dics,cvx.final_posr().R(),tmp);
		SavedDot = dics.dot(cvx.planesV[0]);
		for(int i=1;i<cvx.planecount;++i)
		{
			Dot = dCalcVectorDot3(dics,cvx.planesV[i]);//+(i*4));
			if(Dot>SavedDot)
			{
				SavedDot=Dot;
				side=i;
			}
		}
		return side;
	}

	/** 
	 * Does an axis separation test between the 2 convex shapes
	 * using faces and edges.
	 */
	//int TestConvexIntersection(dxConvex& cvx1,dxConvex& cvx2, int flags,
	//			   dContactGeom *contact, int skip)
	private static int TestConvexIntersection(DxConvex cvx1,DxConvex cvx2, int flags,
			DContactGeomBuffer contactBuf, int skip)
	{
		ConvexConvexSATOutput ccso = new ConvexConvexSATOutput();
		ccso.min_depth=dInfinity; // Min not min at all
		ccso.depth_type=0; // no type
		// precompute distance vector
		dSubtractVectors3(ccso.dist, cvx2.final_posr().pos(), cvx1.final_posr().pos());
		int maxc = flags & NUMC_MASK;
		dIASSERT(maxc != 0);
		DVector3 i1 = new DVector3(),i2 = new DVector3(),
			r1 = new DVector3(),r2 = new DVector3(); // edges of incident and reference faces respectively
		int contacts=0;
		if(!CheckSATConvexFaces(cvx1,cvx2,ccso))
		{
			return 0;
		}
		else if(!CheckSATConvexFaces(cvx2,cvx1,ccso))
		{
			return 0;
		}
		else if(!CheckSATConvexEdges(cvx1,cvx2,ccso))
		{
			return 0;
		}
		// If we get here, there was a collision
		if(ccso.depth_type==1) // face-face
		{
			// cvx1 MUST always be in contact->g1 and cvx2 in contact->g2
			// This was learned the hard way :(    
			//unsigned 
			int incident_side;
			//unsigned int* pIncidentPoly;
			int pIncidentPolyPos;
			//unsigned int* pIncidentPoints;
			int pIncidentPointsPos;
			//unsigned int reference_side;
			int reference_side;
			//unsigned int* pReferencePoly;
			int pReferencePolyPos;
			//unsigned int* pReferencePoints;
			int pReferencePointsPos;
			//dVector4 plane,rplane,iplane;
			DVector3 planeV = new DVector3(),rplaneV = new DVector3(),iplaneV = new DVector3();
			double planeD,rplaneD,iplaneD;
			DVector3 tmp = new DVector3();
			DVector3 dist,p = new DVector3();
			RefDouble t = new RefDouble();
			double d,d1,d2;
			boolean outside,out;
			//CollisionUtil.dVector3Copy(ccso.dist,dist);
			dist = new DVector3(ccso.dist);
			reference_side = GetSupportSide(dist,cvx1);
			dNegateVector3(dist);
			incident_side = GetSupportSide(dist,cvx2);

			pReferencePolyPos = 0;//cvx1.polygons;
			pIncidentPolyPos  = 0;//cvx2.polygons;
			int[] refPolys = cvx1.polygons;
			int[] incPolys = cvx2.polygons;
			// Get Reference plane (We may not have to apply transforms Optimization Oportunity)
			// Rotate
			dMultiply0_331(rplaneV,cvx1.final_posr().R(),cvx1.planesV[reference_side]);//+(reference_side*4));
			dNormalize3(rplaneV);
			// Translate
			rplaneD = //[3]=
				(cvx1.planesD[reference_side]) + //;[(reference_side*4)+3])+
//				((rplane[0] * cvx1.final_posr.pos[0]) + 
//						(rplane[1] * cvx1.final_posr.pos[1]) + 
//						(rplane[2] * cvx1.final_posr.pos[2]));
				rplaneV.dot(cvx1.final_posr().pos());
			// flip 
//			rplane[0]=-rplane[0];
//			rplane[1]=-rplane[1];
//			rplane[2]=-rplane[2];
			rplaneV.scale(-1);
			//rplane[3]=-rplane[3];
			rplaneD =- rplaneD;
			for(int i=0;i<incident_side;++i)
			{
				pIncidentPolyPos+=incPolys[pIncidentPolyPos]+1;//IncidentPoly[0]+1;
			}
			pIncidentPointsPos = pIncidentPolyPos+1;//pIncidentPoly+1;
			// Get the first point of the incident face
			dMultiply0_331(i2,cvx2.final_posr().R(),cvx2.points,(incPolys[pIncidentPointsPos]*3));
			i2.add(cvx2.final_posr().pos());//dVector3Add(i2,cvx2._final_posr.pos,i2);
			// Get the same point in the reference convex space
			r2.set(i2);//dVector3Copy(i2,r2);
			r2.sub(cvx1.final_posr().pos());//dVector3Subtract(r2,cvx1._final_posr.pos,r2);
			tmp.set(r2);//dVector3Copy(r2,tmp);
			dMultiply1_331(r2,cvx1.final_posr().R(),tmp);
			for(int i=0;i<cvx2.polygons[pIncidentPolyPos];++i)
			{
				// Move i2 to i1, r2 to r1
				i1.set(i2);//dVector3Copy(i2,i1);
				r1.set(r2);//dVector3Copy(r2,r1);
				dMultiply0_331(i2,cvx2.final_posr().R(),
						//cvx2.points, (pIncidentPoints[(i+1)%pIncidentPoly[0]]*3) );
						cvx2.points, incPolys[pIncidentPointsPos + (i+1)%incPolys[pIncidentPolyPos]]*3) ;
				i2.add(cvx2.final_posr().pos());//dVector3Add(i2,cvx2._final_posr.pos,i2);
				// Get the same point in the reference convex space
				r2.set(i1);//dVector3Copy(i2,r2);
				r2.sub(cvx1.final_posr().pos());//dVector3Subtract(r2,cvx1._final_posr.pos,r2);
				tmp.set(r2);//dVector3Copy(r2,tmp);
				dMultiply1_331(r2,cvx1.final_posr().R(),tmp);
				outside=false;
				for(int j=0;j<cvx1.planecount;++j)
				{
//					plane[0]=cvx1.planes[(j*4)+0];
//					plane[1]=cvx1.planes[(j*4)+1];
//					plane[2]=cvx1.planes[(j*4)+2];
//					plane[3]=cvx1.planes[(j*4)+3];
					planeV.set(cvx1.planesV[j]);
					planeD = cvx1.planesD[j];
					// Get the distance from the points to the plane
//					d1 = r1[0]*plane[0]+
//					r1[1]*plane[1]+
//					r1[2]*plane[2]-
//					plane[3];
					d1 = r1.dot(planeV) - planeD;
//					d2 = r2[0]*plane[0]+
//					r2[1]*plane[1]+
//					r2[2]*plane[2]-
//					plane[3];
					d2 = r2.dot(planeV) - planeD;
					if(d1*d2<0)
					{
						out = false;

						// Edge intersects plane
						if (!IntersectSegmentPlane(r1,r2,planeV, planeD,t,p))
						{
							out = true;
						}

						if (!out) {
							out = false;
							for (int k = 0; k < cvx1.planecount; ++k) {
								//							d = p[0]*cvx1.planes[(k*4)+0]+
								//							p[1]*cvx1.planes[(k*4)+1]+
								//							p[2]*cvx1.planes[(k*4)+2]-
								//							cvx1.planes[(k*4)+3];
								d = p.dot(cvx1.planesV[k]) - cvx1.planesD[k];
								if (d > 0) {
									out = true;
									break;
								}
							}
						}
						if(!out)
						{
//							if (false) {//#if 0
//								// Use t to move p into global space
//								//	p[0] = i1[0]+((i2[0]-i1[0])*t);
//								//	p[1] = i1[1]+((i2[1]-i1[1])*t);
//								//	p[2] = i1[2]+((i2[2]-i1[2])*t);
//								p.eqSum(i1, 1-t.get(), i2, t.get());
//							} else { //#else // #if0
								// Apply reference convex transformations to p
								// The commented out piece of code is likelly to
								// produce less operations than this one, but
								// this way we know we are getting the right data
								dMultiply0_331(tmp,cvx1.final_posr().R(),p);
								p.eqSum(tmp, cvx1.final_posr().pos());//dVector3Add(tmp,cvx1._final_posr.pos,p);
//							} //#endif // #if 0
							// get p's distance to reference plane
//							d = p[0]*rplane[0]+
//							p[1]*rplane[1]+
//							p[2]*rplane[2]-
//							rplane[3];
							d = p.dot(rplaneV) - rplaneD;
							if(d>0)
							{
								// SAFECONTACT(flags, contact, contacts, skip);
								DContactGeom target = contactBuf.getSafe(flags, contacts);
								dVector3Copy(p, target.pos);
								dVector3Copy(rplaneV, target.normal);
								target.g1 = cvx1;
								target.g2 = cvx2;
								target.depth = d;
								++contacts;
								if (contacts==maxc) return contacts;
							}
						}
					}
					if(d1>0)
					{
						outside=true;
					}
				}
				if(outside) continue;
//				d = i1[0]*rplane[0]+
//				i1[1]*rplane[1]+
//				i1[2]*rplane[2]-
//				rplane[3];
				d = i1.dot(rplaneV) - rplaneD;
				if(d>0)
				{
					// dContactGeom *target = SAFECONTACT(flags, contact, contacts, skip);
					DContactGeom target = contactBuf.getSafe(flags, contacts);
					dVector3Copy(i1, target.pos);
					dVector3Copy(rplaneV, target.normal);
					target.g1 = cvx1;
					target.g2 = cvx2;
					target.depth = d;
					++contacts;
					if (contacts==maxc) return contacts;
				}
			}
			// IF we get here, we got the easiest contacts to calculate, 
			// but there is still space in the contacts array for more.
			// So, project the Reference's face points onto the Incident face
			// plane and test them for inclusion in the reference plane as well.
			// We already have computed intersections so, skip those.

			// Get Incident plane, we need it for projection
			// Rotate
			dMultiply0_331(iplaneV,cvx2.final_posr().R(),cvx2.planesV[incident_side]);//+(incident_side*4));
			dNormalize3(iplaneV);
			// Translate
//			iplane[3]=
//				(cvx2.planes[(incident_side*4)+3])+
//				((iplane[0] * cvx2.final_posr.pos[0]) + 
//						(iplane[1] * cvx2.final_posr.pos[1]) + 
//						(iplane[2] * cvx2.final_posr.pos[2]));
			iplaneD = cvx2.planesD[incident_side] + iplaneV.dot(cvx2.final_posr().pos());
			// get reference face
			for(int i=0;i<reference_side;++i)
			{
				//pReferencePoly+=pReferencePoly[0]+1;
				pReferencePolyPos += refPolys[pReferencePolyPos]+1;
			}
			//pReferencePoints = pReferencePoly+1;
			pReferencePointsPos = pReferencePolyPos + 1;
			//for(int i=0;i<pReferencePoly[0];++i)
			for(int i=0;i<refPolys[pReferencePolyPos];++i)
			{
				//dMULTIPLY0_331(i1.v,0,cvx1._final_posr.R.v,0,cvx1.points, (pReferencePoints[i]*3) );
				dMultiply0_331(i1,cvx1.final_posr().R(),cvx1.points, refPolys[pReferencePointsPos+i]*3 );
				i1.add(cvx1.final_posr().pos());//dVector3Add(cvx1._final_posr.pos,i1,i1);
				// Project onto Incident face plane      
//				t = -(i1[0]*iplane[0]+
//						i1[1]*iplane[1]+
//						i1[2]*iplane[2]-
//						iplane[3]);
				t.set( - (i1.dot(iplaneV)-iplaneD) );
//				i1[0]+=iplane[0]*t;
//				i1[1]+=iplane[1]*t;
//				i1[2]+=iplane[2]*t;
				i1.eqSum(i1, iplaneV, t.get());
				// Get the same point in the incident convex space
				r1.set(i1);//dVector3Copy(i1,r1);
				r1.sub(cvx2.final_posr().pos());//dVector3Subtract(r1,cvx2._final_posr.pos,r1);
				tmp.set(r1);//dVector3Copy(r1,tmp);
				dMultiply1_331(r1,cvx2.final_posr().R(),tmp);
				// Check if it is outside the incident convex
				out = false;
				for(int j=0;j<cvx2.planecount;++j)
				{
//					d = r1[0]*cvx2.planes[(j*4)+0]+
//					r1[1]*cvx2.planes[(j*4)+1]+
//					r1[2]*cvx2.planes[(j*4)+2]-
//					cvx2.planes[(j*4)+3];
					d = r1.dot(cvx2.planesV[j]) - cvx2.planesD[j];
					if(d>=0){out = true;break;}
				}
				if(!out)
				{
					// check that the point is not a duplicate
					outside = false;
					for(int j=0;j<contacts;++j)
					{
						// dContactGeom *cur_contact = SAFECONTACT(flags, contact, j, skip);
						DContactGeom cur_contact = contactBuf.getSafe(flags, j);
						if (cur_contact.pos.isEq(i1, 0))
						{
							outside=true;
						}
					}
					if(!outside)
					{
//						d = i1[0]*rplane[0]+
//						i1[1]*rplane[1]+
//						i1[2]*rplane[2]-
//						rplane[3];
						d = i1.dot(rplaneV) - rplaneD;
						if(d>0)
						{
							// dContactGeom *target = SAFECONTACT(flags, contact, contacts, skip);
							DContactGeom target = contactBuf.getSafe(flags, contacts);
							dVector3Copy(i1, target.pos);
							dVector3Copy(rplaneV, target.normal);
							target.g1 = cvx1;
							target.g2 = cvx2;
							target.depth = d;
							++contacts;
							if (contacts==maxc) return contacts;
						}
					}
				}
			}
		}
		else if (ccso.depth_type == 2) // edge-edge
		{
		    DVector3 c1 = new DVector3(), c2 = new DVector3();
			ClosestPointBetweenSegments(ccso.e1a, ccso.e1b, ccso.e2a, ccso.e2b, c1, c2);

			// dContactGeom *target = SAFECONTACT(flags, contact, contacts, skip);
			DContactGeom target = contactBuf.getSafe(flags, contacts);
			dSubtractVectors3(target.normal, c2, c1);
			double depth_square = dCalcVectorLengthSquare3(target.normal);

			if (dxSafeNormalize3(target.normal))
			{
				target.depth = dSqrt(depth_square);
			}
			else
			{
				// If edges coincide return direction from one center to the other as the contact normal
				dVector3Copy(ccso.dist, target.normal);

				if (!dxSafeNormalize3(target.normal))
				{
					// If the both centers coincide as well return an arbitrary vector. The depth is going to be zero anyway.
					// dAssignVector3(target.normal, 1, 0, 0);
					target.normal.set(1, 0, 0);
				}

				target.depth = 0; // Since the edges coincide, return a contact of zero depth
			}

			target.g1 = cvx1;
			target.g2 = cvx2;
			dVector3Copy(c1, target.pos);
		    contacts++;
		}
		return contacts;
	}

	static class CollideConvexConvex implements DColliderFn {
		//int dCollideConvexConvex (dxGeom *o1, dxGeom *o2, int flags,
		//			  dContactGeom *contact, int skip)
		int dCollideConvexConvex (DxConvex Convex1, DxConvex Convex2, int flags,
				DContactGeomBuffer contacts, int skip)
		{
			dIASSERT (skip >= 1);//(int)sizeof(dContactGeom));
			//  dIASSERT (o1.type == dConvexClass);
			//  dIASSERT (o2.type == dConvexClass);
			dIASSERT ((flags & NUMC_MASK) >= 1);
			//  dxConvex *Convex1 = (dxConvex*) o1;
			//  dxConvex *Convex2 = (dxConvex*) o2;
			//TODO? Passing actual objects/clone????
			//  return TestConvexIntersection(*Convex1,*Convex2,flags,
			//				contact,skip);
			return TestConvexIntersection(Convex1,Convex2,flags,
					contacts,skip);
		}

		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return dCollideConvexConvex((DxConvex)o1, (DxConvex)o2, flags, contacts, 1);
		}
	}

	//#if 0
	//int dCollideRayConvex (dxGeom *o1, dxGeom *o2, int flags, 
	//		       dContactGeom *contact, int skip)
	//{
	//  dIASSERT (skip >= (int)sizeof(dContactGeom));
	//  dIASSERT( o1->type == dRayClass );
	//  dIASSERT( o2->type == dConvexClass );
	//  dIASSERT ((flags & NUMC_MASK) >= 1);
	//  dxRay* ray = (dxRay*) o1;
	//  dxConvex* convex = (dxConvex*) o2;
	//  dVector3 origin,destination,contactpoint,out;
	//  dReal depth;
	//  dVector4 plane;
	//  unsigned int *pPoly=convex->polygons;
	//  // Calculate ray origin and destination
	//  destination[0]=0;
	//  destination[1]=0;
	//  destination[2]= ray->length;
	//  // -- Rotate --
	//  dMULTIPLY0_331(destination,ray->final_posr->R,destination);
	//  origin[0]=ray->final_posr->pos[0];
	//  origin[1]=ray->final_posr->pos[1];
	//  origin[2]=ray->final_posr->pos[2];
	//  destination[0]+=origin[0];
	//  destination[1]+=origin[1];
	//  destination[2]+=origin[2];
	//  for(int i=0;i<convex->planecount;++i)
	//    {
	//      // Rotate
	//      dMULTIPLY0_331(plane,convex->final_posr->R,convex->planes+(i*4));
	//      // Translate
	//      plane[3]=
	//	(convex->planes[(i*4)+3])+
	//	((plane[0] * convex->final_posr->pos[0]) + 
	//	 (plane[1] * convex->final_posr->pos[1]) + 
	//	 (plane[2] * convex->final_posr->pos[2]));
	//      if(IntersectSegmentPlane(origin, 
	//			       destination, 
	//			       plane, 
	//			       depth, 
	//			       contactpoint))
	//	{
	//	  if(IsPointInPolygon(contactpoint,pPoly,planeV,convex,out))
	//	    {
	//	      contact->pos[0]=contactpoint[0];
	//	      contact->pos[1]=contactpoint[1];
	//	      contact->pos[2]=contactpoint[2];
	//	      contact->normal[0]=plane[0];
	//	      contact->normal[1]=plane[1];
	//	      contact->normal[2]=plane[2];
	//	      contact->depth=depth;
	//	      contact->g1 = ray;
	//	      contact->g2 = convex;
	//  contact->side1 = -1;
	//  contact->side2 = -1; // TODO: set plane index?
	//	      return 1;
	//	    }
	//	}
	//      pPoly+=pPoly[0]+1;
	//    }
	//  return 0;
	//}
	//#else

	static class CollideRayConvex implements DColliderFn {
		// Ray - Convex collider by David Walters, June 2006
		//int dCollideRayConvex( dxGeom *o1, dxGeom *o2,
		//					   int flags, dContactGeom *contact, int skip )
		int dCollideRayConvex( DxRay ray, DxConvex convex,
				int flags, DContactGeomBuffer contacts, int skip )
		{
			dIASSERT( skip >= 1);//(int)sizeof(dContactGeom) );
			//	dIASSERT( o1.type == dRayClass );
			//	dIASSERT( o2.type == dConvexClass );
			dIASSERT ((flags & NUMC_MASK) >= 1);
	
			//	dxRay* ray = (dxRay*) o1;
			//	dxConvex* convex = (dxConvex*) o2;
	
			DContactGeom contact = contacts.get(0);
			contact.g1 = ray;
			contact.g2 = convex;
			contact.side1 = -1;
			contact.side2 = -1; // TODO: set plane index?
	
			double alpha, beta, nsign;
			boolean flag = false;
	
			//
			// Compute some useful info
			//

			//			dVector3 ray_pos = {
			//					ray->final_posr->pos[0] - convex->final_posr->pos[0],
			//					ray->final_posr->pos[1] - convex->final_posr->pos[1],
			//					ray->final_posr->pos[2] - convex->final_posr->pos[2]
			//			};
			DVector3 ray_pos = new DVector3(ray.final_posr().pos()).sub(convex.final_posr().pos());
			//
			//			dVector3 ray_dir = {
			//					ray->final_posr->R[0 * 4 + 2],
			//					ray->final_posr->R[1 * 4 + 2],
			//					ray->final_posr->R[2 * 4 + 2]
			//			};
			DVector3 ray_dir = ray.final_posr().R().columnAsNewVector(2);

			//dMultiply1_331(ray_pos, convex->final_posr->R, ray_pos);
			//dMultiply1_331(ray_dir, convex->final_posr->R, ray_dir);
			dMultiply1_331(ray_pos, convex.final_posr().R(), ray_pos);
			dMultiply1_331(ray_dir, convex.final_posr().R(), ray_dir);

			for ( int i = 0; i < convex.planecount; ++i )
			{
				// Alias this plane.
				//double* plane = convex.planes + ( i * 4 );
				int planePos = i;//*4;
	
				// If alpha >= 0 then start point is outside of plane.
				alpha = dCalcVectorDot3( convex.planesV[planePos], ray_pos ) - convex.planesD[planePos];//] - plane[3];
	
				// If any alpha is positive, then
				// the ray start is _outside_ of the hull
				if ( alpha >= 0 )
				{
					flag = true;
					break;
				}
			}
			// If the ray starts inside the convex hull, then everything is flipped.
			nsign = ( flag ) ? ( 1.0 ) : ( -1.0 );
	
	
			//
			// Find closest contact point
			//
	
			// Assume no contacts.
			contact.depth = dInfinity;
	
			for ( int i = 0; i < convex.planecount; ++i )
			{
				// Alias this plane.
				//double* plane = convex.planes + ( i * 4 );
				int planePos = i;//*4;
	
				// If alpha >= 0 then point is outside of plane.
				alpha = nsign * ( dCalcVectorDot3( convex.planesV[planePos], ray_pos ) - convex.planesD[planePos] );
				// Compute [ plane-normal DOT ray-normal ], (/flip)
				beta = convex.planesV[planePos].dot( ray_dir ) * nsign;

				// Ray is pointing at the plane? ( beta < 0 )
				// Ray start to plane is within maximum ray length?
				// Ray start to plane is closer than the current best distance?
				if ( beta < -dEpsilon &&
						alpha >= 0 && alpha <= ray.getLength() &&
						alpha < contact.depth )
				{
					// Compute contact point on convex hull surface.
					contact.pos.eqSum(ray_pos, 1, ray_dir, alpha);
	
					flag = false;
	
					// For all _other_ planes.
					for ( int j = 0; j < convex.planecount; ++j )
					{
						if ( i == j )
							continue;	// Skip self.
	
						// Alias this plane.
						//double* planej = convex.planes + ( j * 4 );
						int planePosJ = j;//*4;
	
						// If beta >= 0 then start is outside of plane.
						//beta = dDOT( planej, contact.pos ) - plane[3];
						//TODO use planePos+3 or planePosJ+3 ???
						//beta = dDOT( convex.planesV[planePosJ], contact.pos) - convex.planesD[planePosJ];
						beta = dCalcVectorDot3( convex.planesV[planePosJ], contact.pos) - convex.planesD[planePosJ];
	
						// If any beta is positive, then the contact point
						// is not on the surface of the convex hull - it's just
						// intersecting some part of its infinite extent.
						if ( beta > dEpsilon )
						{
							flag = true;
							break;
						}
					}
	
					// Contact point isn't outside hull's surface? then it's a good contact!
					if ( flag == false )
					{
						// Store the contact normal, possibly flipped.
						contact.normal.set(convex.planesV[planePos]).scale(nsign);
	
						// Store depth
						contact.depth = alpha;
	
						if ((flags & CONTACTS_UNIMPORTANT)!=0 && contact.depth <= ray.getLength() )
						{
							// Break on any contact if contacts are not important
							break; 
						}
					}
				}
			}
			// Contact?
			if (contact.depth <= ray.getLength()) {
				// Adjust contact position and normal back to global space
				dMultiply0_331(contact.pos, convex.final_posr().R(), contact.pos);
				dMultiply0_331(contact.normal, convex.final_posr().R(), contact.normal);
				contact.pos.add(convex.final_posr().pos());
				return 1;				
			}
			return 0;
		}

		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return dCollideRayConvex((DxRay)o1, (DxConvex)o2, flags, contacts, 1);
		}
	}

	@Override
	public void setConvex(double[] planes, int planeCount, double[] points,
						  int pointCount, int[] polygons) {
		dGeomSetConvex(planes, planeCount, points, pointCount, polygons);
	}

	// Only in ode4j
	@Override
	public void setConvex(double[] planes, double[] points, int[] polygons) {
		dUASSERT(planes.length % 4 == 0, "planes are defined by four values: x, y, z, depth");
		dUASSERT(points.length % 3 == 0, "points are defined by three values: x, y, z");
		dGeomSetConvex(planes, planes.length/4, points, points.length/3, polygons);
	}

	public double[] getPoints() {
		return points;
	}

	public int getPointcount() {
		return pointcount;
	}
}