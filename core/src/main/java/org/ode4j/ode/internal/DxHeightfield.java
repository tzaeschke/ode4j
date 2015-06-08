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
import static org.ode4j.ode.OdeMath.dMultiply0_331;
import static org.ode4j.ode.OdeMath.dMultiply1_331;
import static org.ode4j.ode.OdeMath.dMultiply1_333;
import static org.ode4j.ode.internal.Common.dCeil;
import static org.ode4j.ode.internal.Common.dEpsilon;
import static org.ode4j.ode.internal.Common.dFabs;
import static org.ode4j.ode.internal.Common.dFloor;
import static org.ode4j.ode.internal.Common.dIASSERT;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DAABB;
import org.ode4j.ode.DColliderFn;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DHeightfieldData;
import org.ode4j.ode.internal.cpp4j.java.ObjArray;

/**
 *
 * @author Tilmann Zaeschke
 */
public class DxHeightfield extends DxAbstractHeightfield {

	static final int HEIGHTFIELDMAXCONTACTPERCELL = 10;

	//#define dMIN(A,B)  ((A)>(B) ? (B) : (A))
	private static final double dMIN(double A, double B) { return ((A)>(B) ? (B) : (A)); }
	//#define dMAX(A,B)  ((A)>(B) ? (A) : (B))
	private static final double dMAX(double A, double B) { return ((A)>(B) ? (A) : (B)); }
	//
	//
	////Three-way MIN and MAX
	//#define dMIN3(A,B,C)	( (A)<(B) ? dMIN((A),(C)) : dMIN((B),(C)) )
	private final double dMIN3(double A, double B, double C) { return A<B ? dMIN(A,C) : dMIN(B,C); }
	//#define dMAX3(A,B,C)	( (A)>(B) ? dMAX((A),(C)) : dMAX((B),(C)) )
	private final double dMAX3(double A, double B, double C) { return A>B ? dMAX(A,C) : dMAX(B,C); }
	//
	//#define dOPESIGN(a, op1, op2,b) \
	//(a)[0] op1 op2 ((b)[0]); \
	//(a)[1] op1 op2 ((b)[1]); \
	//(a)[2] op1 op2 ((b)[2]);
	//
	//#define dGeomRaySetNoNormalize(myRay, MyPoint, MyVector) {  \
	//\
	//dVector3Copy (MyPoint, (myRay).final_posr.pos);   \
	//(myRay).final_posr.R[2] = (MyVector)[0];       \
	//(myRay).final_posr.R[6] = (MyVector)[1];       \
	//(myRay).final_posr.R[10] = (MyVector)[2];      \
	//dGeomMoved (&myRay);                        \
	//        }
	private void dGeomRaySetNoNormalize(DxRay myRay, DVector3C MyPoint, DVector3C MyVector) {  
		myRay._final_posr.pos.set(MyPoint);  
		DMatrix3 R = myRay._final_posr.Rw();
		R.set(0, 2, MyVector.get0());       
		R.set(1, 2, MyVector.get1());       
		R.set(2, 2, MyVector.get2());      
		myRay.dGeomMoved();                        
	}
	//
	//#define dGeomPlaneSetNoNormalize(MyPlane, MyPlaneDef) { \
	//\
	//(MyPlane).p[0] = (MyPlaneDef)[0];  \
	//(MyPlane).p[1] = (MyPlaneDef)[1];  \
	//(MyPlane).p[2] = (MyPlaneDef)[2];  \
	//(MyPlane).p[3] = (MyPlaneDef)[3];  \
	//dGeomMoved (MyPlane);           \
	//                }
	private void dGeomPlaneSetNoNormalize(DxPlane MyPlane, DVector3 MyPlaneDefV, double MyPlaneDefD) {
		//		MyPlane.p[0] = (MyPlaneDef)[0];  
		//		MyPlane.p[1] = (MyPlaneDef)[1];  
		//		MyPlane.p[2] = (MyPlaneDef)[2];  
		//		MyPlane.p[3] = (MyPlaneDef)[3];  
		MyPlane.setParams( MyPlaneDefV, MyPlaneDefD );  
		MyPlane.dGeomMoved ();           
	}



	////////Local Build Option ////////////////////////////////////////////////////

	//Uncomment this #define to use the (0,0) corner of the geom as the origin,
	//rather than the center. This was the way the original heightfield worked,
	//but as it does not match the way all other geometries work, so for constancy it
	//was changed to work like this.

	//#define DHEIGHTFIELD_CORNER_ORIGIN
	private static final boolean DHEIGHTFIELD_CORNER_ORIGIN = false;  //DO NOT ENABLE without uncommenting code!!


	//Uncomment this #define to add heightfield triangles edge colliding
	//Code is not guaranteed and I didn't find the need to add that as 
	//colliding planes triangles and edge triangles seems enough.
	//#define _HEIGHTFIELDEDGECOLLIDING
	//private static final boolean _HEIGHTFIELDEDGECOLLIDING = false;  //DO NOT ENABLE without uncommenting code!!


	private static final boolean NO_CONTACT_CULLING_BY_ISONHEIGHTFIELD2 = false;


	////////dxHeightfield /////////////////////////////////////////////////////////////////


	//	    dxHeightfield( dSpaceID space, dHeightfieldDataID data, int bPlaceable );
	//	    ~dxHeightfield();
	//
	//	    void computeAABB();
	//
	//	    int dCollideHeightfieldZone( const int minX, const int maxX, const int minZ, const int maxZ,  
	//	        dxGeom *o2, const int numMaxContacts,
	//	        int flags, dContactGeom *contact, int skip );

	//		private enum TEMP
	//		{
	private static final int TEMP_PLANE_BUFFER_ELEMENT_COUNT_ALIGNMENT = 4;
	private static final int TEMP_HEIGHT_BUFFER_ELEMENT_COUNT_ALIGNMENT_X = 4;
	private static final int TEMP_HEIGHT_BUFFER_ELEMENT_COUNT_ALIGNMENT_Z = 4;
	private static final int TEMP_TRIANGLE_BUFFER_ELEMENT_COUNT_ALIGNMENT = 1; // Triangles are easy to reallocate and hard to predict
	//		};

	//static inline size_t AlignBufferSize(size_t value, size_t alignment) { dIASSERT((alignment & (alignment - 1)) == 0); return (value + (alignment - 1)) & ~(alignment - 1); }
	private static int AlignBufferSize(int value, int alignment) { 
		dIASSERT((alignment & (alignment - 1)) == 0); 
		return (value + (alignment - 1)) & ~(alignment - 1); 
	}

	//		void  allocateTriangleBuffer(size_t numTri);
	//		void  resetTriangleBuffer();
	//		void  allocatePlaneBuffer(size_t numTri);
	//		void  resetPlaneBuffer();
	//		void  allocateHeightBuffer(size_t numX, size_t numZ);
	//	    void  resetHeightBuffer();

	//	    void  sortPlanes(const size_t numPlanes);

	//	    HeightFieldPlane    **tempPlaneBuffer;
	//	    HeightFieldPlane    *tempPlaneInstances;
	//	    size_t              tempPlaneBufferSize;
	//
	//	    HeightFieldTriangle *tempTriangleBuffer;
	//	    size_t              tempTriangleBufferSize;
	//
	//	    HeightFieldVertex   **tempHeightBuffer;
	//		HeightFieldVertex   *tempHeightInstances;
	//	    size_t              tempHeightBufferSizeX;
	//	    size_t              tempHeightBufferSizeZ;
	//private HeightFieldPlane[]    tempPlaneBuffer;
	private ObjArray<HeightFieldPlane>[]  tempPlaneBuffer;
	private HeightFieldPlane[]    tempPlaneInstances;
	private int              tempPlaneBufferSize;

	private HeightFieldTriangle[] tempTriangleBuffer;
	private int              tempTriangleBufferSize;

	//private HeightFieldVertex[]  tempHeightBuffer;
	private ObjArray<HeightFieldVertex>[]  tempHeightBuffer;
	private HeightFieldVertex[]   tempHeightInstances;
	private int              tempHeightBufferSizeX;
	private int              tempHeightBufferSizeZ;



	// dxHeightfield constructor
	private DxHeightfield( DxSpace space, DxHeightfieldData data, boolean bPlaceable ) {			//:
		//	    dxGeom( space, bPlaceable ),
		//	    tempPlaneBuffer(0),
		//		tempPlaneInstances(0),
		//	    tempPlaneBufferSize(0),
		//	    tempTriangleBuffer(0),
		//	    tempTriangleBufferSize(0),
		//	    tempHeightBuffer(0),
		//		tempHeightInstances(0),
		//	    tempHeightBufferSizeX(0),
		//	    tempHeightBufferSizeZ(0)
		super( space, bPlaceable );
		tempPlaneBuffer = null;
		tempPlaneInstances = null;
		tempPlaneBufferSize = 0;
		tempTriangleBuffer = null;
		tempTriangleBufferSize = 0;
		tempHeightBuffer = null;
		tempHeightInstances = null;
		tempHeightBufferSizeX = 0;
		tempHeightBufferSizeZ = 0;

		type = dHeightfieldClass;
		m_p_data = data;
	}


	// compute axis aligned bounding box
	@Override
	void computeAABB()
	{
		final DxHeightfieldData d = m_p_data;

		if ( d.m_bWrapMode == false )
		{
			// Finite
			//if ( (_gflags & GEOM_PLACEABLE)!=0 )
			if (hasFlagPlaceable())
			{
				double[] dx=new double[6], dy=new double[6], dz=new double[6];

				// Y-axis
	            if (d.m_fMinHeight != -dInfinity)
	            {
	            	dy[0] = ( final_posr().R().get01() * d.m_fMinHeight );
	            	dy[1] = ( final_posr().R().get11() * d.m_fMinHeight );
	            	dy[2] = ( final_posr().R().get21() * d.m_fMinHeight );
	            } else {
	                // Multiplication is performed to obtain infinity of correct sign
	                dy[0] = ( final_posr().R().get01()!=0 ? final_posr().R().get01() * -dInfinity : (0.0) );
	                dy[1] = ( final_posr().R().get11()!=0 ? final_posr().R().get11() * -dInfinity : (0.0) );
	                dy[2] = ( final_posr().R().get21()!=0 ? final_posr().R().get21() * -dInfinity : (0.0) );
	            }

	            if (d.m_fMaxHeight != dInfinity)
	            {
				dy[3] = ( final_posr().R().get01() * d.m_fMaxHeight );
				dy[4] = ( final_posr().R().get11() * d.m_fMaxHeight );
				dy[5] = ( final_posr().R().get21() * d.m_fMaxHeight );
	            } else {
	                dy[3] = ( final_posr().R().get01()!=0 ? final_posr().R().get01() * dInfinity : (0.0) );
	                dy[4] = ( final_posr().R().get11()!=0 ? final_posr().R().get11() * dInfinity : (0.0) );
	                dy[5] = ( final_posr().R().get21()!=0 ? final_posr().R().get21() * dInfinity : (0.0) );
	            }


				//	#ifdef DHEIGHTFIELD_CORNER_ORIGIN
				//
				//	            // X-axis
				//	            dx[0] = 0;	dx[3] = ( _final_posr.R[ 0] * d.m_fWidth );
				//	            dx[1] = 0;	dx[4] = ( _final_posr.R[ 4] * d.m_fWidth );
				//	            dx[2] = 0;	dx[5] = ( _final_posr.R[ 8] * d.m_fWidth );
				//
				//	            // Z-axis
				//	            dz[0] = 0;	dz[3] = ( _final_posr.R[ 2] * d.m_fDepth );
				//	            dz[1] = 0;	dz[4] = ( _final_posr.R[ 6] * d.m_fDepth );
				//	            dz[2] = 0;	dz[5] = ( _final_posr.R[10] * d.m_fDepth );
				//
				//	#else // DHEIGHTFIELD_CORNER_ORIGIN

				// X-axis
				dx[0] = ( final_posr().R().get00() * -d.m_fHalfWidth );
				dx[1] = ( final_posr().R().get10() * -d.m_fHalfWidth );
				dx[2] = ( final_posr().R().get20() * -d.m_fHalfWidth );
				dx[3] = ( final_posr().R().get00() * d.m_fHalfWidth );
				dx[4] = ( final_posr().R().get10() * d.m_fHalfWidth );
				dx[5] = ( final_posr().R().get20() * d.m_fHalfWidth );

				// Z-axis
				dz[0] = ( final_posr().R().get02() * -d.m_fHalfDepth );
				dz[1] = ( final_posr().R().get12() * -d.m_fHalfDepth );
				dz[2] = ( final_posr().R().get22() * -d.m_fHalfDepth );
				dz[3] = ( final_posr().R().get02() * d.m_fHalfDepth );
				dz[4] = ( final_posr().R().get12() * d.m_fHalfDepth );
				dz[5] = ( final_posr().R().get22() * d.m_fHalfDepth );

				//	#endif // DHEIGHTFIELD_CORNER_ORIGIN

				// X extents
				_aabb.setMin0(final_posr().pos().get0() +
						dMIN3( dMIN( dx[0], dx[3] ), dMIN( dy[0], dy[3] ), dMIN( dz[0], dz[3] ) ) );
				_aabb.setMax0(final_posr().pos().get0() +
						dMAX3( dMAX( dx[0], dx[3] ), dMAX( dy[0], dy[3] ), dMAX( dz[0], dz[3] ) ) );

				// Y extents
				_aabb.setMin1(final_posr().pos().get1() +
						dMIN3( dMIN( dx[1], dx[4] ), dMIN( dy[1], dy[4] ), dMIN( dz[1], dz[4] ) ) );
				_aabb.setMax1(final_posr().pos().get1() +
						dMAX3( dMAX( dx[1], dx[4] ), dMAX( dy[1], dy[4] ), dMAX( dz[1], dz[4] ) ) );

				// Z extents
				_aabb.setMin2(final_posr().pos().get2() +
						dMIN3( dMIN( dx[2], dx[5] ), dMIN( dy[2], dy[5] ), dMIN( dz[2], dz[5] ) ) );
				_aabb.setMax2(final_posr().pos().get2() +
						dMAX3( dMAX( dx[2], dx[5] ), dMAX( dy[2], dy[5] ), dMAX( dz[2], dz[5] ) ) );
			}
			else
			{

				//	#ifdef DHEIGHTFIELD_CORNER_ORIGIN
				//
				//	            aabb[0] = 0;					aabb[1] = d.m_fWidth;
				//	            aabb[2] = d.m_fMinHeight;		aabb[3] = d.m_fMaxHeight;
				//	            aabb[4] = 0;					aabb[5] = d.m_fDepth;
				//
				//	#else // DHEIGHTFIELD_CORNER_ORIGIN

				//				aabb[0] = -d.m_fHalfWidth;		aabb[1] = +d.m_fHalfWidth;
				//				aabb[2] = d.m_fMinHeight;		aabb[3] = d.m_fMaxHeight;
				//				aabb[4] = -d.m_fHalfDepth;		aabb[5] = +d.m_fHalfDepth;
				_aabb.set(-d.m_fHalfWidth, +d.m_fHalfWidth,
						d.m_fMinHeight, d.m_fMaxHeight, 
						-d.m_fHalfDepth, +d.m_fHalfDepth);

				//	#endif // DHEIGHTFIELD_CORNER_ORIGIN

			}
		}
		else
		{
			// Infinite
			//if ( (_gflags & GEOM_PLACEABLE)!=0 )
			if (hasFlagPlaceable())
			{
				//				aabb[0] = -dInfinity;			aabb[1] = +dInfinity;
				//				aabb[2] = -dInfinity;			aabb[3] = +dInfinity;
				//				aabb[4] = -dInfinity;			aabb[5] = +dInfinity;
				_aabb.set(-dInfinity, +dInfinity,
						-dInfinity, +dInfinity,
						-dInfinity, +dInfinity);

			}
			else
			{
				//				aabb[0] = -dInfinity;			aabb[1] = +dInfinity;
				//				aabb[2] = d.m_fMinHeight;		aabb[3] = d.m_fMaxHeight;
				//				aabb[4] = -dInfinity;			aabb[5] = +dInfinity;
				_aabb.set(-dInfinity, +dInfinity,
						d.m_fMinHeight, d.m_fMaxHeight,
						-dInfinity, +dInfinity);
			}
		}

	}


	// dxHeightfield destructor
	//dxHeightfield::~dxHeightfield()
	@Override
	public void DESTRUCTOR()
	{
		resetTriangleBuffer();
		resetPlaneBuffer();
		resetHeightBuffer();
		super.DESTRUCTOR();
	}

	private void allocateTriangleBuffer(int numTri)
	{
		int alignedNumTri = AlignBufferSize(numTri, TEMP_TRIANGLE_BUFFER_ELEMENT_COUNT_ALIGNMENT);
		tempTriangleBufferSize = alignedNumTri;
		tempTriangleBuffer = new HeightFieldTriangle[alignedNumTri];
		for (int i = 0; i < tempTriangleBuffer.length; i++) tempTriangleBuffer[i] = new HeightFieldTriangle();
	}

	private void resetTriangleBuffer()
	{
		//delete[] tempTriangleBuffer;
		tempTriangleBuffer = null;
		//TODO set size == 0? TZ
	}

	@SuppressWarnings("unchecked")
    private void allocatePlaneBuffer(int numTri)
	{
		int alignedNumTri = AlignBufferSize(numTri, TEMP_PLANE_BUFFER_ELEMENT_COUNT_ALIGNMENT);
		tempPlaneBufferSize = alignedNumTri;
		//tempPlaneBuffer = new HeightFieldPlane [alignedNumTri];
		tempPlaneBuffer = new ObjArray[alignedNumTri];
		tempPlaneInstances = new HeightFieldPlane[alignedNumTri];
		for (int indexTri = 0; indexTri != alignedNumTri; indexTri++)
			tempPlaneInstances[indexTri] = new HeightFieldPlane();

		//HeightFieldPlane ptrPlaneMatrix = tempPlaneInstances;
		for (int indexTri = 0; indexTri != alignedNumTri; indexTri++)
		{
			//tempPlaneBuffer[indexTri] = tempPlaneInstances[indexTri];//ptrPlaneMatrix;
			//ptrPlaneMatrix += 1;
			tempPlaneBuffer[indexTri] = new ObjArray<HeightFieldPlane>(tempPlaneInstances, indexTri);//ptrPlaneMatrix;
		}
	}

	private void resetPlaneBuffer()
	{
		//		delete[] tempPlaneInstances;
		//		delete[] tempPlaneBuffer;
		tempPlaneInstances = null;
		tempPlaneBuffer = null;
	}

	@SuppressWarnings("unchecked")
    private void allocateHeightBuffer(int numX, int numZ)
	{
		int alignedNumX = AlignBufferSize(numX, TEMP_HEIGHT_BUFFER_ELEMENT_COUNT_ALIGNMENT_X);
		int alignedNumZ = AlignBufferSize(numZ, TEMP_HEIGHT_BUFFER_ELEMENT_COUNT_ALIGNMENT_Z);
		tempHeightBufferSizeX = alignedNumX;
		tempHeightBufferSizeZ = alignedNumZ;
		//tempHeightBuffer = new HeightFieldVertex *[alignedNumX];
		tempHeightBuffer = new ObjArray[alignedNumX];
		int numCells = alignedNumX * alignedNumZ;
		tempHeightInstances = new HeightFieldVertex [numCells];
		for (int i = 0; i < tempHeightInstances.length; i++) {
			tempHeightInstances[i] = new HeightFieldVertex();
		}

		//HeightFieldVertex *ptrHeightMatrix = tempHeightInstances;
		for (int indexX = 0; indexX != alignedNumX; indexX++)
		{
			//			tempHeightBuffer[indexX] = ptrHeightMatrix;
			//			ptrHeightMatrix += alignedNumZ;
			//tempHeightBuffer[indexX] = tempHeightInstances[indexX];
			tempHeightBuffer[indexX] = new ObjArray<HeightFieldVertex>(tempHeightInstances, indexX*alignedNumZ);
		}
	}

	void resetHeightBuffer()
	{
		//delete[] tempHeightInstances;
		//delete[] tempHeightBuffer;
		tempHeightInstances = null;
		tempHeightBuffer = null;
	}

	//////// Heightfield geom interface ////////////////////////////////////////////////////

	/**
	 * @param space 
	 * @param data 
	 * @param bPlaceable 
	 * @return New DHeightfield
	 */
	public static DxHeightfield dCreateHeightfield( DxSpace space, DxHeightfieldData data, boolean bPlaceable )
	{
		return new DxHeightfield( space, data, bPlaceable );
	}


	//	void dGeomHeightfieldSetHeightfieldData( dGeom g, dHeightfieldData d )
	void dGeomHeightfieldSetHeightfieldData( DHeightfieldData d )
	{
		//dxHeightfield* geom = (dxHeightfield*) g;
		//TODO change type of m_p_data?
		m_p_data = (DxHeightfieldData) d;
	}


	//	dHeightfieldData dGeomHeightfieldGetHeightfieldData( dGeom g )
	DHeightfieldData dGeomHeightfieldGetHeightfieldData( )
	{
		//dxHeightfield* geom = (dxHeightfield*) g;
		return m_p_data;
	}





	// from heightfield.h

	//typedef int HeightFieldVertexCoords[2];

	class HeightFieldVertex
	{
		//	public:
		HeightFieldVertex(){};

		DVector3 vertex = new DVector3();
		//HeightFieldVertexCoords coords;
		//int[] coords = new int[2]; //use c1 & c2 (TZ)
		int coords0, coords1;
		boolean state;
	};

	//TODO TZ not used
//	private class HeightFieldEdge
//	{
//		//public:
//		HeightFieldEdge(){};
//
//		//HeightFieldVertex   *vertices[2];
//		HeightFieldVertex[]   vertices = new HeightFieldVertex[2];  //TODO v1/v1 (TZ)
//	};

	private class HeightFieldTriangle
	{
		//public:
		HeightFieldTriangle(){};

		void setMinMax()
		{
			maxAAAB = vertices[0].vertex.get1() > vertices[1].vertex.get1() ? vertices[0].vertex.get1() : vertices[1].vertex.get1();
			maxAAAB = vertices[2].vertex.get1() > maxAAAB  ? vertices[2].vertex.get1() : maxAAAB;
		};

		//HeightFieldVertex   *vertices[3];
		HeightFieldVertex[]   vertices = new HeightFieldVertex[3]; //TODO c1, c2, c3 (TZ)
		//double[]               planeDef=new double[4];
		DVector3 planeDefV		 = new DVector3();
		double planeDefD;				 
		double               maxAAAB;

		boolean                isUp;
		boolean                state;
	};

	private class HeightFieldPlane
	{
		void setMinMax()
		{
			final int asize = trianglelistCurrentSize;
			if (asize > 0)
			{
				maxAAAB = trianglelist[0].maxAAAB;
				for (int k = 1; asize > k; k++)
				{
					if (trianglelist[k].maxAAAB >  maxAAAB)
						maxAAAB = trianglelist[k].maxAAAB;
				}
			}
		};

		public void resetTriangleListSize(final int newSize)
		{
			if (trianglelistReservedSize < newSize)
			{
				//delete [] trianglelist;
				trianglelistReservedSize = newSize;
				trianglelist = new HeightFieldTriangle [newSize];
			}
			trianglelistCurrentSize = 0;
		}

		//void addTriangle(HeightFieldTriangle *tri)
		public void addTriangle(HeightFieldTriangle tri)
		{
			//TODO use ArrayList or ArrayBUCKETList? (TZ)
			dIASSERT(trianglelistCurrentSize < trianglelistReservedSize);

			trianglelist[trianglelistCurrentSize++] = tri;
		}

		//	    HeightFieldTriangle **trianglelist;
		//	    size_t              trianglelistReservedSize;
		//	    size_t              trianglelistCurrentSize;
		HeightFieldTriangle[] trianglelist;
		int              trianglelistReservedSize;
		int              trianglelistCurrentSize;

		double   maxAAAB;
		//double[]   planeDef = new double[4];
		DVector3   planeDefV = new DVector3();
		double planeDefD;
	};





	//////// dxHeightfield /////////////////////////////////////////////////////////////////


	// Typedef for generic 'get point depth' function
	//typedef double dGetDepthFn( dGeomID g, dReal x, dReal y, dReal z );
	private interface dGetDepthFn {
		double get( DGeom g, double x, double y, double z );
	}


	//	#define DMESS(A)	\
	//	    dMessage(0,"Contact Plane (%d %d %d) %.5e %.5e (%.5e %.5e %.5e)(%.5e %.5e %.5e)).",	\
	//	    x,z,(A),	\
	//	    pContact->depth,	\
	//	    dGeomSphereGetRadius(o2),		\
	//	    pContact->pos[0],	\
	//	    pContact->pos[1],	\
	//	    pContact->pos[2],	\
	//	    pContact->normal[0],	\
	//	    pContact->normal[1],	\
	//	    pContact->normal[2]);
	//	private static final void DMESS(A)

	//TZ not used
	//static inline bool DescendingTriangleSort(const HeightFieldTriangle * const A, const HeightFieldTriangle * const B)
//	private boolean DescendingTriangleSort(final HeightFieldTriangle A, final HeightFieldTriangle B)
//	{
//		return ((A.maxAAAB - B.maxAAAB) > dEpsilon);
//	}
	//static inline boolean DescendingPlaneSort(final HeightFieldPlane * final A, final HeightFieldPlane * final B)
	private boolean DescendingPlaneSort(final HeightFieldPlane A, final HeightFieldPlane B)
	{
		return ((A.maxAAAB - B.maxAAAB) > dEpsilon);
	}

	void sortPlanes(final int numPlanes)
	{
		boolean has_swapped = true;
		do
		{
			has_swapped = false;//reset flag
			for (int i = 0; i < numPlanes - 1; i++)
			{
				//if they are in the wrong order
				if (DescendingPlaneSort(tempPlaneBuffer[i].at0(), tempPlaneBuffer[i + 1].at0()))
				{
					//exchange them
					//HeightFieldPlane * tempPlane = tempPlaneBuffer[i];
					ObjArray<HeightFieldPlane> tempPlane = tempPlaneBuffer[i];
					tempPlaneBuffer[i] = tempPlaneBuffer[i + 1];
					tempPlaneBuffer[i + 1] = tempPlane;

					//we have swapped at least once, list may not be sorted yet
					has_swapped = true;
				}
			}
		}    //if no swaps were made during this pass, the list has been sorted
		while (has_swapped);
	}

	//TZ not used
	//	static inline dReal DistancePointToLine(final dVector3 &_point,
	//            final dVector3 &_pt0,
	//            final dVector3 &_Edge,
	//            final dReal _Edgelength)
//	private double DistancePointToLine(final dVector3 _point,
//			final dVector3 _pt0,
//			final dVector3 _Edge,
//			final double _Edgelength)
//	{
//		dVector3 v = new dVector3();
//		v.eqDiff(_point, _pt0);//dVector3Subtract(_point, _pt0, v);
//		dVector3 s = new dVector3();
//		//dVector3Copy (_Edge, s);
//		s.set(_Edge);
//		final double dot = v.reDot(_Edge) / _Edgelength;//dVector3Dot(v, _Edge) / _Edgelength;
//		s.scale(dot);//dVector3Scale(s, dot);
//		v.eqDiff(v, s);//dVector3Subtract(v, s, v);
//		return v.length();//dVector3Length(v);
//	}


//	int dCollideRayTrimesh( dxGeom *ray, dxGeom *trimesh, int flags,
//	dContactGeom *contact, int skip )
//{
//// Swapped case, for code that needs it (heightfield initially)
//// The other ray-geom colliders take geoms in a swapped order to the
//// dCollideRTL function which is annoying when using function pointers.
//return dCollideRTL( trimesh, ray, flags, contact, skip );
//}
	private static class CollideRayTrimesh implements DColliderFn {
		CollideTrimeshRay collider = new CollideTrimeshRay();
		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return collider.dCollideRTL((DxTriMesh)o2, (DxRay)o1, flags, contacts, 1);
		}
	};

	//	int dxHeightfield::dCollideHeightfieldZone( final int minX, final int maxX, final int minZ, final int maxZ,
	//            dxGeom* o2, final int numMaxContactsPossible,
	//            int flags, dContactGeom* contact,
	//            int skip )
	int dCollideHeightfieldZone( final int minX, final int maxX, final int minZ, final int maxZ,
			DxGeom o2, final int numMaxContactsPossible,
			int flags, DContactGeomBuffer contacts,
			int skip )
	{
		DContactGeom pContact = null;
		int  x, z;
		// check if not above or inside terrain first
		// while filling a heightmap partial temporary buffer
		//	    final unsigned int numX = (maxX - minX) + 1;
		//	    final unsigned int numZ = (maxZ - minZ) + 1;
		final int numX = (maxX - minX) + 1;
		final int numZ = (maxZ - minZ) + 1;
		final double minO2Height = o2._aabb.getMin1();
		final double maxO2Height = o2._aabb.getMax1();
		//unsigned 
		int x_local, z_local;
		double maxY = - dInfinity;
		double minY = dInfinity;
		// localize and final for faster access
		final double cfSampleWidth = m_p_data.m_fSampleWidth;
		final double cfSampleDepth = m_p_data.m_fSampleDepth;
		{
			if (tempHeightBufferSizeX < numX || tempHeightBufferSizeZ < numZ)
			{
				resetHeightBuffer();
				allocateHeightBuffer(numX, numZ);
			}

			double Xpos, Ypos;

			for ( x = minX, x_local = 0; x_local < numX; x++, x_local++)
			{
				Xpos = x * cfSampleWidth; // Always calculate pos via multiplication to avoid computational error accumulation during multiple additions

				final double c_Xpos = Xpos;
				//HeightFieldVertex HeightFieldRow = tempHeightBuffer[x_local];
				//ObjArray<HeightFieldVertex> HeightFieldRow = new ObjArray<HeightFieldVertex>(tempHeightBuffer, x_local);
				ObjArray<HeightFieldVertex> HeightFieldRow = tempHeightBuffer[x_local];
				for ( z = minZ, z_local = 0; z_local < numZ; z++, z_local++)
				{
					Ypos = z * cfSampleDepth; // Always calculate pos via multiplication to avoid computational error accumulation during multiple additions

					final double h = m_p_data.GetHeight(x, z);
					//					HeightFieldRow.at(z_local).vertex[0] = c_Xpos;
					//					HeightFieldRow.at(z_local).vertex[1] = h;
					//					HeightFieldRow.at(z_local).vertex[2] = Ypos;
					HeightFieldRow.at(z_local).vertex.set( c_Xpos, h, Ypos);
					HeightFieldRow.at(z_local).coords0 = x;
					HeightFieldRow.at(z_local).coords1 = z;

					maxY = dMAX(maxY, h);
					minY = dMIN(minY, h);
				}
			}
			if (minO2Height - maxY > -dEpsilon )
			{
				//totally above heightfield
				return 0;
			}
			if (minY - maxO2Height > -dEpsilon )
			{
				// totally under heightfield
				//pContact = CONTACT(contact, 0);
				pContact = contacts.get(0); 

				//				pContact.pos[0] = o2._final_posr.pos.get0();
				//				pContact.pos[1] = minY;
				//				pContact.pos[2] = o2._final_posr.pos.get2();
				pContact.pos.set( o2.final_posr().pos().get0(), minY, o2.final_posr().pos().get2() );

				//				pContact.normal[0] = 0;
				//				pContact.normal[1] = - 1;
				//				pContact.normal[2] = 0;
				pContact.normal.set( 0, -1, 0 );

				pContact.depth =  minY - maxO2Height;

	            pContact.side1 = -1;
	            pContact.side2 = -1;

				return 1;
			}
		}
		// get All Planes that could collide against.
		DColliderFn geomRayNCollider=null;
		DColliderFn geomNPlaneCollider=null;
		dGetDepthFn geomNDepthGetter=null;

		// int max_collisionContact = numMaxContactsPossible; -- not used
		switch (o2.type)
		{
		case dRayClass:
			geomRayNCollider		= null;
			geomNPlaneCollider	    = new DxRay.CollideRayPlane();//dCollideRayPlane;
			geomNDepthGetter		= null;
			//max_collisionContact    = 1;
			break;

		case dSphereClass:
			geomRayNCollider		= new DxRay.CollideRaySphere();//dCollideRaySphere;
			geomNPlaneCollider  	= new DxSphere.CollideSpherePlane();//.dCollideSpherePlane;
			geomNDepthGetter		= //dGeomSpherePointDepth;
				new dGetDepthFn() {
				@Override
				public double get(DGeom g, double x, double y, double z) {
					return ((DxSphere)g).dGeomSpherePointDepth(x, y, z);
				}};
				//max_collisionContact    = 3;
				break;

		case dBoxClass:
			geomRayNCollider		= new DxRay.CollideRayBox();//dCollideRayBox;
			geomNPlaneCollider	    = new CollideBoxPlane();//dCollideBoxPlane;
			geomNDepthGetter		= //dGeomBoxPointDepth;
				new dGetDepthFn() {
				@Override
				public double get(DGeom g, double x, double y, double z) {
					return ((DxBox)g).dGeomBoxPointDepth(x, y, z);
				}};
				//max_collisionContact    = 8;
				break;

		case dCapsuleClass:
			geomRayNCollider		= new DxRay.CollideRayCapsule();//dCollideRayCapsule;
			geomNPlaneCollider  	= new DxCapsule.CollideCapsulePlane();//dCollideCapsulePlane;
			geomNDepthGetter		= //dGeomCapsulePointDepth;
				new dGetDepthFn() {
				@Override
				public double get(DGeom g, double x, double y, double z) {
					return ((DxCapsule)g).dGeomCapsulePointDepth(x, y, z);
				}};
				// max_collisionContact    = 3;
				break;

		case dCylinderClass:
			geomRayNCollider		= new DxRay.CollideRayCylinder();//dCollideRayCylinder;
			geomNPlaneCollider	    = new CollideCylinderPlane();//dCollideCylinderPlane;
			geomNDepthGetter		= null;// TODO: dGeomCCylinderPointDepth
			//max_collisionContact    = 3;
			break;

		case dConvexClass:
			geomRayNCollider		= new DxConvex.CollideRayConvex();//dCollideRayConvex;
			geomNPlaneCollider  	= new DxConvex.CollideConvexPlane();//dCollideConvexPlane;
			geomNDepthGetter		= null;// TODO: dGeomConvexPointDepth;
			//max_collisionContact    = 3;
			break;

			//	#if dTRIMESH_ENABLED

		case dTriMeshClass:
			geomRayNCollider		= new CollideRayTrimesh();//dCollideRayTrimesh;
			geomNPlaneCollider	    = new CollideTrimeshPlane();//dCollideTrimeshPlane;
			geomNDepthGetter		= null;// TODO: dGeomTrimeshPointDepth;
			//max_collisionContact    = 3;
			break;

			//	#endif // dTRIMESH_ENABLED

		default:
			dIASSERT(false);	// Shouldn't ever get here.
		break;

		}

		DxPlane myplane = new DxPlane(null,0,0,0,0);
		DxPlane sliding_plane = myplane;
		//double[] triplane = new double[4];
		DVector3 triplaneV = new DVector3();
		double triplaneD = 0;
		//int i;

		// check some trivial case.
		// Vector Up plane
		if (maxY - minY < dEpsilon)
		{
			// it's a single plane.
			//			triplane[0] = 0;
			//			triplane[1] = 1;
			//			triplane[2] = 0;
			//			triplane[3] =  minY;
			triplaneV.set(0, 1, 0);
			triplaneD = minY;
			dGeomPlaneSetNoNormalize (sliding_plane, triplaneV, triplaneD);
			// find collision and compute contact points
			if (skip != 1) throw new IllegalArgumentException("SKIP="+skip); //TZ
			//final int numTerrainContacts = geomNPlaneCollider.dColliderFn(o2, sliding_plane, flags, contacts, skip);
			final int numTerrainContacts = geomNPlaneCollider.dColliderFn(o2, sliding_plane, flags, contacts);
			dIASSERT(numTerrainContacts <= numMaxContactsPossible);
			for (int i = 0; i < numTerrainContacts; i++)
			{
				pContact = contacts.get(i*skip);//CONTACT(contact, i*skip);
				//dOPESIGN(pContact.normal, =, -, triplane);
				pContact.normal.set(triplaneV).scale(-1);
			}
			return numTerrainContacts;
		}

		/* -- This block is invalid as per Martijn Buijs <buijs512@planet.nl>

	    The problem seems to be based on the erroneously assumption that if two of
	    the four vertices of a 'grid' are at the same height, the entire grid can be
	    represented as a single plane. It works for an axis aligned slope, but fails
	    on all 4 grids of a 3x3 spike feature. Since the plane normal is constructed
	    from only 3 vertices (only one of the two triangles) this often results in
	    discontinuities at the grid edges (causing small jumps when the contact
	    point moves from one grid to another).

	    // unique plane
	    {
	        // check for very simple plane heightfield
	        dReal minXHeightDelta = dInfinity, maxXHeightDelta = - dInfinity;
	        dReal minZHeightDelta = dInfinity, maxZHeightDelta = - dInfinity;


	        dReal lastXHeight = tempHeightBuffer[0][0].vertex[1];
	        for ( x_local = 1; x_local < numX; x_local++)
	        {
	            HeightFieldVertex *HeightFieldRow = tempHeightBuffer[x_local];

	            const dReal deltaX = HeightFieldRow[0].vertex[1] - lastXHeight;

	            maxXHeightDelta = dMAX (maxXHeightDelta,  deltaX);
	            minXHeightDelta = dMIN (minXHeightDelta,  deltaX);

	            dReal lastZHeight = HeightFieldRow[0].vertex[1];
	            for ( z_local = 1; z_local < numZ; z_local++)
	            {
	                const dReal deltaZ = (HeightFieldRow[z_local].vertex[1] - lastZHeight);

	                maxZHeightDelta = dMAX (maxZHeightDelta,  deltaZ);
	                minZHeightDelta = dMIN (minZHeightDelta,  deltaZ);

	            }
	        }

	        if (maxZHeightDelta - minZHeightDelta < dEpsilon &&
	            maxXHeightDelta - minXHeightDelta < dEpsilon )
	        {
	            // it's a single plane.
	            const dVector3 &A = tempHeightBuffer[0][0].vertex;
	            const dVector3 &B = tempHeightBuffer[1][0].vertex;
	            const dVector3 &C = tempHeightBuffer[0][1].vertex;

	            // define 2 edges and a point that will define collision plane
	            {
	                dVector3 Edge1, Edge2;
	                dVector3Subtract(C, A, Edge1);
	                dVector3Subtract(B, A, Edge2);
	                dVector3Cross(Edge1, Edge2, triplane);
	            }

	            // Define Plane
	            // Normalize plane normal
	            const dReal dinvlength = REAL(1.0) / dVector3Length(triplane);
	            triplane[0] *= dinvlength;
	            triplane[1] *= dinvlength;
	            triplane[2] *= dinvlength;
	            // get distance to origin from plane
	            triplane[3] = dVector3Dot(triplane, A);

	            dGeomPlaneSetNoNormalize (sliding_plane, triplane);
	            // find collision and compute contact points
	            const int numTerrainContacts = geomNPlaneCollider (o2, sliding_plane, flags, contact, skip);
				dIASSERT(numTerrainContacts <= numMaxContactsPossible);
	            for (i = 0; i < numTerrainContacts; i++)
	            {
					pContact = CONTACT(contact, i*skip);
	                dOPESIGN(pContact->normal, =, -, triplane);
	            }
	            return numTerrainContacts;
	        }
	    }
		 */

		int numTerrainContacts = 0;
		//dContactGeom *PlaneContact = m_p_data.m_contacts;
		DContactGeomBuffer PlaneContact = m_p_data.m_contacts;

		//final unsigned 
		final int numTriMax = (maxX - minX) * (maxZ - minZ) * 2;
		if (tempTriangleBufferSize < numTriMax)
		{
			resetTriangleBuffer();
			allocateTriangleBuffer(numTriMax);
		}

		// Sorting triangle/plane  resulting from heightfield zone
		// Perhaps that would be necessary in case of too much limited
		// maximum contact point...
		// or in complex mesh case (trimesh and convex)
		// need some test or insights on this before enabling this.
		final boolean isContactNumPointsLimited =
			true;
		// (numMaxContacts < 8)
		//    || o2->type == dConvexClass
		//    || o2->type == dTriMeshClass
		//    || (numMaxContacts < (int)numTriMax)



		// if small heightfield triangle related to O2 colliding
		// or no Triangle colliding at all.
		boolean needFurtherPasses = (o2.type == dTriMeshClass);
		//compute Ratio between Triangle size and O2 aabb size
		// no FurtherPasses are needed in ray class
		if (o2.type != dRayClass  && needFurtherPasses == false)
		{
			final double xratio = (o2._aabb.getMax0() - o2._aabb.getMin0()) * m_p_data.m_fInvSampleWidth;
			if (xratio > (1.5))
				needFurtherPasses = true;
			else
			{
				final double zratio = (o2._aabb.getMax2() - o2._aabb.getMin2()) * m_p_data.m_fInvSampleDepth;
				if (zratio > (1.5))
					needFurtherPasses = true;
			}

		}

		//unsigned 
		int numTri = 0;
		HeightFieldVertex A, B, C, D;
		/*    (y is up)
	         A--------B-...x
	         |       /|
	         |      / |
	         |     /  |
	         |    /   |
	         |   /    |
	         |  /     |
	         | /      |
	         |/       |
	         C--------D
	         .
	         .
	         .
	         z
		 */
		// keep only triangle that does intersect geom

		//final unsigned 
		final int maxX_local = maxX - minX;
		//final unsigned 
		final int maxZ_local = maxZ - minZ;

		for ( x_local = 0; x_local < maxX_local; x_local++)
		{
			//			HeightFieldVertex HeightFieldRow      = tempHeightBuffer[x_local];
			//			HeightFieldVertex HeightFieldNextRow  = tempHeightBuffer[x_local + 1];
//			int posHeightFieldRow      = x_local;
//			int posHeightFieldNextRow  = x_local + 1;
			ObjArray<HeightFieldVertex> HeightFieldRow      = tempHeightBuffer[x_local];
			ObjArray<HeightFieldVertex> HeightFieldNextRow  = tempHeightBuffer[x_local + 1];

			// First A
	        //C = &HeightFieldRow    [0];
			//C = tempHeightBuffer[posHeightFieldRow];//    [0];
			C = HeightFieldRow.at(0);
			// First B
			//D = &HeightFieldNextRow[0];
			//D = tempHeightBuffer[posHeightFieldNextRow];//[0];
			D = HeightFieldNextRow.at(0);

			for ( z_local = 0; z_local < maxZ_local; z_local++)
			{
				A = C;
				B = D;

//	            C = &HeightFieldRow    [z_local + 1];
//	            D = &HeightFieldNextRow[z_local + 1];
//				C = tempHeightBuffer[posHeightFieldRow  + z_local + 1];//  [z_local + 1];
//				D = tempHeightBuffer[posHeightFieldNextRow + z_local + 1];//[z_local + 1];
				C = HeightFieldRow.at(z_local + 1);//  [z_local + 1];
				D = HeightFieldNextRow.at(z_local + 1);//[z_local + 1];

				final double AHeight = A.vertex.get1();
				final double BHeight = B.vertex.get1();
				final double CHeight = C.vertex.get1();
				final double DHeight = D.vertex.get1();

				final boolean isACollide = AHeight > minO2Height;
				final boolean isBCollide = BHeight > minO2Height;
				final boolean isCCollide = CHeight > minO2Height;
				final boolean isDCollide = DHeight > minO2Height;

				A.state = !(isACollide);
				B.state = !(isBCollide);
				C.state = !(isCCollide);
				D.state = !(isDCollide);

				if (isACollide || isBCollide || isCCollide)
				{
					HeightFieldTriangle CurrTriUp = tempTriangleBuffer[numTri++];// final ?? TZ

					CurrTriUp.state = false;

					// changing point order here implies to change it in isOnHeightField
					CurrTriUp.vertices[0] = A;
					CurrTriUp.vertices[1] = B;
					CurrTriUp.vertices[2] = C;

					if (isContactNumPointsLimited)
						CurrTriUp.setMinMax();
					CurrTriUp.isUp = true;
				}

				if (isBCollide || isCCollide || isDCollide)
				{
					HeightFieldTriangle CurrTriDown = tempTriangleBuffer[numTri++];//final ?? TZ

					CurrTriDown.state = false;
					// changing point order here implies to change it in isOnHeightField

					CurrTriDown.vertices[0] = D;
					CurrTriDown.vertices[1] = B;
					CurrTriDown.vertices[2] = C;


					if (isContactNumPointsLimited)
						CurrTriDown.setMinMax();
					CurrTriDown.isUp = false;
				}


				if (needFurtherPasses &&
						(isBCollide || isCCollide)
						&&
						(AHeight > CHeight &&
						AHeight > BHeight &&
						DHeight > CHeight &&
						DHeight > BHeight ))
				{
					// That means Edge BC is concave, therefore
					// BC Edge and B and C vertices cannot collide

					B.state = true;
					C.state = true;
				}
				// should find a way to check other edges (AB, BD, CD) too for concavity
			}
		}

		// at least on triangle should intersect geom
		dIASSERT (numTri != 0);
		// pass1: VS triangle as Planes
		// Group Triangle by same plane definition
		// as Terrain often has many triangles using same plane definition
		// then collide against that list of triangles.
		{

			DVector3 Edge1 = new DVector3(), Edge2 = new DVector3();
			//compute all triangles normals.
			for (int k = 0; k < numTri; k++)
			{
				HeightFieldTriangle itTriangle = tempTriangleBuffer[k];  // final? TZ

				// define 2 edges and a point that will define collision plane
				//dVector3Subtract(itTriangle.vertices[2].vertex, itTriangle.vertices[0].vertex, Edge1);
				Edge1.eqDiff(itTriangle.vertices[2].vertex, itTriangle.vertices[0].vertex);
				//dVector3Subtract(itTriangle.vertices[1].vertex, itTriangle.vertices[0].vertex, Edge2);
				Edge2.eqDiff(itTriangle.vertices[1].vertex, itTriangle.vertices[0].vertex);

				// find a perpendicular vector to the triangle
				if  (itTriangle.isUp)
					DxCollisionUtil.dVector3Cross(Edge1, Edge2, triplaneV);
				else
					DxCollisionUtil.dVector3Cross(Edge2, Edge1, triplaneV);

				// Define Plane
				// Normalize plane normal
				final double dinvlength = 1.0 / triplaneV.length();//CollisionUtil.dVector3Length(triplane);
				//				triplane[0] *= dinvlength;
				//				triplane[1] *= dinvlength;
				//				triplane[2] *= dinvlength;
				triplaneV.scale(dinvlength);
				// get distance to origin from plane
				//triplane[3] = dVector3Dot(triplane, itTriangle.vertices[0].vertex);
				triplaneD = triplaneV.dot(itTriangle.vertices[0].vertex);

				// saves normal for collision check (planes, triangles, vertices and edges.)
				//dVector3Copy(triplane, itTriangle.planeDef);
				itTriangle.planeDefV.set( triplaneV );
				// saves distance for collision check (planes, triangles, vertices and edges.)
				itTriangle.planeDefD = triplaneD;//[3];
			}

			// group by Triangles by Planes sharing shame plane definition
			if (tempPlaneBufferSize  < numTri)
			{
				resetPlaneBuffer();
				allocatePlaneBuffer(numTri);
			}
			//unsigned 
			int numPlanes = 0;
			for (int k = 0; k < numTri; k++)
			{
				HeightFieldTriangle tri_base = tempTriangleBuffer[k];  // final ? TZ

				if (tri_base.state == true)
					continue;// already tested or added to plane list.

				//HeightFieldPlane * const currPlane = tempPlaneBuffer[numPlanes];
				HeightFieldPlane currPlane = tempPlaneBuffer[numPlanes].at0();// final ? TZ
				currPlane.resetTriangleListSize(numTri - k);
				currPlane.addTriangle(tri_base);
				// saves normal for collision check (planes, triangles, vertices and edges.)
				//dVector3Copy(tri_base.planeDef, currPlane.planeDef);
				currPlane.planeDefV.set(tri_base.planeDefV);
				// saves distance for collision check (planes, triangles, vertices and edges.)
				currPlane.planeDefD= tri_base.planeDefD;

				final double normx = tri_base.planeDefV.get0();
				final double normy = tri_base.planeDefV.get1();
				final double normz = tri_base.planeDefV.get2();
				final double dist = tri_base.planeDefD;

				for (int m = k + 1; m < numTri; m++)
				{

					HeightFieldTriangle tri_test = tempTriangleBuffer[m];  // final? TZ
					if (tri_test.state == true)
						continue;// already tested or added to plane list.

					// normals and distance are the same.
					if (
							dFabs(normy - tri_test.planeDefV.get1()) < dEpsilon &&
							dFabs(dist  - tri_test.planeDefD) < dEpsilon &&
							dFabs(normx - tri_test.planeDefV.get0()) < dEpsilon &&
							dFabs(normz - tri_test.planeDefV.get2()) < dEpsilon
					)
					{
						currPlane.addTriangle (tri_test);
						tri_test.state = true;
					}
				}

				tri_base.state = true;
				if (isContactNumPointsLimited)
					currPlane.setMinMax();

				numPlanes++;
			}

			// sort planes
			if (isContactNumPointsLimited)
				sortPlanes(numPlanes);

			int numMaxContactsPerPlane;
			int planeTestFlags;
			if (!NO_CONTACT_CULLING_BY_ISONHEIGHTFIELD2) {//#if !defined(NO_CONTACT_CULLING_BY_ISONHEIGHTFIELD2)
				/*
				Note by Oleh_Derevenko:
				It seems to be incorrect to limit contact count by some particular value
				since some of them (and even all of them) may be culled in following condition.
				However I do not see an easy way to fix this.
				If not that culling the flags modification should be changed here and
				additionally repeated after some contacts have been generated (in "if (didCollide)").
				The maximum of contacts in flags would then be set to minimum of contacts
				remaining and HEIGHTFIELDMAXCONTACTPERCELL.
				 */
				planeTestFlags = (flags & ~NUMC_MASK) | HEIGHTFIELDMAXCONTACTPERCELL;
				//dIASSERT((HEIGHTFIELDMAXCONTACTPERCELL & ~NUMC_MASK) == 0);
			} else {//#else // if defined(NO_CONTACT_CULLING_BY_ISONHEIGHTFIELD2)
				//				int numMaxContactsPerPlane = dMIN(numMaxContactsPossible - numTerrainContacts, HEIGHTFIELDMAXCONTACTPERCELL);
				//				int planeTestFlags = (flags & ~NUMC_MASK) | numMaxContactsPerPlane;
				numMaxContactsPerPlane = (int) dMIN(numMaxContactsPossible - numTerrainContacts, HEIGHTFIELDMAXCONTACTPERCELL);
				planeTestFlags = (flags & ~NUMC_MASK) | numMaxContactsPerPlane;
				//dIASSERT((HEIGHTFIELDMAXCONTACTPERCELL & ~NUMC_MASK) == 0);
			}//#endif

			for (int k = 0; k < numPlanes; k++)
			{
	            //HeightFieldPlane * const itPlane = tempPlaneBuffer[k];
				HeightFieldPlane itPlane = tempPlaneBuffer[k].at0();//final TZ

				//set Geom
				dGeomPlaneSetNoNormalize (sliding_plane,  itPlane.planeDefV, itPlane.planeDefD);
				//dGeomPlaneSetParams (sliding_plane, triangle_Plane[0], triangle_Plane[1], triangle_Plane[2], triangle_Plane[3]);
				// find collision and compute contact points
				boolean didCollide = false;
				final int numPlaneContacts = 
					geomNPlaneCollider.dColliderFn(o2, sliding_plane, planeTestFlags, PlaneContact);//, sizeof(dContactGeom));
				final int planeTriListSize = itPlane.trianglelistCurrentSize;
				for (int i = 0; i < numPlaneContacts; i++)
				{
					DContactGeom planeCurrContact = PlaneContact.get(i);
					// Check if contact point found in plane is inside Triangle.
					final DVector3C pCPos = planeCurrContact.pos;
					for (int b = 0; planeTriListSize > b; b++)
					{
						if (m_p_data.IsOnHeightfield2 (itPlane.trianglelist[b].vertices[0],
								pCPos,
								itPlane.trianglelist[b].isUp))
						{
							pContact = contacts.get(numTerrainContacts*skip);//CONTACT(contact, numTerrainContacts*skip);
							pContact.pos.set(pCPos);//dVector3Copy(pCPos, pContact.pos);
							//dOPESIGN(pContact.normal, =, -, itPlane.planeDef);
							pContact.normal.set(itPlane.planeDefV).scale(-1);
							pContact.depth = planeCurrContact.depth;
							pContact.side1 = planeCurrContact.side1;
							pContact.side2 = planeCurrContact.side2;
							numTerrainContacts++;
							if ( numTerrainContacts == numMaxContactsPossible )
								return numTerrainContacts;

							didCollide = true;
							break;
						}
					}
				}
				if (didCollide)
				{
					if (NO_CONTACT_CULLING_BY_ISONHEIGHTFIELD2) {//#if defined(NO_CONTACT_CULLING_BY_ISONHEIGHTFIELD2)
						/* Note by Oleh_Derevenko:
						This code is not used - see another note above
						 */
						numMaxContactsPerPlane = (int) dMIN(numMaxContactsPossible - numTerrainContacts, HEIGHTFIELDMAXCONTACTPERCELL);
						planeTestFlags = (flags & ~NUMC_MASK) | numMaxContactsPerPlane;
						//dIASSERT((HEIGHTFIELDMAXCONTACTPERCELL & ~NUMC_MASK) == 0);
					}//#endif
					for (int b = 0; planeTriListSize > b; b++)
					{
						// flag Triangles Vertices as collided
						// to prevent any collision test of those
						for (int i = 0; i < 3; i++)
							itPlane.trianglelist[b].vertices[i].state = true;
					}
				}
				else
				{
					// flag triangle as not collided so that Vertices or Edge
					// of that triangles will be checked.
					for (int b = 0; planeTriListSize > b; b++)
					{
						itPlane.trianglelist[b].state = false;
					}
				}
			}
		}



		// pass2: VS triangle vertices
		if (needFurtherPasses)
		{
			DxRay tempRay = new DxRay(null, 1);
			double depth = 0;
			boolean vertexCollided;

			// Only one contact is necessary for ray test
			int rayTestFlags = (flags & ~NUMC_MASK) | 1;
			//dIASSERT((1 & ~NUMC_MASK) == 0);
			//
			// Find Contact Penetration Depth of each vertices
			//
			for (int k = 0; k < numTri; k++)
			{
				//final HeightFieldTriangle * final itTriangle = &tempTriangleBuffer[k];
				HeightFieldTriangle itTriangle = tempTriangleBuffer[k];  // final ? TZ
				if (itTriangle.state == true)
					continue;// plane triangle did already collide.

				for (int i = 0; i < 3; i++)
				{
					//HeightFieldVertex *vertex = itTriangle.vertices[i];
					HeightFieldVertex vertex = itTriangle.vertices[i];
					if (vertex.state == true)
						continue;// vertice did already collide.

					vertexCollided = false;
					DVector3C triVertex = vertex.vertex;
					if ( geomNDepthGetter!=null )
					{
						depth = geomNDepthGetter.get( o2,
								triVertex.get0(), triVertex.get1(), triVertex.get2() );
						if (depth > dEpsilon)
							vertexCollided = true;
					}
					else
					{
						// We don't have a GetDepth function, so do a ray cast instead.
						// NOTE: This isn't ideal, and a GetDepth function should be
						// written for all geom classes.
						tempRay.setLength( (minO2Height - triVertex.get1()) * (1000.0) );

						//dGeomRaySet( &tempRay, pContact->pos[0], pContact->pos[1], pContact->pos[2],
						//    - itTriangle->Normal[0], - itTriangle->Normal[1], - itTriangle->Normal[2] );
						dGeomRaySetNoNormalize(tempRay, triVertex, itTriangle.planeDefV);

						if ( geomRayNCollider.dColliderFn( tempRay, o2, rayTestFlags, PlaneContact) != 0)//, sizeof( dContactGeom ) ) )
						{
							depth = PlaneContact.get(0).depth;
							vertexCollided = true;
						}
					}
					if (vertexCollided)
					{
						pContact = contacts.get(numTerrainContacts*skip);//CONTACT(contact, numTerrainContacts*skip);
						//create contact using vertices
						//dVector3Copy (triVertex, pContact.pos);
						pContact.pos.set(triVertex);
						//create contact using Plane Normal
						//dOPESIGN(pContact.normal, =, -, itTriangle.planeDef);
						pContact.normal.set(itTriangle.planeDefV).scale(-1);

						pContact.depth = depth;
						pContact.side1 = -1;
						pContact.side2 = -1;

						numTerrainContacts++;
						if ( numTerrainContacts == numMaxContactsPossible )
							return numTerrainContacts;

						vertex.state = true;
					}
				}
			}
		}

		//	#ifdef _HEIGHTFIELDEDGECOLLIDING
		//	    // pass3: VS triangle Edges
		//	    if (needFurtherPasses)
		//	    {
		//	        dVector3 Edge;
		//	        dxRay edgeRay(0, 1);
		//
		//			int numMaxContactsPerTri = dMIN(numMaxContactsPossible - numTerrainContacts, HEIGHTFIELDMAXCONTACTPERCELL);
		//			int triTestFlags = (flags & ~NUMC_MASK) | numMaxContactsPerTri;
		//			dIASSERT((HEIGHTFIELDMAXCONTACTPERCELL & ~NUMC_MASK) == 0);
		//
		//	        for (unsigned int k = 0; k < numTri; k++)
		//	        {
		//	            final HeightFieldTriangle * final itTriangle = &tempTriangleBuffer[k];
		//
		//	            if (itTriangle.state == true)
		//	                continue;// plane did already collide.
		//
		//	            for (size_t m = 0; m < 3; m++)
		//	            {
		//	                final size_t next = (m + 1) % 3;
		//	                HeightFieldVertex *vertex0 = itTriangle.vertices[m];
		//	                HeightFieldVertex *vertex1 = itTriangle.vertices[next];
		//
		//	                // not concave or under the AABB
		//	                // nor triangle already collided against vertices
		//	                if (vertex0.state == true && vertex1.state == true)
		//	                    continue;// plane did already collide.
		//
		//	                dVector3Subtract(vertex1.vertex, vertex0.vertex, Edge);
		//	                edgeRay.length = dVector3Length (Edge);
		//	                dGeomRaySetNoNormalize(edgeRay, vertex1.vertex, Edge);
		//					int prevTerrainContacts = numTerrainContacts;
		//					pContact = CONTACT(contact, prevTerrainContacts*skip);
		//	                final int numCollision = geomRayNCollider(&edgeRay,o2,triTestFlags,pContact,skip);
		//					dIASSERT(numCollision <= numMaxContactsPerTri);
		//
		//					if (numCollision)
		//					{
		//						numTerrainContacts += numCollision;
		//
		//						do
		//						{
		//							pContact = CONTACT(contact, prevTerrainContacts*skip);
		//
		//							//create contact using Plane Normal
		//							dOPESIGN(pContact.normal, =, -, itTriangle.planeDef);
		//
		//							pContact.depth = DistancePointToLine(pContact.pos, vertex1.vertex, Edge, edgeRay.length);
		//						}
		//						while (++prevTerrainContacts != numTerrainContacts);
		//
		//						if ( numTerrainContacts == numMaxContactsPossible )
		//							return numTerrainContacts;
		//
		//						numMaxContactsPerTri = dMIN(numMaxContactsPossible - numTerrainContacts, HEIGHTFIELDMAXCONTACTPERCELL);
		//						triTestFlags = (flags & ~NUMC_MASK) | numMaxContactsPerTri;
		//						dIASSERT((HEIGHTFIELDMAXCONTACTPERCELL & ~NUMC_MASK) == 0);
		//					}
		//	            }
		//
		//	            itTriangle.vertices[0].state = true;
		//	            itTriangle.vertices[1].state = true;
		//	            itTriangle.vertices[2].state = true;
		//	        }
		//	    }
		//	#endif // _HEIGHTFIELDEDGECOLLIDING
		return numTerrainContacts;
	}

	static class CollideHeightfield implements DColliderFn {
		int dCollideHeightfield( DxAbstractHeightfield o1, DxGeom o2, int flags, DContactGeomBuffer contacts, int skip )
		{
			dIASSERT( skip >= 1);//(int)sizeof(dContactGeom) );
			//dIASSERT( o1.type == dHeightfieldClass );
			dIASSERT((flags & NUMC_MASK) >= 1);

			int i;

			// if ((flags & NUMC_MASK) == 0) -- An assertion check is made on entry
			//	{ flags = (flags & ~NUMC_MASK) | 1; dIASSERT((1 & ~NUMC_MASK) == 0); }

			int numMaxTerrainContacts = (flags & NUMC_MASK);

			DxAbstractHeightfield terrain = o1;

			DVector3 posbak = new DVector3();
			DMatrix3 Rbak = new DMatrix3();
			DAABB aabbbak = new DAABB();
			int gflagsbak = 0;
			DVector3 pos0 = new DVector3(), pos1 = new DVector3();
			DMatrix3 R1 = new DMatrix3();

			int numTerrainContacts = 0;
			int numTerrainOrigContacts = 0;

			//@@ Should find a way to set reComputeAABB to false in default case
			// aka DHEIGHTFIELD_CORNER_ORIGIN not defined and terrain not PLACEABLE
			// so that we can free some memory and speed up things a bit
			// while saving some precision loss
			boolean reComputeAABB;
			if (!DHEIGHTFIELD_CORNER_ORIGIN) {//#ifndef DHEIGHTFIELD_CORNER_ORIGIN
				//final boolean reComputeAABB = true;
				reComputeAABB = true;
			} else {//#else
				//final boolean reComputeAABB = ( (terrain._gflags & GEOM_PLACEABLE)!=0 ) ? true : false;
				//reComputeAABB = ( (terrain._gflags & GEOM_PLACEABLE)!=0 ) ? true : false;
				reComputeAABB = terrain.hasFlagPlaceable();
			}//#endif //DHEIGHTFIELD_CORNER_ORIGIN

			//
			// Transform O2 into Heightfield Space
			//
			if (reComputeAABB)
			{
				// Backup original o2 position, rotation and AABB.
				posbak.set(o2.final_posr().pos());//dVector3Copy( o2._final_posr.pos, posbak );
				Rbak.set(o2.final_posr().R());//dMatrix3Copy( o2._final_posr.R, Rbak );
				aabbbak.set(o2._aabb);//memcpy( aabbbak, o2.aabb, sizeof( double ) * 6 );
				gflagsbak = o2.getFlags();//_gflags;
			}

			//if ( (terrain._gflags & GEOM_PLACEABLE)!=0 )
			if (terrain.hasFlagPlaceable())
			{
				// Transform o2 into heightfield space.
				//dOP( pos0, OP.SUB, o2._final_posr.pos, terrain._final_posr.pos );
				pos0.eqDiff( o2.final_posr().pos(), terrain.final_posr().pos() );
				dMultiply1_331( pos1, terrain.final_posr().R(), pos0 );
				dMultiply1_333( R1, terrain.final_posr().R(), o2.final_posr().R() );

				// Update o2 with transformed position and rotation.
				o2._final_posr.pos.set(pos1);//dVector3Copy( pos1, o2._final_posr.pos );
				o2._final_posr.Rw().set(R1);//dMatrix3Copy( R1, o2._final_posr.R );
			}

			if (!DHEIGHTFIELD_CORNER_ORIGIN) {//#ifndef DHEIGHTFIELD_CORNER_ORIGIN
				o2._final_posr.pos.add( 0, +terrain.m_p_data.m_fHalfWidth );
				o2._final_posr.pos.add( 2, +terrain.m_p_data.m_fHalfDepth );
			}////#endif // DHEIGHTFIELD_CORNER_ORIGIN

			// Rebuild AABB for O2
			if (reComputeAABB) {
				//o2.computePosr();
				o2.computeAABB();
			}

			//
			// Collide
			//

			//check if inside boundaries
			// using O2 aabb
			//  aabb[6] is (minx, maxx, miny, maxy, minz, maxz)
			final boolean wrapped = terrain.m_p_data.m_bWrapMode != false;

			//TZ
			boolean dCollideHeightfieldExit = false;

			if ( !wrapped )
			{
				if (    o2._aabb.getMin0() > terrain.m_p_data.m_fWidth //MinX
						||  o2._aabb.getMin2() > terrain.m_p_data.m_fDepth) { //MinZ
					//goto dCollideHeightfieldExit;
					dCollideHeightfieldExit = true;
				}

				if (    o2._aabb.getMax0() < 0 //MaxX
						||  o2._aabb.getMax2() < 0) { //MaxZ
					//goto dCollideHeightfieldExit;
					dCollideHeightfieldExit = true;
				}

			}

			DContactGeom pContact;
			if (!dCollideHeightfieldExit) {
			    {
			        // To narrow scope of following variables
			        final double fInvSampleWidth = terrain.m_p_data.m_fInvSampleWidth;
			        int nMinX = (int)dFloor(Common.dNextAfter(o2._aabb.getMin0() * fInvSampleWidth, -dInfinity));
			        int nMaxX = (int)dCeil(Common.dNextAfter(o2._aabb.getMax0() * fInvSampleWidth, dInfinity));
			        final double fInvSampleDepth = terrain.m_p_data.m_fInvSampleDepth;
			        int nMinZ = (int)dFloor(Common.dNextAfter(o2._aabb.getMin2() * fInvSampleDepth, -dInfinity));
			        int nMaxZ = (int)dCeil(Common.dNextAfter(o2._aabb.getMax2() * fInvSampleDepth, dInfinity));

			        if ( !wrapped )
			        {
			            nMinX = (int) dMAX( nMinX, 0 );
			            nMaxX = (int) dMIN( nMaxX, terrain.m_p_data.m_nWidthSamples - 1 );
			            nMinZ = (int) dMAX( nMinZ, 0 );
			            nMaxZ = (int) dMIN( nMaxZ, terrain.m_p_data.m_nDepthSamples - 1 );

			            dIASSERT ((nMinX < nMaxX) && (nMinZ < nMaxZ));
			        }


			        numTerrainOrigContacts = numTerrainContacts;
			        numTerrainContacts += terrain.dCollideHeightfieldZone(
			                nMinX,nMaxX,nMinZ,nMaxZ,o2,numMaxTerrainContacts - numTerrainContacts,
			                flags,
			                //CONTACT(contact,numTerrainContacts*skip),
			                contacts.createView(numTerrainContacts*skip),
			                skip );
			        dIASSERT( numTerrainContacts <= numMaxTerrainContacts );
			    }
			    
				for ( i = numTerrainOrigContacts; i != numTerrainContacts; ++i )
				{
					pContact = contacts.get(i*skip);//CONTACT(contact,i*skip);
					pContact.g1 = o1;
					pContact.g2 = o2;
					// pContact->side1 = -1; -- Oleh_Derevenko: sides must not
					// be erased here as they are set by respective colliders during ray/plane tests 
					// pContact->side2 = -1;
				}
			}

			//------------------------------------------------------------------------------

			//dCollideHeightfieldExit:
			//if (dCollideHeightfieldExit)
				if (reComputeAABB)
				{
					// Restore o2 position, rotation and AABB
					//dVector3Copy( posbak, o2._final_posr.pos );
					o2._final_posr.pos.set(posbak);
					//dMatrix3Copy( Rbak, o2._final_posr.R );
					o2._final_posr.Rw().set(Rbak);
					//memcpy( o2.aabb, aabbbak, sizeof(double)*6 );
					o2._aabb.set(aabbbak);
					o2.setFlags(gflagsbak);//_gflags = gflagsbak;
					if (o2 instanceof DxGimpact)
						o2.computeAABB();  //TODO TZ

					//
					// Transform Contacts to World Space
					//
					//if ( (terrain._gflags & GEOM_PLACEABLE)!=0 )
					if ( terrain.hasFlagPlaceable() )
					{
						for ( i = 0; i < numTerrainContacts; ++i )
						{
							pContact = contacts.get(i*skip);//CONTACT(contact,i*skip);
							//dOPE( pos0, =, pContact.pos );
							pos0.set( pContact.pos );

							if (!DHEIGHTFIELD_CORNER_ORIGIN) {//#ifndef DHEIGHTFIELD_CORNER_ORIGIN
								pos0.add( 0, -terrain.m_p_data.m_fHalfWidth );
								pos0.add( 2, -terrain.m_p_data.m_fHalfDepth );
							}//#endif // !DHEIGHTFIELD_CORNER_ORIGIN

							dMultiply0_331( pContact.pos, terrain.final_posr().R(), pos0 );

							//dOP( pContact.pos, +, pContact.pos, terrain._final_posr.pos );
							pContact.pos.add(terrain.final_posr().pos());
							//dOPE( pos0, =, pContact.normal );
							pos0.set(pContact.normal);

							dMultiply0_331( pContact.normal, terrain.final_posr().R(), pos0 );
						}
					} 
					else
					{
						if (!DHEIGHTFIELD_CORNER_ORIGIN) {//#ifndef DHEIGHTFIELD_CORNER_ORIGIN
							for ( i = 0; i < numTerrainContacts; ++i )
							{
								pContact = contacts.get(i*skip);//CONTACT(contact,i*skip);
								pContact.pos.add( 0, -terrain.m_p_data.m_fHalfWidth );
								pContact.pos.add( 2, -terrain.m_p_data.m_fHalfDepth );
							}
						}//#endif // !DHEIGHTFIELD_CORNER_ORIGIN
					}
				}
			// Return contact count.
			return numTerrainContacts;
		}

		@Override
		public int dColliderFn(DGeom o1, DGeom o2, int flags,
				DContactGeomBuffer contacts) {
			return dCollideHeightfield((DxAbstractHeightfield)o1, (DxGeom)o2, flags, contacts, 1);
		}
	}


	@Override
	public DHeightfieldData getHeightfieldData() {
		return dGeomHeightfieldGetHeightfieldData();
	}
	
	
	@Override
	public void setHeightfieldData(DHeightfieldData d) {
		dGeomHeightfieldSetHeightfieldData(d);
	}
}
