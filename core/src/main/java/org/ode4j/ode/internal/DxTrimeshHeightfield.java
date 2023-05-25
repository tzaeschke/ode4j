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
import static org.ode4j.ode.internal.Common.dFloor;
import static org.ode4j.ode.internal.Common.dIASSERT;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import org.ode4j.ode.DAABB;
import org.ode4j.ode.DColliderFn;
import org.ode4j.ode.DContactGeom;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DHeightfieldData;
import org.ode4j.ode.DSpace;

/**
 *
 * ode4j enhances the functionality of the gimpact-based heightfield geom by adding simple support for holes and
 * multiple layers. There is a corresponding demo showing 2 overlapping heightfields that look somewhat like a
 * Swiss cheese. Moving bodies can fall through the holes and roll on both heightfields. The functionality is
 * not very interesting on its own, but it allows to build complex environments including tunnels or caves
 * (e.g. a pipe-like trimesh combined with a heightfield with 2 holes at the end of the tunnel) The code
 * purposely enforces the rectangular shape of the holes to make it compatible with most of the rendering
 * techniques. In order to keep the heightfield data API intact NaN value was chosen to denote a hole.
 */
public class DxTrimeshHeightfield extends DxAbstractHeightfield {

    //#define dMIN(A,B)  ((A)>(B) ? (B) : (A))
    private static double dMIN(double A, double B) { return (Math.min((A), (B))); }
    //#define dMAX(A,B)  ((A)>(B) ? (A) : (B))
    private static double dMAX(double A, double B) { return (Math.max((A), (B))); }
    //
    //
    ////Three-way MIN and MAX
    //#define dMIN3(A,B,C)  ( (A)<(B) ? dMIN((A),(C)) : dMIN((B),(C)) )
    private double dMIN3(double A, double B, double C) { return A<B ? dMIN(A,C) : dMIN(B,C); }
    //#define dMAX3(A,B,C)  ( (A)>(B) ? dMAX((A),(C)) : dMAX((B),(C)) )
    private double dMAX3(double A, double B, double C) { return A>B ? dMAX(A,C) : dMAX(B,C); }
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


    ////////dxHeightfield /////////////////////////////////////////////////////////////////

    //      dxHeightfield( dSpaceID space, dHeightfieldDataID data, int bPlaceable );
    //      ~dxHeightfield();
    //
    //      void computeAABB();
    //
    //      int dCollideHeightfieldZone( const int minX, const int maxX, const int minZ, const int maxZ,  
    //          dxGeom *o2, const int numMaxContacts,
    //          int flags, dContactGeom *contact, int skip );

    //      private enum TEMP
    //      {
    private static final int TEMP_HEIGHT_BUFFER_ELEMENT_COUNT_ALIGNMENT_X = 4;
    private static final int TEMP_HEIGHT_BUFFER_ELEMENT_COUNT_ALIGNMENT_Z = 4;
    private static final int TEMP_TRIANGLE_BUFFER_ELEMENT_COUNT_ALIGNMENT = 1; // Triangles are easy to reallocate and hard to predict
    //      };

    //static inline size_t AlignBufferSize(size_t value, size_t alignment) { dIASSERT((alignment & (alignment - 1)) == 0); return (value + (alignment - 1)) & ~(alignment - 1); }
    private static int AlignBufferSize(int value, int alignment) { 
        dIASSERT((alignment & (alignment - 1)) == 0); 
        return (value + (alignment - 1)) & ~(alignment - 1); 
    }


    private HeightFieldTriangle[] tempTriangleBuffer;
    private int              tempTriangleBufferSize;

    //private HeightFieldVertex[]  tempHeightBuffer;
    private int[]  tempHeightBuffer;
    private HeightFieldVertex[]   tempHeightInstances;
    private int              tempHeightBufferSizeX;
    private int              tempHeightBufferSizeZ;

    private final boolean	layered;

    public DxTrimeshHeightfield(DSpace space, DHeightfieldData data, boolean bPlaceable) {
        this(space, data, bPlaceable, false);
    }

    public DxTrimeshHeightfield(DSpace space, DHeightfieldData data, boolean bPlaceable, boolean layered) {
        //      dxGeom( space, bPlaceable ),
        //      tempPlaneBuffer(0),
        //      tempPlaneInstances(0),
        //      tempPlaneBufferSize(0),
        //      tempTriangleBuffer(0),
        //      tempTriangleBufferSize(0),
        //      tempHeightBuffer(0),
        //      tempHeightInstances(0),
        //      tempHeightBufferSizeX(0),
        //      tempHeightBufferSizeZ(0)
        super( (DxSpace) space, bPlaceable );
        tempTriangleBuffer = null;
        tempTriangleBufferSize = 0;
        tempHeightBuffer = null;
        tempHeightInstances = null;
        tempHeightBufferSizeX = 0;
        tempHeightBufferSizeZ = 0;

        type = dHeightfieldClass;
        m_p_data = (DxHeightfieldData) data;
        this.layered = layered;
    }


    // compute axis aligned bounding box
    @Override
    protected void computeAABB()
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


                //  #ifdef DHEIGHTFIELD_CORNER_ORIGIN
                //
                //              // X-axis
                //              dx[0] = 0;  dx[3] = ( _final_posr.R[ 0] * d.m_fWidth );
                //              dx[1] = 0;  dx[4] = ( _final_posr.R[ 4] * d.m_fWidth );
                //              dx[2] = 0;  dx[5] = ( _final_posr.R[ 8] * d.m_fWidth );
                //
                //              // Z-axis
                //              dz[0] = 0;  dz[3] = ( _final_posr.R[ 2] * d.m_fDepth );
                //              dz[1] = 0;  dz[4] = ( _final_posr.R[ 6] * d.m_fDepth );
                //              dz[2] = 0;  dz[5] = ( _final_posr.R[10] * d.m_fDepth );
                //
                //  #else // DHEIGHTFIELD_CORNER_ORIGIN

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

                //  #endif // DHEIGHTFIELD_CORNER_ORIGIN

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

                //  #ifdef DHEIGHTFIELD_CORNER_ORIGIN
                //
                //              aabb[0] = 0;                    aabb[1] = d.m_fWidth;
                //              aabb[2] = d.m_fMinHeight;       aabb[3] = d.m_fMaxHeight;
                //              aabb[4] = 0;                    aabb[5] = d.m_fDepth;
                //
                //  #else // DHEIGHTFIELD_CORNER_ORIGIN

                //              aabb[0] = -d.m_fHalfWidth;      aabb[1] = +d.m_fHalfWidth;
                //              aabb[2] = d.m_fMinHeight;       aabb[3] = d.m_fMaxHeight;
                //              aabb[4] = -d.m_fHalfDepth;      aabb[5] = +d.m_fHalfDepth;
                _aabb.set(-d.m_fHalfWidth, +d.m_fHalfWidth,
                        d.m_fMinHeight, d.m_fMaxHeight, 
                        -d.m_fHalfDepth, +d.m_fHalfDepth);

                //  #endif // DHEIGHTFIELD_CORNER_ORIGIN

            }
        }
        else
        {
            // Infinite
            //if ( (_gflags & GEOM_PLACEABLE)!=0 )
            if (hasFlagPlaceable())
            {
                //              aabb[0] = -dInfinity;           aabb[1] = +dInfinity;
                //              aabb[2] = -dInfinity;           aabb[3] = +dInfinity;
                //              aabb[4] = -dInfinity;           aabb[5] = +dInfinity;
                _aabb.set(-dInfinity, +dInfinity,
                        -dInfinity, +dInfinity,
                        -dInfinity, +dInfinity);

            }
            else
            {
                //              aabb[0] = -dInfinity;           aabb[1] = +dInfinity;
                //              aabb[2] = d.m_fMinHeight;       aabb[3] = d.m_fMaxHeight;
                //              aabb[4] = -dInfinity;           aabb[5] = +dInfinity;
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
        tempTriangleBufferSize = 0; // TZ just to be clean
    }

    private void allocateHeightBuffer(int numX, int numZ)
    {
        int alignedNumX = AlignBufferSize(numX, TEMP_HEIGHT_BUFFER_ELEMENT_COUNT_ALIGNMENT_X);
        int alignedNumZ = AlignBufferSize(numZ, TEMP_HEIGHT_BUFFER_ELEMENT_COUNT_ALIGNMENT_Z);
        tempHeightBufferSizeX = alignedNumX;
        tempHeightBufferSizeZ = alignedNumZ;
        //tempHeightBuffer = new HeightFieldVertex *[alignedNumX];
        tempHeightBuffer = new int[alignedNumX];
        int numCells = alignedNumX * alignedNumZ;
        tempHeightInstances = new HeightFieldVertex [numCells];
        for (int i = 0; i < tempHeightInstances.length; i++) {
            tempHeightInstances[i] = new HeightFieldVertex();
        }

        //HeightFieldVertex *ptrHeightMatrix = tempHeightInstances;
        for (int indexX = 0; indexX != alignedNumX; indexX++)
        {
            //          tempHeightBuffer[indexX] = ptrHeightMatrix;
            //          ptrHeightMatrix += alignedNumZ;
            //tempHeightBuffer[indexX] = tempHeightInstances[indexX];
            tempHeightBuffer[indexX] = indexX * alignedNumZ;
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
     * @param space space
     * @param data data
     * @param bPlaceable placeable flag 
     * @return New DHeightfield
     */
    public static DxTrimeshHeightfield dCreateHeightfield( DxSpace space, DxHeightfieldData data, boolean bPlaceable )
    {
        return new DxTrimeshHeightfield( space, data, bPlaceable );
    }

    public static DxTrimeshHeightfield dCreateHeightfield( DxSpace space, DxHeightfieldData data, boolean bPlaceable, boolean layered )
    {
        return new DxTrimeshHeightfield( space, data, bPlaceable, layered );
    }

    //  void dGeomHeightfieldSetHeightfieldData( dGeom g, dHeightfieldData d )
    void dGeomHeightfieldSetHeightfieldData( DHeightfieldData d )
    {
        //dxHeightfield* geom = (dxHeightfield*) g;
        //TODO change type of m_p_data?
        m_p_data = (DxHeightfieldData) d;
    }


    //  dHeightfieldData dGeomHeightfieldGetHeightfieldData( dGeom g )
    DHeightfieldData dGeomHeightfieldGetHeightfieldData( )
    {
        //dxHeightfield* geom = (dxHeightfield*) g;
        return m_p_data;
    }

    // from heightfield.h

    //typedef int HeightFieldVertexCoords[2];

    class HeightFieldVertex
    {
        //  public:
        // HeightFieldVertex(){};

        DVector3 vertex = new DVector3();
        //HeightFieldVertexCoords coords;
        //int[] coords = new int[2]; //use c1 & c2 (TZ)
        int coords0, coords1;
    }

    //TODO TZ not used
//  private class HeightFieldEdge
//  {
//      //public:
//      HeightFieldEdge(){};
//
//      //HeightFieldVertex   *vertices[2];
//      HeightFieldVertex[]   vertices = new HeightFieldVertex[2];
//      HeightFieldVertex[]  virtices0, vertices1; // TZ: This is better than an array[2]
//  };

    private class HeightFieldTriangle
    {
        //public:
        // HeightFieldTriangle(){};

        //HeightFieldVertex   *vertices[3];
        // HeightFieldVertex[]   vertices = new HeightFieldVertex[3]; //TODO c1, c2, c3 (TZ)
        HeightFieldVertex   vertices0, vertices1, vertices2;
        //double[]               planeDef=new double[4];
    }


    //////// dxHeightfield /////////////////////////////////////////////////////////////////


    //  int dxHeightfield::dCollideHeightfieldZone( final int minX, final int maxX, final int minZ, final int maxZ,
    //            dxGeom* o2, final int numMaxContactsPossible,
    //            int flags, dContactGeom* contact,
    //            int skip )
    @Override
	int dCollideHeightfieldZone( final int minX, final int maxX, final int minZ, final int maxZ,
            DxGeom o2, final int numMaxContactsPossible,
            int flags, DContactGeomBuffer contacts,
            int skip )
    {
        DContactGeom pContact = null;
        int  x, z;
        // check if not above or inside terrain first
        // while filling a heightmap partial temporary buffer
        //      final unsigned int numX = (maxX - minX) + 1;
        //      final unsigned int numZ = (maxZ - minZ) + 1;
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
                int posHeightFieldRow = tempHeightBuffer[x_local];
                for ( z = minZ, z_local = 0; z_local < numZ; z++, z_local++)
                {
                    Ypos = z * cfSampleDepth; // Always calculate pos via multiplication to avoid computational error accumulation during multiple additions

                    final double h = m_p_data.GetHeight(x, z);
                    //                  HeightFieldRow.at(z_local).vertex[0] = c_Xpos;
                    //                  HeightFieldRow.at(z_local).vertex[1] = h;
                    //                  HeightFieldRow.at(z_local).vertex[2] = Ypos;
                    HeightFieldVertex HeightFieldRowZ = tempHeightInstances[posHeightFieldRow + z_local];
                    HeightFieldRowZ.vertex.set( c_Xpos, h, Ypos);
                    HeightFieldRowZ.coords0 = x;
                    HeightFieldRowZ.coords1 = z;

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
                if (layered)
                    return 0;
                // totally under heightfield
                //pContact = CONTACT(contact, 0);
                pContact = contacts.get(0); 
                pContact.pos.set( o2.final_posr().pos().get0(), minY, o2.final_posr().pos().get2() );
                pContact.normal.set( 0, -1, 0 );

                pContact.depth =  minY - maxO2Height;

                pContact.side1 = -1;
                pContact.side2 = -1;

                return 1;
            }
        }

        int numTerrainContacts = 0;
        //dContactGeom *PlaneContact = m_p_data.m_contacts;

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
        int numTri = 0;
        HeightFieldVertex A, B, C, D;
        // keep only triangle that does intersect geom

        //final unsigned 
        final int maxX_local = maxX - minX;
        //final unsigned 
        final int maxZ_local = maxZ - minZ;

        for ( x_local = 0; x_local < maxX_local; x_local++)
        {
            // ObjArray<HeightFieldVertex> HeightFieldRow      = tempHeightBuffer[x_local];
            // ObjArray<HeightFieldVertex> HeightFieldNextRow  = tempHeightBuffer[x_local + 1];
            int posHeightFieldRow      = tempHeightBuffer[x_local];
            int posHeightFieldNextRow  = tempHeightBuffer[x_local + 1];

            C = tempHeightInstances[posHeightFieldRow];//.at(0);
            D = tempHeightInstances[posHeightFieldNextRow];//.at(0);

            for ( z_local = 0; z_local < maxZ_local; z_local++)
            {
                A = C;
                B = D;
                // C = HeightFieldRow.at(z_local + 1);//  [z_local + 1];
                // D = HeightFieldNextRow.at(z_local + 1);//[z_local + 1];
                C = tempHeightInstances[posHeightFieldRow + z_local + 1];//  [z_local + 1];
                D = tempHeightInstances[posHeightFieldNextRow + z_local + 1];//[z_local + 1];

                final double AHeight = A.vertex.get1();
                final double BHeight = B.vertex.get1();
                final double CHeight = C.vertex.get1();
                final double DHeight = D.vertex.get1();

                final boolean isACollide = AHeight > minO2Height;
                final boolean isBCollide = BHeight > minO2Height;
                final boolean isCCollide = CHeight > minO2Height;
                final boolean isDCollide = DHeight > minO2Height;

                if (Double.isNaN(AHeight) || Double.isNaN(BHeight) || Double.isNaN(CHeight) || Double.isNaN(DHeight))
                    continue;

                if (isACollide || isBCollide || isCCollide)
                {
                    HeightFieldTriangle CurrTriUp = tempTriangleBuffer[numTri++];// final ?? TZ
                    CurrTriUp.vertices0 = A;
                    CurrTriUp.vertices1 = C;
                    CurrTriUp.vertices2 = B;
                }
                if (isBCollide || isCCollide || isDCollide)
                {
                    HeightFieldTriangle CurrTriDown = tempTriangleBuffer[numTri++];//final ?? TZ
                    CurrTriDown.vertices0 = D;
                    CurrTriDown.vertices1 = B;
                    CurrTriDown.vertices2 = C;
                }
            }
        }
        if (numTri == 0) {
        	return 0;
        }
        float[] vertices = new float[numTri * 9];
        int[] faces = new int[numTri * 3];
        for (int k = 0; k < numTri; k++) {
            HeightFieldTriangle itTriangle = tempTriangleBuffer[k];
            for (int j = 0; j < 3; j ++) {
                HeightFieldVertex hFVertex = j == 0 ? itTriangle.vertices0 : j == 1 ? itTriangle.vertices1 : itTriangle.vertices2;
                for (int i = 0; i < 3; i ++) {
                    // vertices[k * 9 + j * 3 + i] = (float) itTriangle.vertices[j].vertex.get(i);
                    vertices[k * 9 + j * 3 + i] = (float) hFVertex.vertex.get(i);
                }
                faces[k * 3 + j] = k * 3 + j;
            }
        }
        DxGimpactData data = new DxGimpactData();
        data.build(vertices, faces);
        DxGimpact trimesh = new DxGimpact(null, data, null, null, null);
        trimesh.recomputeAABB();
        numTerrainContacts = DxGeom.dCollide(trimesh, o2, flags, contacts, skip);
        trimesh.destroy();
        return numTerrainContacts;
    }

    static class CollideHeightfield implements DColliderFn {
        int dCollideHeightfield( DxTrimeshHeightfield o1, DxGeom o2, int flags, DContactGeomBuffer contacts, int skip )
        {
            dIASSERT( skip >= 1);//(int)sizeof(dContactGeom) );
            //dIASSERT( o1.type == dHeightfieldClass );
            dIASSERT((flags & NUMC_MASK) >= 1);

            int i;

            // if ((flags & NUMC_MASK) == 0) -- An assertion check is made on entry
            //  { flags = (flags & ~NUMC_MASK) | 1; dIASSERT((1 & ~NUMC_MASK) == 0); }

            int numMaxTerrainContacts = (flags & NUMC_MASK);

            DxTrimeshHeightfield terrain = o1;

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
            return dCollideHeightfield((DxTrimeshHeightfield)o1, (DxGeom)o2, flags, contacts, 1);
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
