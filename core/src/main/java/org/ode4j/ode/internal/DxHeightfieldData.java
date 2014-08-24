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

import java.util.Arrays;

import org.ode4j.math.DVector3C;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DHeightfieldData;
import org.ode4j.ode.DHeightfield.DHeightfieldGetHeight;
import org.ode4j.ode.internal.DxHeightfield.HeightFieldVertex;

import static org.ode4j.ode.OdeMath.*;

/**
 * dHeightfield Collider
 * Paul Cheyrou-Lagreze aka Tuan Kuranes 2006 Speed enhancements http://www.pop-3d.com
 * Martijn Buijs 2006 http://home.planet.nl/~buijs512/
 * Based on Terrain & Cone contrib by:
 * Benoit CHAPEROT 2003-2004 http://www.jstarlab.com
 * Some code inspired by Magic Software
 */ 
public class DxHeightfieldData implements DHeightfieldData {


//	static final int HEIGHTFIELDMAXCONTACTPERCELL = 10;  -> Moved to dxHeightField


	double m_fWidth;				// World space heightfield dimension on X axis
	double m_fDepth;				// World space heightfield dimension on Z axis
	double m_fSampleWidth;		// Vertex spacing on X axis edge (== m_vWidth / (m_nWidthSamples-1))
	double m_fSampleDepth;		// Vertex spacing on Z axis edge (== m_vDepth / (m_nDepthSamples-1))
	double m_fSampleZXAspect;    // Relation of Z axis spacing to X axis spacing (== m_fSampleDepth / m_fSampleWidth)
	double m_fInvSampleWidth;		// Cache of inverse Vertex count on X axis edge (== m_vWidth / (m_nWidthSamples-1))
	double m_fInvSampleDepth;		// Cache of inverse Vertex count on Z axis edge (== m_vDepth / (m_nDepthSamples-1))

	double m_fHalfWidth;			// Cache of half of m_fWidth
	double m_fHalfDepth;			// Cache of half of m_fDepth

	double m_fMinHeight;        // Min sample height value (scaled and offset)
	double m_fMaxHeight;        // Max sample height value (scaled and offset)
	double m_fThickness;        // Surface thickness (added to bottom AABB)
	double m_fScale;            // Sample value multiplier
	double m_fOffset;           // Vertical sample offset

	int m_nWidthSamples;       // Vertex count on X axis edge (number of samples)
	int m_nDepthSamples;       // Vertex count on Z axis edge (number of samples)
	boolean m_bCopyHeightData;     // Do we own the sample data?
	boolean m_bWrapMode;           // Heightfield wrapping mode (0=finite, 1=infinite)
	int m_nGetHeightMode;      // GetHeight mode ( 0=callback, 1=byte, 2=short, 3=float )

	//    const void* m_pHeightData; // Sample data array
	//    void* m_pUserData;         // Callback user data
	Object m_pHeightData; // Sample data array
	Object m_pUserData;         // Callback user data

	//dContactGeom[]            m_contacts = new dContactGeom[HEIGHTFIELDMAXCONTACTPERCELL];
	DContactGeomBuffer            m_contacts = new DContactGeomBuffer(DxHeightfield.HEIGHTFIELDMAXCONTACTPERCELL);

	//dHeightfieldGetHeight* m_pGetHeightCallback;		// Callback pointer.
	/** TODO uses CPP-API! */
	DHeightfieldGetHeight m_pGetHeightCallback;		// Callback pointer.

	//    dxHeightfieldData();
	//    ~dxHeightfieldData();
	//
	//    void SetData( int nWidthSamples, int nDepthSamples,
	//        dReal fWidth, dReal fDepth,
	//        dReal fScale, dReal fOffset,
	//        dReal fThickness, int bWrapMode );
	//
	//    void ComputeHeightBounds();
	//
	//    bool IsOnHeightfield2  ( const HeightFieldVertex * const CellCorner,
	//        const dReal * const pos,  const bool isABC) const;
	//
	//    dReal GetHeight(int x, int z);
	//    dReal GetHeight(dReal x, dReal z);




	////////dxHeightfieldData /////////////////////////////////////////////////////////////

	//dxHeightfieldData constructor
	DxHeightfieldData() {	
		m_fWidth = 0;
		m_fDepth = 0;
		m_fSampleWidth = 0;
		m_fSampleDepth = 0;
		m_fSampleZXAspect = 0;
		m_fInvSampleWidth = 0;
		m_fInvSampleDepth = 0;

		m_fHalfWidth = 0;
		m_fHalfDepth = 0;

		m_fMinHeight = 0;
		m_fMaxHeight = 0;
		m_fThickness = 0;
		m_fScale = 0;
		m_fOffset = 0;

		m_nWidthSamples = 0;
		m_nDepthSamples = 0;
		m_bCopyHeightData = false;
		m_bWrapMode = false;
		m_nGetHeightMode = 0;

		m_pHeightData = null;
		m_pUserData = null;

		m_pGetHeightCallback = null;

		//memset( m_contacts, 0, sizeof( m_contacts ) );
	}

	//build Heightfield data
	//void dxHeightfieldData::SetData( int nWidthSamples, int nDepthSamples,
	//        dReal fWidth, dReal fDepth,
	//        dReal fScale, dReal fOffset, dReal fThickness,
	//        int bWrapMode )
	void SetData( int nWidthSamples, int nDepthSamples,
			double fWidth, double fDepth,
			double fScale, double fOffset, double fThickness,
			boolean bWrapMode )
	{
		dIASSERT( fWidth > ( 0.0 ) );
		dIASSERT( fDepth > ( 0.0 ) );
		dIASSERT( nWidthSamples > 0 );
		dIASSERT( nDepthSamples > 0 );

		// x,z bounds
		m_fWidth = fWidth;
		m_fDepth = fDepth;

		// cache half x,z bounds
		m_fHalfWidth = fWidth / ( 2.0 );
		m_fHalfDepth = fDepth / ( 2.0 );

		// scale and offset
		m_fScale = fScale;
		m_fOffset = fOffset;

		// infinite min height bounds
		m_fThickness = fThickness;

		// number of vertices per side
		m_nWidthSamples = nWidthSamples;
		m_nDepthSamples = nDepthSamples;

		m_fSampleWidth = m_fWidth / ( m_nWidthSamples - ( 1.0 ) );
		m_fSampleDepth = m_fDepth / ( m_nDepthSamples - ( 1.0 ) );

		m_fSampleZXAspect = m_fSampleDepth / m_fSampleWidth;

		m_fInvSampleWidth = ( 1.0 ) / m_fSampleWidth;
		m_fInvSampleDepth = ( 1.0 ) / m_fSampleDepth;

		// finite or repeated terrain?
		m_bWrapMode = bWrapMode;
	}


	//recomputes heights bounds
	void ComputeHeightBounds()
	{
		int i;
		double h;
		//unsigned char *data_byte;
		byte[] data_byte;
		//short *data_short;
		//float *data_float;
		//double *data_double;
		short []data_short;
		float []data_float;
		double []data_double;

		switch ( m_nGetHeightMode )
		{

		// callback
		case 0:
			// change nothing, keep using default or user specified bounds
			return;

			// byte
		case 1:
			data_byte = (byte[])m_pHeightData;
			m_fMinHeight = dInfinity;
			m_fMaxHeight = -dInfinity;

			for (i=0; i<m_nWidthSamples*m_nDepthSamples; i++)
			{
				h = data_byte[i];
				if (h < m_fMinHeight)	m_fMinHeight = h;
				if (h > m_fMaxHeight)	m_fMaxHeight = h;
			}

			break;

			// short
		case 2:
			data_short = (short[])m_pHeightData;
			m_fMinHeight = dInfinity;
			m_fMaxHeight = -dInfinity;

			for (i=0; i<m_nWidthSamples*m_nDepthSamples; i++)
			{
				h = data_short[i];
				if (h < m_fMinHeight)	m_fMinHeight = h;
				if (h > m_fMaxHeight)	m_fMaxHeight = h;
			}

			break;

			// float
		case 3:
			data_float = (float[])m_pHeightData;
			m_fMinHeight = dInfinity;
			m_fMaxHeight = -dInfinity;

			for (i=0; i<m_nWidthSamples*m_nDepthSamples; i++)
			{
				h = data_float[i];
				if (h < m_fMinHeight)	m_fMinHeight = h;
				if (h > m_fMaxHeight)	m_fMaxHeight = h;
			}

			break;

			// double
		case 4:
			data_double = (double[])m_pHeightData;
			m_fMinHeight = dInfinity;
			m_fMaxHeight = -dInfinity;

			for (i=0; i<m_nWidthSamples*m_nDepthSamples; i++)
			{
				h = data_double[i];//static_cast< double >( data_double[i] );
				if (h < m_fMinHeight)	m_fMinHeight = h;
				if (h > m_fMaxHeight)	m_fMaxHeight = h;
			}

			break;

		}

		// scale and offset
		m_fMinHeight *= m_fScale;
		m_fMaxHeight *= m_fScale;
		m_fMinHeight += m_fOffset;
		m_fMaxHeight += m_fOffset;

		// add thickness
		m_fMinHeight -= m_fThickness;
	}


	//returns whether point is over terrain Cell triangle?
	//bool dxHeightfieldData::IsOnHeightfield2 ( const HeightFieldVertex * const CellCorner, 
	//		const double * const pos,  const bool isABC) const
	boolean IsOnHeightfield2 ( final HeightFieldVertex CellCorner, 
			final DVector3C pos,  final boolean isABC) 
	{
		// WARNING!!!
		// This function must be written in the way to make sure that every point on
		// XZ plane falls in one and only one triangle. Keep that in mind if you 
		// intend to change the code.
		// Also remember about computational errors and possible mismatches in 
		// values if they are calculated differently in different places in the code.
		// Currently both the implementation has been optimized and effects of 
		// computational errors have been eliminated.

		double MaxX, MinX;
		double MaxZ, MinZ;

		if (isABC)
		{
			// point A
			MinX = CellCorner.vertex.get0();
			if (pos.get0() < MinX)
				return false;

			MaxX = (CellCorner.coords0 + 1) * m_fSampleWidth;
			if (pos.get0() >= MaxX)
				return false;

			MinZ = CellCorner.vertex.get2();
			if (pos.get2() < MinZ)
				return false;

			MaxZ = (CellCorner.coords1 + 1) * m_fSampleDepth;
			if (pos.get2() >= MaxZ)
				return false;

			return (MaxZ - pos.get2()) > (pos.get0() - MinX) * m_fSampleZXAspect;
		}
		else
		{
			// point D
			MaxX = CellCorner.vertex.get0();
			if (pos.get0() >= MaxX)
				return false;

			MinX = (CellCorner.coords0 - 1) * m_fSampleWidth;
			if (pos.get0() < MinX)
				return false;

			MaxZ = CellCorner.vertex.get2();
			if (pos.get2() >= MaxZ)
				return false;

			MinZ = (CellCorner.coords1 - 1) * m_fSampleDepth;
			if (pos.get2() < MinZ)
				return false;

			return (MaxZ - pos.get2()) <= (pos.get0() - MinX) * m_fSampleZXAspect;
		}
	}


	//returns height at given sample coordinates
	//double dxHeightfieldData::GetHeight( int x, int z )
	double GetHeight( int x, int z )
	{
		double h=0;
		byte[] data_byte;
		short[] data_short;
		float[] data_float;
		double[] data_double;

		if ( m_bWrapMode == false )
		{
			// Finite
			if ( x < 0 ) x = 0;
			if ( z < 0 ) z = 0;
			if ( x > m_nWidthSamples - 1 ) x = m_nWidthSamples - 1;
			if ( z > m_nDepthSamples - 1 ) z = m_nDepthSamples - 1;
		}
		else
		{
			// Infinite
			x %= m_nWidthSamples - 1;
			z %= m_nDepthSamples - 1;
			if ( x < 0 ) x += m_nWidthSamples - 1;
			if ( z < 0 ) z += m_nDepthSamples - 1;
		}

		switch ( m_nGetHeightMode )
		{

		// callback (dReal)
		case 0:
			//h = (*m_pGetHeightCallback)(m_pUserData, x, z);
			h = m_pGetHeightCallback.call(m_pUserData, x, z);
			break;

			// byte
		case 1:
			data_byte = (byte[])m_pHeightData;
			h = data_byte[x+(z * m_nWidthSamples)];
			break;

			// short
		case 2:
			data_short = (short[])m_pHeightData;
			h = data_short[x+(z * m_nWidthSamples)];
			break;

			// float
		case 3:
			data_float = (float[])m_pHeightData;
			h = data_float[x+(z * m_nWidthSamples)];
			break;

			// double
		case 4:
			data_double = (double[])m_pHeightData;
			h = data_double[x+(z * m_nWidthSamples)];
			break;
		}

		return (h * m_fScale) + m_fOffset;
	}


	//returns height at given coordinates
	//double dxHeightfieldData::GetHeight( double x, double z )
	//TODO TZ report: this is never used.
//	private double GetHeight( double x, double z )
//	{
//		double dnX = dFloor( x * m_fInvSampleWidth );
//		double dnZ = dFloor( z * m_fInvSampleDepth );
//
//		double dx = ( x - ( dnX * m_fSampleWidth ) ) * m_fInvSampleWidth;
//		double dz = ( z - ( dnZ * m_fSampleDepth ) ) * m_fInvSampleDepth;
//
//		int nX = (int) dnX;
//		int nZ = (int) dnZ;
//
//		//dIASSERT( ( dx + dEpsilon >= 0.0f ) && ( dx - dEpsilon <= 1.0f ) );
//		//dIASSERT( ( dz + dEpsilon >= 0.0f ) && ( dz - dEpsilon <= 1.0f ) );
//
//		double y, y0;
//
//		if ( dx + dz <= ( 1.0 ) ) // Use <= comparison to prefer simpler branch
//		{
//			y0 = GetHeight( nX, nZ );
//
//			y = y0 + ( GetHeight( nX + 1, nZ ) - y0 ) * dx
//			+ ( GetHeight( nX, nZ + 1 ) - y0 ) * dz;
//		}
//		else
//		{
//			y0 = GetHeight( nX + 1, nZ + 1 );
//
//			y = y0	+ ( GetHeight( nX + 1, nZ ) - y0 ) * ( (1.0) - dz ) +
//			( GetHeight( nX, nZ + 1 ) - y0 ) * ( (1.0) - dx );
//		}
//
//		return y;
//	}


	//dxHeightfieldData destructor
	//dxHeightfieldData::~dxHeightfieldData()
	void DESTRUCTOR()
	{
		//TZ: Nothing to do here...
//		byte[] data_byte;
//		short[] data_short;
//		float[] data_float;
//		double[] data_double;
//
//		if ( m_bCopyHeightData )
//		{
//			switch ( m_nGetHeightMode )
//			{
//
//			// callback
//			case 0:
//				// do nothing
//				break;
//
//				// byte
//			case 1:
////				dIASSERT( m_pHeightData != null );
////				data_byte = (byte[])m_pHeightData;
////				delete [] data_byte;
//				break;
//
//				// short
//			case 2:
////				dIASSERT( m_pHeightData != null );
////				data_short = (short[])m_pHeightData;
////				delete [] data_short;
//				break;
//
//				// float
//			case 3:
////				dIASSERT( m_pHeightData != null );
////				data_float = (float[])m_pHeightData;
////				delete [] data_float;
//				break;
//
//				// double
//			case 4:
////				dIASSERT( m_pHeightData != null);
////				data_double = (double[])m_pHeightData;
////				delete [] data_double;
//				break;
//
//			}
//		}
	}
	//////// Heightfield data interface ////////////////////////////////////////////////////


	public static DHeightfieldData dGeomHeightfieldDataCreate()
	{
		return new DxHeightfieldData();
	}


	//	void dGeomHeightfieldDataBuildCallback( dHeightfieldDataID d,
	//            void* pUserData, dHeightfieldGetHeight* pCallback,
	//            dReal width, dReal depth, int widthSamples, int depthSamples,
	//            dReal scale, dReal offset, dReal thickness, int bWrap )
	public void dGeomHeightfieldDataBuildCallback( 
			Object pUserData, DHeightfieldGetHeight pCallback,
			double width, double depth, int widthSamples, int depthSamples,
			double scale, double offset, double thickness, boolean bWrap )
	{
		//dUASSERT( d, "argument not Heightfield data" );
		dIASSERT( pCallback !=null);
		dIASSERT( widthSamples >= 2 );	// Ensure we're making something with at least one cell.
		dIASSERT( depthSamples >= 2 );

		// callback
		m_nGetHeightMode = 0;
		m_pUserData = pUserData;
		m_pGetHeightCallback = pCallback;

		// set info
		SetData( widthSamples, depthSamples, width, depth, scale, offset, thickness, bWrap );

		// default bounds
		m_fMinHeight = -dInfinity;
		m_fMaxHeight = dInfinity;
	}


	//	void dGeomHeightfieldDataBuildByte( dHeightfieldDataID d,
	//            const unsigned char *pHeightData, int bCopyHeightData,
	//            dReal width, dReal depth, int widthSamples, int depthSamples,
	//            dReal scale, dReal offset, dReal thickness, int bWrap )
	void dGeomHeightfieldDataBuildByte( 
			final byte[] pHeightData, boolean bCopyHeightData,
			double width, double depth, int widthSamples, int depthSamples,
			double scale, double offset, double thickness, boolean bWrap )
	{
//		dUASSERT( d, "Argument not Heightfield data" );
		dIASSERT( pHeightData!=null );
		dIASSERT( widthSamples >= 2 );	// Ensure we're making something with at least one cell.
		dIASSERT( depthSamples >= 2 );

		// set info
		SetData( widthSamples, depthSamples, width, depth, scale, offset, thickness, bWrap );
		m_nGetHeightMode = 1;
		m_bCopyHeightData = bCopyHeightData;

		if ( m_bCopyHeightData == false )
		{
			// Data is referenced only.
			m_pHeightData = pHeightData;
		}
		else
		{
			// We own the height data, allocate storage
//			d.m_pHeightData = new byte[ d.m_nWidthSamples * d.m_nDepthSamples ];
			//dIASSERT( d.m_pHeightData );

			// Copy data.
//			memcpy( (void*)d.m_pHeightData, pHeightData,
//					sizeof( unsigned char ) * d.m_nWidthSamples * d.m_nDepthSamples );
			m_pHeightData = Arrays.copyOf(pHeightData, m_nWidthSamples * m_nDepthSamples);
		}

		// Find height bounds
		ComputeHeightBounds();
	}


	//	void dGeomHeightfieldDataBuildShort( dHeightfieldDataID d,
	//            const short* pHeightData, int bCopyHeightData,
	//            dReal width, dReal depth, int widthSamples, int depthSamples,
	//            dReal scale, dReal offset, dReal thickness, int bWrap )
	void dGeomHeightfieldDataBuildShort( 
			final short[] pHeightData, boolean bCopyHeightData,
			double width, double depth, int widthSamples, int depthSamples,
			double scale, double offset, double thickness, boolean bWrap )
	{
		//dUASSERT( d, "Argument not Heightfield data" );
		dIASSERT( pHeightData!=null );
		dIASSERT( widthSamples >= 2 );	// Ensure we're making something with at least one cell.
		dIASSERT( depthSamples >= 2 );

		// set info
		SetData( widthSamples, depthSamples, width, depth, scale, offset, thickness, bWrap );
		m_nGetHeightMode = 2;
		m_bCopyHeightData = bCopyHeightData;

		if ( m_bCopyHeightData == false )
		{
			// Data is referenced only.
			m_pHeightData = pHeightData;
		}
		else
		{
			// We own the height data, allocate storage
//			d.m_pHeightData = new short[ d.m_nWidthSamples * d.m_nDepthSamples ];
//			dIASSERT( d.m_pHeightData );

			// Copy data.
//			memcpy( (void*)d.m_pHeightData, pHeightData,
//					sizeof( short ) * d.m_nWidthSamples * d.m_nDepthSamples );
			m_pHeightData = Arrays.copyOf(pHeightData, m_nWidthSamples * m_nDepthSamples);
		}

		// Find height bounds
		ComputeHeightBounds();
	}


	//	void dGeomHeightfieldDataBuildSingle( dHeightfieldDataID d,
	//            const float *pHeightData, int bCopyHeightData,
	//            dReal width, dReal depth, int widthSamples, int depthSamples,
	//            dReal scale, dReal offset, dReal thickness, int bWrap )
	void dGeomHeightfieldDataBuildSingle( 
			final float[] pHeightData, boolean bCopyHeightData,
			double width, double depth, int widthSamples, int depthSamples,
			double scale, double offset, double thickness, boolean bWrap )
	{
		//dUASSERT( d, "Argument not Heightfield data" );
		dIASSERT( pHeightData!=null );
		dIASSERT( widthSamples >= 2 );	// Ensure we're making something with at least one cell.
		dIASSERT( depthSamples >= 2 );

		// set info
		SetData( widthSamples, depthSamples, width, depth, scale, offset, thickness, bWrap );
		m_nGetHeightMode = 3;
		m_bCopyHeightData = bCopyHeightData;

		if ( m_bCopyHeightData == false )
		{
			// Data is referenced only.
			m_pHeightData = pHeightData;
		}
		else
		{
			// We own the height data, allocate storage
//			d.m_pHeightData = new float[ d.m_nWidthSamples * d.m_nDepthSamples ];
			//dIASSERT( d.m_pHeightData );

			// Copy data.
//			memcpy( (void*)d.m_pHeightData, pHeightData,
//					sizeof( float ) * d.m_nWidthSamples * d.m_nDepthSamples );
			m_pHeightData = Arrays.copyOf(pHeightData, m_nWidthSamples * m_nDepthSamples);
		}

		// Find height bounds
		ComputeHeightBounds();
	}

	//	void dGeomHeightfieldDataBuildDouble( dHeightfieldDataID d,
	//            const double *pHeightData, int bCopyHeightData,
	//            dReal width, dReal depth, int widthSamples, int depthSamples,
	//            dReal scale, dReal offset, dReal thickness, int bWrap )
	void dGeomHeightfieldDataBuildDouble( 
			final double[] pHeightData, boolean bCopyHeightData,
			double width, double depth, int widthSamples, int depthSamples,
			double scale, double offset, double thickness, boolean bWrap )
	{
		//dUASSERT( d, "Argument not Heightfield data" );
		dIASSERT( pHeightData!=null );
		dIASSERT( widthSamples >= 2 );	// Ensure we're making something with at least one cell.
		dIASSERT( depthSamples >= 2 );

		// set info
		SetData( widthSamples, depthSamples, width, depth, scale, offset, thickness, bWrap );
		m_nGetHeightMode = 4;
		m_bCopyHeightData = bCopyHeightData;

		if ( m_bCopyHeightData == false )
		{
			// Data is referenced only.
			m_pHeightData = pHeightData;
		}
		else
		{
			// We own the height data, allocate storage
//			d.m_pHeightData = new double[ d.m_nWidthSamples * d.m_nDepthSamples ];
			//dIASSERT( d.m_pHeightData );

			// Copy data.
//			memcpy( (void*)d.m_pHeightData, pHeightData,
//					sizeof( double ) * d.m_nWidthSamples * d.m_nDepthSamples );
			m_pHeightData = Arrays.copyOf(pHeightData, m_nWidthSamples * m_nDepthSamples);
		}

		// Find height bounds
		ComputeHeightBounds();
	}




//	void dGeomHeightfieldDataSetBounds( dxHeightfieldData d, double minHeight, double maxHeight )
	public void dGeomHeightfieldDataSetBounds( double minHeight, double maxHeight )
	{
		//dUASSERT(d, "Argument not Heightfield data");
		m_fMinHeight = ( minHeight * m_fScale ) + m_fOffset - m_fThickness;
		m_fMaxHeight = ( maxHeight * m_fScale ) + m_fOffset;
	}


//	void dGeomHeightfieldDataDestroy( dxHeightfieldData d )
	public void dGeomHeightfieldDataDestroy()
	{
		//dUASSERT(d, "argument not Heightfield data");
		//delete d;
		//TZ:
		DESTRUCTOR();
	}

	@Override
	public void destroy() {
		dGeomHeightfieldDataDestroy();
	}

	@Override
	public void setBounds(double minHeight, double maxHeight) {
		dGeomHeightfieldDataSetBounds( minHeight, maxHeight );
	}

	@Override
	public void buildCallback(
			Object userData, DHeightfieldGetHeight callback, double width,
			double depth, int widthSamples, int depthSamples, double scale,
			double offset, double thickness, boolean wrap) {
		dGeomHeightfieldDataBuildCallback(userData, 
				callback, width, depth, widthSamples, depthSamples, 
				scale, offset, thickness, wrap);
	}
		
	@Override
	public void build(
			final byte[] pHeightData, boolean bCopyHeightData,
			double width, double depth, int widthSamples, int depthSamples,
			double scale, double offset, double thickness, boolean bWrap ) {
		dGeomHeightfieldDataBuildByte( pHeightData, bCopyHeightData, 
				width, depth, widthSamples, depthSamples, scale, offset, thickness, bWrap);
	}

	@Override
	public void build(short[] pHeightData, boolean bCopyHeightData, double width,
			double depth, int widthSamples, int depthSamples, double scale,
			double offset, double thickness, boolean bWrap) {
		dGeomHeightfieldDataBuildShort( pHeightData, bCopyHeightData, 
				width, depth, widthSamples, depthSamples, scale, offset, thickness, bWrap);
	}

	@Override
	public void build(float[] pHeightData, boolean bCopyHeightData, double width,
			double depth, int widthSamples, int depthSamples, double scale,
			double offset, double thickness, boolean bWrap) {
		dGeomHeightfieldDataBuildSingle( pHeightData, bCopyHeightData, 
				width, depth, widthSamples, depthSamples, scale, offset, thickness, bWrap);
	}

	@Override
	public void build(double[] pHeightData, boolean bCopyHeightData, double width,
			double depth, int widthSamples, int depthSamples, double scale,
			double offset, double thickness, boolean bWrap) {
		dGeomHeightfieldDataBuildDouble( pHeightData, bCopyHeightData, 
				width, depth, widthSamples, depthSamples, scale, offset, thickness, bWrap);
	}

}