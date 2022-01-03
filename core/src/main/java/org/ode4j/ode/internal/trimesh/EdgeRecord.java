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

import static org.ode4j.ode.DTriMesh.*;
import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.Common.dIN_RANGE;
import static org.ode4j.ode.internal.trimesh.DxTriDataBase.*;

class EdgeRecord implements Comparable<EdgeRecord> {
    // public:
    // void setupEdge(dMeshTriangleVertex edgeIdx, int triIdx, const unsigned vertexIndices[dMTV__MAX]);


    //void dxTriDataBase::EdgeRecord::setupEdge(dMeshTriangleVertex edgeIdx, int triIdx, const unsigned vertexIndices[dMTV__MAX])
    void setupEdge(int edgeIdx, int triIdx, final int[] vertexIndices)
    {
        if (edgeIdx < dMTV_SECOND)
        {
            dIASSERT(edgeIdx == dMTV_FIRST);

            m_edgeFlags  = CUF_USE_FIRST_EDGE;
            m_vert1Flags = CUF_USE_FIRST_VERTEX;
            m_vert2Flags = CUF_USE_SECOND_VERTEX;
            m_vertIdx1 = vertexIndices[dMTV_FIRST];
            m_vertIdx2 = vertexIndices[dMTV_SECOND];
        }
        else if (edgeIdx == dMTV_SECOND)
        {
            m_edgeFlags  = CUF_USE_SECOND_EDGE;
            m_vert1Flags = CUF_USE_SECOND_VERTEX;
            m_vert2Flags = CUF_USE_THIRD_VERTEX;
            m_vertIdx1 = vertexIndices[dMTV_SECOND];
            m_vertIdx2 = vertexIndices[dMTV_THIRD];
        }
        else
        {
            dIASSERT(edgeIdx == dMTV_THIRD);

            m_edgeFlags  = CUF_USE_THIRD_EDGE;
            m_vert1Flags = CUF_USE_THIRD_VERTEX;
            m_vert2Flags = CUF_USE_FIRST_VERTEX;
            m_vertIdx1 = vertexIndices[dMTV_THIRD];
            m_vertIdx2 = vertexIndices[dMTV_FIRST];
        }

        // Make sure vertex index 1 is less than index 2 (for easier sorting)
        if (m_vertIdx1 > m_vertIdx2)
        {
            //dxSwap(m_vertIdx1, m_vertIdx2);
            int tmpI = m_vertIdx1;
            m_vertIdx1 = m_vertIdx2;
            m_vertIdx2 = tmpI;
            //dxSwap(m_vert1Flags, m_vert2Flags);
            byte tmpB = m_vert1Flags;
            m_vert1Flags = m_vert2Flags;
            m_vert2Flags = tmpB;
        }

        m_triIdx = triIdx;
        m_absVertexFlags = 0;
    }


    // Get the vertex opposite this edge in the triangle
    //DMeshTriangleVertex
    int getOppositeVertexIndex() {
        //extern const
        //CEnumUnsortedElementArray < unsigned, dxTriDataBase::CUF__USE_VERTICES_LAST / dxTriDataBase::CUF__USE_VERTICES_MIN, dMeshTriangleVertex, 0x161116DC > g_VertFlagOppositeIndices;

        //dMeshTriangleVertex
        int oppositeIndex = DxTriDataBase.g_VertFlagOppositeIndices.Encode(((m_vert1Flags | m_vert2Flags) ^ CUF__USE_ALL_VERTICES) / CUF__USE_VERTICES_MIN - 1);
        dIASSERT(dIN_RANGE(oppositeIndex, dMTV__MIN, dMTV__MAX));

        return oppositeIndex;
    }

    //DMeshTriangleVertex
    int getEdgeStartVertexIndex() {
        //extern const
        //CEnumUnsortedElementArray < unsigned, dxTriDataBase::CUF__USE_VERTICES_LAST / dxTriDataBase::CUF__USE_VERTICES_MIN, dMeshTriangleVertex, 0x161225E9 > g_VertFlagEdgeStartIndices;

        //dMeshTriangleVertex
        int startIndex = DxTriDataBase.g_VertFlagEdgeStartIndices.Encode(((m_vert1Flags | m_vert2Flags) ^ CUF__USE_ALL_VERTICES) / CUF__USE_VERTICES_MIN - 1);
        dIASSERT(dIN_RANGE(startIndex, dMTV__MIN, dMTV__MAX));

        return startIndex;
    }

    //  public
    //  bool operator <(const EdgeRecord &anotherEdge)const
    //  {
    //      return m_vertIdx1 < anotherEdge.m_vertIdx1 || (m_vertIdx1 == anotherEdge.m_vertIdx1 && m_vertIdx2 < anotherEdge.m_vertIdx2);
    //  }
    @Override
    public int compareTo(EdgeRecord other) {
        int c1 = Integer.compare(m_vertIdx1, other.m_vertIdx1);
        return c1 != 0 ? c1 : Integer.compare(m_vertIdx2, other.m_vertIdx2);
    }


    //public enum
    //{
    public static final int AVF_VERTEX_USED = 0x01;
    public static final int AVF_VERTEX_HAS_CONCAVE_EDGE = 0x02;
    //}

    //public:
    int m_vertIdx1;    // Index into vertex array for this edges vertices
    int m_vertIdx2;
    int m_triIdx;        // Index into triangle array for triangle this edge belongs to

    //uint8
    byte m_edgeFlags;
    byte m_vert1Flags;
    byte m_vert2Flags;
    byte m_absVertexFlags;


}
