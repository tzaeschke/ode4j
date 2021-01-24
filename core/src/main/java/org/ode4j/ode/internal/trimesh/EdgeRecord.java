package org.ode4j.ode.internal.trimesh;

import org.ode4j.ode.internal.DxTriMeshData;
import org.ode4j.ode.ou.CEnumUnsortedElementArray;

import static org.ode4j.ode.DTriMesh.*;
import static org.ode4j.ode.internal.Common.dIASSERT;
import static org.ode4j.ode.internal.Common.dIN_RANGE;
import static org.ode4j.ode.internal.trimesh.DxTriDataBase.*;

class EdgeRecord {
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
    public boolean isLessThan(EdgeRecord anotherEdge) {
        return m_vertIdx1 < anotherEdge.m_vertIdx1 ||
                (m_vertIdx1 == anotherEdge.m_vertIdx1 && m_vertIdx2 < anotherEdge.m_vertIdx2);
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
