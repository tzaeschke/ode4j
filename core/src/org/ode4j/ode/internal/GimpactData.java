package org.ode4j.ode.internal;

import org.ode4j.math.DVector3;
import org.ode4j.math.DVector4;

import static org.ode4j.ode.internal.Common.*;

public class GimpactData extends DxTriMeshData {

	private float[] m_Vertices;//const char* m_Vertices;
	int m_VertexStride;
	int m_VertexCount;
	private int[] m_Indices;//const char* m_Indices;
	int m_TriangleCount;
	int m_TriStride;
	boolean m_single;

    GimpactData()//dxTriMeshData()
	{
		m_Vertices=null;
		m_VertexStride = 12;
		m_VertexCount = 0;
		m_Indices = null;
		m_TriangleCount = 0;
		m_TriStride = 12;
		m_single = true;
	}

//    void Build(const void* Vertices, int VertexStride, int VertexCount,
//	       const void* Indices, int IndexCount, int TriStride,
//	       const void* Normals,
//	      bool Single)
    void Build(final float[] Vertices, int VertexStride, int VertexCount,
 	       final int[] Indices, int IndexCount, int TriStride,
 	       final float[] Normals,
 	      boolean Single)
	{
		dIASSERT(Vertices!=null);
		dIASSERT(Indices!=null);
 		dIASSERT(VertexStride!=0);
 		dIASSERT(TriStride!=0);
 		dIASSERT(IndexCount!=0);
		m_Vertices = Vertices;
		m_VertexStride = VertexStride;
		m_VertexCount = VertexCount;
		m_Indices = Indices;
		m_TriangleCount = IndexCount/3;
		m_TriStride = TriStride;
		m_single = Single;
	}

	void GetVertex(int i, DVector4 Out)
	{
		//TZ commented out, special treatment not required (?)
//		if(m_single)
//		{
			//const float * fverts = (const float * )(m_Vertices + m_VertexStride*i);
			int p = i*m_VertexStride;
//			Out[0] = fverts[0];
//			Out[1] = fverts[1];
//			Out[2] = fverts[2];
//			Out[3] = 1.0f;
			Out.set(m_Vertices[p], m_Vertices[p+1], m_Vertices[p+2], 1.0f);
//		}
//		else
//		{
//			const double * dverts = (const double * )(m_Vertices + m_VertexStride*i);
//			Out[0] = (float)dverts[0];
//			Out[1] = (float)dverts[1];
//			Out[2] = (float)dverts[2];
//			Out[3] = 1.0f;
//
//		}
	}

	//void GetTriIndices(unsigned int itriangle, unsigned int triindices[3])
	void GetTriIndices(int itriangle, int[] triindices)
	{
		//const unsigned int * ind = (const unsigned int * )(m_Indices + m_TriStride*itriangle);
		int p = itriangle*m_TriStride;
		triindices[0] = m_Indices[p+0];
		triindices[1] = m_Indices[p+1];
		triindices[2] = m_Indices[p+2];
	}
//#endif  // dTRIMESH_GIMPACT

	@Override
	void Preprocess() {
		// TODO Auto-generated method stub
		
	}

	@Override
	void UpdateData() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void buildSingle(double[] Vertices, int VertexStride,
			int VertexCount, int[] Indices, int IndexCount, int TriStride) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void buildSingle(float[] Vertices, int VertexStride,
			int VertexCount, int[] Indices, int IndexCount, int TriStride) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void destroy() {
		// TODO Auto-generated method stub
		
	}

}
