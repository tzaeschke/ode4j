package org.ode4j.ode.internal;

import org.ode4j.math.DVector3;
import org.ode4j.ode.DAABB;
import org.ode4j.ode.DTriMeshData;

public class Gimpact extends DxTriMesh {

	GimpactData _Data;
	
	//TZ TODO
//    GIM_TRIMESH  m_collision_trimesh;
//	GBUFFER_MANAGER_DATA m_buffer_managers[G_BUFFER_MANAGER__MAX];

	public Gimpact(DxSpace space, GimpactData data) {
		super(space, data);
		_Data = data;
		// TODO Auto-generated constructor stub
	}

	@Override
	boolean AABBTest(DAABB aabb) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	void ClearTCCache() {
		// TODO Auto-generated method stub
		
	}

	@Override
	void computeAABB() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public Object dGeomTriMeshDataGet(DTriMeshData g, int dataId) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void dGeomTriMeshDataSet(DTriMeshData g, int dataId, Object inData) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public int FetchTriangleCount() {
		return GimpactCollision.FetchTriangleCount(this);
	}

	@Override
	public void FetchTransformedTriangle(int i, DVector3[] v) {
		GimpactCollision.FetchTransformedTriangle(this, i, v);
	}

}
