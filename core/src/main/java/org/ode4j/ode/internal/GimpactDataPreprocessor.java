package org.ode4j.ode.internal;

import java.util.Arrays;

import org.ode4j.math.DVector3;

public class GimpactDataPreprocessor {

	private final DxGimpactData data;

	public GimpactDataPreprocessor(DxGimpactData data) {
		this.data = data;
	}

	float[] buildAngles() {
		int numVertices = data.getDataRef().length / 3;
		int numIndices = data.getIndexRef().length;
		// Distribute triangles to buckets based on indices of their vertices to
		// speed up the search for neighbours
		// Store counts in the first array and offsets in the second
		int[][] triangleListInfo = new int[2][numVertices];
		// 1. Find triangle count for each bucket
		for (int i = 0; i < numIndices; i++) {
			triangleListInfo[0][getVertexIndex(i)]++;
		}
		int startOffset = 0;
		// 2. Set list offset for each bucket and reset triangle counts
		for (int i = 0; i < numVertices; i++) {
			triangleListInfo[1][i] = startOffset;
			startOffset += triangleListInfo[0][i];
			triangleListInfo[0][i] = 0;
		}
		// 3. Fill buckets with triangles
		int[] triangleLists = new int[numIndices];
		for (int i = 0; i < numIndices; i++) {
			int triangle = i / 3;
			int index = getVertexIndex(i);
			triangleLists[triangleListInfo[1][index] + triangleListInfo[0][index]++] = triangle;
		}
		// 4. Search for neighbours and find the angles
		float[] angles = new float[numIndices];
		Arrays.fill(angles, (float)Math.PI * 2);
		for (int i = 0; i < numIndices; i++) {
			int triangle = i / 3;
			int startIndex = getVertexIndex(i);
			int endIndex = getVertexIndex(triangle * 3 + (i + 1) % 3);
			int count = triangleListInfo[0][startIndex];
			int offset = triangleListInfo[1][startIndex];
			for (int j = 0; j < count; j++) {
				int t = triangleLists[offset + j];
				if (t != triangle && hasEdge(t, endIndex, startIndex)) {
					angles[i] = getAngle(triangle, i % 3, t);
					break;
				}
			}
		}
		return angles;
	}

	private boolean hasEdge(int triangle, int startIndex, int endIndex) {
		for (int i = 0; i < 3; i++) {
			int index1 = getVertexIndex(triangle * 3 + i);
			int index2 = getVertexIndex(triangle * 3 + (i + 1) % 3);
			if (index1 == startIndex && index2 == endIndex) {
				return true;
			}
		}
		return false;
	}

	private int getVertexIndex(int index) {
		return data.getIndexRef()[index];
	}

	private float getAngle(int triangle, int vertexIndex, int neighbourTriangle) {
		DVector3 startVertex = getVertex(data.getIndexRef()[triangle * 3 + vertexIndex]);
		DVector3 endVertex = getVertex(data.getIndexRef()[triangle * 3 + (vertexIndex + 1) % 3]);
		DVector3 oppositeVertex = getVertex(data.getIndexRef()[triangle * 3 + (vertexIndex + 2) % 3]);
		DVector3 edgeAxis = new DVector3(endVertex).sub(startVertex);
		DVector3 secondEdge = new DVector3(oppositeVertex).sub(endVertex);
		DVector3 normal = new DVector3();
		normal.eqCross(edgeAxis, secondEdge);
		DVector3 tangent = new DVector3();
		tangent.eqCross(edgeAxis, normal);

		DVector3 neighbourVertex1 = getVertex(data.getIndexRef()[neighbourTriangle * 3]);
		DVector3 neighbourVertex2 = getVertex(data.getIndexRef()[neighbourTriangle * 3 + 1]);
		DVector3 neighbourVertex3 = getVertex(data.getIndexRef()[neighbourTriangle * 3 + 2]);
		DVector3 neighbourEdge1 = new DVector3(neighbourVertex2).sub(neighbourVertex1);
		DVector3 neighbourEdge2 = new DVector3(neighbourVertex3).sub(neighbourVertex2);
		DVector3 neighbourNormal = new DVector3();
		neighbourNormal.eqCross(neighbourEdge1, neighbourEdge2);
		
		float angle = (float) Math.signum(tangent.dot(neighbourNormal));
		double angleCos = normal.dot(neighbourNormal); 
		double length = Math.sqrt(normal.lengthSquared() * neighbourNormal.lengthSquared());
		if (length > Double.MIN_VALUE) {
			angleCos /= length;
		}
		angleCos = Math.max(Math.min(1.0, angleCos), -1.0);
		angle *= (float) Math.acos(angleCos);
		return angle;
	}

	private DVector3 getVertex(int index) {
		return new DVector3(data.getDataRef()[index * 3], data.getDataRef()[index * 3 + 1],
				data.getDataRef()[index * 3 + 2]);
	}

}
