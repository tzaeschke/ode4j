package org.ode4j.ode.internal.trimesh;

import org.ode4j.ode.internal.cpp4j.java.RefDouble;

public interface IFaceAngleStorageView {
    //    class IFaceAngleStorageView
    //    {
    //        public:
    //        virtual FaceAngleDomain retrieveFacesAngleFromStorage(dReal &out_AngleValue, unsigned triangleIndex, dMeshTriangleVertex vertexIndex) = 0;
    //    };
    int retrieveFacesAngleFromStorage(RefDouble out_AngleValue, int triangleIndex, int vertexIndex);
}
