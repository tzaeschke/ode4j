package org.ode4j.ode.internal.trimesh;

interface IFaceAngleStorageControl {
    //    class IFaceAngleStorageControl
    //    {
    //        public:
    //        virtual void disposeStorage() = 0;
    //
    //        virtual bool areNegativeAnglesStored() const = 0;
    //
    //        // This is to store angles between neighbor triangle normals as positive value for convex and negative for concave edges
    //        virtual void assignFacesAngleIntoStorage(unsigned triangleIndex, dMeshTriangleVertex vertexIndex, dReal dAngleValue) = 0;
    //    };
    //

    void disposeStorage();
    boolean areNegativeAnglesStored();

    // This is to store angles between neighbor triangle normals as positive value for convex and negative for concave edges
    void assignFacesAngleIntoStorage(int triangleIndex, int vertexIndex, double dAngleValue);
}