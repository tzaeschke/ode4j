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
package org.ode4j.ode;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;

/**
 * Common base class for all geometries.
 *
 */
public interface DGeom {

	/** the maximum number of user classes that are supported. */
	//	enum {
	public static final int dMaxUserClasses = 4;
	//	};

	/** 
	 * class numbers - each geometry object needs a unique number.
	 * (TZ) This has not been made an 'enum' to leave it easily extensible
	 * for user defined classes.
	 */
	//	enum {
	public static final int   dSphereClass = 0;
	public static final int   dBoxClass = 1;
	public static final int   dCapsuleClass = 2;
	public static final int   dCylinderClass = 3;
	public static final int   dPlaneClass = 4;
	public static final int   dRayClass = 5;
	public static final int   dConvexClass = 6;
	public static final int   dGeomTransformClass = 7;
	public static final int   dTriMeshClass = 8;
	public static final int   dHeightfieldClass = 9;

	public static final int   dFirstSpaceClass = 10;
	public static final int   dSimpleSpaceClass = dFirstSpaceClass; //10
	public static final int   dHashSpaceClass = 11;
	public static final int   dSweepAndPruneSpaceClass = 12;
	public static final int   dQuadTreeSpaceClass = 13;
	public static final int   dBVHSpaceClass = 14;
	public static final int   dLastSpaceClass = dBVHSpaceClass; //14

	/** 
	 * ID of the first user defined class. 
	 * <b>These IDs may change between software releases!</b><br>
	 * To avoid problem, user defined classes should have their IDs allocated 
	 * based on <tt>dFirstUserClass</tt>, for example: 
	 * <tt>int myCLassID = dFirstUserClass + x;</tt>, where (x &ge; 1). 
	 */
	public static final int   dFirstUserClass = 14;
	public static final int   dLastUserClass = dFirstUserClass + dMaxUserClasses - 1;
	public static final int   dGeomNumClasses = dLastUserClass + 1; 
	//	};



	void DESTRUCTOR();

	/**
	 * Destroy a geom, removing it from any space.
	 * <p>
	 * Destroy a geom, removing it from any space it is in first. This one
	 * function destroys a geom of any type, but to create a geom you must call
	 * a creation function for that type.
	 * <p>
	 * When a space is destroyed, if its cleanup mode is 1 (the default) then all
	 * the geoms in that space are automatically destroyed as well.
	 */
	void destroy();

	
	/**
	 * Given a geom, this returns its class.
	 * <p>
	 * The ODE classes are:
	 * <ul>
	 *  <li> dSphereClass </li>
	 *  <li> dBoxClass </li>
	 *  <li> dCylinderClass </li>
	 *  <li> dPlaneClass </li>
	 *  <li> dRayClass </li>
	 *  <li> dConvexClass </li>
	 *  <li> dGeomTransformClass </li>
	 *  <li> dTriMeshClass </li>
	 *  <li> dSimpleSpaceClass </li>
	 *  <li> dHashSpaceClass </li>
	 *  <li> dQuadTreeSpaceClass </li>
	 *  <li> dFirstUserClass </li>
	 *  <li> dLastUserClass </li>
	 * </ul>
	 * <p>
	 * User-defined class will return their own number.
	 *
	 * @return The geom class ID.
	 */
	int getClassID();

	
	/**
	 * Query for the space containing a particular geom.
	 * 
	 * @return The space that contains the geom, or NULL if the geom is
	 *          not contained by a space.
	 */
	DSpace getSpace();

	
	/**
	 * Set the user-defined data pointer stored in the geom.
	 *
	 * @param data the data pointer to be stored
	 */
	void setData (Object data);

	
	/**
	 * Get the user-defined data pointer stored in the geom.
	 * @return user data
	 */
	Object getData();

	
	/**
	 * Set the body associated with a placeable geom.
	 * <p>
	 * Setting a body on a geom automatically combines the position vector and
	 * rotation matrix of the body and geom, so that setting the position or
	 * orientation of one will set the value for both objects. Setting a body
	 * ID of zero gives the geom its own position and rotation, independent
	 * from any body. If the geom was previously connected to a body then its
	 * new independent position/rotation is set to the current position/rotation
	 * of the body.
	 * <p>
	 * Calling these functions on a non-placeable geom results in a runtime
	 * error in the debug build of ODE.
	 *
	 * @param body the body to attach to the geom
	 */
	void setBody (DBody body);

	
	/**
	 * Get the body associated with a placeable geom.
	 * @return body object
	 * @see #setBody(DBody)
	 */
	DBody getBody();

	
	/**
	 * Set the position vector of a placeable geom.
	 * <p>
	 * If the geom is attached to a body, the body's position will also be changed.
	 * Calling this function on a non-placeable geom results in a runtime error in
	 * the debug build of ODE.
	 *
	 * @param x the new X coordinate.
	 * @param y the new Y coordinate.
	 * @param z the new Z coordinate.
	 * @see DBody#setPosition(double, double, double)
	 */
	void setPosition (double x, double y, double z);
	
	
	/**
	 * Set the position vector of a placeable geom.
	 * <p>
	 * If the geom is attached to a body, the body's position will also be changed.
	 * Calling this function on a non-placeable geom results in a runtime error in
	 * the debug build of ODE.
	 *
	 * @param xyz the new X, Y and Z coordinate.
	 * @see DBody#setPosition(DVector3C)
	 */
	void setPosition (DVector3C xyz);


	/**
	 * Get the position vector of a placeable geom.
	 * <p>
	 * If the geom is attached to a body, the body's position will be returned.
	 * <p>
	 * Calling this function on a non-placeable geom results in a runtime error in
	 * the debug build of ODE.
	 * 
	 * <p>REMARK: The returned value is a pointer to the geom's internal
	 *          data structure. It is valid until any changes are made
	 *          to the geom.
	 *
	 * @return A pointer to the geom's position vector.
	 * @see DBody#getPosition()
	 */
	DVector3C getPosition();

	
	/**
	 * Copy the position of a geom into a vector.
	 * @param pos   a copy of the geom position
	 * @see #getPosition()
	 */
	void copyPosition (DVector3 pos);

	/**
	 * Set the rotation matrix of a placeable geom.
	 * <p>
	 * If the geom is attached to a body, the body's rotation will also be changed.
	 * Calling this function on a non-placeable geom results in a runtime error in
	 * the debug build of ODE.
	 *
	 * @param R the new rotation matrix.
	 * @see DBody#setRotation(DMatrix3C)
	 */
	void setRotation (DMatrix3C R);
	
	
	/**
	 * Get the rotation matrix of a placeable geom.
	 * <p>
	 * If the geom is attached to a body, the body's rotation will be returned.
	 * <p>
	 * Calling this function on a non-placeable geom results in a runtime error in
	 * the debug build of ODE.
	 * 
	 * <p>REMARK: The returned value is a pointer to the geom's internal
	 *          data structure. It is valid until any changes are made
	 *          to the geom.
	 *
	 * @return A pointer to the geom's rotation matrix.
	 * @see DBody#getRotation()
	 */
	DMatrix3C getRotation();
	
	
	/**
	 * Get the rotation matrix of a placeable geom.
	 * <p>
	 * If the geom is attached to a body, the body's rotation will be returned.
	 * <p>
	 * Calling this function on a non-placeable geom results in a runtime error in
	 * the debug build of ODE.
	 *
	 * @param R      a copy of the geom rotation
	 * @see #getRotation()
	 */
	void copyRotation(DMatrix3 R);


	/**
	 * Set the rotation of a placeable geom.
	 * <p>
	 * If the geom is attached to a body, the body's rotation will also be changed.
	 * <p>
	 * Calling this function on a non-placeable geom results in a runtime error in
	 * the debug build of ODE.
	 *
	 * @param quat the new rotation.
	 * @see DBody#setQuaternion(DQuaternionC)
	 */
	void setQuaternion (DQuaternionC quat);

	
	/**
	 * Get the rotation quaternion of a placeable geom.
	 * <p>
	 * If the geom is attached to a body, the body's quaternion will be returned.
	 * <p>
	 * Calling this function on a non-placeable geom results in a runtime error in
	 * the debug build of ODE.
	 * @return Rotation quaternion
	 *
	 * @see DBody#getQuaternion()
	 */
	DQuaternionC getQuaternion ();

	
	/**
	 * Return the axis-aligned bounding box.
	 * <p>
	 * Return in aabb an axis aligned bounding box that surrounds the given geom.
	 * The aabb array has elements (minx, maxx, miny, maxy, minz, maxz). If the
	 * geom is a space, a bounding box that surrounds all contained geoms is
	 * returned.
	 * <p>
	 * This function may return a pre-computed cached bounding box, if it can
	 * determine that the geom has not moved since the last time the bounding
	 * box was computed.
	 * @return bounding box
	 */
	DAABBC getAABB ();

	
	/**
	 * Determing if a geom is a space.
	 * @return Non-zero if the geom is a space, zero otherwise.
	 */
	boolean isSpace();

	
	/**
	 * Set the "category" bitfield for the given geom.
	 * <p>
	 * The category bitfield is used by spaces to govern which geoms will
	 * interact with each other. The bitfield is guaranteed to be at least
	 * 32 bits wide. The default category values for newly created geoms
	 * have all bits set.
	 *
	 * @param bits the new bitfield value
	 */
	void setCategoryBits (long bits);//unsigned long bits)
	
	
	/**
	 * Set the "collide" bitfield for the given geom.
	 * <p>
	 * The collide bitfield is used by spaces to govern which geoms will
	 * interact with each other. The bitfield is guaranteed to be at least
	 * 32 bits wide. The default category values for newly created geoms
	 * have all bits set.
	 *
	 * @param bits the new bitfield value
	 */
	void setCollideBits (long bits);//unsigned long bits)
	
	
	/**
	 * Get the "category" bitfield for the given geom.
	 *
	 * @return The bitfield value
	 * @see #setCategoryBits(long)
	 */
	//unsigned 
	long getCategoryBits();
	
	
	/**
	 * Get the "collide" bitfield for the given geom.
	 *
	 * @return The bitfield value
	 * @see #setCollideBits(long)
	 */
	//unsigned 
	long getCollideBits();

	
	/**
	 * Enable a geom.
	 * <p>
	 * Disabled geoms are completely ignored by dSpaceCollide and dSpaceCollide2,
	 * although they can still be members of a space. New geoms are created in
	 * the enabled state.
	 *
	 * @see #disable()
	 * @see #isEnabled()
	 */
	void enable();
	
	
	/**
	 * Disable a geom.
	 * <p>
	 * Disabled geoms are completely ignored by dSpaceCollide and dSpaceCollide2,
	 * although they can still be members of a space. New geoms are created in
	 * the enabled state.
	 *
	 * @see #enable()
	 * @see #isEnabled()
	 */
	void disable();
	
	
	/**
	 * Check to see if a geom is enabled.
	 * <p>
	 * Disabled geoms are completely ignored by dSpaceCollide and dSpaceCollide2,
	 * although they can still be members of a space. New geoms are created in
	 * the enabled state.
	 *
	 * @return Non-zero if the geom is enabled, zero otherwise.
	 * @see #disable()
	 * @see #enable()
	 */
	boolean isEnabled();

	
	/**
	 * User callback for geom-geom collision testing.
	 *
	 * <p>REMARK: The callback function can call dCollide on o1 and o2 to generate
	 * contact points between each pair. Then these contact points may be added
	 * to the simulation as contact joints. The user's callback function can of
	 * course chose not to call dCollide for any pair, e.g. if the user decides
	 * that those pairs should not interact.
	 */
	//typedef void dNearCallback (void *data, dGeom o1, dGeom o2);
	interface DNearCallback {
	    /**
	     * @param data The user data object, as passed to dSpaceCollide.
	     * @param o1   The first geom being tested.
	     * @param o2   The second geom being test.
	     */
		public void call (Object data, DGeom o1, DGeom o2);
	}

	void collide2 (DGeom g, Object data, DNearCallback callback);


	

//	enum CONTROL_CLASS {
//	    dGeomCommonControlClass,// = 0,
//	    dGeomColliderControlClass;// = 1
//	}
//
//	enum CONTROL_CODE {
//	    dGeomCommonAnyControlCode,// = 0,
//	    dGeomColliderSetMergeSphereContactsControlCode,// = 1,
//	    dGeomColliderGetMergeSphereContactsControlCode;// = 2
//	}
//
//	enum COLLIDER_MERGE_CONTACTS_VALUE {
//	    dGeomColliderMergeContactsValue__Default,// = 0, // Used with Set... to restore default value
//	    dGeomColliderMergeContactsValue_None,// = 1,
//	    dGeomColliderMergeContactsValue_Normals,// = 2,
//	    dGeomColliderMergeContactsValue_Full;// = 3
//	}
//
//	/**
//	 * 
//	 *
//	 * @author Tilmann Zaeschke
//	 * @deprecated not implemented yet.
//	 */
//	static interface DataValue {};
//	
//	/**
//	 * Execute low level control operation for geometry.
//	 *
//	 * The variable the dataSize points to must be initialized before the call.
//	 * If the size does not match the one expected for the control class/code function
//	 * changes it to the size expected and returns failure. This implies the function 
//	 * can be called with NULL data and zero size to test if control class/code is supported
//	 * and obtain required data size for it.
//	 * <p>
//	 * dGeomCommonAnyControlCode applies to any control class and returns success if 
//	 * at least one control code is available for the given class with given geom.
//	 * <br>
//	 * Currently there are the folliwing control classes supported:<br>
//	 * <li> dGeomColliderControlClass</li>
//	 * <p>
//	 * For dGeomColliderControlClass there are the following codes available:<br>
//	 *  <li> dGeomColliderSetMergeSphereContactsControlCode (arg of type int, dGeomColliderMergeContactsValue_*)</li>
//	 *  <li> dGeomColliderGetMergeSphereContactsControlCode (arg of type int, dGeomColliderMergeContactsValue_*)</li>
//	 * 
//	 * @param controlClass   the control class
//	 * @param controlCode   the control code for the class
//	 * @param dataValue   the control argument pointer
//	 * @param dataSize   the control argument size provided or expected
//	 * @return Boolean execution status
//	 */
//	boolean lowLevelControl(CONTROL_CLASS controlClass, CONTROL_CODE controlCode, 
//			DataValue dataValue, RefInt dataSize);


	/**
	 * Get world position of a relative point on geom.
	 *
	 * Calling this function on a non-placeable geom results in the same point being
	 * returned.
	 * @param px px
	 * @param py py
	 * @param pz pz
	 *
	 * @param result will contain the result.
	 */
	void getRelPointPos(double px, double py, double pz, DVector3 result);

	/**
	 * Takes a point in global coordinates and returns
	 * the point's position in geom-relative coordinates.
	 *
	 * Calling this function on a non-placeable geom results in the same point being
	 * returned.
	 *
	 * <p>REMARK:
	 * This is the inverse of dGeomGetRelPointPos()
	 * 
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * @param result will contain the result.
	 */
	void getPosRelPoint(double px, double py, double pz, DVector3 result);

	/**
	 * Convert from geom-local to world coordinates.
	 *
	 * Calling this function on a non-placeable geom results in the same vector being
	 * returned.
	 *
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * @param result will contain the result.
	 */
	void vectorToWorld(double px, double py, double pz, DVector3 result);

	/**
	 * Convert from world to geom-local coordinates.
	 *
	 * Calling this function on a non-placeable geom results in the same vector being
	 * returned.
	 *
	 * @param px px
	 * @param py py
	 * @param pz pz
	 * @param result will contain the result.
	 */
	void vectorFromWorld(double px, double py, double pz, DVector3 result);

	
	/**
	 * Get the offset position vector of a geom.
	 * <p>
	 * Returns the positional offset of the geom in local coordinates.
	 * If the geom has no offset, this function returns the zero vector.
	 * 
	 * <p>REMARK: The returned value is a pointer to the geom's internal
	 *          data structure. It is valid until any changes are made
	 *          to the geom.
	 *
	 * @return A pointer to the geom's offset vector.
	 */
	DVector3C getOffsetPosition();

	
	/**
	 * Copy the offset position vector of a geom.
	 * <p>
	 * Returns the positional offset of the geom in local coordinates.
	 * If the geom has no offset, this function returns the zero vector.
	 *
	 * @param pos    returns the offset position
	 */
	void copyOffsetPosition (DVector3 pos);
	
	
	/**
	 * Set the local offset position of a geom from its body.
	 * <p>
	 * Sets the geom's positional offset in local coordinates.
	 * After this call, the geom will be at a new position determined from the
	 * body's position and the offset.
	 * The geom must be attached to a body.
	 * If the geom did not have an offset, it is automatically created.
	 *
	 * @param x the new X coordinate.
	 * @param y the new Y coordinate.
	 * @param z the new Z coordinate.
	 */
	void setOffsetPosition(double x, double y, double z);
	/**
	 * 
	 * @param xyz xyz
	 * @see #setOffsetPosition(double, double, double) 
	 */
	void setOffsetPosition(DVector3C xyz);

	
	/**
	 * Get the offset rotation matrix of a geom.
	 * <p>
	 * Returns the rotational offset of the geom in local coordinates.
	 * If the geom has no offset, this function returns the identity
	 * matrix.
	 * 
	 * <p>REMARK: The returned value is a pointer to the geom's internal
	 *          data structure. It is valid until any changes are made
	 *          to the geom.
	 *
	 * @return A pointer to the geom's offset rotation matrix.
	 */
	DMatrix3C getOffsetRotation();
	
	
	/**
	 * Copy the offset rotation matrix of a geom.
	 * <p>
	 * Returns the rotational offset of the geom in local coordinates.
	 * If the geom has no offset, this function returns the identity
	 * matrix.
	 *
	 * @param R      returns the rotation matrix.
	 */
	void copyOffsetRotation (DMatrix3 R);

	
	/**
	 * Set the local offset rotation matrix of a geom from its body.
	 * <p>
	 * Sets the geom's rotational offset in local coordinates.
	 * After this call, the geom will be at a new position determined from the
	 * body's position and the offset.
	 * The geom must be attached to a body.
	 * If the geom did not have an offset, it is automatically created.
	 *
	 * @param R the new rotation matrix.
	 */
	void setOffsetRotation(DMatrix3C R);
	
	
	/**
	 * Get the offset rotation quaternion of a geom.
	 *
	 * Returns the rotation offset of the geom as a quaternion.
	 * If the geom has no offset, the identity quaternion is returned.
	 *
	 * @param result a copy of the rotation quaternion.
	 */
	void getOffsetQuaternion(DQuaternion result);

	
	/**
	 * Set the local offset rotation of a geom from its body.
	 * <p>
	 * Sets the geom's rotational offset in local coordinates.
	 * After this call, the geom will be at a new position determined from the
	 * body's position and the offset.
	 * The geom must be attached to a body.
	 * If the geom did not have an offset, it is automatically created.
	 *
	 * @param q the new rotation.
	 */
	void setOffsetQuaternion(DQuaternionC q);

	
	/**
	 * Set the offset position of a geom from its body.
	 * <p>
	 * Sets the geom's positional offset to move it to the new world
	 * coordinates.
	 * After this call, the geom will be at the world position passed in,
	 * and the offset will be the difference from the current body position.
	 * The geom must be attached to a body.
	 * If the geom did not have an offset, it is automatically created.
	 *
	 * @param x the new X coordinate.
	 * @param y the new Y coordinate.
	 * @param z the new Z coordinate.
	 */
	void setOffsetWorldPosition(double x, double y, double z);

	
	/**
	 * Set the offset rotation of a geom from its body.
	 * <p>
	 * Sets the geom's rotational offset to orient it to the new world
	 * rotation matrix.
	 * After this call, the geom will be at the world orientation passed in,
	 * and the offset will be the difference from the current body orientation.
	 * The geom must be attached to a body.
	 * If the geom did not have an offset, it is automatically created.
	 *
	 * @param R the new rotation matrix.
	 */
	void setOffsetWorldRotation(DMatrix3C R);
	
	
	/**
	 * Set the offset rotation of a geom from its body.
	 * <p>
	 * Sets the geom's rotational offset to orient it to the new world
	 * rotation matrix.
	 * After this call, the geom will be at the world orientation passed in,
	 * and the offset will be the difference from the current body orientation.
	 * The geom must be attached to a body.
	 * If the geom did not have an offset, it is automatically created.
	 *
	 * @param q the new rotation.
	 */
	void setOffsetWorldQuaternion(DQuaternionC q);
	
	
	/**
	 * Clear any offset from the geom.
	 * <p>
	 * If the geom has an offset, it is eliminated and the geom is
	 * repositioned at the body's position.  If the geom has no offset,
	 * this function does nothing.
	 * This is more efficient than calling dGeomSetOffsetPosition(zero)
	 * and dGeomSetOffsetRotation(identiy), because this function actually
	 * eliminates the offset, rather than leaving it as the identity transform.
	 *
	 */
	void clearOffset();
	
	
	/**
	 * Check to see whether the geom has an offset.
	 * <p>
	 * This function will return non-zero if the offset has been created.
	 * Note that there is a difference between a geom with no offset,
	 * and a geom with an offset that is the identity transform.
	 * In the latter case, although the observed behaviour is identical,
	 * there is a unnecessary computation involved because the geom will
	 * be applying the transform whenever it needs to recalculate its world
	 * position.
	 *
	 * @return Non-zero if the geom has an offset, zero otherwise.
	 */
	boolean isOffset();
}
