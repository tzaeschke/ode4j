/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
package org.ode4j.cpp.internal;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.math.DVector4;
import org.ode4j.ode.DAABB;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DBox;
import org.ode4j.ode.DCapsule;
import org.ode4j.ode.DColliderFn;
import org.ode4j.ode.DContactGeomBuffer;
import org.ode4j.ode.DConvex;
import org.ode4j.ode.DCylinder;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DGeom.DNearCallback;
import org.ode4j.ode.DHeightfield;
import org.ode4j.ode.DHeightfield.DHeightfieldGetHeight;
import org.ode4j.ode.DHeightfieldData;
import org.ode4j.ode.DPlane;
import org.ode4j.ode.DRay;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DSphere;
import org.ode4j.ode.OdeHelper;
import org.ode4j.ode.internal.DxBox;
import org.ode4j.ode.internal.DxCollisionUtil;
import org.ode4j.ode.internal.cpp4j.java.RefBoolean;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;
import org.ode4j.ode.internal.cpp4j.java.RefInt;


/**
 * Collision Detection.
 *
 * ODE has two main components: a dynamics simulation engine and a collision
 * detection engine. The collision engine is given information about the
 * shape of each body. At each time step it figures out which bodies touch
 * each other and passes the resulting contact point information to the user.
 * The user in turn creates contact joints between bodies.
 *
 * Using ODE's collision detection is optional - an alternative collision
 * detection system can be used as long as it can supply the right kinds of
 * contact information.
 */
public abstract class ApiCppCollision extends ApiCppCollisionSpace {

	/* ************************************************************************ */
	/* general functions */

	/**
	 * Destroy a geom, removing it from any space.
	 *
	 * Destroy a geom, removing it from any space it is in first. This one
	 * function destroys a geom of any type, but to create a geom you must call
	 * a creation function for that type.
	 *
	 * When a space is destroyed, if its cleanup mode is 1 (the default) then all
	 * the geoms in that space are automatically destroyed as well.
	 *
	 * @param geom the geom to be destroyed.
	 */
	//ODE_API 
	public static void dGeomDestroy (DGeom geom) {
		geom.destroy();
	}


	/**
	 * Set the user-defined data pointer stored in the geom.
	 *
	 * @param geom the geom to hold the data
	 * @param data the data pointer to be stored
	 */
	//ODE_API 
	//void dGeomSetData (dGeom geom, void* data) {
	public static void dGeomSetData (DGeom geom, Object data) {
		geom.setData(data);
	}


	/**
	 * Get the user-defined data pointer stored in the geom.
	 *
	 * @param geom the geom containing the data
	 * @return user data
	 */
	//ODE_API 
	// void *dGeomGetData (dGeom geom) {
	public static Object dGeomGetData (DGeom geom) {
		return geom.getData();
	}


	/**
	 * Set the body associated with a placeable geom.
	 *
	 * Setting a body on a geom automatically combines the position vector and
	 * rotation matrix of the body and geom, so that setting the position or
	 * orientation of one will set the value for both objects. Setting a body
	 * ID of zero gives the geom its own position and rotation, independent
	 * from any body. If the geom was previously connected to a body then its
	 * new independent position/rotation is set to the current position/rotation
	 * of the body.
	 *
	 * Calling these functions on a non-placeable geom results in a runtime
	 * error in the debug build of ODE.
	 *
	 * @param geom the geom to connect
	 * @param body the body to attach to the geom
	 */
	//ODE_API 
	public static void dGeomSetBody (DGeom geom, DBody body) {
		geom.setBody(body);
	}


	/**
	 * Get the body associated with a placeable geom.
	 * @param geom the geom to query.
	 * @return body
	 * @see #dGeomSetBody(DGeom, DBody)
	 */
	//ODE_API 
	public static DBody dGeomGetBody (DGeom geom) {
		return geom.getBody();
	}


	/**
	 * Set the position vector of a placeable geom.
	 *
	 * If the geom is attached to a body, the body's position will also be changed.
	 * Calling this function on a non-placeable geom results in a runtime error in
	 * the debug build of ODE.
	 *
	 * @param geom the geom to set.
	 * @param x the new X coordinate.
	 * @param y the new Y coordinate.
	 * @param z the new Z coordinate.
	 * @see ApiCppBody#dBodySetPosition(DBody, double, double, double)
	 */
	//ODE_API 
	public static void dGeomSetPosition (DGeom geom, double x, double y, double z) {
		geom.setPosition(new DVector3(x, y, z));
	}


	/**
	 * Set the rotation matrix of a placeable geom.
	 *
	 * If the geom is attached to a body, the body's rotation will also be changed.
	 * Calling this function on a non-placeable geom results in a runtime error in
	 * the debug build of ODE.
	 *
	 * @param geom the geom to set.
	 * @param R the new rotation matrix.
	 * @see ApiCppBody#dBodySetRotation(DBody, DMatrix3)
	 */
	//ODE_API 
	public static void dGeomSetRotation (DGeom geom, final DMatrix3C R) {
		geom.setRotation(R);
	}


	/**
	 * Set the rotation of a placeable geom.
	 *
	 * If the geom is attached to a body, the body's rotation will also be changed.
	 *
	 * Calling this function on a non-placeable geom results in a runtime error in
	 * the debug build of ODE.
	 *
	 * @param geom the geom to set.
	 * @param quat the new rotation.
	 * @see ApiCppBody#dBodySetQuaternion(DBody, DQuaternion)
	 */
	//ODE_API 
	public static void dGeomSetQuaternion (DGeom geom, DQuaternionC quat) {
		geom.setQuaternion(quat);
	}


	/**
	 * Get the position vector of a placeable geom.
	 *
	 * If the geom is attached to a body, the body's position will be returned.
	 *
	 * Calling this function on a non-placeable geom results in a runtime error in
	 * the debug build of ODE.
	 *
	 * @param geom the geom to query.
	 * @return A pointer to the geom's position vector.
	 * <p>REMARK: The returned value is a pointer to the geom's internal
	 *          data structure. It is valid until any changes are made
	 *          to the geom.
	 * @see ApiCppBody#dBodyGetPosition(DBody)
	 */
	//ODE_API 
	public static final DVector3C dGeomGetPosition (DGeom geom) {
		return geom.getPosition();
	}


	/**
	 * Copy the position of a geom into a vector.
	 * 
	 * @param geom  the geom to query
	 * @param pos   a copy of the geom position
	 * @see #dGeomGetPosition(DGeom)
	 */
	//ODE_API 
	public static void dGeomCopyPosition (DGeom geom, DVector3 pos) {
		pos.set(geom.getPosition());
	}


	/**
	 * Get the rotation matrix of a placeable geom.
	 *
	 * If the geom is attached to a body, the body's rotation will be returned.
	 *
	 * Calling this function on a non-placeable geom results in a runtime error in
	 * the debug build of ODE.

	 * <p>REMARK: The returned value is a pointer to the geom's internal
	 *          data structure. It is valid until any changes are made
	 *          to the geom.
	 *
	 * @param geom the geom to query.
	 * @return A pointer to the geom's rotation matrix.
	 * @see ApiCppBody#dBodyGetRotation(DBody)
	 */
	//ODE_API 
	public static final DMatrix3C dGeomGetRotation (DGeom geom) {
		return geom.getRotation();
	}


	/**
	 * Get the rotation matrix of a placeable geom.
	 *
	 * If the geom is attached to a body, the body's rotation will be returned.
	 *
	 * Calling this function on a non-placeable geom results in a runtime error in
	 * the debug build of ODE.
	 *
	 * @param geom   the geom to query.
	 * @param R      a copy of the geom rotation
	 * @see #dGeomGetRotation(DGeom)
	 */
	//ODE_API 
	public static void dGeomCopyRotation(DGeom geom, DMatrix3 R) {
		R.set(geom.getRotation());
	}


	/**
	 * Get the rotation quaternion of a placeable geom.
	 *
	 * If the geom is attached to a body, the body's quaternion will be returned.
	 *
	 * Calling this function on a non-placeable geom results in a runtime error in
	 * the debug build of ODE.
	 *
	 * @param geom the geom to query.
	 * @param result a copy of the rotation quaternion.
	 * @see ApiCppBody#dBodyGetQuaternion(DBody)
	 */
	//ODE_API 
	public static void dGeomGetQuaternion (DGeom geom, DQuaternion result) {
		result.set(geom.getQuaternion());
	}


	/**
	 * Return the axis-aligned bounding box.
	 *
	 * Return in aabb an axis aligned bounding box that surrounds the given geom.
	 * The aabb array has elements (minx, maxx, miny, maxy, minz, maxz). If the
	 * geom is a space, a bounding box that surrounds all contained geoms is
	 * returned.
	 *
	 * This function may return a pre-computed cached bounding box, if it can
	 * determine that the geom has not moved since the last time the bounding
	 * box was computed.
	 *
	 * @param geom the geom to query
	 * @param aabb the returned bounding box
	 */
	//ODE_API 
	// void dGeomGetAABB (dGeom geom, double aabb[6]) {
	public static void dGeomGetAABB (DGeom geom, DAABB aabb) {
		aabb.set(geom.getAABB());
	}


	/**
	 * Determing if a geom is a space.
	 * @param geom the geom to query
	 * @return Non-zero if the geom is a space, zero otherwise.
	 */
	//ODE_API 
	public static boolean dGeomIsSpace (DGeom geom) {
		return geom instanceof DSpace;
	}


	/**
	 * Query for the space containing a particular geom.
	 * @param geom the geom to query
	 * @return The space that contains the geom, or NULL if the geom is
	 *          not contained by a space.
	 */
	//ODE_API 
	public static DSpace dGeomGetSpace (DGeom geom) {
		return geom.getSpace();
	}


	/**
	 * Given a geom, this returns its class.
	 *
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
	 *
	 * User-defined class will return their own number.
	 *
	 * @param geom the geom to query
	 * @return The geom class ID.
	 */
	//ODE_API 
	public static int dGeomGetClass (DGeom geom) {
		return geom.getClassID();
	}


	/**
	 * Set the "category" bitfield for the given geom.
	 *
	 * The category bitfield is used by spaces to govern which geoms will
	 * interact with each other. The bitfield is guaranteed to be at least
	 * 32 bits wide. The default category values for newly created geoms
	 * have all bits set.
	 *
	 * @param geom the geom to set
	 * @param bits the new bitfield value
	 */
	//ODE_API 
	// void dGeomSetCategoryBits (dGeom geom, unsigned long bits) {
	public static void dGeomSetCategoryBits (DGeom geom, long bits) {
		geom.setCategoryBits(bits);
	}


	/**
	 * Set the "collide" bitfield for the given geom.
	 *
	 * The collide bitfield is used by spaces to govern which geoms will
	 * interact with each other. The bitfield is guaranteed to be at least
	 * 32 bits wide. The default category values for newly created geoms
	 * have all bits set.
	 *
	 * @param geom the geom to set
	 * @param bits the new bitfield value
	 */
	//ODE_API 
	// void dGeomSetCollideBits (dGeom geom, unsigned long bits) {
	public static void dGeomSetCollideBits (DGeom geom, long bits) {
		geom.setCollideBits(bits);
	}


	/**
	 * Get the "category" bitfield for the given geom.
	 *
	 * @param geom the geom to set
	 * @return the bitfield value
	 * @see #dGeomSetCategoryBits(DGeom, long)
	 */
	//ODE_API 
	// unsigned long dGeomGetCategoryBits (dGeom geom) {
	public static long dGeomGetCategoryBits (DGeom geom) {
		return geom.getCategoryBits();
	}


	/**
	 * Get the "collide" bitfield for the given geom.
	 *
	 * @param geom the geom to set
	 * @return the bitfield value
	 * @see #dGeomSetCollideBits(DGeom, long)
	 */
	//ODE_API 
	// unsigned long dGeomGetCollideBits (dGeom geom) {
	public static long dGeomGetCollideBits (DGeom geom) {
		return geom.getCollideBits();
	}


	/**
	 * Enable a geom.
	 *
	 * Disabled geoms are completely ignored by dSpaceCollide and dSpaceCollide2,
	 * although they can still be members of a space. New geoms are created in
	 * the enabled state.
	 *
	 * @param geom   the geom to enable
	 * @see #dGeomDisable(DGeom)
	 * @see #dGeomIsEnabled(DGeom)
	 */
	//ODE_API 
	public static void dGeomEnable (DGeom geom) {
		geom.enable();
	}


	/**
	 * Disable a geom.
	 *
	 * Disabled geoms are completely ignored by dSpaceCollide and dSpaceCollide2,
	 * although they can still be members of a space. New geoms are created in
	 * the enabled state.
	 *
	 * @param geom   the geom to disable
	 * @see #dGeomDisable(DGeom)
	 * @see #dGeomIsEnabled(DGeom)
	 */
	//ODE_API 
	public static void dGeomDisable (DGeom geom) {
		geom.disable();
	}


	/**
	 * Check to see if a geom is enabled.
	 *
	 * Disabled geoms are completely ignored by dSpaceCollide and dSpaceCollide2,
	 * although they can still be members of a space. New geoms are created in
	 * the enabled state.
	 *
	 * @param geom   the geom to query
	 * @return Non-zero if the geom is enabled, zero otherwise.
	 * @see #dGeomDisable(DGeom)
	 * @see #dGeomIsEnabled(DGeom)
	 */
	//ODE_API 
	public static boolean dGeomIsEnabled (DGeom geom) {
		return geom.isEnabled();
	}

	/* ************************************************************************ */
	/* geom offset from body */

	/**
	 * Set the local offset position of a geom from its body.
	 *
	 * Sets the geom's positional offset in local coordinates.
	 * After this call, the geom will be at a new position determined from the
	 * body's position and the offset.
	 * The geom must be attached to a body.
	 * If the geom did not have an offset, it is automatically created.
	 *
	 * @param geom the geom to set.
	 * @param x the new X coordinate.
	 * @param y the new Y coordinate.
	 * @param z the new Z coordinate.
	 */
	//ODE_API 
	public static void dGeomSetOffsetPosition (DGeom geom, double x, double y, double z) {
		geom.setOffsetPosition(x, y, z);
	}
	public static void dGeomSetOffsetPosition (DGeom geom, DVector3C xyz) {
		geom.setOffsetPosition(xyz);
	}


	/**
	 * Set the local offset rotation matrix of a geom from its body.
	 *
	 * Sets the geom's rotational offset in local coordinates.
	 * After this call, the geom will be at a new position determined from the
	 * body's position and the offset.
	 * The geom must be attached to a body.
	 * If the geom did not have an offset, it is automatically created.
	 *
	 * @param geom the geom to set.
	 * @param R the new rotation matrix.
	 */
	//ODE_API 
	public static void dGeomSetOffsetRotation (DGeom geom, DMatrix3C R) {
		geom.setOffsetRotation(R);
	}


	/**
	 * Set the local offset rotation of a geom from its body.
	 *
	 * Sets the geom's rotational offset in local coordinates.
	 * After this call, the geom will be at a new position determined from the
	 * body's position and the offset.
	 * The geom must be attached to a body.
	 * If the geom did not have an offset, it is automatically created.
	 *
	 * @param geom the geom to set.
	 * @param Q the new rotation.
	 */
	//ODE_API 
	public static void dGeomSetOffsetQuaternion (DGeom geom, DQuaternionC Q) {
		geom.setOffsetQuaternion(Q);
	}


	/**
	 * Set the offset position of a geom from its body.
	 *
	 * Sets the geom's positional offset to move it to the new world
	 * coordinates.
	 * After this call, the geom will be at the world position passed in,
	 * and the offset will be the difference from the current body position.
	 * The geom must be attached to a body.
	 * If the geom did not have an offset, it is automatically created.
	 *
	 * @param geom the geom to set.
	 * @param x the new X coordinate.
	 * @param y the new Y coordinate.
	 * @param z the new Z coordinate.
	 */
	//ODE_API 
	public static void dGeomSetOffsetWorldPosition (DGeom geom, double x, double y, double z) {
		geom.setOffsetWorldPosition(x, y, z);
	}


	/**
	 * Set the offset rotation of a geom from its body.
	 *
	 * Sets the geom's rotational offset to orient it to the new world
	 * rotation matrix.
	 * After this call, the geom will be at the world orientation passed in,
	 * and the offset will be the difference from the current body orientation.
	 * The geom must be attached to a body.
	 * If the geom did not have an offset, it is automatically created.
	 *
	 * @param geom the geom to set.
	 * @param R the new rotation matrix.
	 */
	//ODE_API 
	public static void dGeomSetOffsetWorldRotation (DGeom geom, DMatrix3C R) {
		geom.setOffsetWorldRotation(R);
	}


	/**
	 * Set the offset rotation of a geom from its body.
	 *
	 * Sets the geom's rotational offset to orient it to the new world
	 * rotation matrix.
	 * After this call, the geom will be at the world orientation passed in,
	 * and the offset will be the difference from the current body orientation.
	 * The geom must be attached to a body.
	 * If the geom did not have an offset, it is automatically created.
	 *
	 * @param geom the geom to set.
	 * @param q the new rotation.
	 */
	//ODE_API 
	public static void dGeomSetOffsetWorldQuaternion (DGeom geom, DQuaternionC q) {
		geom.setOffsetWorldQuaternion(q);
	}


	/**
	 * Clear any offset from the geom.
	 *
	 * If the geom has an offset, it is eliminated and the geom is
	 * repositioned at the body's position.  If the geom has no offset,
	 * this function does nothing.
	 * This is more efficient than calling dGeomSetOffsetPosition(zero)
	 * and dGeomSetOffsetRotation(identiy), because this function actually
	 * eliminates the offset, rather than leaving it as the identity transform.
	 *
	 * @param geom the geom to have its offset destroyed.
	 */
	//ODE_API 
	public static void dGeomClearOffset(DGeom geom) {
		geom.clearOffset();
	}


	/**
	 * Check to see whether the geom has an offset.
	 *
	 * This function will return non-zero if the offset has been created.
	 * Note that there is a difference between a geom with no offset,
	 * and a geom with an offset that is the identity transform.
	 * In the latter case, although the observed behaviour is identical,
	 * there is a unnecessary computation involved because the geom will
	 * be applying the transform whenever it needs to recalculate its world
	 * position.
	 *
	 * @param geom the geom to query.
	 * @return Non-zero if the geom has an offset, zero otherwise.
	 */
	//ODE_API 
	public static boolean dGeomIsOffset(DGeom geom) {
		return geom.isOffset();
	}


	/**
	 * Get the offset position vector of a geom.
	 *
	 * Returns the positional offset of the geom in local coordinates.
	 * If the geom has no offset, this function returns the zero vector.
	 *
	 * <p>REMARK: The returned value is a pointer to the geom's internal
	 *          data structure. It is valid until any changes are made
	 *          to the geom.

	 * @param geom the geom to query.
	 * @return A pointer to the geom's offset vector.
	 */
	//ODE_API 
	public static DVector3C dGeomGetOffsetPosition (DGeom geom) {
		return geom.getOffsetPosition();
	}


	/**
	 * Copy the offset position vector of a geom.
	 *
	 * Returns the positional offset of the geom in local coordinates.
	 * If the geom has no offset, this function returns the zero vector.
	 *
	 * @param geom   the geom to query.
	 * @param pos    returns the offset position
	 */
	//ODE_API 
	public static void dGeomCopyOffsetPosition (DGeom geom, DVector3 pos) {
		pos.set(geom.getOffsetPosition());
	}


	/**
	 * Get the offset rotation matrix of a geom.
	 *
	 * Returns the rotational offset of the geom in local coordinates.
	 * If the geom has no offset, this function returns the identity
	 * matrix.
	 *
	 * <p>REMARK: The returned value is a pointer to the geom's internal
	 *          data structure. It is valid until any changes are made
	 *          to the geom.

	 * @param geom the geom to query.
	 * @return A pointer to the geom's offset rotation matrix.
	 */
	//ODE_API 
	public static DMatrix3C dGeomGetOffsetRotation (DGeom geom) {
		return geom.getOffsetRotation();
	}


	/**
	 * Copy the offset rotation matrix of a geom.
	 *
	 * Returns the rotational offset of the geom in local coordinates.
	 * If the geom has no offset, this function returns the identity
	 * matrix.
	 *
	 * @param geom   the geom to query.
	 * @param R      returns the rotation matrix.
	 */
	//ODE_API 
	public static void dGeomCopyOffsetRotation (DGeom geom, DMatrix3 R) {
		R.set(geom.getOffsetRotation());
	}


	/**
	 * Get the offset rotation quaternion of a geom.
	 *
	 * Returns the rotation offset of the geom as a quaternion.
	 * If the geom has no offset, the identity quaternion is returned.
	 *
	 * @param geom the geom to query.
	 * @param result a copy of the rotation quaternion.
	 */
	//ODE_API 
	public static void dGeomGetOffsetQuaternion (DGeom geom, DQuaternion result) {
		geom.getOffsetQuaternion(result);
	}


	/* ************************************************************************ */
	/* collision detection */

	/**
	 *
	 * Given two geoms o1 and o2 that potentially intersect,
	 * generate contact information for them.
	 *
	 * Internally, this just calls the correct class-specific collision
	 * functions for o1 and o2.
	 *
	 * <p>REMARK: If a space is passed as o1 or o2 then this function will collide
	 * all objects contained in o1 with all objects contained in o2, and return
	 * the resulting contact points. This method for colliding spaces with geoms
	 * (or spaces with spaces) provides no user control over the individual
	 * collisions. To get that control, use dSpaceCollide or dSpaceCollide2 instead.
	 *
	 * <p>REMARK: If o1 and o2 are the same geom then this function will do nothing
	 * and return 0. Technically speaking an object intersects with itself, but it
	 * is not useful to find contact points in this case.
	 *
	 * <p>REMARK: This function does not care if o1 and o2 are in the same space or not
	 * (or indeed if they are in any space at all).
	 *
	 * @param o1 The first geom to test.
	 * @param o2 The second geom to test.
	 *
	 * @param flags The flags specify how contacts should be generated if
	 * the geoms touch. The lower 16 bits of flags is an integer that
	 * specifies the maximum number of contact points to generate. You must
	 * ask for at least one contact.
	 * Additionally, following bits may be set:
	 * CONTACTS_UNIMPORTANT -- just generate any contacts (skip contact refining).
	 * All other bits in flags must be set to zero. In the future the other bits
	 * may be used to select from different contact generation strategies.
	 *
	 * @param contacts Points to an array of dContactGeom structures. The array
	 * must be able to hold at least the maximum number of contacts. These
	 * dContactGeom structures may be embedded within larger structures in the
	 * array -- the skip parameter is the byte offset from one dContactGeom to
	 * the next in the array. If skip is sizeof(dContactGeom) then contact
	 * points to a normal (C-style) array. It is an error for skip to be smaller
	 * than sizeof(dContactGeom).
	 *
	 * @return If the geoms intersect, this function returns the number of contact
	 * points generated (and updates the contact array), otherwise it returns 0
	 * (and the contact array is not touched).
	 */
	//ODE_API 
	// int dCollide (dGeom o1, dGeom o2, int flags, dContactGeom *contact,
	//	      int skip) {
	public static int dCollide (DGeom o1, DGeom o2, int flags, 
			DContactGeomBuffer contacts) {
		return OdeHelper.collide(o1, o2, flags, contacts);
	}

	/**
	 * Determines which pairs of geoms in a space may potentially intersect,
	 * and calls the callback function for each candidate pair.
	 *
	 * <p>REMARK: Other spaces that are contained within the colliding space are
	 * not treated specially, i.e. they are not recursed into. The callback
	 * function may be passed these contained spaces as one or both geom
	 * arguments.
	 *
	 * <p>REMARK: dSpaceCollide() is guaranteed to pass all intersecting geom
	 * pairs to the callback function, but may also pass close but
	 * non-intersecting pairs. The number of these calls depends on the
	 * internal algorithms used by the space. Thus you should not expect
	 * that dCollide will return contacts for every pair passed to the
	 * callback.
	 *
	 * @param space The space to test.
	 *
	 * @param data Passed from dSpaceCollide directly to the callback
	 * function. Its meaning is user defined. The o1 and o2 arguments are the
	 * geoms that may be near each other.
	 *
	 * @param callback A callback function is of type {@link DNearCallback}.
	 *
	 * @see #dSpaceCollide2(DGeom, DGeom, Object, DGeom.DNearCallback)
	 */
	//ODE_API 
	// void dSpaceCollide (dSpace space, void *data, dNearCallback *callback) {
	public static void dSpaceCollide (DSpace space, Object data, DNearCallback callback) {
		space.collide(data, callback);
	}


	/**
	 * Determines which geoms from one space may potentially intersect with
	 * geoms from another space, and calls the callback function for each candidate
	 * pair.
	 *
	 * <p>REMARK: This function can also test a single non-space geom against a
	 * space. This function is useful when there is a collision hierarchy, i.e.
	 * when there are spaces that contain other spaces.
	 *
	 * <p>REMARK: Other spaces that are contained within the colliding space are
	 * not treated specially, i.e. they are not recursed into. The callback
	 * function may be passed these contained spaces as one or both geom
	 * arguments.
	 *
	 * <p>REMARK: Sublevel value of space affects how the spaces are iterated.
	 * Both spaces are recursed only if their sublevels match. Otherwise, only
	 * the space with greater sublevel is recursed and the one with lesser sublevel
	 * is used as a geom itself.
	 *
	 * <p>REMARK: dSpaceCollide2() is guaranteed to pass all intersecting geom
	 * pairs to the callback function, but may also pass close but
	 * non-intersecting pairs. The number of these calls depends on the
	 * internal algorithms used by the space. Thus you should not expect
	 * that dCollide will return contacts for every pair passed to the
	 * callback.
	 *
	 * @param space1 The first space to test.
	 *
	 * @param space2 The second space to test.
	 *
	 * @param data Passed from dSpaceCollide directly to the callback
	 * function. Its meaning is user defined. The o1 and o2 arguments are the
	 * geoms that may be near each other.
	 *
	 * @param callback A callback function is of type {@link DNearCallback}.
	 *
	 * @see #dSpaceCollide(DSpace, Object, DGeom.DNearCallback)
	 * @see #dSpaceSetSublevel(DSpace, int)
	 */
	//ODE_API 
	// void dSpaceCollide2 (dGeom space1, dGeom space2, void *data, dNearCallback *callback) {
	public static void dSpaceCollide2 (DGeom space1, DGeom space2, Object data, DNearCallback callback) {
		OdeHelper.spaceCollide2(space1, space2, data, callback);
	}


	//TZ moved to All.java
	///* ************************************************************************ */
	///* standard classes */
	//
	///* the maximum number of user classes that are supported */
	//enum {
	//  dMaxUserClasses = 4
	//};
	//
	///* class numbers - each geometry object needs a unique number */
	//enum {
	//  dSphereClass = 0,
	//  dBoxClass,
	//  dCapsuleClass,
	//  dCylinderClass,
	//  dPlaneClass,
	//  dRayClass,
	//  dConvexClass,
	//  dGeomTransformClass,
	//  dTriMeshClass,
	//  dHeightfieldClass,
	//
	//  dFirstSpaceClass,
	//  dSimpleSpaceClass = dFirstSpaceClass,
	//  dHashSpaceClass,
	//  dSweepAndPruneSpaceClass, // SAP
	//  dQuadTreeSpaceClass,
	//  dLastSpaceClass = dQuadTreeSpaceClass,
	//
	//  dFirstUserClass,
	//  dLastUserClass = dFirstUserClass + dMaxUserClasses - 1,
	//  dGeomNumClasses
	//};


	/**
	 * Create a sphere geom of the given radius, and return its ID.
	 *
	 * <p>REMARKS:
	 * The point of reference for a sphere is its center.
	 *
	 * @param space   a space to contain the new geom. May be null.
	 * @param radius  the radius of the sphere.
	 *
	 * @return A new sphere geom.
	 *
	 * @see #dGeomDestroy(DGeom)
	 * @see #dGeomSphereSetRadius(DSphere, double)
	 */
	//ODE_API 
	public static DSphere dCreateSphere (DSpace space, double radius) {
		return OdeHelper.createSphere(space, radius);
	}


	/**
	 * Set the radius of a sphere geom.
	 *
	 * @param sphere  the sphere to set.
	 * @param radius  the new radius.
	 *
	 * @see #dGeomSphereGetRadius(DSphere)
	 */
	//ODE_API 
	public static void dGeomSphereSetRadius (DSphere sphere, double radius) {
		sphere.setRadius(radius);
	}


	/**
	 * Retrieves the radius of a sphere geom.
	 *
	 * @param sphere  the sphere to query.
	 * @return radius
	 *
	 * @see #dGeomSphereSetRadius(DSphere, double)
	 */
	//ODE_API 
	public static double dGeomSphereGetRadius (DSphere sphere) {
		return sphere.getRadius();
	}


	/**
	 * Calculate the depth of the a given point within a sphere.
	 *
	 * @param sphere  the sphere to query.
	 * @param x       the X coordinate of the point.
	 * @param y       the Y coordinate of the point.
	 * @param z       the Z coordinate of the point.
	 *
	 * @return The depth of the point. Points inside the sphere will have a
	 * positive depth, points outside it will have a negative depth, and points
	 * on the surface will have a depth of zero.
	 */
	//ODE_API 
	public static double dGeomSpherePointDepth (DSphere sphere, 
			double x, double y, double z) {
		return sphere.getPointDepth(new DVector3(x, y, z));
	}


	//--> Convex Functions
	//ODE_API 
	// dGeom dCreateConvex (dSpace space,
	//			       double *_planes,
	//			       unsigned int _planecount,
	//			       double *_points,
	//			       unsigned int _pointcount,unsigned int *_polygons) {
	public static DConvex dCreateConvex (DSpace space,
			double []planes,
			int planecount,
			double []points,
			int pointcount, int []polygons) {
		return OdeHelper.createConvex(space, planes, planecount, 
				points, pointcount, polygons);
	}

	//ODE_API 
	// void dGeomSetConvex (dGeom g,
	//			     double *_planes,
	//			     unsigned int _count,
	//			     double *_points,
	//			     unsigned int _pointcount,unsigned int *_polygons) {
	public static void dGeomSetConvex (DConvex g,
			double []_planes,
			int _count,
			double []_points,
			int _pointcount, int []_polygons) {
		g.setConvex(_planes, _count, _points, _pointcount, _polygons);
	}
	//<-- Convex Functions

	/**
	 * Create a box geom with the provided side lengths.
	 *
	 * <p>REMARKS:
	 * The point of reference for a box is its center.
	 *
	 * @param space   a space to contain the new geom. May be null.
	 * @param lx      the length of the box along the X axis
	 * @param ly      the length of the box along the Y axis
	 * @param lz      the length of the box along the Z axis
	 *
	 * @return A new box geom.
	 *
	 * @see #dGeomDestroy(DGeom)
	 * @see #dGeomBoxSetLengths(DBox, double, double, double)
	 */
	//ODE_API 
	public static DBox dCreateBox (DSpace space, double lx, double ly, double lz) {
		return OdeHelper.createBox(space, lx, ly, lz);
	}


	/**
	 * Set the side lengths of the given box.
	 *
	 * @param box  the box to set
	 * @param lx      the length of the box along the X axis
	 * @param ly      the length of the box along the Y axis
	 * @param lz      the length of the box along the Z axis
	 *
	 * @see #dGeomBoxGetLengths(DBox, DVector3)
	 */
	//ODE_API 
	public static void dGeomBoxSetLengths (DBox box, double lx, double ly, double lz) {
		box.setLengths(new DVector3(lx, ly, lz));
	}


	/**
	 * Get the side lengths of a box.
	 *
	 * @param box     the box to query
	 * @param result  the returned side lengths
	 *
	 * @see #dGeomBoxSetLengths(DBox, double, double, double)
	 */
	//ODE_API 
	public static void dGeomBoxGetLengths (DBox box, DVector3 result) {
		box.getLengths(result);
	}


	/**
	 * Return the depth of a point in a box.
	 *
	 * @param box  the box to query
	 * @param x    the X coordinate of the point to test.
	 * @param y    the Y coordinate of the point to test.
	 * @param z    the Z coordinate of the point to test.
	 *
	 * @return The depth of the point. Points inside the box will have a
	 * positive depth, points outside it will have a negative depth, and points
	 * on the surface will have a depth of zero.
	 */
	//ODE_API 
	public static double dGeomBoxPointDepth (DBox box, double x, double y, double z) {
		return box.getPointDepth(new DVector3(x, y, z));
	}


	//ODE_API 
	public static DPlane dCreatePlane (DSpace space, double a, double b, double c, double d) {
		return OdeHelper.createPlane(space, a, b, c, d);
	}
	//ODE_API 
	public static void dGeomPlaneSetParams (DPlane plane, double a, double b, double c, double d) {
		plane.setParams(a, b, c, d);
	}
	//ODE_API 
	public static void dGeomPlaneGetParams (DPlane plane, DVector4 result) {
		DVector3C t = plane.getPosition();
		result.set( t.get0(), t.get1(), t.get2(), plane.getDepth());
	}
	//ODE_API 
	public static double dGeomPlanePointDepth (DPlane plane, double x, double y, double z) {
		return plane.getPointDepth(new DVector3(x, y, z));
	}

	//ODE_API 
	public static DCapsule dCreateCapsule (DSpace space, double radius, double length) {
		return OdeHelper.createCapsule(space, radius, length);
	}
	//ODE_API 
	public static void dGeomCapsuleSetParams (DCapsule ccylinder, double radius, double length) {
		ccylinder.setParams(radius, length);
	}
	//ODE_API 
	// void dGeomCapsuleGetParams (dGeom ccylinder, double *radius, double *length) {
	public static void dGeomCapsuleGetParams (DCapsule ccylinder, RefDouble radius, 
			RefDouble length) {
		radius.d = ccylinder.getRadius();
		length.d = ccylinder.getLength();
	}
	//ODE_API 
	public static double dGeomCapsulePointDepth (DCapsule ccylinder, 
			double x, double y, double z) {
		return ccylinder.getPointDepth(new DVector3(x, y, z));
	}

	// For now we want to have a backwards compatible C-API, note: C++ API is not.
	//#define dCreateCCylinder dCreateCapsule
	//#define dGeomCCylinderSetParams dGeomCapsuleSetParams
	//#define dGeomCCylinderGetParams dGeomCapsuleGetParams
	//#define dGeomCCylinderPointDepth dGeomCapsulePointDepth
	//#define dCCylinderClass dCapsuleClass

	//ODE_API 
	public static DCylinder dCreateCylinder (DSpace space, double radius, double length) {
		return OdeHelper.createCylinder(space, radius, length);
	}
	//ODE_API 
	public static void dGeomCylinderSetParams (DCylinder cylinder, 
			double radius, double length) {
		cylinder.setParams(radius, length);
	}
	//ODE_API 
	// void dGeomCylinderGetParams (dGeom cylinder, double *radius, double *length) {
	public static void dGeomCylinderGetParams (DCylinder cylinder, 
			RefDouble radius, RefDouble length) {
		radius.d = cylinder.getRadius();
		length.d = cylinder.getLength();
	}

	//ODE_API 
	public static DRay dCreateRay (DSpace space, double length) {
		return OdeHelper.createRay(space, length);
	}
	//ODE_API 
	public static void dGeomRaySetLength (DRay ray, double length) {
		ray.setLength(length);
	}
	//ODE_API 
	public static double dGeomRayGetLength (DRay ray) {
		return ray.getLength();
	}
	//ODE_API 
	public static void dGeomRaySet (DRay ray, double px, double py, double pz,
			double dx, double dy, double dz) {
		ray.set(px, py, pz, dx, dy, dz);
	}
	//ODE_API 
	public static void dGeomRayGet (DRay ray, DVector3 start, DVector3 dir) {
		ray.get(start, dir);
	}

	/**
	 * Set/get ray flags that influence ray collision detection.
	 * These flags are currently only noticed by the trimesh collider, because
	 * they can make a major differences there.
	 * @param g ray
	 * @param firstContact first contact 
	 * @param backfaceCull cull
	 */
	//ODE_API 
	public static void dGeomRaySetParams (DRay g, 
			boolean firstContact, boolean backfaceCull) {
		g.setFirstContact(firstContact);
		g.setBackfaceCull(backfaceCull);
	}
	//ODE_API 
	// void dGeomRayGetParams (dGeom g, int *FirstContact, int *BackfaceCull) {
	public static void dGeomRayGetParams (DRay g, RefBoolean firstContact, 
			RefBoolean backfaceCull) {
		firstContact.b = g.getFirstContact();
		backfaceCull.b = g.getBackfaceCull();
	}
	//ODE_API 
	public static void dGeomRaySetClosestHit (DRay g, boolean closestHit) {
		g.setClosestHit(closestHit);
	}
	//ODE_API 
	public static boolean dGeomRayGetClosestHit (DRay g) {
		return g.getClosestHit();
	}


	/* ************************************************************************ */
	/* heightfield functions */


	// Data storage for heightfield data.
	//struct dxHeightfieldData;
	//typedef struct dxHeightfieldData* dHeightfieldData;
	//TZ moved to separate Interface.



	/**
	 * Creates a heightfield geom.
	 *
	 * Uses the information in the given dHeightfieldData to construct
	 * a geom representing a heightfield in a collision space.
	 *
	 * @param space The space to add the geom to.
	 * @param data The dHeightfieldData created by dGeomHeightfieldDataCreate and
	 * setup by dGeomHeightfieldDataBuildCallback, dGeomHeightfieldDataBuildByte,
	 * dGeomHeightfieldDataBuildShort or dGeomHeightfieldDataBuildFloat.
	 * @param bPlaceable If non-zero this geom can be transformed in the world using the
	 * usual functions such as dGeomSetPosition and dGeomSetRotation. If the geom is
	 * not set as placeable, then it uses a fixed orientation where the global y axis
	 * represents the dynamic 'height' of the heightfield.
	 *
	 * @return A geom id to reference this geom in other calls.
	 */
	//ODE_API 
	public static DGeom dCreateHeightfield( DSpace space,
			DHeightfieldData data, boolean bPlaceable ) {
		return OdeHelper.createHeightfield(space, data, bPlaceable);
	}


	/**
	 * Creates a new empty dHeightfieldData.
	 *
	 * Allocates a new dHeightfieldData and returns it. You must call
	 * dGeomHeightfieldDataDestroy to destroy it after the geom has been removed.
	 * The dHeightfieldData value is used when specifying a data format type.
	 *
	 * @return A dHeightfieldData for use with dGeomHeightfieldDataBuildCallback,
	 * dGeomHeightfieldDataBuildByte, dGeomHeightfieldDataBuildShort or
	 * dGeomHeightfieldDataBuildFloat.
	 */
	//ODE_API 
	public static DHeightfieldData dGeomHeightfieldDataCreate() {
		return OdeHelper.createHeightfieldData();
	}


	/**
	 * Destroys a dHeightfieldData.
	 *
	 * Deallocates a given dHeightfieldData and all managed resources.
	 *
	 * @param d A dHeightfieldData created by dGeomHeightfieldDataCreate
	 */
	//ODE_API 
	public static void dGeomHeightfieldDataDestroy( DHeightfieldData d ) {
		d.destroy();
	}



	/**
	 * Configures a dHeightfieldData to use a callback to
	 * retrieve height data.
	 *
	 * Before a dHeightfieldData can be used by a geom it must be
	 * configured to specify the format of the height data.
	 * This call specifies that the heightfield data is computed by
	 * the user and it should use the given callback when determining
	 * the height of a given element of it's shape.
	 *
	 * @param d A new dHeightfieldData created by dGeomHeightfieldDataCreate
	 * @param pUserData user data
	 * @param pCallback callback
	 *
	 * @param width Specifies the total 'width' of the heightfield along
	 * the geom's local x axis.
	 * @param depth Specifies the total 'depth' of the heightfield along
	 * the geom's local z axis.
	 *
	 * @param widthSamples Specifies the number of vertices to sample
	 * along the width of the heightfield. Each vertex has a corresponding
	 * height value which forms the overall shape.
	 * Naturally this value must be at least two or more.
	 * @param depthSamples Specifies the number of vertices to sample
	 * along the depth of the heightfield.
	 *
	 * @param scale A uniform scale applied to all raw height data.
	 * @param offset An offset applied to the scaled height data.
	 *
	 * @param thickness A value subtracted from the lowest height
	 * value which in effect adds an additional cuboid to the base of the
	 * heightfield. This is used to prevent geoms from looping under the
	 * desired terrain and not registering as a collision. Note that the
	 * thickness is not affected by the scale or offset parameters.
	 *
	 * @param bWrap If non-zero the heightfield will infinitely tile in both
	 * directions along the local x and z axes. If zero the heightfield is
	 * bounded from zero to width in the local x axis, and zero to depth in
	 * the local z axis.
	 */
	//ODE_API 
	// void dGeomHeightfieldDataBuildCallback( dHeightfieldData d,
	//				void* pUserData, dHeightfieldGetHeight* pCallback,
	//				double width, double depth, int widthSamples, int depthSamples,
	//				double scale, double offset, double thickness, int bWrap ) {
	public static void dGeomHeightfieldDataBuildCallback( DHeightfieldData d,
				Object[] pUserData, DHeightfieldGetHeight pCallback,
				double width, double depth, int widthSamples, int depthSamples,
				double scale, double offset, double thickness, boolean bWrap ) {
		d.buildCallback(pUserData, pCallback, width, depth, widthSamples, 
				depthSamples, scale, offset, thickness, bWrap);
	}

	/**
	 * Configures a dHeightfieldData to use height data in byte format.
	 *
	 * Before a dHeightfieldData can be used by a geom it must be
	 * configured to specify the format of the height data.
	 * This call specifies that the heightfield data is stored as a rectangular
	 * array of bytes (8 bit unsigned) representing the height at each sample point.
	 *
	 * @param d A new dHeightfieldData created by dGeomHeightfieldDataCreate
	 *
	 * @param pHeightData A pointer to the height data.
	 * @param bCopyHeightData When non-zero the height data is copied to an
	 * internal store. When zero the height data is accessed by reference and
	 * so must persist throughout the lifetime of the heightfield.
	 *
	 * @param width Specifies the total 'width' of the heightfield along
	 * the geom's local x axis.
	 * @param depth Specifies the total 'depth' of the heightfield along
	 * the geom's local z axis.
	 *
	 * @param widthSamples Specifies the number of vertices to sample
	 * along the width of the heightfield. Each vertex has a corresponding
	 * height value which forms the overall shape.
	 * Naturally this value must be at least two or more.
	 * @param depthSamples Specifies the number of vertices to sample
	 * along the depth of the heightfield.
	 *
	 * @param scale A uniform scale applied to all raw height data.
	 * @param offset An offset applied to the scaled height data.
	 *
	 * @param thickness A value subtracted from the lowest height
	 * value which in effect adds an additional cuboid to the base of the
	 * heightfield. This is used to prevent geoms from looping under the
	 * desired terrain and not registering as a collision. Note that the
	 * thickness is not affected by the scale or offset parameters.
	 *
	 * @param bWrap If non-zero the heightfield will infinitely tile in both
	 * directions along the local x and z axes. If zero the heightfield is
	 * bounded from zero to width in the local x axis, and zero to depth in
	 * the local z axis.
	 */
	//ODE_API 
	// void dGeomHeightfieldDataBuildByte( dHeightfieldData d,
	//				final unsigned char* pHeightData, int bCopyHeightData,
	//				double width, double depth, int widthSamples, int depthSamples,
	//				double scale, double offset, double thickness,	int bWrap ) {
	public static void dGeomHeightfieldDataBuildByte( DHeightfieldData d,
			final byte[] pHeightData, boolean bCopyHeightData,
			double width, double depth, int widthSamples, int depthSamples,
			double scale, double offset, double thickness, boolean bWrap ) {
		d.build( pHeightData, bCopyHeightData, width, depth, 
				widthSamples, depthSamples, scale, offset, thickness, bWrap);
	}

	/**
	 * Configures a dHeightfieldData to use height data in short format.
	 *
	 * Before a dHeightfieldData can be used by a geom it must be
	 * configured to specify the format of the height data.
	 * This call specifies that the heightfield data is stored as a rectangular
	 * array of shorts (16 bit signed) representing the height at each sample point.
	 *
	 * @param d A new dHeightfieldData created by dGeomHeightfieldDataCreate
	 *
	 * @param pHeightData A pointer to the height data.
	 * @param bCopyHeightData When non-zero the height data is copied to an
	 * internal store. When zero the height data is accessed by reference and
	 * so must persist throughout the lifetime of the heightfield.
	 *
	 * @param width Specifies the total 'width' of the heightfield along
	 * the geom's local x axis.
	 * @param depth Specifies the total 'depth' of the heightfield along
	 * the geom's local z axis.
	 *
	 * @param widthSamples Specifies the number of vertices to sample
	 * along the width of the heightfield. Each vertex has a corresponding
	 * height value which forms the overall shape.
	 * Naturally this value must be at least two or more.
	 * @param depthSamples Specifies the number of vertices to sample
	 * along the depth of the heightfield.
	 *
	 * @param scale A uniform scale applied to all raw height data.
	 * @param offset An offset applied to the scaled height data.
	 *
	 * @param thickness A value subtracted from the lowest height
	 * value which in effect adds an additional cuboid to the base of the
	 * heightfield. This is used to prevent geoms from looping under the
	 * desired terrain and not registering as a collision. Note that the
	 * thickness is not affected by the scale or offset parameters.
	 *
	 * @param bWrap If non-zero the heightfield will infinitely tile in both
	 * directions along the local x and z axes. If zero the heightfield is
	 * bounded from zero to width in the local x axis, and zero to depth in
	 * the local z axis.
	 */
	//ODE_API 
	// void dGeomHeightfieldDataBuildShort( dHeightfieldData d,
	//				final short* pHeightData, int bCopyHeightData,
	//				double width, double depth, int widthSamples, int depthSamples,
	//				double scale, double offset, double thickness, int bWrap ) {
	void dGeomHeightfieldDataBuildShort( DHeightfieldData d,
			final short[] pHeightData, boolean bCopyHeightData,
			double width, double depth, int widthSamples, int depthSamples,
			double scale, double offset, double thickness, boolean bWrap ) {
		d.build( pHeightData, bCopyHeightData, width, depth, 
				widthSamples, depthSamples, scale, offset, thickness, bWrap);
	}

	/**
	 * Configures a dHeightfieldData to use height data in
	 * single precision floating point format.
	 *
	 * Before a dHeightfieldData can be used by a geom it must be
	 * configured to specify the format of the height data.
	 * This call specifies that the heightfield data is stored as a rectangular
	 * array of single precision floats representing the height at each
	 * sample point.
	 *
	 * @param d A new dHeightfieldData created by dGeomHeightfieldDataCreate
	 *
	 * @param pHeightData A pointer to the height data.
	 * @param bCopyHeightData When non-zero the height data is copied to an
	 * internal store. When zero the height data is accessed by reference and
	 * so must persist throughout the lifetime of the heightfield.
	 *
	 * @param width Specifies the total 'width' of the heightfield along
	 * the geom's local x axis.
	 * @param depth Specifies the total 'depth' of the heightfield along
	 * the geom's local z axis.
	 *
	 * @param widthSamples Specifies the number of vertices to sample
	 * along the width of the heightfield. Each vertex has a corresponding
	 * height value which forms the overall shape.
	 * Naturally this value must be at least two or more.
	 * @param depthSamples Specifies the number of vertices to sample
	 * along the depth of the heightfield.
	 *
	 * @param scale A uniform scale applied to all raw height data.
	 * @param offset An offset applied to the scaled height data.
	 *
	 * @param thickness A value subtracted from the lowest height
	 * value which in effect adds an additional cuboid to the base of the
	 * heightfield. This is used to prevent geoms from looping under the
	 * desired terrain and not registering as a collision. Note that the
	 * thickness is not affected by the scale or offset parameters.
	 *
	 * @param bWrap If non-zero the heightfield will infinitely tile in both
	 * directions along the local x and z axes. If zero the heightfield is
	 * bounded from zero to width in the local x axis, and zero to depth in
	 * the local z axis.
	 */
	//ODE_API 
	// void dGeomHeightfieldDataBuildSingle( dHeightfieldData d,
	//				final float* pHeightData, int bCopyHeightData,
	//				double width, double depth, int widthSamples, int depthSamples,
	//				double scale, double offset, double thickness, int bWrap ) {
	void dGeomHeightfieldDataBuildSingle( DHeightfieldData d,
			final float[] pHeightData, boolean bCopyHeightData,
			double width, double depth, int widthSamples, int depthSamples,
			double scale, double offset, double thickness, boolean bWrap ) {
		d.build( pHeightData, bCopyHeightData, width, depth, 
				widthSamples, depthSamples, scale, offset, thickness, bWrap);
	}

	/**
	 * Configures a dHeightfieldData to use height data in
	 * double precision floating point format.
	 *
	 * Before a dHeightfieldData can be used by a geom it must be
	 * configured to specify the format of the height data.
	 * This call specifies that the heightfield data is stored as a rectangular
	 * array of double precision floats representing the height at each
	 * sample point.
	 *
	 * @param d A new dHeightfieldData created by dGeomHeightfieldDataCreate
	 *
	 * @param pHeightData A pointer to the height data.
	 * @param bCopyHeightData When non-zero the height data is copied to an
	 * internal store. When zero the height data is accessed by reference and
	 * so must persist throughout the lifetime of the heightfield.
	 *
	 * @param width Specifies the total 'width' of the heightfield along
	 * the geom's local x axis.
	 * @param depth Specifies the total 'depth' of the heightfield along
	 * the geom's local z axis.
	 *
	 * @param widthSamples Specifies the number of vertices to sample
	 * along the width of the heightfield. Each vertex has a corresponding
	 * height value which forms the overall shape.
	 * Naturally this value must be at least two or more.
	 * @param depthSamples Specifies the number of vertices to sample
	 * along the depth of the heightfield.
	 *
	 * @param scale A uniform scale applied to all raw height data.
	 * @param offset An offset applied to the scaled height data.
	 *
	 * @param thickness A value subtracted from the lowest height
	 * value which in effect adds an additional cuboid to the base of the
	 * heightfield. This is used to prevent geoms from looping under the
	 * desired terrain and not registering as a collision. Note that the
	 * thickness is not affected by the scale or offset parameters.
	 *
	 * @param bWrap If non-zero the heightfield will infinitely tile in both
	 * directions along the local x and z axes. If zero the heightfield is
	 * bounded from zero to width in the local x axis, and zero to depth in
	 * the local z axis.
	 */
	//ODE_API 
	// void dGeomHeightfieldDataBuildDouble( dHeightfieldData d,
	//				final double* pHeightData, int bCopyHeightData,
	//				double width, double depth, int widthSamples, int depthSamples,
	//				double scale, double offset, double thickness, int bWrap ) {
	void dGeomHeightfieldDataBuildDouble( DHeightfieldData d,
			final double[] pHeightData, boolean bCopyHeightData,
			double width, double depth, int widthSamples, int depthSamples,
			double scale, double offset, double thickness, boolean bWrap ) {
		d.build( pHeightData, bCopyHeightData, width, depth, 
				widthSamples, depthSamples, scale, offset, thickness, bWrap);
	}

	/**
	 * Manually set the minimum and maximum height bounds.
	 *
	 * This call allows you to set explicit min / max values after initial
	 * creation typically for callback heightfields which default to +/- infinity,
	 * or those whose data has changed. This must be set prior to binding with a
	 * geom, as the the AABB is not recomputed after it's first generation.
	 *
	 * <p>REMARKS:
	 * The minimum and maximum values are used to compute the AABB
	 * for the heightfield which is used for early rejection of collisions.
	 * A close fit will yield a more efficient collision check.
	 *
	 * @param d A dHeightfieldData created by dGeomHeightfieldDataCreate
	 * @param minHeight The new minimum height value. Scale, offset and thickness is then applied.
	 * @param maxHeight The new maximum height value. Scale and offset is then applied.
	 */
	//ODE_API 
	public static void dGeomHeightfieldDataSetBounds( DHeightfieldData d,
			double minHeight, double maxHeight ) {
		d.setBounds(minHeight, maxHeight);
	}


	/**
	 * Assigns a dHeightfieldData to a heightfield geom.
	 *
	 * Associates the given dHeightfieldData with a heightfield geom.
	 * This is done without affecting the GEOM_PLACEABLE flag.
	 *
	 * @param g A geom created by dCreateHeightfield
	 * @param d A dHeightfieldData created by dGeomHeightfieldDataCreate
	 */
	//ODE_API 
	public static void dGeomHeightfieldSetHeightfieldData( DHeightfield g, DHeightfieldData d ) {
		g.setHeightfieldData(d);
	}


	/**
	 * Gets the dHeightfieldData bound to a heightfield geom.
	 *
	 * Returns the dHeightfieldData associated with a heightfield geom.
	 *
	 * @param g A geom created by dCreateHeightfield
	 * @return The dHeightfieldData which may be NULL if none was assigned.
	 */
	//ODE_API 
	public static DHeightfieldData dGeomHeightfieldGetHeightfieldData( DHeightfield g ) {
		return g.getHeightfieldData();
	}



	/* ************************************************************************ */
	/* utility functions */

	//ODE_API 
	void dClosestLineSegmentPoints (final DVector3 a1, final DVector3 a2,
			final DVector3 b1, final DVector3 b2,
			DVector3 cp1, DVector3 cp2) {
		throw new UnsupportedOperationException();
	}

	//ODE_API 
	public static boolean dBoxTouchesBox (final DVector3 p1, final DMatrix3 R1,
			final DVector3 side1, final DVector3 p2,
			final DMatrix3 R2, final DVector3 side2) {
		return DxCollisionUtil.dBoxTouchesBox(p1, R1, side1, p2, R2, side2);
	}

	// The meaning of flags parameter is the same as in dCollide()
	//ODE_API 
	// int dBoxBox (final dVector3 p1, final dMatrix3 R1,
	//	     final dVector3 side1, final dVector3 p2,
	//	     final dMatrix3 R2, final dVector3 side2,
	//	     dVector3 normal, double *depth, int *return_code,
	//	     int flags, dContactGeom *contact, int skip) {
	public static int dBoxBox (final DVector3 p1, final DMatrix3 R1,
			final DVector3 side1, final DVector3 p2,
			final DMatrix3 R2, final DVector3 side2,
			DVector3 normal, RefDouble depth, RefInt return_code,
			int flags, DContactGeomBuffer contacts) {
		return DxBox.dBoxBox(p1, R1, side1, p2, R2, side2, normal, 
				depth, return_code, flags, contacts, 1);
	}

	//ODE_API 
	// void dInfiniteAABB (dGeom geom, double aabb[6]) {
	void dInfiniteAABB (DGeom geom, DAABB aabb) {
		throw new UnsupportedOperationException();
	}


	/* ************************************************************************ */
	/* custom classes */

	//typedef void dGetAABBFn(dGeom g, double aabb[6])
	public static interface dGetAABBFn {
		void run(DGeom g, DAABB aabb);
	}
	//typedef int dColliderFn (dGeom o1, dGeom o2,
	//			 int flags, dContactGeom *contact, int skip) {
//	public static interface dColliderFn {
//		int dColliderFn (dGeom o1, dGeom o2,
//				int flags, dContactGeomBuffer contacts);
//	}
	//typedef dColliderFn * dGetColliderFnFn (int num) {
	public static interface dGetColliderFnFn {
		DColliderFn  run (int num);
	}
	//typedef void dGeomDtorFn (dGeom o) {
	public static interface dGeomDtorFn {
		void run (DGeom o);
	}
	//typedef int dAABBTestFn (dGeom o1, dGeom o2, double aabb[6]) {
	public static interface dAABBTestFn {
		boolean run (DGeom o1, DGeom o2, DAABB aabb);
	}

	//TZ moved to dxGeom
	//typedef struct dGeomClass {
	//  int bytes;
	//  dGetColliderFnFn *collider;
	//  dGetAABBFn *aabb;
	//  dAABBTestFn *aabb_test;
	//  dGeomDtorFn *dtor;
	//} dGeomClass;
	public static class dGeomClass {
		public int bytes;
		public dGetColliderFnFn collider;
		public dGetAABBFn aabb;
		public dAABBTestFn aabb_test;
		public dGeomDtorFn dtor;
		public void dtor(DGeom o) {
			dtor.run(o);
		}
		public DColliderFn collider(int num) {
			return collider.run(num);
		}
		public void aabb(DGeom g, DAABB aabb) {
			this.aabb.run(g, aabb);
		}
		public boolean aabb_test(DGeom o1, DGeom o2, DAABB aabb) {
			return aabb_test.run(o1, o2, aabb);
		}

		public dGeomClass() {}
	}

	//ODE_API 
	// int dCreateGeomClass (final dGeomClass *classptr) {
	int dCreateGeomClass (final Class<? extends dGeomClass> classptr) {
		throw new UnsupportedOperationException();
	}
	//ODE_API 
	// void * dGeomGetClassData (dGeom) {
	Object[] dGeomGetClassData (DGeom g) {
		throw new UnsupportedOperationException();
	}
	//ODE_API 
	DGeom dCreateGeom (int classnum) {
		throw new UnsupportedOperationException();
	}

	/**
	 * Sets a custom collider function for two geom classes.
	 *
	 * @param i The first geom class handled by this collider
	 * @param j The second geom class handled by this collider
	 * @param fn The collider function to use to determine collisions.
	 */
	//ODE_API 
	// void dSetColliderOverride (int i, int j, dColliderFn *fn) {
	public static void dSetColliderOverride (int i, int j, DColliderFn fn) {
		OdeHelper.setColliderOverride(i, j, fn);
	}

	protected ApiCppCollision() {}
}
