/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
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
package org.ode4j.ode;

import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3C;

public interface DGeom {

	// ******** TZ from collision.h
	/* ************************************************************************ */
	/* standard classes */

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
	public static final int   dLastSpaceClass = dQuadTreeSpaceClass; //13

	/** 
	 * ID of the first user defined class. 
	 * <b>These IDs may change between software releases!</b><br>
	 * To avoid problem, user defined classes should have their IDs allocated 
	 * based on <tt>dFirstUserClass</tt>, for example: 
	 * <tt>int myCLassID = dFirstUserClass + x;</tt>, where (x >= 1). 
	 */
	public static final int   dFirstUserClass = 14;
	public static final int   dLastUserClass = dFirstUserClass + dMaxUserClasses - 1;
	public static final int   dGeomNumClasses = dLastUserClass + 1; 
	//	};



	void DESTRUCTOR();

	void destroy();

	int getClassID();

	DSpace getSpace();

	void setData (Object data);
	Object getData();

	void setBody (DBody b);
	DBody getBody();

	void setPosition (double x, double y, double z);
	void setPosition (DVector3C xyz);
	//const dReal * getPosition() const
	DVector3C getPosition();

	void setRotation (final DMatrix3 R);
	//const dReal * getRotation() const
	DMatrix3C getRotation();

	void setQuaternion (final DQuaternion quat);
	DQuaternionC getQuaternion ();

	DAABBC getAABB ();

	//	int isSpace();

	void setCategoryBits (long bits);//unsigned long bits)
	void setCollideBits (long bits);//unsigned long bits)
	//unsigned 
	long getCategoryBits();
	//unsigned 
	long getCollideBits();

	void enable();
	void disable();
	boolean isEnabled();

	/**
	 * @brief User callback for geom-geom collision testing.
	 *
	 * @param data The user data object, as passed to dSpaceCollide.
	 * @param o1   The first geom being tested.
	 * @param o2   The second geom being test.
	 *
	 * @remarks The callback function can call dCollide on o1 and o2 to generate
	 * contact points between each pair. Then these contact points may be added
	 * to the simulation as contact joints. The user's callback function can of
	 * course chose not to call dCollide for any pair, e.g. if the user decides
	 * that those pairs should not interact.
	 *
	 * @ingroup collide
	 */
	//typedef void dNearCallback (void *data, dGeom o1, dGeom o2);
	public interface DNearCallback {
		public void call (Object data, DGeom o1, DGeom o2);
	}

	void collide2 (DGeom g, Object data, DNearCallback callback);

	void setOffsetPosition(double d, double e, double f);

	void setOffsetRotation(DMatrix3C matrix3);



	//	protected
	//		dGeom _id;
	//
	//	public
	//		dGeom()
	//		{ _id = null; }
	//	//~dGeom()
	//	public void DESTRUCTOR()
	//	{ if (_id!=null) dGeomDestroy (_id); }
	//
	//	dGeom id() //const
	//	{ return _id; }
	//	//TODO TZ ?
	////	operator dGeom() //const
	////	{ return _id; }
	//
	//	void destroy() {
	//		if (_id!=null) dGeomDestroy (_id);
	//		_id = null;
	//	}
	//
	//	int getClass()// const
	//	{ return dGeomGetClass (_id); }
	//
	//	dSpace getSpace() //const
	//	{ return dGeomGetSpace (_id); }
	//
	//	void setData (Object data)
	//	{ dGeomSetData (_id,data); }
	//	Object getData() //const
	//	{ return dGeomGetData (_id); }
	//
	//	void setBody (dBody b)
	//	{ dGeomSetBody (_id,b); }
	//	dBody getBody() //const
	//	{ return dGeomGetBody (_id); }
	//
	//	void setPosition (dReal x, dReal y, dReal z)
	//	{ dGeomSetPosition (_id,x,y,z); }
	//	//const dReal * getPosition() const
	//	double getPosition()
	//	{ return dGeomGetPosition (_id); }
	//
	//	void setRotation (final dMatrix3 R)
	//	{ dGeomSetRotation (_id,R); }
	//	//const dReal * getRotation() const
	//	dMatrix3 getRotation()
	//	{ return dGeomGetRotation (_id); }
	//
	//	void setQuaternion (final dQuaternion quat)
	//	{ dGeomSetQuaternion (_id,quat); }
	//	void getQuaternion (dQuaternion quat) //const
	//	{ dGeomGetQuaternion (_id,quat); }
	//
	//	void getAABB (double[] aabb) //const
	//	{ dGeomGetAABB (_id, aabb); }
	//
	//	int isSpace()
	//	{ return dGeomIsSpace (_id); }
	//
	//	void setCategoryBits (long bits)//unsigned long bits)
	//	{ dGeomSetCategoryBits (_id, bits); }
	//	void setCollideBits (long bits)//unsigned long bits)
	//	{ dGeomSetCollideBits (_id, bits); }
	//	//unsigned 
	//	long getCategoryBits()
	//	{ return dGeomGetCategoryBits (_id); }
	//	//unsigned 
	//	long getCollideBits()
	//	{ return dGeomGetCollideBits (_id); }
	//
	//	void enable()
	//	{ dGeomEnable (_id); }
	//	void disable()
	//	{ dGeomDisable (_id); }
	//	int isEnabled()
	//	{ return dGeomIsEnabled (_id); }
	//
	//	void collide2 (dGeom g, Object data, dNearCallback callback)
	//	{ dSpaceCollide2 (_id,g,data,callback); }
}
