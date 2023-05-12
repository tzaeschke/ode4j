/**
 * libccd
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *
 *  This file is part of libccd.
 *
 *  Distributed under the OSI-approved BSD License (the "License");
 *  see accompanying file BDS-LICENSE for details or see
 *  <http://www.opensource.org/licenses/bsd-license.php>.
 *
 *  This software is distributed WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the License for more information.
 */
package org.ode4j.libccd;

import static org.ode4j.ode.internal.libccd.CCD.*;
import static org.ode4j.ode.internal.libccd.CCDQuat.*;
import static org.ode4j.ode.internal.libccd.CCDVec3.*;

import org.ode4j.ode.internal.libccd.CCDVec3.ccd_vec3_t;

public class CCDTestSupport {

	/***
	 * Some support() functions for some convex shapes.
	 */

	static final int CCD_OBJ_BOX = 1;
	private static final int CCD_OBJ_SPHERE = 2;
	static final int CCD_OBJ_CYL = 3;

//	#define __CCD_OBJ__ \
//	    int type; \
//	    ccd_vec3_t pos; \
//	    ccd_quat_t quat;

	static class ccd_obj_t {
		//__CCD_OBJ__
	    int type;
	    final ccd_vec3_t pos = new ccd_vec3_t();
	    final ccd_quat_t quat = new ccd_quat_t();
	}
	//typedef struct _ccd_obj_t ccd_obj_t;

	static final class ccd_box_t extends ccd_obj_t {
	    //__CCD_OBJ__
	    double x, y, z; //!< Lengths of box's edges
	}
	//typedef struct _ccd_box_t ccd_box_t;

	static final class ccd_sphere_t extends ccd_obj_t {
	    //__CCD_OBJ__
	    double radius;
	}
	//typedef struct _ccd_sphere_t ccd_sphere_t;

	static final class ccd_cyl_t extends ccd_obj_t {
	    //__CCD_OBJ__
	    double radius;
	    double height;
	}
	//typedef struct _ccd_cyl_t ccd_cyl_t;


	static final ccd_box_t CCD_BOX() {
	    ccd_box_t name = new ccd_box_t();
	    name.type = CCD_OBJ_BOX;
	    name.pos.set( 0., 0., 0. );
	    name.quat.set( 0., 0., 0., 1. );
	    name.x = 0.;
	    name.y = 0.;
	    name.z = 0.;
	    return name;
	}

	static final ccd_sphere_t CCD_SPHERE() {
	    ccd_sphere_t name = new ccd_sphere_t();
	    name.type = CCD_OBJ_SPHERE;
	    name.pos.set( 0., 0., 0. );
	    name.quat.set( 0., 0., 0., 1.) ;
	    name.radius = 0.;
	    return name;
	}

	static final ccd_cyl_t CCD_CYL() {
		ccd_cyl_t name = new ccd_cyl_t();
		name.type = CCD_OBJ_CYL;
		name.pos.set( 0., 0., 0. );
		name.quat.set( 0., 0., 0., 1. );
		name.radius = 0.;
		name.height = 0.;
		return name;
	}

//	/**
//	 * Returns supporting vertex via v.
//	 * Supporting vertex is fathest vertex from object in direction dir.
//	 */
//	void ccdSupport(const void *obj, const ccd_vec3_t *dir,
//	                ccd_vec3_t *v);
//
//	/**
//	 * Returns center of object.
//	 */
//	void ccdObjCenter(const void *obj, ccd_vec3_t *center);

	/***
	 * libccd
	 * ---------------------------------
	 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
	 *
	 *
	 *  This file is part of libccd.
	 *
	 *  Distributed under the OSI-approved BSD License (the "License");
	 *  see accompanying file BDS-LICENSE for details or see
	 *  <http://www.opensource.org/licenses/bsd-license.php>.
	 *
	 *  This software is distributed WITHOUT ANY WARRANTY; without even the
	 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
	 *  See the License for more information.
	 */

	
	static final ccd_support_fn ccdSupport  = new ccd_support_fn() {
		
		@Override
		public void run(Object obj, ccd_vec3_t dir, ccd_vec3_t vec) {
			CCDTestSupport.ccdSupport(obj, dir, vec);
		}
	};

	static void ccdSupport(final Object _obj, final ccd_vec3_t _dir,
	                ccd_vec3_t v)
	{
	    // Support function is made according to Gino van den Bergen's paper
	    //  A Fast and Robust CCD Implementation for Collision Detection of
	    //  Convex Objects

	    ccd_obj_t obj = (ccd_obj_t )_obj;
	    ccd_vec3_t dir = new ccd_vec3_t();
	    ccd_quat_t qinv = new ccd_quat_t();

	    ccdVec3Copy(dir, _dir);
	    ccdQuatInvert2(qinv, obj.quat);

	    ccdQuatRotVec(dir, qinv);

	    if (obj.type == CCD_OBJ_BOX){
	        ccd_box_t box = (ccd_box_t )obj;
	        ccdVec3Set(v, ccdSign(ccdVec3X(dir)) * box.x * (0.5),
	                      ccdSign(ccdVec3Y(dir)) * box.y * (0.5),
	                      ccdSign(ccdVec3Z(dir)) * box.z * (0.5));
	    }else if (obj.type == CCD_OBJ_SPHERE){
	        ccd_sphere_t sphere = (ccd_sphere_t )obj;
	        double len;

	        len = ccdVec3Len2(dir);
	        if (len - CCD_EPS > CCD_ZERO){
	            ccdVec3Copy(v, dir);
	            ccdVec3Scale(v, sphere.radius / CCD_SQRT(len));
	        }else{
	            ccdVec3Set(v, CCD_ZERO, CCD_ZERO, CCD_ZERO);
	        }
	    }else if (obj.type == CCD_OBJ_CYL){
	        ccd_cyl_t cyl = (ccd_cyl_t )obj;
	        double zdist, rad;

	        //zdist = dir.v[0] * dir.v[0] + dir.v[1] * dir.v[1];
	        zdist = dir.get0() * dir.get0() + dir.get1() * dir.get1();
	        zdist = CCD_SQRT(zdist);
	        if (ccdIsZero(zdist)){
	            ccdVec3Set(v, CCD_ZERO, CCD_ZERO,
	                          ccdSign(ccdVec3Z(dir)) * cyl.height * (0.5));
	        }else{
	            rad = cyl.radius / zdist;

	            ccdVec3Set(v, rad * ccdVec3X(dir),
	                          rad * ccdVec3Y(dir),
	                          ccdSign(ccdVec3Z(dir)) * cyl.height * (0.5));
	        }
	    }

	    // transform support vertex
	    ccdQuatRotVec(v, obj.quat);
	    ccdVec3Add(v, obj.pos);
	}

	public static final ccd_center_fn ccdObjCenter = new ccd_center_fn() {
		@Override
		public void run(Object obj1, ccd_vec3_t center) {
			CCDTestSupport.ccdObjCenter(obj1, center);
		}
	};
	
	static void ccdObjCenter(final Object _obj, ccd_vec3_t center)
	{
	    ccd_obj_t obj = (ccd_obj_t )_obj;

	    ccdVec3Set(center, CCD_ZERO, CCD_ZERO, CCD_ZERO);
	    // rotation is not needed
	    ccdVec3Add(center, obj.pos);
	}

}
