/***
 * libccd
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 * Java-port: Copyright (c) 2009-2014 Tilmann Zaeschke<ode4j@gmx.de>  
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
package org.ode4j.ode.internal.libccd;

import static org.ode4j.ode.internal.libccd.CCDVec3.*;

import org.ode4j.ode.internal.libccd.CCD.ccd_t;
import org.ode4j.ode.internal.libccd.CCDVec3.ccd_vec3_t;

/**
 * 
 *
 * LibCCD class for support points.
 */
public class CCDSupport {

	/**
	 * Support point type.
	 */
	public static final class ccd_support_t {
		final ccd_vec3_t v = new ccd_vec3_t();  //!< Support point in minkowski sum
		final ccd_vec3_t v1 = new ccd_vec3_t(); //!< Support point in obj1
		final ccd_vec3_t v2 = new ccd_vec3_t(); //!< Support point in obj2

		ccd_support_t() {}
		/**
		 * @return Support point minkowski sum.
		 */
		public ccd_vec3_t v() {
			return v;
		}
	};

	/**
	 * Computes support point of obj1 and obj2 in direction dir.
	 * Support point is returned via supp.
	 */
	static void __ccdSupport(final Object obj1, final Object obj2,
							 final ccd_vec3_t _dir, final ccd_t ccd,
							 final ccd_support_t supp)
	{
		ccd_vec3_t dir = new ccd_vec3_t();

		ccdVec3Copy(dir, _dir);

		ccd.support1.run(obj1, dir, supp.v1);

		ccdVec3Scale(dir, -CCDVec3.CCD_ONE);
		ccd.support2.run(obj2, dir, supp.v2);

		ccdVec3Sub2(supp.v, supp.v1, supp.v2);
	}


	/**** INLINES ****/
	static void ccdSupportCopy(ccd_support_t d, final ccd_support_t s)
	{
		//*d = *s;
		CCDVec3.ccdVec3Copy(d.v, s.v);
		CCDVec3.ccdVec3Copy(d.v1, s.v1);
		CCDVec3.ccdVec3Copy(d.v2, s.v2);
	}

	protected CCDSupport() {}
}
