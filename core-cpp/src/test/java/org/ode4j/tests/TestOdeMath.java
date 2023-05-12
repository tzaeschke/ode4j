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
package org.ode4j.tests;

import org.junit.Test;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DVector3;
import static org.ode4j.tests.UnitTestPlusPlus.CheckMacros.*;
import static org.ode4j.ode.OdeMath.*;

public class TestOdeMath {

	@Test public void test_dNormalization3() {
		//		TEST();
		//	}
		//	
		//TEST(test_dNormalization3)
		//{
		final DVector3 x = new DVector3(1,0,0);
		final DVector3 y = new DVector3(0,1,0);
		final DVector3 z = new DVector3(0,0,1);
		DVector3 v3 = new DVector3();
		//double[] v = v3.v;

		// Check when value in first component (i.e. [0])
		v3.set( 1.0, 0.0, 0.0 );
		v3.normalize();
		CHECK_ARRAY_CLOSE(x, v3, 3, 1e-6);
		CHECK_EQUAL(v3.length(), 1.0);

		v3.set( 0.1, 0.0, 0.0 );
		v3.normalize();
		CHECK_ARRAY_CLOSE(x, v3, 3, 1e-6);
		CHECK_EQUAL(v3.length(), 1.0);

		v3.set( 1e-20, 0.0, 0.0 );
		v3.normalize();
		CHECK_ARRAY_CLOSE(x, v3, 3, 1e-6);
		CHECK_EQUAL(v3.length(), 1.0);


		// Check when value in first component (i.e. [0])
		v3.set( 0.0, 1.0, 0.0 );
		v3.normalize();
		CHECK_ARRAY_CLOSE(y, v3, 3, 1e-6);
		CHECK_EQUAL(v3.length(), 1.0);

		v3.set( 0.0, 0.1, 0.0);
		v3.normalize();
		CHECK_ARRAY_CLOSE(y, v3, 3, 1e-6);
		CHECK_EQUAL(v3.length(), 1.0);

		v3.set( 0.0, 1e-20, 0.0 );
		v3.normalize();
		CHECK_ARRAY_CLOSE(y, v3, 3, 1e-6);
		CHECK_EQUAL(v3.length(), 1.0);


		// Check when value in first component (i.e. [0])
		v3.set( 0.0, 0.0, 1.0 );
		v3.normalize();
		CHECK_ARRAY_CLOSE(z, v3, 3, 1e-6);
		CHECK_EQUAL(v3.length(), 1.0);

		v3.set( 0.0, 0.0, 0.1 );
		v3.normalize();
		CHECK_ARRAY_CLOSE(z, v3, 3, 1e-6);
		CHECK_EQUAL(v3.length(), 1.0);

		v3.set( 0.0, 0.0, 1e-20);
		v3.normalize();
		CHECK_ARRAY_CLOSE(z, v3, 3, 1e-6);
		CHECK_EQUAL(v3.length(), 1.0);


		// Check negative
		// Check when value in first component (i.e. [0])
		v3.set( -1.0, 0.0, 0.0 );
		v3.normalize();
		CHECK_EQUAL(v3.length(), 1.0);

		v3.set( -0.1, 0.0, 0.0 );
		v3.normalize();
		CHECK_EQUAL(v3.length(), 1.0);

		v3.set( -1e-20, 0.0, 0.0 );
		v3.normalize();
		CHECK_EQUAL(v3.length(), 1.0);


		// Check when value in first component (i.e. [0])
		v3.set( 0.0, -1.0, 0.0 );
		v3.normalize();
		CHECK_EQUAL(v3.length(), 1.0);

		v3.set( 0.0, -0.1, 0.0 );
		v3.normalize();
		CHECK_EQUAL(v3.length(), 1.0);

		v3.set( 0.0, -1e-20, 0.0 );
		v3.normalize();
		CHECK_EQUAL(v3.length(), 1.0);


		// Check when value in first component (i.e. [0])
		v3.set( 0.0, 0.0, -1.0 );
		v3.normalize();
		CHECK_EQUAL(v3.length(), 1.0);

		v3.set( 0.0, 0.0, -0.1 );
		v3.normalize();
		CHECK_EQUAL(v3.length(), 1.0);

		v3.set( 0.0, 0.0, -1e-20 );
		v3.normalize();
		CHECK_EQUAL(v3.length(), 1.0);


		v3.set( 9999999999.0, 0.0, 1e-20 );
		v3.normalize();
		CHECK_EQUAL(v3.length(), 1.0);


		v3.set( 9999999999.0, 9999.0, 9.0 );
		v3.normalize();
		CHECK_CLOSE(v3.length(), 1.0, 0.001);
	}


	@Test public void test_dOrthogonalizeR() {
		//	TEST(test_dOrthogonalizeR)
		{
			{
				DMatrix3 r1 = new DMatrix3( 1, 0, 0, //0,
						0, 1, 0, //0,
						0, 0, 1 //0
				);
				DMatrix3 r2 = new DMatrix3();
				r2.set(r1);//memcpy(r2, r1, sizeof(DMatrix3));
				dOrthogonalizeR(r2);
				CHECK_ARRAY_EQUAL(r1, r2, 12);
			}
			{
				DMatrix3 r1 = new DMatrix3( 0, 1, 0, //0,
						0, 0, 1, //0,
						1, 0, 0 //0
				);
				DMatrix3 r2 = new DMatrix3();
				r2.set(r1);//memcpy(r2, r1, sizeof(DMatrix3));
				dOrthogonalizeR(r2);
				CHECK_ARRAY_EQUAL(r1, r2, 12);
			}
			{
				DMatrix3 r1 = new DMatrix3( 0, 0, 1,
						1, 0, 0,
						0, 1, 0
				);
				DMatrix3 r2 = new DMatrix3();
				r2.set(r1);//memcpy(r2, r1, sizeof(DMatrix3));
				dOrthogonalizeR(r2);
				CHECK_ARRAY_EQUAL(r1, r2, 12);
			}
			{
				DMatrix3 r1 = new DMatrix3( -1, 0, 0,
						0, 1,  0,
						0, 0, -1
				);
				DMatrix3 r2 = new DMatrix3();
				r2.set(r1);//memcpy(r2, r1, sizeof(DMatrix3));
				dOrthogonalizeR(r2);
				CHECK_ARRAY_EQUAL(r1, r2, 12);
			}
			{
				DMatrix3 r1 = new DMatrix3( 0, -1, 0,
						0,  0, 1,
						-1,  0, 0
				);
				DMatrix3 r2 = new DMatrix3();
				r2.set(r1);//memcpy(r2, r1, sizeof(DMatrix3));
				dOrthogonalizeR(r2);
				CHECK_ARRAY_EQUAL(r1, r2, 12);
			}
			{
				DMatrix3 r1 = new DMatrix3( 0, 0, -1,
						0, -1, 0,
						-1, 0, 0
				);
				DMatrix3 r2 = new DMatrix3();
				r2.set(r1);//memcpy(r2, r1, sizeof(DMatrix3));
				dOrthogonalizeR(r2);
				CHECK_ARRAY_EQUAL(r1, r2, 12);
			}

		}
	}
}
