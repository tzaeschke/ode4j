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
package org.ode4j.tests.UnitTestPlusPlus;

import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.internal.cpp4j.java.RefDouble;

import static org.junit.Assert.*;

//#ifndef UNITTEST_CHECKMACROS_H 
//#define UNITTEST_CHECKMACROS_H
//
//#include "Checks.h"
//#include "AssertException.h"
//#include "MemoryOutStream.h"
//#include "TestDetails.h"
//
//#ifdef CHECK
//    #error UnitTest++ redefines CHECK
//#endif
//

/**
 * Test macros.
 */
public class CheckMacros {

	public interface Expr {
		public void run();
	}

	//#define CHECK(value) \
	//    do \
	//    { \
	//        try { \
	//            if (!UnitTest::Check(value)) \
	//                testResults_.OnTestFailure( UnitTest::TestDetails(m_details, __LINE__), #value); \
	//        } \
	//        catch (...) { \
	//            testResults_.OnTestFailure(UnitTest::TestDetails(m_details, __LINE__), \
	//                    "Unhandled exception in CHECK(" #value ")"); \
	//        } \
	//    } while (0)
	public static void CHECK(Object value) {
		throw new UnsupportedOperationException();
//		try { 
//			if (!UnitTest.Check(value)) 
//				testResults_.OnTestFailure( UnitTest.TestDetails(m_details, __LINE__), #value); 
//		} 
//		catch (Throwable t) { 
//			testResults_.OnTestFailure(UnitTest.TestDetails(m_details, __LINE__), 
//					"Unhandled exception in CHECK(" #value ")"); 
//		} 
	}
	public static void CHECK(boolean value) {
		assertTrue(value);
	}

	//#define CHECK_EQUAL(expected, actual) \
	//    do \
	//    { \
	//        try { \
	//            UnitTest::CheckEqual(testResults_, expected, actual, UnitTest::TestDetails(m_details, __LINE__)); \
	//        } \
	//        catch (...) { \
	//            testResults_.OnTestFailure(UnitTest::TestDetails(m_details, __LINE__), \
	//                    "Unhandled exception in CHECK_EQUAL(" #expected ", " #actual ")"); \
	//        } \
	//    } while (0)

	public static void CHECK_EQUAL(Object expected, Object actual) {
		assertEquals(expected, actual);
		//    try { 
		//        UnitTest.CheckEqual(testResults_, expected, actual, UnitTest.TestDetails(m_details, __LINE__)); 
		//    } 
		//    catch (Throwable t) { 
		//        testResults_.OnTestFailure(UnitTest.TestDetails(m_details, __LINE__), 
		//                "Unhandled exception in CHECK_EQUAL(" #expected ", " #actual ")"); 
		//    } 
	}
	public static void CHECK_EQUAL(int expected, int actual) {
		assertEquals(expected, actual);
	}

	//#define CHECK_CLOSE(expected, actual, tolerance) \
	//    do \
	//    { \
	//        try { \
	//            UnitTest::CheckClose(testResults_, expected, actual, tolerance, UnitTest::TestDetails(m_details, __LINE__)); \
	//        } \
	//        catch (...) { \
	//            testResults_.OnTestFailure(UnitTest::TestDetails(m_details, __LINE__), \
	//                    "Unhandled exception in CHECK_CLOSE(" #expected ", " #actual ")"); \
	//        } \
	//    } while (0)
	//
	public static void CHECK_CLOSE(double expected, double actual, double tolerance) {
		assertTrue("Expected=" + expected + " / actual=" + actual + " / tolerance=" + tolerance, 
				tolerance >= Math.abs(expected - actual));
		//    try { 
		//        UnitTest::CheckClose(testResults_, expected, actual, tolerance, UnitTest.TestDetails(m_details, __LINE__)); 
		//    } 
		//    catch (Throwable t) { 
		//        testResults_.OnTestFailure(UnitTest.TestDetails(m_details, __LINE__), 
		//                "Unhandled exception in CHECK_CLOSE(" #expected ", " #actual ")"); 
		//    } 
	}
	public static void CHECK_CLOSE(double expected, RefDouble actual, double tolerance) {
		assertTrue("Expected=" + expected + " / actual=" + actual + " / tolerance=" + tolerance, 
				tolerance >= Math.abs(expected - actual.d));
	}

	//#define CHECK_ARRAY_EQUAL(expected, actual, count) \
	//    do \
	//    { \
	//        try { \
	//            UnitTest::CheckArrayEqual(testResults_, expected, actual, count, UnitTest::TestDetails(m_details, __LINE__)); \
	//        } \
	//        catch (...) { \
	//            testResults_.OnTestFailure(UnitTest::TestDetails(m_details, __LINE__), \
	//                    "Unhandled exception in CHECK_ARRAY_EQUAL(" #expected ", " #actual ")"); \
	//        } \
	//    } while (0)
    public static void CHECK_ARRAY_EQUAL(DMatrix3C expected, DMatrix3C actual, int count) {
        assertEquals(12, count);
        assertEquals(expected, actual);
    }
	public static void CHECK_ARRAY_EQUAL(Object expected, Object actual, int count) {
		assertEquals(expected, actual);
//		try { 
//			UnitTest::CheckArrayEqual(testResults_, expected, actual, count, UnitTest::TestDetails(m_details, __LINE__)); 
//		} 
//		catch (Throwable t) { 
//			testResults_.OnTestFailure(UnitTest.TestDetails(m_details, __LINE__), 
//					"Unhandled exception in CHECK_ARRAY_EQUAL(" #expected ", " #actual ")"); 
//		} 
	}

	//#define CHECK_ARRAY_CLOSE(expected, actual, count, tolerance) \
	//    do \
	//    { \
	//        try { \
	//            UnitTest::CheckArrayClose(testResults_, expected, actual, count, tolerance, UnitTest::TestDetails(m_details, __LINE__)); \
	//        } \
	//        catch (...) { \
	//            testResults_.OnTestFailure(UnitTest::TestDetails(m_details, __LINE__), \
	//                    "Unhandled exception in CHECK_ARRAY_CLOSE(" #expected ", " #actual ")"); \
	//        } \
	//    } while (0)
	public static void CHECK_ARRAY_CLOSE(double[] expected, double[] actual, int count, double tolerance) {
		for (int i = 0; i < count; i++) {
			assertTrue("Expected: " + expected[i] + "  actual: " + actual[i] +
					"   tolerance: " + tolerance, 
					Math.abs(actual[i]-expected[i]) <= tolerance);
		}
	}
	public static void CHECK_ARRAY_CLOSE(DVector3C expected, DVector3 actual, int count, double tolerance) {
		for (int i = 0; i < count; i++) {
			assertTrue("Expected: " + expected.get(i) + "  actual: " + actual.get(i) +
					"   tolerance: " + tolerance, 
					Math.abs(actual.get(i)-expected.get(i)) <= tolerance);
		}
	}
    public static void CHECK_ARRAY_CLOSE(DMatrix3C expected, double[] actual, int count, double tolerance) {
        assertEquals(12, count);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                int pos = i*4 + j;
                assertTrue("Expected: " + expected.get(i, j) + "  actual: " + actual[pos] +
                        "   tolerance: " + tolerance, 
                        Math.abs(actual[pos]-expected.get(i, j)) <= tolerance);
            }
        }
    }

	//#define CHECK_ARRAY2D_CLOSE(expected, actual, rows, columns, tolerance) \
	//    do \
	//    { \
	//        try { \
	//            UnitTest::CheckArray2DClose(testResults_, expected, actual, rows, columns, tolerance, UnitTest::TestDetails(m_details, __LINE__)); \
	//        } \
	//        catch (...) { \
	//            testResults_.OnTestFailure(UnitTest::TestDetails(m_details, __LINE__), \
	//                    "Unhandled exception in CHECK_ARRAY_CLOSE(" #expected ", " #actual ")"); \
	//        } \
	//    } while (0)
	public static void CHECK_ARRAY2D_CLOSE(Object expected, Object actual, int rows, int columns, 
			double tolerance) {
		throw new UnsupportedOperationException();
//		try { 
//			UnitTest::CheckArray2DClose(testResults_, expected, actual, rows, columns, tolerance, UnitTest::TestDetails(m_details, __LINE__)); 
//		} 
//		catch (Throwable t) { 
//			testResults_.OnTestFailure(UnitTest.TestDetails(m_details, __LINE__), 
//					"Unhandled exception in CHECK_ARRAY_CLOSE(" #expected ", " #actual ")"); 
//		} 
	}


	//#define CHECK_THROW(expression, ExpectedExceptionType) \
	//    do \
	//    { \
	//        bool caught_ = false; \
	//        try { expression; } \
	//        catch (ExpectedExceptionType const&) { caught_ = true; } \
	//        catch (...) {} \
	//        if (!caught_) \
	//            testResults_.OnTestFailure(UnitTest::TestDetails(m_details, __LINE__), "Expected exception: \"" #ExpectedExceptionType "\" not thrown"); \
	//    } while(0)
	public static void CHECK_THROW(Expr expression, Class<Throwable> ExpectedExceptionType) {
		throw new UnsupportedOperationException();
//		boolean caught_ = false; 
//		try { expression.run(); } 
//		catch (Throwable t) {
//			if (ExpectedExceptionType.isAssignableFrom(t.getClass())) { caught_ = true; }
//		} 
//		if (!caught_) 
//			testResults_.OnTestFailure(UnitTest.TestDetails(m_details, __LINE__), "Expected exception: \"" #ExpectedExceptionType "\" not thrown"); 
	}

	//#define CHECK_ASSERT(expression) \
	//    CHECK_THROW(expression, UnitTest::AssertException);
	public static void CHECK_ASSERT(Expr expression) {
		throw new UnsupportedOperationException();
//		CHECK_THROW(expression, UnitTest.AssertException);
	}

	//#endif
}
