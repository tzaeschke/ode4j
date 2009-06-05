package org.ode4j.tests;

// openode_UnitTest++.cpp : Defines the entry point for the console application.
//

//#include <UnitTest++.h>

class Main {

 public static void main(String[] args) {
	 org.junit.runner.JUnitCore.runClasses(
			 TestOdeMath.class
			 ); 

//	  return UnitTest::RunAllTests();	
}


}