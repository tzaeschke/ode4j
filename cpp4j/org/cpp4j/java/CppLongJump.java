package org.cpp4j.java;

/**
 * Catching this exception can be used to implement a java version of 
 * <tt>longjump(...)</tt>.
 * 
 * @author Tilmann Zaeschke
 */
public class CppLongJump extends RuntimeException {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	public CppLongJump(String string) {
		super(string);
	}

}
