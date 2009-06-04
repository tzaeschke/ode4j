package org.cpp4j;

public class Cstdarg extends Cstdio{

	public static class va_list {
		final Object[] l;
		public va_list(Object[] varArgsOfCallingMethod) {
			l = varArgsOfCallingMethod;
		}
	}
	
	/**
	 * This method does nothing.
	 * @param ap
	 * @param argToStartAfter_isIgnored
	 */
	public static void va_start(va_list ap, Object argToStartAfter_isIgnored ) {
		
	}
	
	/**
	 * This method does nothing.
	 * @param ap
	 */
	public static void va_end(va_list ap) {
		
	}
}
