package org.cpp4j;

public class Cstring extends Ctime {

	public static void memcpy(double[] to, double[] from, int count) {
		System.arraycopy(from, 0, to, 0, count);
	}
	
	public static void memcpy(double[] to, int i, double[] from, int j, int count) {
		System.arraycopy(from, j, to, i, count);
	}
	
	public static void memmove(double[] to, double[] from, int count) {
		System.arraycopy(from, 0, to, 0, count);
	}

	public static void memmove(double[] to, int i, double[] from, int j, int count) {
		System.arraycopy(from, j, to, i, count);
	}

	public static void memmove(int[] to, int i, int[] from, int j, int count) {
		System.arraycopy(from, j, to, i, count);
	}

	/**
	 * 
	 * @param data
	 * @param c value to set
	 * @param l number of values to set
	 * @deprecated Do not user for c=0
	 */
	public static void memset(int[] data, int c, int l) {
		for (int i = 0; i < l; i++) {
			data[i] = c;
		}
		
	}
	
	public static int strcmp(char[] s, String string) {
		return new String(s).compareTo(string);
	}

	public static int strcmp(String s, String string) {
		return new String(s).compareTo(string);
	}

	/**
	 * returns the string length. The null terminator is not counted.
	 */
	public static int strlen(char[] s) {
		for (int i = 0; i < s.length; i++) {
			if (s[i] == '\0') {
				return i; 
			}
		}
		return s.length;
//		throw new IllegalStateException("String has no delimitor (l=" + 
//				s.length + "): \"" + new String(s) + "\"");
	}

	public static int strlen(String s) {
		return strlen(s.toCharArray());
	}

}
