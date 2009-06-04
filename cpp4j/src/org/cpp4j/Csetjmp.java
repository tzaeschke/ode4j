package org.cpp4j;

import org.cpp4j.java.CppLongJump;

public class Csetjmp extends Cstdarg {

	public static class jmp_buf {
		int _ret = 0;
	};
	
	public static void longjmp(jmp_buf jump_buffer, int i) {
		throw new CppLongJump("" + i);
//		NIW();
//		jump_buffer._ret = i;
	}

	public static int setjmp(jmp_buf jump_buffer) {
		NIW();
		return 0;//jump_buffer._ret;
	}
	
	private static void NIW() {
		String f = new Exception().getStackTrace()[1].toString();
		System.out.println("WARNING: Not implemented: " + f);
	}
}
