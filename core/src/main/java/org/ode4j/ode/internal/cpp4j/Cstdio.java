/*************************************************************************
 *                                                                       *
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
 *       the file ODE4J-LICENSE-BSD.TXT.                                 *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and ODE4J-LICENSE-BSD.TXT for more details.               *
 *                                                                       *
 *************************************************************************/
package org.ode4j.ode.internal.cpp4j;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.io.StringWriter;

import org.ode4j.ode.internal.cpp4j.java.Ref;

public class Cstdio extends Cstdlib {
	
//	public static final PrintStream stdout = System.out;
//	public static final PrintStream stderr = System.err;
//	public static final InputStream stdin = System.in;
	public static final FILE stdout = new FILE(System.out);
	public static final FILE stderr = new FILE(System.err);
	//public static final FILE stdin = new FILE(System.in);
	
	public static void fprintf(OutputStream out, String fmt, Object ... args) {
		//FileWriter 	fw = new FileWriter(f);
		PrintWriter pw = new PrintWriter(out);
		pw.printf(fmt, args);
		if (out.equals(stderr.out())) pw.flush();
	}
	
	public static void fprintf(FILE out, String fmt, Object ... args) {
		PrintWriter pw = new PrintWriter(out.out());
		pw.printf(fmt, args);
		if (out.equals(stderr)) pw.flush();
	}
	
	/**
	 * Write to STDOUT.
	 * @param format format
	 * @param args  args
	 */
	public static void printf(String format, Object ... args) {
		System.out.printf(format, args);
	}


	/**
	 * @param s s
	 * @param pos pos
	 * @param string string
	 * @param args args
	 */
	public static void sprintf(char[] s, int pos, String string, Object ... args) {
		StringWriter sw = new StringWriter();
		PrintWriter pw = new PrintWriter(sw);
		String f = new String(s);
		f = f.substring(pos);
		pw.printf(f, args);
		String result = sw.getBuffer().toString();
		result.getChars(0, result.length(), s, pos);
	}

	
	public static final int EOF = -1;

	public static void vsprintf(Ref<String> str, String format, Object ... ap) {
		StringWriter s = new StringWriter();
		PrintWriter p = new PrintWriter(s);
		p.printf(format, ap);
		str.set(s.toString());
	}

	public static void vprintf(String format, Object ... ap) {
		System.out.printf(format, ap);
	}

	public static FILE fopen(String name, String opt) {
		return new FILE(name, opt);
	}

	public static FILE fopen(OutputStream out) {
		return new FILE(out);
	}

	public static void fclose(FILE f) {
		f.close();
	}

	public static char fgetcC(FILE f) {
		return f.fgetcC();
	}
	
	public static int fgetc(FILE f) {
		return f.fgetc();
	}
	
	public static int fread(byte[] ptr, int size, int nitems, FILE f) {
		f.fread(ptr, size, nitems);
		return nitems;
	}

	public static int fflush(FILE stream) {
		try {
			stream.out().flush();
		} catch (IOException e) {
			throw new RuntimeException(e);
			//TODO
			//Have switch that returns EOF rather than throwning exception
		}
		return 0;
	}
	
	public static int vfprintf ( FILE stream, String format, Object ... ap ) {
		StringWriter s = new StringWriter();
		PrintWriter p = new PrintWriter(s);
		p.printf(format, ap);
		String str = s.toString();
		//TODO this writes ASCII, not unicode, or does it?
//		byte[] ba = new byte[str.length()];
//		for (int i = 0; i<str.length(); i++) {
//			str.
//		}
		try {
			stream.out().write(str.getBytes());
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
		return str.length();
	}
	
	public static int ungetc(int c, FILE f) {
		return f.ungetc(c);
	}
	
	/**
	 * Drop-in replacement functions for std::cout.
	 * @param objs objects
	 */
	public static void std_cout(Object ...  objs) {
		for (Object o: objs) {
			System.out.print(o.toString());
		}
	}
	
	public static int getchar() {
		//TODO test !!!, Does this really wait?
		try {
			return System.in.read();
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	protected Cstdio() {}
}
