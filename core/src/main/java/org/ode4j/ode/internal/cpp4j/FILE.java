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

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PushbackInputStream;

public class FILE {

	private OutputStream _out;
	private PushbackInputStream _in;
	
	public FILE(String pathname, String opt) {
		try {
			if (opt.equals("r") || opt.equals("rb")) {
				_in = new PushbackInputStream(new FileInputStream(pathname));
			} else if (opt.equals("r") || opt.equals("rb")) {
				_out = new FileOutputStream(pathname);
			} else {
				throw new IllegalArgumentException("Unknown option: " + opt);
			}
		} catch (FileNotFoundException e) {
			throw new RuntimeException(e);
		}
	}
	
	public FILE(OutputStream out) {
		_out = out;
	}

	public OutputStream out() {
		checkOut();
		return _out;
	}
	
	public void write(int b) {
		checkOut();
		try {
			_out.write(b);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
	
	private void checkIn() {
		if (_in == null) {
			throw new IllegalStateException("This stream is not open for " +
				"reading.");
		}
	}

	private void checkOut() {
		if (_out == null) {
			throw new IllegalStateException("This stream is not open for " +
				"writing.");
		}
	}

	public void close() {
		if (_in != null) {
			try {
				_in.close();
			} catch (IOException e) {
				throw new RuntimeException(e);
			}
			_in = null;
		}
		if (_out != null) {
			try {
				_out.close();
			} catch (IOException e) {
				throw new RuntimeException(e);
			}
			_out = null;
		}
	}

	public int fgetc() {
		checkIn();
		try {
			return _in.read();
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
	
	public char fgetcC() {
		//TODO can we simply cast here?
		return (char) fgetc();
	}

	public int ungetc(int c) {
		checkIn();
		try {
			_in.unread(c);
			return c;
		} catch (IOException e) {
			//return EOF;
			throw new RuntimeException(e);
		}
	}

	public int fread(byte[] ptr, int size, int nitems) {
		checkIn();
		try {
			return _in.read(ptr, 0, size * nitems);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
}
