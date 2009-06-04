package org.cpp4j;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PushbackInputStream;

public class FILE {

	private static final int EOF = -1;
	
	private static final long serialVersionUID = 1L;

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
