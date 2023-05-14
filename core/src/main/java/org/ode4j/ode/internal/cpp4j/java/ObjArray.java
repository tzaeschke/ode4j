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
package org.ode4j.ode.internal.cpp4j.java;

import java.util.Arrays;

/**
 * Class to simulate pointer operations on object arrays.
 *
 * @author Tilmann Zaeschke
 * @param <T> type
 */
public class ObjArray<T> {
    
    private final T[] _data;
    private int _ofs;
    
    /**
     * Create a new Array referencing the given array.
     * @param array array
     */
    public ObjArray(T[] array) {
        _data = array;
        _ofs = 0;
    }
    
    /**
     * Create a new Array referencing the given array.
     * @param array array
     * @param ofs offset
     */
    public ObjArray(T[] array, int ofs) {
        _data = array;
        _ofs = ofs;
    }
    
    /**
     * Create a new Array referencing the same Array referenced by
     * the argument.
     * @param array array
     */
    public ObjArray(ObjArray<T> array) {
        _data = array._data;
        _ofs = array._ofs;
    }
    
    /**
     * Create a new Array referencing the same Array referenced by
     * the argument, starting at the given offset.
     * @param array array
     * @param ofs offset
     */
    public ObjArray(ObjArray<T> array, int ofs) {
        _data = array._data;
        _ofs = array._ofs + ofs;
        if (_ofs >= _data.length) {
            throw new IndexOutOfBoundsException(array._ofs + " + " + ofs + 
                    " = " + _ofs + " >= " + _data.length);
        }
    }
    
    /**
     * @return value at position 0;
     */
    public T at0() {
        return _data[_ofs];
    }
    
    /**
     * @param ofs offset
     * @return value at position ofs;
     */
    public T at(int ofs) {
        return _data[_ofs + ofs];
    }
    
    /**
     * Set value at position 0;
     * @param d d
     */
    public void setAt0(T d) {
        _data[_ofs] = d;
    }
    
    /**
     * Set value at position ofs;
     * @param ofs offset
     * @param d d
     */
    public void setAt(int ofs, T d) {
        _data[_ofs + ofs] = d;
    }
    
    /**
     * @param data data
     */
    public void setData(T[] data) {
        if (data.length + _ofs >= _data.length) {
            throw new IndexOutOfBoundsException(data.length + " + " + 
                    _ofs + " = " + (data.length + _ofs) + " >= " + _data.length);
        }
        System.arraycopy(data, 0, _data, _ofs, data.length);
    }
    
    /**
     * @param array array
     */
    public void setData(ObjArray<T> array) {
        T[] data = array._data;
        int ofs = array._ofs;
        if (data.length + _ofs + ofs >= _data.length) {
            throw new IndexOutOfBoundsException(data.length + " + " + ofs +
                    " + " + _ofs + " = " + (data.length + _ofs + ofs) + " >= " +
                    _data.length);
        }
        System.arraycopy(data, 0, _data, _ofs, data.length);
    }
    
    /**
     * @return cloned double[].
     */
    public T[] cloneData() {
        T[] ret = Arrays.copyOf(_data, _data.length - _ofs);
//        System.arraycopy(_data, _ofs, ret, 0, _data.length - _ofs);
        return ret;
    }

    /**
     * Decrement offset by 1.
     */
    public void dec() {
        _ofs--;
    }
    
    /**
     * Decrement offset by n.
     * @param n n
     */
    public void dec(int n) {
        _ofs -= n;
    }
    
    /**
     * Increment offset by 1.
     */
    public void inc() {
        _ofs++;
    }
    
    /**
     * Increment offset by n.
     * @param n n
     */
    public void inc(int n) {
        _ofs += n;
    }

	/**
	 * @return current size
	 */
	public int size() {
		return _data.length - _ofs;
	}

	/**
	 * @return Original size
	 */
	public int dataSize() {
		return _data.length;
	}

	/**
	 * @param destPos dest pos
	 * @param srcPos source pos
	 * @param len length
	 */
	public void memcpy(int destPos, int srcPos, int len) {
		if (destPos > srcPos) {
			for (int i = _ofs+len-1; i >= _ofs; i--) {
				_data[destPos + i] = _data[srcPos + i];
			}
		} else if (destPos < srcPos) {
			for (int i = _ofs; i < _ofs+len; i++) {
				_data[destPos + i] = _data[srcPos + i];
			}
		}
	}
}
