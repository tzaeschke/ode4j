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

/**
 * Class to simulate pointer operations on integer arrays.
 *
 * @author Tilmann Zaeschke
 */
public class IntArray {
    
    private final int[] _data;
    private int _ofs;
    
    /**
     * @param size
     */
    public IntArray(int size) {
        _data = new int[size];
        _ofs = 0;
    }
    
    /**
     * Create a new DoubleArray referencing the given int array.
     * @param array
     */
    public IntArray(int[] array) {
        _data = array;
        _ofs = 0;
    }
    
    /**
     * Create a new DoubleArray referencing the given int array.
     * @param array
     * @param ofs 
     */
    public IntArray(int[] array, int ofs) {
        _data = array;
        _ofs = ofs;
    }
    
    /**
     * Create a new DoubleArray referencing the same DoubleArray referenced by
     * the argument.
     * @param array
     */
    public IntArray(IntArray array) {
        _data = array._data;
        _ofs = array._ofs;
    }
    
    /**
     * Create a new DoubleArray referencing the same DoubleArray referenced by
     * the argument, starting at the given offset.
     * @param array
     * @param ofs 
     */
    public IntArray(IntArray array, int ofs) {
        _data = array._data;
        _ofs = array._ofs + ofs;
        if (_ofs >= _data.length) {
            throw new IndexOutOfBoundsException(array._ofs + " + " + ofs + 
                    " = " + _ofs + " >= " + _data.length);
        }
    }
    
    /**
     * @return int value at position 0;
     */
    public int at0() {
        return _data[_ofs];
    }
    
    /**
     * @param ofs 
     * @return int value at position ofs;
     */
    public int at(int ofs) {
        return _data[_ofs + ofs];
    }
    
    /**
     * Set int value at position 0;
     * @param d 
     */
    public void setAt0(int d) {
        _data[_ofs] = d;
    }
    
    /**
     * Set int value at position ofs;
     * @param ofs 
     * @param d 
     */
    public void setAt(int ofs, int d) {
        _data[_ofs + ofs] = d;
    }
    
    /**
     * @param data
     */
    public void setData(int[] data) {
        if (data.length + _ofs >= _data.length) {
            throw new IndexOutOfBoundsException(data.length + " + " + 
                    _ofs + " = " + (data.length + _ofs) + " >= " + _data.length);
        }
        System.arraycopy(data, 0, _data, _ofs, data.length);
    }
    
    /**
     * @param array
     */
    public void setData(IntArray array) {
        int[] data = array._data;
        int ofs = array._ofs;
        if (data.length + _ofs + ofs >= _data.length) {
            throw new IndexOutOfBoundsException(data.length + " + " + ofs +
                    " + " + _ofs + " = " + (data.length + _ofs + ofs) + " >= " +
                    _data.length);
        }
        System.arraycopy(data, 0, _data, _ofs, data.length);
    }
    
    /**
     * @return cloned int[].
     */
    public int[] cloneData() {
        int[] ret = new int[_data.length - _ofs];
        System.arraycopy(_data, _ofs, ret, 0, _data.length - _ofs);
        return ret;
    }
    
    /**
     * @param ofs
     * @param len
     * @return cloned int[]
     */
    public int[] cloneData(int ofs, int len) {
        int[] ret = new int[len];
        System.arraycopy(_data, _ofs + ofs, ret, 0, len);
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
     * @param n 
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
     * @param n 
     */
    public void inc(int n) {
        _ofs += n;
    }
    
    /**
     * 
     * @return The current offset over the referenced array.
     */
    public int getOfs() {
    	return _ofs;
    }

	/**
	 * @return Current size
	 */
	public int size() {
		return _data.length - _ofs;
	}
}
