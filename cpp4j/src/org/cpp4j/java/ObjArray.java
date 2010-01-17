package org.cpp4j.java;

import java.util.Arrays;

/**
 * Class to simulate pointer operations on object arrays.
 *
 * @author Tilmann Zaeschke
 */
public class ObjArray<T> {
    
    private final T[] _data;
    private int _ofs;
    
    /**
     * Create a new Array referencing the given array.
     * @param array
     */
    public ObjArray(T[] array) {
        _data = array;
        _ofs = 0;
    }
    
    /**
     * Create a new Array referencing the given array.
     * @param array
     * @param ofs 
     */
    public ObjArray(T[] array, int ofs) {
        _data = array;
        _ofs = ofs;
    }
    
    /**
     * Create a new Array referencing the same Array referenced by
     * the argument.
     * @param array
     */
    public ObjArray(ObjArray<T> array) {
        _data = array._data;
        _ofs = array._ofs;
    }
    
    /**
     * Create a new Array referencing the same Array referenced by
     * the argument, starting at the given offset.
     * @param array
     * @param ofs 
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
     * @param ofs 
     * @return value at position ofs;
     */
    public T at(int ofs) {
        return _data[_ofs + ofs];
    }
    
    /**
     * Set value at position 0;
     * @param d 
     */
    public void setAt0(T d) {
        _data[_ofs] = d;
    }
    
    /**
     * Set value at position ofs;
     * @param ofs 
     * @param d 
     */
    public void setAt(int ofs, T d) {
        _data[_ofs + ofs] = d;
    }
    
    /**
     * @param data
     */
    public void setData(T[] data) {
        if (data.length + _ofs >= _data.length) {
            throw new IndexOutOfBoundsException(data.length + " + " + 
                    _ofs + " = " + (data.length + _ofs) + " >= " + _data.length);
        }
        System.arraycopy(data, 0, _data, _ofs, data.length);
    }
    
    /**
     * @param array
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
     * @param ofs
     * @param len
     * @return cloned []
     */
    public T[] cloneData(int ofs, int len) {
        T[] ret = _data.clone();
//        System.arraycopy(_data, _ofs + ofs, ret, 0, len);
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

	public int size() {
		return _data.length - _ofs;
	}

	public int dataSize() {
		return _data.length;
	}

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
