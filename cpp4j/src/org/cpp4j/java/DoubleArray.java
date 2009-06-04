package org.cpp4j.java;

/**
 * Class to simulate pointer operations on double arrays.
 *
 * @author Tilmann Zaeschke
 */
public class DoubleArray {
    
    private final double[] _data;
    private int _ofs;
    
    /**
     * @param size
     */
    public DoubleArray(int size) {
        _data = new double[size];
        _ofs = 0;
    }
    
    /**
     * Create a new DoubleArray referencing the given double array.
     * @param array
     */
    public DoubleArray(double[] array) {
        _data = array;
        _ofs = 0;
    }
    
    /**
     * Create a new DoubleArray referencing the given double array.
     * @param array
     * @param ofs 
     */
    public DoubleArray(double[] array, int ofs) {
        _data = array;
        _ofs = ofs;
    }
    
    /**
     * Create a new DoubleArray referencing the same DoubleArray referenced by
     * the argument.
     * @param array
     */
    public DoubleArray(DoubleArray array) {
        _data = array._data;
        _ofs = array._ofs;
    }
    
    /**
     * Create a new DoubleArray referencing the same DoubleArray referenced by
     * the argument, starting at the given offset.
     * @param array
     * @param ofs 
     */
    public DoubleArray(DoubleArray array, int ofs) {
        _data = array._data;
        _ofs = array._ofs + ofs;
        if (_ofs >= _data.length) {
            throw new IndexOutOfBoundsException(array._ofs + " + " + ofs + 
                    " = " + _ofs + " >= " + _data.length);
        }
    }
    
    /**
     * @return double value at position 0;
     */
    public double getAt0() {
        return _data[_ofs];
    }
    
    /**
     * @param ofs 
     * @return double value at position ofs;
     */
    public double getAt(int ofs) {
        return _data[_ofs + ofs];
    }
    
    /**
     * Set double value at position 0;
     * @param d 
     */
    public void setAt0(double d) {
        _data[_ofs] = d;
    }
    
    /**
     * Set double value at position ofs;
     * @param ofs 
     * @param d 
     */
    public void setAt(int ofs, double d) {
        _data[_ofs + ofs] = d;
    }
    
    /**
     * @param data
     */
    public void setData(double[] data) {
        if (data.length + _ofs >= _data.length) {
            throw new IndexOutOfBoundsException(data.length + " + " + 
                    _ofs + " = " + (data.length + _ofs) + " >= " + _data.length);
        }
        System.arraycopy(data, 0, _data, _ofs, data.length);
    }
    
    /**
     * @param array
     */
    public void setData(DoubleArray array) {
        double[] data = array._data;
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
    public double[] cloneData() {
        double[] ret = new double[_data.length - _ofs];
        System.arraycopy(_data, _ofs, ret, 0, _data.length - _ofs);
        return ret;
    }
    
    /**
     * @param ofs
     * @param len
     * @return cloned double[]
     */
    public double[] cloneData(int ofs, int len) {
        double[] ret = new double[len];
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
}
