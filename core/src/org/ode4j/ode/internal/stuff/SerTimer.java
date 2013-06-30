package org.ode4j.ode.internal.stuff;

import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * @author Tilmann Zaeschke
 */
public final class SerTimer {

    private final Map<String, Entry> _data = new HashMap<String, Entry>();
    private long _prev = 0;
    private int _nStart = 0;
    
 
    /**
     * Start time measurement.
     * @param key
     */
    public final void start() {
    	_prev = System.currentTimeMillis();
    	_nStart++;
    }

    
    /**
     * Get elapsed time since last take() or start().
     * @param key
     */
    public final void take(String key) {
    	long now = System.currentTimeMillis();
    	Entry e = _data.get(key);
    	if (e == null) {
    		e = new Entry();
    		_data.put(key, e);
    	}
    	_data.get(key).add(now - _prev);
    	_prev = now;
    }

    /**
     * Print results.
     */
    public final void print() {
        List<String> keys = new LinkedList<String>();
        keys.addAll(_data.keySet());
        Collections.sort(keys);
        for (String key: keys) {
            System.out.println(_data.get(key).print() + " :: " + key);
        }
    }

    private static final class Entry {
        long _total = 0;
        long _nCalls = 0;

        /**
         * @return Returns String representation
         */
        public final String print() {
            StringBuffer b = new StringBuffer();
            b.append("Calls: " + _nCalls);
            while (b.length() < 25) b.append(' ');
            b.append("Time [ms]: " + _total);
            while (b.length() < 50) b.append(' ');
            b.append("T/C [ms]: " + _total/(double)_nCalls);
            return b.toString();
        }

        /**
         * 
         */
        public final void add(long time) {
            _total += time;
            _nCalls++;
        }

    }

    /**
     * 
     */
    public final void reset() {
        _data.clear();
    }
    
    /**
     * 
     */
    public final int getNStart() {
    	return _nStart;
    }
}
