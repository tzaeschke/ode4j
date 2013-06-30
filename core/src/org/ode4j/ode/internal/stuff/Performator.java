package org.ode4j.ode.internal.stuff;

import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * @author Tilmann Zaeschke
 */
public final class Performator {

    private static final Map<String, Entry> _data = 
        Collections.synchronizedMap(new HashMap<String, Entry>());

    /**
     * Start time measurement.
     * @param key
     */
    public static final void begin(String key) {
        Entry e = _data.get(key);
        if (e == null) {
            e = new Entry();
            _data.put(key, e);
        }
        e.begin();
    }

    /**
     * Stop time measurement.
     * @param key
     */
    public static final void end(String key) {
        _data.get(key).end();
    }

    /**
     * Print results.
     */
    public static final void print() {
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
        public final void end() {
            _total += System.currentTimeMillis();
            _nCalls++;
        }

        /**
         * 
         */
        public final void begin() {
            _total -= System.currentTimeMillis();
        }

    }

    /**
     * 
     */
    public static final void reset() {
        _data.clear();
    }
}
