package org.ode4j;

import java.io.Serializable;

public class GWTAtomicIntegerArray implements Serializable {
    private static final long serialVersionUID = 2862133569453604235L;
    private final int[] array;

    public GWTAtomicIntegerArray(int length) {
        this.array = new int[length];
    }

    public GWTAtomicIntegerArray(int[] array) {
        this.array = array;
    }

    public final int length() {
        return this.array.length;
    }

    public final int get(int i) {
        return this.array[i];
    }

    public final void set(int i, int newValue) {
        this.array[i] =  newValue;
    }
//
//    public final void lazySet(int i, int newValue) {
//        AA.setRelease(this.array, i, newValue);
//    }
//
//    public final int getAndSet(int i, int newValue) {
//        return AA.getAndSet(this.array, i, newValue);
//    }

    public final boolean compareAndSet(int i, int expectedValue, int newValue) {
        if (this.array[i] == expectedValue) {
            this.array[i] = newValue;
            return true;
        }
        return false;
    }

//    /** @deprecated */
//    @Deprecated(
//            since = "9"
//    )
//    public final boolean weakCompareAndSet(int i, int expectedValue, int newValue) {
//        return AA.weakCompareAndSetPlain(this.array, i, expectedValue, newValue);
//    }
//
//    public final boolean weakCompareAndSetPlain(int i, int expectedValue, int newValue) {
//        return AA.weakCompareAndSetPlain(this.array, i, expectedValue, newValue);
//    }
//
//    public final int getAndIncrement(int i) {
//        return AA.getAndAdd(this.array, i, 1);
//    }
//
//    public final int getAndDecrement(int i) {
//        return AA.getAndAdd(this.array, i, -1);
//    }
//
//    public final int getAndAdd(int i, int delta) {
//        return AA.getAndAdd(this.array, i, delta);
//    }
//
//    public final int incrementAndGet(int i) {
//        return AA.getAndAdd(this.array, i, 1) + 1;
//    }
//
//    public final int decrementAndGet(int i) {
//        return AA.getAndAdd(this.array, i, -1) - 1;
//    }
//
//    public final int addAndGet(int i, int delta) {
//        return AA.getAndAdd(this.array, i, delta) + delta;
//    }
//
//    public final int getAndUpdate(int i, IntUnaryOperator updateFunction) {
//        int prev = this.get(i);
//        int next = 0;
//        boolean haveNext = false;
//
//        while(true) {
//            if (!haveNext) {
//                next = updateFunction.applyAsInt(prev);
//            }
//
//            if (this.weakCompareAndSetVolatile(i, prev, next)) {
//                return prev;
//            }
//
//            haveNext = prev == (prev = this.get(i));
//        }
//    }
//
//    public final int updateAndGet(int i, IntUnaryOperator updateFunction) {
//        int prev = this.get(i);
//        int next = 0;
//        boolean haveNext = false;
//
//        while(true) {
//            if (!haveNext) {
//                next = updateFunction.applyAsInt(prev);
//            }
//
//            if (this.weakCompareAndSetVolatile(i, prev, next)) {
//                return next;
//            }
//
//            haveNext = prev == (prev = this.get(i));
//        }
//    }
//
//    public final int getAndAccumulate(int i, int x, IntBinaryOperator accumulatorFunction) {
//        int prev = this.get(i);
//        int next = 0;
//        boolean haveNext = false;
//
//        while(true) {
//            if (!haveNext) {
//                next = accumulatorFunction.applyAsInt(prev, x);
//            }
//
//            if (this.weakCompareAndSetVolatile(i, prev, next)) {
//                return prev;
//            }
//
//            haveNext = prev == (prev = this.get(i));
//        }
//    }
//
//    public final int accumulateAndGet(int i, int x, IntBinaryOperator accumulatorFunction) {
//        int prev = this.get(i);
//        int next = 0;
//        boolean haveNext = false;
//
//        while(true) {
//            if (!haveNext) {
//                next = accumulatorFunction.applyAsInt(prev, x);
//            }
//
//            if (this.weakCompareAndSetVolatile(i, prev, next)) {
//                return next;
//            }
//
//            haveNext = prev == (prev = this.get(i));
//        }
//    }
//
//    public String toString() {
//        int iMax = this.array.length - 1;
//        if (iMax == -1) {
//            return "[]";
//        } else {
//            StringBuilder b = new StringBuilder();
//            b.append('[');
//            int i = 0;
//
//            while(true) {
//                b.append(this.get(i));
//                if (i == iMax) {
//                    return b.append(']').toString();
//                }
//
//                b.append(',').append(' ');
//                ++i;
//            }
//        }
//    }
//
//    public final int getPlain(int i) {
//        return AA.get(this.array, i);
//    }
//
//    public final void setPlain(int i, int newValue) {
//        AA.set(this.array, i, newValue);
//    }
//
//    public final int getOpaque(int i) {
//        return AA.getOpaque(this.array, i);
//    }
//
//    public final void setOpaque(int i, int newValue) {
//        AA.setOpaque(this.array, i, newValue);
//    }
//
//    public final int getAcquire(int i) {
//        return AA.getAcquire(this.array, i);
//    }
//
//    public final void setRelease(int i, int newValue) {
//        AA.setRelease(this.array, i, newValue);
//    }
//
//    public final int compareAndExchange(int i, int expectedValue, int newValue) {
//        return AA.compareAndExchange(this.array, i, expectedValue, newValue);
//    }
//
//    public final int compareAndExchangeAcquire(int i, int expectedValue, int newValue) {
//        return AA.compareAndExchangeAcquire(this.array, i, expectedValue, newValue);
//    }
//
//    public final int compareAndExchangeRelease(int i, int expectedValue, int newValue) {
//        return AA.compareAndExchangeRelease(this.array, i, expectedValue, newValue);
//    }
//
//    public final boolean weakCompareAndSetVolatile(int i, int expectedValue, int newValue) {
//        return AA.weakCompareAndSet(this.array, i, expectedValue, newValue);
//    }
//
//    public final boolean weakCompareAndSetAcquire(int i, int expectedValue, int newValue) {
//        return AA.weakCompareAndSetAcquire(this.array, i, expectedValue, newValue);
//    }
//
//    public final boolean weakCompareAndSetRelease(int i, int expectedValue, int newValue) {
//        return AA.weakCompareAndSetRelease(this.array, i, expectedValue, newValue);
//    }
}
