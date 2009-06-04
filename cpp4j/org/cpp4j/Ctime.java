package org.cpp4j;

public class Ctime extends Ctype {
	
	/**
	 * Time type.
	 * 
	 * Type capable of representing times and support arithmetical operations.
	 * This type is returned by the time function and is used as parameter by 
	 * some other functions of the <ctime> header.
	 * 
	 * It is almost universally expected to be an integral value representing 
	 * the number of seconds elapsed since 00:00 hours, Jan 1, 1970 UTC. This 
	 * is due to historical reasons, since it corresponds to a unix timestamp, 
	 * but is widely implemented in C libraries across all platforms.
	 *
	 * @author Tilmann Zaeschke
	 * @deprecated In Java, simply use 'long'.
	 */
	public static class time_t {
		public long seconds;

		public time_t(int n) {
			seconds = n;
		}
	}
	
	/**
	 * Get current time.
	 * Get the current calendar time as a time_t object.
	 * The function returns this value, and if the argument is not a null
	 * pointer, the value is also set to the object pointed by timer.
	 * 
	 * @param timer Pointer to an object of type time_t, where the time 
	 * value is stored.
	 * Alternatively, this parameter can be a null pointer, in which case 
	 * the parameter is not used, but a time_t object is still returned by 
	 * the function.
	 * @return The current calendar time as a time_t object.
	 * If the argument is not a null pointer, the return value is the same 
	 * as the one stored in the location pointed by the argument.
	 * If the function could not retrieve the calendar time, it returns 
	 * a -1 value.
	 */
	public static time_t time(time_t timer) {
		int n = (int)( (double)System.currentTimeMillis()/1000. );
		if (timer != null) {
			timer.seconds = n;
			return timer; 
		}
		return new time_t(n);
	}
}
