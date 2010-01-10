package org.ode4j.ode.internal.gimpact;

public interface GimConstants {
	static final int GUINT_BIT_COUNT = 32;
	static final int GUINT_EXPONENT = 5;

	
	
	/*! \defgroup MEMORY_ACCESS_CONSTANTS
	\brief
	Memory Access constants.
	\sa BUFFERS
	*/
	//! @{
//	#define G_MA_READ_ONLY 1
//	#define G_MA_WRITE_ONLY 2
//	#define G_MA_READ_WRITE 3
	static final int G_MA_READ_ONLY = 1;
	static final int G_MA_WRITE_ONLY = 2;
	static final int G_MA_READ_WRITE = 3;
	//! @}

	/*! \defgroup MEMORY_USAGE_CONSTANTS
	\brief
	Memory usage constants.
	\sa BUFFERS
	*/
	//! @{
	/// Don't care how memory is used
	static final int G_MU_EITHER = 0;
	/// specified once, doesn't allow read information
	static final int G_MU_STATIC_WRITE = 1;
	/// specified once, allows to read information from a shadow buffer
	static final int G_MU_STATIC_READ = 2;
	/// write directly on buffer, allows to read information from a shadow buffer
	static final int G_MU_STATIC_READ_DYNAMIC_WRITE = 3;
	/// upload data to buffer from the shadow buffer, allows to read information from a shadow buffer
	static final int G_MU_STATIC_READ_DYNAMIC_WRITE_COPY = 4;
	/// specified once, allows to read information directly from memory
	static final int G_MU_STATIC_WRITE_DYNAMIC_READ = 5;
	/// write directly on buffer, allows to read information directly from memory
	static final int G_MU_DYNAMIC_READ_WRITE = 6;
	//! @}

	/*! \defgroup BUFFER_ERRORS
	\brief
	Buffer operation errors
	\sa BUFFERS
	*/
	//! @{
//	#define G_BUFFER_OP_SUCCESS 0
//	#define G_BUFFER_OP_INVALID 1
//	#define G_BUFFER_OP_STILLREFCOUNTED 2
	static final int G_BUFFER_OP_SUCCESS = 0;
	static final int G_BUFFER_OP_INVALID = 1;
	static final int G_BUFFER_OP_STILLREFCOUNTED = 2;
	//! @}

	/*! \defgroup BUFFER_MANAGER_IDS
	\brief
	Buffer manager identifiers
	\sa BUFFERS, BUFFER_MANAGERS
	*/
	//! @{
	enum G_BUFFER_MANAGER
	{
		SYSTEM, //G_BUFFER_MANAGER_SYSTEM,
		SHARED; //G_BUFFER_MANAGER_SHARED,

		//MAX;//G_BUFFER_MANAGER__MAX
	};
	//! @}
}
