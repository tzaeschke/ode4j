package org.ode4j.ode.internal.gimpact;

public class GimMemory {
	/*! \defgroup MEMORY_FUNCTION_PROTOTYPES
	Function prototypes to allocate and free memory.
	*/
	//! @{
//	typedef void * gim_alloc_function (size_t size);
//	typedef void * gim_alloca_function (size_t size);//Allocs on the heap
//	typedef void * gim_realloc_function (void *ptr, size_t oldsize, size_t newsize);
//	typedef void gim_free_function (void *ptr, size_t size);
	interface gim_alloc_function { Object[] run(int size); };
	interface gim_alloca_function { Object[] run(int size); };//Allocs on the heap
	interface gim_realloc_function { Object[] run(Object[] ptr, int oldsize, int newsize); };
	interface gim_free_function { Object[] run(Object[] ptr, int size); };
	//! @}


}
