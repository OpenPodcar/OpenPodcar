Easy GSL: Making GSL easy for simple tasks.
=======================================

This is a small C library for _making_ **simple** _matrix 
computations_ **easy**. It is built on top of
[the GNU Scientific Library](http://www.gnu.org/software/gsl/).

@toc
* toc

Introduction
------------

Albeit very powerful, the GSL is definitely not user-friendly for
making simple matrix computations.
Instead, `EGSL` will try to *fool you into thinking that you are using Matlab*.

Yes, it's that easy! You can forget all of that `gsl_matrix_alloc`
and `gsl_matrix_free`.

### Download ###

Download from [here](http://purl.org/censi/2006/egsl).


The two main features of <tt>EGSL</tt>
--------------------------------------

### Feature #1: Automatic (de)allocation of matrices ###

Its main feature is that matrices are automatically allocated and 
deallocated. For example, you can write:

	egsl_push();
	
		val v1 = zeros(10,10);
		val v2 = ones(10,10);
	
		egsl_print("v1+v2=", sum(v1, v2) );
	
	egsl_pop();

and not worry about (de)allocation.


### Feature #2: Caching of matrices</subsubsection number=no>

This feature makes `EGSL` faster than any other C++ equivalent that
uses objects. Consider this code:

	egsl_push();
	
	val v1 = zeros(10,10);
	
	for(int i=0;i<1000000;i++) {
		
		egsl_push();
		
			val v2 = zeros(10,10);
			// make some operation on v2
			....
			// add v2 to v1
			add_to(v1, v2);
		
		egsl_pop();
		
	}
	
	egsl_pop();
	
	
	// Prints statistics about EGSL's usage of memory
	egsl_print_stats();

The output of this program is:

	egsl: total allocations: 2   cache hits: 999999

Even though the loop executes one million times, the total number
of matrix allocations is 2. Note that there is an inner context.
When the loop runs the first time, the `gsl_matrix` for `v2` is allocated.
However, when the `egsl_pop()` is reached, this matrix is not deallocated.
When the loop runs the second time, `EGSL` detects that you already
allocated a matrix of the requested size and re-utilizes the memory.

##  Usage</subsection>

For `EGSL` to work, you must remember some simple rules:

1. Always include your code between a pair of `egsl_push`/`egsl_pop` calls.
2. All values are returned as `val` structs: `val` object are not valid outside of the context they are created into (unless you tell `EGSL`).


### Start with a <tt>egsl\_push()</tt>, end with a <tt>egsl\_pop()</tt>

A first, simple program may look like this:

	#include <egsl.h>
	
	int main() {
		// First of all, push a context
		egsl_push();

			// Allocates a matrix
			val m1 = egsl_zeros(10, 10);
		
			// Print it
			egsl_print("Your first matrix:", m1);
		
		egsl_pop();
		
		// Wrong: you can't use m1 after egsl_pop()
		// egsl_print("Your first matrix:", m1);
	}

### Promotion of `val`s to previous contexts

If you want to make a matrix valid also for the previous context, 
you may use the `egsl_promote(val)` function.

	#include <egsl.h>
	
	int main() {
		// First of all, push a context
		egsl_push();
		
			// push another
			egsl_push();
			
				// Allocates a matrix
				val m1 = egsl_zeros(10, 10);
	
				val m2 = egsl_promote(m1);
			
			egsl_pop();
	
		// ok, m2 is valid
		egsl_print("m2 is valid here", m2);
		// error: m1 is not valid here
		// egsl_print("m1 is not valid here!", m1);
		
		
		egsl_pop();
	}

### Useful macros

If you use the include 

	#include <egsl_macros.h>

you can use a number of short macros instead of the long-named functions
`egsl_*`. Most of these macros are inspired by Matlab syntax.

For example, consider this line of Matlab code (taken from an actual program):

	a = -2 * C_k * v_j1 +  2 * dC_drho_j1 * v2'

Translated into EGSL, with macros defined in `<egsl_macros.h>`, it turns
into:

	val a = sum(sc(-2,m(C_k,v_j1)), sc(2,m(dC_drho_j1,tr(v2))));

where

* `sum(A, B)` is equivalent to `A + B`
* `sc(a, B)` is scalar--matrix multiplication: `a * B`
* `m(A, B)` is matrix--matrix multiplication; `A + B`

This is the best that we can do without operator overloading. 

Notice that for evaluating this expression `EGSL` needs a lot of 
intermediate matrices, and it's managing all memory allocations for you.
How many `egsl_matrix_alloc` would you need to write the same code in `GSL`?

##  Library reference</subsection>

This is a summary of available operations. It is shown both the
extended function name (beginning with `egsl_`) and the macro
defined in `<egsl_macros.h>`

### Allocation of matrices

* `zeros(rows, cols)` (`egsl_zeros`): Allocates a matrix with all zeros.
* `ones(rows, cols)` (`egsl_ones`): Allocates a matrix with all zeros.

### Operation with matrices

All this functions return a new `val`:

* `sum(A, B)` (`egsl_sum`):  Sums two matrices.
* `sc(a, B)` (`egsl_scale`): Represents scalar--matrix multiplication: `a * B`
* `m(A, B)` (`egsl_mult`): Represents matrix--matrix multiplication; `A * B`
* `inv(A)` (`egsl_inverse`): Returns the inverse of `A`.

### Accessing matrix elements

* `egsl_glsm(v)` will return you the underlying `gsl_matrix` for `v`. 
	For example:

		val v = egsl_zeros(10,10);
		gsl_matrix* m = egsl_gslm(v);
		// now do whatever you want with m


### Eigenvalues and eigenvectors

### Conversions

### Printing






