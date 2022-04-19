#include "gpc.h"
#include "gpc_utils.h"

void m_trans(const gsl_matrix*A, gsl_matrix*A_t){
	gsl_matrix_transpose_memcpy(A_t,A);
}

void m_mult(const gsl_matrix*A, const gsl_matrix*B, gsl_matrix*AB){
	gsl_blas_dgemm(CblasNoTrans,CblasNoTrans,1.0,A,B,0.0,AB);
}

void m_add_to(const gsl_matrix*A, gsl_matrix*B){
	gsl_matrix_add(B, A);
}

void m_scale(double m, gsl_matrix*A){
	gsl_matrix_scale(A,m);
}

void m_add (const gsl_matrix*A, const gsl_matrix*B, gsl_matrix*ApB){
	gsl_matrix_memcpy(ApB,A);
	gsl_matrix_add(ApB,B);	
}

void m_inv(const gsl_matrix*A, gsl_matrix*invA) {
	unsigned int n = A->size1;
	gsl_matrix * m = gsl_matrix_alloc(n,n);
	gsl_matrix_memcpy(m,A);
	gsl_permutation * perm = gsl_permutation_alloc (n);
	/* Make LU decomposition of matrix m */
	int s;
	gsl_linalg_LU_decomp (m, perm, &s);
	/* Invert the matrix m */
	gsl_linalg_LU_invert (m, perm, invA);
	gsl_permutation_free(perm);
	gsl_matrix_free(m);
}

double m_det(const gsl_matrix*A) {
	unsigned int n = A->size1;
	gsl_matrix * m = gsl_matrix_alloc(n,n);
	gsl_matrix_memcpy(m,A);
	gsl_permutation * perm = gsl_permutation_alloc (n);
	int sign;
	gsl_linalg_LU_decomp (m, perm, &sign);
	double det = gsl_linalg_LU_det(m, sign);
	
	gsl_permutation_free(perm);
	gsl_matrix_free(m);
	return det;
}

double m_dot(const gsl_matrix*A,const gsl_matrix*B) {
	double sum = 0;
	unsigned int j;
	for(j=0;j<A->size2;j++)
		sum += gmg(A,0,j)*gmg(B,j,0);
	return sum;
}

int poly_real_roots(unsigned int n, const double*a, double *roots) {
	double z[(n-1)*2];
	gsl_poly_complex_workspace * w  = gsl_poly_complex_workspace_alloc(n);
	if(GSL_SUCCESS != gsl_poly_complex_solve (a, n, w, z)) {
		return 0;
	}
	gsl_poly_complex_workspace_free (w);
	
	unsigned int i=0;
	for(i=0;i<n-1;i++) {
		roots[i] = z[2*i];
	}	
	return 1;
}


int poly_greatest_real_root(unsigned int n, const double*a, double *root) {
	double z[(n-1)*2];
	gsl_poly_complex_workspace * w  = gsl_poly_complex_workspace_alloc(n);
	if(GSL_SUCCESS != gsl_poly_complex_solve (a, n, w, z)) {
		return 0;
	}
	gsl_poly_complex_workspace_free (w);
	if(TRACE_ALGO) {
		printf("Solving the equation\n a = [");
		for(unsigned int i=0;i<n;i++) {
			printf("%lf ", a[i]);
		}
		printf("]\n");
	}

	unsigned int i;
	double lambda = 0; int assigned = 0;
	for (i = 0; i < n-1; i++) {
		if(TRACE_ALGO) {
			fprintf (stderr, "root z%d = %+.18f + %+.18f i \n", i, z[2*i], z[2*i+1]);
		}
/*		 XXX ==0 is bad */
		if( z[2*i+1]==0 ) /* real root */
			if(!assigned || (z[2*i]>lambda)) {
				assigned = 1;
				lambda = z[2*i];
			}
	}
	if(TRACE_ALGO)
		fprintf (stderr, "lambda = %+.18f \n", lambda); 
	if(!assigned) {
		fprintf(stderr, "poly_greatest_real_root: Could not find real root for polynomial.\n");
		fprintf(stderr, "polynomial coefficients : ");
		for (i = 0; i < n; i++)
			fprintf(stderr, " %lf ", a[i]);
		fprintf(stderr, "\nRoots:\n");
		
		for (i = 0; i < n-1; i++)
			fprintf (stderr, "root z%d = %+.18f + %+.18f i \n", i, z[2*i], z[2*i+1]);

		return 0;
	}
	
	*root = lambda;
	return 1;
}

void m_display(const char*str, gsl_matrix*m) {
	printf("%s= \n", str);
	unsigned int i,j;
	for(i=0;i<m->size1;i++) {
		printf("   ");
		for(j=0;j<m->size2;j++)
			printf("%e ", gmg(m,i,j));
		printf("\n");
	}
}

