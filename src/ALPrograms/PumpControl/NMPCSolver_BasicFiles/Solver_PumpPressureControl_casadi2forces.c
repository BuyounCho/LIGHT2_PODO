/*
 * CasADi to FORCESPRO Template - missing information to be filled in by createCasadi.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2021. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "Solver_PumpPressureControl.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif


#include "Solver_PumpPressureControl_casadi.h"



/* copies data from sparse matrix into a dense one */
static void sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, Solver_PumpPressureControl_callback_float *data, Solver_PumpPressureControl_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((Solver_PumpPressureControl_float) data[j]);
        }
    }
}




/* CasADi to FORCESPRO interface */
extern void Solver_PumpPressureControl_casadi2forces(Solver_PumpPressureControl_float *x,        /* primal vars                                         */
                                 Solver_PumpPressureControl_float *y,        /* eq. constraint multiplers                           */
                                 Solver_PumpPressureControl_float *l,        /* ineq. constraint multipliers                        */
                                 Solver_PumpPressureControl_float *p,        /* parameters                                          */
                                 Solver_PumpPressureControl_float *f,        /* objective function (scalar)                         */
                                 Solver_PumpPressureControl_float *nabla_f,  /* gradient of objective function                      */
                                 Solver_PumpPressureControl_float *c,        /* dynamics                                            */
                                 Solver_PumpPressureControl_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 Solver_PumpPressureControl_float *h,        /* inequality constraints                              */
                                 Solver_PumpPressureControl_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 Solver_PumpPressureControl_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* CasADi input and output arrays */
    const Solver_PumpPressureControl_callback_float *in[4];
    Solver_PumpPressureControl_callback_float *out[7];
	

	/* Allocate working arrays for CasADi */
	Solver_PumpPressureControl_callback_float w[9];
	
    /* temporary storage for CasADi sparse output */
    Solver_PumpPressureControl_callback_float this_f;
    Solver_PumpPressureControl_callback_float nabla_f_sparse[1];
    
    
    Solver_PumpPressureControl_callback_float c_sparse[1];
    Solver_PumpPressureControl_callback_float nabla_c_sparse[1];
            
    
    /* pointers to row and column info for 
     * column compressed format used by CasADi */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for CasADi */
	in[0] = x;
	in[1] = p;
	in[2] = l;
	in[3] = y;

	if ((stage >= 0) && (stage < 15))
	{
		out[0] = &this_f;
		Solver_PumpPressureControl_objective0rd_1(in, out, NULL, w, 0);
		out[0] = nabla_f_sparse;
		if (nabla_f != NULL)
		{
			Solver_PumpPressureControl_objective1rd_1(in, out, NULL, w, 0);
			nrow = Solver_PumpPressureControl_objective1rd_1_sparsity_out(0)[0];
			ncol = Solver_PumpPressureControl_objective1rd_1_sparsity_out(0)[1];
			colind = Solver_PumpPressureControl_objective1rd_1_sparsity_out(0) + 2;
			row = Solver_PumpPressureControl_objective1rd_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

        Solver_PumpPressureControl_rkfour_0(x, p, c, nabla_c, Solver_PumpPressureControl_dynamics_0rd_0, Solver_PumpPressureControl_dynamics_0, threadID);
	}

	if ((stage >= 15) && (stage < 16))
	{
		out[0] = &this_f;
		Solver_PumpPressureControl_objective0rd_16(in, out, NULL, w, 0);
		out[0] = nabla_f_sparse;
		if (nabla_f != NULL)
		{
			Solver_PumpPressureControl_objective1rd_16(in, out, NULL, w, 0);
			nrow = Solver_PumpPressureControl_objective1rd_16_sparsity_out(0)[0];
			ncol = Solver_PumpPressureControl_objective1rd_16_sparsity_out(0)[1];
			colind = Solver_PumpPressureControl_objective1rd_16_sparsity_out(0) + 2;
			row = Solver_PumpPressureControl_objective1rd_16_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}

	}


    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((Solver_PumpPressureControl_float) this_f);
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif
