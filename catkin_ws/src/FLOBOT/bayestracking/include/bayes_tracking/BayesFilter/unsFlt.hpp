#ifndef _BAYES_FILTER_UNSCENTED
#define _BAYES_FILTER_UNSCENTED

/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id$
 */

/*
 * Unscented Filter Scheme.
 *  A Julier-Uhlmann Unscented non-linear Kalman filter
 *  Uses the classic implementation of the Duplex Unscented transform.
 * The Unscented transform is used for non-linear state and observation predictions
 *
 * Observations are fused using innovation gain equations from a Covariance filter
 *
 * Predictions of state and state covariance (and observation) use
 * Unscented transformations to interpolate the non-linear predict and observe
 * models. Unscented transforms can be further optimised by vary the Kappa
 * parameter from its usual value of 1.
 * Discontinuous observe models require that a normalisation function.
 *
 * The predict model is represented by the state prediction function and a
 * separate prediction noise matrix.
 * The observe model is represented by the observation prediction function and
 * a function to normalise observations.
 *
 * The filter is operated by performing a
 *  predict, observe
 * cycle defined by the base class
 */
#include "bayes_tracking/BayesFilter/bayesFlt.hpp"

/* Filter namespace */
namespace Bayesian_filter
{

class Unscented_predict_model : public Predict_model_base
/* Specific Unscented prediction model for Additive noise
 *  x(k|k-1) = f(x(k-1|k-1)) + w(x(k))
 *
 * Unscented filter requires
 *  f the function part of the non-linear model
 *  Q the covariance of the additive w(x(k)), w is specifically allow to be a function of state
 */
{
public:
	Unscented_predict_model (std::size_t q_size)
	{
		q_unscented = q_size;
	}

	virtual const FM::Vec& f(const FM::Vec& x) const = 0;
	// Functional part of additive model
	// Note: Reference return value as a speed optimisation, MUST be copied by caller.

	virtual const FM::SymMatrix& Q(const FM::Vec& x) const = 0;
	// Covariance of additive noise
	// Note: Reference return value as a speed optimisation, MUST be copied by caller.
private:
	friend class Unscented_filter;	// Filter implementation need to know noise size
	std::size_t q_unscented;
};


class Unscented_scheme : public Linrz_kalman_filter, public Functional_filter
{
private:
	std::size_t q_max;			// Maximum size allocated for noise model, constructed before XX
public:
	FM::ColMatrix XX;		// Unscented form of state, with associated Kappa
	Float kappa;

	Unscented_scheme (std::size_t x_size, std::size_t z_initialsize = 0);
	Unscented_scheme& operator= (const Unscented_scheme&);
	// Optimise copy assignment to only copy filter state

	void init ();
	void init_XX ();
	void update ();
	void update_XX (Float kappa);

	void predict (Unscented_predict_model& f);
	// Efficient Unscented prediction 
	void predict (Functional_predict_model& f);
	void predict (Additive_predict_model& f);
	Float predict (Linrz_predict_model& f)
	{	// Adapt to use the more general additive model
		predict(static_cast<Additive_predict_model&>(f));
		return 1.;		// Always well condition for additive predict
	}
	
	Float observe (Uncorrelated_additive_observe_model& h, const FM::Vec& z);
	Float observe (Correlated_additive_observe_model& h, const FM::Vec& z);
	// Unscented filter implements general additive observe models
	
	Float observe (Linrz_uncorrelated_observe_model& h, const FM::Vec& z)
	{	// Adapt to use the more general additive model
		return observe (static_cast<Uncorrelated_additive_observe_model&>(h),z);
	}
	Float observe (Linrz_correlated_observe_model& h, const FM::Vec& z)
	{	// Adapt to use the more general additive model
		return observe (static_cast<Correlated_additive_observe_model&>(h),z);
	}

public:						// Exposed Numerical Results
	FM::Vec s;					// Innovation
	FM::SymMatrix S, SI;		// Innovation Covariance and Inverse

protected:
	virtual Float predict_Kappa (std::size_t size) const;
	virtual Float observe_Kappa (std::size_t size) const;
	/* Unscented Kappa values
	   default uses the rule which minimise mean squared error of 4th order term
	*/

protected:					// allow fast operation if z_size remains constant
	std::size_t last_z_size;
	void observe_size (std::size_t z_size);

private:
	void unscented (FM::ColMatrix& XX, const FM::Vec& x, const FM::SymMatrix& X, Float scale);
	/* Determine Unscented points for a distribution */
	std::size_t x_size;
	std::size_t XX_size;	// 2*x_size+1

protected:			   		// Permanently allocated temps
	FM::ColMatrix fXX;
};


}//namespace
#endif
