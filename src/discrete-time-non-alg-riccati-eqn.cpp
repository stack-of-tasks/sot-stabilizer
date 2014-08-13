/*
 * Copyright 2014,
 * Mehdi Benallegue
 */

#include <iostream>
#include <sot-stabilizer/tools/discrete-time-non-alg-riccati-eqn.hh>

namespace sotStabilizer
{
  namespace tools
  {

    void discreteTimeNonAlgRiccatiEqn (
      const stateObservation::Matrix & A, const stateObservation::Matrix & B,
      const stateObservation::Matrix & Q, const stateObservation::Matrix & R,
      const stateObservation::Matrix & Qn, stateObservation::DiscreteTimeArray &P,
      unsigned n)
    {
      BOOST_ASSERT(A.rows()==A.cols() && "Ricatti equation requires square A matrix");
      BOOST_ASSERT(B.rows()==A.rows() && "Ricatti equation requires A and B have same row number");
      BOOST_ASSERT(Q.rows()==Q.cols() && "Ricatti equation requires square Q matrix");
      BOOST_ASSERT(Qn.rows()==Qn.cols() && "Ricatti equation requires square Qn matrix");
      BOOST_ASSERT(Qn.rows()==Q.cols() && "Ricatti equation requires Q and Qn of same shape");
      BOOST_ASSERT(Q.rows()==A.cols() && "Ricatti equation requires Q and A have same shape");
      BOOST_ASSERT(R.rows()==R.cols() && "Ricatti equation requires square R matrix");
      BOOST_ASSERT(R.cols()==B.cols() && "Ricatti equation requires R and B have same column number");

      P.reset();
      P.resize(n+1);

      P[n] = Q;


      for (int i=n-1; i>=0; --i)
      {
        P[i]= Q + A.transpose() * ( P[i+1] - P[i+1] *B*(R + B.transpose()*P[i+1]*B).inverse()*B.transpose()*P[i+1])*A;
      }

//      std::cout << "Riccatti difference "<< std::endl;
//      std::cout << P[0]-P[1] << std::endl;


//      std::cout << "Riccatti solutions "<< std::endl;
//      for (int i=0; i<=n; ++i)
//      {
//
//        std::cout << i<< " " << std::endl;
//        std::cout << P[i] << std::endl;
//
//      }


    }

  }
}
