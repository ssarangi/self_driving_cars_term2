#ifndef TWIDDLE_H
#define TWIDDLE_H

#include "PID.h"
#include <uWS/uWS.h>
#include <vector>
#include <limits>

enum TWIDDLE_STEP {
    START_TWIDDLE,
    FIRST_IF_STEP,
    RESET_SIMULATOR,
    AFTER_RESET_SIMULATOR,
};

class Twiddle {
public:
    Twiddle(double tol, int num_iterations) {
      int num_params = 3;  /// For now hardcode it to 3 since we need to update PID controller
      m_tolerance = tol;
      m_bestErr = 0.0;
      m_p = std::vector<double>(num_params, 0);
      m_dp = std::vector<double>(num_params, 1);
      m_isTwiddleActive = true;
      m_twiddleStep = TWIDDLE_STEP::START_TWIDDLE;
      m_currentParameterBeingTuned = 0;
      m_pPID = new PID();
      m_bestErr = std::numeric_limits<double>::max();
      m_numIterations = num_iterations;
      m_currentIteration = 0;
    }

    ~Twiddle() {
      if (m_pPID != nullptr)
        delete m_pPID;
    }

    double sum_dp();

    TWIDDLE_STEP step(double err);

    bool isActive();

    PID* getPID() { return m_pPID; }

    std::vector<double> getDP();

    std::vector<double> getP();

private:
    void updatePID();

private:
    double m_bestErr;
    double m_tolerance;
    std::vector<double> m_p;
    std::vector<double> m_dp;
    bool m_isTwiddleActive;
    TWIDDLE_STEP m_twiddleStep;
    int m_currentParameterBeingTuned;
    PID *m_pPID;
    int m_numIterations;
    int m_currentIteration;
};

#endif