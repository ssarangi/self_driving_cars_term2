#include "twiddle.h"
#include "json.hpp"

#include <numeric>
#include <iostream>

using json = nlohmann::json;

double Twiddle::sum_dp() {
  double sum = 0.0;
  for (double d : m_dp) {
    sum += d;
  }
  return sum;
}

bool Twiddle::isActive() {
  if (!m_isTwiddleActive)
    return m_isTwiddleActive;

  if (m_isTwiddleActive) {
    double sumdp = sum_dp();
    if (sumdp <= m_tolerance) {
      m_isTwiddleActive = false;
      return false;
    }
  }

  return true;
}

std::vector<double> Twiddle::getDP() {
  return m_dp;
}

std::vector<double> Twiddle::getP() {
  return m_p;
}

void Twiddle::updatePID() {
  m_pPID->Kp = m_p[0];
  m_pPID->Kd = 4.0; // m_p[1];
  m_pPID->Ki = 0.0001; // m_p[2];
  m_pPID->p_error = 0.0;
  m_pPID->d_error = 0.0;
  m_pPID->i_error = 0.0;
}

/*
 * def twiddle(tol=0.2):
    # Don't forget to call `make_robot` before you call `run`!
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)
    # TODO: twiddle loop here

    while sum(dp) > tol:
        for i in range(len(p)):
            // START_TWIDDLE
            p[i] += dp[i]
            _, _, err = run(robot, p)

            // FIRST_IF_STEP
            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                _, _, err = run(robot, p)
                if err < best_err:
                    best_err = err
                    dp[i] *= 1.05
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9

    return p, best_err
 */

TWIDDLE_STEP Twiddle::step(double err) {
  if (m_currentIteration < m_numIterations) {
    // std::cout << "Accumulating Error Iteration #" << m_currentIteration << std::endl;
    m_error += err * err;
    m_currentIteration += 1;
    return TWIDDLE_STEP::ACCUMULATE_ERROR;
  }

  double avgError = m_error / m_numIterations;
  std::cout << "Average Error: " << avgError << std::endl;
  m_error = 0.0;

  if (m_twiddleStep == TWIDDLE_STEP::INITIALIZE_TWIDDLE) {
    m_bestErr = avgError;
    bool is_active = isActive();
    if (!is_active) {
      return TWIDDLE_STEP::END_TWIDDLE;
    }

    m_twiddleStep = TWIDDLE_STEP::START_TWIDDLE;
  }

  if (m_twiddleStep == TWIDDLE_STEP::START_TWIDDLE) {
    m_p[m_currentParameterBeingTuned] += m_dp[m_currentParameterBeingTuned];
    updatePID();
    m_twiddleStep = TWIDDLE_STEP::FIRST_IF_STEP;
    m_currentIteration = 0;
    return m_twiddleStep;
  } else if (m_twiddleStep == TWIDDLE_STEP::FIRST_IF_STEP) {
    if (avgError < m_bestErr) {
      m_bestErr = avgError;
      m_dp[m_currentParameterBeingTuned] *= 1.1;
      m_currentParameterBeingTuned = (m_currentParameterBeingTuned + 1) % 3;
      // m_currentParameterBeingTuned = 0;
      m_twiddleStep = TWIDDLE_STEP::START_TWIDDLE;
      return TWIDDLE_STEP::START_TWIDDLE;
    } else {
      m_p[m_currentParameterBeingTuned] -= 2 * m_dp[m_currentParameterBeingTuned];
      updatePID();
      m_twiddleStep = TWIDDLE_STEP::AFTER_RESET_SIMULATOR;
      m_currentIteration = 0;
      return TWIDDLE_STEP::RESET_SIMULATOR;
    }
  } else if (m_twiddleStep == TWIDDLE_STEP::AFTER_RESET_SIMULATOR) {
    if (avgError < m_bestErr) {
      m_bestErr = avgError;
      m_dp[m_currentParameterBeingTuned] *= 1.05;
    } else {
      m_p[m_currentParameterBeingTuned] += m_dp[m_currentParameterBeingTuned];
      updatePID();
      m_dp[m_currentParameterBeingTuned] *= 0.9;
    }

    m_currentParameterBeingTuned = (m_currentParameterBeingTuned + 1) % 3;
    // m_currentParameterBeingTuned = 0;
    m_twiddleStep = TWIDDLE_STEP::START_TWIDDLE;
    m_currentIteration = 0;
  }

  return TWIDDLE_STEP::START_TWIDDLE;
}