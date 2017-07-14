#ifndef TWIDDLE_H
#define TWIDDLE_H

#include "PID.h"
#include <uWS/uWS.h>
#include <vector>
#include <limits>
#include <string>
#include <fstream>

enum TWIDDLE_STEP {
    INITIALIZE_TWIDDLE,
    ACCUMULATE_ERROR,
    START_TWIDDLE,
    FIRST_IF_STEP,
    RESET_SIMULATOR,
    AFTER_RESET_SIMULATOR,
    END_TWIDDLE,
};

class Twiddle {
public:
    Twiddle(std::string twiddle_name, double tol, int num_iterations, double Kp, double Ki, double Kd)
    : m_name(twiddle_name) {
      int num_params = 3;  /// For now hardcode it to 3 since we need to update PID controller
      m_tolerance = tol;
      m_bestErr = 0.0;
      m_p = {Kp, Kd, Ki};
      double dp_Kd = 0.5 * Kd;
      m_dp = {(Kp / dp_Kd) * Kp, dp_Kd, (Ki / dp_Kd) * Ki};
      m_isTwiddleActive = true;
      m_twiddleStep = TWIDDLE_STEP::INITIALIZE_TWIDDLE;
      m_currentParameterBeingTuned = 0;
      m_pPID = new PID();
      m_pPID->Init(Kp, Ki, Kd);
      m_bestErr = std::numeric_limits<double>::max();
      m_numIterations = num_iterations;
      m_currentIteration = -40;
      m_error = 0.0;
      m_csvFile.open(twiddle_name + ".csv");
    }

    ~Twiddle() {
      if (m_pPID != nullptr)
        delete m_pPID;

      m_csvFile.close();
    }

    double sum_dp();

    TWIDDLE_STEP step(double err);

    bool isActive();

    PID* getPID() { return m_pPID; }

    std::vector<double> getDP();

    std::vector<double> getP();

private:
    void updatePID(double error);

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
    double m_error;
    std::string m_name;
    std::ofstream m_csvFile;
};

#endif