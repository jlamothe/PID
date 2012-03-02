// -*- mode: c++ -*-

/// @file PID.h

// PID Library
// Copyright (C) 2012 Jonathan Lamothe <jonathan@jlamothe.net>

// This program is free software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.

// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see: http://www.gnu.org/licenses/

#ifndef PID_H
#define PID_H

//
// CLASSES
//

/// @brief PID loop object.
class PID
{
public:

    double p_factor,            ///< @brief Proportional factor.
        i_factor,               ///< @brief Integral factor.
        d_factor,               ///< @brief Derivitive factor.
        bias,                   ///< @brief Output bias.
        min_i,                  ///< @brief Minimum integral.
        max_i,                  ///< @brief Maximum integral.
        min_out,                ///< @brief Minimum output.
        max_out;                ///< @brief Maximum output.

    /// @brief Indicates a reverse-action PID loop.
    bool reverse;

    /// @brief Constructor.
    /// @param p Proportional factor.
    /// @param i Integral factor.
    /// @param d Derivitive factor.
    /// @param b Output bias.
    /// @param r Indicates a reverse-action PID loop.
    PID(double p, double i, double d, double b = 0, bool r = false);

    /// @brief Resets the PID loop.
    void reset();

    /// @brief Calculates the current PID output.
    /// @param act The measured value.
    /// @pâram sp The desired setpoint.
    /// @param dt The time since the last calculation.
    /// @return The calculated output.
    double process(double act, double sp, double dt);

private:

    double last_err,            ///< @brief The last error value.
        last_i;                 ///< @brief The last integral value.

    /// @brief Calculates the integral value.
    /// @param err The current error value.
    /// @param dt The time since the last calculation.
    /// @return The calculated value.
    double calc_i(double err, double dt);

    /// @brief Calculates the derivitive value.
    /// @param err The current error value.
    /// @param dt The time since the last calculation.
    /// @return The calculated value.
    double calc_i(double err, double dt);

    /// @brief Calculates the final output value.
    /// @param p The proportional value.
    /// @param i The integral value.
    /// @param d The derivitive value.
    /// @return The calculated output.
    double calc_out(double p, double i, double d);

};

#endif  // PID_H

// jl
