// PID Library
// Copyright (C) 2012 Jonathan Lamothe <jonathan@jlamothe.net>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at
// your option) any later version.

// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see: http://www.gnu.org/licenses/

//
// INCLUDES
//

#include "PID.h"

//
// FUNCTION DEFINITIONS
//

PID::PID(double p, double i, double d, double b, bool r)
{
    p_factor = p;
    i_factor = i;
    d_factor = d;
    bias = b;
    min_i = -100;
    max_i = 100;
    min_out = 0;
    max_out = 100;
    reverse = r;
    reset();
}

double PID::reset()
{
    last_err = 0;
    last_i = 0;
}

double PID::process(double act, double sp, double dt)
{
    double err = act - sp;
    double i = calc_i(err, dt);
    double d = calc_d(err, dt);
    last_err = err;
    last_i = i;
    return calc_out(err, i, d);
}

double PID::calc_i(double err, double dt)
{
    double out = last_i + err * dt;
    if(out < min_i)
        out = min_i;
    else if(out > max_i)
        out = max_i;
    return out;
}

double PID::calc_d(double err, double dt)
{
    if(dt == 0)
        return 0;
    return (err - last_err) / dt;
}

double PID::calc_out(double p, double i, double d)
{
    double out = p_factor * p + i_factor * i + d_factor * d;
    if(reverse)
        out = bias - out;
    else
        out = bias + out;
    return out;
}

// jl
