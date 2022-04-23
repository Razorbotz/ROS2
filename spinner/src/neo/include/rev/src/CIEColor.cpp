/*
 * Copyright (c) 2020-2021 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rev/CIEColor.h"

#include <algorithm>
#include <cmath>

using namespace rev;

static double CIERGB_f(double val) {
    return (val > 0.0031308) ? (1.055 * std::pow(val, 1 / 2.4) - 0.055)
                             : (12.92 * val);
}

frc::Color CIEColor::ToRGB() {
    double _X = std::clamp(X / 100, 0.0, 1.0);
    double _Y = std::clamp(Y / 100, 0.0, 1.0);
    double _Z = std::clamp(Z / 100, 0.0, 1.0);
    double r = _X * XYZtoRGB[0] + _Y * XYZtoRGB[1] + _Z * XYZtoRGB[2];
    double g = _X * XYZtoRGB[3] + _Y * XYZtoRGB[4] + _Z * XYZtoRGB[5];
    double b = _X * XYZtoRGB[6] + _Y * XYZtoRGB[7] + _Z * XYZtoRGB[8];

    r = CIERGB_f(r);
    g = CIERGB_f(g);
    b = CIERGB_f(b);

    return frc::Color(r, g, b);
}

constexpr double CIEColor::IlluminantD65[3] = {95.0489, 100.0, 108.8840};

constexpr double CIEColor::XYZtoRGB[9] = {3.2404542,  -1.5371385, -0.4985314,
                                          -0.9692660, 1.8760108,  0.0415560,
                                          0.0556434,  -0.2040259, 1.0572252};
