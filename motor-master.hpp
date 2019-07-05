/***************************************************************************
 *            motor-master.hpp
 *
 *  Copyright  2019 Eldison Dimo, Rudy Vicario
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <cstdarg>
#include "ariadne.hpp"

using namespace Ariadne;

inline AtomicHybridAutomaton getMotorMaster()
{
    // Declare the system constants
    RealConstant Jm("Jm",2.2_decimal);

    // Declare the variables for the dynamics
    RealVariable torque_m("torque_m");
    RealVariable position_m("position_m");
    RealVariable velocity_m("velocity_m");
    RealVariable ref_m("ref_m");

    // Create the dotted real assignment
    DottedRealAssignments moving_ass( dot({velocity_m,position_m}) = {torque_m/Jm,velocity_m} );

    // Create the motor automaton
    AtomicHybridAutomaton motorMaster("motorMaster");

    // Declare a discrete location
    AtomicDiscreteLocation moving("moving");

    // The motor movement is always given by the same dynamic
    motorMaster.new_mode(moving,moving_ass);

    return motorMaster;
}
