/***************************************************************************
 *            teleop-system.hpp
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

inline AtomicHybridAutomaton getMotorControllers()
{
    // Declare some constants.
    RealConstant Kp("Kp",10.2_decimal);

    // Declare the shared system variables
    RealVariable torque_s("torque_s");
    RealVariable position_s("position_s");
    RealVariable ref_s("ref_s");
    RealVariable torque_m("torque_m");
    RealVariable position_m("position_m");
    RealVariable velocity_m("velocity_m");
    RealVariable ref_m("ref_m");
    RealVariable pos_err("pos_err");

    // Create the real assignment
    RealAssignments moving_ass( let({torque_m,torque_s,pos_err}) = {Kp*(ref_m-velocity_m),Kp*(ref_s-position_s),position_m-position_s});

    // Create the controllers automaton
    AtomicHybridAutomaton motorControllers("motorControllers");

    // Declare the location for the automaton
    AtomicDiscreteLocation moving("moving");

    // The controllers dynamics are always the same
    motorControllers.new_mode(moving,moving_ass);

    return motorControllers;
}
