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

inline AtomicHybridAutomaton getTeleopSystem()
{
    // Declare some constants.
    RealConstant one("one",1.0_decimal);
    RealConstant freq("freq",0.25_decimal);
    RealConstant amp("amp",1.0_decimal);
    RealConstant alpha("alpha",0.2_decimal);
    RealConstant beta("beta",0.1_decimal);

    // Declare the shared system variables
    RealVariable t("t"); // Time
    RealVariable ref_s("ref_s");
    RealVariable velocity_m("velocity_m");
    RealVariable ref_m("ref_m");
    RealVariable old_ref_m("old_ref_m");
    RealVariable energy_tank_m("energy_tank_m");

    // Create the dotted real assignments for the two different locations
    DottedRealAssignments moving_ass(dot({t,ref_m,ref_s,energy_tank_m,old_ref_m}) = {one,amp*2*pi*freq*cos(2*pi*t*freq),velocity_m,-beta,0});
    DottedRealAssignments stationary_ass(dot({t,ref_m,ref_s,energy_tank_m,old_ref_m}) = {0,0,velocity_m,alpha,0});

    // Create the two possible events
    DiscreteEvent to_move("to_move");
    DiscreteEvent to_stop("to_stop");

    // Create the automaton
    AtomicHybridAutomaton teleopSystem("teleopSystem");

    // Declare the locations for the valve automaton
    AtomicDiscreteLocation moving("moving");
    AtomicDiscreteLocation stationary("stationary");

    // Each location has different assignments
    teleopSystem.new_mode(moving,moving_ass);
    teleopSystem.new_mode(stationary,stationary_ass);

    // Define the transitions for the two locations
    teleopSystem.new_transition(moving,to_stop,stationary,next({t,ref_m,ref_s,energy_tank_m,old_ref_m}) = {t,0,ref_s,energy_tank_m,ref_m},energy_tank_m<=0.1_decimal,EventKind::URGENT);
    teleopSystem.new_transition(stationary,to_move,moving,next({t,ref_m,ref_s,energy_tank_m,old_ref_m}) = {t,old_ref_m,ref_s,energy_tank_m,old_ref_m},energy_tank_m>=1.0_decimal,EventKind::URGENT);

    return teleopSystem;
}
