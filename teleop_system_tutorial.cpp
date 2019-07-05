/***************************************************************************
 *            teleop-system.cpp
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
#include "motor-master.hpp"
#include "motor-slave.hpp"
#include "motor-controllers.hpp"
#include "teleop-system.hpp"

using namespace Ariadne;
using std::cout; using std::endl;

Int main(Int argc, const char* argv[])
{
    Nat evolver_verbosity=get_verbosity(argc,argv);

    // Declare the shared system variables
    RealVariable t("t");
    RealVariable torque_s("torque_s");
    RealVariable position_s("position_s");
    RealVariable velocity_s("velocity_s");
    RealVariable ref_s("ref_s");
    RealVariable torque_m("torque_m");
    RealVariable position_m("position_m");
    RealVariable velocity_m("velocity_m");
    RealVariable ref_m("ref_m");
    RealVariable old_ref_m("old_ref_m");
    RealVariable energy_tank_m("energy_tank_m");
    RealVariable pos_err("pos_err");

    StringVariable motorControllers("motorControllers");
    StringVariable motorMaster("motorMaster");
    StringVariable motorSlave("motorSlave");
    StringVariable teleopSystem("teleopSystem");

    StringConstant moving("moving");
    StringConstant stationary("stationary");

    // Get the automata and compose them
    HybridAutomaton motorMaster_automaton = getMotorMaster();
    HybridAutomaton motorSlave_automaton = getMotorSlave();
    HybridAutomaton teleopSystem_automaton = getTeleopSystem();
    HybridAutomaton motorControllers_automaton = getMotorControllers();
    CompositeHybridAutomaton master_system({motorMaster_automaton,motorSlave_automaton,teleopSystem_automaton,motorControllers_automaton});

    // Print the system description on the command line
    cout << master_system << endl;

    // Compute the system evolution

    // Create a GeneralHybridEvolver object
    GeneralHybridEvolver evolver(master_system);
    evolver.verbosity = evolver_verbosity;

    // Set the evolution parameters
    evolver.configuration().set_maximum_enclosure_radius(20.05); // The maximum size of an evolved set before early termination
    evolver.configuration().set_maximum_step_size(1.25); // The maximum value that can be used as a time step for integration

    // Declare the type to be used for the system evolution
    typedef GeneralHybridEvolver::OrbitType OrbitType;

    std::cout << "Computing evolution... " << std::flush;

    // Define the initial set, by supplying the location as a list of locations for each composed automata, and
    // the continuous set as a list of variable assignments for each variable controlled on that location
    // (the assignment can be either a singleton value using the == symbol or an interval using the <= symbols)
    // N.B: It must be written in this way: {StringVariable|StringConstant,StringVariable|StringConstant,...}

    Real exe_time(10.0);
    Real start_time(0);
    Real axes_limits(3.0);
    HybridSet initial_set({motorMaster|moving,motorSlave|moving,teleopSystem|moving,motorControllers|moving},{position_m==0,velocity_m==0,ref_m==0,position_s==0,velocity_s==0,ref_s==0,t==0,energy_tank_m==0.5_decimal,old_ref_m==0});    
    // Define the evolution time: continuous time and maximum number of transitions
    HybridTime evolution_time(exe_time,50);
    // Compute the orbit using upper semantics
    OrbitType orbit = evolver.orbit(initial_set,evolution_time,Semantics::UPPER);
    std::cout << "done." << std::endl;

    std::cout << "Plotting trajectory... "<<std::endl;

    Axes2d time_ref_s_axes(start_time<=TimeVariable()<=exe_time,-axes_limits<=ref_s<=axes_limits);
    plot("ariadne_ref_s",time_ref_s_axes, Colour(0.0,0.5,1.0), orbit);
    std::cout << "ref_s done."<<std::endl;

    Axes2d time_ref_m_axes(start_time<=TimeVariable()<=exe_time,-axes_limits<=ref_m<=axes_limits);
    plot("ariadne_ref_m",time_ref_m_axes, Colour(0.0,0.5,1.0), orbit);
    std::cout << "ref_m done."<<std::endl;

    // Axes2d time_pos_err_axes(start_time<=TimeVariable()<=exe_time,-axes_limits<=pos_err<=axes_limits);
    // plot("ariadne_pos_err",time_pos_err_axes, Colour(0.0,0.5,1.0), orbit);
    // std::cout << "pos_err done."<<std::endl;

    Axes2d time_position_s_axes(start_time<=TimeVariable()<=exe_time,-axes_limits<=position_s<=axes_limits);
    plot("ariadne_pos_s",time_position_s_axes, Colour(0.0,0.5,1.0), orbit);
    std::cout << "pos_s done."<<std::endl;

    Axes2d time_position_m_axes(start_time<=TimeVariable()<=exe_time,-axes_limits<=position_m<=axes_limits);
    plot("ariadne_pos_m",time_position_m_axes, Colour(0.0,0.5,1.0), orbit);
    std::cout << "pos_m done."<<std::endl;

    Axes2d time_velocity_s_axes(start_time<=TimeVariable()<=exe_time, -axes_limits<=velocity_s<=axes_limits);
    plot("ariadne_vel_s",time_velocity_s_axes, Colour(0.0,0.5,1.0), orbit);
    std::cout << "vel_s done."<<std::endl;

    Axes2d time_velocity_m_axes(start_time<=TimeVariable()<=exe_time, -axes_limits<=velocity_m<=axes_limits);
    plot("ariadne_vel_m",time_velocity_m_axes, Colour(0.0,0.5,1.0), orbit);
    std::cout << "vel_m done."<<std::endl;

    Axes2d time_energy_tank_m_axes(start_time<=TimeVariable()<=exe_time, 0<=energy_tank_m<=1);
    plot("ariadne_energy_tank_m",time_energy_tank_m_axes, Colour(0.0,0.5,1.0), orbit);
    std::cout << "energy_tank_m done."<<std::endl;
   
    std::cout << "done." << std::endl;


}
