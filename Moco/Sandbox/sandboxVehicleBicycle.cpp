/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSlidingMass.cpp                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/// This example optimizes the motion of a vehicle between provided initial
/// and final states, using a kinematic bicycle model of a vehicle.

#include <OpenSim.h>
#include <Moco/osimMoco.h>
using namespace OpenSim;

/// This actuator adds a control variable to the model and applies no forces.
/// The expectation is that other components in the model will use the control
/// variables.
class ControlVariable : public ScalarActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ControlVariable, ScalarActuator);
public:
    ControlVariable() = default;
private:
    double computeActuation(const SimTK::State&) const final { return 0; }
};

/// This is a custom component that provides a kinematic bicycle model of a
/// vehicle. The model has 4 states, 2 controls, and 2 parameters.
class VehicleBicycle : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(VehicleBicycle, Component);

public:
    OpenSim_DECLARE_PROPERTY(length_rear, double,
            "Distance between center rear wheel and center of mass.");
    OpenSim_DECLARE_PROPERTY(length_front, double,
            "Distance between center front wheel and center of mass.");
    OpenSim_DECLARE_PROPERTY(default_x, double,
            "Default value of the x (horizontal position) state variable.");
    OpenSim_DECLARE_PROPERTY(default_y, double,
            "Default value of the y (horizontal position) state variable.");
    OpenSim_DECLARE_PROPERTY(default_inertial_heading, double,
            "Default value of the inertial_heading state variable.");
    OpenSim_DECLARE_PROPERTY(default_com_speed, double,
            "Default value of the mass center speed state variable.");
    OpenSim_DECLARE_PROPERTY(steering_angle, ControlVariable,
            "A control variable for the front wheel's steering angle.");
    OpenSim_DECLARE_PROPERTY(acceleration, ControlVariable,
            "A control variable for the vehicle's acceleration.");

    VehicleBicycle() {
        constructProperty_length_rear(1.0);
        constructProperty_length_front(1.0);
        constructProperty_default_x(0);
        constructProperty_default_y(0);
        constructProperty_default_inertial_heading(0);
        constructProperty_default_com_speed(0);
        ControlVariable steering_angle;
        steering_angle.setName("steering_angle");
        constructProperty_steering_angle(steering_angle);
        ControlVariable acceleration;
        acceleration.setName("acceleration");
        constructProperty_acceleration(acceleration);
    }
    VehicleBicycle(double lengthRear, double lengthFront) : VehicleBicycle() {
        set_length_rear(lengthRear);
        set_length_front(lengthFront);
    }

protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        addStateVariable("x");
        addStateVariable("y");
        addStateVariable("inertial_heading");
        addStateVariable("com_speed");
    }
    /// Define the dynamics of the model.
    void computeStateVariableDerivatives(const SimTK::State& s) const override {
        const double inertial_heading =
                getStateVariableValue(s, "inertial_heading");
        const double speed = getStateVariableValue(s, "com_speed");
        const double& length_rear = get_length_rear();
        const double& length_front = get_length_front();
        const double steering_angle = get_steering_angle().getControl(s);
        const double velocity_angle = atan(length_rear * tan(steering_angle) /
                                           (length_rear + length_front));
        setStateVariableDerivativeValue(s,
                "x", speed * cos(inertial_heading + velocity_angle));
        setStateVariableDerivativeValue(s,
                "y", speed * sin(inertial_heading + velocity_angle));
        setStateVariableDerivativeValue(s,
                "inertial_heading", speed / length_rear * sin(velocity_angle));

        const double acceleration = get_acceleration().getControl(s);
        setStateVariableDerivativeValue(s, "com_speed", acceleration);
    }
    void extendInitStateFromProperties(SimTK::State& state) const override {
        Super::extendInitStateFromProperties(state);
        setStateVariableValue(state, "x", get_default_x());
        setStateVariableValue(state, "y", get_default_y());
        setStateVariableValue(state,
                "inertial_heading", get_default_inertial_heading());
        setStateVariableValue(state, "com_speed", get_default_com_speed());
    }
};

int main() {
    Model model;
    auto vehicle = make_unique<VehicleBicycle>();
    vehicle->setName("vehicle");
    model.addComponent(vehicle.release());
    auto state = model.initSystem();

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelCopy(model);
    problem.setTimeBounds(0, {0.1, 5});
    // Move from (0, 0) to (5, 1).
    problem.setStateInfo("/vehicle/x", {0, 5}, 0, 5);
    problem.setStateInfo("/vehicle/y", {0, 5}, 0, 1);
    // Start and end with a heading of 0.
    problem.setStateInfo("/vehicle/inertial_heading",
            {-0.5 * SimTK::Pi, 0.5 * SimTK::Pi}, 0, 0);
    // Start and end at rest.
    problem.setStateInfo("/vehicle/com_speed", {-5, 5}, 0, 0);
    // Steering angle is in [-45, 45] degrees.
    problem.setControlInfo("/vehicle/steering_angle",
            {-0.25 * SimTK::Pi, 0.25 * SimTK::Pi});
    // Acceleration is in [-5, 2] m/s^2.
    problem.setControlInfo("/vehicle/acceleration", {-5, 2});

    // Minimize a mix of final time and control signals
    // (surrogate for rider comfort).
    problem.addGoal<MocoFinalTimeGoal>("time");
    problem.addGoal<MocoControlGoal>("control");

    auto rep = problem.createRep();

    MocoSolution solution = study.solve();
    solution.write("vehicle_bicycle_solution.sto");
}
