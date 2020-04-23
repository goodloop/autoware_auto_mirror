#!/usr/bin/env python3

# Copyright 2020 Embotech AG, Zurich, Switzerland

# This module implements a small, but general simulator that interacts
# with application-specific code via some interfaces that are also defined here.

from abc import ABC, abstractmethod
import numpy as np
from typing import List

from scipy.integrate import odeint

# --- Define abstract interface classes for the simulation ---------------------------
class SerdeInterface(ABC):
    """Interface for objects implementing (de-)serialization to numpy ndarrays.
       The serialization can be used for purposes such as interfacing with integrators 
       from numerical packages like scipy.integrate.ode.
    """

    @abstractmethod
    def serialize(self) -> np.ndarray:
        """Turn the object into a numpy ndarray."""

    @staticmethod
    @abstractmethod
    def deserialize(serialized_state: np.ndarray) -> "SerdeInterface":
        """Turn the given ndarray into an object. Inverse of serialize()."""


class DynamicsInterface(ABC):
    """Interface for objects implementing a continuous system dynamics 
       function evaluation, that is a function of the form dx/dt = f(x,u),
       where x is the system state, u is the control input, and dx/dt is the
       derivative of the state for those arguments.

       The interface provides both structured and unstructured (as in, with
       serialized data) calls.
    """

    @abstractmethod
    def evaluate_dynamics(
        self, current_state: SerdeInterface, current_command: SerdeInterface
    ) -> SerdeInterface:
        """Should return the derivative of current state, given current_command 
           is being applied to the system inputs.
        """

    @abstractmethod
    def evaluate_dynamics_serialized(
        self, current_state: np.ndarray, current_command: np.ndarray
    ) -> np.ndarray:
        """Should return the derivative of current state, given current_command 
           is being applied to the system inputs. This can just be a wrapper call
           to evaluate_dynamics.
        """


class ControlInterface(ABC):
    @abstractmethod
    def compute_control(self, current_state: SerdeInterface) -> SerdeInterface:
        """Should compute the control inputs that should be applied to the system given
           the state measurement current_state.
        """


class SimulationInstant:
    """Data class for storing a single time instant of the simulation."""

    def __init__(self, state, command, time):
        self.state = state
        self.command = command
        self.time = time


class RecorderInterface(ABC):
    @abstractmethod
    def record_instant(self, instant: SimulationInstant):
        """Record a SimulationInstant to the history."""


class SimulationRecorderToMemory(RecorderInterface):
    """Class to record simulation history to memory (stored within the object)"""

    def __init__(self):
        self.history: List[SimulationInstant] = []

    def reset(self):
        """Clear history."""
        self.history = []

    def record_instant(self, instant: SimulationInstant):
        """Add an instant to the history."""
        self.history += [instant]


# --- Main simulation class ----------------------------------------------------------
class MiniSim:
    """Simple and small one-object, one-controller simulator. See the documentation of the
       interfaces of the classes required by the constructor for how to implement dynamics
       and controller.
    """

    def __init__(
        self,
        dynamics: DynamicsInterface,
        controller: ControlInterface,
        initial_state: SerdeInterface,
        step_time_seconds: float,
        listeners: dict = {},
    ):
        """Create a simulation object with the specified components. 
           Listeners is a dictionary of objects for callback functionality. Currently
           supported keys:
           - "recorder": object implementing RecorderInterface. Used to either record
             or otherwise publish each simulated simulated state and control input.
        """
        self.dynamics = dynamics
        self.controller = controller
        self.initial_state = initial_state
        self.step_time = step_time_seconds
        self.listeners = listeners
        self.simulation_time = 0.0

    def _integrate_dynamics(
        self, state: SerdeInterface, command: SerdeInterface
    ) -> SerdeInterface:
        # Create a wrapper that closes over the current command
        def ode_wrapper(y: np.ndarray, t: float) -> np.ndarray:
            return self.dynamics.evaluate_dynamics_serialized(y, command.serialize())

        # Integrate the ODE using that facade
        solution_trajectory = odeint(
            ode_wrapper, state.serialize().flatten(), [0, self.step_time]
        )

        # Deserialize again, using the static function of the state class,
        # accessed through the input state object.
        return state.deserialize(solution_trajectory[-1, :])

    def _simulation_iteration(self, current_state: SerdeInterface) -> SerdeInterface:
        """Perform one iteration of the simulation, taking the current state as an 
           input and returning the next state.
        """
        current_command = self.controller.compute_control(current_state)
        next_state = self._integrate_dynamics(current_state, current_command)
        if "recorder" in self.listeners:
            self.listeners["recorder"].record_instant(
                SimulationInstant(current_state, current_command, self.simulation_time)
            )
        return next_state

    def simulate(self, simulation_time):
        """Perform a simulation for a given amount of time."""
        self.simulation_time = 0.0

        number_of_steps = int(np.floor(simulation_time / self.step_time))
        current_state = self.initial_state

        for k in range(number_of_steps):
            current_state = self._simulation_iteration(current_state)
            self.simulation_time += self.step_time
