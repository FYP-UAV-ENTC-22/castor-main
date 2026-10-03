# system

The supervisor: a YASMIN state machine for the vehicle's modes, with behaviour
trees for task sequencing inside them, plus safety supervision and
mission-resilience (MRM) behaviour. Runs on every drone's Raspberry Pi.

The flight controller handles emergencies first and directly; this component
follows it into its emergency states and is never in the emergency path itself.
