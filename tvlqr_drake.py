from pydrake.systems.controllers import (FiniteHorizonLinearQuadraticRegulatorOptions,
                                         FiniteHorizonLinearQuadraticRegulator)
from pydrake.trajectories import PiecewisePolynomial

from double_pendulum.utils.csv_trajectory import load_trajectory

def TVLQRController(csv_path, Q, R, Qf, robot_model, context, torque_limit= [0.0, 0.0]):

    # load the nominal trajectory and express it as a piecewise poly
    T, X, U = load_trajectory(csv_path=csv_path,
                                with_tau=True)
    x0 = PiecewisePolynomial.CubicShapePreserving(T, X, zero_end_point_derivatives=True)
    u0 = PiecewisePolynomial.FirstOrderHold(T, U[1])

    # tvlqr construction with drake
    options = FiniteHorizonLinearQuadraticRegulatorOptions()
    options.x0 = x0
    options.u0 = u0
    options.Qf = Qf
    options.input_port_index = robot_model.get_actuation_input_port().get_index()
    tvlqr = FiniteHorizonLinearQuadraticRegulator(robot_model,
                                                        context,
                                                        t0=options.u0.start_time(),
                                                        tf=options.u0.end_time(),
                                                        Q=Q,
                                                        R=R,
                                                        options=options)
    return tvlqr