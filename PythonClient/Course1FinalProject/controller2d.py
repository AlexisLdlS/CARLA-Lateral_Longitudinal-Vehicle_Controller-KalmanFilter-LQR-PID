#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        if not hasattr(self, '_yaw0'):
            self._yaw0 = yaw
        self._current_yaw = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

         # Save idx of closest waypoint
        self._closest_wp_idx = min_idx

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        self.vars.last_steer_rad = input_steer_in_rad

        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        #            RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0
          
        ######################################################
        ######################################################
        #           DECLARATION OF USAGE VARIABLES 
        ######################################################
        ######################################################
        """
           'self.vars.create_var(<variable name>, <default value>)'
            persistent variables (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

        """
        # Vehicle Config
        self.wheelbase = 3    # Distance between axles (m)
        
        #Vehicle Old Measurements
        self.vars.create_var('vel_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)

        # PID Controller
        self.vars.create_var('error_previous', 0.0)
        self.vars.create_var('integral', 0.0)

        # Kalman Filter 
        self.ekf_x = np.array([[yaw], [0.0], [v]])      # Initial State [yaw, yaw_rate, v]
        self.ekf_P = np.eye(3) * 10                     # State covariance matrix (uncertainty over yaw, yaw_rate, v) // 0.1 is the initial uncertainity of each state
        self.ekf_Q = np.diag([0.05, 0.05, 0.2])         # Model process noise covariance // different noise levels for yaw, yaw_rate, and velocity // Defines how much we trust the model to predict the next state
        self.ekf_R = np.diag([0.02, 0.05])              # Sensor Measurement noise covariance // uncertainty of yaw and velocity measurements // Defines how much we trust the sensors


        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            #     IMPLEMENTATION OF LONGITUDINAL CONTROLLER
            #           PID with simple anti Windup
            ######################################################
            ######################################################
            
            # Vel Error
            vel_error = v_desired - v

            # Calculating Time Stap delta
            dt = t - self.vars.t_previous
            if dt <= 1e-6:
                # tiny dt → skip dynamics update, keep previous dt for derivative terms
                dt = 1e-6

            # Calculating error integral and derivative
            self.vars.integral += vel_error * dt
            derivative = (vel_error - self.vars.error_previous) / dt

            # PID gains 
            Kp = 3.0
            Ki = 0.6
            Kd = 0.1

            # Longitudinal Control signal
            u_unsat = Kp * vel_error + Ki * self.vars.integral + Kd * derivative

            # Definid Saturatiion of Control signal due to Actuator limits
            u = max(-1.0, min(u_unsat, 1.0))

            # Simple Anti-windup: if already saturated, do not integrate to avoid acumulating integral part
            if abs(u_unsat) >= 1.0:
                self.vars.integral -= vel_error * dt  #

            # Mapping to actuators
            if u >= 0.0:
                throttle_output = min(max(u, 0.0), 1.0)
                brake_output = 0.0
            else:
                throttle_output = 0.0
                brake_output = min(max(-u, 0.0), 1.0)

            ######################################################
            ######################################################
            #      KALMAN FILTER FOR YAW RATE ESTIMATION  
            ######################################################
            ######################################################
            
            # EKF Initialization
            if not hasattr(self, 'ekf_x') or self.ekf_x is None:
                self.ekf_x = np.array([[yaw], [0.0], [v]])
            if not hasattr(self, 'ekf_P') or self.ekf_P is None:
                self.ekf_P = np.eye(3) * 0.5

            # Approximating longitudinal acceleration to a = Δv/Δt
            a = (v - self.vars.vel_previous) / dt

            # Taking variables from the previous state estimation
            EKF_yaw_est = self.ekf_x[0,0]
            EKF_yaw_rate_est = self.ekf_x[1,0]
            EKF_v_est = self.ekf_x[2,0]
            EKF_steer_delta = getattr(self.vars, 'last_steer_rad', steer_output)

            # -------- PREDICTION (BICYCLE MODEL) --------
            # yaw_dot = yaw_rate
            # yaw_rate_dot = (v/L) * tan(delta)
            # v_dot = a

            yaw_pred = EKF_yaw_est + EKF_yaw_rate_est * dt
            yaw_rate_pred = EKF_yaw_rate_est - (EKF_v_est/self.wheelbase)*np.tan(EKF_steer_delta) * dt
            v_pred = EKF_v_est + a * dt

            # Updading state
            self.ekf_x = np.array([[yaw_pred],
                                [yaw_rate_pred],
                                [v_pred]])

            #--------  Linealization around the current state ------
            # Since yhe bicycle model is NON-LINEAR (for example since it includes trigonometric functions such as tan)
            # We can´t directly use the Kalman Filter, we need to LINEARIZE the model around the current state using the Jacobian F which also allows us to propagate the uncertainty of the state

            # General structure:
            #   - Diagonal entries are 1 → this means that, without any dynamics, the next state would equal the previous state (identity mapping).
            #   - Off-diagonal terms capture how each state depends on others through the system dynamics.
            #
            # For the bicycle model:
            #   State consists on
            #   - yaw[k+1] ≈ yaw[k] + yaw_rate * dt
            #   - yaw_rate[k+1] ≈ yaw_rate[k] + (v / L) * tan(delta) * dt
            #   - v[k+1] ≈ v[k]
            #
            # In F:
            # Row 1 (yaw equation):
            #   ∂/∂yaw      = 1   → yaw depends on itself
            #   ∂/∂yaw_rate = dt  → yaw grows with yaw_rate
            #   ∂/∂v        = 0   → yaw does not depend directly on velocity
            #
            # Row 2 (yaw_rate equation):
            #   ∂/∂yaw      = 0   → yaw not present in this equation
            #   ∂/∂yaw_rate = 1   → yaw_rate depends on itself
            #   ∂/∂v        = (tan(delta)/L) * dt → yaw_rate grows with velocity
            #
            # Row 3 (velocity equation):
            #   ∂/∂yaw      = 0
            #   ∂/∂yaw_rate = 0
            #   ∂/∂v        = 1   → velocity depends on itself

            F = np.array([
                [1, dt, 0],
                [0, 1, -(np.tan(EKF_steer_delta)/self.wheelbase) * dt],
                [0, 0, 1]
            ])

            # Predicted state covariance
            # P=(F)(P)(Fᵀ)+Q
            # Uncertainty is propagated by dynamics (F) and increased by model noise (Q).
            self.ekf_P = F @ self.ekf_P @ F.T + self.ekf_Q

            # -------- Updating Measurements --------
            # z = Hx → H select yaw and vel from state
            z = np.array([yaw,v])

            # Measurement-state relationship
            # H = matrix relating state to measurements
            # This means only yaw and velocity are directly observable
            H = np.array([
                [1, 0, 0],  # Measuring Yaw
                [0, 0, 1]   # Measuring Vel
            ])

            
            # Predicted measurement
            # zpred = H (x)
            z_pred = H @ self.ekf_x

            # Innovation (residual)
            # Difference between what we measure and what the model predicts.
            # Indicates how much we should correct the estimated state
            # y= z − zpredicted
            y_res = z - z_pred

            # S=(H)(P)(Hᵀ)+R
            # Combines the uncertainty of prediction with that of measurement.
            S = H @ self.ekf_P @ H.T + self.ekf_R

            # Kalman gain
            # Determines how much weight we give to the measurement vs model
            # Example: if R is large (noisy measurement) → K is small → we trust prediction more
            # K=(P)(Hᵀ)(S^-1)
            K = self.ekf_P @ H.T @ np.linalg.inv(S)

            # State correction
            # Applying the correction optimized by K
            # x = x+(K)(y)
            self.ekf_x = self.ekf_x + K @ y_res

            # Updating covariance
            # Uncertainty decreases after incorporating measurement
            # P=(I−KH)P     
            I = np.eye(3)
            self.ekf_P = (I - K @ H) @ self.ekf_P

            # We save in self.vars for use in control
            self.vars.ekf_yaw = self.ekf_x[0,0]
            self.vars.ekf_yaw_rate = self.ekf_x[1,0]
            self.vars.ekf_v = self.ekf_x[2,0]

            # Actualizamos yaw previo para la siguiente iteración
            self.vars.yaw_previous = yaw

            ######################################################
            ######################################################
            #      IMPLEMENTATION OF LATERAL CONTROLLER 
            #         Linear Cuadratic Regulator (LQR) 
            ######################################################
            ######################################################
      
            # 1) WAYPOINT SELECTION (synchronized with desired speed)
            # In this section, we determine the target waypoint to follow
            # and compute the reference heading (ψ_ref) based on the local
            # path tangent. This provides the geometric reference for both
            # lateral (e_y) and heading (e_ψ) errors.

            # Retrieve the nearest index (already calculated in update_desired_speed)
            closest_idx = getattr(self, "_closest_wp_idx", 0)

            # Ensure index remains within array bounds
            closest_idx = int(np.clip(closest_idx, 0, len(waypoints) - 1))

            # # Save the index for reference
            self.vars.last_wp_idx = closest_idx

            # normalize angle difference to [-pi, pi] to constrain orientation to [-π, π]
            def wrap_to_pi(angle):
                return (angle + np.pi) % (2*np.pi) - np.pi
            
            # Current reference waypoint position (closest point on path)
            wp_x = waypoints[closest_idx][0]
            wp_y = waypoints[closest_idx][1]

            # Compute local path heading ψ_ref using the next waypoint segment.
            # This defines the tangent direction of the desired trajectory.
            dx = waypoints[closest_idx + 1][0] - waypoints[closest_idx][0]
            dy = waypoints[closest_idx + 1][1] - waypoints[closest_idx][1]
            psi_ref = wrap_to_pi(np.arctan2(dy, dx))

            # 2) COMPUTATION OF LATERAL AND HEADING ERRORS
            # The heading error (e_ψ) represents how misaligned the vehicle’s
            # orientation is with respect to the local path tangent.
            # The cross-track error (e_y) measures the lateral displacement of
            # the vehicle relative to the reference path, expressed in the
            # vehicle's local frame.

            # Heading error:
            # Positive when the vehicle is rotated counterclockwise relative to the path direction.
            e_psi = wrap_to_pi(psi_ref - self.vars.ekf_yaw)

            # Cross-track error:
            # Projection of the position difference (vehicle - waypoint)
            # into the direction perpendicular to the path.
            # e_y = sin(psi_ref)*(x_wp - x) - cos(psi_ref)*(y_wp - y)
            e_y = np.sin(psi_ref)*(x - wp_x) - np.cos(psi_ref)*(y - wp_y)

            # 3) STATE VECTOR FOR LQR CONTROLLER
            # The controller uses a linearized state-space model based on
            # a simplified "bicycle" vehicle model around the operating point.
            #
            # State vector:
            #   x_state = [ e_y, e_ψ, yaw_rate_est ]ᵀ
            #   e_y         : lateral deviation from path (m)
            #   e_ψ         : heading error (rad)
            #   yaw_rate_est: yaw rate estimated by EKF (rad/s)
            #
            # Control input:
            #   δ (delta)   : front wheel steering angle (rad)
            #
            # The goal of the LQR is to compute δ that minimizes a quadratic cost function
            # combining tracking error (xᵀQx) and control effort (δᵀRδ).
            # x_state = [e_y, e_psi, yaw_rate_est]^T
            ekf_yaw_rate = self.vars.ekf_yaw_rate if hasattr(self.vars, 'ekf_yaw_rate') else 0.0
            x_state = np.array([[e_y],
                                [e_psi],
                                [ekf_yaw_rate]])   # 3x1

            # 4) LINEARIZED LATERAL DYNAMICS MODEL (A, B)
            # Linearized continuous-time model around small angles (tan(δ) ≈ δ)
            # and constant forward velocity (v).
            #
            #   e_y_dot   ≈ v * e_ψ
            #   e_ψ_dot   ≈ yaw_rate
            #   yaw_rate_dot ≈ (v / L) * δ
            #
            # where L = wheelbase (m), v = forward velocity (m/s)
            # This simplified model captures the essential coupling between
            # steering, heading, and lateral deviation.
            L = self.wheelbase
            v_for_model = max(v, 0.1)

            # dynamics approx:
            # e_y_dot   ≈ v * e_psi
            # e_psi_dot ≈ yaw_rate
            # yaw_rate_dot ≈ (v / L) * delta   (approx tan(delta) ≈ delta)
            A = np.array([
                [0.0, v_for_model, 0.0],
                [0.0, 0.0,        1.0],
                [0.0, 0.0,        0.0]
            ])
            B = np.array([
                [0.0],
                [0.0],
                [(v_for_model / L)]
            ])


            # 5) LQR DESIGN AND COST FUNCTION TUNING
            # The optimal control law minimizes:
            #     J = ∫ (xᵀ Q x + δᵀ R δ) dt
            #
            # where:
            #   Q → penalizes deviation from desired path/orientation
            #   R → penalizes control effort (large or abrupt steering)
            #
            # The solution of the continuous-time algebraic Riccati equation:
            #     AᵀP + PA - PBR⁻¹BᵀP + Q = 0
            # gives matrix P, from which the optimal feedback gain is:
            #     K = R⁻¹ Bᵀ P
            #
            # The resulting control law is δ = -Kx

            # Q penalizes lateral deviation, heading error, and yaw-rate.
            # Increasing Q(0,0) tightens lateral tracking.
            # Increasing Q(1,1) reduces orientation error.
            # Increasing Q(2,2) damps oscillations.
            Q = np.diag([1.0, 1.0, 0.5])

            # R penalizes control effort; higher R yields smoother but slower response.
            R = np.array([[10.0]])


            # Solve the continuous-time algebraic Riccati equation (CARE) using SciPy extension
            from scipy.linalg import solve_continuous_are
            P = solve_continuous_are(A, B, Q, R)

            # Compute the optimal LQR feedback gain matrix
            K = np.linalg.inv(R) @ B.T @ P
            self.vars.K_lqr = K

            # 6) COMPUTE STEERING COMMAND
            # Apply the optimal feedback control law.
            # Here we use a positive feedback convention consistent with CARLA’s
            # coordinate system and the sign of e_y.
            
            K_mod = K
            delta = (K_mod @ x_state).item()

            # Clamp steering within physical actuator limits
            max_steer = 1.22 # radians
            delta = float(np.clip(delta, -max_steer, max_steer))

            # assign steer_output
            steer_output = delta

        ######################################################
        # SET CONTROLS OUTPUT
        ######################################################
        self.set_throttle(throttle_output)  # in percent (0 to 1)
        self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
        self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
                        # STORE OLD VALUES
        ######################################################
        ######################################################
        # Updating persistant Variables
        self.vars.vel_previous = v  # Store forward speed to be used in next step
        self.vars.error_previous = vel_error
        self.vars.t_previous = t

        # Store EKF outputs
        self.vars.ekf_yaw = self.ekf_x[0,0]
        self.vars.ekf_yaw_rate = self.ekf_x[1,0]
        self.vars.ekf_v = self.ekf_x[2,0]

