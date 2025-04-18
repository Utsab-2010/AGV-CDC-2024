import numpy as np

class EKF:
    def __init__(self, dt, wheel_radius, wheel_base, process_noise, measurement_noise):
        """
        Initialize the EKF object.
        
        Parameters:
        - dt: Time step (seconds).
        - wheel_radius: Radius of the wheels (meters).
        - wheel_base: Distance between wheels (meters).
        - process_noise: Process noise covariance matrix (Q).
        - measurement_noise: Measurement noise covariance matrix (R).
        """
        self.dt = dt
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        
        # State vector [x, y, theta, v_x, v_y, omega]
        self.x = np.zeros(6)  # Initial state
        
        # Covariance matrix (initial uncertainty)
        self.P = np.eye(6) * 0.1
        
        self.z_pred = np.zeros(3) #predicted sensor observations

        # Process noise covariance matrix
        self.Q = process_noise
        
        # Measurement noise covariance matrix
        self.R = measurement_noise
        
        # # Measurement matrix H (maps state to measurements)
        # self.H = np.array([
        #     [1, 0, 0, 0, 0, 0],  # Maps x position
        #     [0, 1, 0, 0, 0, 0],  # Maps y position
        #     [0, 0, 1, 0, 0, 0]   # Maps orientation theta
        # ])
    
    def motion_model(self, u):
        """
        Predict the next state based on the motion model.
        
        Parameters:
        - u: Control input [v_left_ticks, v_right_ticks].
        
        Returns:
        - Predicted state vector.
        """
        v_left_ticks, v_right_ticks = u
        v_left = v_left_ticks * self.wheel_radius
        v_right = v_right_ticks * self.wheel_radius

        # Compute linear and angular velocities
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / self.wheel_base

        # Update state using motion equations
        theta = self.x[2]
        
        dx = v * np.cos(theta) * self.dt
        dy = v * np.sin(theta) * self.dt
        dtheta = omega * self.dt

        x_next = self.x.copy()
        x_next[0] += dx       # Update x position
        x_next[1] += dy       # Update y position
        x_next[2] += dtheta   # Update orientation (theta)
        
        x_next[3] = v         # Update linear velocity in x direction
        x_next[4] = 0         # Update linear velocity in y direction (approximation for differential drive)
        x_next[5] = omega     # Update angular velocity

        return x_next
    
    def compute_jacobian_F(self):
        """
        Compute the Jacobian matrix F for the motion model.
        
        Returns:
            Jacobian matrix F.
        """
        theta = self.x[2]
        
        F = np.eye(6)          # Start with an identity matrix
        F[0][2] = -self.dt * self.x[3] * np.sin(theta)   # Partial derivative of x w.r.t theta
        F[1][2] = self.dt * self.x[3] * np.cos(theta)    # Partial derivative of y w.r.t theta
        
        return F
    
    def compute_kalman_gain(self, P_predicted):
        """
        Compute the Kalman Gain K.
        
        Parameters:
            P_predicted: Predicted covariance matrix.
            
        Returns:
            Kalman Gain K.
        """
        S = self.H @ P_predicted @ self.H.T + self.R   # Innovation covariance
        K = P_predicted @ self.H.T @ np.linalg.inv(S)   # Kalman Gain formula
        return K
    
    def predict(self, u):
        """
        Perform the prediction step of the EKF.
        
        Parameters:
            u: Control input [delta_left_ticks, delta_right_ticks].
            
        Returns:
            Predicted state (x_predicted) and covariance (P_predicted).
        """
        # Predict state using motion model
        x_predicted = self.motion_model(u)
        
        # Compute Jacobian of motion model
        F_t = self.compute_jacobian_F()
        
        # Predict covariance
        P_predicted = F_t @ self.P @ F_t.T + self.Q
        
        # Update internal state and covariance
        self.x = x_predicted
        self.P = P_predicted
        self.z_pred = 
    
    return x_predicted, P_predicted

    def update(self, z):
        """
        Perform the update step of the EKF.
        
        Parameters:
            z: Measurement vector [yaw, angular_velocity_z, linear_acceleration_x].
            
        Returns:
            Updated state (x_updated) and covariance (P_updated).
        """
        # Compute Kalman Gain
        K_t = self.compute_kalman_gain(self.P)
        
        # Compute measurement residual
        y_t = z - self.H @ self.x
        
        # Update state and covariance
        x_updated = self.x + K_t @ y_t
        P_updated = (np.eye(6) - K_t @ self.H) @ self.P
        
        # Normalize orientation (theta) to [-pi, pi]
        x_updated[2] = np.arctan2(np.sin(x_updated[2]), np.cos(x_updated[2]))
        
        # Update internal state and covariance
        self.x = x_updated
        self.P = P_updated
        
        return x_updated, P_updated

    def step(self, u, z):
        """
        Perform a full EKF step (predict + update).
        
        Parameters:
            u: Control input [delta_left_ticks, delta_right_ticks].
            z: Measurement vector [yaw, angular_velocity_z, linear_acceleration_x].
        """
        self.predict(u)
        self.update(z)
                
