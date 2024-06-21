import numpy as np

class PIDController:
    def __init__(self,gains , dt):
        self.Kp = gains[0]
        self.Ki = gains[0]
        self.Kd = gains[0]
        self.prev_error = 0
        self.integral = 0
        self.dt

    def cacl(self, error, dt):
        dt = self.dt
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

class bug_zero_conditios:

    def quit_fw(self , arr):
        return self.eval_distance(arr[0] , arr[1] , arr[2] , arr[3] , arr[6] , arr[7]) and self.eval_time(arr[8] , arr[9])

    def eval_distance(self , x_pos , y_pos , x_tmp , y_tmp , x_target , y_target):
        d_tmp = np.sqrt((x_target - x_tmp)**2 + (y_target - y_tmp)**2) - 0.07
        d_r = np.sqrt((x_target - x_pos)**2 + (y_target - y_pos)**2)
        return d_tmp > d_r

    def eval_time(self , fw_t , current_t):
        return (current_t - fw_t) > 1

class bug_two_conditios:

    def quit_fw(self , arr):
        return self.eval_line(arr[4] , arr[5] , arr[0] , arr[1]) and self.eval_time(arr[8] , arr[9])

    def eval_line(self , x_init , y_init , x_target , y_target , x_pos , y_pos):
        tolerance = 0.06

        if x_init == x_target: x_target += 0.0001
                
        m = (y_target - y_init)/(x_target - x_init)
        b = y_init - m * x_init
        y_line = m * x_pos + b

        return y_line - tolerance < y_pos < y_line + tolerance
    
    def eval_time(self , fw_t , current_t):
        return (current_t - fw_t) > 1
