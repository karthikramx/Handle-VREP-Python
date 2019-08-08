class PID:
    """
    PID controller class
    """

    def __init__(self,Kp=1.0,Ki=0.0,Kd=0.0,setpoint=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.previous_error = 0.0
        self.error_acc = 0.0
        self.anti_windup = True
        self.accumulated_min = -20
        self.accumulated_max = 20

    def control(self,setpoint=0,process_variable=0,dt=1):
        error           = setpoint - process_variable
        error_diff      = (error - self.previous_error)/dt
        self.error_acc +=error * dt

        if(self.anti_windup):
            if self.error_acc < self.accumulated_min:
                self.error_acc = self.accumulated_min
            elif self.error_acc > self.accumulated_max:
                self.error_acc  = self.accumulated_max

        P = self.Kp * error
        I = self.Ki * self.error_acc
        D = self.Kd * error_diff

        control = P + I + D
        self.previous_error = error
        return  control
