class Visualization:
    def __init__(self):
        self.wheel_radius = 0.12
        # Order: fr, br, bl, fl
        self.fl_xy = (0.35, 0.25)
        self.fr_xy = (0.35, -0.25)
        self.rl_xy = (-0.35, 0.25)
        self.rr_xy = (-0.35, -0.25)

    def wheel_velocity_visualization(self):

        
