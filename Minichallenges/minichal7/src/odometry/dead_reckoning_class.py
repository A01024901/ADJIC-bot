import numpy as np
np.set_printoptions(suppress = True) 
np.set_printoptions(formatter = {'float': '{: 0.4f}'.format})

class dead_reckoning:
    def __init__(self , dt , x , y , theta):
        ###--- Matrix init---###
        self.u = np.array([x , y , theta])
        self.u_prev = np.array([0 , 0 , 0])
        self.e = np.array([[0 , 0 , 0] , [0 ,0 , 0] , [0 , 0 , 0]])
        self.e_prev = np.array([[0 , 0 , 0] , [0 ,0 , 0] , [0 , 0 , 0]])
        self.q = np.array([[0 , 0 , 0] , [0 , 0 , 0] , [0 , 0 , 0]])

        ###--- Constant Set ---###
        self.dt = dt
        self.r = 0.05
        self.l = 0.19
        self.lw = 0.25 #KL
        self.rw = 0.25 #KR

    ###--- Calc best estimated position ---###
    def estimated_pos (self , v , w):
        th = np.copy(self.u[2])
        u_o = np.array([self.u[0] + self.dt * v * np.cos(th) , 
                     self.u[1] + self.dt * v* np.sin(th) ,
                     self.u[2] + self.dt * w])
        self.u = np.copy(u_o)
        
    ###--- Linearice model ---###
    def linearization (self , v ):
        th = np.copy(self.u_prev[2])
        print (th)
        h = np.array([[1 , 0 , -self.dt * v * np.sin(th)] , 
                      [0 , 1 , self.dt * v * np.cos(th)] , 
                      [0 , 0 , 1]])
    
        self.h = np.copy(h)

    ###--- Calculate uncertainty ---###
    def uncertainty (self):
        e_o = self.h.dot(self.e_prev).dot(self.h.T) + self.q
        self.e = np.copy(e_o)

    ###--- Order of execution ---###
    def calculate (self , v , w , wr , wl , flag ,arr):
        self.calcQ(v , w , wr , wl)
        self.estimated_pos(v , w)
        self.linearization(v)
        self.uncertainty()
        if flag:
            self.correction(arr)

        self.u_prev = np.copy(self.u)
        self.e_prev = np.copy(self.e)
        return self.e , self.u
    
    def calcQ (self , v , w , wr , wl):
        th = np.copy(self.u_prev[2])
        m = np.array([[self.rw * np.abs(wr) , 0] , 
                      [0 , self.lw * np.abs(wl)]])
        sigma = np.array([[np.cos(th) , np.cos(th)] ,
                          [np.sin(th) , np.sin(th)] ,
                          [2/self.l , -2/self.l]])
        
        calc = 0.5 * self.r * self.dt
        sigma = sigma * calc

        Q = sigma.dot(m).dot(sigma.T)

        self.q = np.copy(Q)

    def correction(self , arr):
        self.xa = arr[0]
        self.ya = arr[1]
        self.z_c = np.array([[arr[2]] , [arr[3]]])
        self.R = np.array([[0.1 , 0] , [0 , 0.1]])
        self.obs_model()
        self.unsertainty_pro()
        self.calc_u_e()
        

    def obs_model(self):
        d_x = self.u[0] - self.xa
        d_y = self.u[1] - self.ya
        p = d_x**2 + d_y**2
        #Observation Model
        self.z_ob = np.array([[np.sqrt(p)] , 
                              np.arctan2(d_y , d_x) - self.u[2]])
        #Linearization
        self.G = np.array([[-(d_x/np.sqrt(p)) , -(d_y/np.sqrt(p)) , 0] , 
                           [d_y/p , - d_x/p , -1]])
        
    def unsertainty_pro(self):
        #Uncertainty propagation
        self.Z = self.G.dot(self.e).dot(self.G.T) + self.R
        #Kalman Gain 
        self.K = self.e.dot(self.G.T).dot(np.linalog.inv(self.Z))

    def calc_u_e(self):
        self.u = self.u + self.K.dot(self.z_c - self.z_ob)
        self.e = (np.eye(3) - self.K.dot(self.G)).dot(self.e)
        
    