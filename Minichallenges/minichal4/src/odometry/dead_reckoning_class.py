import numpy as np
np.set_printoptions(suppress = True) 
np.set_printoptions(formatter = {'float': '{: 0.4f}'.format})

class dead_reckoning:
    def __init__(self , dt):
        ###--- Matrix init---###
        self.u = np.array([0 , 0 , 0])
        self.u_prev = np.array([0 , 0 , 0])
        self.e = np.array([[0 , 0 , 0] , [0 ,0 , 0] , [0 , 0 , 0]])
        self.e_prev = np.array([[0 , 0 , 0] , [0 ,0 , 0] , [0 , 0 , 0]])
        self.q = np.array([[0 , 0 , 0] , [0 , 0 , 0] , [0 , 0 , 0]])

        ###--- Constant Set ---###
        self.dt = dt
        self.r = 0.05
        self.l = 0.19
        self.lw = 1
        self.rw = 1

    ###--- Calc best estimated position ---###
    def estimated_pos (self , v , w):
        th = np.copy(self.u[2])
        u = np.array([self.u[0] + self.dt * v * np.cos(th) , 
                     self.u[1] + self.dt * v* np.sin(th) ,
                     self.u[2] + self.dt * w])
        self.u = np.copy(u)
        
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
        e = self.h.dot(self.e_prev).dot(self.h.T) + self.q
        self.e = np.copy(e)

    ###--- Order of execution ---###
    def calculate (self , v , w , wr , wl):
        self.calcQ(v , w , wr , wl)
        self.estimated_pos(v , w)
        self.linearization(v)
        self.uncertainty()
        self.u_prev = np.copy(self.u)
        self.e_prev = np.copy(self.e)

        return self.e
    
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
        
    