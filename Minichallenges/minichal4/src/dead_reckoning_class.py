import numpy as np

class dead_reckoning:
    def __init__(self , dt):
        np.set_printoptions(suppress = True) 
        np.set_printoptions(formatter = {'float': '{: 0.4f}'.format})
        self.u = np.array([0 , 0 , 0])
        self.u_prev = np.array([0 , 0 , 0])
        self.e = np.array([[0 , 0 , 0] , [0 ,0 , 0] , [0 , 0 , 0]])
        self.e_prev = np.array([[0 , 0 , 0] , [0 ,0 , 0] , [0 , 0 , 0]])
        self.q = np.array([[0.0002 , 0.0001 , 0.0001] , [0.0001 , 0.0005 , 0.0001] , [0.0001 , 0.0001 , 0.0002]])
        self.dt = dt

    def estimated_pos (self , v , w):
        th = np.copy(self.u[2])
        u = np.array([self.u[0] + self.dt * v * np.cos(th) , 
                     self.u[1] + self.dt * v* np.sin(th) ,
                     self.u[2] + self.dt * w])
        self.u = np.copy(u)
        
    def linearization (self , v ):
        th = np.copy(self.u_prev[2])
        print (th)
        h = np.array([[1 , 0 , -self.dt * v * np.sin(th)] , 
                      [0 , 1 , self.dt * v * np.cos(th)] , 
                      [0 , 0 , 1]])
    
        self.h = np.copy(h)

    def uncertainty (self):
        e = self.h.dot(self.e_prev).dot(self.h.T) + self.q
        self.e = np.copy(e)

    def calculate (self , v , w):
        self.estimated_pos(v , w)
        self.linearization(v)
        self.uncertainty()
        self.u_prev = np.copy(self.u)
        self.e_prev = np.copy(self.e)

        return self.e
    
    def calcG
    