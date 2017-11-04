import numpy as np

def HamiltonOp(h,op):
    h1 = h[0]
    h2 = h[1]
    h3 = h[2]
    h4 = h[3]

    H_ = [[h1,-h2,-h3,-h4],[h2,h1,h4,-h3],[h3,-h4,h1,h2],[h4,h3,-h2,h1]]   
    H = [[h1,-h2,-h3,-h4],[h2,h1,-h4,h3],[h3,h4,h1,-h2],[h4,-h3,h2,h1]]
    
    if op == 1:
        return  H
    else:
        return H_

def dualHamiltonOp(h,op):
    h1 = h[0]
    h2 = h[1]
    h3 = h[2]
    h4 = h[3]
    h5 = h[4]
    h6 = h[5]
    h7 = h[6]
    h8 = h[7]

    q1 = [h1,h2,h3,h4]
    q2 = [h5,h6,h7,h8]
    Hp = HamiltonOp(q1,op)
    Hd = HamiltonOp(q2,op)
    
    T = np.zeros((8,8))
    T[0:4, 0:4] = Hp
    T[0:4, 4:8] = [[0]*4 for i in range(4)]
    T[4:8, 0:4] = Hd
    T[4:8, 4:8] = Hp
    return T
    
