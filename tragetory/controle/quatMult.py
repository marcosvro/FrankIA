def quatAdd(q1, q2):
    qr = [0]*4
    qr[0] = q1[0] + q2[0]
    qr[1] = q1[1] + q2[1]
    qr[2] = q1[2] + q2[2]
    qr[3] = q1[3] + q2[3]
    return qr

def quatMult(q1,q2):
    qr = [0]*4
    qr[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
    qr[1] = q1[0]*q2[1] + q2[0]*q1[1] + q1[2]*q2[3] - q1[3]*q2[2]
    qr[2] = q1[0]*q2[2] + q2[0]*q1[2] + q1[3]*q2[1] - q1[1]*q2[3]
    qr[3] = q1[0]*q2[3] + q2[0]*q1[3] + q1[1]*q2[2] - q1[2]*q2[1]
    return qr

def dualQuatMult(p,q):    
     pp = [p[0], p[1], p[2], p[3]]
     pd = [p[4], p[5], p[6], p[7]]

     qp = [q[0], q[1], q[2], q[3]]
     qd = [q[4], q[5], q[6], q[7]]

     aux1 = quatMult(pp,qp)
     aux2 = quatAdd(quatMult(pp,qd),quatMult(pd,qp))
     r = [aux1[0],aux1[1],aux1[2],aux1[3],aux2[0],aux2[1],aux2[2],aux2[3]]
     return r
