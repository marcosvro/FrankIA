def quatConj(q):
	q[0] =  q[0]
	q[1] = -q[1]
	q[2] = -q[2]
	q[3] = -q[3]
	return q

def dualQuatConj(q):
    q1 = [q[0],q[1],q[2],q[3]]
    q2 = [q[4],q[5],q[6],q[7]]
    qp =  quatConj(q1)
    qd =  quatConj(q2)

    qr = [qp[0],qp[1],qp[2],qp[3],qd[0],qd[1],qd[2],qd[3]]
    return qr
