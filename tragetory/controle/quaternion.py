import math
import numpy as np

def rot2quat(T):
    #iniciar o quartenio
    q = []

    #Elementos da matriz de Rotação
    D1 = T[0,0]
    D2 = T[1,1]
    D3 = T[2,2]

    Au = T[0,1]
    Bu = T[0,2]
    Cu = T[1,2]

    Ad = T[1,0]
    Bd = T[2,0]
    Cd = T[2,1]

    #solução baseada em w
    bw = 1 + D1 + D2 + D3
    if bw > 0:
        w = math.sqrt(bw)/2.
        x = (Cd - Cu)/(4.*w)
        y = (Bu - Bd)/(4.*w)
        z = (Ad - Au)/(4.*w)

        q = [x, y, z, w]
        return q

    #solução baseada em x
    if D1 > D2 and D1 > D3:
        bx = 1. + D1 - D2 -D3
        x = math.sqrt(bx)/2.
        y = (Au+Ad)/(4.*x)
        z = (Bu+Bd)/(4.*x)
        w = (Cd - Cu)/(4.*x)

        q = [x,y,z,w]
        return q

    #solução baseada em y
    if D2 > D1 and D2 > D3:
        by = 1. - D1 + D2 -D3
        y = math.sqrt(by)/2.
        x = (Au+Ad)/(4.*y)
        z = (Cd+Cu)/(4.*y)
        w = (Bu - Bd)/(4.*y)

        q = [x,y,z,w]
        return q

    #solução baseada em z
    if D3 > D1 and D3 > D2:
        bz = 1. - D1 - D2 +D3;
        z = math.sqrt(bz)/2.
        x = (Bu+Bd)/(4.*z)
        y = (Cd+Cu)/(4.*z)
        w = (Ad - Au)/(4.*z)

        q = [x,y,z,w]
        return q

    return q


def rad2quat(v):
    Tx = [[1,0,0,0],[0,math.cos(v[0]),-math.sin(v[0]),0],[0,math.sin(v[0]),math.cos(v[0]),0],[0,0,0,1]]
    Ty = [[math.cos(v[1]),0,math.sin(v[1]),0],[0,1,0,0],[-math.sin(v[1]),0,math.cos(v[1]),0],[0,0,0,1]]
    Tz = [[math.cos(v[2]),-math.sin(v[2]),0,0],[math.sin(v[0]),math.cos(v[2]),0,0],[0,0,1,0],[0,0,0,1]]

    Rot = np.dot(np.dot(Tx, Ty),Tz)
    r = np.array(rot2quat(Rot))

    norma = math.sqrt(r[0]**2 + r[1]**2 + r[2]**2 + r[3]**2)
    r = r/norma
    return r.tolist()
