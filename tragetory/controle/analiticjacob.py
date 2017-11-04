import math

def analiticjacob(theta,L):
    s2 = math.sqrt(2)
    o1 = theta[0]
    o2 = theta[1]
    o3 = theta[2]
    o4 = theta[3]
    o5 = theta[4]
    o6 = theta[5]
    
    L1 = L[0]
    L2 = L[1]
    L3 = L[2]
    L4 = L[3]
    L5 = L[4]
    
    j11 = -(s2*(math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 + math.sin(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2)/2 + math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2))/4
    j12 = (s2*(math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 - math.sin(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2)/2 - math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2))/4
    j13 = (s2*(math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 - math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 + math.sin(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2)/2 - math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2))/4
    j14 = (s2*(math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 - math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 + math.sin(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2)/2 - math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2))/4
    j15 = (s2*(math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 - math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 + math.sin(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2)/2 - math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2))/4
    j16 = -(s2*(math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 + math.sin(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2)/2 + math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2))/4

    j21 = (s2*(math.sin(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2)/2 - math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 + math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2))/4
    j22 = (s2*(math.sin(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2)/2 + math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 - math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2))/4
    j23 = (s2*(math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 - math.sin(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2)/2 + math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2))/4
    j24 = (s2*(math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 - math.sin(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2)/2 + math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2))/4
    j25 = (s2*(math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 - math.sin(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2)/2 + math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2))/4
    j26 = -(s2*(math.sin(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2)/2 - math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 + math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2))/4

    j31 = -(s2*(math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 - math.cos(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2)/2 + math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2))/4
    j32 = (s2*(math.cos(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2)/2 + math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 + math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 - math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2))/4
    j33 = -(s2*(math.cos(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2)/2 - math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 + math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2))/4
    j34 = -(s2*(math.cos(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2)/2 - math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 + math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2))/4
    j35 = -(s2*(math.cos(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2)/2 - math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 + math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2))/4
    j36 = (s2*(math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 - math.cos(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2)/2 + math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2)/2))/4

    j41 = -(s2*(math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 - math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.cos(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2)/2 + math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2))/4
    j42 = -(s2*(math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 - math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 + math.cos(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2)/2 + math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2))/4
    j43 = -(s2*(math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 - math.cos(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2)/2 + math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2))/4
    j44 = -(s2*(math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 - math.cos(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2)/2 + math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2))/4
    j45 = -(s2*(math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 - math.cos(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2)/2 + math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2))/4
    j46 = -(s2*(math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2 - math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2)/2 + math.cos(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2)/2 + math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2)/2))/4

    j51 = (L3*s2*math.sin(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 + (L3*s2*math.sin(o1/2 - o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L3*s2*math.sin(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 + (L3*s2*math.sin(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 + (L4*s2*math.sin(o1/2 - o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 + (L5*s2*math.sin(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2))/16 - (L5*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 + (L5*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16
    j52 = (L3*s2*math.sin(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L3*s2*math.sin(o1/2 - o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L3*s2*math.sin(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L3*s2*math.sin(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L4*s2*math.sin(o1/2 - o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 - (L4*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 + (L5*s2*math.sin(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 - (L5*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 + (L5*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16
    j53 = (L3*s2*math.sin(o1/2 - o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L3*s2*math.sin(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 + (L3*s2*math.sin(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 + (L3*s2*math.sin(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.sin(o1/2 - o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 + (L4*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L5*s2*math.sin(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 + (L5*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16
    j54 = (L3*s2*math.sin(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L3*s2*math.sin(o1/2 - o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L3*s2*math.sin(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L3*s2*math.sin(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.sin(o1/2 - o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 + (L4*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L5*s2*math.sin(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 + (L5*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16
    j55 = (L3*s2*math.sin(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L3*s2*math.sin(o1/2 - o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L3*s2*math.sin(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L3*s2*math.sin(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 + (L4*s2*math.sin(o1/2 - o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 - (L4*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 - (L4*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L5*s2*math.sin(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 + (L5*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16
    j56 = (L3*s2*math.sin(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 + (L3*s2*math.sin(o1/2 - o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L3*s2*math.sin(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 + (L3*s2*math.sin(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 + (L4*s2*math.sin(o1/2 - o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L5*s2*math.sin(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 - (L5*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 - (L5*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16

    j61 = (L3*s2*math.sin(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 - (L3*s2*math.sin(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 - (L3*s2*math.sin(o2/2 - o1/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 + (L3*s2*math.sin(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 - o6/2))/16 + (L4*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 + (L4*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 + (L4*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 - o6/2))/16 + (L4*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 + (L5*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 + (L5*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2))/16 + (L5*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16
    j62 = (L3*s2*math.sin(o2/2 - o1/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 + (L3*s2*math.sin(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 + (L3*s2*math.sin(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 + (L3*s2*math.sin(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 - o6/2))/16 - (L4*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 + (L4*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 - o6/2))/16 + (L4*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 - (L5*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 - (L5*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2))/16 + (L5*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16
    j63 = (L3*s2*math.sin(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 - (L3*s2*math.sin(o2/2 - o1/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L3*s2*math.sin(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 + (L3*s2*math.sin(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 - o6/2))/16 - (L4*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 + (L4*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 - (L4*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 - o6/2))/16 + (L4*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 - (L5*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 + (L5*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 - (L5*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2))/16 + (L5*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16
    j64 = (L3*s2*math.sin(o2/2 - o1/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L3*s2*math.sin(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 + (L3*s2*math.sin(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 - (L3*s2*math.sin(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 - o6/2))/16 - (L4*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 + (L4*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 - (L4*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 - o6/2))/16 + (L4*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 - (L5*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 + (L5*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 - (L5*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2))/16 + (L5*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16
    j65 = (L3*s2*math.sin(o2/2 - o1/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L3*s2*math.sin(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 + (L3*s2*math.sin(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 - (L3*s2*math.sin(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 - o6/2))/16 + (L4*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 + (L4*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 - o6/2))/16 - (L4*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 - (L5*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 + (L5*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 - (L5*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2))/16 + (L5*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16
    j66 = (L3*s2*math.sin(o2/2 - o1/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 + (L3*s2*math.sin(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 - (L3*s2*math.sin(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 - (L3*s2*math.sin(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 - o6/2))/16 - (L4*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 - (L4*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 - o6/2))/16 - (L4*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 + (L5*s2*math.sin(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 + (L5*s2*math.sin(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.sin(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2))/16 + (L5*s2*math.sin(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16

    j71 = (L3*s2*math.cos(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 - (L3*s2*math.cos(o2/2 - o1/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L3*s2*math.cos(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 - (L3*s2*math.cos(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 - o6/2))/16 + (L4*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 - (L4*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 - o6/2))/16 - (L4*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 + (L5*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 - (L5*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 - (L5*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2))/16 - (L5*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16
    j72 = (L3*s2*math.cos(o2/2 - o1/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L3*s2*math.cos(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 - (L3*s2*math.cos(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 - (L3*s2*math.cos(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 - o6/2))/16 - (L4*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 + (L4*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 - (L4*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 - o6/2))/16 - (L4*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 - (L5*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 + (L5*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 - (L5*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2))/16 - (L5*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16
    j73 = (L3*s2*math.cos(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 - (L3*s2*math.cos(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 - (L3*s2*math.cos(o2/2 - o1/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L3*s2*math.cos(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 - o6/2))/16 - (L4*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 + (L4*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 - o6/2))/16 - (L4*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 - (L5*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 - (L5*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2))/16 - (L5*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16
    j74 = (L3*s2*math.cos(o2/2 - o1/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 + (L3*s2*math.cos(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 - (L3*s2*math.cos(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 + (L3*s2*math.cos(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 - o6/2))/16 - (L4*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 + (L4*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 - o6/2))/16 - (L4*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 - (L5*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 - (L5*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2))/16 - (L5*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16
    j75 = (L3*s2*math.cos(o2/2 - o1/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 + (L3*s2*math.cos(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 - (L3*s2*math.cos(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 + (L3*s2*math.cos(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 - o6/2))/16 + (L4*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 + (L4*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 - (L4*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 - o6/2))/16 + (L4*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 - (L5*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 - (L5*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2))/16 - (L5*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16
    j76 = (L3*s2*math.cos(o2/2 - o1/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L3*s2*math.cos(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 + (L3*s2*math.cos(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 - o6/2))/16 + (L3*s2*math.cos(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 - o6/2))/16 - (L4*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 + (L4*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 + (L4*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 - o6/2))/16 + (L4*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 - o6/2))/16 + (L5*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 - (L5*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 - (L5*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 - o5/2 + o6/2))/16 - (L5*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 + o6/2))/16

    j81 = (L3*s2*math.cos(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 + (L3*s2*math.cos(o1/2 - o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L3*s2*math.cos(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 + (L3*s2*math.cos(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 + (L4*s2*math.cos(o1/2 - o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L5*s2*math.cos(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 + (L5*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16
    j82 = (L3*s2*math.cos(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L3*s2*math.cos(o1/2 - o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L3*s2*math.cos(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L3*s2*math.cos(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L4*s2*math.cos(o1/2 - o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 - (L4*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L5*s2*math.cos(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2))/16 - (L5*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 - (L5*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 + (L5*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16
    j83 = (L3*s2*math.cos(o1/2 - o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L3*s2*math.cos(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 + (L3*s2*math.cos(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 + (L3*s2*math.cos(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.cos(o1/2 - o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 + (L4*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 + (L5*s2*math.cos(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2))/16 - (L5*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 + (L5*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16
    j84 = (L3*s2*math.cos(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L3*s2*math.cos(o1/2 - o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L3*s2*math.cos(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L3*s2*math.cos(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.cos(o1/2 - o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 + (L4*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 + (L5*s2*math.cos(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2))/16 - (L5*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 + (L5*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16
    j85 = (L3*s2*math.cos(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L3*s2*math.cos(o1/2 - o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L3*s2*math.cos(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 - (L3*s2*math.cos(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 + (L4*s2*math.cos(o1/2 - o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 - (L4*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 - (L4*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 + (L5*s2*math.cos(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2))/16 - (L5*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 + (L5*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 + (L5*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16
    j86 = (L3*s2*math.cos(o1/2 - o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 + (L3*s2*math.cos(o1/2 - o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 - (L3*s2*math.cos(o1/2 + o2/2 - o3/2 + o4/2 + o5/2 + o6/2))/16 + (L3*s2*math.cos(o1/2 + o2/2 + o3/2 - o4/2 - o5/2 + o6/2))/16 + (L4*s2*math.cos(o1/2 - o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 - (L4*s2*math.cos(o1/2 + o2/2 - o3/2 - o4/2 + o5/2 + o6/2))/16 + (L4*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 - o5/2 + o6/2))/16 + (L5*s2*math.cos(o3/2 - o2/2 - o1/2 + o4/2 + o5/2 + o6/2))/16 - (L5*s2*math.cos(o2/2 - o1/2 + o3/2 + o4/2 + o5/2 + o6/2))/16 - (L5*s2*math.cos(o1/2 - o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16 - (L5*s2*math.cos(o1/2 + o2/2 + o3/2 + o4/2 + o5/2 - o6/2))/16

    J=[[j11,j12,j13,j14,j15,j16],[j21,j22,j23,j24,j25,j26],[j31,j32,j33,j34,j35,j36],[j41,j42,j43,j44,j45,j46],[j51,j52,j53,j54,j55,j56],[j61,j62,j63,j64,j65,j66],[j71,j72,j73,j74,j75,j76],[j81,j82,j83,j84,j85,j86]]
    return J
