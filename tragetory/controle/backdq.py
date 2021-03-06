import math

def backdq(theta,L):
    
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
    

    q = [0]*8
    q[0] = (s2*(math.cos((o2)/2 - o3/2 - o5/2 - o6/2 - o1/2 - (o4)/2) - math.cos(o3/2 - o1/2 + o5/2 - o6/2 + (o2)/2 + (o4)/2) + math.cos(o1/2 - o3/2 - o5/2 + o6/2 + (o2)/2 - (o4)/2) + math.cos(o1/2 + o3/2 + o5/2 + o6/2 + (o2)/2 + (o4)/2)))/4
    q[1] = -(s2*(math.cos(o6/2 - o3/2 - o5/2 - o1/2 + (o2)/2 - (o4)/2) + math.cos(o3/2 - o1/2 + o5/2 + o6/2 + (o2)/2 + (o4)/2) - math.cos(o1/2 - o3/2 - o5/2 - o6/2 + (o2)/2 - (o4)/2) + math.cos(o1/2 + o3/2 + o5/2 - o6/2 + (o2)/2 + (o4)/2)))/4
    q[2] = (s2*(math.sin(o6/2 - o3/2 - o5/2 - o1/2 + (o2)/2 - (o4)/2) + math.sin(o3/2 - o1/2 + o5/2 + o6/2 + (o2)/2 + (o4)/2) + math.sin(o1/2 - o3/2 - o5/2 - o6/2 + (o2)/2 - (o4)/2) - math.sin(o1/2 + o3/2 + o5/2 - o6/2 + (o2)/2 + (o4)/2)))/4
    q[3] = -(s2*(math.sin(o3/2 - o1/2 + o5/2 - o6/2 + (o2)/2 + (o4)/2) - math.sin((o2)/2 - o3/2 - o5/2 - o6/2 - o1/2 - (o4)/2) + math.sin(o1/2 - o3/2 - o5/2 + o6/2 + (o2)/2 - (o4)/2) + math.sin(o1/2 + o3/2 + o5/2 + o6/2 + (o2)/2 + (o4)/2)))/4
    q[4] = (s2*L4*math.cos(o1/2 - o3/2 + o5/2 + o6/2 + (o2)/2 - (o4)/2))/8 - (s2*L4*math.cos(o3/2 - o1/2 - o5/2 - o6/2 + (o2)/2 + (o4)/2))/8 - (s2*L4*math.cos(o5/2 - o3/2 - o1/2 - o6/2 + (o2)/2 - (o4)/2))/8 - (s2*L4*math.cos(o1/2 + o3/2 - o5/2 + o6/2 + (o2)/2 + (o4)/2))/8 - (s2*math.cos(o5/2 - o3/2 - o1/2 - o6/2 + (o2)/2 + (o4)/2)*(L3))/8 - (s2*math.cos(o3/2 - o1/2 - o5/2 - o6/2 + (o2)/2 - (o4)/2)*(L3))/8 + (s2*math.cos(o1/2 - o3/2 + o5/2 + o6/2 + (o2)/2 + (o4)/2)*(L3))/8 - (s2*math.cos(o1/2 + o3/2 - o5/2 + o6/2 + (o2)/2 - (o4)/2)*(L3))/8 - (s2*math.cos(o6/2 - o3/2 - o5/2 - o1/2 + (o2)/2 - (o4)/2)*(L5))/8 - (s2*math.cos(o3/2 - o1/2 + o5/2 + o6/2 + (o2)/2 + (o4)/2)*(L5))/8 + (s2*math.cos(o1/2 - o3/2 - o5/2 - o6/2 + (o2)/2 - (o4)/2)*(L5))/8 - (s2*math.cos(o1/2 + o3/2 + o5/2 - o6/2 + (o2)/2 + (o4)/2)*(L5))/8
    q[5] = (s2*L4*math.cos(o3/2 - o1/2 - o5/2 + o6/2 + (o2)/2 + (o4)/2))/8 - (s2*L4*math.cos(o5/2 - o3/2 - o1/2 + o6/2 + (o2)/2 - (o4)/2))/8 - (s2*L4*math.cos(o1/2 - o3/2 + o5/2 - o6/2 + (o2)/2 - (o4)/2))/8 - (s2*L4*math.cos(o1/2 + o3/2 - o5/2 - o6/2 + (o2)/2 + (o4)/2))/8 - (s2*math.cos(o5/2 - o3/2 - o1/2 + o6/2 + (o2)/2 + (o4)/2)*(L3))/8 + (s2*math.cos(o3/2 - o1/2 - o5/2 + o6/2 + (o2)/2 - (o4)/2)*(L3))/8 - (s2*math.cos(o1/2 - o3/2 + o5/2 - o6/2 + (o2)/2 + (o4)/2)*(L3))/8 - (s2*math.cos(o1/2 + o3/2 - o5/2 - o6/2 + (o2)/2 - (o4)/2)*(L3))/8 - (s2*math.cos((o2)/2 - o3/2 - o5/2 - o6/2 - o1/2 - (o4)/2)*(L5))/8 + (s2*math.cos(o3/2 - o1/2 + o5/2 - o6/2 + (o2)/2 + (o4)/2)*(L5))/8 - (s2*math.cos(o1/2 - o3/2 - o5/2 + o6/2 + (o2)/2 - (o4)/2)*(L5))/8 - (s2*math.cos(o1/2 + o3/2 + o5/2 + o6/2 + (o2)/2 + (o4)/2)*(L5))/8
    q[6] = (s2*L4*math.sin(o5/2 - o3/2 - o1/2 + o6/2 + (o2)/2 - (o4)/2))/8 - (s2*L4*math.sin(o3/2 - o1/2 - o5/2 + o6/2 + (o2)/2 + (o4)/2))/8 - (s2*L4*math.sin(o1/2 - o3/2 + o5/2 - o6/2 + (o2)/2 - (o4)/2))/8 - (s2*L4*math.sin(o1/2 + o3/2 - o5/2 - o6/2 + (o2)/2 + (o4)/2))/8 + (s2*math.sin(o5/2 - o3/2 - o1/2 + o6/2 + (o2)/2 + (o4)/2)*(L3))/8 - (s2*math.sin(o3/2 - o1/2 - o5/2 + o6/2 + (o2)/2 - (o4)/2)*(L3))/8 - (s2*math.sin(o1/2 - o3/2 + o5/2 - o6/2 + (o2)/2 + (o4)/2)*(L3))/8 - (s2*math.sin(o1/2 + o3/2 - o5/2 - o6/2 + (o2)/2 - (o4)/2)*(L3))/8 + (s2*math.sin((o2)/2 - o3/2 - o5/2 - o6/2 - o1/2 - (o4)/2)*(L5))/8 - (s2*math.sin(o3/2 - o1/2 + o5/2 - o6/2 + (o2)/2 + (o4)/2)*(L5))/8 - (s2*math.sin(o1/2 - o3/2 - o5/2 + o6/2 + (o2)/2 - (o4)/2)*(L5))/8 - (s2*math.sin(o1/2 + o3/2 + o5/2 + o6/2 + (o2)/2 + (o4)/2)*(L5))/8
    q[7] = (s2*L4*math.sin(o1/2 + o3/2 - o5/2 + o6/2 + (o2)/2 + (o4)/2))/8 - (s2*L4*math.sin(o3/2 - o1/2 - o5/2 - o6/2 + (o2)/2 + (o4)/2))/8 - (s2*L4*math.sin(o1/2 - o3/2 + o5/2 + o6/2 + (o2)/2 - (o4)/2))/8 - (s2*L4*math.sin(o5/2 - o3/2 - o1/2 - o6/2 + (o2)/2 - (o4)/2))/8 - (s2*math.sin(o5/2 - o3/2 - o1/2 - o6/2 + (o2)/2 + (o4)/2)*(L3))/8 - (s2*math.sin(o3/2 - o1/2 - o5/2 - o6/2 + (o2)/2 - (o4)/2)*(L3))/8 - (s2*math.sin(o1/2 - o3/2 + o5/2 + o6/2 + (o2)/2 + (o4)/2)*(L3))/8 + (s2*math.sin(o1/2 + o3/2 - o5/2 + o6/2 + (o2)/2 - (o4)/2)*(L3))/8 - (s2*math.sin(o6/2 - o3/2 - o5/2 - o1/2 + (o2)/2 - (o4)/2)*(L5))/8 - (s2*math.sin(o3/2 - o1/2 + o5/2 + o6/2 + (o2)/2 + (o4)/2)*(L5))/8 - (s2*math.sin(o1/2 - o3/2 - o5/2 - o6/2 + (o2)/2 - (o4)/2)*(L5))/8 + (s2*math.sin(o1/2 + o3/2 + o5/2 - o6/2 + (o2)/2 + (o4)/2)*(L5))/8
    
    
    return q
    
