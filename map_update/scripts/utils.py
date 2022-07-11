import numpy as np
import math

def undistort_point(coords, dist_coeffs, fx, fy, width, height):

    coords[0] *= width
    coords[1] *= height

    c = np.array([width / 2, height / 2])
    f = [2*fx, 2*fy]

    coords = (coords - c) / f
    r = (coords[0] * coords[0] + coords[1] * coords[1])

    dp = np.array([
        2 * dist_coeffs[2] * coords[0] * coords[1] + dist_coeffs[3] * (r + 2 * (coords[0] * coords[0])),
        dist_coeffs[2] * (r + 2 * (coords[1] * coords[1])) +  2 * dist_coeffs[3] * coords[0] * coords[1]])
    pd = (1 + dist_coeffs[0] * r + dist_coeffs[1] * r * r + dist_coeffs[4] * r * r * r) * coords + dp

    xp, yp = pd * f + c

    return [xp/width, yp/height]

def normalize(a):
    mod = math.sqrt((a[0]*a[0]) + (a[1]*a[1]) + (a[2]*a[2]))
    a[0] /= mod
    a[1] /= mod
    a[2] /= mod

    return a

def create_transformation_matrix(pos, look):

    mat = [ [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0] ]
    mat[0][3] = pos[0]
    mat[1][3] = pos[1]
    mat[2][3] = pos[2]
    mat[3][3] = 1

    upVec = normalize([(look[0] - pos[0]), (look[1] - pos[1]), ((look[0] - pos[0])**2 + (look[1] - pos[1])**2)/(pos[2] - look[2])])

    direc = normalize([look[0] - pos[0], look[1] - pos[1], look[2] - pos[2]])

    right = normalize(np.cross(direc, upVec).tolist())

    newUp = np.cross(right, direc).tolist()

    mat[0][0] = right[0]
    mat[1][0] = right[1]
    mat[2][0] = right[2]
    mat[3][0] = 0
    
    mat[0][1] = newUp[0]
    mat[1][1] = newUp[1]
    mat[2][1] = newUp[2]
    mat[3][1] = 0

    mat[0][2] = direc[0]
    mat[1][2] = direc[1]
    mat[2][2] = direc[2]
    mat[3][2] = 0

    return mat

def transform_point(coords, pos, look, fx, fy, width, height, dist_coeffs):

    # coords = undistort_point(coords, dist_coeffs, fx, fy, width, height)

    coords = [2 * (coords[0]) - 1, 
              1 - 2 * (coords[1])]

    fx = (fx * 2)/width
    fy = (fy * 2)/height

    Caz = 3
    Cax = (Caz * coords[0]) / fx
    Cay = (Caz * coords[1]) / fy

    mat = create_transformation_matrix(pos, look)

    Rx, Ry, Rz = np.matmul(mat, [Cax, Cay, Caz, 1])[:3]

    return [Rx, Ry, Rz]

def loss_function(lines, p):
    L = 0
    for li in lines:
        L += li.perp_dist(p)
    return L

def refine(lines, eps=0.000001, lr=0.05):
    p_init = [0, 0, 0]
    prev_loss = float('inf')
    c = 0
    num_iter=0
    while True:
        L = loss_function(lines, p_init)

        L2 = loss_function(lines, [p_init[0]+eps, p_init[1], p_init[2]])
        dLx = (L2 - L) / eps

        L2 = loss_function(lines, [p_init[0], p_init[1]+eps, p_init[2]])
        dLy = (L2 - L) / eps

        L2 = loss_function(lines, [p_init[0], p_init[1], p_init[2]+eps])
        dLz = (L2 - L) / eps
        
        p_init = [p_init[0] - lr*dLx, p_init[1] - lr*dLy, p_init[2] - lr*dLz]

        if prev_loss < L:
            lr *= 0.1
            c += 1
        if c == 3 or num_iter == 15000:
            break
        prev_loss = L
        num_iter+=1

    return p_init

class line3d:
    def __init__(self, a, b):
        """
        a : a point on line
        b : a point on line
        """
        self.x1, self.y1, self.z1 = b
        self.l, self.m, self.n = a[0]-b[0], a[1]-b[1], a[2]-b[2]
    
    def perp_dist(self, p):
        """
        p : list of 3 integers
        """
        a, b, c = self.x1 - p[0], self.y1 - p[1], self.z1 - p[2]
        l, m, n = self.l, self.m, self.n
        
        temp = list(np.cross([a,b,c],[l,m,n]))

        temp = math.sqrt((temp[0] * temp[0]) + (temp[1] * temp[1]) + (temp[2] * temp[2]))
        temp = temp / (math.sqrt((l*l)+(m*m)+(n*n)))
        return abs(temp)