import numpy as np
from scipy import optimize

# function catenary
def f(c,l,x_bar):
    return (- l/2 + c * np.sinh(x_bar/(2*c)))  # only one real root at x = 1

# Rotation matrix
def Rotz(th):
    Rot = np.array([[np.cos(th), -np.sin(th), 0],
                  [np.sin(th), np.cos(th), 0],
                  [0, 0, 1]])
    return Rot

# Hat_map
def hat_map(aux):
    A = np.array([[0, -aux[2], aux[1]], [aux[2], 0, -aux[0]], [-aux[1], aux[0], 0]])
    return A

# function for compute the position of the robots
# here the minimum point is fixed and we are rotating
# and increasing and decreasing the span.

def trajectory(t):

    # minimum point fixed

    dst_xCd = np.array([1., 0., 0.4], dtype='f') ############# --> this is Xc
    dst_dxCd = np.array([0., 0., 0.], dtype='f')
    dst_ddxCd = np.array([0., 0., 0.], dtype='f')

    # rotating with the time
    dst_yaw = t/10 ############# --> this is yaw for the lowest point
    dst_Omega = np.array([0, 0, 0], dtype='f')

    dst_R = Rotz(dst_yaw)
    dst_dR = dst_R.dot(hat_map(dst_Omega))

    ## span desired
    dst_x_bar_d = np.sin(t/10)/2 + 0.3 ############# --> this is s

    dst_dx_bar_d = 0

    # solution using bisection
    sol = optimize.bisect(f, 0.01, 5, args=(1.55, dst_x_bar_d))  ############# --> this is parameters in args l and s


    
    #(f, x0=[0.15], args=(1.5, dst_x_bar_d), method='hybr', tol=1e-6) 
    #print(sol)
    dst_c = np.absolute(sol)
    dst_dc = 0

    # equation for compute the slag
    dst_zABd = dst_c*(np.cosh(dst_x_bar_d/(2*dst_c)) - 1)

    # relative positions
    xAdv = np.array([-dst_x_bar_d/2, 0, dst_zABd], dtype='f')
    xBdv = np.array([dst_x_bar_d/2, 0, dst_zABd], dtype='f')

    ## Desired positions to vectors
    dst_xAd = dst_xCd + dst_R.dot(xAdv)
    dst_xBd = dst_xCd + dst_R.dot(xBdv)

   # dst_dxAd = dst_dxCd + dst_dR.dot(xAdv) + dst_R.dot(dxAdv)
   # dst_dxBd = dst_dxCd + dst_dR.dot(xBdv) + dst_R.dot(dxBdv)

    return dst_xAd, dst_xBd, dst_yaw