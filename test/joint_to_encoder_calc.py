import numpy as np
import math
from matplotlib import pyplot as plt
from matplotlib.figure import Figure

# solve for a and b
def best_fit(X, Y):

    coef = np.polyfit(X,Y,1)
    a = coef[0]
    b = coef[1]
    poly1d_fn = np.poly1d(coef) 
 
    #plt.plot(X,Y, 'yo', Y, poly1d_fn(Y), '--k') #'--k'=black dashed line, 'yo' = yellow circle marker
    plt.xlabel("Joint_space")
    plt.ylabel("Actuator_space")

    return a,b,poly1d_fn

def error(a,b,c,x,y):
    return abs((a * x + b * y + c)) / (math.sqrt(a * a + b * b))
class transmission_calc:
    def __init__(self,r,l,b,j_offset,mr,ppr,a_offset):
        self.r = r
        self.l = l
        self.b = b
        self.j_offset =j_offset
        self.mr = mr
        self.ppr = ppr
        self.a_offset = a_offset
    def debug_params(self,j_pos):
        print("*) r^2=" , pow(self.r,2) )
        print("**) b^2=" , pow(self.b,2) )
        print("***) -2rbcos(offset+pos) = " , -2*self.r*self.b*np.cos(self.j_offset+j_pos))
        print("****) r^2 + b^2 -2rbcos(offset+pos) = " ,(pow(self.r,2)+pow(self.b,2))-2*self.r*self.b*np.cos(self.j_offset+j_pos))
    def j2a(self,j_pos,debug):
        if debug is True:
            self.debug_params(j_pos=j_pos)
        print("1) (r^2 + b^2 -2rbcos(offset+pos))/l = " ,((pow(self.r,2)+pow(self.b,2))-2*self.r*self.b*np.cos(self.j_offset+j_pos)/self.l))
        print("2) mr*ppr/2pi = ", ((self.mr*self.ppr)/(2*np.pi)) )
        print("1 -> 2: ",((pow(self.r,2)+pow(self.b,2)-2*self.r*self.b*np.cos(self.j_offset+j_pos))/self.l)*((self.mr*self.ppr)/(2*np.pi)) )
        print("1 -> 2: -a_offset",((pow(self.r,2)+pow(self.b,2)-2*self.r*self.b*np.cos(self.j_offset+j_pos))/self.l)*((self.mr*self.ppr)/(2*np.pi)) - self.a_offset )
        return ((pow(self.r,2)+pow(self.b,2)-2*self.r*self.b*np.cos(self.j_offset+j_pos))/self.l)*((self.mr*self.ppr)/(2*np.pi)) - self.a_offset
    def alt_j2a(self,j_pos,debug):
        if debug is True:
            self.debug_params(j_pos=j_pos)
        if(debug):
            print("1) r^2+b^2-2rbcos(j_pos+j_offset) - l = ",(pow(pow(self.r,2)+pow(self.b,2)-2*self.r*self.b*np.cos(self.j_offset+j_pos),0.5)-self.l))
        mr_ = (self.mr*pow(10,-3))
        if(debug):
            print("2) mr: (motor_spins/1[m])*(Encoder_ticks/motor_spin)= ",(1/mr_)*(self.ppr))
        ppr_ = ((1/mr_)*(self.ppr))
        if(debug):
            print("3) = 1) *  2) = ",((pow(pow(self.r,2)+pow(self.b,2)-2*self.r*self.b*np.cos(self.j_offset+j_pos),0.5)-self.l)*ppr_))
        if(j_pos == 0.0):
            print("OFFSET CALCULATION",(pow(pow(self.r,2)+pow(self.b,2)-2*self.r*self.b*np.cos(self.j_offset+j_pos),0.5)-self.l))
        if(debug):
            print("after offset, before mr: ",(pow(pow(self.r,2)+pow(self.b,2)-2*self.r*self.b*np.cos(self.j_offset+j_pos),0.5)-self.l-self.a_offset))
        return np.floor((pow(pow(self.r,2)+pow(self.b,2)-2*self.r*self.b*np.cos(self.j_offset+j_pos),0.5)-self.l-self.a_offset)*ppr_ )
    def alt_a2j(self,a_pos):
        mr_ = ((1/(self.mr*pow(10,-3)))*(self.ppr))
        I=((a_pos)/mr_)
        print("I) (A_pos)/mr= ",I)
        I_ = pow((a_pos+(self.l+self.a_offset)*mr_)/mr_,2)
        print("I_) ((A_pos+(A_offset+l)*mr)/mr)^2 = ",I_)
        II = -pow(I_,2) + pow(self.r,2) + pow(self.b,2)
        print("II) 2rbcos(j_pos+j_offset) = -((A_pos+(A_offset+l)*mr)/mr)^2+r^2+b^2 = ",II)
        II_ = (2*self.r*self.b)
        print("II_ = 2*self.r*self.b",II_)
        III = np.arccos(II/II_)
        print("j_pos+j_offset = arccos((r^2+b^2-((a_pos+(a_offset+l)*mr)/mr)^2)/2rb)= ",III)
        IV = III-self.j_offset
        return IV

    
def main():

    tr = transmission_calc(r=0.12554,l=0.1557,b=0.169,j_offset=1.2159709,mr=3.864,ppr=4000,a_offset=0.01628576217921973)
    # print("\n Original- 0.137881",tr.j2a(0.137881,False))
    # print("\n Original- 0.0",tr.j2a(0.0,False))
    # print("\n Original- -0.261",tr.j2a(-0.261,False))
    print("Inverse Kinematics \n")
    print("\n Alt j2a- 0.137881",tr.alt_j2a(0.137881,False))
    print("\n Alt j2a- 0.0",tr.alt_j2a(0.0,False))
    print("\n Alt j2a- -0.261",tr.alt_j2a(-0.261,False))
    x_ = np.linspace(-0.261,0.137,num=100,endpoint=True)
    y_ = np.zeros(len(x_))
    for i in range(len(x_)):
        y_[i] = tr.alt_j2a(x_[i],False)
    plt.plot(x_,y_,'-r')


    a, b,poly1d_fn = best_fit(x_,y_)
    plt.plot( x_, poly1d_fn(x_), '--k') #'--k'=black dashed line, 'yo' = yellow circle marker
    # yfit = [a + b * xi for xi in x_]

    # plt.plot(x_, yfit,color = 'green')
    err_ = np.zeros(len(x_))
    for i in range(len(x_)):
        err_[i] = error(a,-1,b,x_[i],y_[i])
    print("max error: " ,max(err_)) 
    print("mean error: " ,np.mean(err_)) 
    plt.show()
    # print("Forward Kinematics \n")
    # print("\n alt a2j- 16148.0",tr.alt_a2j(16148.0))
    # print("\n alt a2j- 0.0",tr.alt_a2j(0.0))
    # print("\n alt a2j- -32333.0",tr.alt_a2j(-32333.0))

if __name__ == '__main__':
    main()