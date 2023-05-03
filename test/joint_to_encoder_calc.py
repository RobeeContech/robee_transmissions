import numpy as np
class transmission_calc:
    def __init__(self,r,l,b,j_offset,mr,ppr,a_offset):
        self.r = r
        self.l = l
        self.b = b
        self.j_offset =j_offset
        self.mr = mr
        self.ppr = ppr
        self.a_offset = a_offset
    def j2a(self,j_pos):
        # print("r^2=" , pow(self.r,2) )
        # print("b^2=" , pow(self.b,2) )
        # print("-2rbcos(offset+pos) = " , -2*self.r*self.b*np.cos(self.j_offset+j_pos))
        # print(" r^2 + b^2 -2rbcos(offset+pos) = " ,(pow(self.r,2)+pow(self.b,2))-2*self.r*self.b*np.cos(self.j_offset+j_pos))
        # print("1) (r^2 + b^2 -2rbcos(offset+pos))/l = " ,((pow(self.r,2)+pow(self.b,2))-2*self.r*self.b*np.cos(self.j_offset+j_pos)/self.l))
        # print("2) mr*ppr/2pi = ", ((self.mr*self.ppr)/(2*np.pi)) )
        # print("1 * 2: ",((pow(self.r,2)+pow(self.b,2)-2*self.r*self.b*np.cos(self.j_offset+j_pos))/self.l)*((self.mr*self.ppr)/(2*np.pi)) )
        # print("1 * 2: -a_offset",((pow(self.r,2)+pow(self.b,2)-2*self.r*self.b*np.cos(self.j_offset+j_pos))/self.l)*((self.mr*self.ppr)/(2*np.pi)) - self.a_offset )
        return ((pow(self.r,2)+pow(self.b,2)-2*self.r*self.b*np.cos(self.j_offset+j_pos))/self.l)*((self.mr*self.ppr)/(2*np.pi)) - self.a_offset
    
def main():
    tr = transmission_calc(r=0.12554,l=0.1557,b=0.169,j_offset=1.2159709,mr=3.864,ppr=4000,a_offset=0.01585)
    print(tr.j2a(0.137881))
    print(tr.j2a(-0.261))

if __name__ == '__main__':
    main()