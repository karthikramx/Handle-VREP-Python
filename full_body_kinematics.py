import math

class FBK:
    """
    all functionality related to full body kinematics
    link_lengths: list of link link_lengths
    type: 2R,2RP,3R,3RP,4R,5R
    """

    def __init__(self,link_lengths,type):
        self.link_lengths = link_lengths
        self.type = type
        self.joint_angles = []

    def inverse_kinematics_3R(self,x,z,phi,sig):
            #try:
                l1=self.link_lengths[0];
                l2=self.link_lengths[1];
                l3=self.link_lengths[2];

                x=-x

                x1 = x-l3*math.cos((phi*math.pi)/180);
                z1 = z-l3*math.sin((phi*math.pi)/180);

                z2 =  -(z1)/math.sqrt((x1**2)+(z1**2));
                x2 =  -(x1)/math.sqrt((x1**2)+(z1**2));

                #pitch angles
                t1 = math.atan2(z2,x2)+ sig*math.acos(-((x1**2)+(z1**2)+(l1**2)-(l2**2))/((2*l1)*(math.sqrt((x1**2)+(z1**2)))));
                z3 = (z1-l1*math.sin(t1))/l2;
                x3 = (x1-l1*math.cos(t1))/l2;
                t2 = (math.atan2(z3,x3) % (2*math.pi))-t1;
                t3 = (phi*(math.pi/180))-(t1+t2);

                self.joint_angles = []

                self.joint_angles.append(t1)
                self.joint_angles.append(t2)
                self.joint_angles.append(t3)

                self.joint_angles[0] = self.joint_angles[0] + 4.712388980384690
                self.joint_angles[1] = self.joint_angles[1] - 6.283185307180000


                # self.joint_angles[0] = (self.joint_angles[0]) % (2*math.pi)
                # self.joint_angles[1] = (self.joint_angles[1]) % (2*math.pi)
                # self.joint_angles[2] = (self.joint_angles[2]) % (2*math.pi)

                print str(self.joint_angles[0]) + ' ' + str(self.joint_angles[1]) + ' ' + str(self.joint_angles[2])

                return self.joint_angles

            #except:
        #        print("out of work space!")

            #return self.joint_angles
