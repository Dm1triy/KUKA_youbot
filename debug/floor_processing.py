## angle are degrees
## positive rotation is clockwise rotation
## zero angle is calculated from the direction line of the previous link

## platform height is ?
## first link height is ?
## camera is located ? sm above the 5th link
## the manipulator is located at a distance of ?sm from the center of the robot

## angles: a1 is rotation of the first link in the plane OXY
##         a2, a3, a4 is rotations of the links with m2_len, m3_len and m4_len in the plane OYZ (local y)
##         a5 is rotation of the grip (can affect camera height)


class arm:
    def __init__(self):
        self.robot_center = 0, 0, 0 # x, y, z
        self.arm_pos = 0, 0.2, 0.35 # manipulator pos from robot center
        self.m2_len = 155
        self.m3_len = 135
        self.m4_len = 200
        self.angles = self.read_angs()

    @staticmethod
    def read_angs():
        arm = 'armpos.txt'
        f = open(arm, "r")
        angles = list(map(float, f.read().split(' ')))
        f.close()
        return angles
