# from math import sin, cos
# PI = 3.14159265359

# def required_motor_rpm(r:float, d:float, theta:float, v:float) -> tuple:
#     """return u1, u2, u3, ie. how fast each wheel must rotate to move the robot linearly in any direction"""
#     vbx = sin(theta) * v
#     vby = cos(theta) * v
#     u1 = (-d*theta + vbx) / r *( 60 / (2 * PI))# how fast wheel 1 must rotate
#     u2 = (-d*theta - 0.5*vbx - 0.866*vby) / r * ( 60 / (2 * PI))
#     u3 = (-d*theta - 0.5*vbx + 0.866*vby) / r * ( 60 / (2 * PI))
#     return u1, u2, u3 # return the _targetrpm of each wheel 
#     #that is angular velocity of the wheel

# pi = 3.14159265359
# r = 5 # TODO: changel latercm
# d = 12.5 # TODO: changel later
# v = 1 # say 200 rpm constant 

# theta = 0 / 180 * 2 * pi# radian
# u1, u2, u3 = required_motor_rpm(r, d, theta, v)
# print(f"u1={u1}, u2={u2}, u3={u3:.2f}") # u1=-8.272924284408578e-12, u2=34.640000000004136, u3=-34.63999999999586

# theta = 90 / 180 * 2 * pi# 
# u1, u2, u3 = required_motor_rpm(r, d, theta, v)
# print(f"u1={u1}, u2={u2}, u3={u3}") #u1=-6.283185307188273, u2=28.356814692824134, u3=-40.923185307175864
# theta = 180 / 180 * 2 * pi#
# u1, u2, u3 = required_motor_rpm(r, d, theta, v)
# print(f"u1={u1}, u2={u2}, u3={u3}") #u1=-12.566370614368273, u2=22.073629385644136, u3=-47.20637061435586
# theta = 270 / 180 * 2 * pi# 
# u1, u2, u3 = required_motor_rpm(r, d, theta, v)
# print(f"u1={u1}, u2={u2}, u3={u3}") # u1=-18.849555921548276, u2=15.79044407846413, u3=-53.48955592153586
# theta = 360 / 180 * 2 * pi# 
# u1, u2, u3 = required_motor_rpm(r, d, theta, v)
# print(f"u1={u1}, u2={u2}, u3={u3}") #u1=-25.0629280586485, u2=9.577071941363908, u3=-59.70292805863609


x= 0.0
y = int(x)
print(x == 0)
print(y == 0)