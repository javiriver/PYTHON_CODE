import cv2
import numpy as np
import math as m


#Camera2OI
# width = 7700
# Height = 10300
# pixelsize = 0.0052
# focal = 122.9697
# fx = focal / pixelsize
# fy = fx
# cx = (width/2) + (-0.024416/pixelsize)
# cy = (Height/2) - (6.7923/pixelsize)


#Camera4OI
width = 10300
Height = 7700
pixelsize = 0.0052
focal = 122.9857
fx = focal / pixelsize
fy = fx
cx = (width/2) + (-0.01726/pixelsize)
cy = (Height/2) - 0.0962/pixelsize

#Camera1
# width = 13470
# Height = 8670
# pixelsize = 0.0052
# focal = 82
# fx = focal / pixelsize
# fy = fx
# cx = (width/2)
# cy = (Height/2)


#Camera1
# x0 = 496494.689101
# y0 = 4414227.50303
# z0 = 1500
# x1 = 498093.482704
# y1 = 4414221.50202
# z1 = 1500
# x2 = 498089.19423
# y2 = 4413193.51472
# z2 = 1500
# x3 = 496491.992154
# y3 = 4413198.37782
# z3 = 1500


#Camera2
# x0 = 496532.648009
# y0 = 4413071.61255
# z0 = 1500
# x1 = 496472.478909
# y1 = 4414092.0689
# z1 = 1500
# x2 = 497997.068655
# y2 = 4414009.8808
# z2 = 1500
# x3 = 498037.413884
# y3 = 4413337.59738
# z3 = 1500

#Camera4
x0 = 497928.830375
y0 = 4412795.83779
z0 = 1500
x1 = 496558.414197
y1 = 4412821.09971
z1 = 1500
x2 = 496777.537167
y2 = 4414063.5688
z2 = 1500
x3 = 497764.35058
y3 = 4414043.86498
z3 = 1500

#Camera2
# x0 = 494486.601698
# y0 = 4412533.65853
# z0 = 0
# x1 = 494378.009486
# y1 = 4414375.36154
# z1 = 0
# x2 = 497129.56424
# y2 = 4414227.0298
# z2 = 0
# x3 = 497202.378652
# y3 = 4413013.70363
# z3 = 0

#camera2
# x0 = 496761.558755
# y0 = 4413131.7986
# z0 = 1677.0125114
#
# x1 = 496697.80867
# y1 = 4414061.59137
# z1 = 1677.0125114
#
# x2 = 498106.058874
# y2 = 4413982.59896
# z2 = 1677.0125114
#
# x3 = 498134.686401
# y3 = 4413375.32748
# z3 = 1677.0125114

#Camera 4
# x0 = 497868.338191
# y0 = 4413047.71527
# z0 = 1678.91588988
#
# x1 = 496624.629928
# y1 = 4413049.35916
# z1 = 1678.91588988
#
# x2 = 496826.342595
# y2 = 4414186.72434
# z2 = 1678.91588988
#
# x3 = 497716.588354
# y3 = 4414184.20406
# z3 = 1678.91588988

#Camera 1
# x0 = 496563.149655
# y0 = 4414182.95031
# z0 = 1672.99031415
#
# x1 = 498012.072673
# y1 = 4414168.989
# z1 = 1672.99031415
#
# x2 = 498016.008324
# y2 = 4413240.80314
# z2 = 1672.99031415
#
# x3 = 496560.170578
# y3 = 4413241.51492
# z3 = 1672.99031415

px0 = 0
py0 = 0
px1 = width
py1 = 0
px2 = width
py2 = Height
px3 = 0
py3 = Height



World = np.array([[[x0], [y0], [z0]], [[x1], [y1], [z1]], [[x2], [y2], [z2]], [[x3], [y3], [z3]]])

pix = np.array([[[px0], [py0]], [[px1], [py1]], [[px2], [py2]], [[px3], [py3]]], dtype=np.float64)
coef = np.zeros((5, 1))

CamMatrix = np.matrix([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

ret, rvec, tvec = cv2.solvePnP(World, pix, CamMatrix, coef)
imagepoints,jacobian = cv2.projectPoints(World, rvec, tvec, CamMatrix, coef)

error = pix - imagepoints

M = cv2.Rodrigues(rvec)
rot = M[0]

ext0 = rot[0, 0]
ext1 = rot[0, 1]
ext2 = rot[0, 2]
ext3 = rot[1, 0]
ext4 = rot[1, 1]
ext5 = rot[1, 2]
ext6 = rot[2, 0]
ext7 = rot[2, 1]
ext8 = rot[2, 2]


omega2 = m.atan2(-ext7, ext8) + m.pi
phiatan2 = m.atan2(ext6, m.sqrt(pow(ext7, 2) + pow(ext8, 2))) * -1
kappa2 = m.atan2(-ext3, ext0) * -1


print(omega2, phiatan2, kappa2)

z=1