import cv2
import numpy as np
import math as m
import csv



class ImageVexcel:
    ImageName = "Name"
    CameraID = "1"
    focalLength = 0.0
    ppox = 0.0
    ppoy = 0.0
    PixSize = 0.0
    Width = 0.0
    Height = 0.0
    omega = 0.0
    phi = 0.0
    kappa = 0.0
    Xcp = 0.0
    Ycp = 0.0
    Zcp = 0.0

    Corner_ul = np.empty((1, 3))
    Corner_ur = np.empty((1, 3))
    Corner_dr = np.empty((1, 3))
    Corner_dl = np.empty((1, 3))

    extXYZ = np.empty((3, 3))
    extYXZ = np.empty((3, 3))

    omegaXYZ = 0
    phiXYZ = 0
    kappaXYZ = 0

def CalculateMatrixYXZ(omega, phi, kappa):

    sinome = m.sin(omega)
    sinphi = m.sin(phi)
    sinkap = m.sin(kappa)

    cosome = m.cos(omega)
    cosphi = m.cos(phi)
    coskap = m.cos(kappa)


    ImageVexcel.extYXZ[0, 0] = coskap*cosphi + sinkap*sinome*sinphi
    ImageVexcel.extYXZ[0, 1] = sinkap*cosome
    ImageVexcel.extYXZ[0, 2] = -coskap*sinphi + sinkap*sinome*cosphi
    ImageVexcel.extYXZ[1, 0] = -sinkap * cosphi + coskap * sinome * sinphi
    ImageVexcel.extYXZ[1, 1] = coskap * cosome
    ImageVexcel.extYXZ[1, 2] = sinkap*sinphi + coskap*sinome*cosphi
    ImageVexcel.extYXZ[2, 0] = cosome*sinphi
    ImageVexcel.extYXZ[2, 1] = -sinome
    ImageVexcel.extYXZ[2, 2] = cosome*cosphi

def ProjectPixeltoPlaneYXZrot(ZPlane, PixelX, PixelY):

    Photocoord = TransformPixelToPhotocoord(PixelX, PixelY)

    u = ImageVexcel.extYXZ[0, 0] * Photocoord[0] + ImageVexcel.extYXZ[1, 0] * Photocoord[1] - ImageVexcel.extYXZ[2, 0] * ImageVexcel.focalLength
    v = ImageVexcel.extYXZ[0, 1] * Photocoord[0] + ImageVexcel.extYXZ[1, 1] * Photocoord[1] - ImageVexcel.extYXZ[2, 1] * ImageVexcel.focalLength
    w = ImageVexcel.extYXZ[0, 2] * Photocoord[0] + ImageVexcel.extYXZ[1, 2] * Photocoord[1] - ImageVexcel.extYXZ[2, 2] * ImageVexcel.focalLength

    worldx = ImageVexcel.Xcp + (ZPlane - ImageVexcel.Zcp) * u / w
    worldy = ImageVexcel.Ycp + (ZPlane - ImageVexcel.Zcp) * v / w

    return worldx, worldy, ZPlane

def TransformPixelToPhotocoord(pixelx, pixely):

    photox = pixelx * ImageVexcel.PixSize - ((ImageVexcel.Width * ImageVexcel.PixSize) / 2) - (ImageVexcel.ppox )
    photoy = pixely * ImageVexcel.PixSize + ((ImageVexcel.Height * ImageVexcel.PixSize) / 2) - (ImageVexcel.ppoy )

    return photox, photoy

    # rrSq = x0 * x0 + y0 * y0
    # dr = lensInformation.K0 +
    # lensInformation.K1 * rrSq +
    # lensInformation.K2 * rrSq * rrSq +
    # lensInformation.K3 * rrSq * rrSq * rrSq;
    # photox = x0 * (1 + dr) + lensInformation.P1 * (
    #     rrSq + 2 * x0 * x0) + 2 * lensInformation.P2 * x0 * y0 + lensInformation.B1 * x0 + lensInformation.B2 * y0;
    # photoy = y0 * (1 + dr) + lensInformation.P2 * (rrSq + 2 * y0 * y0) + 2 * lensInformation.P1 * x0 * y0;

def CalculateMatrixXYZ():
    World = np.array([[[ImageVexcel.Corner_ul[0]], [ImageVexcel.Corner_ul[1]], [ImageVexcel.Corner_ul[2]]],
                      [[ImageVexcel.Corner_ur[0]], [ImageVexcel.Corner_ur[1]], [ImageVexcel.Corner_ur[2]]],
                      [[ImageVexcel.Corner_dr[0]], [ImageVexcel.Corner_dr[1]], [ImageVexcel.Corner_dr[2]]],
                      [[ImageVexcel.Corner_dl[0]], [ImageVexcel.Corner_dl[1]], [ImageVexcel.Corner_dl[2]]]])

    pix = np.array([[[0], [0]], [[ImageVexcel.Width], [0]], [[ImageVexcel.Width], [ImageVexcel.Height]],
                    [[0], [ImageVexcel.Height]]], dtype=np.float64)

    DitortionCoef = np.zeros((5, 1))

    fx = ImageVexcel.focalLength / ImageVexcel.PixSize
    fy = fx
    cx = (ImageVexcel.Width / 2) + (ImageVexcel.ppox / ImageVexcel.PixSize)
    cy = (ImageVexcel.Height / 2) - (ImageVexcel.ppoy / ImageVexcel.PixSize)

    CamMatrix = np.matrix([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    ret, rvec, tvec = cv2.solvePnP(World, pix, CamMatrix, DitortionCoef)
    imagepoints, jacobian = cv2.projectPoints(World, rvec, tvec, CamMatrix, DitortionCoef)
    ErrorReprojection = pix - imagepoints
    M = cv2.Rodrigues(rvec)
    ImageVexcel.extXYZ = M[0]

    ImageVexcel.omegaXYZ = m.atan2(-ImageVexcel.extXYZ[2, 1], ImageVexcel.extXYZ[2, 2]) + m.pi
    ImageVexcel.phiXYZ = m.atan2(ImageVexcel.extXYZ[2, 0],
                                 m.sqrt(pow(ImageVexcel.extXYZ[2, 1], 2) + pow(ImageVexcel.extXYZ[2, 2], 2))) * -1
    ImageVexcel.kappaXYZ = m.atan2(-ImageVexcel.extXYZ[1, 0], ImageVexcel.extXYZ[0, 0]) * -1


Pathentrada = "E:/025-Vexcel/OFF-LINE/westminster/USCOWES1907/D190731/B1/Fulleo-vexcel_westminster_rotado_YXZ.csv"

corners = np.array([[0, 0], [ImageVexcel.Width, 0], [ImageVexcel.Width, ImageVexcel.Height], [0, ImageVexcel.Height]])

with open(Pathentrada) as f:
     reader = csv.reader(f, delimiter=";")
     i=0
     for row in reader:
        if i > 7:
         ImageVexcel.ImageName = str(row[0])
         ImageVexcel.Xcp = float(row[5])
         ImageVexcel.Ycp = float(row[6])
         ImageVexcel.Zcp = float(row[7])
         ImageVexcel.omega = float(row[8])
         ImageVexcel.phi = float(row[9])
         ImageVexcel.kappa = float(row[10])
         ImageVexcel.CameraID = row[11]
         ImageVexcel.PixSize = float(row[12])
         ImageVexcel.Width = float(row[13])
         ImageVexcel.Height = float(row[14])
         ImageVexcel.focalLength = float(row[15])
         ImageVexcel.ppox = float(row[16])
         ImageVexcel.ppoy = float(row[17])

         CalculateMatrixYXZ(ImageVexcel.omega, ImageVexcel.phi, ImageVexcel.kappa)

         ImageVexcel.Corner_ul = ProjectPixeltoPlaneYXZrot(0, 0, 0)
         ImageVexcel.Corner_ur = ProjectPixeltoPlaneYXZrot(0, ImageVexcel.Width, 0)
         ImageVexcel.Corner_dr = ProjectPixeltoPlaneYXZrot(0, ImageVexcel.Width, -ImageVexcel.Height)
         ImageVexcel.Corner_dl = ProjectPixeltoPlaneYXZrot(0, 0, -ImageVexcel.Height)

         CalculateMatrixXYZ()

         outpathfile = "E:/025-Vexcel/OFF-LINE/westminster/USCOWES1907/D190731/B1/Fulleo-vexcel_westminster_XYZ-test.csv"
         EOstring = str(ImageVexcel.ImageName)+";" + str(ImageVexcel.CameraID) + ";" + str(ImageVexcel.Xcp) +";" + str(ImageVexcel.Ycp) +";" + str(ImageVexcel.Zcp) +";" + str(ImageVexcel.omegaXYZ) +";" + str(ImageVexcel.phiXYZ) +";" + str(ImageVexcel.kappaXYZ) + "\n"
         file = open(outpathfile, "a")
         file.write(EOstring)
         file.close()

        else:
            i = i+1




