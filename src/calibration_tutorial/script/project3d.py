#!/usr/bin/env python2
import numpy as np
import cv2

def getMatrix(filename):
    f=cv2.FileStorage(filename,cv2.FileStorage_READ)
    M1=f.getNode('M1').mat()
    M2=f.getNode('M2').mat()
    D1=f.getNode('D1').mat()
    D2=f.getNode('D2').mat()
    R=f.getNode('R').mat()
    T=f.getNode('T').mat()
    f.release()
    return {"M1":M1,"M2":M2,"D1":D1,"D2":D2,"R":R,"T":T}

def project2pixel(points,m):
    m1=m['M1']
    d1=m['D1']
    m2=m['M2']
    d2=m['D2']
    r=m['R']
    t=m['T']
    # d0=np.array([0,0,0,0,0],np.float32)
    # project 3D points to image plane
    imgpts_l, jac = cv2.projectPoints(points, np.array([0,0,0], np.float32), np.array([0,0,0], np.float32), m1, d1) 
    imgpts_r,_=cv2.projectPoints(points, r, t,m2,d2)
    imgpts_l = np.int32(imgpts_l).reshape(-1,2)
    imgpts_r = np.int32(imgpts_r).reshape(-1,2)

    return imgpts_l,imgpts_r

def project2d(points,m):
    d0=np.array([0,0,0,0,0],np.float32)
    pts, jac = cv2.projectPoints(points, np.array([0,0,0], np.float32), np.array([0,0,0], np.float32), m, d0)
    pts = np.int32(pts).reshape(-1,2)
    return pts
def project2d_simple(points,m):
    pts=[]
    for point in points:
        pt=np.matmul(m,point)
        pt=pt/pt[2]
        pts.append(pt[:2])
    return np.int32(pts).reshape(-1,2)

def draw_point(img, points):
    for point in points:
        cv2.circle(img,tuple(point.ravel()),3,(0,255,0),3,cv2.LINE_8)

def project3d(point,m):
    p= np.matmul(m,point.transpose())
    return np.int32(p/p[2])[:2]

if __name__ =="__main__":
    fn="test/projection/intrinsics.xml"
    m=getMatrix(fn)
    print(m)

    m1=np.array([[5.3398795245975896e+02, 0., 3.2838647449406972e+02],
       [0.,5.2871082110006125e+02, 2.3684272831168110e+02],
       [0., 0., 1.]])
    
    img_l_file="test/projection/left01.jpg"
    img_r_file="test/projection/right01.jpg"
    img_l=cv2.imread(img_l_file)
    img_r=cv2.imread(img_r_file)

    # p2d=project3d(points[0],m['M1'])
    # pts_l, pts_r=project2pixel(points,m)

    points=np.array([[-48.831894, -86.462112, 316.351135],[-29.942320, -85.639320, 311.097839],[-11.202976, -85.063042, 306.122345]])
    
    pts=project2d(points,m1)

    pts_1=project2d_simple(points,m1)
    draw_point(img_l,pts)


    # for pt in pts_l:
    #     cv2.drawMarker(img_l,tuple(pt.ravel()),(0,255,0),thickness=4)
    # for pt in pts_r:
    #     cv2.drawMarker(img_r,tuple(pt.ravel()),(0,255,0),thickness=4)

    # draw_point(img_l,pts_l)
    # draw_point(img_r,pts_r)

    cv2.imwrite('projection_demo_l.jpg',img_l)
    # cv2.imwrite('projection_r.jpg',img_r)

    