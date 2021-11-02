# 【相机标定05】单应矩阵的由来

在[【相机标定02】从世界坐标系到像素坐标系](https://zhuanlan.zhihu.com/p/423473576)中，我们已经知道一个世界坐标系中的点投影到像素坐标系的过程为：


<img src="https://www.zhihu.com/equation?tex=\begin{align}
w
\left[
\begin{matrix}
u\\
v\\
1
\end{matrix}
\right]
&=
\left[
\begin{matrix}
f_x & 0 & c_{x} & 0\\
0 & f_{y} & c_{y} & 0\\
0 & 0 & 1 & 0
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
r_{11} & r_{12} & r_{13} & T_x\\
r_{21} & r_{22} & r_{23} & T_y\\
r_{31} & r_{32} & r_{33} & T_z\\
0 &　0 & 0 & 1
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
x_o\\
y_o\\
z_o\\
1
\end{matrix}
\right]\\
&=
\left[
\begin{matrix}
f_x & 0 & c_{x} \\
0 & f_{y} & c_{y} \\
0 & 0 & 1 
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
r_{11} & r_{12} & r_{13} & T_x\\
r_{21} & r_{22} & r_{23} & T_y\\
r_{31} & r_{32} & r_{33} & T_z
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
x_o\\
y_o\\
z_o\\
1
\end{matrix}
\right]

\end{align}\\
" alt="\begin{align}
w
\left[
\begin{matrix}
u\\
v\\
1
\end{matrix}
\right]

&=
\left[
\begin{matrix}
f_x & 0 & c_{x} & 0\\
0 & f_{y} & c_{y} & 0\\
0 & 0 & 1 & 0
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
r_{11} & r_{12} & r_{13} & T_x\\
r_{21} & r_{22} & r_{23} & T_y\\
r_{31} & r_{32} & r_{33} & T_z\\
0 &　0 & 0 & 1
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
x_o\\
y_o\\
z_o\\
1
\end{matrix}
\right]\\
&=
\left[
\begin{matrix}
f_x & 0 & c_{x} \\
0 & f_{y} & c_{y} \\
0 & 0 & 1 
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
r_{11} & r_{12} & r_{13} & T_x\\
r_{21} & r_{22} & r_{23} & T_y\\
r_{31} & r_{32} & r_{33} & T_z
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
x_o\\
y_o\\
z_o\\
1
\end{matrix}
\right]

\end{align}\\
" class="ee_img tr_noresize" eeimg="1">

我们用一个简单的式子来概括上面的投影过程：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
\vec p &= s \cdot M \cdot W \cdot \vec Q
\end{align}\\
" alt="\begin{align}
\vec p &= s \cdot M \cdot W \cdot \vec Q
\end{align}\\
" class="ee_img tr_noresize" eeimg="1">
其中 <img src="https://www.zhihu.com/equation?tex=\vec p=\left[\begin{matrix}u\\v\\1\end{matrix}\right] " alt="\vec p=\left[\begin{matrix}u\\v\\1\end{matrix}\right] " class="ee_img tr_noresize" eeimg="1"> ， <img src="https://www.zhihu.com/equation?tex=s=1/w" alt="s=1/w" class="ee_img tr_noresize" eeimg="1"> ， <img src="https://www.zhihu.com/equation?tex=M" alt="M" class="ee_img tr_noresize" eeimg="1"> 为内参， <img src="https://www.zhihu.com/equation?tex=W=\left[\begin{matrix}\vec r_1 & \vec r_2 & \vec r3 & \vec t\end{matrix} \right]" alt="W=\left[\begin{matrix}\vec r_1 & \vec r_2 & \vec r3 & \vec t\end{matrix} \right]" class="ee_img tr_noresize" eeimg="1"> 为外参， <img src="https://www.zhihu.com/equation?tex=\vec Q=\left[\begin{matrix}x_o\\y_o\\z_o\\1\end{matrix}\right]" alt="\vec Q=\left[\begin{matrix}x_o\\y_o\\z_o\\1\end{matrix}\right]" class="ee_img tr_noresize" eeimg="1"> 。

到这里，我们需要停下来思考一下，上式的已知量是哪些，哪些又是未知的。内参 <img src="https://www.zhihu.com/equation?tex=M" alt="M" class="ee_img tr_noresize" eeimg="1"> 是我们需要标定的未知量，外参W也是未知的；p可以从图片上查得，Q是人为选择的，这两个点都是已知的。

但由于 <img src="https://www.zhihu.com/equation?tex=Q" alt="Q" class="ee_img tr_noresize" eeimg="1"> 所在的世界坐标系是人为定义的，我们就可以简化求解过程，让 <img src="https://www.zhihu.com/equation?tex=z_o=0" alt="z_o=0" class="ee_img tr_noresize" eeimg="1"> ，也就是所有的点都在同一个平面上且z轴为0，由此，可以将卡式简化为：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
\vec p &= s \cdot M \cdot 
\left[
\begin{matrix}
\vec r_1 & \vec r_2 & \vec r3 & \vec t
\end{matrix} \right]
\cdot
\left[
\begin{matrix}
x_o\\
y_o\\
0\\
1
\end{matrix}
\right]\\
&=
s \cdot M \cdot 
\left[
\begin{matrix}
\vec r_1 & \vec r_2 & \vec t
\end{matrix} \right]
\cdot
\left[
\begin{matrix}
x_o\\
y_o\\
1
\end{matrix}
\right]\\
&= s \cdot H \cdot \vec Q^\prime

\end{align}\\
" alt="\begin{align}
\vec p &= s \cdot M \cdot 
\left[
\begin{matrix}
\vec r_1 & \vec r_2 & \vec r3 & \vec t
\end{matrix} \right]
\cdot
\left[
\begin{matrix}
x_o\\
y_o\\
0\\
1
\end{matrix}
\right]\\
&=
s \cdot M \cdot 
\left[
\begin{matrix}
\vec r_1 & \vec r_2 & \vec t
\end{matrix} \right]
\cdot
\left[
\begin{matrix}
x_o\\
y_o\\
1
\end{matrix}
\right]\\
&= s \cdot H \cdot \vec Q^\prime

\end{align}\\
" class="ee_img tr_noresize" eeimg="1">

其中， <img src="https://www.zhihu.com/equation?tex=H= s \cdot M \cdot \left[\begin{matrix}\vec r_1 & \vec r_2 & \vec t\end{matrix} \right]" alt="H= s \cdot M \cdot \left[\begin{matrix}\vec r_1 & \vec r_2 & \vec t\end{matrix} \right]" class="ee_img tr_noresize" eeimg="1">  是一个 <img src="https://www.zhihu.com/equation?tex=3 \times 3" alt="3 \times 3" class="ee_img tr_noresize" eeimg="1"> 的矩阵，称之为单应矩阵（Homography matrix）。

用一幅图总结一下这个过程：

![image-20211101212423998](https://raw.githubusercontent.com/GreatWaller/ros-tutorial-for-beginners/main/doc/images/image-20211101212423998.png)

#### 小结

共面的点可以通过同一个变换 <img src="https://www.zhihu.com/equation?tex=H" alt="H" class="ee_img tr_noresize" eeimg="1"> 映射到成像平面上，这就是矩阵的全部意义。同时，我们注意到单应矩阵是融入了内参的，因此内参的估计就可以从求解单应矩阵开始，到这里我们已经了解了相机内参估计的重点前置知识，这也是即将介绍的`张氏标定法`的核心思想。
