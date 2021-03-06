

# 【相机标定02】从世界坐标系到像素坐标系

在上一节[【相机标定01】内参的作用--投影](https://zhuanlan.zhihu.com/p/413081555)中，我们已经了解了内参的投影作用，换句话说，就是**相机坐标系到像素坐标系的转换**，本节我们再接触一个名词——**外参**。

首先从相机模型的成像过程说起，可以分为两个阶段：

- 第一阶段：从世界坐标系到相机坐标系
- 第二阶段：从相机坐标系投影到像素坐标系

习惯上，把第一阶段的变换矩阵称为外参，也就是**两个三维坐标系间的刚体变换**。

#### 1 从世界坐标系到相机坐标系

两个坐标系间转换在[动手学ROS（8）：坐标系间的欧氏变换](https://zhuanlan.zhihu.com/p/376835802)中已经介绍过，简单说就是左乘一个旋转矩阵 <img src="https://www.zhihu.com/equation?tex=R" alt="R" class="ee_img tr_noresize" eeimg="1"> ，然后再平移 <img src="https://www.zhihu.com/equation?tex=\vec t" alt="\vec t" class="ee_img tr_noresize" eeimg="1"> 。用公式表示如下：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
\left[
\begin{matrix}
x_c\\
y_c\\
z_c
\end{matrix}
\right]
&=
\left[
\begin{matrix}
r_{11} & r_{12} & r_{13}\\
r_{21} & r_{22} & r_{23}\\
r_{31} & r_{32} & r_{33}
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
x_o\\
y_o\\
z_o
\end{matrix}
\right]
+
\left[
\begin{matrix}
T_x\\
T_y\\
T_z
\end{matrix}
\right]
\end{align}\\
" alt="\begin{align}
\left[
\begin{matrix}
x_c\\
y_c\\
z_c
\end{matrix}
\right]
&=
\left[
\begin{matrix}
r_{11} & r_{12} & r_{13}\\
r_{21} & r_{22} & r_{23}\\
r_{31} & r_{32} & r_{33}
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
x_o\\
y_o\\
z_o
\end{matrix}
\right]
+
\left[
\begin{matrix}
T_x\\
T_y\\
T_z
\end{matrix}
\right]
\end{align}\\
" class="ee_img tr_noresize" eeimg="1">
也就是：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
P_c &= R \cdot P_o + T
\end{align}\\
" alt="\begin{align}
P_c &= R \cdot P_o + T
\end{align}\\
" class="ee_img tr_noresize" eeimg="1">
其中， <img src="https://www.zhihu.com/equation?tex=P_o" alt="P_o" class="ee_img tr_noresize" eeimg="1"> 为世界坐标系中的点， <img src="https://www.zhihu.com/equation?tex=P_c" alt="P_c" class="ee_img tr_noresize" eeimg="1"> 为相机坐标系中的点。这里面有个加法，如果需要转换多个坐标系就会层层嵌套，不够优雅，这里我们引入齐次坐标：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
\left[
\begin{matrix}
x_c\\
y_c\\
z_c\\
1
\end{matrix}
\right]
&=
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
\right]
\end{align}\\
" alt="\begin{align}
\left[
\begin{matrix}
x_c\\
y_c\\
z_c\\
1
\end{matrix}
\right]
&=
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
\right]
\end{align}\\
" class="ee_img tr_noresize" eeimg="1">
用齐次坐标就可以用一个矩阵乘法完成坐标变换，需要变换多个坐标系时也仅需要左乘多次即可。

**此处的 <img src="https://www.zhihu.com/equation?tex=(R,T)" alt="(R,T)" class="ee_img tr_noresize" eeimg="1"> 即为外参**。

#### 2 从相机坐标系到像素坐标系

由上一步的变换，我们已经求得了相机坐标系的点，接下来我们就可以运用上一节的内参投影公式，将3D点投影到成像平面上了：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
\left[
\begin{matrix}
x\\
y\\
w
\end{matrix}
\right]
&=
\left[
\begin{matrix}
f_x & 0 & c_{x}\\
0 & f_{y} & c_{y}\\
0 & 0 & 1
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
x_c\\
y_c\\
z_c\
\end{matrix}
\right]
\end{align}\\
" alt="\begin{align}
\left[
\begin{matrix}
x\\
y\\
w
\end{matrix}
\right]
&=
\left[
\begin{matrix}
f_x & 0 & c_{x}\\
0 & f_{y} & c_{y}\\
0 & 0 & 1
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
x_c\\
y_c\\
z_c\
\end{matrix}
\right]
\end{align}\\
" class="ee_img tr_noresize" eeimg="1">

即：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
\vec q &= M \cdot P_c
\end{align}\\
" alt="\begin{align}
\vec q &= M \cdot P_c
\end{align}\\
" class="ee_img tr_noresize" eeimg="1">
其中，M为内参， <img src="https://www.zhihu.com/equation?tex=P_c" alt="P_c" class="ee_img tr_noresize" eeimg="1"> 为相机坐标系下一点。也把它用齐次坐标形式表示出来：

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
x\\
y\\
w
\end{matrix}
\right]
=
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
x_c\\
y_c\\
z_c\\
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
x\\
y\\
w
\end{matrix}
\right]
=
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
x_c\\
y_c\\
z_c\\
1
\end{matrix}
\right]
\end{align}\\
" class="ee_img tr_noresize" eeimg="1">
**最终的 <img src="https://www.zhihu.com/equation?tex=(u,v)" alt="(u,v)" class="ee_img tr_noresize" eeimg="1"> 即是 <img src="https://www.zhihu.com/equation?tex=P_o" alt="P_o" class="ee_img tr_noresize" eeimg="1"> 的像素坐标**。

#### 小结

世界坐标系中的点需要经过两步运算转换到像素坐标：

- 从世界坐标系到相机坐标系
- 从相机坐标系投影到像素坐标系

这个过程可以描述为：

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
x_c\\
y_c\\
z_c\\
1
\end{matrix}
\right]\\
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
\right]\\
&=
P \cdot 
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
x_c\\
y_c\\
z_c\\
1
\end{matrix}
\right]\\
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
\right]\\
&=
P \cdot 
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
其中 <img src="https://www.zhihu.com/equation?tex=P" alt="P" class="ee_img tr_noresize" eeimg="1"> 是一个 <img src="https://www.zhihu.com/equation?tex=3 \times 4" alt="3 \times 4" class="ee_img tr_noresize" eeimg="1"> 的矩阵，称为投影矩阵，概括了整个投影过程，我们以后还会使用到它，这里暂时不展开介绍。

最后，我们目前只是看到了内参的构成，但还没有介绍它为什么是这个形式。笔者的意图是首先通过了解内参的用途和意义，知其然后再知其所以然，这部分在相机模型中会涉及。

