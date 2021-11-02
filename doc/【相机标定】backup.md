



##### 3 旋转相机拍摄的两张图片

可认为重叠部分的景物处于无限远的同一平面上。

![homography_transformation_example3.jpg](images/homography_transformation_example3.jpg)

最常用的场景就是全景拼接：

![homography_panorama_stitching.jpg](images/homography_panorama_stitching.jpg)


$$
\begin{align}
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
$$
