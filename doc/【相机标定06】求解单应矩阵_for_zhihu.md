# 【相机标定06】求解单应矩阵

在上一节《【相机标定05】单应矩阵的由来》中我们已经一步一步认识了相机标定的核心--单应矩阵的由来，本节就简单介绍下如何求解单应矩阵，这也是相机标定中最主要的过程，因为**内参就是从单应矩阵里分解出来的**。

我们已经知道，单应矩阵 <img src="https://www.zhihu.com/equation?tex=H= s \cdot M \cdot \left[\begin{matrix}\vec r_1 & \vec r_2 & \vec t\end{matrix} \right]" alt="H= s \cdot M \cdot \left[\begin{matrix}\vec r_1 & \vec r_2 & \vec t\end{matrix} \right]" class="ee_img tr_noresize" eeimg="1">  是一个 <img src="https://www.zhihu.com/equation?tex=3 \times 3" alt="3 \times 3" class="ee_img tr_noresize" eeimg="1"> 的矩阵。在《视觉SLAM十四讲》中，作者提供了一种比较简单的方法，叫直接线性变换法(Direct Linear Transform)，这种作法把 <img src="https://www.zhihu.com/equation?tex=H" alt="H" class="ee_img tr_noresize" eeimg="1"> 矩阵看成一个向量，通过解该向量的线性方程来恢复 <img src="https://www.zhihu.com/equation?tex=H" alt="H" class="ee_img tr_noresize" eeimg="1"> 。

我们重写一下上节中的结论，可以更清楚地表达出单应矩阵的含义：共面的点在两个图像上的变换关系。

<img src="https://www.zhihu.com/equation?tex=\begin{align}
\left[
\begin{matrix}
u\\
v\\
1
\end{matrix}
\right]
&= s \cdot H \cdot
\left[
\begin{matrix}
u_0\\
v_0\\
1
\end{matrix}
\right]\\
&=
s \cdot 
\left[
\begin{matrix}
h_1 & h_2 & h_3\\
h_4 & h_5 & h_6\\
h_7 & h_8 & h_9\\
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
u_0\\
v_0\\
1
\end{matrix}
\right]\\
\end{align}\\
" alt="\begin{align}
\left[
\begin{matrix}
u\\
v\\
1
\end{matrix}
\right]
&= s \cdot H \cdot
\left[
\begin{matrix}
u_0\\
v_0\\
1
\end{matrix}
\right]\\
&=
s \cdot 
\left[
\begin{matrix}
h_1 & h_2 & h_3\\
h_4 & h_5 & h_6\\
h_7 & h_8 & h_9\\
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
u_0\\
v_0\\
1
\end{matrix}
\right]\\
\end{align}\\
" class="ee_img tr_noresize" eeimg="1">

由于s所代表的齐次性，可以把 <img src="https://www.zhihu.com/equation?tex=h_9" alt="h_9" class="ee_img tr_noresize" eeimg="1"> 置为1，具体如下：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
\left[
\begin{matrix}
u\\
v\\
1
\end{matrix}
\right]
&=
s \cdot 
\left[
\begin{matrix}
h_1 & h_2 & h_3\\
h_4 & h_5 & h_6\\
h_7 & h_8 & h_9\\
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
u_0\\
v_0\\
1
\end{matrix}
\right]\\
&=
{s \over h_9}
\cdot 
\left[
\begin{matrix}
h_1 \over h_9 & h_2 \over h_9 & h_3 \over h_9\\
h_4 \over h_9 & h_5 \over h_9 & h_6 \over h_9\\
h_7 \over h_9 & h_8 \over h_9 & 1
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
u_0\\
v_0\\
1
\end{matrix}
\right]\\
&=
w \cdot
\left[
\begin{matrix}
H_1 & H_2 & H_3\\
H_4 & H_5 & H_6\\
H_7 & H_8 & 1
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
u_0\\
v_0\\
1
\end{matrix}
\right]\\

\end{align}\\
" alt="\begin{align}
\left[
\begin{matrix}
u\\
v\\
1
\end{matrix}
\right]
&=
s \cdot 
\left[
\begin{matrix}
h_1 & h_2 & h_3\\
h_4 & h_5 & h_6\\
h_7 & h_8 & h_9\\
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
u_0\\
v_0\\
1
\end{matrix}
\right]\\
&=
{s \over h_9}
\cdot 
\left[
\begin{matrix}
h_1 \over h_9 & h_2 \over h_9 & h_3 \over h_9\\
h_4 \over h_9 & h_5 \over h_9 & h_6 \over h_9\\
h_7 \over h_9 & h_8 \over h_9 & 1
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
u_0\\
v_0\\
1
\end{matrix}
\right]\\
&=
w \cdot
\left[
\begin{matrix}
H_1 & H_2 & H_3\\
H_4 & H_5 & H_6\\
H_7 & H_8 & 1
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
u_0\\
v_0\\
1
\end{matrix}
\right]\\

\end{align}\\
" class="ee_img tr_noresize" eeimg="1">
展开后得：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
u &= {{H_1u_0 + H_{2} v_0+H_3} \over {H_7u_0+H_8v_0+1}}\\
v &= {H_4u_0 + H_5 v_0+H_6 \over {H_7u_0+H_8v_0+1}}
\end{align}
" alt="\begin{align}
u &= {{H_1u_0 + H_{2} v_0+H_3} \over {H_7u_0+H_8v_0+1}}\\
v &= {H_4u_0 + H_5 v_0+H_6 \over {H_7u_0+H_8v_0+1}}
\end{align}
" class="ee_img tr_noresize" eeimg="1">
稍微调整一下位置：

<img src="https://www.zhihu.com/equation?tex=\begin{align}
H_1u_0 + H_2 v_0+H_3 - H_7u_0u-H_8v_0u &= u\\
H_4u_0 + H_5 v_0+H_6 - H_7u_0v-H_8v_0v &= v
\end{align}
" alt="\begin{align}
H_1u_0 + H_2 v_0+H_3 - H_7u_0u-H_8v_0u &= u\\
H_4u_0 + H_5 v_0+H_6 - H_7u_0v-H_8v_0v &= v
\end{align}
" class="ee_img tr_noresize" eeimg="1">


也就是说，一对点的像素坐标可以构成两个约束，那8个未知量或者说自由度为8的单应矩阵就可以由4对匹配的点求出，我们写为矩阵的形式。假如这4对点为 <img src="https://www.zhihu.com/equation?tex=(u_1,v_1)(u_2,v_2)、(u_3,v_3)(u_4,v_4)、(u_5,v_5)(u_6,v_6)、(u_7,v_7)(u_8,v_8)" alt="(u_1,v_1)(u_2,v_2)、(u_3,v_3)(u_4,v_4)、(u_5,v_5)(u_6,v_6)、(u_7,v_7)(u_8,v_8)" class="ee_img tr_noresize" eeimg="1"> 两两匹配：

<img src="https://www.zhihu.com/equation?tex=\begin{align} 
\left[
\begin{matrix}
u_1 & v_1 & 1 & 0 & 0 & 0 & -u_1u_2 & -v_1u_2\\
0 & 0 & 0 & u_1 & v_1 & 1 & -u_1v_2 & -v_1v_2\\
u_3 & v_3 & 1 & 0 & 0 & 0 & -u_3u_4 & -v_3u_4\\
0 & 0 & 0 & u_3 & v_3 & 1 & -u_3v_4 & -v_3v_4\\
u_5 & v_5 & 1 & 0 & 0 & 0 & -u_5u_6 & -v_5u_6\\
0 & 0 & 0 & u_5 & v_5 & 1 & -u_5v_6 & -v_5v_6\\
u_7 & v_7 & 1 & 0 & 0 & 0 & -u_7u_8 & -v_7u_8\\
0 & 0 & 0 & u_7 & v_7 & 1 & -u_7v_8 & -v_7v_8
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
H_1\\
H_2\\
H_3\\
H_4\\
H_5\\
H_6\\
H_7\\
H_8\\
\end{matrix}
\right]
=
\left[
\begin{matrix}
u_2\\
v_2\\
u_4\\
v_4\\
u_6\\
v_6\\
u_8\\
v_8\\
\end{matrix}
\right]
\end{align}\\
" alt="\begin{align} 
\left[
\begin{matrix}
u_1 & v_1 & 1 & 0 & 0 & 0 & -u_1u_2 & -v_1u_2\\
0 & 0 & 0 & u_1 & v_1 & 1 & -u_1v_2 & -v_1v_2\\
u_3 & v_3 & 1 & 0 & 0 & 0 & -u_3u_4 & -v_3u_4\\
0 & 0 & 0 & u_3 & v_3 & 1 & -u_3v_4 & -v_3v_4\\
u_5 & v_5 & 1 & 0 & 0 & 0 & -u_5u_6 & -v_5u_6\\
0 & 0 & 0 & u_5 & v_5 & 1 & -u_5v_6 & -v_5v_6\\
u_7 & v_7 & 1 & 0 & 0 & 0 & -u_7u_8 & -v_7u_8\\
0 & 0 & 0 & u_7 & v_7 & 1 & -u_7v_8 & -v_7v_8
\end{matrix}
\right]
\cdot
\left[
\begin{matrix}
H_1\\
H_2\\
H_3\\
H_4\\
H_5\\
H_6\\
H_7\\
H_8\\
\end{matrix}
\right]
=
\left[
\begin{matrix}
u_2\\
v_2\\
u_4\\
v_4\\
u_6\\
v_6\\
u_8\\
v_8\\
\end{matrix}
\right]
\end{align}\\
" class="ee_img tr_noresize" eeimg="1">

接下来就是个解方程问题了。求出单应矩阵以后需要对其进行分解，就可以得到相应的旋转矩阵 <img src="https://www.zhihu.com/equation?tex=R" alt="R" class="ee_img tr_noresize" eeimg="1">  和平衡向量 <img src="https://www.zhihu.com/equation?tex=t" alt="t" class="ee_img tr_noresize" eeimg="1"> 了。留待后续讲解。

#### 小结

求解单应矩阵还是比较直观的，仅需4对点。当然点对越多越好，我们可以用最小二乘或RANSAC算法等来求解。

#### 参考

1 视觉SLAM十四讲--从理论到实践，高翔。
