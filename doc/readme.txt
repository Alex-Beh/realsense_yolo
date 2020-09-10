Equation png::Computer_Modern_10pt_120_White
1. From Intrinsic Camera Calibration
\newline u=\frac{(x_{min} +x_{max})}{2}\newline
v=\frac{(y_{min} +y_{max})}{2}\newline
u=\frac{(x_{min} +x_{max})}{2}\newline \newline
\therefore \newline
Z_{real} = 0.0001\cdot \left [cv\_ptr.at<u\_int16\_t>(v,u)\right ] \newline
X_{real} = (u-c_x)*\frac{Z_{real}}{f_x} \newline
Y_{real} = (v-c_y)*\frac{Z_{real}}{f_y}

2. PointCloud from PointCloud2 to get xyz
\newline sensor\_msgs::convertPointCloud2ToPointCloud(*pointcloud\_msg, out\_cloud);\newline
x\_center = \frac{(i.m\_bbox.x_{min}+i.m\_bbox.x{max})}{2};\newline
y\_center = \frac{(i.m\_bbox.y_{min}+i.m\_bbox.y{max})}{2};\newline
\therefore \newline
int\hspace{0.2cm}ind = x_{centre} + y_{centre}\cdot pointcloud\_msg\rightarrow width; \newline
X_{real}= (float)out\_cloud.points[ind].x; \newline
Y_{real}= (float)out\_cloud.points[ind].y; \newline
Z_{real}= (float)out\_cloud.points[ind].z; \newline
