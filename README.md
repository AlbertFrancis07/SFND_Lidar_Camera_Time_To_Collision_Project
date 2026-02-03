# SFND_Lidar_Camera_TTC_Estimation_FINAL_Project-Report

FP.5)Performance Evaluation based on lIdar BASED TTC Computation
While evaluating Lidar based TTC estimations, several frames were identified where the computed TTC  deviates significantly from a manually estimated TTC derived from  the nearest LiDAR point in top-view. The manual estimation was based on reduction in observed distance between consecutive frames and serves asd a physical reference. Three such frames are discussed below .
    
    i)Frame 4 : The computed TTC was around 16.68 s but the manual esitmated TTC was observed to be around 7 seconds. Based on Manual calculated TTC The distance reduction is significantly large compared to previous frame  which indicated a strong close motion.

    ii)Frame 7 : The computed TTC was around 12s but the manual esitmated TTC was observed to be around 25 seconds. However, inspection of the top-view LiDAR data shows that the distance to the nearest LiDAR point has in fact decreased compared to the previous frame. This inconsistency indicates that the TTC estimation does not accurately reflect the actual relative motion in this frame.

    iii)Frame 12 : The computed TTC was around 9 sec but the manually estimated TTC was observed to be negative in sign which indicate a increase in distance between ego vehicle and the predding vehicle. This result contradicts the observed top-view LiDAR measurements, which show a continued reduction in distance to the nearest point on the preceding vehicle. This further highlights the instability of the TTC estimation in certain frames.

The observed discrepancies can be attributed to to several factors which include the following:-

        i) Motion Model Used : The TTC computation assumes a 'Constant Velocity Model'. On the event of any possible acceleration or decceleration by either the  ego vehicle or by preceding vehicle, this model fails to compute the correct TTC.


        ii)Distance calculation Strategy: The TTC algorithm relys on Median distance of LiDAR points within the bounding box to make it robust against outliers. If only a few points are close to the rear of the vehicle and all other points are lying farrzther away then the median distance will also be larger than true distance , resulting in artificially increased TTC.


FP.6)
