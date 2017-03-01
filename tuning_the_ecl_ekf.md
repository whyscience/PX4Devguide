# 使用 ecl EKF

本节教程回答了使用 ECL EKF 算法相关的常见问题。

## 什么是 ecl EKF?

ECL \(Estimation and Control Library，估计和控制库\) 使用扩展卡尔曼滤波算法来处理传感器测量值并提供以下状态的估计:

* 四元数，定义了从North（北），East（东），Down（地）地球坐标系到X，Y，Z机体坐标系的旋转。
* IMU中的速度 - North,East,Down \(m/s\)
* IMU中的位置 - North,East,Down \(m\)
* IMU 角度增量偏差（bias）估计 - X,Y,Z \(rad\)
* IMU 速度增量偏差（bias）估计 - X,Y,Z\(m/s\)
* 地球磁场分量 - North,East,Down \(gauss\)
* 机体坐标系磁场偏差 - X,Y,Z \(gauss\)
* 风速 - North,East \(m/s\)

EKF在一个延迟的‘融合时间范围（fusion time horizon）’内工作，这是考虑到涉及到IMU的每个测量的时间延迟不同。每个传感器的数据是以FIFO（先进先出）形式缓存的，然后被EKF取出在正确的时间使用。EKF2\_\*\_DELAY 控制了每个传感器的延时补偿。

互补滤波用于将状态从‘融合时间范围’向当前时间推进，它使用IMU的数据。互补滤波的时间常数由参数EKF2\_TAU\_VEL 和 EKF2\_TAU\_POS控制。

注意：‘融合时间范围’的延迟和缓存的长度由 EKF2\_\*\_DELAY 中的最大值决定。如果一个传感器没有被使用，推荐将其时间延迟设置为0。降低‘融合时间范围’的延迟就降低了互补滤波中用于将状态推荐到当前时间的误差（error）。

位置和速度状态在被输出至控制回路之前，需要调整以说明IMU和机体坐标系的偏移（offset）。IMU相对于机体坐标系的位置可由参数 EKF2\_IMU\_POS\_X,Y,Z 设置。

EKF使用IMU仅用于状态预测。在EKF推到中IMU没有被用作观测量。协方差预测、状态更新和协方差更新的代数方程由 Matlab symbolic toolbox 生成，可以在这里找到: [Matlab Symbolic Derivation](https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m)

## 使用了什么传感器测量量？

EKF包含了不同的操作模式，考虑到了不同的传感器测量值得组合。在启动时，滤波器检测最小可用传感器组合，在初始倾斜、偏航和高度校准完成后，进入了一个提供旋转、垂直速度、垂直位置、IMU角度增量偏差和IMU速度增量偏离估计的模式。

这个模式需要IMU数据、一个偏航数据来源\(磁力计或者外部视觉信息\)和一个高度数据来源。所有EKF操作模式都需要这个最小数据集。其他传感器数据可用于估计其他的状态。

### IMU

* 机体三轴固连的惯性测量单元（Inertial Measurement unit）以最低100Hz提供角度增量和速度增量的数据。注意：在IMU角度增量数据被EKF使用之前应进行锥进修正。

### 磁力计

三轴机体固连的磁力计数据（或者外部视觉系统姿态数据）要求至少以5Hz更新。磁力计数据可用以下两种方式使用：

* 使用倾斜估计和磁力计偏差可将磁力计测量值转换为偏航角。这个偏航角接着被EKF用作观测量。这个方法更不准确而且没有考虑到机体坐标系磁场偏移，但是它对磁场异常和启动时大的陀螺仪偏差更加健壮（robust）。这是在启动时和在地面上时使用的默认方法。
* XYZ 磁力计读数被用做独立的观测量。这个方法更加准确而且允许习得机体坐标系磁场偏移，但是需假设地球磁场环境缓慢变化而且在有明显外部磁场干扰的时候表现欠佳。飞机在空中或者爬升超过1.5m高度之后默认使用这个方法。

选择模式的逻辑由参数 EKF2\_MAG\_TYPE 设置。

### Height

A source of height data - either GPS, barometric pressure, range finder or external vision at a minimum rate of 5Hz is required. Note: The primary source of height data is controlled by the EKF2\_HGT\_MODE parameter.

If these measurements are not present, the EKF will not start. When these measurements have been detected, the EKF will initialise the states and complete the tilt and yaw alignment. When tilt and yaw alignment is complete, the EKF can then transition to other modes of operation  enabling use of additional sensor data:

### GPS

GPS measurements will be used for position and velocity if the following conditions are met:

* GPS use is enabled via setting of the EKF2\_AID\_MASK parameter.
* GPS quality checks have passed. These checks are controlled by the EKF2\_GPS\_CHECK and EKF2\_REQ&lt;&gt; parameters. 
* GPS height can be used directly by the EKF via setting of the EKF2\_HGT\_MODE parameter.

### Range Finder

Range finder distance to ground is used a by a single state filter to estimate the vertical position of the terrain relative to the height datum.

If operating over a flat surface that can be used as a zero height datum, the range finder data can also be used directly by the EKF to estimate height by setting the EKF2\_HGT\_MODE parameter to 2.

### Airspeed

Equivalent Airspeed \(EAS\) data can be used to estimate wind velocity and reduce drift when GPS is lost by setting EKF2\_ARSP\_THR to a positive value. Airspeed data will be used when it exceeds the threshold set by a positive value for EKF2\_ARSP\_THR and the vehicle type is not rotary wing.

### Synthetic Sideslip

Fixed wing platforms can take advantage of an assumed sideslip observation of zero to improve wind speed estimation and also enable wind speed estimation without an airspeed sensor. This is enabled by setting the EKF2\_FUSE\_BETA parameter to 1.

### Optical Flow

Optical flow data will be used if the following conditions are met:

* Valid range finder data is available.
* Bit position 1 in the EKF2\_AID\_MASK parameter is true.
* The quality metric returned by the flow sensor is greater than the minimum requirement set by the EKF2\_OF\_QMIN parameter

### External Vision System

Position and Pose Measurements from an external vision system, eg Vicon, can be used:

* External vision system horizontal position data will be used if bit position 3 in the EKF2\_AID\_MASK parameter is true.
* External vision system vertical position data will be used if the EKF2\_HGT\_MODE parameter is set to 3.
* External vision system pose data will be used for yaw estimation if bit position 4 in the EKF2\_AID\_MASK parameter is true.

## How do I use the 'ecl' library EKF?

Set the SYS\_MC\_EST\_GROUP parameter to 2 to use the ecl EKF.

## What are the advantages and disadvantages of the ecl EKF over other estimators?

Like all estimators, much of the performance comes from the tuning to match sensor characteristics. Tuning is a compromise between accuracy and robustness and although we have attempted to provide a tune that meets the needs of most users, there will be applications where tuning changes are required.

For this reason, no claims for accuracy relative to the legacy combination of attitude\_estimator\_q + local\_position\_estimator have been made and the best choice of estimator will depend on the application and tuning.

### Disadvantages

* The ecl EKF is a complex algorithm that requires a good understanding of extended Kalman filter theory and its application to navigation problems to tune successfully. It is therefore more difficult for users that are not achieving good results to know what to change.
* The ecl EKF uses more RAM and flash space
* The ecl EKF uses more logging space.
* The ecl EKF has had less flight time

### Advantages

* The ecl EKF is able to fuse data from sensors with different time delays and data rates in a mathematically consistent way which improves accuracy during dynamic manoeuvres once time delay parameters are set correctly.
* The ecl EKF is capable of fusing a large range of different sensor types.
* The ecl EKF detects and reports statistically significant inconsistencies in sensor data, assisting with diagnosis of sensor errors.
* For fixed wing operation, the ecl EKF estimates wind speed with or without an airspeed sensor and is able to use the estimated wind in combination with airspeed measurements and sideslip assumptions to extend the dead-reckoning time available if GPS is lost in flight.
* The ecl EKF estimates 3-axis accelerometer bias which improves accuracy for tailsitters and other vehicles that experience large attitude changes between flight phases.
* The federated architecture \(combined attitude and position/velocity estimation\) means that attitude estimation benefits from all sensor measurements. This should provide the potential for improved attitude estimation if tuned correctly. 

## How do I check the EKF performance?

EKF outputs, states and status data are published to a number of uORB topics which are logged to the SD card during flight. The following guide assumes that data has been logged using the .ulog file format. To use the .ulog format, set the SYS\_LOGGER parameter to 1.

The .ulog format data can be parsed in python by using the [PX4 pyulog library](https://github.com/PX4/pyulog).

Most of the EKF data is found in the [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg) and [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) uORB messages that are logged to the .ulog file.

### Output Data

* Attitude output data is found in the [vehicle\_attitude](https://github.com/PX4/Firmware/blob/master/msg/vehicle_attitude.msg) message.
* Local position output data is found in the [vehicle\_local\_position](https://github.com/PX4/Firmware/blob/master/msg/vehicle_local_position.msg) message.
* Control loop feedback data is found in the the [control\_state](https://github.com/PX4/Firmware/blob/master/msg/control_state.msg) message.
* Global \(WGS-84\) output data is found in the [vehicle\_global\_position](https://github.com/PX4/Firmware/blob/master/msg/vehicle_global_position.msg) message.
* Wind velocity output data is found in the [wind\_estimate](https://github.com/PX4/Firmware/blob/master/msg/wind_estimate.msg) message.

### States

Refer to states\[32\] in [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg). The index map for states\[32\] is as follows:

* \[0 ... 3\] Quaternions
* \[4 ... 6\] Velocity NED \(m/s\)
* \[7 ... 9\] Position NED \(m\)
* \[10 ... 12\] IMU delta angle bias XYZ \(rad\)
* \[13 ... 15\] IMU delta velocity bias XYZ \(m/s\)
* \[16 ... 18\] Earth magnetic field NED \(gauss\)
* \[19 ... 21\] Body magnetic field XYZ \(gauss\)
* \[22 ... 23\] Wind velocity NE \(m/s\)
* \[24 ... 32\] Not Used

### State Variances

Refer to covariances\[28\] in [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg). The index map for covariances\[28\] is as follows:

* \[0 ... 3\] Quaternions
* \[4 ... 6\] Velocity NED \(m/s\)^2
* \[7 ... 9\] Position NED \(m^2\)
* \[10 ... 12\] IMU delta angle bias XYZ \(rad^2\)
* \[13 ... 15\] IMU delta velocity bias XYZ \(m/s\)^2
* \[16 ... 18\] Earth magnetic field NED \(gauss^2\)
* \[19 ... 21\] Body magnetic field XYZ \(gauss^2\)
* \[22 ... 23\] Wind velocity NE \(m/s\)^2
* \[24 ... 28\] Not Used

### Observation Innovations

* Magnetometer XYZ \(gauss\) : Refer to mag\_innov\[3\] in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Yaw angle \(rad\) : Refer to heading\_innov in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Velocity and position innovations : Refer to vel\_pos\_innov\[6\] in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg). The index map for vel\_pos\_innov\[6\] is as follows:
  * \[0 ... 2\] Velocity NED \(m/s\)
  * \[3 ... 5\] Position NED \(m\)
* True Airspeed \(m/s\) : Refer to airspeed\_innov in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Synthetic sideslip \(rad\) : Refer to beta\_innov in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Optical flow XY \(rad/sec\) : Refer to flow\_innov in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Height above ground \(m\) : Refer to hagl\_innov in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).

### Observation Innovation Variances

* Magnetometer XYZ \(gauss^2\) : Refer to mag\_innov\_var\[3\] in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Yaw angle \(rad^2\) : Refer to heading\_innov\_var in the ekf2\_innovations message.
* Velocity and position innovations : Refer to vel\_pos\_innov\_var\[6\] in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg). The index map for vel\_pos\_innov\_var\[6\] is as follows:
  * \[0 ... 2\] Velocity NED \(m/s\)^2
  * \[3 ... 5\] Position NED \(m^2\)
* True Airspeed \(m/s\)^2 : Refer to airspeed\_innov\_var in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Synthetic sideslip \(rad^2\) : Refer to beta\_innov\_var in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Optical flow XY \(rad/sec\)^2 : Refer to flow\_innov\_var in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).
* Height above ground \(m^2\) : Refer to hagl\_innov\_var in [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).

### Output Complementary Filter

The output complementary filter is used to propagate states forward from the fusion time horizon to current time. To check the magnitude of the angular, velocity and position tracking errors measured at the fusion time horizon, refer to output\_tracking\_error\[3\] in the ekf2\_innovations message. The index map is as follows:

* \[0\] Angular tracking error magnitude \(rad\)
* \[1\] Velocity tracking error magnitude \(m/s\). The velocity tracking time constant can be adjusted using the EKF2\_TAU\_VEL parameter. Reducing this parameter reduces steady state errors but increases the amount of observation noise on the NED velocity outputs.
* \[2\] Position tracking error magnitude \(m\). The position tracking time constant can be adjusted using the EKF2\_TAU\_POS parameter. Reducing this parameter reduces steady state errors but increases the amount of observation noise on the NED position outputs.

### EKF Errors

The EKF contains internal error checking for badly conditioned state and covariance updates. Refer to the filter\_fault\_flags in [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).

### Observation Errors

There are two categories of observation faults:

* Loss of data. An example of this is a range finder failing to provide a return.
* The innovation, which is the difference between the state prediction and sensor observation is excessive. An example of this is excessive vibration causing a large vertical position error, resulting in the barometer height measurement being rejected.

Both of these can result in observation data being rejected for long enough to cause the EKF to attempt a reset of the states using the sensor observations. All observations have a statistical confidence check applied to the innovations. The number of standard deviations for the check are controlled by the EKF2\_&lt;&gt;\_GATE parameter for each observation type.

Test levels are  available in [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) as follows:

* mag\_test\_ratio : ratio of the largest magnetometer innovation component to the innovation test limit
* vel\_test\_ratio : ratio of the largest velocity innovation component to the innovation test limit
* pos\_test\_ratio : ratio of the largest horizontal position innovation component to the innovation test limit
* hgt\_test\_ratio : ratio of the vertical position innovation to the innovation test limit
* tas\_test\_ratio : ratio of the true airspeed innovation to the innovation test limit
* hagl\_test\_ratio : ratio of the height above ground innovation to the innovation test limit

For a binary pass/fail summary for each sensor, refer to innovation\_check\_flags in [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).

### GPS Quality Checks

The EKF applies a number of GPS quality checks before commencing GPS aiding. These checks are controlled by the EKF2\_GPS\_CHECK and EKF2\_REQ&lt;&gt; parameters. The pass/fail status for these checks is logged in the [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).gps\_check\_fail\_flags message. This integer will be zero when all required GPS checks have passed. If the EKF is not commencing GPS alignment, check the value of the integer against the bitmask definition gps\_check\_fail\_flags in [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).

### EKF Numerical Errors

The EKF uses single precision floating point operations for all of its computations and first order approximations for derivation of the covariance prediction and update equations in order to reduce processing requirements. This means that it is possible when re-tuning the EKF to encounter conditions where the covariance matrix operations become badly conditioned enough to cause divergence or significant errors in the state estimates.

To prevent this, every covariance and state update step contains the following error detection and correction steps:

* If the innovation variance is less than the observation variance \(this requires a negative state variance which is impossible\) or the covariance update will produce a negative variance for any of the states, then:
  * The state and covariance update is skipped
  * The corresponding rows and columns in the covariance matrix are reset
  * The failure is recorded in the [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg) filter\_fault\_flags message
* State variances \(diagonals in the covariance matrix\) are constrained to be non-negative.
* An upper limit is applied to state variances.
* Symmetry is forced on the covariance matrix.

After re-tuning the filter, particularly re-tuning that involve reducing the noise variables,  the value of estimator\_status.gps\_check\_fail\_flags should be checked to ensure that it remains zero.

## What should I do if the height estimate is diverging?

The most common cause of EKF height diverging away from GPS and altimeter measurements during flight is clipping and/or aliasing of the IMU measurements caused by vibration. If this is occurring, then the following signs should be evident in the data

* [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).vel\_pos\_innov\[3\] and  [ekf2\_innovations](https://github.com/PX4/Firmware/blob/master/msg/ekf2_innovations.msg).vel\_pos\_innov\[5\] will both have the same sign.
* [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).hgt\_test\_ratio will be greater than 1.0

The recommended first step is to  esnure that the autopilot is isolated from the airframe using an effective isolatoin mounting system. An isolaton mount has 6 degrees of freedom, and therefore 6 resonant frequencies. As a general rule, the 6 resonant frequencies of the autopilot on the isolation mount should be above 25Hz to avoid interaction with the autopilot dynamics and below the frequency of the motors.

An isolation mount can make vibration worse if the resonant frequncies coincide with motor or propeller blade passage frequencies.

The EKF can be made more resistant to vibration induced height divergence by making the following parameter changes:

* Double the value of the innovation gate for the primary height sensor. If using barometeric height this is EK2\_EKF2\_BARO\_GATE.
* Increase the value of EKF2\_ACC\_NOISE to 0.5 initially. If divergence is still occurring,   increase in further increments of 0.1 but do not go above 1.0

Note that the effect of these changes will make the EKF more sensitive to errors in GPS vertical velocity and barometric pressure.

## What should I do if the position estimate is diverging?

The most common causes of position divergence are:

* High vibration levels. 
  * Fix by improving mechanical isolation of the autopilot.
  * Increasing the value of EKF2\_ACC\_NOISE and EKF2\_GYR\_NOISE can help, but does make the EKF more vulnerable to GPS glitches.
* Large gyro bias offsets. 
  * Fix by re-calibrating the gyro. Check for excessive temperature sensitivity \(&gt; 3 deg/sec bias change during warm-up from a cold start and replace the sensor if affected of insulate to to slow the rate of temeprature change.
* Bad yaw alignment
  * Check the magntometer calibration and alignment.
  * Check the heading shown QGC is within within 15 deg truth
* Poor GPS accuracy
  * Check for interference
  * Improve separation and shielding
  * Check flying location for GPS signal obstructions and reflectors \(nearboy tall buildings\)
* Loss of GPS

Determining which of these is the primary cause requires a methodical approach to analysis of the EKF log data:

* Plot the velocty innovation test ratio - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vel\_test\_ratio
* Plot the horizontal position innovation test ratio - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).pos\_test\_ratio
* Plot the height innovation test ratio - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).hgt\_test\_ratio
* Plot the magnetoemrer innovation test ratio - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).mag\_test\_ratio
* Plot the GPS receier reported speed accuracy - [vehicle\_gps\_position](https://github.com/PX4/Firmware/blob/master/msg/vehicle_gps_position.msg).s\_variance\_m\_s
* Plot the IMU delta angle state estimates - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).states\[10\], states\[11\] and states\[12\]
* Plot the EKF internal high frequency vibration metrics:
  * Delta angle coning vibration - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[0\]
  * High frequency delta angle vibration - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[1\]
  * High frequency delta velocity vibration - [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vibe\[2\]

During normal operation, all the test ratios should remain below 0.5 with only occasional spikes above this as shown in the example below from a successful flight:  
![Position, Velocity, Height and Magnetometer Test Ratios](Screen Shot 2016-12-02 at 9.20.50 pm.png)  
The following plot shows the EKF vibration metrics for a multirotor with good isolation. The landing shock and the increased vibration during takeoff and landing can be seen. Insufficient data has been gathered with these metrics to provide specific advice on maximum thresholds.  
![](Screen Shot 2016-12-02 at 10.24.00 pm.png)  
The above vibration metrics are of limited value as the presence of vibration at a frequency close to the IMU sampling frequency \(1 kHz for most boards\) will cause  offsets to appear in the data that do not show up in the high frequency vibration metrics. The only way to detect aliasing errors is in their effect on inertial navigation accuracy and the rise in innovation levels.

In addition to generating large position and velocity test ratios of &gt; 1.0, the different error mechanisms affect the other test ratios in different ways:

### Determination of Excessive Vibration

High vibration levels normally affect vertical position and velocity innovations as well as the horizontal components. Magnetometer test levels are only affected to a small extent.

\(insert example plots showing bad vibration here\)

### Determination of Excessive Gyro Bias

Large gyro bias offsets are normally characterised by a change in the value of delta angle bias greater than 5E-4 during flight \(equivalent to ~3 deg/sec\) and can also cause a large increase in the magnetometer test ratio if the yaw axis is affected. Height is normally unaffected other than extreme cases. Switch on bias value of up to 5 deg/sec can be tolerated provided the filter is given time time settle before flying . Pre-flight checks performed by the commander should prevent arming if the position is diverging.

\(insert example plots showing bad gyro bias here\)

### Determination of Poor Yaw Accuracy

Bad yaw alignment causes a velocity test ratio that increases rapidly when the vehicle starts moving due inconsistency in the direction of velocity calculated by the inertial nav and the  GPS measurement. Magnetometer innovations are slightly affected. Height is normally unaffected.

\(insert example plots showing bad yaw alignment here\)

### Determination of Poor GPS Accuracy

Poor GPS accuracy is normally accompanied by a rise in the reported velocity error of the receiver in conjunction with a rise in innovations. Transient errors due to multipath, obscuration and interference are more common causes. Here is an example of a temporary loss of GPS accuracy where the multi-rotor started drifting away from its loiter location and had to be corrected using the sticks. The rise in [estimator\_status](https://github.com/PX4/Firmware/blob/master/msg/estimator_status.msg).vel\_test\_ratio to greater than 1 indicates the GPs velocity was inconsistent with other measurements and has been rejected.

![](gps glitch - test ratios.png)

This is accompanied with rise in the GPS receivers reported velocity accuracy which indicates that it was likely a GPS error.  
![](gps glitch - reported receiver accuracy.png)

If we also look at the GPS horizontal velocity innovations and innovation variances, we can see the large spike in North velocity innovation that accompanies this GPS 'glitch' event.  
![](gps glitch - velocity innovations.png)

### Determination of GPS Data Loss

Loss of GPS data will be shown by the velocity and position innvoation test ratios 'flat-lining'. If this occurs, check the oher GPS status data in vehicle\_gps\_position for further information.

\(insert example plots showing loss of GPS data here\)

