# ESP32_S3_Tiny_Quad_Flight_Controller
This is repository for ESP32-S3 Super Mini Tiny Quad Flight Controller Drone

## Components For Drone

| **Components**                                                                               | **Quantity** |
|----------------------------------------------------------------------------------------------|--------------|
| ESP32 S3 Super Mini                                                                          | 1            |
| MPU6050                                                                                      | 1            |
| ESC Motor Driver 1. IRLML6344TRPBF SOT23 (MOS) 2. IN4148WS T4 SOD123 (Diode) 3. 10K 0805 SMD | 2            |
| Flysky FS-A8S 2.4G 8CH                                                                       | 1            |
| 8520 Tello Motor                                                                             | 4            |
| Jst socket red Female                                                                        | 1            |
| Jumper T-Lite V2 transmitter                                                                 | 1            |
| Lithium 3.7V 200mAH battery  (6x2.5cm) JST Socket Red                                        | 1            |
| DJI Tello Propeller                                                                          | 4            |
| VL53L0X V2 TOF                                                                               | 1            |

For Altitude Hold, please upload below flight controller code to your ESP32-S3
```
ESP32_S3_Tiny_Quad_anglemode_kalman_filter_pid_arm_disarm_logic_alt_hold_v1
```

Then on your controller, toogle `SWD` to enable altitude hold. By default the drone is in angle mode
