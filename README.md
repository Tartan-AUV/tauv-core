# tauv_core ROS 2 Nodes

## Overview

The `tauv_core` package contains nodes that convert raw sensor measurements into standard ROS 2 message types suitable for **state estimation, navigation, and sensor fusion**.

Currently included nodes:

1. **DepthConverter** – Converts depth measurements into odometry (`nav_msgs::msg::Odometry`).
2. **DvlConverter** – Converts Doppler Velocity Log (DVL) data into velocity messages (`geometry_msgs::msg::TwistWithCovarianceStamped`).
3. **ImuConverter** – Adds covariance to IMU data (`sensor_msgs::msg::Imu`) for use in EKFs or other fusion pipelines.

---

## Nodes

### 1. DepthConverter

**Purpose:**
Converts depth sensor readings into Z-position in an odometry message.

**Subscribed Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `/vehicle/depth` | `tauv_msgs::msg::Depth` | Raw depth sensor measurements |

**Published Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `<prefix>/sensors/depth` | `nav_msgs::msg::Odometry` | Z-position converted for EKF/state estimation |

**Features:**

- Converts depth (positive downward) to ROS Z-axis (positive upward)
- Sets high covariance for unmeasured axes
- Uses depth sensor variance for Z covariance
