## 🔌 Electronics & Schematic Overview

The 4-DOF Robotic Arm is powered and controlled via a custom circuit shield topology designed around the **Arduino Uno R3**. The schematic is structured to safely handle high-current inductive loads (servo motors) without disrupting the microcontroller's logic power.


---

### 🏗️ Subsystem Breakdown

#### 1. Power Supply Unit (PSU)
* **DC Input:** 7V–12V DC via a standard $2.1\text{mm}$ barrel jack (`J1`).
* **Reverse Polarity Protection:** A high-current Schottky Diode (`D1`, 1N5822) prevents reverse voltage damage.
* **Regulation:** An onboard LM7805 / DC-DC Buck Converter (`U2`) drops the main line down to a stable $5\text{V}$ rail.
* **Fusing & Filtering:** * Main $5\text{A}$ fuse (`F1`) protects the main input line.
  * $3\text{A}$ resettable PTC Polyfuse (`F2`) protects the servo supply rail.
  * $100\,\mu\text{F}$ electrolytic capacitor (`C1`) buffers sudden current surges.

#### 2. Microcontroller Core (Arduino Uno)
* Powered via the `VIN` pin directly from the protected main power line.
* Logic and PWM signal pins drive the servo motors independently.
* **Common Ground:** Standard star-ground layout guarantees a reliable signal reference between the logic and power domains.

#### 3. Servo Output Channels
To protect the MCU GPIO pins from current spikes and back-EMF, each channel incorporates a **$220\,\Omega$ damping resistor** in series with the PWM signal, alongside dedicated local decoupling capacitors ($10\,\mu\text{F}$ and $0.1\,\mu\text{F}$) positioned next to each 3-pin connector.

| Joint / Function | Servo Connector | Arduino PWM Pin | Net Name | Inline Protection | Decoupling |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Base Joint** | `J2` | Digital `D3` | `PWM_BASE` | $220\,\Omega$ ($R_2$) | $10\,\mu\text{F}$ + $0.1\,\mu\text{F}$ |
| **Shoulder Joint** | `J3` | Digital `D5` | `PWM_SHOULDER` | $220\,\Omega$ ($R_3$) | $10\,\mu\text{F}$ + $0.1\,\mu\text{F}$ |
| **Elbow Joint** | `J4` | Digital `D6` | `PWM_ELBOW` | $220\,\Omega$ ($R_4$) | $10\,\mu\text{F}$ + $0.1\,\mu\text{F}$ |
| **Gripper Axis** | `J5` | Digital `D9` | `PWM_GRIPPER` | $220\,\Omega$ ($R_5$) | $10\,\mu\text{F}$ + $0.1\,\mu\text{F}$ |

---

### ⚠️ Powering & Wiring Best Practices

1. **Do NOT power servos from the Arduino 5V pin:** Servo motors draw significant stall current (up to $1\text{A}$–$1.5\text{A}$ each under load). Always use the dedicated `5V_SERVO` bus fed by the regulator.
2. **Capacitor Rating:** Ensure all electrolytic capacitors are rated for **$16\text{V}$ or higher**.
3. **Power Indicator:** The onboard green LED (`LED1`) signals when the $5\text{V}$ regulated bus is active.

---

### 📑 Document Revision History

* **Project:** 4-DOF Robotic Arm
* **Drawing No:** SK001
* **Revision:** 1.2
* **Date:** JUL 2026
* **Designer:** D. EZE
