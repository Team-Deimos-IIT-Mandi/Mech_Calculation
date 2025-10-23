# 🛠️ Mars Rover Steering Mechanism – Worm Gear System (Design A)

## 📘 Overview
This repository contains the design details and calculation parameters for **Design A**, a **non-backdrivable worm gear system** developed for a **Mars Rover steering mechanism**.  
The system is optimized for **high torque and low-speed operation**, ensuring precise control and resistance to external backdriving forces during uneven terrain navigation.

---

## ⚙️ System Requirements

| Parameter | Description | Value |
|------------|-------------|-------|
| **Application** | Mars Rover steering mechanism | — |
| **Input Speed** | From motor | 470 RPM |
| **Input Torque** | From motor | 8.5 kg·cm |
| **Required Output Torque** | At wheel hub | 200 kg·cm |
| **Max Gearbox Width** | Limited by hub casing | ≤ 75 mm |
| **Output Shaft Diameter** | Connected to wheel hub | 12 mm |
| **Duty Cycle** | Low (≤ 1 hour intermittent operation) | — |
| **Environment** | Prototype testing (terrestrial conditions) | — |

---

## ⚙️ Design Objectives

- Achieve **high torque multiplication** from a compact gearbox.
- Ensure **non-backdrivability** for stability in parked or stationary rover states.
- Maintain **lightweight design** without compromising strength.
- Enable **easy integration** with wheel hub and steering linkages.
- Operate reliably under **intermittent load** with minimal heat generation.

---

## 🧮 Design Parameters

| Parameter | Symbol | Value | Notes |
|------------|---------|--------|-------|
| **Worm Gear Ratio** | i | ≈ 24:1 | To reach 200 kg·cm output torque |
| **Worm Type** | — | Single-start, right-hand thread | Ensures non-backdrivable configuration |
| **Worm Material** | — | Case-hardened steel | High wear resistance |
| **Gear Material** | — | Phosphor bronze | Good frictional compatibility |
| **Module (m)** | — | 2 mm | Compact size for 75 mm width constraint |
| **Number of Worm Starts (z₁)** | — | 1 | For self-locking design |
| **Number of Gear Teeth (z₂)** | — | 24 | Matches ratio requirement |
| **Lead Angle (λ)** | — | ~4° | Below critical angle for non-backdrivability |
| **Center Distance (a)** | — | ≈ 40–45 mm | Derived from module and tooth count |
| **Efficiency (η)** | — | ~0.6 (60%) | Acceptable due to self-locking |
| **Output Speed** | n₂ | ≈ 19.6 RPM | For controlled steering motion |
| **Output Torque** | T₂ | ≈ 204 kg·cm | Meets torque requirement |
| **Lubrication** | — | Grease-based (EP type) | For low-duty applications |
| **Mounting** | — | Flange or face mount | Direct hub connection |
| **Bearing Type** | — | Tapered roller or deep groove | To withstand thrust loads |

---

## 📊 Derived Calculations

1. **Speed Reduction:**
   \[
   i = \frac{n_1}{n_2} = \frac{470}{19.6} \approx 24
   \]

2. **Torque Amplification:**
   \[
   T_2 = T_1 \times i \times \eta = 8.5 \times 24 \times 0.6 \approx 204\ \text{kg·cm}
   \]

3. **Lead Angle (λ):**
   \[
   \tan{\lambda} = \frac{l}{\pi \times d_w}
   \]
   where *l* = lead = π × m × z₁, and *d_w* ≈ 25 mm  
   → λ ≈ 4°, ensuring **non-backdrivability** (λ < friction angle).

---

## 🧰 Construction Notes

- **Housing Material:** Aluminum alloy for lightweight structure.
- **Shaft Diameter:** 12 mm output (matches wheel hub interface).
- **Bearing Support:** Double bearing on worm shaft for stability.
- **Cover & Seals:** Dust-sealed casing (IP54 equivalent for prototype use).
- **Mounting Interface:** Compact design to fit ≤ 75 mm hub width.

---

## 🧩 Integration Notes

- Worm output shaft directly couples with **wheel hub**.
- Worm shaft connected to **470 RPM, 8.5 kg·cm motor** via flexible coupling.
- Designed for **slow, precise angular steering** (< 20 RPM output).
- Self-locking feature ensures **zero backdrive** under external wheel load.

---

## 📐 CAD and Simulation

| File Type | Description |
|------------|-------------|
| `worm_gear_assembly.SLDASM` | SolidWorks assembly |
| `worm.SLDPRT` | Worm component model |
| `worm_wheel.SLDPRT` | Worm wheel model |
| `housing.SLDPRT` | Gearbox casing |
| `analysis.FEA` | Structural and contact stress analysis |

---

## 🧾 Future Improvements

- Introduce **dual-start worm** with optimized lead angle for higher efficiency if needed.
- Conduct **FEA validation** for housing and tooth stress.
- Evaluate **bronze-aluminum composite** for weight reduction.
- Integrate **encoder mount** for precise steering angle feedback.

---

## 🧪 References

- Machine Design Data Book by PSG College of Technology  
- Khurmi & Gupta – *Theory of Machines*  
- AGMA Standards for Worm Gearing  
- MIT Mechanical Design Notes on Non-Backdrivable Mechanisms

---

## 🛰️ License
This design is open for research and educational use under the **MIT License**.

---

**Author:** Rihaansh Saraswat  
**Last Updated:** October 2025  
