# 📡 Utility-Driven Intelligent RF–VLC link Aggregated Link Selection

_A machine learning-based framework for adaptive link selection in vehicular communication systems using RF, VLC, and link Aggregated links under varying weather conditions._

---

## 📌 Table of Contents
- Overview  
- Problem Statement  
- Dataset  
- Tools & Technologies  
- Project Structure  
- Methods  
- Algorithms Used  
- Evaluation Metrics  
- Key Insights  
- Output & Visualizations  
- How to Run This Project  
- Results & Conclusion  
- Future Work  
- Learnings  

---

## 📖 Overview
This project proposes a **link Aggregated RF–VLC communication system** for intelligent transportation systems (ITS). It uses **machine learning and physics-based modeling** to dynamically select the best communication mode:

- RF (Radio Frequency)
- VLC (Visible Light Communication)
- Link-Aggregated ( RF + VLC)

The system optimizes performance using a **multi-objective utility function** considering:
- Throughput
- Delay
- Outage probability :contentReference[oaicite:1]{index=1}  

---

## ❗ Problem Statement
Traditional communication systems suffer from:

- RF → interference, congestion, fading  
- VLC → sensitivity to fog, blockage, alignment  

This project aims to:
- Dynamically select the best communication link  
- Improve reliability and latency  
- Handle weather impairments (rain, fog)  
- Optimize overall system utility  

---

## 📊 Dataset
- **Total Samples:** 60,000 (Monte Carlo simulation)  
- **Generated Using:** MATLAB  

### Features:
- Distance between vehicles  
- Vehicle speed & relative speed  
- Vehicle density  
- Weather class (Clear, Rain, Fog)  
- Fog attenuation coefficient (β)  
- Rain rate  
- Line-of-Sight (LOS)  

### Target:
- `link_label`
  - 1 → RF  
  - 2 → VLC  
  - 3 → link Aggregated  

---

## 🛠 Tools & Technologies
- MATLAB (Dataset generation)  
- Python (ML modeling)  
- Scikit-learn  
- Random Forest, Decision Tree, XGBoost  
- Data Visualization  

---

## 📁 Project Structure
<img width="6764" height="1095" alt="image" src="https://github.com/user-attachments/assets/ae502261-53a9-4e25-8a7d-5feecee7b7aa" />

---

## ⚙️ Methods

### 🔹 Dataset Generation
- Poisson distribution for vehicle spacing  
- Weather simulation (Clear, Rain, Light Fog, Dense Fog)  
- Mobility modeling  

### 🔹 Channel Modeling
#### RF:
- Path loss + shadowing  
- Rain attenuation  
- Doppler fading  
- Co-channel interference  

#### VLC:
- Lambertian optical model  
- Fog attenuation (Beer–Lambert law)  
- LOS blockage  
- Alignment jitter  

### 🔹 Hybrid Link
- Load splitting based on capacity  
- Delay optimization  
- Packet imbalance penalty  
- MAC overhead consideration  

### 🔹 Utility Function
U = wC * Capacity - wD * Delay - wO * Outage


Optimal weights:

(wC, wD, wO) = (0.7, 0.2, 0.1)

---

## 🤖 Algorithms Used
- Random Forest (Primary model)  
- Decision Tree  
- XGBoost  

- Grid Search with 10-fold Cross Validation  
- Utility-based labeling strategy  

---

## 📊 Evaluation Metrics
- Accuracy  
- Precision  
- Recall  
- F1-score  
- Average Utility Score  

---

## 🔍 Key Insights
- link Aggregated link consistently outperforms RF and VLC individually  
- VLC performs best in clear conditions  
- RF dominates under fog and high mobility  
- link Aggregated adapts dynamically → best overall performance  
- Fog attenuation and distance are most important features :contentReference[oaicite:2]{index=2}  

---

## 📈 Output & Visualizations
- Utility vs Weather Conditions  
- Utility vs Fog Density  
- Utility vs Distance  
- Delay vs Distance (log scale)  
- Correlation Matrix  
- Feature Importance  

---

## ▶️ How to Run This Project

1️⃣ Clone the Repository

git clone https://github.com/AlokSinghYadav/utility_driven_intelligent_link_selection.git

cd link Aggregated-rf-vlc

2️⃣ Install Dependencies

pip install pandas numpy scikit-learn

3️⃣ Generate Dataset (MATLAB)

Open MATLAB

Run the script:

utility_link_dataset_generation.m

This will generate:

Final_RF_VLC_HYBRID_OPTIMIZED.csv

4️⃣ Run Machine Learning Notebook

jupyter notebook utility_link.ipynb

---
📊 Results & Conclusion

Random Forest Accuracy: ~89.7%

XGBoost Accuracy: ~92.9%

Conclusion:

link Aggregated RF–VLC system improves:

Reliability

Latency

Overall communication performance

Achieves better utility across all weather conditions

---

🚀 Future Work

Integrate real-world vehicular datasets

Apply deep learning models

Deploy real-time edge-based system

Extend to 5G / 6G V2X communication

---
📚 Learnings
Wireless communication modeling

RF vs VLC channel behavior

Physics-aware simulation

Multi-objective optimization

Machine learning for network selection.
