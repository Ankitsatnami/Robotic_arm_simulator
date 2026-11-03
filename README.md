# Fanuc Cobot Kinematics Playground (6-DOF Web Simulator)

एक आधुनिक, हाई-क्वालिटी और इंटरैक्टिव 6-DOF इंडस्ट्रियल रोबोटिक आर्म (Fanuc CRX-10iA Cobot) वेब सिम्युलेटर। इसमें फॉरवर्ड एवं एनालिटिकल इनवर्स काइनेमेटिक्स (8 सॉल्यूशन्स), रियल-टाइम 4x4 होमोजेनियस ट्रांसफॉर्मेशन मैट्रिक्स, 6x6 जेकोबियन मैट्रिक्स, सिंगुलैरिटी डिटेक्शन, न्यूमेटिक ग्रिपर और लर्निंग ट्यूटोरियल्स शामिल हैं।

---

## 🌐 Live Web Demos (लाइव लिंक)
- 🚀 **GitHub Pages Live Link**: [https://ankitsatnami.github.io/Robotic_arm_simulator/](https://ankitsatnami.github.io/Robotic_arm_simulator/)
- ⚡ **Direct Cloud Live Mirror**: [https://eight-kitchen-mngsz.ship.place/](https://eight-kitchen-mngsz.ship.place/)

---

## 🚀 How to Run (कैसे चलाएं)

### विकल्प 1: सीधे ब्राउज़र में खोलें (Direct File Access)
1. अपने फाइल मैनेजर में जाएं: `c:\Ankit Satnami\Projects\Robotic Arm simulator`
2. `index.html` पर डबल-क्लिक करें। यह आपके डिफ़ॉल्ट ब्राउज़र (Chrome, Edge, Firefox) में तुरंत खुल जाएगा।

### विकल्प 2: लोकल वेब सर्वर (Recommended Local Server)
1. फ़ोल्डर में मौजूद **`start_simulator.bat`** पर डबल-क्लिक करें।
2. यह ऑटोमैटिक लोकल सर्वर शुरू करेगा और आपके ब्राउज़र में लिंक खोलेगा:
   👉 **`http://localhost:8080`**

---

## 🌟 Key Features (मुख्य विशेषताएं)

### 1. Visual Interface (बिल्कुल स्क्रीनशॉट के अनुसार)
* **Fanuc CRX-10iA Cobot Aesthetic**: ग्लॉसी वाइट एरोडायनामिक आर्म्स, सिग्नेचर ग्रीन एक्सेंट रिंग्स, मेटैलिक जॉइंट्स और सॉफ्ट शैडोज़।
* **Robot Model Selector**: 
  * Fanuc CRX-10iA (Standard Cobot)
  * Universal Robots UR5e
  * KUKA KR6 R900 (Agilus)
  * Unimation PUMA 560
* **Joint Controls (J1 – J6)**:
  * स्मूथ स्लाइडर्स (-180° से +180° / जॉइंट लिमिट्स)
  * रियल-टाइम डिग्री डिस्प्ले (जैसे स्क्रीनशॉट में: J1=-62°, J2=26°, J3=78°, J4=69°, J5=-68°, J6=75°)
  * 1-क्लिक "Screenshot Pose" बटन।
* **Cartesian Target (XYZ & WPR)**:
  * X, Y, Z (मिलीमीटर में)
  * W (Yaw), P (Pitch), R (Roll) (डिग्री में)
  * "Solve Inverse Kinematics" बटन।
* **Current Configuration**:
  * FANUC स्टैंडर्ड कॉन्फ़िगरेशन स्ट्रिंग: `Current Config: N D T, 0, 0, 0`
* **8 Available Analytical IK Solutions (इंटरैक्टिव पिल्स)**:
  * `F D T, 0, 0, 0`
  * `N D T, 0, 0, 0` (Active)
  * `F U T, 0, 0, 0`
  * `N U T, 0, 0, 0`
  * `F U B, 0, 0, 0`
  * `N U B, 0, 0, 0`
  * `F D B, 0, 0, 0`
  * `N D B, 0, 0, 0`
  *(किसी भी पिल पर क्लिक करने से रोबोट आर्म तुरंत उस कॉन्फ़िगरेशन में स्मूथली एनिमेट हो जाता है!)*

---

### 2. Degree of Freedoms (DOF) Detail
* 6-DOF का पूरा ब्रेकडाउन:
  * जॉइंट का प्रकार (Revolute)
  * फिजिकल मोशन एक्सिस (Yaw, Pitch, Roll)
  * मोशन रेंज लिमिट्स (Min / Max degrees)
  * मैक्सिमम रोटेशन स्पीड (°/s)
  * लाइव जॉइंट एंगल्स
* 3D ओवरले टॉगल:
  * जॉइंट रोटेशन रिंग्स (Axes)
  * लोकल कोऑर्डिनेट फ्रेम्स (Frames - RGB: Red=X, Green=Y, Blue=Z)
  * रीचेबल वर्कस्पेस एनवलप पॉइंट क्लाउड (Cloud)

---

### 3. Live Transformation Matrix & Jacobian Details
* **4x4 Homogeneous Transformation Matrix $[T_0^6]$**:
  $$\begin{bmatrix} r_{11} & r_{12} & r_{13} & p_x \\ r_{21} & r_{22} & r_{23} & p_y \\ r_{31} & r_{32} & r_{33} & p_z \\ 0 & 0 & 0 & 1 \end{bmatrix}$$
  * $3 \times 3$ रोटेशन सब-मैट्रिक्स $[R]$ (ब्लू कलर कोडेड)
  * $3 \times 1$ पोजीशन वेक्टर $[P]$ (ग्रीन कलर कोडेड)
* **6x6 Geometric Jacobian Matrix $[J]$**:
  * लिनियर वेलोसिटी पार्ट ($J_v$) और एंगुलर वेलोसिटी पार्ट ($J_\omega$)
  * योशिकावा मैनिपुलेबिलिटी इंडेक्स ($\mu = \sqrt{\det(J J^T)}$)
* **लाइव सिंगुलैरिटी वार्निंग HUD**:
  * **Wrist Singularity**: जब Joint 5 का एंगल 0° के करीब हो (J4 और J6 कोलिनियर हो जाते हैं)।
  * **Shoulder Singularity**: जब रिस्ट सेंटर $Z_0$ बेस एक्सिस के पास आ जाए।
  * **Boundary Singularity**: जब रोबोट आर्म अपने मैक्सिमम रीच (1249 mm) पर पूरी तरह स्ट्रेच हो जाए।

---

### 4. Learning & Educational Features
* **6 इंटरैक्टिव गाइडेड लेसन्स**:
  1. **Degrees of Freedom (DOF)**: 3D स्पेस में 6 DOFs क्यों ज़रूरी हैं।
  2. **Forward Kinematics (FK)**: जॉइंट एंगल्स से टूल पोजीशन निकालने का मैथमैटिकल फॉर्मूला।
  3. **Denavit-Hartenberg (DH) Convention**: $\theta, d, a, \alpha$ पैरामीटर्स का कॉन्सेप्ट।
  4. **Inverse Kinematics & 8 Configurations**: Flip/No-Flip, Up/Down, Top/Bottom का विवरण।
  5. **Kinematic Singularities**: सिंगुलैरिटी क्या होती है और इससे रोबोट क्यों रुकते हैं।
  6. **Motion Planning (MOVEJ vs MOVEL)**: जॉइंट-स्पेस PTP बनाम कार्टेशियन लिनियर मोशन।
* प्रत्येक लेसन में "Demonstrate in 3D" बटन दिया गया है जो सीधे रोबोट आर्म को संबंधित पोज़ में ले जाता है।

---

### 5. Trajectory Planning & Pick-and-Place Automation
* **वेपॉइंट रिकॉर्डर**: किसी भी पोज़ को वेपॉइंट लिस्ट में रिकॉर्ड करें।
* **मोशन टाइप**: `MOVEJ` (Joint Space Interpolation) या `MOVEL` (Linear Cartesian Interpolation)।
* **3D टूलपाथ ट्रेसिंग**: एंड-इफेक्टर का 3D मोशन पाथ हवा में ड्रा होता है।
* **फंक्शनल न्यूमेटिक ग्रिपर**:
  * ग्रिपर ओपन / क्लोज स्लाइडर
  * टेबल पर रखे ब्लॉक्स (Red Cube, Cyan Pin) को फिजिकली पिक और प्लेस करने का डेमो।

---

### 6. Multi-Language Code Generator
वर्तमान रोबोट पोज़ और काइनेमेटिक्स के लिए रेडी-टू-रन कोड जनरेट करता है:
* **Python**: `numpy` आधारित फॉरवर्ड काइनेमेटिक्स स्क्रिप्ट
* **FANUC Karel / TP**: ओरिजिनल टीच पेंडेंट प्रोग्राम (`J P[1] 100% FINE`, `L P[2] 250mm/sec FINE`)
* **C++ (ROS 2 / MoveIt)**: `move_group_interface` जॉइंट गोल कोड
* **MATLAB**: Robotics System Toolbox स्क्रिप्ट
* **JSON**: पोर्टेबल पोज़ एक्सपोर्ट

---

## 📂 Project Directory Structure

```
c:\Ankit Satnami\Projects\Robotic Arm simulator\
│
├── index.html              # मुख्य यूजर इंटरफ़ेस और 3D कैनवास
├── start_simulator.bat     # 1-क्लिक लोकल सर्वर लॉन्चर
├── server.ps1              # लाइटवेट पॉवरशेल HTTP सर्वर
├── README.md               # डाक्यूमेंटेशन और गाइड
│
├── css/
│   └── style.css           # इंडस्ट्रियल डार्क-थीम स्टाइलिंग और लेआउट
│
└── js/
    ├── kinematics.js       # FK, 8-Solution Analytical IK, DH & Jacobian Engine
    ├── robot-model.js      # 3D PBR मेष, ग्रिपर, जॉइंट्स और वर्कस्पेस एनवायरनमेंट
    ├── trajectory.js       # वेपॉइंट्स, MOVEJ/MOVEL और पिक-एंड-प्लेस सीक्वेंसर
    ├── learning.js         # 6 इंटरैक्टिव लेसन्स और मल्टी-लैंग्वेज कोड जनरेटर
    └── main.js             # थ्री.जेएस सीन, इवेंट लिसनर्स और ऑर्केस्ट्रेशन
```
