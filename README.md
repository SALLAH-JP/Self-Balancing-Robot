<div align="center">
  <img src="https://capsule-render.vercel.app/api?type=waving&color=gradient&customColorList=12,20,24&height=180&section=header&text=Self-Balancing%20Robot&fontSize=40&fontColor=fff&animation=twinkling&fontAlignY=36&desc=Arduino%20%7C%20MPU6050%20%7C%20PID%20Controller&descAlignY=58&descAlign=50" width="100%"/>
</div>

<div align="center">

![Arduino](https://img.shields.io/badge/Arduino-C%2B%2B-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![PID](https://img.shields.io/badge/Control-PID-FF6B35?style=for-the-badge)
![IMU](https://img.shields.io/badge/Sensor-MPU6050-6C3483?style=for-the-badge)
![IR](https://img.shields.io/badge/Remote-IR%20Control-E74C3C?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)

</div>

---

## Description

Robot biphasique auto-équilibrant basé sur Arduino. Il utilise le **DMP** (Digital Motion Processor) du MPU6050 pour extraire l'angle de tangage avec précision, et un **contrôleur PID** pour corriger en temps réel la vitesse des moteurs et maintenir l'équilibre.

Le robot dispose de **3 modes de fonctionnement** commutables via un bouton physique ou une télécommande IR.

---

## Modes de fonctionnement

| Mode | LED | Description |
|------|-----|-------------|
| **0 — Balance** | Jaune | Équilibre autonome, le robot reste sur place |
| **1 — Remote Control** | Verte | Contrôle directionnel par télécommande IR |
| **2 — Line Tracking** | Rouge | Suivi de ligne autonome par capteurs IR |

> Le bouton physique sur **PIN 2** démarre/arrête le robot. La télécommande peut également changer de mode à distance.

---

## Matériel requis

| Composant | Quantité | Rôle |
|-----------|----------|------|
| Arduino Uno / Nano | 1 | Microcontrôleur principal |
| MPU6050 | 1 | IMU 6 axes (accéléromètre + gyroscope) |
| Moteurs DC + encodeurs | 2 | Propulsion |
| Driver moteur (L298N ou similaire) | 1 | Commande PWM des moteurs |
| Récepteur IR | 1 | Télécommande |
| Capteurs de suivi de ligne IR | 2 | Mode Line Tracking |
| LED bicolore (rouge/vert) | 1 | Indicateur de mode |
| Bouton poussoir | 1 | Démarrage / arrêt |
| Batterie LiPo ou 18650 | 1 | Alimentation |

---

## Câblage

```
Arduino Pin  │ Composant
─────────────┼─────────────────────────────────
2            │ Bouton mode (INPUT_PULLUP + interruption)
3            │ MPU6050 INT (interruption DMP)
4            │ Récepteur IR
5            │ Moteur Droit PWM1
6            │ Moteur Droit PWM2
7            │ Capteur de ligne GAUCHE
8            │ Capteur de ligne DROIT
9            │ Moteur Gauche PWM1
10           │ Moteur Gauche PWM2
11           │ LED Rouge
12           │ LED Verte
A4 (SDA)     │ MPU6050 SDA
A5 (SCL)     │ MPU6050 SCL
```

---

## Dépendances

Installer via le **Library Manager** de l'IDE Arduino :

| Bibliothèque | Source |
|---|---|
| `IRremote` | Arduino Library Manager |
| `PID_v1` | [br3ttb/Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library) |
| `I2Cdev` | [jrowberg/i2cdevlib](https://github.com/jrowberg/i2cdevlib) |
| `MPU6050_6Axis_MotionApps20` | Incluse dans i2cdevlib |

---

## Installation

1. **Cloner le repo**
   ```bash
   git clone https://github.com/SALLAH-JP/Self-Balancing-Robot.git
   ```

2. **Installer les dépendances** via le Library Manager de l'IDE Arduino

3. **Ouvrir** `Stage.ino` dans l'IDE Arduino (les 3 fichiers doivent être dans le même dossier)

4. **Calibrer le MPU6050** — modifier les offsets dans `Fonctions.ino` selon ton module :
   ```cpp
   mpu.setXGyroOffset(-76);
   mpu.setYGyroOffset(-54);
   mpu.setZGyroOffset(2);
   mpu.setZAccelOffset(1384);
   ```

5. **Flasher** sur l'Arduino et ouvrir le **Serial Monitor** à `115200 baud` pour vérifier l'angle de tangage

---

## Tuning PID

Les constantes PID sont définies dans `header.h` :

```cpp
double Kp_angle = 15;   // Gain proportionnel
double Ki_angle = 150;  // Gain intégral
double Kd_angle = 0.8;  // Gain dérivé
#define EQUILIBRE 0.1   // Angle d'équilibre (en degrés)
```

### Méthode de réglage recommandée

1. Mettre `Ki` et `Kd` à `0`, augmenter `Kp` jusqu'à ce que le robot oscille
2. Augmenter `Kd` pour amortir les oscillations
3. Augmenter `Ki` pour corriger l'erreur statique (drift)
4. Ajuster `EQUILIBRE` si le robot avance ou recule naturellement au repos

> Le cutoff de sécurité est fixé à **±40°** — au-delà, les moteurs sont coupés.

---

## Télécommande IR

Codes IR utilisés (protocole NEC) :

| Bouton | Code HEX | Action |
|--------|----------|--------|
| Haut | `0x18` | Avancer |
| Bas | `0x52` | Reculer |
| Gauche | `0x08` | Tourner gauche |
| Droite | `0x5A` | Tourner droite |
| OK | `0x45` | Démarrer / Arrêter |
| `*` | `0x46` | Cycle mode (0→1→2) |
| `1` | `0x44` | Mode Remote Control |
| `2` | `0x19` | Mode Line Tracking |
| `3` | `0x07` | Mode Balance |

---

## Architecture du code

```
Self-Balancing-Robot/
├── header.h          # Includes, pins, variables, constantes PID
├── Stage.ino         # setup(), loop(), logique des 3 modes
└── Fonctions.ino     # IMU, moteurs, modes, télécommande
```

| Fichier | Rôle |
|---------|------|
| `header.h` | Déclarations globales, configuration PID, broches |
| `Stage.ino` | Boucle principale, `balancing()`, `lineTracking()`, `directionRemoteControl()` |
| `Fonctions.ino` | `initIMU()`, `getPitchIMU()`, `driveMotors()`, `changeMode()` |

---

## Auteur

**Jean-Paul SALLAH** — [github.com/SALLAH-JP](https://github.com/SALLAH-JP) · [LinkedIn](https://www.linkedin.com/in/jeanpaul-sallah/)

<div align="center">
  <img src="https://capsule-render.vercel.app/api?type=waving&color=gradient&customColorList=12,20,24&height=100&section=footer" width="100%"/>
</div>
