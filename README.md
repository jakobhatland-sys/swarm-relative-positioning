# Relativ posisjonering i dronesverm

Estimering av relativ og absolutt posisjon i en sverm på 64 UAV-er ved fusjon av UWB-avstandsmålinger, RTK-ankre og barometer i et sentralisert (iterert) Extended Kalman Filter.

**Problemstilling:** Hvor få ankre og hvor lite konnektivitet tåler svermen før relativ og absolutt posisjonsnøyaktighet bryter sammen?

![UWB-nettverk for k = 4 og k = 8](figures/uwb_network.png)

## Metode

| Komponent | Modell |
|---|---|
| Sverm | 64 droner, 8×8-rutenett, 10 m avstand, ca. 50 m høyde |
| Bevegelse | Felles hastighet 2 m/s + individuelt Gauss-Markov-avvik |
| UWB | Avstand til k nærmeste naboer, σ = 0,1 m |
| Ankre | RTK-posisjon, σ = 0,02 m, plassert med farthest point sampling |
| Barometer | Høyde for alle droner, σ = 0,3 m (valgfritt) |
| Estimator | Sentralisert EKF/IEKF, tilstand [p, v] per drone (384 tilstander), konstant-hastighetsmodell, Joseph-form |
| Evaluering | Absolutt RMS, relativ RMS (etter Kabsch-tilpasning), ANEES, Monte Carlo |

Konvergens er definert som relativ RMS < 0,3 m (3σ_UWB). RMS og ANEES rapporteres som median blant konvergerte kjøringer.

## Resultater

### 1. Verifikasjon av filteret
Med 16 ankre og k = 8 konvergerer filteret til 7 cm RMS med ANEES 0,97 (95 % intervall [0,91, 1,09]) for initialfeil ≤ 1 m. Ved 2 m initialfeil divergerer det. Implementasjonen er korrekt, og konvergensområdet ligger mellom 1 og 2 m.

### 2. Ankre og konnektivitet (EKF, initialfeil 1 m, M = 10)

| Ankre | k | Konv. | Abs. RMS [m] | Rel. RMS [m] | ANEES |
|---:|---:|---:|---:|---:|---:|
| 1 | 4 | 0/10 | – | – | – |
| 2 | 4 | 0/10 | – | – | – |
| 4 | 4 | 0/10 | – | – | – |
| 8 | 4 | 0/10 | – | – | – |
| 16 | 4 | 1/10 | 0,187 | 0,183 | 1,09 |
| 1 | 6 | 4/10 | 5,803 | 0,190 | 39,57 |
| 2 | 6 | 2/10 | 3,815 | 0,211 | 26,77 |
| 4 | 6 | 5/10 | 0,205 | 0,163 | 1,03 |
| 8 | 6 | 4/10 | 0,156 | 0,135 | 1,02 |
| 16 | 6 | 10/10 | 0,098 | 0,094 | 1,00 |
| 1 | 8 | 6/10 | 6,256 | 0,119 | 47,75 |
| 2 | 8 | 9/10 | 1,356 | 0,115 | 1,20 |
| 4 | 8 | 7/10 | 0,109 | 0,093 | 0,91 |
| 8 | 8 | 10/10 | 0,094 | 0,088 | 1,01 |
| 16 | 8 | 10/10 | 0,075 | 0,074 | 1,00 |

### 3. EKF vs. IEKF (8 ankre, M = 5)

| Initialfeil | k | EKF konv. | IEKF konv. | Rel. RMS IEKF [m] |
|---:|---:|---:|---:|---:|
| 1 m | 4 | 0/5 | 0/5 | – |
| 1 m | 6 | 1/5 | 4/5 | 0,111 |
| 1 m | 8 | 5/5 | 5/5 | 0,095 |
| 2 m | 4 | 0/5 | 0/5 | – |
| 2 m | 6 | 0/5 | 3/5 | 0,110 |
| 2 m | 8 | 1/5 | 5/5 | 0,095 |

### 4. Barometer (IEKF, 8 ankre, initialfeil 1 m, M = 5)

| k | Barometer | Konv. | Rel. RMS [m] | ANEES |
|---:|---|---:|---:|---:|
| 4 | nei | 0/5 | – | – |
| 4 | σ = 0,3 m | 5/5 | 0,171 | 1,01 |
| 6 | nei | 4/5 | 0,111 | 0,93 |
| 6 | σ = 0,3 m | 5/5 | 0,100 | 1,02 |

## Funn

1. **Relativ geometri er uavhengig av ankre.** Ved konvergens er relativ RMS 7–21 cm uansett antall ankre. Den bestemmes av UWB-nettverket.
2. **Absolutt posisjon krever ≥ 4 ankre.** Med 1 anker er rotasjon om ankeret uobserverbar (ca. 6 m feil). Med 2 ankre er rotasjon om aksen gjennom ankrene uobserverbar (ca. 1,4 m). Med ≥ 4 ankre er absolutt RMS ≈ relativ RMS.
3. **Falsk observerbarhet.** Med 1–2 ankre er ANEES 27–48. Lineariseringen gir filteret informasjon i uobserverbare retninger, og P blir overkonfident.
4. **Konnektivitet styrer robustheten.** k = 8 er robust, k = 6 krever mange ankre, og k = 4 feiler.
5. **IEKF utvider konvergensområdet.** Konvergensraten øker, særlig ved 2 m initialfeil, mens nøyaktigheten er uendret.
6. **k = 4 feiler selv fra perfekt initialisering.** Problemet er observerbarhet, ikke initialisering.
7. **Rotårsaken er plan geometri.** Svermen spenner ca. 70 m horisontalt og bare ca. 4 m vertikalt, så ∂ρ/∂z ≈ 0 for nesten alle UWB-linker. Barometer gir k = 4 5/5 konvergens med ANEES ≈ 1, og konnektivitetskravet reduseres fra d+1 = 4 til 3.

**Foreløpig svar på problemstillingen:** relativ posisjonering krever 0–1 ankre, absolutt posisjonering ≥ 4 ankre. Uten barometer kreves k ≥ 8 for robust konvergens; med barometer er k = 4 tilstrekkelig.

## Begrensninger

- Filteret initialiseres med sann posisjon pluss støy; ingen initialiseringsalgoritme (f.eks. MDS).
- Nabogrupper velges fra sann posisjon.
- Barometer modellert med hvit støy, uten langsom bias.
- UWB uten utliggere (NLOS) og uten pakketap.
- Sentralisert estimator; ikke skalerbar til sanntid på drone.
- M = 5–10 gir stor usikkerhet i konvergensrater (ca. ±30 prosentpoeng).

## Videre arbeid

- Verifisere rigiditetsteori: k = 2 og k = 3 med barometer.
- Øke Monte Carlo til M ≥ 30.
- MDS-initialisering og robust UWB-modell (NLOS, Huber-tap).
- Desentralisert estimator (covariance intersection) og sammenligning mot sentralisert referanse.
- Feltvalidering med 2–4 RTK-mottakere og UWB-moduler.

## Struktur

```
src/swarm/        simulation.py, sensors.py, estimator.py, metrics.py
tests/            enhetstester (målemodell, Jacobian, IEKF, Kabsch)
experiments/      anchor_sweep.py, compare_iekf.py, compare_baro.py, diagnose.py, plot_network.py
figures/          genererte figurer
```

## Kjøring

```powershell
py -3.15 -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python -m pytest
python experiments\anchor_sweep.py
```