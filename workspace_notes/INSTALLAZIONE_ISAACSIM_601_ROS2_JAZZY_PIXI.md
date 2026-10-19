# Isaac Sim 6.0.1 + Isaac Lab + ROS2 Jazzy (pixi) — Windows

Tutto l'ambiente (env Python, Isaac Lab, workspace ROS2) va tenuto in un'**unica cartella
radice**, in una posizione **con percorso corto** (es. `C:\IsaacSim`, non dentro cartelle
utente annidate) — necessario per pixi ed evita errori di percorso troppo lungo.

Struttura finale:
```
C:\IsaacSim\
  env\                    (ambiente conda Isaac Sim + Isaac Lab)
  IsaacLab\                (repo Isaac Lab)
  ros_workspaces\
    jazzy_ws\               (workspace ROS2 Jazzy via pixi)
```

## Requisiti
- Windows 11 (Windows 10 non supportato)
- GPU NVIDIA RTX, ≥16 GB VRAM consigliati
- ≥40 GB disco libero

---

## FASE 0 — Preparazione

**0.1 — Driver NVIDIA ≥595.97**
Scarica da https://www.nvidia.com/download/index.aspx, installa, riavvia.
Su GPU RTX PRO Blackwell: usare il ramo **Enterprise Production Branch (R595)**, non il Game Ready.

**0.2 — Percorsi lunghi**
PowerShell come amministratore:
```powershell
New-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" `
  -Name "LongPathsEnabled" -Value 1 -PropertyType DWORD -Force
```
Riavvia il PC.

**0.3 — MSVC Build Tools 2022**
Da https://visualstudio.microsoft.com/visual-cpp-build-tools/, workload "Desktop development with C++".

**0.4 — Miniconda**
Da https://docs.conda.io/en/latest/miniconda.html, opzioni predefinite.

**0.5 — Cartella radice**
```powershell
New-Item -ItemType Directory -Path C:\IsaacSim -Force
```

---

## FASE 1 — Isaac Sim 6.0.1

Anaconda Prompt:
```powershell
conda create --prefix C:\IsaacSim\env python=3.12 -y
conda activate C:\IsaacSim\env
pip install torch==2.11.0 --index-url https://download.pytorch.org/whl/cu128
pip install "isaacsim[all,extscache]==6.0.1.0" --extra-index-url https://pypi.nvidia.com
```

---

## FASE 2 — Isaac Lab

```powershell
conda activate C:\IsaacSim\env
cd C:\IsaacSim
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
git checkout v3.0.0-beta2.patch1
.\isaaclab.bat -i
```
Durante l'install compaiono righe `ERROR: pip's dependency resolver...`: non fatali, proseguire. Risolte in FASE 2.5.

---

## FASE 2.5 — Allineamento versioni pacchetti

```powershell
pip install torch==2.11.0 torchvision==0.26.0 torchaudio==2.11.0 --index-url https://download.pytorch.org/whl/cu128
pip install "packaging==26.0" "psutil==5.9.8"
```

Verifica:
```powershell
pip list | Select-String "^(torch|torchvision|torchaudio|packaging|psutil) "
```
Atteso: torch 2.11.0, torchvision 0.26.0, torchaudio 2.11.0, packaging 26.0, psutil 5.9.8.

---

## FASE 3 — Verifica

```powershell
conda activate C:\IsaacSim\env
cd C:\IsaacSim\IsaacLab
.\isaaclab.bat -p scripts\environments\random_agent.py --task Isaac-Cartpole-v0 --num_envs 1
```
Atteso: finestra 3D con cartpole in movimento.

**Problema noto (6.0.1, non risolto)**: questo comando può aprire una GUI reattiva ma con
**viewport completamente bianco** (nessuna griglia, nessun cartpole). Non è un problema di
driver — Isaac Sim puro (`pixi run sim`, FASE 5) renderizza correttamente; il problema è
isolato al lancio tramite `isaaclab.bat`. L'override
`--kit_args="--/app/useFabricSceneDelegate=false"` non lo risolve su questa versione. Non
blocca ROS2.

Se invece la finestra non si apre o crasha con `ERROR_DEVICE_LOST`: driver GPU, torna a FASE 0.1.

---

## FASE 4 — ROS2 Jazzy (pixi)

PowerShell normale (no admin):
```powershell
winget install prefix-dev.pixi
```
Chiudi e riapri PowerShell.

```powershell
git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git C:\IsaacSim\ros_workspaces
cd C:\IsaacSim\ros_workspaces\jazzy_ws
pixi install
pixi run build
```

Note:
- Il workspace pixi installa un **proprio Isaac Sim 6.0.1.0** indipendente (`.pixi\envs\default`, ~11 GB con ROS2): `pixi run sim` non usa l'env conda della FASE 1, è autocontenuto.
- `pixi run build` richiede MSVC Build Tools 2022 (FASE 0.3).

**Firewall/proxy con SSL inspection (Fortinet, Zscaler, ecc.)** — se `pixi install` fallisce con `invalid peer certificate: UnknownIssuer`:

Causa: pixi usa la libreria Rust `rustls` per le connessioni HTTPS (es. verso `prefix.dev/conda-forge`), che **non legge lo store dei certificati di Windows** — quindi non conosce il certificato "intermedio" che un proxy con SSL-inspection inserisce nella connessione, e la rifiuta come non fidata.

Fix: generare un bundle PEM con tutti i certificati radice di Windows (incluso quello del proxy aziendale) e puntarci la variabile `SSL_CERT_FILE`, così pixi si fida anche di quello:
```powershell
$out = "C:\IsaacSim\pixi-ca-bundle.pem"
$certs = Get-ChildItem Cert:\LocalMachine\Root, Cert:\CurrentUser\Root -ErrorAction SilentlyContinue
$lines = foreach ($c in $certs) { "-----BEGIN CERTIFICATE-----"; [Convert]::ToBase64String($c.RawData,'InsertLineBreaks'); "-----END CERTIFICATE-----" }
$lines | Out-File -FilePath $out -Encoding ascii
[Environment]::SetEnvironmentVariable("SSL_CERT_FILE", $out, "User"); $env:SSL_CERT_FILE = $out
```
Eseguire prima di `pixi install`. Il file generato è specifico di questa macchina/rete (non versionarlo in git).

Nota: `git submodule update --init --recursive` non ha (finora) lo stesso problema — solo pixi/rustls è affetto, non git.

---

## FASE 5 — Isaac Sim ↔ ROS2 (test)

Bridge via Zenoh. 3 finestre PowerShell, tutte da `C:\IsaacSim\ros_workspaces\jazzy_ws`.

**Router Zenoh obbligatorio anche in locale** (senza, `ros2 topic list` non mostra nulla — con `rmw_zenoh_cpp` la discovery passa sempre da `rmw_zenohd`). Avviarlo per primo, tenerlo aperto per tutta la sessione.

**Finestra 1:**
```powershell
pixi run zenoh
```

**Finestra 2:**
```powershell
pixi run sim
```
Apri una scena con dati ROS2 (es. ROS2 Clock), premi Play.

**Finestra 3:**
```powershell
pixi run ros2 topic list
```
Atteso: `/clock`, `/parameter_events`, `/rosout`.
```powershell
pixi run ros2 topic echo /clock
```
Numeri che scorrono = bridge funzionante.

---

## Uso quotidiano

**Solo Isaac Lab:**
```powershell
conda activate C:\IsaacSim\env
cd C:\IsaacSim\IsaacLab
.\isaaclab.bat -p <script>.py
```

**Isaac Sim + ROS2:** 3 finestre come FASE 5 (zenoh → sim → ros2).

---

## Problemi comuni

| Sintomo | Soluzione |
|---|---|
| Install pip si interrompe | Rilancia lo stesso comando |
| Errore percorsi troppo lunghi | FASE 0.2 (LongPathsEnabled) + cartella radice corta (`C:\IsaacSim`) |
| Isaac Sim crasha (`DEVICE_LOST`) | Aggiorna driver NVIDIA (FASE 0.1) |
| Viewport bianco solo con `isaaclab.bat` | Bug noto 6.0.1, non driver — vedi FASE 3. Usa `pixi run sim` per verificare il rendering |
| `pixi` non riconosciuto | Chiudi/riapri PowerShell dopo l'install |
| `pixi run build` fallisce | Verifica MSVC Build Tools 2022 (FASE 0.3) |
| `pixi install` → `UnknownIssuer` | Proxy con SSL inspection → fix `SSL_CERT_FILE` (FASE 4) |
| `ros2 topic list` vuoto | 1) `pixi run zenoh` avviato? 2) Play premuto in Isaac? 3) `RMW_IMPLEMENTATION=rmw_zenoh_cpp` su tutte le finestre? |

---

## Link ufficiali
- Isaac Sim install: https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_python.html
- ROS2 + pixi (guida seguita): https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros_other_platforms.html#isaac-sim-ros-workspace-other-platforms
- Isaac Lab: https://github.com/isaac-sim/IsaacLab
- Workspace ROS2: https://github.com/isaac-sim/IsaacSim-ros_workspaces
- pixi: https://pixi.sh/latest/tutorials/ros2/
