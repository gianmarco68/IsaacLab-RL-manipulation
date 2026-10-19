# Architettura task `Move-To-Grasp-Apps-v0` — macchina a stadi

Descrive **cosa fa ogni modulo, ogni gate, ogni parametro**.

## 1. Scenario e obiettivo

Un braccio FANUC (`m710id_50m`) con un gripper a doppia pinza (`right_claw`/`left_claw`, ognuna con
pollice e indice) deve afferrare una "staffa quadrata" appoggiata a
terra. Nessuna randomizzazione: ogni reset riparte dalla stessa posa di pre-grasp
(`robots/apps.py`, `init_state.joint_pos`).

Il vincolo "prima allinea, poi scorri, poi scendi" è **imposto sulle azioni, non solo sulle reward**. Una reward a zero lascia comunque l'asse libero di muoversi; qui
invece l'asse è congelato finché lo stadio non lo sblocca.

## 2. La macchina a stadi (`mdp/stages.py`)

```
ALIGN (0)   — dx, dy, dyaw scrivibili; dz e slitte congelate
   ↓ gate: is_aligned (angolo < angle_tol, perp < perp_tol) AND fermo
SLIDE (1)   — slitte scrivibili; braccio congelato (posa latchata)
   ↓ gate: is_ready_to_descend (= is_aligned AND along < along_tol) AND fermo
DESCEND (2) — dz scrivibile; dx/dy/dyaw e slitte congelate al target raggiunto
```

`StageTracker` (uno stato per env, `torch.long`) ricalcola lo stadio una volta per step
(`common_step_counter` distingue la chiamata dei due action term dallo step successivo) e lo
azzera quando `episode_length_buf == 0`. **Monotono per design**: promuovere è irreversibile,
altrimenti il rumore che attraversa una tolleranza farebbe oscillare gli assi avanti e indietro
(`StageTracker` in `stages.py`).

`is_still` fa parte del gate: la promozione congela assi che potrebbero ancora correggere la posa,
quindi non deve scattare mentre la pinza è in volo (`ClawGeometry.is_still`).

## 3. Scomposizione dell'errore (`ClawGeometry._errors` in `claw_geometry.py`)

Per ogni pinza, l'offset dal suo target di presa si scompone in quattro numeri indipendenti, letti
in top-view salvo `height`:

| Componente | Cosa misura | Chi lo annulla | Stadio |
|---|---|---|---|
| `angle` | angolo fra la guida (asse del gripper) e il lato del target | il braccio (dx/dy/dyaw) | ALIGN |
| `perp` | offset laterale della guida rispetto al lato | il braccio | ALIGN |
| `along` | offset lungo la guida fra punto di presa e target | la slitta | SLIDE |
| `height` | differenza di quota, con segno (positivo = punto sopra il target) | il braccio | DESCEND |

`perp` e `along` ricostruiscono insieme l'offset orizzontale senza sovrapporsi. La direzione della
guida è letta a runtime dalla rotazione del frame `ee` (non da una misura fissa): se cambia la
cinematica del polso, continua a funzionare senza modifiche.

`ClawGeometry` è la classe base di ogni reward/observation/termination/event di questo task:
risolve nomi di frame e giunti a indici una sola volta, così tutti condividono la stessa geometria
e gli stessi gate (`is_aligned`, `is_ready_to_descend`, `_ready_to_clamp`, `is_still`, `_tilt`).

## 4. Azioni a stadi (`mdp/actions.py`)

Due action term, entrambi guardano lo stesso `StageTracker` tramite `get_stage_tracker` e scrivono
solo gli assi di competenza; il resto **latcha** il target invece di essere semplicemente ignorato.

### 4.1 `StagedLevelIKAction` — IK relativa, orizzontale in ALIGN, verticale in DESCEND

Il termine IK di libreria ricostruisce il target dalla posa misurata ogni step: comando zero
significa "tieni quello che hai", quindi una deriva o un'inclinazione diventa il nuovo target. Qui
invece:

- **livellamento attivo** (`_level_quat_from_measured` in `mdp/actions.py`): l'orientamento target è ricostruito da zero
  come *livello + yaw comandato*, non ereditato dalla posa misurata. La correzione è la rotazione
  minima che riporta `level_axis_in_ee` sulla verticale — lascia intatta la rotazione attorno a
  quell'asse, quindi non combatte con lo yaw. Tutto a quaternioni: a questa posa il pitch è vicino a
  90°, dove gli euler degenerano.
- **latch per asse**: `_pos_des`/`_quat_des` sono stato persistente del term. In ALIGN si scrive
  `dx, dy` e l'orientamento livellato; in DESCEND si scrive `dz`; fuori dal proprio stadio ogni asse
  riemette il valore salvato (`process_actions` in `mdp/actions.py`).
- **seeding**: al primo `process_actions` dopo un reset (`_needs_seed`), il target parte dalla posa
  misurata — non esiste ancora nulla da latchare.

### 4.2 `StagedJointPositionAction` — le slitte, scrivibili solo in SLIDE

Fuori dallo stadio SLIDE riemette `_held_target` invece del default: droppare il comando tornerebbe
all'offset e disfarebbe quanto già scorso (`StagedJointPositionAction.process_actions`).

### 4.3 Le cfg (`mdp/actions_cfg.py`)

Separate da `actions.py` perché le env cfg si costruiscono **prima** che `SimulationApp` esista:
`class_type` è una stringa (`"{DIR}.actions:StagedLevelIKAction"`), risolta da Isaac Lab solo
quando l'action manager istanzia il termine — nessun import di `pxr` a livello di modulo.
`StageGateCfg` porta le tre soglie condivise (`stage_gate`, `stage_hold_steps`, `stage_vel_tol`),
identiche sui due action term perché devono vedere lo stesso stadio.

## 5. Reward: gate sullo stato, non su una riverifica delle tolleranze (`mdp/rewards.py`)

Un gate basato su una finestra di tolleranza si spegnerebbe appena la posa ne uscisse di un
millimetro, anche se la policy non ha più il permesso di correggerla. Per questo il gate è
`self._stage(env) >= N`, latchato come lo stadio stesso:

| Reward | Paga da stadio | Copre |
|---|---|---|
| `guides_parallel_to_sides` | sempre (nessun gate, è il primo) | `angle` |
| `guides_over_sides` | sempre | `perp` |
| `claw_slide_alignment` | `>= SLIDE` | `along` |
| `grasp_points_at_target_height` | `>= DESCEND` | `height` |
| `align_grasp_around_target` | `>= DESCEND` | target fra le dita (booleano, peso basso: non guida la traiettoria) |
| `hold_still_at_target` | `_ready_to_clamp` (stadio DESCEND + quota + target fra le dita) | velocità → 0 |
| `both_fingers_in_contact` | `_ready_to_clamp` | forza di contatto per dito |

Ogni `std` (l'errore a cui la reward si è dimezzata) è dimensionato sulla distanza che il suo
stadio copre davvero: se troppo piccolo, `tanh` satura e il termine nasce a zero senza gradiente. I
valori misurati alla posa di pre-grasp sono nel commento sopra `ALIGN_TOL`/`SLIDE_TOL` in
`move_to_grasp_env_cfg.py`, punto di partenza per la taratura.

## 6. Osservazioni (`mdp/observations.py`)

Oltre a `joint_pos_rel`/`joint_vel_rel`/`last_action` (generici), quattro termini specifici:

| Termine | Contenuto |
|---|---|
| `task_stage` | stadio corrente, one-hot su 3 classi — necessario perché gli assi scrivibili cambiano con lo stadio, quindi la stessa osservazione avrebbe conseguenze diverse senza |
| `grasp_to_target_error` | `angle, perp, height, along` per pinza |
| `gripper_tilt` | inclinazione del piano delle guide rispetto all'orizzontale — le proiezioni top-view sopra non la vedono |
| `finger_contact_forces` | forza per dito, altrimenti la reward di contatto sarebbe invisibile alla policy |

## 7. Terminazioni ed evento di chiusura

- `descended_before_aligned` (`terminations.py`): finisce l'episodio se un punto di presa scende
  sotto il target (`height > depth_tol`) mentre lo stadio è ancora `< DESCEND` — a quel punto la
  pinza è a fianco della staffa, non intorno, e le dita possono solo spingerla via.
- `gripper_not_level` (`terminations.py`): finisce l'episodio se `_tilt` supera la tolleranza.
  L'action term tiene il piano livellato di suo, quindi questo scatta solo se qualcosa ha
  sovrastato l'IK.
- `close_fingers_when_in_position` (`events.py`): evento a intervallo (una volta per step),
  **non è la policy a comandare le dita**. Chiude quando `_ready_to_clamp AND is_still` per
  `hold_steps` consecutivi; una volta chiuse restano tali fino al reset perché nessun action term
  scrive più su quei giunti.

## 8. Descrittori robot-agnostici (`arm_descriptor.py`, `gripper_descriptor.py`)

Il task stesso (`move_to_grasp_env_cfg.py`) non nomina mai un giunto o un corpo del FANUC: tutto
passa da `ArmDescriptorCfg`/`GripperDescriptorCfg`, riempiti per questa cella in
`config/apps/move_to_grasp_apps_cfg.py` (`APPS_ARM`, `APPS_GRIPPER`). Portare il task su un altro
braccio significa scrivere un nuovo file come questo, non toccare `mdp/`.

Punti specifici della cella Apps in quel file:

- `planar_scale` azzera roll/pitch: il livellamento li tiene fissi, quindi il campo serve solo a
  bloccare la policy dal comandarli.
- I due target di presa (`right_grasp_target`/`left_grasp_target`) sono `OffsetCfg` misurati in
  GUI, quaternioni in convenzione `(x, y, z, w)`, non `(w, x, y, z)`.
- `stiffness=1.0e6, damping=4.0e4` sul braccio (`robots/apps.py`): guadagni scelti perché il cedimento
  sotto gravità (8 mm) resti sotto `HEIGHT_TOL`.

## 9. Note operative

- **Nessun import di Omniverse a livello di modulo in `mdp/`**: le env cfg si costruiscono prima
  che `SimulationApp` esista (§4.3).
- **`Usd.Stage.Open` senza `load=Usd.Stage.LoadAll` non carica i payload**: un'ispezione conclude
  che manca l'`ArticulationRootAPI` quando invece sta dentro un payload non caricato.
- **Il ground plane e il braccio del robot sono payload S3**: dietro un'ispezione SSL il fetch va in
  timeout e l'articolazione non si compone. Copie locali in `Apps/Props/` e
  `Apps/Simulation/robotApps_local.usd`.

## 10. Parametri da tarare

- **Numero di env**: i buffer PhysX (`gpu_found_lost_pairs_capacity`, `gpu_max_rigid_patch_count` in
  `move_to_grasp_apps_cfg.py`) sono dimensionati per pochi env pesanti (mesh CAD); con 256 env vanno
  riverificati.
- **`angle_tol` (3°)** è la tolleranza che condiziona l'apertura dello stadio SLIDE: se
  `guides_parallel_to_sides` (angle) si stabilizza appena sopra `angle_tol`, la policy non riesce a
  tenere l'angolo sotto soglia per `stage_hold_steps` step consecutivi e `claw_slide_alignment` resta a
  zero. Prima di allargare la tolleranza, verificare con `play_gate_tmp.py` (§12) se il residuo è
  diviso fra le due guide o concentrato su una sola — in quel caso serve un offset/rotazione del
  target, non una tolleranza maggiore.
- **`force_threshold = 1.0` N** per `both_fingers_in_contact` (peso 10 su ~19.5): da misurare.
- `HEIGHT_TOL`, `depth_tol`, `STILL_VEL_TOL`, `stage_hold_steps`: valori iniziali da tarare.
- `_is_in_position` in `claw_geometry.py`: senza chiamanti, dopo il passaggio ai gate sullo stato.
- `pregrasp_ee_rp` in `arm_descriptor.py`: superato dal livellamento attivo.

## 11. File coinvolti — mappa rapida

| File | Contenuto |
|---|---|
| `mdp/stages.py` | `StageTracker`, `get_stage_tracker` |
| `mdp/actions.py` | `StagedLevelIKAction`, `StagedJointPositionAction` |
| `mdp/actions_cfg.py` | `StageGateCfg` + le due cfg, `class_type` come stringa |
| `mdp/claw_geometry.py` | scomposizione errore, gate (`is_aligned`, `is_ready_to_descend`, `_ready_to_clamp`, `is_still`, `_tilt`) |
| `mdp/rewards.py` | i 7 termini di reward, gated sullo stadio |
| `mdp/observations.py` | `task_stage`, `grasp_to_target_error`, `gripper_tilt`, `finger_contact_forces` |
| `mdp/terminations.py` | `descended_before_aligned`, `gripper_not_level` |
| `mdp/events.py` | `close_fingers_when_in_position` |
| `move_to_grasp_env_cfg.py` | scena, azioni/reward/terminazioni/curriculum robot-agnostici, tolleranze e `std` |
| `config/apps/move_to_grasp_apps_cfg.py` | descrittori Apps, frame di presa, buffer PhysX |
| `arm_descriptor.py`, `gripper_descriptor.py` | schema dei descrittori robot-agnostici |
| `robots/apps.py` | `APPS_CFG`: articolazione, guadagni, posa di pre-grasp |

## 12. Consigli per i prossimi step

1. `isaaclab.bat -p .\scripts\rsl_rl\play_gate_tmp.py --task Move-To-Grasp-Apps-Play-v0 --num_envs 16 --headless --load_run <cartella> --checkpoint <model_N.pt>`
   stampa `angle`/`perp`/`along` per singola pinza (non la media) — lo strumento per rispondere
   a "perché il gate non si apre" e per distinguere un residuo diffuso da un bias su una sola guida.
2. Per leggere le curve di un run senza aprire TensorBoard: `EventAccumulator` di
   `tensorboard.backend.event_processing` sul file `events.out.tfevents.*` della cartella del run,
   con l'interprete `C:\isaacsim\kit\python\python.exe` (ha `tensorboard` già installato, non serve
   lanciare `isaaclab.bat`/`SimulationApp`).
3. Sul Reward Manager: `guides_parallel_to_sides`/`guides_over_sides` devono salire da subito;
   `claw_slide_alignment` e successive restano a zero finché lo stadio non le sblocca — non è un
   sintomo di bug, è il gate che funziona.
4. Su TensorBoard, l'osservazione `task_stage` (via `Episode_Reward`/log custom) per confermare che
   gli env avanzano di stadio nel tempo, non restano bloccati ad ALIGN.
5. In generale va sempre ricordato di usare la combo TensorBoard (all'inizio di ogni run il terminale stampa il `link al localhost`) + la funzione di resume del training tra una modifica e l'altra 

## 13. Script di lancio

Tutti dalla posizione ...\Apps\Training\source\Training: 

1. `isaaclab.bat -p scripts\rsl_rl\train.py --task Move-To-Grasp-Apps-v0 --num_envs 256 --headless`
Allena a pieno ritmo senza il viewport, quindi è la modalità più veloce. Crea una nuova cartella di run sotto logs/.

2. `isaaclab.bat -p scripts\rsl_rl\train.py --task Move-To-Grasp-Apps-v0 --num_envs 16 --viz kit`
Con poche env e il viewport aperto serve a controllare a occhio che le pinze si muovano come previsto. Senza --viz kit Isaac Lab 3.0 parte headless.

3. `isaaclab.bat -p scripts\rsl_rl\train.py --task Move-To-Grasp-Apps-v0 --num_envs 256 --headless --resume --load_run <cartella_run> --checkpoint model_1000.pt --max_iterations 1500`
Riprendere un training (resume), riparte dal checkpoint indicato e scrive in una nuova cartella di run. Se ometti --load_run e --checkpoint, prende l'ultimo run e l'ultimo checkpoint

4. `isaaclab.bat -p scripts\rsl_rl\train.py --task Move-To-Grasp-Apps-v0 --num_envs 4 --max_iterations 20 --viz kit --run_name prova`
Fa 20 iterazioni con 4 env, per verificare che task, azioni e reward girino prima di lanciare un run lungo. --run_name aggiunge un suffisso alla cartella, così riconosci il run di prova.

5. `isaaclab.bat -p scripts\rsl_rl\play.py --task Move-To-Grasp-Apps-Play-v0 --num_envs 16 --viz kit --load_run <cartella_run> --checkpoint model_1500.pt`
Carica la policy allenata, la esegue senza aggiornarla e usa la variante -Play del task, con meno env e senza rumore. Se non passi --checkpoint, prende l'ultimo.

<cartella_run> è il nome della sottocartella in logs/…/move_to_grasp/