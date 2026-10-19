# Problemi noti di Isaac Sim (UX/tooling, non specifici alla scena)

Comportamenti inaffidabili di Isaac Sim emersi con `Manipolazione/Apps/Simulation/gripper.usd`, con causa e mitigazione. Non sono sintomi della scena (quelli stanno in `SETUP_SCENARIO_CAD.md`) ma limiti/bug del tool stesso.

## 1. Coordinate nel pannello Transform fuorvianti

Stessa causa e mitigazione già descritte in
[SETUP_SCENARIO_CAD.md §7bis](SETUP_SCENARIO_CAD.md#7bis-il-gizmo-del-joint-nel-viewport-non-è-affidabile)
(pannello Property in Local, rotazione ereditata diversa per body) — qui vale come limite generale
del tool, non solo per questa scena.

## 2. Gizmo dei Joint inaffidabile

Stesso debug-draw stantio descritto in [SETUP_SCENARIO_CAD.md §7bis](SETUP_SCENARIO_CAD.md), con gli
stessi script di verifica (`print_joint_anchor_world.py`, `print_joint_current_state.py` in Pausa) —
qui vale come limite generale del tool, non solo per questa scena.

## 3. Script Editor: la console non si può copiare

Il testo stampato nella console dello Script Editor non è selezionabile/copiabile, e scrolla via su output lunghi.

**Mitigazione**: ogni script di ispezione scrive anche su un `.txt` in `Manipolazione/Scripts/logs/` (`physics_api_inspector_output.txt`, `physics_joint_inspector_output.txt`), leggibile da fuori Isaac Sim.

## 4. Measure Tool praticamente inutilizzabile

Stesso problema e stessa tecnica marker (con la tabella completa degli script) già descritti in
[SETUP_SCENARIO_CAD.md §7ter](SETUP_SCENARIO_CAD.md#7ter-trovare-un-punto-fisico-preciso-quando-il-measure-tool-non-si-aggancia) —
qui vale come limite generale del tool, non solo per questa scena.

## 5. Nessun readout live della posizione di un Joint nel pannello Property

Un PrismaticJoint non espone un campo "State"/posizione corrente leggibile a pannello in questa versione di Kit — impossibile confrontare a occhio la posizione attuale coi limiti configurati durante il Play.

**Mitigazione**: `print_joint_current_state.py`, da eseguire in **Pausa** (non Stop, che resetta alla posa di riposo) — calcola la posizione attuale lungo l'asse dagli anchor world dei due body e la confronta coi limiti. Stesso script descritto in [SETUP_SCENARIO_CAD.md §7bis](SETUP_SCENARIO_CAD.md).
