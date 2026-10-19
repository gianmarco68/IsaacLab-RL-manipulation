# Guida sintetica — Scena FANUC + gripper da CAD

Procedura dall'import CAD al robot pronto per Isaac Lab (setup RigidBody/Collider/Joint del gripper).

## Prerequisito

**Isaac Sim standalone (`C:\isaacsim`)**, non la copia pip di pixi — il CAD Converter richiede
l'interprete Python imbustato in `kit/python`, assente nella copia pip.

## 1. Import CAD

- Rinomina i file `.stp` in `.STEP` (maiuscolo) — il CAD Converter riconosce solo quell'estensione.
- Converter Options: **Instancing Style = None**, **Composition Style = None**, **Unit =
  Millimeters** (esplicito), **Convert Metadata = attivo**, **Global Xforms = disattivo**,
  **Convert Physics Data = disattivo**.

## 2. Aprire il file convertito senza rompere l'editing

Chiudi la scena di anteprima **senza salvare**, poi `File → Open` **direttamente sul `.usd`**
prodotto — altrimenti stai editando una reference, non il file, e ogni rename/reparent fallisce
con "ancestral prim".

## 3. Unità e scala — leggere `metersPerUnit` PRIMA di toccare la scala

⚠️ Applicare `Scale = 0.001` senza prima leggere `metersPerUnit` è l'errore più comune: causa il
blocco descritto in §9.

Contano due cose insieme:
- i **point data** delle mesh, che il CAD Converter emette in **millimetri**
- **`metersPerUnit`**, metadata dello stage, che dichiara quanto vale un'unità

Isaac Sim vuole stage in **metri** (`metersPerUnit = 1.0`): FANUC ufficiale, ambienti e Robot
Assembler sono tutti in metri, e **USD non converte le unità quando referenzia**.

**Configurazione corretta**: `metersPerUnit = 1.0` **+** `Scale = 0.001` sul prim radice — la
conversione mm→m avviene una volta sola.

**Il CAD Converter può aver già dichiarato `metersPerUnit = 0.001`** (stage in mm, coerente coi
suoi point data in mm). In quel caso aggiungere `Scale = 0.001` è una **seconda** conversione e il
modello diventa **1000× più piccolo del reale**. Verifica con `stage_units_report.py`, correggi con
`stage_set_meters_per_unit.py`.

⚠️ **La griglia del viewport non è una verifica valida**: è disegnata in unità di stage, non in
metri. Con `metersPerUnit = 0.001` un quadretto è un millimetro, quindi un modello 1000× troppo
piccolo "sembra giusto". Verifica coi
numeri (`measure_rail_travel.py`): una pinza industriale sta sui 200÷300 mm, non su 0,3.

## 4. Distinguere assembly da parti finali

Il CAD Converter scrive il metadata `kind` (`group` = assembly, `component` = parte con Mesh) —
usalo per orientarti nella gerarchia invece di fidarti delle icone della Stage window, che
possono sparire per motivi di visualizzazione senza che i dati siano persi.

## 5. Isolare e riorganizzare la gerarchia

**Obiettivo**: un prim distinto per ogni pezzo cinematicamente indipendente (tutto ciò che deve
poter muoversi relativamente al resto), più un prim per ogni gruppo di parti che invece restano
sempre solidali tra loro.

**Come orientarsi nella gerarchia grezza del CAD Converter**: usa lo script `kind_hierarchy_inspector.py` che scrive
in automatico `group` = assembly/sottoassieme, `component` = parte finale con Mesh per capire
cosa contiene cosa.

**Riparenting: drag&drop diretto nello Stage panel**  — funziona senza l'errore "ancestral prim"
solo se stai editando `robot.usd`(il file generato dopo l'importazione del cad).

**Errore "Cannot move/rename ancestral prim" appena dopo l'import**: causa, stai ancora editando il
file di importazione mentre le modifiche vivono in un layer di sessione temporaneo sopra il file
vero (i prim non sono ancora autorati nel layer che stai editando). Fix: salva il file, chiudilo e
riapri quello salvato come nuovo stage — a quel punto il file diventa layer radice unico e il
riparenting funziona. Verifica nel pannello Layer che non resti un solo layer (nessun sub-layer/
reference): se ne restano, serve un Flatten Layer Stack prima di procedere.

**⚠️ La gerarchia USD (chi è figlio di chi) è indipendente dalla catena cinematica dei Joint.**

Dove la gerarchia **conta davvero** è per il Collider: una mesh senza un proprio RigidBody viene
assegnata automaticamente al RigidBody dell'antenato più vicino (vedi i 4 ruoli al punto 6) — è
questo il motivo per cui, ad esempio, i tre pannelli statici della base vanno raggruppati sotto
un unico genitore comune con un solo RigidBody, invece che uno ciascuno.

## 6. RigidBody + Collider — i 4 ruoli di un prim

Prima di applicare API fisiche, classifica ogni prim in uno di questi 4 ruoli (vale per
qualsiasi robot/meccanismo futuro, non solo questo gripper):

1. **Corpo che si muove indipendentemente** (collegato da un Joint — es. un dito che apre/chiude,
   un carrello che trasla): **RigidBodyAPI** obbligatorio (lo rende un corpo fisico indipendente
   su cui un Joint può agire) + **CollisionAPI sulla mesh reale** obbligatorio (senza, PhysX non
   ha geometria da cui calcolare massa/inerzia → warning "invalid inertia tensor... negative
   mass"). **Kinematic Enabled = True** finché il Joint non è completo (altrimenti cade per
   gravità durante il setup), `False` a joint pronto e testato. **Approximation = Convex Hull**,
   e in pratica sempre: Triangle Mesh classico non è ammesso su corpi con RigidBody, e **SDF Mesh
   non va usato per i fine-corsa** (§8 — i limiti si dichiarano, non si misurano a contatto). SDF
   resta giustificato solo per il contatto vero col mondo esterno (la presa del pezzo), e va
   applicato **alla singola mesh che tocca davvero, mai a un sottoalbero**: ~200 SDF simultanei
   saturano il generatore di contatti.
2. **Sotto-parte solidale a un corpo mobile** (non ha un proprio grado di libertà — es. viti,
   coperchi, il corpo di un dito distinto dal suo meccanismo di apertura): **solo CollisionAPI**,
   nessun RigidBody proprio — PhysX la assegna in automatico al RigidBody dell'antenato più
   vicino nella gerarchia. Darle un RigidBody per errore la rende un corpo indipendente slegato,
   che senza un Joint esplicito cade/fluttua per conto suo.
3. **Corpo fisso alla radice della catena** (es. la base — non ha un grado di libertà attivo ma è
   comunque un link dell'Articulation, verrà agganciato più avanti es. via Fixed Joint):
   **RigidBodyAPI** obbligatorio + **CollisionAPI** sulle sue mesh, **Kinematic Enabled = True**
   finché non viene effettivamente vincolato.
4. **Prim puramente organizzativo** (contenitore nello Stage panel senza corrispondere a un
   pezzo fisico reale, es. "tutto il gripper", "tutta una ganascia"): **nessuna API fisica** —
   altrimenti cattura per gerarchia le mesh sottostanti che dovrebbero appartenere a un corpo
   mobile annidato più in basso, rompendo la separazione.

**Nota sul menu di Isaac Sim**: se un prim è già dentro il sottoalbero di un altro RigidBody
(caso: hai messo il preset sul genitore per errore, ruolo 4 diventato ruolo 1), il menu rapido
smette di offrire "Rigid Body" sui figli — solo "Colliders Preset". Rimuovi il RigidBody dal
genitore (icona cestino nella sezione Physics → Rigid Body del Property panel) e riassegnalo ai
figli corretti, oppure usa gli script `physics_api_inspector.py` (verifica cosa manca) e
`physics_api_fixup.py` (applica RigidBody+Kinematic+Collider in blocco su una selezione multipla)
in `myProjects/Manipolazione/Scripts/` — lavorano sullo schema USD direttamente, bypassano quel
limite del menu.

## 7. Joint — cosa impostare, in ordine

Saltare un passo produce sintomi che sembrano appartenere a un altro problema (§10).

**1. Body0 e Body1.** Seleziona **Body0 poi Body1** → Create → Physics → Joint → tipo. Convenzione:
Body0 = il più "fisso"
Body1 = quello che si muove.

⚠️ Entrambi devono avere **RigidBodyAPI**. Un prim di ruolo 2 (solo Collider — es. `guide_panel`)
non è un corpo valido: PhysX risolve comunque al RigidBody dell'antenato più vicino (`base`). È
quello, non il figlio, che va indicato come Body.

**2. Asse — `Local Rotation 0` e `Local Rotation 1`.** L'asse attivo è sempre la **X locale** del
frame del Joint: "ruotare l'asse" significa ruotare quel frame finché la X locale punta nella
direzione giusta.

⚠️ **I due valori vanno tenuti identici.** Se ne cambi uno solo compaiono **due coni rossi
separati** nella visualizzazione, invece di uno: è il segnale che i due frame sono disallineati tra
loro e il comportamento diventa imprevedibile.

Il gizmo degli assi del **prim** resta fermo mentre modifichi `Local Rotation` — è corretto,
l'attributo appartiene al Joint e non ruota il corpo.

**3. Posizione** — `joint_aligner.py`. Seleziona **il Joint** (non i corpi) ed esegui. Scrive lo
stesso punto world in `localPos0`/`localPos1` (spazi locali diversi, stesso punto fisico → i due
frame coincidono, quella posa resta joint = 0). Non tocca mai l'origine di un prim: è la via giusta
per la posizione.

**4. Kinematic = False sul corpo mobile** (`physics_set_kinematic_false.py`). Body0 resta `True` se
è l'ancora statica.

⚠️ Con **entrambi** i corpi Kinematic PhysX rifiuta il giunto: *"cannot create a joint between
static bodies"*.

**5. Limiti**

**6. `physics:collisionEnabled`: lasciare `False`** (già di default). Vedi §9: non accenderlo.

**Verifica finale**: `physics_joint_inspector.py` stampa Body0/Body1, `collisionEnabled` e i limiti
effettivamente scritti sullo schema — senza fidarsi di ciò che mostra il Property panel.

### 7bis. Il gizmo del Joint nel viewport non è affidabile

Il disegno del Joint (frame, linee di anchor, cappucci dei limiti) è un debug-draw di PhysX che
**non si aggiorna in modo affidabile** dopo modifiche fatte via script — resta stantio, a volte
sparisce del tutto. Non interpretarlo per diagnosticare un allineamento: verifica sempre coi numeri.

- **`print_joint_anchor_world.py`** — anchor world di Body0 e Body1 letti da `localPos0/1` +
  trasformo mondo di ciascun body; il `Gap` stampato deve essere ~0 se l'allineamento è riuscito.
- **`print_joint_current_state.py`** (da eseguire in **Pausa**, mai in Stop — Stop resetta alla
  posa di riposo) — posizione attuale lungo l'asse, confrontata coi limiti configurati: dice se il
  corpo si è fermato *al* limite o *dentro* il range (nel qual caso non è il Joint a fermarlo).

Anche il pannello Property va letto con cautela: mostra **Local**, non World, e ogni pezzo CAD
porta la propria rotazione locale ereditata dal file originale — la stessa traslazione globale può
finire su assi locali diversi a seconda del body. Per leggere una posizione world affidabile:
**`print_prim_world_position.py`**.

### 7ter. Trovare un punto fisico preciso quando il Measure Tool non si aggancia

Lo snap-to-vertex del Measure Tool è inaffidabile su questa scena (anche col global Snap attivo).
Tecnica alternativa: piazza un marker (Xform/Sphere) **a occhio**, vicino al punto — non serve
precisione — poi seleziona **marker, poi mesh target**, ed esegui lo script adatto:

| Situazione | Script |
|---|---|
| Centro di un componente circolare senza mesh propria (perno, foro) | `find_circle_center.py` — baricentro dei vertici entro `RADIUS` dal marker |
| Superficie testurizzata (grip a piramidi, rigature) | `find_surface_plane_point.py` — fit a piano PCA sui vertici locali, la texture ripetuta si media via |
| Punto su una faccia piatta ampia | `find_surface_centroid.py` — centroide pesato per area delle facce entro `RADIUS` |

Le coordinate stampate vanno in `WORLD_POINT` di `joint_aligner.py` (`ANCHOR_MODE = "world_point"`)
o di `measure_joint_limit.py`.

⚠️ **Ogni volta che l'anchor cambia** (nuovo `joint_aligner.py`, o l'origine del prim è stata
ricentrata con `prim_center_origin.py`), **lo zero del Joint cambia** — i limiti già scritti non
sono più validi, vanno ri-misurati con `measure_joint_limit.py`.

### 7quater. Revolute Joint — differenze dal Prismatic

- **L'anchor deve stare sull'asse di rotazione reale**, quasi mai il centro bbox del corpo — misura
  **due punti circolari diversi lungo lo stesso perno/cuscinetto** con `find_circle_center.py` per
  ricavare la retta dell'asse vero, non un solo punto a caso sulla superficie.
- **I limiti sono in gradi**, non in distanza — `measure_joint_limit.py` (proietta su un asse
  lineare) non si applica; serve una variante angolare se il caso lo richiede.
- Il Drive lavora sugli stessi principi spring-damper del Prismatic, ma su gradi/gradi-al-secondo.

### 7quinquies. Drive — la forza dipende dal target di velocità, non solo dal damping

```
forza = stiffness × (target_position − posizione_attuale) + damping × (target_velocity − velocità_attuale)
```

Con `stiffness ≈ 0` (drive a sola velocità), la forza dipende **solo** da
`damping × (target_velocity − velocità_attuale)`: a velocità attuale ≈ 0 (corpo fermo/lento),
l'errore è quasi tutto il target — **un target di velocità più alto genera più forza**, non è un
tetto di velocità indipendente come su un motore reale. Un Drive che sembra "debole" nonostante un
damping alto può semplicemente avere un target troppo basso.

## 8. Limiti dei Joint — si dichiarano, non si misurano a contatto

⚠️ **Il metodo a collisione** (drive lento contro il fine-corsa fisico, lettura nel Joint
Inspector) **è stato abbandonato.** Verdetto di uno sviluppatore NVIDIA sul forum, per assiemi CAD
con molte mesh — un limite dichiarato è:

- un **vincolo del solver**: arresto esatto e deterministico, mentre un arresto a contatto varia
  con contact offset e penetrazione
- **immune al tunneling**: un vincolo non può essere "mancato" a velocità, un contatto sì
- a **costo ~zero**, contro la generazione di contatti a ogni step

Da dove prendere i valori, in ordine di affidabilità:

1. **Datasheet dei componenti commerciali.** Il CAD Converter conserva i codici articolo nei nomi
   delle mesh: `tn__3426250HGPL4040ABGEH` → Festo **HGPL-40-40**, corsa **40 mm per griffa**.
2. **CAD originale** — unica via per le parti custom (la rotaia qui ha un codice interno, non
   commerciale).
3. **`measure_rail_travel.py`** — seleziona **rotaia poi pattino**: stampa la corsa per asse e i
   `lowerLimit`/`upperLimit` già calcolati. L'asse di scorrimento è quello con la corsa grande.

⚠️ **Seleziona le due parti che si accoppiano davvero, non gli assiemi.** Dando tutto `right_fixed`
invece del solo pattino `tn__305437_`, il bounding box include pinza HGPL e cilindri montati sopra:
la corsa esce negativa o gonfiata.

⚠️ **Lo zero è la posa in cui il CAD è disegnato**, non l'estremo della rotaia. Gli stessi 40 mm di
corsa diventano `0/+40`, `-40/0` o `-20/+20` a seconda di dove il pezzo è disegnato. Due limiti
entrambi negativi non sono un errore: significano che il pezzo parte già quasi appoggiato a un
estremo.

## 9. ⚠️ Gli assiemi CAD sono modellati a contatto — la regola generale

**Il problema.** In un CAD ogni pezzo è disegnato a contatto con quelli su cui è montato: facce
combacianti, spesso con micro-compenetrazioni. PhysX non sa che quel contatto è intenzionale e
passa ogni step a cercare di separare le superfici → esplosione del numero di contatti. Su un
assieme con centinaia di mesh satura i buffer:

```
PxGpuDynamicsMemoryConfig::collisionStackSize buffer overflow detected,
please increase its size to at least 844606416 in the scene desc! Contacts have been dropped.
```

Il default è **64 MiB**: una richiesta da ~840 MB non significa "assieme grande", significa **una
coppia che si compenetra**. Se restringere gli SDF non fa scendere il numero in proporzione, il
costo è concentrato in una coppia sola.

E `Contacts have been dropped` ha una conseguenza visibile: i contatti scartati fanno
**attraversare** la geometria — sembra un collider mancante, e non lo è.

**La regola.** Ogni coppia di parti a contatto nel CAD va protetta, in uno dei due modi:

| Situazione | Protezione |
|---|---|
| C'è un Joint tra le due | `physics:collisionEnabled = False` sul Joint (**è già il default**) — copre i due sottoalberi |
| Non c'è (ancora) un Joint | **`FilteredPairsAPI`** sulla coppia — `physics_add_filtered_pair.py` |

`FilteredPairsAPI` dice a PhysX "tra questi due non generare mai contatti", ma **entrambi restano
collidibili con tutto il resto** — differenza cruciale rispetto a togliere `CollisionAPI`, che li
renderebbe fantasmi anche verso il pezzo da afferrare.

**Corollario controintuitivo**: la compenetrazione tra due parti così protette è **prevista e
innocua**. Vederla in viewport non è un bug e non va "risolta".

### Cos'è una "coppia", e come trovarle tutte

Una coppia è fatta di due **rigid body** (prim con `RigidBodyAPI`) — non due mesh, non due prim
qualsiasi — **montati l'uno sull'altro nel CAD**, e quindi disegnati a facce combacianti.

⚠️ Due parti che si **avvicinano durante il moto** non sono una coppia da filtrare: le griffe che
si chiudono sul pezzo devono collidere davvero. Si filtra solo ciò che è **assemblato insieme**.

**Le coppie da filtrare sono esattamente le righe del piano Joint.** Non è una coincidenza: un
Joint esiste proprio tra due parti meccanicamente montate una sull'altra, che è la definizione di
coppia accoppiata nel CAD.

Quindi **filtrale tutte in blocco all'inizio**, invece di scoprirle una alla volta sbattendo contro
il buffer a ogni Joint nuovo. Quando il Joint arriva, il suo `collisionEnabled = False` copre la
stessa coppia: il filtro diventa ridondante ma innocuo, non serve toglierlo.

Verifica la copertura con `physics_filtered_pairs_inspector.py`. L'API è asimmetrica nel modo in
cui è scritta (sta su un prim e punta all'altro) ma **filtra in entrambi i versi**: una coppia
elencata una volta è coperta.

**Per le coppie non previste a piano**: se dopo aver filtrato tutto il buffer esplode ancora,
esiste una sovrapposizione che non avevi messo a piano. Usa il **Robot Self-Collision Detector**
([tutorial 04](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/openusd_tuning_tutorials/tutorial_04_collider_pairs.html)),
che serve esattamente a trovare le coppie di link con geometria di collisione sovrapposta, invece
di indovinarle.

### Esempio — setup del gripper FANUC

Sequenza di setup che porta la simulazione a funzionare:

1. **`metersPerUnit = 1.0` + `Scale = 0.001`** sul root (§3) — causa di fondo: il modello girava a
   0,27 mm invece di 273 mm, e a quella scala le tolleranze di PhysX (contact offset, soglie di
   depenetrazione, tarate su corpi a scala metrica) leggono tutto come compenetrato
2. **Tutti i collider a Convex Hull** — `physics_set_convex_hull.py` su `components`, via l'SDF
3. **`collisionEnabled = False`** sul Joint `RightClawSlide` → copre `base`↔`right_fixed`
4. **Limiti** dalla misura rotaia `tn__501249981_` ↔ pattino `tn__305437_`: corsa 467,7 mm su Z
5. **`FilteredPairsAPI`** su `right_fixed`↔`right_forefinger` e `right_fixed`↔`right_thumb` — le
   coppie ancora senza Joint, tenute a contatto dagli adattatori HGPL
6. **GPU Dynamics riattivata**

## 10. Diagnostica — sintomo → causa

| Sintomo | Causa probabile |
|---|---|
| `collisionStackSize buffer overflow`, blocco **immediato** al Play | coppia CAD a contatto non protetta (§9) |
| Il corpo **attraversa** la geometria | contatti scartati per overflow; oppure la coppia è filtrata — e allora è **corretto** |
| Freeze di **minuti**, poi pochi mm di moto e posa storta | troppi collider SDF con GPU Dynamics spenta |
| `cannot create a joint between static bodies` | entrambi i corpi Kinematic=True (§7.4) |
| `invalid inertia tensor {1,1,1} and a negative mass` | RigidBody senza alcun Collider sotto (§6.1) |
| **Due coni rossi** sul Joint | `Local Rotation 0` ≠ `Local Rotation 1` (§7.2) |
| Freccia del Joint enorme, fuori dal modello | limiti sbagliati di ordine di grandezza (mm inseriti dove servono metri) |
| Modello "giusto" sulla griglia ma fisica assurda | la griglia è in unità di stage, non in metri (§3) |
| Compenetrazione che blocca una claw dopo il montaggio | gerarchia asimmetrica rispetto al lato gemello — mesh (es. la rotaia) finita su un RigidBody indipendente non protetto da Joint/FilteredPairsAPI (§12.1) |
| `"disjointed body transforms"` alla creazione di un Joint | `localPos0`/`localPos1` non coincidono in world, spesso perché Body0/Body1 presi dal prim sbagliato (§12.2) |
| `TF_PYTHON_EXCEPTION` / `AttributeError: 'NoneType' ... create_articulation_view` | Articulation Root presente ma `Enabled = False` (§12.4) |
| Slider del Physics Inspector non muovono il Joint | Drive con solo `damping`, `stiffness = 0` — nessuna forza verso una posizione (§12.5) |
| Revolute che gira senza fermarsi mai | nessun limite dichiarato + `target_velocity` residuo nel Drive (§12.6) |

Elenco e uso di tutti gli script: `myProjects/Manipolazione/Scripts/README.md`.

## 11. Robot Assembler — farsi riconoscere il gripper

Il Robot Assembler (`isaacsim.robot_setup.assembler`) **non** usa l'Articulation Root di PhysX per
popolare le sue tendine "Base Robot"/"Attach Robot" — cerca prim con la **Robot API** di Isaac Sim
(`isaacsim.robot.schema`, uno schema separato, non-PhysX):

```python
# da ui_builder.py, _find_all_robot_assets()
robots = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath("/"))
          if p.HasAPI(robot_schema.Classes.ROBOT_API.value)]
```

Senza questa API il gripper non compare mai nella tendina "Attach Robot": il campo resta `None`, e
cliccando "Begin Assemble" si ottiene `ArgumentError: Stage.GetPrimAtPath(Stage, NoneType)` da
`_attach_selection` — **non è un bug del Robot Schema, è la Robot API mancante**, l'errore è
fuorviante.

**Procedura, in ordine:**

1. **Articulation Root** sul prim radice del gripper (`Add → Physics → Articulation Root`) —
   prerequisito concettuale, ma **da solo non basta** a farlo comparire nel Robot Assembler.
2. **Robot API** sullo stesso prim radice: pannello Property → pulsante **"+ Add"** (quello
   generico, non una ricerca a parte) → categoria **Isaac → Robot Schema** → **Robot API**. Nel
   pannello compare come sezione intitolata **"Robot Schema"** (non "Robot API" — il titolo del
   widget e il nome dello schema non coincidono).
3. **Link API su ogni Link** (ogni prim con `RigidBodyAPI`: `right_fixed`, `right_thumb`, ecc.) e
   **Joint API su ogni Joint** (ogni prim `PhysicsPrismaticJoint`/`PhysicsRevoluteJoint`) — stesso
   percorso "+ Add → Isaac → Robot Schema", ma **individualmente su ciascun prim**, non in blocco
   dalla radice.
4. Dentro la sezione "Robot Schema" della radice, popolare le relationship **Robot Links** e
   **Robot Joints** coi picker dedicati ("Add Link"/"Add Joint") — questi picker mostrano **solo**
   prim che hanno già rispettivamente Link API o Joint API applicata (punto 3): se un Joint compare
   nel picker dei Link invece che in quello dei Joint, gli è stata applicata l'API sbagliata al
   punto 3 — va tolta (pulsante "−" nella sua sezione Robot Schema) e rimessa quella corretta.

⚠️ **Terminologia**: Link = corpo rigido (nodo del grafo cinematico); Joint = vincolo tra due Link
(arco del grafo) — stessa distinzione di `<link>`/`<joint>` in URDF.

Con questi 4 punti fatti, il gripper compare nella tendina "Attach Robot" e "Begin Assemble" non
crasha più.

**Gripper gigante dopo "Begin Assemble".** Causa: l'Assembler sovrascrive
`xformOpOrder` del prim allineato con un `xformOp:transform` nuovo a scala identità, perdendo lo
`Scale = 0.001` che compensa il CAD 1000× troppo grande (resta nel layer come attributo orfano,
non in `xformOpOrder`). `gripper.usd` standalone è corretto, si rompe solo in scena a ogni
riassemblaggio. Fix: dopo ogni Begin Assemble, in Property panel modalità matrice (non il toggle a
RST, azzera tutto), editare a mano la diagonale 4×4 da `1.0` a `0.001`.

**Limite noto — traslazione/rotazione errate nell'allineamento.** L'Assembler legge il
transform locale del Link scelto come punto di aggancio; quel Link ha origine traslata e rotazione
locale ~90° non a identità (stessa classe di problema risolta da `prim_zero_local_rotation.py`). Corretto a monte, l'Assembler allineerebbe da solo. Rifare
`joint_aligner.py` dopo.

## 12. Dal gripper montato al robot pronto per Isaac Lab

### 12.1 Compenetrazione di una claw dopo il montaggio

Le gerarchie delle due claw devono essere speculari. Un livello in più su un lato (es. `rotor`, assente
sul lato gemello) può raggruppare `finger_guide` come RigidBody indipendente, con la mesh della rotaia
che finisce lì invece che sul corpo `_fixed`: nessun Joint né `FilteredPairsAPI` protegge quel
contatto CAD → overflow di `collisionStackSize` (§9), letto come compenetrazione bloccata. Fix:
riportare la gerarchia speculare al lato gemello (stesso ruolo per ogni mesh, stessa coppia protetta).

### 12.2 Joint con "disjointed body transforms"

Warning alla creazione del Joint: `localPos0`/`localPos1` non coincidono nello stesso punto world.
PhysX allo start "scatta" i corpi per soddisfare il vincolo → compenetrazione/blocco violento.
Verifica prima **Body0/Body1** con `physics_joint_inspector.py` (spesso presi dal prim sbagliato,
conseguenza di una gerarchia come §12.1), poi ri-misura l'anchor con `joint_aligner.py`.

### 12.3 Physics Inspector, Drive e Articulation funzionano solo in Play

I Drive di PhysX applicano forza solo durante lo stepping; le operazioni dell'Inspector che
attraversano il grafo fisico (collegare il root joint, leggere l'Articulation) sono inconsistenti a
sim ferma. Non è un bug da inseguire: verificare/operare sempre in Play.

### 12.4 Articulation non riconosciuta — root joint/slider che non rispondono

Sintomo: `TF_PYTHON_EXCEPTION` su "collega root joint", oppure
`AttributeError: 'NoneType' object has no attribute 'create_articulation_view'` da
`isaacsim.core.prims.Articulation.initialize()`. Causa: Articulation Root presente sul prim ma con
**Enabled = False** — PhysX non costruisce nessuna ArticulationView per quel path. Fix: Property
panel → Physics → Articulation Root → `Enabled = True`. Verifica in Play:
```python
from isaacsim.core.prims import Articulation
art = Articulation("/path/della/radice")
art.initialize()
print(art.dof_names)
```
`dof_names` completo conferma che tutti i Joint sono dentro l'Articulation.

### 12.5 Slider del Drive inerti

Un Drive con solo `damping` (senza `stiffness`) applica forza solo come freno sulla velocità, non
tiene nessuna posizione — lo slider di posizione dell'Inspector non ha effetto visibile. Serve
`stiffness > 0` per un drive di posizione (formula in §7quinquies).

### 12.6 Revolute che gira senza fermarsi

Combinazione di due cause: nessun limite dichiarato (`enableLimit = False`, §8) e un
`target_velocity` rimasto diverso da zero nel Drive (drive di sola velocità, §7quinquies) — gira
indefinitamente. Fix: dichiarare i limiti angolari e azzerare `target_velocity` se il moto continuo
non è voluto, passando a un drive di posizione (`stiffness > 0` + `target_position` fisso).

### Checklist minima prima di Isaac Lab

- Ogni Joint mobile: limiti dichiarati (§8) + Drive con `stiffness > 0` che tiene la posa a riposo.
- Articulation Root **Enabled = True** su ogni radice (gripper e FANUC), `dof_names` completo via
  `Articulation.initialize()` in Play.
- Gerarchia delle claw speculare, nessun RigidBody isolato dal grafo dei Joint.
- Self-collision check (tutorial 04) e GPU Dynamics riattivata prima del training.
