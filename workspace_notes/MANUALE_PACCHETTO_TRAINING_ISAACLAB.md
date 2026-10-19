# Struttura di `Apps/Training/`

> `Apps/Training/` è generato da `isaaclab.bat --new` (il codice del generatore è
> `IsaacLab/tools/template/cli.py`, che copia lo scheletro da `tools/template/templates/external/`).
> È un progetto Python **esterno** a Isaac Lab: non vive dentro il repo di Isaac Lab, lo *usa* come
> libreria installata. Gira sull'interprete Python di `C:\isaacsim` (via `isaaclab.bat`), non su
> pixi — pixi è un mondo completamente separato (ROS2 Jazzy), non c'entra qui.

## Le due macro-cartelle, e perché sono separate

```
Apps/Training/
├── scripts/         → programmi che LANCI tu da terminale
└── source/Training/ → il pacchetto Python vero e proprio (installato con `pip install -e`)
```

La separazione rispecchia una distinzione concettuale importante: **`scripts/` non fa parte del
pacchetto**. Sono semplici file `.py` con un `if __name__ == "__main__":`, pensati per essere
lanciati da terminale — non li importa mai nessun altro modulo. `source/Training/` invece è
*codice di libreria*: definisce classi e config che qualcun altro (uno script, o in futuro un altro
tuo progetto) importa con `import Training...`. Questa distinzione è la stessa che trovi in
qualunque pacchetto Python ben fatto: "entry point" vs "libreria riusabile".

## `scripts/` — punti di ingresso

| Script | Cosa fa concretamente | Quando ti serve |
|---|---|---|
| `random_agent.py` | ad ogni step campiona un'azione uniforme in [-1,1] per ogni giunto e la applica | validare che l'articolazione non esploda |
| `zero_agent.py` | come sopra ma l'azione è sempre 0 | debug: il robot resta esattamente nella posa di riposo dell'USD? Se si muove da solo con azione zero, il problema è nei drive/limiti, non nel training |
| `list_envs.py` | itera il registro Gymnasium e stampa una tabella di tutti i task registrati | sanity check dopo aver scritto `gym.register(...)`: il tuo task compare con il nome giusto? |
| `rsl_rl/train.py` | avvia un training con l'algoritmo PPO (libreria `rsl_rl`) | richiede una `RewardsCfg` con termini veri, non vuota |
| `rsl_rl/play.py` | carica una policy già allenata (checkpoint) e la esegue, senza aggiornarla | dopo un training, per vedere il risultato |

Forma del comando:
```
isaaclab.bat -p scripts/random_agent.py --task Move-To-Grasp-Apps-v0 --num_envs 16
```
`-p` dice a `isaaclab.bat` "esegui questo script Python con l'interprete giusto" — è l'equivalente
Windows di `./python.sh` nei tutorial Linux (stessa idea già vista per pixi con `pixi run python`).

## `source/Training/` — il pacchetto

Il nome del pacchetto è `Training` (con la maiuscola) — è per questo che vedi sempre
`import Training.tasks`, non `import training.tasks`. Se rinominassi la cartella, dovresti
rinominare anche tutti gli import corrispondenti (il template lo fa in automatico quando lo generi
con `--new <NomeProgetto>`, chiedendoti il nome).

```
source/Training/
├── pyproject.toml, setup.py, config/extension.toml   → metadati di packaging, MAI toccare
└── Training/                                          → il codice vero, questo è il pacchetto
    ├── ui_extension_example.py   → boilerplate per un pannello GUI dentro Kit (Isaac Sim interattivo)
    │                                IGNORALO: per RL headless con script da terminale non serve mai
    └── tasks/
        ├── __init__.py           → il meccanismo di auto-discovery, spiegato sotto
        └── manager_based/
            └── <nome_task>/      → UN task = UNA cartella qui dentro (qui: `move_to_grasp/`)
```

`ui_extension_example.py` esiste perché il template è pensato anche per creare estensioni Kit vere
e proprie (pannelli, tool visuali dentro l'editor) — un caso d'uso diverso dal nostro. Per il
training via script da terminale è morto, non verrà mai eseguito.

## Il meccanismo di auto-discovery — perché un task "esiste" senza essere registrato a mano da nessuna parte

Il meccanismo, passo per passo.

**Passo 1 — lo script fa un solo import apparentemente innocuo**:
```python
import Training.tasks  # noqa: F401
```
In Python, importare un pacchetto **esegue** il suo `__init__.py`. Il commento `# noqa: F401`
(che leggi in `random_agent.py`) dice al linter "so che sembra un import inutilizzato, è voluto" —
perché nessuna funzione di quel modulo viene mai chiamata esplicitamente: serve solo per l'*effetto
collaterale* dell'esecuzione del suo `__init__.py`.

**Passo 2 — `Training/tasks/__init__.py` contiene**:
```python
from isaaclab_tasks.utils import import_packages
_BLACKLIST_PKGS = ["utils", ".mdp"]
import_packages(__name__, _BLACKLIST_PKGS)
```
`import_packages` è una funzione di libreria di Isaac Lab (non nostra) che fa una cosa semplice:
**cammina ricorsivamente dentro la cartella corrente e importa ogni sottopacchetto che trova**,
escludendo quelli in blacklist (`mdp/` è escluso perché contiene solo funzioni di supporto, non
task; `utils/` idem). Non c'è nessuna lista scritta a mano da aggiornare quando crei un task nuovo.

**Passo 3 — importare la cartella di un task sotto `tasks/manager_based/` esegue il suo `__init__.py`**, che contiene:
```python
gym.register(id="<Nome-Task-v0>", kwargs={"env_cfg_entry_point": "<modulo>:<ClasseEnvCfg>"})
```
`gym.register` **non istanzia nulla**: scrive una riga in un dizionario globale di Gymnasium,
associando la stringa `"<Nome-Task-v0>"` a una *stringa di percorso* `"modulo:NomeClasse"` — non alla
classe stessa. La classe viene importata davvero solo più avanti, quando qualcosa chiama
`gym.make("<Nome-Task-v0>", ...)`.

**Conseguenza pratica**: per far "esistere" un nuovo task ti bastano due cose — (1) la cartella deve
stare sotto `manager_based/` (dove `import_packages` guarda), (2) il suo `__init__.py` deve
contenere `gym.register`. Nessun altro file dell'albero va toccato per "collegare" il task: non
esiste un registro centrale da aggiornare a mano.

## Anatomia minima di una cartella-task

| File | Obbligatorio? | Contiene |
|---|---|---|
| `__init__.py` | sì | `gym.register(...)` |
| `<nome>_env_cfg.py` | sì | la classe che eredita da `ManagerBasedRLEnvCfg` |
| `mdp/` | no | funzioni MDP custom del task — vedi sotto |
| `agents/rsl_rl_ppo_cfg.py` | no | iperparametri dell'algoritmo — vedi sotto |

### Cosa sono le "funzioni mdp"

MDP = *Markov Decision Process*, il formalismo matematico standard dell'RL (stato, azione, reward,
transizione). In Isaac Lab, "funzioni mdp" è il nome per le funzioni Python che calcolano i **pezzi**
del problema ad ogni step: un'osservazione (`joint_pos_rel`), un termine di reward
(`joint_pos_target_l2`), una condizione di terminazione (`time_out`), un evento di reset
(`reset_joints_by_offset`). Isaac Lab ne fornisce già decine di generiche in `isaaclab.envs.mdp` —
un task che le usa tutte così come sono può limitarsi a un `mdp/__init__.py` con `lazy_export()`
("ri-esporta tutto quello che c'è in `isaaclab.envs.mdp`"). Un task si crea una cartella `mdp/`
propria **quando ha bisogno di funzioni che non esistono già** — come `move_to_grasp`, con i suoi
reward, osservazioni, terminazioni, eventi e azioni a stadi (vedi `ARCHITETTURA_APPS_PREGRASP.md`).

### Cosa sono gli "iperparametri" (`agents/rsl_rl_ppo_cfg.py`)

Sono i numeri che configurano **l'algoritmo di apprendimento** (qui: PPO, via la libreria `rsl_rl`),
non l'ambiente. Esempi: quanto grande è il passo di aggiornamento della rete (`learning_rate`),
quanti step raccogliere prima di ogni update, la dimensione delle reti neurali actor/critic, quante
epoche per update. È un file completamente separato dalla `EnvCfg`: la stessa `EnvCfg` (lo stesso
ambiente fisico) può essere allenata con iperparametri diversi senza toccare l'ambiente, e lo stesso
algoritmo può allenare ambienti diversi. Serve solo per il training: `random_agent.py` e
`zero_agent.py` non usano nessun algoritmo.

## Anatomia di un `*EnvCfg`

```
EnvCfg
├── scene: SceneCfg          → cosa esiste nel mondo (robot, terreno, luci, oggetti)
│   └── robot: ArticulationCfg
│       ├── spawn            → quale file USD + quali proprietà fisiche sovrascrivere
│       ├── init_state        → posizione iniziale del robot + posa di riposo di ogni giunto
│       └── actuators         → guadagni PID per gruppi di giunti (vedi trappola sotto)
├── actions: ActionsCfg      → come i numeri della policy diventano comandi ai giunti
├── observations: ObsCfg     → cosa vede la policy ad ogni step
├── rewards: RewardsCfg      → può essere vuota se non si fa training
├── terminations: TermCfg    → quando finisce un episodio (timeout, caduta, ecc.)
└── events: EventCfg         → randomizzazioni/reset periodici — default sensato se lo ometti
```

⚠️ **`actuators` sovrascrive i drive già scritti nell'USD**, non li completa. PhysX puro (Isaac Sim
da solo) legge `stiffness`/`damping`/`maxForce` scritti nel file `.usd`; Isaac Lab, quando istanzia
l'`Articulation`, **rimpiazza** quei valori per ogni giunto con quelli che dichiari qui — è pensato
per poter randomizzare/tunare i guadagni da esperimento a esperimento senza riaprire l'USD ogni
volta. Se un giunto non compare in nessun gruppo di attuatori, resta con default impliciti (spesso
stiffness 0 → non tiene nessuna posizione, "colla al pavimento" al primo step).

## Cosa ignorare sempre

`.egg-info/`, `__pycache__/` (generati da `pip install -e`, si rigenerano da soli), `.vscode/`,
`.pre-commit-config.yaml`, `.dockerignore`, `.gitattributes`, `pyproject.toml`/`setup.py`
(infrastruttura standard del template) — non vanno modificati per definire un task.

## Trappola gitignore del template

`Apps/Training/.gitignore` ha una regola `**/__*` (pensata per scartare `__pycache__/`) che
**matcha anche `__init__.py`** — qualunque nuovo `__init__.py` in un task nuovo non viene visto da
`git add -A`. Va aggiunto esplicitamente col path completo se lo vuoi versionare.
