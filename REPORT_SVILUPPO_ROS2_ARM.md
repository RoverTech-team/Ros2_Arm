> **Scopo del documento.** Riassumere in ordine logico il lavoro svolto, le principali decisioni implementative e i problemi risolti fino al 13/11/2026 (a grandi linee). La fase più recente ancora presenta dei problemi e c'è da fare debug. 

**2.1 — Modello fisico e collegamento al braccio.** È stato creato un modulo URDF/Xacro per simulare la camera **Orbbec Gemini 305**.  Dopo alcune prove con mesh STL/STEP, difficili da scalare e orientare, si è preferita una geometria semplificata a *box*. La camera è stata montata con un **joint fisso** vicino all'end-effector, in modo da seguire il movimento del braccio. Durante l'integrazione è stata individuata, per la configurazione testata, l'orientazione `rpy="0 -1.57 3.14"`: va mantenuta coerente con i frame effettivi e verificata prima di modificarla. La posizione della camera è tale da impedire la vista sulla parte superiore, essendo coperta dalle dita. Se la si montasse SOPRA Il polso, ssarebbe oscurata la parte inferiore. Ho scelto di montarla sotto perché così il pannello e i pulsanti sono visibili senza rischiare di portare i giunti verso posizioni scomode.

**2.2 — Frame ottico e sensore Gazebo.** Oltre al link fisico è stato definito un *optical frame*, perché le immagini usano la convenzione ottica (Z in avanti, X a destra, Y in basso), diversa da quella dei normali link ROS. Nel blocco `<gazebo>` del modello sono state aggiunte le caratteristiche del sensore RGB-D: risoluzione, frequenza, campo visivo, profondità e topic pubblicati. Nel mondo simulato è stato necessario abilitare il sistema di rendering/sensori (`gz-sim-sensors-system`, con renderer OGRE2), altrimenti un sensore descritto nel modello poteva non produrre immagini.

**2.3 — Collegamento Gazebo–ROS 2.** Il sensore pubblica dati nel sistema di trasporto di Gazebo; per renderli utilizzabili da nodi ROS 2 e RViz è stato configurato `ros_gz_bridge`.



## 2. Integrazione della camera RGB-D

| Topic ROS 2: I topic pubblicati dalla camera

| `/camera/rgbd/image` | Immagine utilizzata dal detector ArUco. |
| `/camera/rgbd/camera_info` | Parametri intrinseci della camera. |
| `/camera/rgbd/depth_image` | Mappa di profondità per le verifiche spaziali. |
| `/camera/rgbd/points` | Nuvola di punti, presente nelle prove di percezione 3D. |

Poi per sincronizzare tutto
| `/clock` | Tempo simulato condiviso dai nodi ROS 2. |

La parte di points con la octomap nel caso di aruco/depth etc non è necessaria. è stato fatto solo come esperimento iniziale per prendere mano.


## 3. Problemi di sincronizzazione temporale e TF

Durante l'integrazione della camera sono emersi problemi nella gestione delle trasformazioni TF2, dovuti a un disallineamento tra il tempo simulato di Gazebo (`/clock`) e il tempo reale utilizzato da alcuni nodi ROS 2.

Questo provocava errori di *extrapolation*, perché il sistema cercava trasformazioni relative a istanti temporali per i quali non erano disponibili dati nel buffer TF2.

Per aggirare il problema, è stata modificata la logica di interrogazione di TF2, richiedendo **l'ultima trasformazione disponibile** anziché quella corrispondente esattamente al timestamp dell'immagine.

Questi interventi hanno consentito di eseguire test più affidabili. Le modifiche sperimentali di configurazione riportate negli appunti non vanno considerate tutte necessariamente presenti nella versione corrente: la configurazione effettiva è quella dei file sorgente attualmente tracciati.

## 4. Inserimento dei marker ArUco sul pannello

Sono stati generati **due marker ArUco, ID 0 e 1**, appartenenti al dizionario OpenCV `DICT_5X5_250`. Le immagini `aruco_0.png` e `aruco_1.png` sono state collocate in `rover_arm_panel_description/materials/textures/`.

Nel modello `panel.urdf.xacro` sono stati aggiunti due link dedicati, fissati rigidamente al pannello. Ogni marker ha dimensioni **45 × 45 mm**, con spessore modellato di 1 mm; 

Per renderli visibili in Gazebo è stata usata una texture PBR tramite `<albedo_map>`. Inizialmente comparivano soltanto quadrati grigi: l'analisi di URDF→SDF e dei log ha mostrato che i riferimenti alle immagini arrivavano al simulatore, ma **Gazebo non riusciva a risolverne il percorso**. Un riferimento assoluto ai PNG ha permesso di verificare il corretto rendering, ed è una soluzione distribuibile. Avevo provato con Gz Resource path ma mi bloccava la simulazione per qualche motivo sconosciuto.

## 5. Riconoscimento visivo e interruzione della scansione

È stato sviluppato un nodo Python ROS 2, poi evoluto nello script `test_aruco_depth_search_center.py`, per elaborare le immagini della camera con **OpenCV**:

1. sottoscrizione a `/camera/rgbd/image`;
2. conversione di `sensor_msgs/Image` tramite `CvBridge`;
3. riconoscimento dei marker con `cv2.aruco`, usando lo stesso dizionario `DICT_5X5_250` dei PNG;
4. estrazione degli ID e delle coordinate 2D dei quattro angoli di ciascun marker;
5. visualizzazione diagnostica dei contorni e degli identificativi.

È stato risolto un problema iniziale di mancato riconoscimento causato dall'uso del dizionario `DICT_4X4_50`, diverso da quello dei marker generati. Sono state inoltre gestite differenze di API tra versioni di OpenCV.

La scansione del braccio è stata collegata al rilevamento: quando **entrambi gli ID 0 e 1** compaiono nello stesso frame, il detector pubblica `True` su `/panel/markers_detected`. Il nodo C++ usa questo evento per fermare la ricerca; durante lo sviluppo è stata introdotta anche una richiesta di cancellazione della traiettoria in corso tramite `async_cancel_goal()`. Il riconoscimento e la segnalazione sono stati osservati.

Per integrare i marker ArUco nella simulazione sono state utilizzate due immagini PNG, associate alle superfici del pannello tramite texture definite nel file `panel.urdf.xacro`.

Successivamente è stato sviluppato un nodo Python personalizzato, `test_aruco_depth_search_center.py`, che utilizza OpenCV (`cv2.aruco`) per individuare i marker nelle immagini RGB della camera.

Il rilevamento costituisce il punto di partenza per la stima della posa del pannello e per le successive operazioni di localizzazione dei suoi componenti.

## 6. Stima della posa del pannello con solvePnP e TF2

Il riconoscimento 2D è stato esteso alla stima 3D del pannello. Poiché il modello conosce la dimensione e la disposizione reciproca dei due marker, il detector costruisce **otto corrispondenze 3D–2D** (quattro angoli per ciascun ArUco) e utilizza `cv2.solvePnP()` con i parametri intrinseci della camera per stimare rotazione e traslazione del pannello nel frame ottico.

La stima viene riportata nel riferimento del rover tramite TF2:

\[
{}^{base\_link}T_{panel} = {}^{base\_link}T_{camera\_optical}\;{}^{camera\_optical}T_{panel}
\]

La fase di debug ha evidenziato due problemi distinti:

- **Sincronizzazione temporale.** Le richieste TF al timestamp dell'immagine generavano errori di extrapolazione, perché alcune trasformazioni e immagini usavano basi temporali differenti. Per le prove statiche è stata usata, provvisoriamente, l'ultima trasformazione disponibile (`Time()`); questa scelta non sostituisce una sincronizzazione corretta durante il movimento.
- **Coerenza dei frame e degli offset.** Le prime pose ottenute da solvePnP non coincidevano con le trasformazioni lette da TF2. È stata ispezionata la struttura di Gazebo e si è individuato, tra i problemi analizzati, **un offset di circa 1,5 m applicato due volte** tra modello e configurazione di lancio. Sono stati controllati anche l'orientazione del frame ottico e la distinzione tra origine del modello, `panel_base_frame` e frame TF del pannello.

Un errore di riproiezione piccolo (circa **0,63 pixel** in una prova) indicava buona coerenza delle proiezioni nell'immagine, **non** garantiva da solo che la posa nel frame del rover fosse corretta. La lezione fondamentale è verificare sempre le trasformazioni rispetto alla geometria effettivamente caricata nella scena.

## 7. Utilizzo della profondità per localizzare i controlli

Una volta stimata la posa del pannello, la geometria nota dell'URDF/Xacro permette di **prevedere** dove si trovano leve, interruttori e altri controlli. Le posizioni previste possono essere proiettate nell'immagine, mentre la **depth** permette di verificare e affinare la distanza degli elementi effettivamente osservati.

La pipeline sviluppata è arrivata a pubblicare coordinate tridimensionali dei controlli e a comunicare al nodo di movimento C++ la posizione del **main switch**. In una sessione di test, il target della leva principale nel frame `base_link` era circa **(0.677, −0.114, 0.177) m**. Questi valori sono **risultati di una specifica configurazione di simulazione**, non coordinate fisse da codificare per tutti gli scenari.

È importante distinguere i due contributi: **l'URDF fornisce la posizione nominale del controllo**, mentre l'immagine RGB-D fornisce osservazioni con cui stimarla o confermarla. La depth, da sola, non identifica semanticamente quale oggetto sia una leva.

## 8. Primo pre-avvicinamento automatico del braccio

La fase successiva consiste nel trasformare la posizione stimata del main switch in un target raggiungibile dal manipolatore. È stato inizialmente definito un **punto di pre-approach** a circa **8 cm dalla leva**, senza contatto fisico.

I primi tentativi con MoveIt 2/OMPL fallivano, con errori come `Unable to sample any valid states for goal tree`. Il debug ha riguardato le collisioni iniziali del modello semplificato, la compatibilità URDF/SRDF, i limiti dei giunti, la cinematica inversa e l'eccesso di vincoli sull'orientamento. Sono state provate anche richieste di IK solo posizionale (`position_only_ik`).

Un aspetto essenziale è la distinzione tra **`ee_base` (riferimento del polso)**, **`ee_finger1` (link del dito)** e **punta reale del dito**: pianificare il polso sul target non implica che il dito raggiunga quel punto. La compensazione dell'offset deve essere coerente con l'orientazione e con il TCP utilizzato.

Le note del **1° novembre 2026** documentano una **prima esecuzione completa del pre-approach automatico in Gazebo**, dopo i test separati di cinematica e pianificazione. Questo risultato non equivale ancora a una pressione affidabile del controllo: la fase successiva richiede un raffinamento della posa ravvicinata, la gestione della visibilità del target e il movimento di contatto.

## 9. Situazione attuale e perimetro della prossima relazione

### Risultati raggiunti

- Modello e flusso RGB-D della camera integrati nella simulazione.
- Marker ArUco presenti sul pannello e riconosciuti in tempo reale con OpenCV.
- Segnale di rilevamento collegato alla logica di scansione del braccio.
- Stima della posa tramite solvePnP e conversione dei riferimenti con TF2, dopo diversi interventi di debug.
- Utilizzo di geometria e profondità per ricavare le coordinate nominali/osservate dei controlli e inoltrare il target al sistema di movimento.
- Primo pre-approach automatico documentato in Gazebo.
- **Compilazione riuscita dei nove pacchetti** della copia di `Ros2_Arm` fuori dal vecchio workspace (test del novembre 2026), utile come primo controllo di portabilità (avevo fatto un casino con le repo).

### Da consolidare / approfondire separatamente

1. **Avvicinamento finale alla leva:** mantenerla nel campo visivo dopo il pre-approach; valutare eventuali nuove pose del polso/camera e un aggiornamento della stima mediante depth.
2. **Interazione fisica:** definire traiettoria di contatto coerente con l'articolazione della leva, comportamento della pinza e gestione delle collisioni; non è ancora documentata come operazione completata e robusta.
3. **Riproducibilità della repository:** eliminare ogni riferimento assoluto a cartelle personali, verificare l'installazione degli script Python con `CMakeLists.txt` e quella delle texture/mesh, e dichiarare le dipendenze necessarie. Una build riuscita non dimostra, da sola, che tutti i nodi e gli asset siano distribuibili.
4. **Robustezza temporale e cinematica:** usare TF coerenti col timestamp delle immagini e validare la posa dopo il movimento del braccio; distinguere le criticità correnti da quelle osservate soltanto nelle versioni precedenti.

## 10. Avvio della demo e verifiche essenziali

Dalla radice del workspace in cui è presente `src/`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Lancio utilizzato per la pipeline integrata:
1. prima demo incompleta 

ros2 launch rover_arm_panel_description prova_demo.launch.py use_gazebo:=true trajectory_action_topic:=/arm_controller/follow_joint_trajectory use_sim_time:=trueA

altro terminale:

python3 ./src/rover_arm_panel_description/scripts/test_aruco_depth_search_center.py

2. Seconda demo con pre approach
```bash
ros2 launch rover_arm_panel_description demo_rover_panel_depth.launch.py \
  use_gazebo:=true \
  auto_execute:=false \
  final_approach:=false \
  keep_visible:=true
```

I parametri qui disabilitano l'esecuzione automatica e l'avvicinamento finale **per un primo controllo prudente della scena**. Abilitare i movimenti solo dopo aver verificato che simulazione, controller, camera e trasformazioni siano coerenti. Il launch della pipeline integrata può già avviare il detector ArUco: evitare di lanciarne una seconda istanza inutilmente.

Comandi diagnostici ricorrenti:

```bash
ros2 topic list | grep -E 'camera|clock|panel'
ros2 run tf2_ros tf2_echo base_link gemini_camera_optical_frame
ros2 pkg prefix rover_arm_panel_description
```

Per distribuire il progetto, accertarsi che lo script di percezione sia **installato come eseguibile ROS 2** nel pacchetto: la presenza del file sotto `src/` non basta se il launch lo richiama tramite `Node(package=..., executable=...)`.

---

### Nota sullo stato delle informazioni

Questo report sintetizza gli appunti di sviluppo e il successivo test di compilazione/avvio. Non costituisce una revisione riga per riga dell'attuale codice Git: alcune modifiche riportate nei diari riguardavano file di prova o configurazioni poi sostituite. Per una guida di installazione definitiva andranno verificati sul commit finale i nomi degli eseguibili, le dipendenze, i percorsi delle risorse e il comportamento dei controller.



