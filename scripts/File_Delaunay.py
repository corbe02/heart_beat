import cv2
import numpy as np

class PointTracker:
    def __init__(self, movement_threshold=4.35, max_corners=1000, quality_level=0.000001, min_distance=50, show_mode=all):
        """
        Inizializza la classe per tracciare i punti, disegnare triangoli Delaunay e diagrammi di Voronoi.
        """
        self.movement_threshold = movement_threshold  # Soglia per classificare i punti come dinamici
        self.max_corners = max_corners  # Numero massimo di punti da rilevare
        self.quality_level = quality_level  # Livello minimo di qualità per i punti da rilevare
        self.min_distance = min_distance  # Distanza minima tra punti rilevati
        self.show_mode = "all"  # Modalità di visualizzazione iniziale ('all', 'lines', 'voronoi', 'points')

        # Apertura del video sorgente
        video_source = 'video2.1.mp4'  # Nome del file video
        self.cap = cv2.VideoCapture(video_source)  # Apre il file video
        ret, first_frame = self.cap.read()  # Legge il primo frame
        if not self.cap.isOpened() or not ret:  # Verifica che il video sia stato aperto correttamente
            print("Errore nell'apertura del video o nella lettura del primo frame")
            exit()

        # Dimensioni del frame
        self.height, self.width = first_frame.shape[:2]  # Altezza e larghezza del frame
        # Converte il primo frame in scala di grigi
        self.old_gray = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)
        # Rileva i punti di interesse nel primo frame
        self.p0 = cv2.goodFeaturesToTrack(self.old_gray, maxCorners=self.max_corners, qualityLevel=self.quality_level,
                                          minDistance=self.min_distance, blockSize=7)
        # Stato dei punti (False = statico, True = dinamico)
        self.point_states = np.zeros(self.p0.shape[0], dtype=bool)

    def update(self):
        """
        Aggiorna i punti tracciati, classifica i punti statici/dinamici e disegna in base alla modalità selezionata.
        """
        ret, frame = self.cap.read()  # Legge il frame successivo
        if not ret:  # Se non ci sono più frame, termina
            return False, frame

        # Converte il frame in scala di grigi per il tracciamento ottico
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Aggiorna la posizione dei punti tracciati tramite flusso ottico
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None) #ritorna il nuovo punto e st ==1 se il tracciamento ha avuto successo

        # Seleziona i punti tracciati correttamente
        good_new = p1[st == 1]  # Punti nel frame corrente
        good_old = self.p0[st == 1]  # Punti nel frame precedente
        self.point_states = self.point_states[st.ravel() == 1]  # Stato dei punti filtrato

        dynamic_points = []  # Lista per i punti dinamici (rossi)
        all_points = []  # Lista per tutti i punti (statici e dinamici)

        # Classifica i punti come statici o dinamici
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = map(int, new.ravel())  # Coordinate del punto corrente
            if 0 <= a < self.width and 0 <= b < self.height:  # Controlla se il punto è dentro i limiti del frame
                distance = np.linalg.norm(new - old)  # Calcola lo spostamento del punto
                if distance > self.movement_threshold:  # Se supera la soglia, il punto è dinamico
                    self.point_states[i] = True
                    dynamic_points.append((a, b))  # Aggiungi il punto alla lista dei dinamici

                # Disegna sempre i punti (rosso per dinamici, blu per statici)
                color = (0, 0, 255) if self.point_states[i] else (255, 0, 0)
                cv2.circle(frame, (a, b), 6, color, -1)

                all_points.append((a, b))  # Aggiungi il punto alla lista generale

        # Disegna in base alla modalità selezionata
        """La triangolazione di Delaunay per un insieme di punti nel piano è una suddivisione del piano in triangoli tali che nessun punto dell'
        insieme si trovi all'interno della circonferenza circoscritta di nessuno dei triangoli.
        La triangolazione di Delaunay è strettamente legata ai diagrammi di Voronoi:
        Ogni triangolo di Delaunay è il duale di una cella di Voronoi.
        I vertici di ogni triangolo di Delaunay sono i centri di celle adiacenti nel diagramma di Voronoi.
        """

        if self.show_mode in ["all", "lines"]:  # Linee (triangoli Delaunay)
            self.draw_triangles(frame, all_points, (255, 0, 0))
        if self.show_mode in ["all", "voronoi"]:  # Diagramma di Voronoi
            self.draw_voronoi(frame, all_points)

        # Aggiorna il frame precedente e i punti tracciati
        self.old_gray = frame_gray.copy()
        self.p0 = good_new.reshape(-1, 1, 2)

        return True, frame  # Restituisce il frame aggiornato

    def draw_triangles(self, frame, points, color):
        """
        Disegna triangoli Delaunay tra i punti forniti.
        """
        if len(points) < 3:  # Controlla se ci sono almeno 3 punti
            return

        rect = (0, 0, self.width, self.height)  # Definisci il rettangolo del frame
        subdiv = cv2.Subdiv2D(rect)  # Crea l'oggetto per la triangolazione

        # Inserisci i punti per la triangolazione
        for p in points:
            subdiv.insert(p)

        # Ottieni i triangoli Delaunay
        triangles = subdiv.getTriangleList()
        triangles = np.array(triangles, dtype=np.int32)  # Converte i triangoli in array di interi

        # Disegna i triangoli
        for t in triangles:
            pt1 = (t[0], t[1])  # Primo vertice del triangolo
            pt2 = (t[2], t[3])  # Secondo vertice del triangolo
            pt3 = (t[4], t[5])  # Terzo vertice del triangolo

            # Controlla se i vertici sono dentro i limiti del rettangolo
            if (
                self.in_rect(pt1, rect)
                and self.in_rect(pt2, rect)
                and self.in_rect(pt3, rect)
            ):
                # Disegna le linee blu tra i vertici
                cv2.line(frame, pt1, pt2, color, 1)
                cv2.line(frame, pt2, pt3, color, 1)
                cv2.line(frame, pt3, pt1, color, 1)

    def draw_voronoi(self, frame, points):
        """
        Disegna il diagramma di Voronoi per i punti forniti.
        """
        if len(points) < 2:  # Serve almeno 1 punto per creare Voronoi
            return

        rect = (0, 0, self.width, self.height)  # Rettangolo dei limiti del frame
        subdiv = cv2.Subdiv2D(rect)  # Crea l'oggetto Voronoi

        # Inserisci i punti per il diagramma di Voronoi
        for p in points:
            subdiv.insert(p)

        # Ottieni le facce di Voronoi
        (facets, centers) = subdiv.getVoronoiFacetList([])

        # Disegna ogni faccia di Voronoi
        for facet in facets:
            ifacet = np.array(facet, np.int32)  # Converte il poligono in array di interi
            cv2.polylines(frame, [ifacet], isClosed=True, color=(0, 255, 0), thickness=1)

    def in_rect(self, point, rect):
        """
        Controlla se un punto è dentro i limiti del frame.
        """
        return rect[0] <= point[0] < rect[2] and rect[1] <= point[1] < rect[3]

    def release(self):
        """
        Rilascia le risorse allocate.
        """
        self.cap.release()
        cv2.destroyAllWindows()


# Utilizzo della classe PointTracker
tracker = PointTracker()

# Ciclo principale per aggiornare e visualizzare il video
while True:
    success, frame = tracker.update()  # Aggiorna il frame
    if not success:
        break

    # Mostra il frame aggiornato
    cv2.imshow('Delaunay & Voronoi', frame)

    # Intercetta i tasti premuti
    key = cv2.waitKey(25) & 0xFF
    if key == ord('l'):  # Modalità linee (triangoli Delaunay)
        tracker.show_mode = "lines"
    elif key == ord('m'):  # Modalità Voronoi
        tracker.show_mode = "voronoi"
    elif key == ord('p'):  # Modalità solo punti
        tracker.show_mode = "points"
    elif key == ord('a'):  # Modalità tutto
        tracker.show_mode = "all"
    elif key in (ord('q'), 27):  # Premi 'q' o ESC per uscire
        break

tracker.release()  # Rilascia le risorse
