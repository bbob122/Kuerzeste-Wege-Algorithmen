# -*- coding: utf-8 -*-
"""
Created on Fri Oct 18 18:02:42 2024

@author: ctholen
"""

from tkinter import *
from tkinter import filedialog
from functools import partial
from time import sleep
import time
from collections import deque
import random
import numpy as np
import json

    
def center_gui(root):
    windowWidth = root.winfo_reqwidth()
    windowHeight = root.winfo_reqheight()
    screenWidth = root.winfo_screenwidth()
    screenHeight = root.winfo_screenheight()

    # Setze die maximale Breite und Höhe
    max_width = min(screenWidth, windowWidth)
    max_height = min(screenHeight, windowHeight)

    positionRight = int(screenWidth / 2 - max_width / 2)
    positionDown = int(screenHeight / 2 - max_height / 2)

    root.geometry(f"{max_width}x{max_height}+{positionRight}+{positionDown}")


class App:
    
    update_frequency=10     # Parameter gibt an, wie oft die GUI geupdatet wird
    sleep_time=0.01         # Parameter gibt an, wie lange die Ausführung pausiert wird, wenn slow down true gesetzt wird
    
    
    def __init__(self, master):
       # Initialisiert das Hauptfenster der Anwendung und deren Eigenschaften 
       self.master = master
       master.wm_title("Pathfinding Algorithms")
       # Definiert die Eigenschaften des Grids
       self.cell_size = 20    # Größe jeder Zelle in Pixeln
       self.grid_size = 25    # Anzahl der Zellen in jeder Reihe und Spalte (25x25 Grid)
       
       # Initialisiert Listen für Start-, Ziel- und Hinderniskoordinaten
       self.start = []
       self.goal = []
       self.obstacles = []
       self.mode = 0  # Modus zur Bestimmung, welche Aktion durchgeführt wird (0: Start setzen, 1: Ziel setzen)
       
       # Hauptframe für Grid und Slider
       self.main_frame = Frame(master)
       self.main_frame.grid(row=0, column=0, sticky="nsew")
       master.grid_rowconfigure(0, weight=1)
       master.grid_columnconfigure(0, weight=1)
       
       # Frame für Save und Load button 
       self.load_save_frame = Frame(self.main_frame)
       self.load_save_frame.grid(row=0, column=0, columnspan=3, sticky="ew")
       
       # Frame für das Grid
       self.grid_frame = Frame(self.main_frame)
       self.grid_frame.grid(row=1, column=0, sticky="nsew")
       
       # Frame für die Slider
       self.slider_frame = Frame(self.main_frame)
       self.slider_frame.grid(row=1, column=2, sticky="n", padx=10)
       
       # Frame für die Buttons
       self.button_frame = Frame(self.main_frame)
       self.button_frame.grid(row=1, column=1, sticky="n", padx=10)
       
       # Frame für Reset und Clear Buttons 
       self.button_frame2 = Frame(self.main_frame)
       self.button_frame2.grid(row=2, column=0, columnspan=3, sticky="ew", pady=10)
       
       # Canvas für das Grid
       self.canvas = Canvas(self.grid_frame, width=self.cell_size*self.grid_size, height=self.cell_size*self.grid_size)
       self.canvas.grid(row=0, column=0)
      
       self.cells = [[None for _ in range(self.grid_size)] for _ in range(self.grid_size)]

        # Berechne die Position der einzelnen Zellen und zeichne sie ein
       for i in range(self.grid_size):
           for j in range(self.grid_size):
               x1, y1 = j * self.cell_size, i * self.cell_size
               x2, y2 = x1 + self.cell_size, y1 + self.cell_size
               self.cells[i][j] = self.canvas.create_rectangle(x1, y1, x2, y2, fill='white', outline='gray')
       
        # Bindet Mausereignisse zur Interaktion mit der Canvas (Klicks und Ziehen)
       self.canvas.bind('<Button-1>', self.on_canvas_click)   
       self.canvas.bind('<B1-Motion>', self.on_canvas_drag)
       
       # Definiert verschiedene Schaltflächen zur Interaktion mit dem Benutenden
       self.save_button = Button(self.load_save_frame, text="Save Configuration", command=self.save_configuration)
       self.save_button.pack(side=LEFT, padx=5)

       self.load_button = Button(self.load_save_frame, text="Load Configuration", command=self.load_configuration)
       self.load_button.pack(side=LEFT, padx=5)
   
       self.dijkstra_button = Button(self.button_frame, text="Run Dijkstra Algorithm", command=self.run_dijkstra)
       self.dijkstra_button.pack(pady=5) 
       
       self.a_star_button = Button(self.button_frame, text="Run A* Algorithm", command=self.run_a_star)
       self.a_star_button.pack(pady=5)
   
       self.bfs_button = Button(self.button_frame, text="Run BFS Algorithm", command=self.run_bfs)
       self.bfs_button.pack(pady=5)
   
       self.aco_button = Button(self.button_frame, text="Run ACO Algorithm", command=self.run_aco)
       self.aco_button.pack(pady=5)
   
       self.clear_path_button = Button(self.button_frame2, text="Clear Path", command=self.clear_path, state="disabled")
       self.clear_path_button.pack(side=LEFT, padx=5)
   
       self.reset_button = Button(self.button_frame2, text="Reset", command=self.reset)
       self.reset_button.pack(side=LEFT, padx=5)
       
       # # Checkboxen zur Steuerung der Geschwindigkeit während der Ausführung und des Nachbarschaftstyps
       self.slow_down = IntVar(value=1)
       self.slow_down_checkbox = Checkbutton(self.button_frame2, text="Slow down", variable=self.slow_down)
       self.slow_down_checkbox.pack(side=LEFT, padx=5)
       
       self.neighbourhood = IntVar(value=1)
       self.slow_down_checkbox = Checkbutton(self.button_frame2, text="von Neumann Neighbourhood", variable=self.neighbourhood)
       self.slow_down_checkbox.pack(side=LEFT, padx=5)

       # Erweiterungen für ACO-Parameter wie Anzahl der Ameisen und Pheromon-Einstellungen
       self.num_ants = 10
       self.num_iterations = 100
       self.alpha = 2.0
       self.beta = 1.0
       self.evaporation_rate = 0.2
       self.initial_pheromone = 0.1
       self.pheromone = np.full((self.grid_size, self.grid_size), self.initial_pheromone)   
       self.create_aco_sliders()
       
       # Abschließende Anpassungen zur Sicherstellung eines ordnungsgemäßen Layouts und zur Zentrierung der GUI.
       self.main_frame.grid_rowconfigure(1, weight=1)
       self.main_frame.grid_columnconfigure(0, weight=1)
       
       master.update_idletasks()
       center_gui(master)
        
        
    def save_configuration(self):
        # Speichert die aktuelle Konfiguration (Startpunkt, Zielpunkt, Hindernisse) in einer JSON-Datei
        config = {
            'start': self.start,
            'goal': self.goal,
            'obstacles': self.obstacles
        }
        filename = filedialog.asksaveasfilename(defaultextension=".json",
                                                filetypes=[("JSON files", "*.json")])
        if filename:
            with open(filename, 'w') as f:
                json.dump(config, f)

    def load_configuration(self):
        # Lädt eine Konfiguration aus einer JSON-Datei
        filename = filedialog.askopenfilename(filetypes=[("JSON files", "*.json")])
        if filename:
            with open(filename, 'r') as f:
                config = json.load(f)
            self.reset()
            self.start = config['start']
            self.goal = config['goal']
            self.obstacles = config['obstacles']
            self.update_grid_display()

    def update_grid_display(self):
        # Aktualisiert die Anzeige des Grids basierend auf dem aktuellen Status von Startpunkt, Zielpunkt und Hindernissen
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                if [i, j] == self.start:
                    self.canvas.itemconfig(self.cells[i][j], fill='green')
                elif [i, j] == self.goal:
                    self.canvas.itemconfig(self.cells[i][j], fill='red')
                elif [i, j] in self.obstacles:
                    self.canvas.itemconfig(self.cells[i][j], fill='black')
                else:
                    self.canvas.itemconfig(self.cells[i][j], fill='white')
        

    def enable_buttons(self):
        # Aktiviert alle interaktiven Schaltflächen in der GUI
        self.canvas.config(state="normal")

    def disable_buttons(self):
        # Deaktiviert alle interaktiven Schaltflächen im GUI während der Algorithmusausführung 
        self.canvas.config(state="disabled")
        self.clear_path_button.configure(state="normal")
        
    def create_aco_sliders(self):
        # Erstellt Slider zur Anpassung der ACO-Parameter (Ant Colony Optimization)
        Label(self.slider_frame, text="ACO Parameters:", font=("Calibri", 12, "bold")).grid(row=0, column=0, columnspan=2, pady=(0,10))
    
        parameters = [
            ("Number of Ants", "num_ants", 1, 50),
            ("Number of Iterations", "num_iterations", 10, 500),
            ("Alpha", "alpha", 0, 5),
            ("Beta", "beta", 0, 5),
            ("Evaporation Rate", "evaporation_rate", 0, 0.9),
            ("Initial Pheromone", "initial_pheromone", 0, 1)
        ]
    
        for i, (label, attr, min_val, max_val) in enumerate(parameters):
            Label(self.slider_frame, text=label).grid(row=i+1, column=0, sticky="e", padx=(0,5))
            scale = Scale(self.slider_frame, from_=min_val, to=max_val, orient=HORIZONTAL, 
                          resolution=0.1 if attr in ["alpha", "beta", "evaporation_rate", "initial_pheromone"] else 1,
                          length=150)
            scale.set(getattr(self, attr))    # Setzt den aktuellen Wert des Sliders auf den aktuellen Wert des Attributs
            scale.grid(row=i+1, column=1)     # Platziert den Slider im Grid-Layout
            scale.bind("<ButtonRelease-1>", partial(self.update_aco_param, attr))   # Bindet das Ereignis beim Loslassen der Maustaste an eine Funktion zur Aktualisierung des ACO-Parameter

    def update_aco_param(self, attr, event):
        # Aktualisiert den Wert eines ACO-Parameters, wenn der Benutzer den Slider bewegt
        setattr(self, attr, event.widget.get())


    def on_canvas_click(self, event):
        # Behandelt Klickereignisse auf der Canvas, um Start- und Zielpunkte festzulegen
        col = event.x // self.cell_size   # Berechnet die Spalte basierend auf der x-Position des Klicks
        row = event.y // self.cell_size   # Berechnet die Zeile basierend auf der y-Position des Klicks
        # Aktion basieren auf aktuellem Modus 
        if self.mode == 0:   # Setze Startpunk
            self.start = [row, col]
            self.canvas.itemconfig(self.cells[row][col], fill='green')
            self.mode = 1
        elif self.mode == 1:   # Setze Zielpunkt
            self.goal = [row, col]
            self.canvas.itemconfig(self.cells[row][col], fill='red')
            self.mode = 2
        elif self.mode == 2:   # Wechselt in den Modus für Hindernisse
            self.mode = 3
        else:
            self.mode = 2

    def on_canvas_drag(self, event):
        # Behandelt Ziehereignisse auf der Canvas, um Hindernisse hinzuzufügen
        if self.mode == 3:
            col = event.x // self.cell_size
            row = event.y // self.cell_size
            if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
                obstacle = [row, col]
                if obstacle not in self.obstacles:
                    self.obstacles.append(obstacle)
                    self.canvas.itemconfig(self.cells[row][col], fill='black')

    def heuristic(self, node1, node2):
        # Berechnet die heuristische Distanz zwischen zwei Knoten (Manhattan-Distanz)
       return abs(node1[0] - node2[0]) + abs(node1[1] - node2[1])
       

    def find_neighbors(self, current):
        # Findet benachbarte Knoten eines aktuellen Knotens basierend auf dem gewählten Nachbarschaftstyp
        neighbors = []        
        if self.neighbourhood.get():
            directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]   # Von Neumann-Nachbarschaft (4 Richtungen)
        else:        
            directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, 1), (-1, -1), (1, -1)]   # Moore-Nachbarschaft (8 Richtungen)
        
        
        for dx, dy in directions:
            neighbor = [current[0] + dx, current[1] + dy]
            if 0 <= neighbor[0] < self.grid_size and 0 <= neighbor[1] < self.grid_size and neighbor not in self.obstacles:
                neighbors.append(neighbor)    # Fügt benachbarte Knoten zur Liste hinzu
        return neighbors

    def sort_open_set(self, open_set, f_score):
        return sorted(open_set, key=lambda node: f_score[node[0]][node[1]])     # Sortiert die offenen Knoten basierend auf ihrem f_score-Wert für A* Algorithmus

    def reconstruct_path(self, cameFrom, current):
        # Rekonstruiert den Pfad von dem Ziel zurück zum Startpunkt
        total_path = []
        while current != self.start:
            self.canvas.itemconfig(self.cells[current[0]][current[1]], fill='yellow')
            total_path.append(current[:])
            current = cameFrom[current[0]][current[1]]
        return total_path

    def a_star_algorithm(self, start, goal):
        # Implementiert den A*-Algorithmus zur Pfadfindung zwischen Start und Ziel
        start_time = time.time()   # Startzeit für die Berechnung
        visited_cells = 0   # Zähler für besuchte Zellen
        open_set = [start]   # Liste der offenen Knoten, beginnend mit dem Startpunkt
        g_score = [[float('inf')] * self.grid_size for _ in range(self.grid_size)]   # Kosten vom Startpunkt zu jedem Knoten
        f_score = [[float('inf')] * self.grid_size for _ in range(self.grid_size)]   # Geschätzte Gesamtkosten vom Start zum Ziel
        came_from = [[[] for _ in range(self.grid_size)] for _ in range(self.grid_size)]   # Zurückverfolgung der Pfade

        g_score[start[0]][start[1]] = 0
        f_score[start[0]][start[1]] = self.heuristic(start, goal)
        
        step_count=0
        while open_set:            
            step_count+=1
            if self.slow_down.get():   # Überprüfe, ob die Ausführung verlangsamt werden soll
                sleep(self.sleep_time)
            
            if step_count % self.update_frequency ==0:   # Aktualisiere das GUI in regelmäßigen Abständen
                
                self.master.update_idletasks()
                

            current = self.sort_open_set(open_set, f_score)[0]   # Wähle den Knoten mit dem niedrigsten f_score aus
            if current == goal:   # Überprüfe ob das Ziel erreicht wurde 
                end_time = time.time()
                return self.reconstruct_path(came_from, current), end_time - start_time, visited_cells

            open_set.remove(current)   # Entferne den aktuellen Knoten aus der offenen Liste
            visited_cells +=1 #Increment visited cell counter 
            for neighbor in self.find_neighbors(current):   # Durchlaufe alle Nachbarn des aktuellen Knotens
                tentative_gScore = g_score[current[0]][current[1]] + 1     # Berechne vorläufigen g_score
                if tentative_gScore < g_score[neighbor[0]][neighbor[1]]:   # Wenn der neue Score besser ist als der alte
                    came_from[neighbor[0]][neighbor[1]] = current          # Setze den aktuellen Knoten als Vorgänger des Nachbarn
                    g_score[neighbor[0]][neighbor[1]] = tentative_gScore
                    f_score[neighbor[0]][neighbor[1]] = g_score[neighbor[0]][neighbor[1]] + self.heuristic(neighbor, goal)
                    if neighbor not in open_set:
                        self.canvas.itemconfig(self.cells[neighbor[0]][neighbor[1]], fill='blue')
                        open_set.append(neighbor)    # Füge den Nachbarn zur offenen Liste hinzu

        end_time = time.time()
        return None, end_time - start_time, visited_cells   # Wenn kein Weg gefunden wurde, gebe None zurück

    def dijkstra_algorithm(self, start, goal):
        start_time = time.time()
        visited_cells = 0
        open_set = [start]   # Liste der offenen Knoten, beginnend mit dem Startpunkt
        g_score = [[float('inf')] * self.grid_size for _ in range(self.grid_size)]
        came_from = [[[] for _ in range(self.grid_size)] for _ in range(self.grid_size)]

        g_score[start[0]][start[1]] = 0
        
        step_count=0
        while open_set:
            if self.slow_down.get():
                sleep(self.sleep_time)
            
            step_count+=1
            if step_count % self.update_frequency ==0:
                self.master.update_idletasks()

            current = min(open_set, key=lambda x: g_score[x[0]][x[1]])    # Wähle den Knoten mit dem niedrigsten g_score aus

            if current == goal:   # Überprüfe, ob das Ziel erreicht wurde
                end_time = time.time()
                return self.reconstruct_path(came_from, current), end_time - start_time, visited_cells

            open_set.remove(current)
            visited_cells +=1

            for neighbor in self.find_neighbors(current):
                tentative_gScore = g_score[current[0]][current[1]] + 1

                if tentative_gScore < g_score[neighbor[0]][neighbor[1]]:
                    came_from[neighbor[0]][neighbor[1]] = current
                    g_score[neighbor[0]][neighbor[1]] = tentative_gScore

                    if neighbor not in open_set:
                        self.canvas.itemconfig(self.cells[neighbor[0]][neighbor[1]], fill='blue')
                        open_set.append(neighbor)

        end_time = time.time()
        return None, end_time - start_time, visited_cells

    def bfs_algorithm(self, start, goal):
        start_time = time.time()
        visited_cells = 0
        queue = deque([[start]])    # Initialisiere eine Warteschlange mit dem Startpunkt
        visited = set([tuple(start)])    # Set zur Verfolgung besuchter Knoten
        
        step_count=0
        while queue:
            if self.slow_down.get():
                sleep(self.sleep_time)
            step_count+=1
            visited_cells +=1
            if step_count % self.update_frequency ==0:
                self.master.update_idletasks()
                

            path = queue.popleft()   # Nimm den ersten Knoten aus der Warteschlange
            node = path[-1]          # Der letzte Knoten im Pfad ist der aktuelle Knoten

            if node == goal:        # Wenn das Ziel erreicht wurde färbe den Pfad gelb 
                end_time = time.time()
                for cell in path[1:-1]:  # Skip start and goal
                    self.canvas.itemconfig(self.cells[cell[0]][cell[1]], fill='yellow')
                return path, end_time - start_time, visited_cells

            for neighbor in self.find_neighbors(node):    # Durchlaufe alle Nachbarn des aktuellen Knotens
                if tuple(neighbor) not in visited:        
                    visited.add(tuple(neighbor))          
                    new_path = list(path)
                    new_path.append(neighbor)
                    queue.append(new_path)
                    self.canvas.itemconfig(self.cells[neighbor[0]][neighbor[1]], fill='blue')

        end_time = time.time()
        return None, end_time - start_time, visited_cells

    def run_a_star(self):
        # Führt den A*-Algorithmus aus und zeigt das Ergebnis an
        path, runtime, visited_cells = self.a_star_algorithm(self.start, self.goal)
        self.show_result("A* Algorithm", runtime, path, visited_cells)
        self.disable_buttons()

    def run_dijkstra(self):
        # Führt den Dijkstra-Algorithmus aus und zeigt das Ergebnis an
        path, runtime, visited_cells = self.dijkstra_algorithm(self.start, self.goal)
        self.show_result("Dijkstra Algorithm", runtime, path, visited_cells)
        self.disable_buttons()

    def run_bfs(self):
        # Führt den BFS-Algorithmus aus und zeigt das Ergebnis an
        path, runtime, visited_cells = self.bfs_algorithm(self.start, self.goal)
        self.show_result("BFS Algorithm", runtime, path, visited_cells)
        self.disable_buttons()
        
        
    def aco_algorithm(self, start, goal):
        # Implementiert den Ant Colony Optimization (ACO) Algorithmus zur Pfadfindung
        start_time = time.time()   # Startzeit für die Berechnung
        visited_cells = set()
        best_path = None
        best_path_length = float('inf')    # Initialisiere die beste Pfadlänge mit unendlich
    
        for iteration in range(self.num_iterations):
            if self.slow_down.get():
                sleep(self.sleep_time)
            for ant in range(self.num_ants):
                path = self.construct_solution(start, goal)
                if path:
                    path_length = len(path)
                    if path_length < best_path_length:
                        best_path = path
                        best_path_length = path_length
                    self.update_pheromones(path, path_length)
                    visited_cells.update(tuple(cell) for cell in path)
    
            self.evaporate_pheromones()
            
            # Aktualisiere den maximalen Pheromonwert
            self.max_pheromone = np.max(self.pheromone)
            
            # Aktualisiere die Anzeige in jeder nten Iteration
            if iteration % self.update_frequency ==0:
                self.update_display(best_path, visited_cells)
                sleep(self.sleep_time) 
            
    
        end_time = time.time()
        return best_path, end_time - start_time, len(visited_cells)



    def construct_solution(self, start, goal):
        # Konstruiert eine Lösung (Pfad) vom Start- zum Zielpunkt für eine Ameise im ACO-Algorithmus
        current = start
        path = [current]
        visited = set([tuple(current)])

        while current != goal:
            neighbors = self.find_neighbors(current)    # Finde alle Nachbarn des aktuellen Knotens
            unvisited_neighbors = [n for n in neighbors if tuple(n) not in visited]   # Filtere die noch nicht besuchten Nachbarn
            
            if not unvisited_neighbors:
                # Wenn keine unbesuchten Nachbarn vorhanden sind, versuche zurückzugehen
                if len(path) > 1:
                    current = path[-2]  # Backtrack
                    path.pop()
                    continue
                else:
                    return None  # Kein Pfad gefunden

            probabilities = self.calculate_probabilities(current, unvisited_neighbors, goal)   # Berechne Wahrscheinlichkeiten für die Auswahl des nächsten Knotens
            next_node = random.choices(unvisited_neighbors, weights=probabilities)[0]    # Wähle den nächsten Knoten zufällig basierend auf den berechneten Wahrscheinlichkeiten

            path.append(next_node)
            visited.add(tuple(next_node))
            current = next_node

            if len(path) > self.grid_size * self.grid_size:  # Verhindere endlose Schleifen
                return None

        return path

    def calculate_probabilities(self, current, neighbors, goal):
        # Berechnet die Wahrscheinlichkeiten für die Auswahl des nächsten Knotens basierend auf Pheromon und Heuristik
        pheromone_values = [self.pheromone[n[0]][n[1]] for n in neighbors]   # Hole die Pheromonwerte für alle Nachbarn
        heuristic_values = [1 / (self.heuristic(n, goal) + 0.1) for n in neighbors]   # Berechne die heuristischen Werte (inverse Distanz zum Ziel)
        
        probabilities = [
            (pheromone ** self.alpha) * (heuristic ** self.beta)
            for pheromone, heuristic in zip(pheromone_values, heuristic_values)
        ]   # Berechne die Wahrscheinlichkeiten nach der ACO-Formel
        
        total = sum(probabilities)
        return [p / total for p in probabilities] if total > 0 else [1/len(neighbors)] * len(neighbors)    # Normalisiere die Wahrscheinlichkeiten oder verwende Gleichverteilung, wenn alle Werte Null sind

    def update_pheromones(self, path, path_length):
        # Aktualisiert die Pheromonwerte auf dem gefundenen Pfad
        for node in path:
            self.pheromone[node[0]][node[1]] += 1 / path_length   # Erhöhe den Pheromonwert umgekehrt proportional zur Pfadlänge

    def evaporate_pheromones(self):
        # Lässt die Pheromone auf dem gesamten Grid verdunsten
        self.pheromone *= (1 - self.evaporation_rate)   # Reduziere alle Pheromonwerte um den Verdunstungsrate-Faktor
        
        
    def pheromone_to_color(self, pheromone_level):
        # Normalisieren des Pheromonwerts auf einen Bereich von 0 bis 1
        normalized = min(pheromone_level / self.max_pheromone, 1)
        # Interpolieren zwischen Weiß (niedrig) und Rot (hoch)
        r = 255
        g = int(255 * (1 - normalized))
        b = int(255 * (1 - normalized))
        return f'#{r:02x}{g:02x}{b:02x}'


    def update_display(self, current_path, visited):
        #  Aktualisiert die Anzeige des Grids basierend auf den aktuellen Pheromonwerten und dem Pfad
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                ## Färbt jede Zelle entsprechend ihres Pheromonwerts ein, außer Start, Ziel und Hindernisse
                if [i, j] not in [self.start, self.goal] and [i, j] not in self.obstacles:
                    color = self.pheromone_to_color(self.pheromone[i][j])
                    self.canvas.itemconfig(self.cells[i][j], fill=color)
        
        for i, j in current_path:
            # Markiert den aktuellen besten Pfad in Gelb
            if [i, j] not in [self.start, self.goal]:
                self.canvas.itemconfig(self.cells[i][j], fill='yellow')
        
        self.master.update_idletasks()   # Aktualisiert die Anzeige
        


    def run_aco(self):
        #Führt den Ant Colony Optimization (ACO) Algorithmus aus
        self.pheromone = np.full((self.grid_size, self.grid_size), self.initial_pheromone)    # Initialisiert die Pheromonmatrix mit dem Anfangswert
        path, runtime, visited_cells = self.aco_algorithm(self.start, self.goal)   # Führt den ACO-Algorithmus aus
        self.show_result("ACO Algorithm", runtime, path, visited_cells)    # Zeigt das Ergebnis an
        self.disable_buttons()   # Deaktiviert die Buttons nach der Ausführung

    def show_result(self, algorithm_name, runtime, path, visited_cells):
        # Zeigt die Ergebnisse des ausgeführten Algorithmus in einem neuen Fenster an
        result_window = Toplevel(self.master)
        result_window.title(f"Result - {algorithm_name}")
        Label(result_window, text=f"Result - {algorithm_name}", font=("Calibri", 13), pady=10, padx=10).pack()
        Label(result_window, text=f"Runtime: {runtime:.4f} seconds", font=("Calibri", 13), pady=10, padx=10).pack()
        #Label(result_window, text=f"Visited Cells: {visited_cells}", font=("Calibri", 13), pady=10, padx=10).pack()
        
        path_length = len(path) if path else 'No path found'
        Label(result_window, text=f"Path length: {path_length}", font=("Calibri", 13), pady=10, padx=10).pack()
        
        
        
        
        Button(result_window, text="Close", command=result_window.destroy).pack(pady=5)
        center_gui(result_window)


    def clear_path(self):
        # Löscht den angezeigten Pfad und setzt die Zellen auf ihre ursprünglichen Farben zurück
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                # Setzt alle Zellen außer Start, Ziel und Hindernisse auf Weiß zurück
                if [i, j] not in [self.start, self.goal] and [i, j] not in self.obstacles:
                    self.canvas.itemconfig(self.cells[i][j], fill='white')
                if [i, j] in [self.goal]:
                    self.canvas.itemconfig(self.cells[i][j], fill='red')
        self.clear_path_button.configure(state="disabled")   # Deaktiviert den Clear Path Button, da der Pfad jetzt gelöscht ist
        self.mode = 2
        

    def reset(self):
        # Setzt das gesamte Grid und alle Variablen auf den Anfangszustand zurück
        # Löscht Start- und Zielpunkte sowie alle Hindernisse
        self.start = []
        self.goal = []
        self.obstacles = []
        self.mode = 0   # Setzt den Modus zurück, um einen neuen Startpunkt setzen zu können
        for i in range(self.grid_size):
            # Setzt alle Zellen auf Weiß zurück
            for j in range(self.grid_size):
                self.canvas.itemconfig(self.cells[i][j], fill='white')
        self.enable_buttons()
        self.clear_path_button.configure(state="disabled")   # Deaktiviert den Clear Path Button, da es keinen Pfad mehr gibt
        

if __name__ == '__main__':
    root = Tk()       # Erstellt das Hauptfenster
    app = App(root)   # Initialisiert die Anwendung
    root.mainloop()   # Startet die Hauptschleife der GUI
