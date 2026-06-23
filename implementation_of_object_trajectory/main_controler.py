import numpy as np
import time
import heapq

import yolo_detection 
import kinematika

REZOLUCIJA = 5 # mm
DIMENZIJA_PLOCE = 350 # mm
VELICINA_MATRICE = int(DIMENZIJA_PLOCE / REZOLUCIJA) # 35x35
VISINA_KRETANJA_Z = 0.0

def yolo_to_grid(x_mm, y_mm):
    x_poz = x_mm + (DIMENZIJA_PLOCE / 2)
    y_poz = y_mm + (DIMENZIJA_PLOCE / 2)
    col = max(0, min(VELICINA_MATRICE - 1, int(x_poz / REZOLUCIJA)))
    row = max(0, min(VELICINA_MATRICE - 1, int(y_poz / REZOLUCIJA)))
    return row, col

def grid_to_yolo(row, col):
    x_poz = col * REZOLUCIJA
    y_poz = row * REZOLUCIJA
    x_mm = x_poz - (DIMENZIJA_PLOCE / 2)
    y_mm = y_poz - (DIMENZIJA_PLOCE / 2)
    return x_mm, y_mm

def stvori_kartu_okoline(live_objects, start_name, end_name, sigurnosna_margina_mm=40):
    grid = np.zeros((VELICINA_MATRICE, VELICINA_MATRICE), dtype=int)
    margina_celija = int(sigurnosna_margina_mm / REZOLUCIJA)
    
    for ime_objekta, (x, y) in live_objects.items():
        if ime_objekta == start_name or ime_objekta == end_name:
            continue # Njih ne zaobilazimo
            
        r_centar, c_centar = yolo_to_grid(x, y)
        for r in range(r_centar - margina_celija, r_centar + margina_celija + 1):
            for c in range(c_centar - margina_celija, c_centar + margina_celija + 1):
                if 0 <= r < VELICINA_MATRICE and 0 <= c < VELICINA_MATRICE:
                    grid[r][c] = 1 
    return grid

def astar(grid, start, goal):
    # Jednostavna heuristika i kretanje u 8 smjerova
    def heuristika(a, b): return np.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2)
    neighbors = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristika(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            return data[::-1]
            
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            if 0 <= neighbor[0] < VELICINA_MATRICE and 0 <= neighbor[1] < VELICINA_MATRICE:
                if grid[neighbor[0]][neighbor[1]] == 1: continue
            else: continue
                
            tentative_g_score = gscore[current] + heuristika(current, neighbor)
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0): continue
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristika(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return False

# --- GLAVNI PROCES ---
if __name__ == "__main__":
    target_start = 'cat' # Objekt 1 koji spajamo (prilagodi iz YOLO klasa)
    target_end = 'clock'   # Objekt 2 koji spajamo

    print("1. Pokrećem YOLO viziju...")
    objects_on_table = yolo_detection.get_snapshot()

    if not objects_on_table:
        print("Prekinuto ili ništa nije nađeno.")
        exit()

    if target_start not in objects_on_table or target_end not in objects_on_table:
        print(f"Greška: Na stolu nisam pronašao i {target_start} i {target_end}.")
        exit()

    # 2. Planiranje putanje
    print("\n2. Planiram putanju i zaobilazim prepreke...")
    x_s, y_s = objects_on_table[target_start]
    x_e, y_e = objects_on_table[target_end]

    grid = stvori_kartu_okoline(objects_on_table, target_start, target_end)
    start_grid = yolo_to_grid(x_s, y_s)
    end_grid = yolo_to_grid(x_e, y_e)

    putanja_grid = astar(grid, start_grid, end_grid)

    if not putanja_grid:
        print("A* Algoritam ne može pronaći put! Prepreke blokiraju sve prolaze.")
        exit()

    putanja_mm = [grid_to_yolo(r, c) for (r, c) in putanja_grid]
    print(f"Putanja pronađena! Sadrži {len(putanja_mm)} točaka.")

    # 3. Izvršavanje na motorima
    print("\n3. Spajam se s Dynamixel motorima i izvršavam putanju...")
    portHandler, packetHandler = kinematika.inicijaliziraj_motore()
    
    try:
        for index, (x, y) in enumerate(putanja_mm):
            print(f"\n---> Korak {index+1}/{len(putanja_mm)}: X={x:.1f}, Y={y:.1f}")
            
            # Računanje inverzne kinematike za ovu točku
            try:
                q1_rad, q2_rad, q3_rad = kinematika.izracunaj_IK(x, y, VISINA_KRETANJA_Z)
            except ValueError as e:
                print(f"Upozorenje: Točka {index+1} izvan dohvata ruke: {e}. Preskačem je.")
                continue # Ovdje algoritam preskače nemoguću točku umjesto da pukne
                
            q1_deg = np.rad2deg(q1_rad)
            q2_deg = np.rad2deg(q2_rad)
            q3_deg = np.rad2deg(q3_rad)

            # Slanje na motore. Primijeti da sam smanjio pauzu na 0.5s kako bi kretanje
            # između bliskih točaka bilo brže i sličilo "crtanju"
            kinematika.pomakni_na_tocku(portHandler, packetHandler, q1_deg, q2_deg, q3_deg, pauza=0.5)
            
    finally:
        kinematika.ugasi_motore(portHandler, packetHandler)
        print("\nGotovo!")