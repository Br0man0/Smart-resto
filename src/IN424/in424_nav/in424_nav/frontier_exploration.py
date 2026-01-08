import numpy as np
import heapq
import math

# Constantes correspondant aux valeurs dans l'OccupancyGrid
OCCUPIED_SPACE_VALUE = 100
FREE_SPACE_VALUE = 0
UNKNOWN_SPACE_VALUE = -1

def get_neighbors(node, grid):
    """
    Retourne la liste des voisins accessibles pour un nœud donné (4-connexité).
    """
    x, y = node
    neighbors = []
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]:
            # On considère le voisin accessible si la cellule n'est pas marquée comme occupée.
            if grid[ny, nx] != OCCUPIED_SPACE_VALUE:
                neighbors.append((nx, ny))
    return neighbors

def heuristic(a, b):
    """
    Fonction heuristique (distance Euclidienne) pour A*.
    """
    return math.hypot(a[0] - b[0], a[1] - b[1])

def a_star_search(grid, start, goal):
    """
    Implémente l'algorithme A* sur la grille.
    Retourne une liste de cellules (indices de la grille) formant le chemin ou None s'il n'existe aucun chemin.
    """
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current_f, current = heapq.heappop(open_set)
        if current == goal:
            # Reconstruction du chemin
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for neighbor in get_neighbors(current, grid):
            tentative_g = g_score[current] + 1  # Coût constant pour un déplacement
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None

def detect_frontiers(grid):
    """
    Parcourt la grille pour détecter les frontiers.
    Une frontier est une cellule libre (FREE_SPACE_VALUE) ayant au moins un voisin inconnu (UNKNOWN_SPACE_VALUE).
    Retourne une liste de tuples (x, y) correspondant aux indices des cellules frontalières.
    """
    frontiers = []
    rows, cols = grid.shape
    for y in range(rows):
        for x in range(cols):
            if grid[y, x] == FREE_SPACE_VALUE:
                # Vérifier en 4-connexité
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < cols and 0 <= ny < rows:
                        if grid[ny, nx] == UNKNOWN_SPACE_VALUE:
                            frontiers.append((x, y))
                            break  # On ajoute la cellule une seule fois
    return frontiers
