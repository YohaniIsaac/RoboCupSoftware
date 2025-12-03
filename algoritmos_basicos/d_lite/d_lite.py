import pygame
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation


# Configuración inicial
WIDTH, HEIGHT = 1500, 900
GRID_SIZE = 20
GRID_WIDTH, GRID_HEIGHT = WIDTH // GRID_SIZE, HEIGHT // GRID_SIZE
FPS = 30

# Colores
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)


# Definición del nodo
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = float('inf')  # Costo de movimiento desde el inicio
        self.h = 0  # Costo heurístico al objetivo
        self.f = self.g + self.h  # Costo total
        self.parent = None
        self.is_obstacle = False

    def __lt__(self, other):
        return self.f < other.f

# Inicializa Pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("D* Lite Simulation")
clock = pygame.time.Clock()

# Crear la cuadrícula
grid = [[Node(x, y) for y in range(GRID_HEIGHT)] for x in range(GRID_WIDTH)]
start = grid[1][1]  # Nodo inicial
goal = grid[GRID_WIDTH - 2][GRID_HEIGHT - 2]  # Nodo objetivo

# Obstáculos fijos
def add_obstacle(x, y):
    for i in range(-1, 2):
        for j in range(-1, 2):
            if 0 <= x + i < GRID_WIDTH and 0 <= y + j < GRID_HEIGHT:
                grid[x + i][y + j].is_obstacle = True

# Añadir obstáculos fijos
add_obstacle(5, 5)
add_obstacle(10, 10)
add_obstacle(15, 15)
add_obstacle(20, 5)

# Inicializar el obstáculo móvil
moving_obstacle_pos = [10, 5]  # Posición inicial del obstáculo móvil
grid[moving_obstacle_pos[0]][moving_obstacle_pos[1]].is_obstacle = True

# D* Lite simplificado
def d_star_lite():
    open_set = []
    closed_set = set()
    start.g = 0
    start.f = start.g + start.h
    open_set.append(start)

    while open_set:
        current = min(open_set)
        if current == goal:
            return reconstruct_path(current)

        open_set.remove(current)
        closed_set.add(current)

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor_x = current.x + dx
            neighbor_y = current.y + dy
            if 0 <= neighbor_x < GRID_WIDTH and 0 <= neighbor_y < GRID_HEIGHT:
                neighbor = grid[neighbor_x][neighbor_y]
                if neighbor.is_obstacle or neighbor in closed_set:
                    continue

                tentative_g = current.g + 1
                if tentative_g < neighbor.g:
                    neighbor.g = tentative_g
                    neighbor.h = abs(goal.x - neighbor.x) + abs(goal.y - neighbor.y)
                    neighbor.f = neighbor.g + neighbor.h
                    neighbor.parent = current
                    if neighbor not in open_set:
                        open_set.append(neighbor)

    return []

# Reconstruir la ruta
def reconstruct_path(current):
    path = []
    while current is not None:
        path.append((current.x, current.y))
        current = current.parent
    return path[::-1]

# Dibujar la cuadrícula y la ruta
def draw_grid(path):
    screen.fill(WHITE)
    for x in range(GRID_WIDTH):
        for y in range(GRID_HEIGHT):
            node = grid[x][y]
            rect = pygame.Rect(x * GRID_SIZE, y * GRID_SIZE, GRID_SIZE, GRID_SIZE)
            if node.is_obstacle:
                pygame.draw.rect(screen, BLACK, rect)
            else:
                pygame.draw.rect(screen, WHITE, rect, 1)

    # Dibuja la ruta
    if path:
        for i in range(len(path) - 1):
            start_pos = (path[i][0] * GRID_SIZE + GRID_SIZE // 2, path[i][1] * GRID_SIZE + GRID_SIZE // 2)
            end_pos = (path[i + 1][0] * GRID_SIZE + GRID_SIZE // 2, path[i + 1][1] * GRID_SIZE + GRID_SIZE // 2)
            pygame.draw.line(screen, GREEN, start_pos, end_pos, 5)

        # Dibuja los puntos de la ruta
        for (x, y) in path:
            pygame.draw.circle(screen, GREEN, (x * GRID_SIZE + GRID_SIZE // 2, y * GRID_SIZE + GRID_SIZE // 2), 5)

    # Dibuja el nodo inicial y el objetivo
    pygame.draw.rect(screen, RED, pygame.Rect(start.x * GRID_SIZE, start.y * GRID_SIZE, GRID_SIZE, GRID_SIZE))
    pygame.draw.rect(screen, BLUE, pygame.Rect(goal.x * GRID_SIZE, goal.y * GRID_SIZE, GRID_SIZE, GRID_SIZE))
    
    # Dibuja el obstáculo móvil
    pygame.draw.rect(screen, YELLOW, pygame.Rect(moving_obstacle_pos[0] * GRID_SIZE, moving_obstacle_pos[1] * GRID_SIZE, GRID_SIZE, GRID_SIZE))

    pygame.display.flip()


# Juego principal
running = True
i = 0 
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Mover el obstáculo móvil con las teclas WASD
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w] and moving_obstacle_pos[1] > 0:  # Arriba
        grid[moving_obstacle_pos[0]][moving_obstacle_pos[1]].is_obstacle = False
        moving_obstacle_pos[1] -= 1
        grid[moving_obstacle_pos[0]][moving_obstacle_pos[1]].is_obstacle = True
    if keys[pygame.K_s] and moving_obstacle_pos[1] < GRID_HEIGHT - 1:  # Abajo
        grid[moving_obstacle_pos[0]][moving_obstacle_pos[1]].is_obstacle = False
        moving_obstacle_pos[1] += 1
        grid[moving_obstacle_pos[0]][moving_obstacle_pos[1]].is_obstacle = True
    if keys[pygame.K_a] and moving_obstacle_pos[0] > 0:  # Izquierda
        grid[moving_obstacle_pos[0]][moving_obstacle_pos[1]].is_obstacle = False
        moving_obstacle_pos[0] -= 1
        grid[moving_obstacle_pos[0]][moving_obstacle_pos[1]].is_obstacle = True
    if keys[pygame.K_d] and moving_obstacle_pos[0] < GRID_WIDTH - 1:  # Derecha
        grid[moving_obstacle_pos[0]][moving_obstacle_pos[1]].is_obstacle = False
        moving_obstacle_pos[0] += 1
        grid[moving_obstacle_pos[0]][moving_obstacle_pos[1]].is_obstacle = True

    path = d_star_lite()
    if i == 0:
        draw_grid(path)
        print(path)

    i = 100
    clock.tick(FPS)

pygame.quit()




