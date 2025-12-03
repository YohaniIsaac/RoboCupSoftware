import pygame
import math

# Inicializar Pygame
pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Simulación de Figura Compuesta")
clock = pygame.time.Clock()

# Crear una superficie para la figura compuesta
figure_surface = pygame.Surface((1080, 720), pygame.SRCALPHA)
figure_surface.fill((0, 0, 0, 0))  # Hacer la superficie transparente

# Dibujar las figuras geométricas en la superficie
# Dibujar dos círculos (ruedas)
pygame.draw.circle(figure_surface, (0, 0, 255), (50, 50), 20)
pygame.draw.circle(figure_surface, (0, 0, 255), (150, 50), 20)

# Dibujar un rectángulo (cuerpo)
pygame.draw.rect(figure_surface, (255, 0, 0), pygame.Rect(40, 40, 120, 20))

# Posición inicial de la figura
figure_x, figure_y = 400, 300
angle = 0  # Ángulo inicial de rotación
negro = (0,0,0)



            arucoTag_img = pygame.image.load("arucoMarkers/" + path)
            self.arucoTag_img = pygame.transform.scale(arucoTag_img, (35,35))


x = 300
y = 300
running = True
while running:
    dt = clock.tick(60) / 1000  # Tiempo delta en segundos

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Controlar la rotación
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        angle += 180 * dt  # Rotar a la izquierda
    if keys[pygame.K_RIGHT]:
        angle -= 90 * dt  # Rotar a la derech
    f_separacion = 10
    radio = 30
    pygame.draw.circle(figure_surface, negro, (x - f_separacion, y), radio)
    pygame.draw.circle(figure_surface, negro, (x + f_separacion, y), radio)
    pygame.draw.rect(figure_surface, negro, (x-f_separacion, y-radio,2*f_separacion,2*radio))

    p1 = (x + f_separacion + radio , y - 6 ) 
    p2 = (x + f_separacion + radio + 6, y - 6 )
    p3 = (x + f_separacion + radio, y)
    pygame.draw.polygon(figure_surface, (255,0,0), [p1,p2,p3])

    # Limpiar la pantalla
    screen.fill((255, 255, 255))

    # Rotar la superficie de la figura
    rotated_surface = pygame.transform.rotate(figure_surface, angle)
    rotated_rect = rotated_surface.get_rect(center=(figure_x, figure_y))

    # Dibujar la figura compuesta rotada
    screen.blit(rotated_surface, rotated_rect.topleft)

    pygame.display.flip()

pygame.quit()
