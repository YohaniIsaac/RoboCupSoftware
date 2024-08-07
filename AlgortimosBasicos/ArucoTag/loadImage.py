import pygame
import os 
import time


def check_collision(img1, rect1, img2, rect2):
    # Crear una máscara para la primera imagen
    mask1 = pygame.mask.from_surface(img1)
    mask2 = pygame.mask.from_surface(img2)

    # Obtener los offsets entre las dos imágenes
    offset = (rect2.left - rect1.left, rect2.top - rect1.top)

    # Comprobar la colisión usando la máscara
    collision = mask1.overlap(mask2, offset)

    return collision is not None




# Inicializar Pygame
pygame.init()

# Configuración de la pantalla
screen_width = 800
screen_height = 600
screen = pygame.display.set_mode((screen_width, screen_height))

# Definir el color verde
green = (0, 255, 0)

########################################################################

# Construir la ruta de la imagen
current_dir = os.path.dirname(__file__)  # Directorio del script
image_path = os.path.join(current_dir, '..', '..', '..','Programacion', 'arucoMarkers', 'robot_1.png')

# Cargar la imagen
image = pygame.image.load(image_path)  # Reemplaza con la ruta a tu imagen

original_width, original_height = image.get_size()
# Definir el nuevo ancho o alto
new_width = 50  # Por ejemplo, nuevo ancho deseado

# Calcular el nuevo alto para mantener la relación de aspecto
aspect_ratio = original_width / original_height
new_height = int(new_width / aspect_ratio)

image = pygame.transform.scale(image, (new_width, new_height))


robot = image.get_rect(topleft=(130, 200))  # Centra la imagen en la pantalla

########################################################

obs_path = os.path.join(current_dir, '..', '..', '..','Programacion', 'arucoMarkers', 'robot.png')

# Cargar la imagen
obs = pygame.image.load(obs_path)  # Reemplaza con la ruta a tu imagen

original_width, original_height = obs.get_size()
# Definir el nuevo ancho o alto
new_width = 50  # Por ejemplo, nuevo ancho deseado

# Calcular el nuevo alto para mantener la relación de aspecto
aspect_ratio = original_width / original_height
new_height = int(new_width / aspect_ratio)

obs = pygame.transform.scale(obs, (new_width, new_height))
obs_rotate =  pygame.transform.rotate(obs, 90)
obs_rect = obs_rotate.get_rect(topleft=(200, 200))



# Variables para el movimiento
robot_speed = 300  # píxeles por segundo
last_time = time.time()  # tiempo del último fotograma

        # rotated_image = pygame.transform.rotate(self.arucoTag_img, -self.angulo)
        # rotated_rect = rotated_image.get_rect(center=(self.x, self.y))
        # fondo.blit(rotated_image, rotated_rect)

# Ciclo principal
running = True
while running:
    current_time = time.time()
    delta_time = current_time - last_time  # tiempo transcurrido desde el último fotograma
    last_time = current_time
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Rellenar la pantalla con color verde
    screen.fill(green)

    # Actualizar posición del robot
    robot.x += robot_speed * delta_time  # Movimiento de ejemplo

    # Comprobar colisión
    if check_collision(image, robot, obs_rotate, obs_rect):
        print("Colisión detectada!")

    # Dibujar imágenes
    screen.blit(image, robot.topleft)
    screen.blit(obs_rotate, obs_rect.topleft)


    # Actualizar la pantalla
    pygame.display.flip()

# Salir de Pygame
pygame.quit()
