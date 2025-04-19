import pygame
import math


def detectar_colisiones(objetos):
    colisiones_procesadas = set()  # Conjunto para evitar la doble verificación de colisiones

    # Iterar sobre cada par de objetos
    for i in range(len(objetos)):
        for j in range(i + 1, len(objetos)):  # Empezamos desde i+1 para evitar dobles verificaciones
            obj1 = objetos[i]
            obj2 = objetos[j]

            # Si cualquiera de los objetos es un robot y está sosteniendo la pelota, omitir la colisión
            if (obj1.identificador != 0 and obj1.ball_hold) or (obj2.identificador != 0 and obj2.ball_hold):
                continue

            # Crear un identificador único para este par de objetos
            par_objetos = tuple(sorted([id(obj1), id(obj2)]))

            # comprobar cooldown
            if obj1.cooldown > 0 or obj2.cooldown > 0:
                continue  # Si cualñquiera de los objetos está en cooldown, salta la verificacion

            if par_objetos not in colisiones_procesadas:
                # Verificar colisión entre obj1 y obj2
                check_collision(obj1, obj2)
                # Marcar la colisión como procesada
                colisiones_procesadas.add(par_objetos)
    for obj in objetos:
        if obj.cooldown > 0:
            obj.cooldown -= 1  # Disminuir el cooldown en cada frame


def check_collision(obj1, obj2):
    mask1 = pygame.mask.from_surface(obj1.img_robot_rotated)
    mask2 = pygame.mask.from_surface(obj2.img_robot_rotated)

    # Obtener los offsets entre las dos imagenes
    offset = (obj2.rotated_rect.left - obj1.rotated_rect.left, obj2.rotated_rect.top - obj1.rotated_rect.top)

    # Comprobar la colision usando la máscara
    collision = mask1.overlap(mask2, offset)
    if collision:
        # Calcular la dirección del golpe
        direccion = (obj2.x - obj1.x, obj2.y - obj1.y)

        # Calcular la distancia
        distancia = math.sqrt(direccion[0]**2 + direccion[1]**2)

        # Normalizar la dirección
        direccion_normal = (direccion[0] / distancia, direccion[1] / distancia)
        ####
        # Velocidad relatica en la direccion de la colision
        velocidad_relativa_x = obj2.dx - obj1.dx
        velocidad_relativa_y = obj2.dy - obj1.dy
        velocidad_normal = velocidad_relativa_x * direccion_normal[0] + velocidad_relativa_y * direccion_normal[1]

        # Si la velocidad normal es mayor a 0, los objetos ya se están separando, no se aplica impulso
        if velocidad_normal > 0:
            return

        # Coef. de restitución (elasticidad de la colision)
        e = 1

        # Cálculo de impulso escalar
        j = -(1+e) * velocidad_normal
        j /= (1 / obj1.masa + 1 / obj2.masa)

        impulso_x = j * direccion_normal[0]
        impulso_y = j * direccion_normal[1]

        # Actualizar las velocidades de los obejtos en funcion de sus masas
        obj1.dx -= impulso_x / obj1.masa
        obj1.dy -= impulso_y / obj1.masa
        obj2.dx += impulso_x / obj2.masa
        obj2.dy += impulso_y / obj2.masa

        # Ajustar posiciones para evitar superposición
        # Calcular la distancia de superposición (superposition_distance)7 días anteriores
        superposition_distance = (obj1.rotated_rect.width / 2 + obj2.rotated_rect.width / 2) - distancia
        factor_suavizado = 0.7
        if superposition_distance > 0:
            # Mover los objetos en direcciones opuestas para separarlos
            correction_x = direccion_normal[0] * superposition_distance * factor_suavizado
            correction_y = direccion_normal[1] * superposition_distance * factor_suavizado

            # Congelar el objeto en el límite de colisión para evitar superposición
            obj1.x -= correction_x / 2
            obj1.y -= correction_y / 2
            obj2.x += correction_x / 2
            obj2.y += correction_y / 2

        # Actualiza cooldown
        obj1.cooldown = 8
        obj2.cooldown = 8

        # Actualizar los rectángulos rotados después de mover los objetos
        obj1.rotated_rect = obj1.img_robot_rotated.get_rect(center=(obj1.x, obj1.y))
        obj2.rotated_rect = obj2.img_robot_rotated.get_rect(center=(obj2.x, obj2.y))
