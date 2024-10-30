
def comparar_listas(lista1, lista2):
    """
    Compara dos listas de obstáculos y devuelve los elementos diferentes.
    """
    diferencias = []

    # Verifica cada elemento de lista1 que no esté en lista2
    # for elem in lista1:
    #     if elem not in lista2:
    #         diferencias.append(elem)

    # Verifica cada elemento de lista2 que no esté en lista1
    for elem in lista2:
        if elem not in lista1:
            diferencias.append(elem)

    return diferencias

# Ejemplo de uso
obs_rect_actual = [[(504, 460), (296, 460), (296, 740), (504, 740)],
                   [(902, 380), (798, 380), (798, 520), (902, 520)]]
obs_rect_nuevo = [[(504, 460), (296, 460), (296, 740), (504, 740)],
                  [(900, 380), (798, 380), (798, 520), (900, 520)]]  # Cambiado a propósito

obs_circ_actual = [[1000, 800, 100]]
obs_circ_nuevo = [[1000, 800, 180]]  # Cambiado a propósito

# Comparación
diferencias_rect = comparar_listas(obs_rect_actual, obs_rect_nuevo)
diferencias_circ = comparar_listas(obs_circ_actual, obs_circ_nuevo)

print("Diferencias en rectángulos:", diferencias_rect)
print("Diferencias en círculos:", diferencias_circ)

# Si hay diferencias, almacena en variables para su manejo
if diferencias_rect or diferencias_circ:
    print("Se encontraron diferencias, actualizando...")
    obs_rectangle = obs_rect_nuevo  # Actualiza a la nueva versión
    obs_circle = obs_circ_nuevo      # Actualiza los círculos

a = []

if a:
    print("si")
else:
    print("no")