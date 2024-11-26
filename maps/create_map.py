import numpy as np
import pandas as pd
from PIL import Image
import random

def load_heightmap(image_path):
    """
    Carga un heightmap en escala de grises y lo convierte en un array NumPy.
    
    Args:
        image_path (str): Ruta al archivo de heightmap.
    
    Returns:
        np.ndarray: Representación del heightmap como matriz NumPy.
    """
    image = Image.open(image_path).convert("L")  # Convertir a escala de grises
    heightmap = np.array(image, dtype=np.float32)  # Convertir a float para cálculos
    return heightmap

def adjust_heightmap(heightmap):
    """
    Ajusta el heightmap restando 100 y convirtiendo de centímetros a metros.
    
    Args:
        heightmap (np.ndarray): Matriz del heightmap cargada desde una imagen.
    
    Returns:
        np.ndarray: Heightmap ajustado con base en 0 y en metros.
    """
    adjusted_heightmap = (heightmap - 100) / 10  # Restar 100 y convertir a metros
    return adjusted_heightmap

def generate_maps(heightmap):
    """
    Genera mapas de transitabilidad y alturas a partir del heightmap.
    
    Args:
        heightmap (np.ndarray): Matriz del heightmap.
    
    Returns:
        tuple: Mapa binario de transitabilidad (todos 1), mapa de alturas (Z).
    """
    # Mapa binario de transitabilidad (1 para todas las celdas)
    traversability_map = np.ones_like(heightmap, dtype=int)
    
    # Mapa de alturas (Z)
    z_map = heightmap
    
    return traversability_map, z_map


def generate_interest_points(heightmap, num_points=1):
    """
    Genera puntos de interés en el mapa basados en alturas máximas, mínimas, o aleatorias.
    
    Args:
        heightmap (np.ndarray): Matriz del heightmap.
        num_points (int): Número de puntos de interés a generar.
    
    Returns:
        list: Lista de puntos de interés como tuplas (x, y, altura).
    """
    points = []

    # Encontrar alturas máxima y mínima
    max_height = np.max(heightmap)
    min_height = np.min(heightmap)

    # Puntos aleatorios dentro del rango del mapa
    for _ in range(num_points):
        x = random.randint(0, heightmap.shape[1] - 1)  # Columna
        y = random.randint(0, heightmap.shape[0] - 1)  # Fila
        z = heightmap[y, x]  # Altura en ese punto
        points.append((x, y, z))

    print(f"Puntos de interés generados: {points}")
    return points

def save_interest_points(points, output_file):
    """
    Guarda los puntos de interés en un archivo CSV.
    
    Args:
        points (list): Lista de puntos de interés como tuplas (x, y, altura).
        output_file (str): Ruta del archivo de salida.
    """
    points_df = pd.DataFrame(points, columns=["X", "Y", "Altura"])
    points_df.to_csv(output_file, index=False,header=False,sep= ' ')
    print(f"Puntos de interés guardados en {output_file}")

def visualize_interest_points(heightmap, points):
    """
    Visualiza el mapa con los puntos de interés marcados.
    
    Args:
        heightmap (np.ndarray): Matriz del heightmap.
        points (list): Lista de puntos de interés como tuplas (x, y, altura).
    """
    plt.figure(figsize=(8, 8))
    plt.title("Puntos de Interés en el Mapa")
    plt.imshow(heightmap, cmap="viridis", interpolation="nearest")
    plt.colorbar(label="Altura (m)")

    # Marcar puntos de interés
    for (x, y, z) in points:
        plt.scatter(x, y, color="red", label=f"({x}, {y}, {z:.2f})")
    plt.axis("off")
    plt.show()


def save_maps_to_csv(traversability_map, z_map, output_prefix):
    """
    Guarda los mapas de transitabilidad y alturas como archivos CSV.
    
    Args:
        traversability_map (np.ndarray): Mapa binario de transitabilidad.
        z_map (np.ndarray): Mapa de alturas (Z).
        output_prefix (str): Prefijo para los archivos de salida.
    """
    # Guardar mapa de transitabilidad
    traversability_df = pd.DataFrame(traversability_map)
    traversability_df.to_csv(f"{output_prefix}_traversability.txt", index=False, header=False,sep= ' ')
    
    # Guardar mapa de alturas
    z_map_df = pd.DataFrame(z_map)
    z_map_df.to_csv(f"{output_prefix}_z_values.txt", index=False, header=False,sep=' ')
            
    print("Mapas guardados exitosamente como CSV.")

import matplotlib.pyplot as plt

def visualize_maps(traversability_map, z_map):
    """
    Representa visualmente los mapas de transitabilidad y alturas.
    
    Args:
        traversability_map (np.ndarray): Mapa binario de transitabilidad.
        z_map (np.ndarray): Mapa de alturas (Z).
    """
    # Configurar el tamaño de las gráficas
    plt.figure(figsize=(12, 6))

    # Mapa de transitabilidad
    plt.subplot(1, 2, 1)
    plt.title("Mapa de Transitabilidad")
    plt.imshow(traversability_map, cmap="gray", interpolation="nearest")
    plt.colorbar(label="Transitabilidad (1: Transitable, 0: No transitable)")
    plt.axis("off")

    # Mapa de alturas (Z)
    plt.subplot(1, 2, 2)
    plt.title("Mapa de Alturas (Z)")
    plt.imshow(z_map, cmap="viridis", interpolation="nearest")
    plt.colorbar(label="Altura (m)")
    plt.axis("off")

    # Mostrar las gráficas
    plt.tight_layout()
    plt.show()



# Main function
if __name__ == "__main__":
    # Ruta del heightmap de entrada
    heightmap_path = "test_map.png"  # Cambia esta ruta a tu archivo
    
    # Prefijo para los archivos de salida
    output_prefix = "output/heightmap"  # Carpeta y prefijo para los archivos de salida
    
    # Cargar el heightmap
    heightmap = load_heightmap(heightmap_path)
    
    # Ajustar el heightmap (restar 100 y convertir a metros)
    heightmap_adjusted = adjust_heightmap(heightmap)
    
    # Generar los mapas
    traversability_map, z_map = generate_maps(heightmap_adjusted)
    
    # Guardar los mapas en archivos CSV
    save_maps_to_csv(traversability_map, z_map, output_prefix)

    # Visualizar los mapas generados
    visualize_maps(traversability_map, z_map)

     # Ruta del archivo de salida para puntos de interés
    points_output_file = "output/interest_points.txt"

    # Generar puntos de interés
    interest_points = generate_interest_points(heightmap_adjusted, num_points=1)

    # Guardar puntos de interés en un archivo CSV
    save_interest_points(interest_points, points_output_file)

    # Visualizar el mapa con puntos de interés
    visualize_interest_points(heightmap_adjusted, interest_points)