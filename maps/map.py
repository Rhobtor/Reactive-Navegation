import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from PIL import Image

# Cargar el mapa de alturas (heightmap)
def load_heightmap(image_path):
    # Abrir la imagen y convertirla a escala de grises
    img = Image.open(image_path).convert("L")
    # Convertir la imagen en un array numpy
    height_data = np.array(img)
    return height_data

# Proyectar el mapa en 3D
def plot_heightmap_3d(heightmap, scale=1):
    # Crear una malla de coordenadas
    x = np.arange(0, heightmap.shape[1])
    y = np.arange(0, heightmap.shape[0])
    x, y = np.meshgrid(x, y)
    
    # Escalar las alturas para aumentar el efecto visual
    z = heightmap * scale
    
    # Crear la figura 3D
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    
    # Graficar la superficie
    surface = ax.plot_surface(
        x, y, z, cmap=cm.terrain, edgecolor='none', rstride=1, cstride=1, antialiased=True
    )
    
    # Añadir barra de color
    fig.colorbar(surface, ax=ax, shrink=0.5, aspect=10, label="Altura")
    
    # Etiquetas y título
    ax.set_title("Proyección 3D del Heightmap")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Altura")
    
    # Mostrar el gráfico
    plt.show()

# Ruta al archivo heightmap
image_path = "test_map4.png"

# Cargar y proyectar el heightmap
heightmap = load_heightmap(image_path)
plot_heightmap_3d(heightmap, scale=0.1)
