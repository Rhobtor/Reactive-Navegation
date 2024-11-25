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

# Llama a esta función después de generar los mapas
if __name__ == "__main__":
    # Visualizar los mapas generados
    visualize_maps(traversability_map, z_map)
