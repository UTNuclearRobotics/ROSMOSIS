#%%
import open3d as o3d
import numpy as np

mesh_path = "/home/talal/projects/rosmosis_ws/src/sensor_model/assets/seabed_1000x1000m.obj"
mesh = o3d.io.read_triangle_mesh(mesh_path, enable_post_processing=True)

vertices = np.asarray(mesh.vertices)
triangles = np.asarray(mesh.triangles)

print("=== Mesh Summary ===")
print(f"Vertices:  {len(vertices)}")
print(f"Triangles: {len(triangles)}")

print("\n=== Bounds ===")
mins = vertices.min(axis=0)
maxs = vertices.max(axis=0)
spans = maxs - mins
for i, axis in enumerate(['X', 'Y', 'Z']):
    print(f"  {axis}: [{mins[i]:.2f}, {maxs[i]:.2f}]  span={spans[i]:.2f}m")

print(f"\n=== Properties ===")
print(f"Watertight:    {mesh.is_watertight()}")
print(f"Has normals:   {mesh.has_vertex_normals()}")
print(f"Has colors:    {mesh.has_vertex_colors()}")
print(f"Is orientable: {mesh.is_orientable()}")

# Edge length statistics
edge_set = set()
for tri in triangles:
    for i in range(3):
        edge = tuple(sorted([tri[i], tri[(i+1) % 3]]))
        edge_set.add(edge)
edges = np.array(list(edge_set))
edge_lengths = np.linalg.norm(vertices[edges[:, 0]] - vertices[edges[:, 1]], axis=1)
print(f"\n=== Edge Lengths ===")
print(f"  Min:    {edge_lengths.min():.3f}m")
print(f"  Max:    {edge_lengths.max():.3f}m")
print(f"  Mean:   {edge_lengths.mean():.3f}m")
print(f"  Median: {np.median(edge_lengths):.3f}m")

# Triangle area statistics
v0 = vertices[triangles[:, 0]]
v1 = vertices[triangles[:, 1]]
v2 = vertices[triangles[:, 2]]
areas = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)
print(f"\n=== Triangle Areas ===")
print(f"  Min:    {areas.min():.4f} m²")
print(f"  Max:    {areas.max():.4f} m²")
print(f"  Mean:   {areas.mean():.4f} m²")
print(f"  Total:  {areas.sum():.2f} m²")

#%% Elevation heatmap visualization
from matplotlib import cm

z_vals = np.asarray(mesh.vertices)[:, 2]
z_norm = (z_vals - z_vals.min()) / (z_vals.max() - z_vals.min())
colors = cm.get_cmap('viridis')(z_norm)[:, :3]
mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
mesh.compute_vertex_normals()

o3d.visualization.draw_geometries([mesh], window_name="High-Fidelity Seabed", width=1280, height=960)

#%% 2D elevation heatmap with colorbar
import matplotlib.pyplot as plt

fig, ax = plt.subplots(figsize=(8, 7))
im = ax.tricontourf(vertices[:, 0], vertices[:, 1], vertices[:, 2], levels=50, cmap='viridis')
fig.colorbar(im, ax=ax, label="Elevation (m)")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("Seabed Elevation Heatmap")
ax.set_aspect('equal')
plt.tight_layout()
plt.show()
