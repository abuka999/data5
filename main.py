import open3d as o3d
import numpy as np


def step1_load_mesh(armadillo_path):
    print("\n=== STEP 1: Загрузка и визуализация исходного меша ===")

    mesh = o3d.io.read_triangle_mesh(armadillo_path)

    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()

    print(f"Количество вершин       : {len(mesh.vertices)}")
    print(f"Количество треугольников: {len(mesh.triangles)}")
    print(f"Есть цвет вершин        : {mesh.has_vertex_colors()}")
    print(f"Есть нормали вершин     : {mesh.has_vertex_normals()}")

    # Визуализация исходного меша
    o3d.visualization.draw_plotly([mesh])

    return mesh


def step2_mesh_to_point_cloud(armadillo_path):
    print("\n=== STEP 2: Преобразование в облако точек ===")

    # В задании просят read_point_cloud — читаем тот же файл как point cloud
    pcd = o3d.io.read_point_cloud(armadillo_path)

    # Немного нормалей (полезно для следующих шагов)
    if not pcd.has_normals():
        pcd.estimate_normals()

    points = np.asarray(pcd.points)
    z = points[:, 2]
    z_min, z_max = z.min(), z.max()
    z_norm = (z - z_min) / (z_max - z_min + 1e-8)

    # Раскраска по высоте (ось Z)
    colors = np.zeros((len(z_norm), 3))
    colors[:, 0] = z_norm            # красный — верх
    colors[:, 1] = 0.5 * (1 - z_norm)
    colors[:, 2] = 1 - z_norm        # синий — низ
    pcd.colors = o3d.utility.Vector3dVector(colors)

    print(f"Количество точек        : {len(pcd.points)}")
    print(f"Есть цвета у точек      : {pcd.has_colors()}")

    o3d.visualization.draw_plotly([pcd])

    return pcd


def step3_surface_reconstruction(pcd):
    print("\n=== STEP 3: Реконструкция поверхности (Poisson) ===")

    # Нормали нужны для Poisson
    pcd.estimate_normals()

    mesh_rec, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=8
    )

    # Обрезаем артефакты по bounding box исходного облака
    bbox = pcd.get_axis_aligned_bounding_box()
    mesh_crop = mesh_rec.crop(bbox)

    print(f"Вершины после реконструкции   : {len(mesh_crop.vertices)}")
    print(f"Треугольники после реконстр.  : {len(mesh_crop.triangles)}")
    print(f"Есть цвета                    : {mesh_crop.has_vertex_colors()}")
    print(f"Есть нормали                  : {mesh_crop.has_vertex_normals()}")

    o3d.visualization.draw_plotly([mesh_crop])

    return mesh_crop


def step4_voxelization(pcd):
    print("\n=== STEP 4: Вокселизация ===")

    voxel_size = 5.0  # можно поменять при защите
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)

    voxels = voxel_grid.get_voxels()
    print(f"Количество вокселей          : {len(voxels)}")

    # Для визуализации делаем pcd из центров вокселей
    centers = np.array(
        [voxel_grid.get_voxel_center_coordinate(v.grid_index) for v in voxels]
    )
    vox_pcd = o3d.geometry.PointCloud()
    vox_pcd.points = o3d.utility.Vector3dVector(centers)

    z = centers[:, 2]
    z_min, z_max = z.min(), z.max()
    z_norm = (z - z_min) / (z_max - z_min + 1e-8)

    colors = np.zeros((len(z_norm), 3))
    colors[:, 0] = 0.2 + 0.8 * z_norm
    colors[:, 1] = 0.3
    colors[:, 2] = 1 - z_norm
    vox_pcd.colors = o3d.utility.Vector3dVector(colors)

    print(f"Есть цвета у вокс. облака    : {vox_pcd.has_colors()}")

    o3d.visualization.draw_plotly([vox_pcd])

    return voxel_grid


def step5_add_plane(mesh_crop):
    print("\n=== STEP 5: Добавление плоскости рядом с объектом ===")

    bbox = mesh_crop.get_axis_aligned_bounding_box()
    center = bbox.get_center()
    extent = bbox.get_extent()

    # Плоскость как очень тонкий бокс
    plane = o3d.geometry.TriangleMesh.create_box(
        width=1.0,        # тонкая по X
        height=extent[1] * 1.4,
        depth=extent[2] * 1.4,
    )

    # Ставим плоскость чуть правее объекта по оси X
    plane.translate([
        center[0] + extent[0] * 0.3,
        center[1] - (extent[1] * 0.7),
        center[2] - (extent[2] * 0.7),
    ])
    plane.paint_uniform_color([0.8, 0.1, 0.1])  # красная плоскость

    o3d.visualization.draw_plotly([mesh_crop, plane])

    return plane


def step6_clipping_by_plane(mesh_crop, plane):
    print("\n=== STEP 6: Обрезка по поверхности (клиппинг) ===")

    bbox_mesh = mesh_crop.get_axis_aligned_bounding_box()
    bbox_plane = plane.get_axis_aligned_bounding_box()

    # Берём X-координату плоскости как границу отсечения
    plane_x = bbox_plane.get_center()[0]

    # Оставляем только то, что ЛЕВЕЕ плоскости (x <= plane_x)
    clip_bbox = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=[bbox_mesh.min_bound[0], bbox_mesh.min_bound[1], bbox_mesh.min_bound[2]],
        max_bound=[plane_x, bbox_mesh.max_bound[1], bbox_mesh.max_bound[2]],
    )

    clipped_mesh = mesh_crop.crop(clip_bbox)

    print(f"Осталось вершин        : {len(clipped_mesh.vertices)}")
    print(f"Осталось треугольников : {len(clipped_mesh.triangles)}")
    print(f"Есть цвет              : {clipped_mesh.has_vertex_colors()}")
    print(f"Есть нормали           : {clipped_mesh.has_vertex_normals()}")

    o3d.visualization.draw_plotly([clipped_mesh, plane])

    return clipped_mesh


def step7_color_and_extrema(mesh):
    print("\n=== STEP 7: Цвет и экстремальные точки ===")

    vertices = np.asarray(mesh.vertices)
    z = vertices[:, 2]
    z_min, z_max = z.min(), z.max()
    z_norm = (z - z_min) / (z_max - z_min + 1e-8)

    # 💥 более контрастная раскраска
    colors = np.zeros((len(z_norm), 3))
    colors[:, 0] = z_norm                   # красный — верх
    colors[:, 1] = 0.1 + 0.6 * (1 - z_norm) # зелёный приглушённый
    colors[:, 2] = 1 - z_norm               # синий — низ
    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)

    # экстремумы
    idx_min = int(np.argmin(z))
    idx_max = int(np.argmax(z))
    min_point = vertices[idx_min]
    max_point = vertices[idx_max]

    print(f"Мин. по Z: {min_point}")
    print(f"Макс. по Z: {max_point}")

    # 💥 увеличенный радиус сфер
    r = (z_max - z_min) * 0.07
    sphere_min = o3d.geometry.TriangleMesh.create_sphere(radius=r)
    sphere_min.translate(min_point)
    sphere_min.paint_uniform_color([0.0, 1.0, 0.0])  # зелёная сфера

    sphere_max = o3d.geometry.TriangleMesh.create_sphere(radius=r)
    sphere_max.translate(max_point)
    sphere_max.paint_uniform_color([1.0, 0.0, 0.0])  # красная сфера

    # 💥 визуализация с выделением сфер
    o3d.visualization.draw_plotly([mesh, sphere_min, sphere_max])



if __name__ == "__main__":
    # Загружаем путь к стандартной модели Armadillo
    armadillo_data = o3d.data.ArmadilloMesh()
    armadillo_path = armadillo_data.path

    # Шаг 1: исходный меш
    mesh_original = step1_load_mesh(armadillo_path)

    # Шаг 2: облако точек
    pcd = step2_mesh_to_point_cloud(armadillo_path)

    # Шаг 3: реконструкция поверхности
    mesh_reconstructed = step3_surface_reconstruction(pcd)

    # Шаг 4: вокселизация
    voxel_grid = step4_voxelization(pcd)

    # Шаг 5: добавляем плоскость
    plane = step5_add_plane(mesh_reconstructed)

    # Шаг 6: клиппинг по плоскости
    mesh_clipped = step6_clipping_by_plane(mesh_reconstructed, plane)

    # Шаг 7: цвет и экстремальные точки
    step7_color_and_extrema(mesh_clipped)
