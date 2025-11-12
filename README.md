# 🧊 3D Visualization Project (Open3D)

## 📚 Overview
This project demonstrates the use of the **Open3D** library for working with 3D geometry in Python.  
It includes all **7 required stages** of processing a 3D object — from loading to color mapping and finding extreme points.

---

## ⚙️ Steps Implemented

### **1. Loading and Visualization**
- Loads a 3D mesh (`.ply`, `.obj`, or `.stl`).
- Displays the original model.
- Prints:
  - Number of vertices and triangles
  - Presence of colors and normals

### **2. Conversion to Point Cloud**
- Converts the mesh to a point cloud.
- Displays the point cloud.
- Prints the number of points and color information.

### **3. Surface Reconstruction (Poisson)**
- Restores a surface mesh from the point cloud.
- Removes artifacts using a bounding box crop.
- Displays reconstructed mesh.
- Prints vertex and triangle counts.

### **4. Voxelization**
- Converts the point cloud into a voxel grid (`create_from_point_cloud()`).
- Displays the voxel model.
- Prints the number of voxels and color presence.

### **5. Adding a Plane**
- Adds a simple geometric plane next to the object for comparison.

### **6. Clipping (Surface Cutting)**
- Removes part of the object by plane intersection.
- Displays the clipped model.
- Prints updated counts.

### **7. Color Gradient & Extremes**
- Applies a custom color gradient by axis.
- Finds minimum and maximum points (e.g., by Z-axis).
- Marks them with colored spheres.
- Prints their coordinates.

---

## 🧩 Technologies Used
- **Language:** Python 3.12  
- **Library:** Open3D 0.19.0  
- **Environment:** Conda (Miniforge3)  
- **Visualization Backend:** OpenGL (KDE Wayland compatible)

---

## ▶️ How to Run
1. Activate environment:
   ```bash
   conda activate o3d
