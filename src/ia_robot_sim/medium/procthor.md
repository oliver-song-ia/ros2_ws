

# 🏛️✨ Bringing Procedural AI2-THOR Scenes to Robotics Simulators

## 1. 🌍 The Need for Procedural Diversity in Robotics Simulators

Modern robotics thrives on simulated environments to validate perception, planning, and control algorithms. However, crafting diverse, rich 3D indoor scenes manually is time-consuming and unscalable. Enter **ProcTHOR**, a procedural scene generator designed for embodied AI research in Unity. It creates thousands of layout-consistent home environments effortlessly. The challenge? ProcTHOR is deeply embedded in the AI2-THOR Unity ecosystem, making direct integration with robotics simulators like **Gazebo** or **Isaac Sim** cumbersome.

## 2. 🚧 The Challenge: Extracting ProcTHOR Scenes from Unity

Most robotics simulators rely on open 3D formats like **GLB**, **SDF**, or **USD**. ProcTHOR scenes, however, are Unity-specific, stored as procedural data or Unity scenes—formats not easily portable. So, how do we bridge this gap and bring these virtual homes into robotics-friendly ecosystems?

## 3. 🌉 The Solution: Exporting GLB with `procthor_glb_exporter`

To address this, I developed **`procthor_glb_exporter`**, a Unity-based pipeline that converts ProcTHOR-generated scenes into **GLB** files, compatible with most 3D software, simulators, and rendering engines. This unlocks workflows like:

- Exporting ProcTHOR houses as GLB models via Unity.
- Visualizing or refining scenes in **Blender** before deployment.
- Importing into **Isaac Sim** for photorealistic robotics testing.
- Converting GLB to **SDF** for **Gazebo** simulations.


Here’s how to set it up.

## 4. 🛠️ Setting Up the Export Pipeline

This one-time setup enables exporting unlimited ProcTHOR scenes.

**✅ Step 1: Clone AI2-THOR**  
Clone the [AI2-THOR repository](https://github.com/allenai/ai2thor) and open it in **Unity 2021**. Stick to this version, as newer ones may break AI2-THOR or its dependencies.

**✅ Step 2: Add the Export Script**  
Copy the export script from the [`procthor_glb_exporter` repository](https://github.com/your-repo/procthor_glb_exporter) into your Unity project’s `Assets/Editor/` folder. This script adds a custom Unity menu item for GLB export.  
💡 *Why Unity scripting?* Unity lacks native GLB export. The script leverages a **GLTF export plugin** to automate the process.

**✅ Step 3: Install the Unity-GLTF Package**  
To support GLTF/GLB assets:  
- Open Unity’s **Package Manager** (`Window > Package Manager`).  
- Search for **Unity-GLTF** and click **Install**.  
- If unavailable, manually import it from the [official UnityGLTF GitHub](https://github.com/KhronosGroup/UnityGLTF).

**✅ Step 4: Import ProcTHOR JSON Scenes**  
Define simulation environments using ProcTHOR’s Python API:  

- **Generate Scenes Manually**  
  Use ProcTHOR’s Python API to create scenes programmatically. Each scene is saved as a `.json` file detailing room layouts, object placements, materials, and textures. Store these in:  
  ```
  Assets/Resources/rooms/
  ```

- **Use Prebuilt Scenes (Recommended for Quick Setup)**  
  Download pre-generated JSON files from the [ProcTHOR-10K dataset](https://procthor.allenai.org/). These are ready-to-use and compatible with Unity’s loader. Place them in `Assets/Resources/rooms/`.

**✅ Step 5: Load and Initialize the Procedural Scene**  
In Unity, open the **Procedural** scene from AI2-THOR. Enter **Play** mode, type `initp` in the debug console, and watch your JSON-defined layout come to life.

## 5. 📤 Exporting to GLB

Once the scene loads correctly:  

1. Exit **Play** mode.  
2. Access the custom menu added by the exporter.  
3. Select **Export to GLB**.  
4. Find the `.glb` file in the script’s output folder (or specify your own path).  

Your scene is now ready for robotics simulators!

## 6. 🤖 Using GLB in Robotics Simulators

With your `.glb` file in hand, you can:  

**🧊 Isaac Sim**  
- Import the GLB directly or convert to **USD** using **Blender** or **Omniverse Kit**.  
- Add collision and physics properties in Isaac Sim.  
- Place your robot in the scene and start testing.  
- You can also use the python script to load the glb scene with colliders automatically created based on the mesh.

**🤖 Gazebo**  
- Open Blender, load the glb file
- Use the script I have created to export the scene into gazebo world files (Note that the script still has some issues as Gazebo seems to be unable to support a lot of materials)
- Launch with **ROS 2** to test your robot in the generated home.  

## 7. 🌟 Why This Matters

This pipeline unlocks thousands of diverse indoor scenes for robotics, previously confined to Unity. By exporting to **GLB**, we decouple scene generation from simulation, empowering researchers to scale testing without manual modeling.

## 8. 🔮 Future Work

- 🛠️ Improve the visual for Gazebo
- 📦 Batch export for multiple scenes.  
- 🌌 Direct **USD** export for **Omniverse** workflows.  

## 🔗 Resources

- 🔧 [procthor_glb_exporter GitHub](https://github.com/GreatenAnoymous/procthor_glb_exporter)  
- 📦 [AI2-THOR](https://github.com/allenai/ai2thor)  
- 📚 [ProcTHOR Overview](https://procthor.allenai.org/)  

