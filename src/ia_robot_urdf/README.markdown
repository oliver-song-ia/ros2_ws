# Procthor_glb_exporter

## Description

The **Procthor_glb_exporter** is a tool designed to export ProcTHOR scenes into GLB (GL Transmission Format) models. ProcTHOR, developed by the Allen Institute for AI (AI2), is a procedural generation framework within the AI2-THOR environment that creates diverse, customizable 3D indoor scenes (e.g., houses) for AI research, simulation, and training. This repository provides a script to convert ProcTHOR-generated house scenes, represented as JSON files, into GLB models for use in other 3D applications or rendering pipelines.

## Requirements

To use this tool, ensure you have the following:

- **AI2-THOR**: The AI2-THOR framework, which includes the ProcTHOR procedural generation system. Install via the official AI2-THOR repository.
- **Unity 2021**: The exporter is designed to work with Unity 2021. Other versions of Unity are not guaranteed to work due to potential compatibility issues with AI2-THOR or the GLTF export package.

## Tutorial

Follow these steps to set up and use the Procthor_glb_exporter:

1. **Clone AI2-THOR**:

   - Clone the AI2-THOR repository from GitHub:

     ```bash
     git clone https://github.com/allenai/ai2thor.git
     ```

2. **Open the AI2-THOR Unity Project**:

   - Launch Unity 2021.
   - In the Unity Hub, select "Add" and navigate to the cloned AI2-THOR repository folder to add the project.
   - Open the AI2-THOR project in Unity.

3. **Install the Unity-GLTF Package**:

   - In Unity, go to `Window > Package Manager`.
   - Search for and install the `Unity-GLTF` package (or import it manually from Unity-GLTF GitHub if not available in the Package Manager).

4. **Set Up the Exporter Script**:

   - Copy the `exporter.cs` script provided in this repository to the `Editor` folder of the AI2-THOR Unity project (e.g., `ai2thor/Editor/`).
   - Copy your ProcTHOR house JSON files to the `Assets/Resources/rooms/` folder in the AI2-THOR project.

5. **Open the Procedural Scene**:

   - In the Unity project, navigate to the Scenes folder and open the "Procedural" scene (typically located in `Assets/Scenes/Procedural`).

6. **Run the Scene**:

   - Click the "Play" button in Unity to start the scene.
   - In the input field within the scene, type `initp` and press Enter to initialize the procedural generation.

7. **Export GLB Files**:

   - In the Unity menu, navigate to `Tools` (or the custom menu created by the exporter script).
   - Select the option to export GLB files. This will convert the ProcTHOR house JSONs in `Assets/Resources/rooms/` into GLB models.
   - The exported GLB files will be saved to a designated output folder (check the exporter script for the default path or configure it as needed).
   