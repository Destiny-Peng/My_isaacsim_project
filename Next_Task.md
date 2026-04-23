# Role
Expert NVIDIA Isaac Sim Developer.

# Task
Implement a `GraspMetricEvaluator` using **Isaac Sim Replicator API** (`omni.replicator.core`) to automatically record object metrics during a simulation trial.

# Technical Requirements
1. **Use Replicator Annotators**: Do NOT manually compute transforms using `pxr.UsdGeom`. Instead, use `rep.AnnotatorRegistry.get_annotator("pose")` to track the object's world position.
2. **Contact Data**: Use `rep.AnnotatorRegistry.get_annotator("contact")` or `physics_sim_view` to detect if the object is in contact with the gripper vs. the table.
3. **Data Structure**: Store time-series data (timestamp, position, velocity) in a `pandas.DataFrame` or `numpy` arrays for easy post-processing.
4. **Metrics**:
   - **Success**: Final Z > Target Z.
   - **Stability**: Std dev of Z position during the "hold" phase.
   - **Slippage**: Detect sudden drops in Z velocity while gripper is closed.

# Implementation Steps
1. **Setup**: Initialize annotators for the Object Prim and Gripper Prim.
2. **Recording**: Create a method `start_recording()` and `stop_recording()` that subscribes/unsubscribes to the simulation step event.
3. **Evaluation**: A method `compute_metrics(target_z)` that analyzes the recorded DataFrame.

# Code Constraint
- Use `omni.replicator.core` for data extraction.
- Use `omni.physx` for contact queries if needed.
- Keep the code modular and clean.

# Output
Provide the complete Python class implementation.