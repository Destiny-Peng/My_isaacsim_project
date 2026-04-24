请生成一个用于 NVIDIA Isaac Sim 5.1.0 的 standalone Python 控制脚本，具体要求如下：

【版本与环境】
1. 目标环境：Isaac Sim 5.1.0，Python 3.10+
2. 所有导入必须使用 `isaacsim.*` 命名空间，严格避免使用已废弃的 `omni.isaac.*` 前缀。
3. 参考官方示例 `stacking.py` 的仿真生命周期管理方式。

【核心目标】
搭建 Franka Panda 机械臂的纯力矩控制仿真环境，提供标准化的关节状态读取与力矩下发接口，便于后续直接接入滑模控制或阻抗控制算法。无需添加 Effort Sensor，直接使用 Articulation 原生 API。

【API规范与架构要求】
1. 初始化：使用 `from isaacsim import SimulationApp`，启动时设置 `{"headless": False}`。
2. 场景管理：使用 `from isaacsim.core.api import World` 创建 `my_world = World(stage_units_in_meters=1.0)`，并设置物理步长 `physics_dt=0.01`（100Hz）。
3. 机器人加载：
   - 使用内置资产路径加载 Franka Panda（如 `isaacsim.robot.manipulators` 或标准 USD 路径）。
   - 仅操作前 7 个臂关节，忽略末端夹爪（2 DOF）。
   - 初始化后通过关节属性接口将前 7 个关节的 stiffness 与 damping 设为 0，确保纯力矩控制生效。
4. 控制器获取：使用 `articulation_controller = my_franka.get_articulation_controller()` 获取底层执行器接口。

【状态读取与力控接口】
1. 状态读取：直接调用 Articulation 对象原生方法获取前 7 个关节的位置、速度与当前施加力矩。返回类型为 `numpy.ndarray`。
   - 封装为 `get_robot_state() -> dict: {"q": np.array, "dq": np.array, "tau": np.array}`
2. 力矩下发：使用 `isaacsim.core.prims.ArticulationAction` 包装力矩数组，通过 `articulation_controller.apply_action(action)` 下发。
   - 封装为 `set_robot_torques(torques: np.array) -> None`
3. 所有接口需包含维度校验（确保输入/输出长度为 7），维度不匹配时抛出 `ValueError`。

【主循环与仿真控制】
1. 严格遵循 `stacking.py` 的生命周期模式：
   - 使用 `while simulation_app.is_running():`
   - 内部调用 `my_world.step(render=True)`
   - 处理 `my_world.is_stopped()` 与 `my_world.is_playing()` 状态，实现安全重置逻辑（`reset_needed` 标志位）。
2. 控制循环结构：
   - 在 `is_playing()` 分支内：读取状态 -> 调用占位控制函数 -> 下发力矩。
   - 预留函数 `compute_control_torques(state: dict) -> np.array`，内部默认返回零向量，并添加清晰注释标注算法插入点。
3. 循环外注册 `signal.signal(signal.SIGINT, ...)` 确保终端 Ctrl+C 可安全退出仿真。

【代码规范】
1. 完整可独立运行的脚本，包含所有必要 import、初始化、主循环与清理逻辑。
2. 关键 API 调用添加注释说明 5.1.0 版本下的作用。
3. 控制频率、关节索引、默认物理参数等使用常量定义，避免魔法数字。
4. 输出前确保符合 PEP8，无语法错误，且严格匹配 Isaac Sim 5.1.0 官方 standalone 脚本结构。

【输出格式】
仅输出完整 Python 代码，不包含额外解释、运行截图或环境配置说明。代码首行注明 `# Isaac Sim 5.1.0 Standalone Force Control Template`。