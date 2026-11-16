# 内部运行辅助

这些脚本由GUI手动按钮或`scripts/n01`路由调用，不是另一套“一键全启动”。

- `start_localization_stack.sh`：甲方定位及适配器；底盘/雷达/RSP由外部入口提供。
- `start_fast_lio_gui_backend.sh`：自部署FAST-LIO2后端，显式选择定位、建图或实时模式。
- `start_navigation_execution.sh`：执行器环境；默认不使能行驶。

GUI入口：`bash scripts/n01 gui`。不要在这些脚本中增加自动启动底盘、清场或雷达回退。
