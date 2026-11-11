#!/usr/bin/env python3
"""desk_guard.py — desk 看门狗：GUI 进程消失时自动停掉整个 desk 进程组。

    desk_guard.py <gui_pid> <gui_start_ticks> <cmd...>

背景
----
GUI 用 start_new_session=True 启 desk，desk 是独立会话组。若 GUI 意外退出
（终端被关、SIGKILL、崩溃），desk 的 launch 被 init 收养；其中
complementary_filter 是 respawn=True，launch 被非正常杀掉时叶子会以
ppid=1 的孤儿形式一直往 /imu/data、/tf_static 发数据。

本看门狗就是该进程组的组长：
  · GUI 正常停止 desk → GUI 对本组 SIGINT → launch 正常收尾（本守护忽略自身 SIGINT）
  · GUI 进程消失      → 轮询发现后对本组 SIGINT，8s 不退再 SIGKILL
  · launch 自行退出   → 清理同组残留叶子后退出

只发 SIGINT/SIGKILL，不读写任何话题，不碰进程组之外的进程。
"""

import os
import signal
import subprocess
import sys
import threading
import time


def group_members(pgid: int) -> list[int]:
    """本进程组内除自己以外的 pid（读 /proc，不依赖 ps）。"""
    out = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == os.getpid():
            continue
        try:
            with open(f"/proc/{pid}/stat", "rb") as f:
                raw = f.read().decode("ascii", "replace")
            rest = raw.rsplit(") ", 1)[1].split()
            if rest[0] != "Z" and int(rest[2]) == pgid:  # 活跃进程组成员
                out.append(pid)
        except Exception:                      # noqa: BLE001
            continue
    return out


def _pid_exists_active(pid: int) -> bool:
    identity = process_identity(pid)
    return bool(identity and identity[0] != "Z")


def log(msg: str) -> None:
    # GUI 退出后本进程的 stdout/stderr 管道失去读者，write 会 EPIPE；
    # 日志行不能成为看门狗自己先死（进而放跑整个进程组）的理由。
    try:
        print(f"[desk_guard] {msg}", file=sys.stderr, flush=True)
    except (BrokenPipeError, OSError):
        pass


def process_identity(pid: int):
    """返回 (state, start_ticks)，PID 不存在或不可读则返回 None。"""
    try:
        with open(f"/proc/{pid}/stat", "rb") as fh:
            raw = fh.read().decode("ascii", "replace")
        fields = raw.rsplit(") ", 1)[1].split()
        return fields[0], int(fields[19])
    except (OSError, IndexError, ValueError):
        return None


def main() -> int:
    if len(sys.argv) < 4:
        print("usage: desk_guard.py <gui_pid> <gui_start_ticks> <cmd...>", file=sys.stderr)
        return 2
    gui_pid = int(sys.argv[1])
    gui_start_ticks = int(sys.argv[2])
    cmd = sys.argv[3:]

    # 必须自己当进程组组长，否则 killpg 会波及调用者的整个组
    if os.getpgid(0) != os.getpid():
        try:
            os.setsid()
        except OSError as exc:
            log(f"无法成为会话组长（{exc}），拒绝启动")
            return 3
    pgid = os.getpgid(0)

    # 忽略自身收到的 INT/TERM/HUP，好让看门狗能撑到升级 SIGKILL；
    # 用 Python handler（不是 SIG_IGN），exec 后的 launch 仍拿到默认处置。
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_a: None)

    proc = subprocess.Popen(cmd)           # 与守护同组（继承 pgid）

    def shutdown_children(reason: str, wait_s: float = 8.0) -> None:
        left = group_members(pgid)
        log(f"{reason}；对进程组 {pgid} 发 SIGINT（成员 {left}）")
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            if not group_members(pgid):
                log("进程组已清空")
                return
            time.sleep(0.2)
        left = group_members(pgid)
        if left:
            log(f"{wait_s:.0f}s 未退出，SIGKILL 进程组（成员 {left}）")
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def gui_watch() -> None:
        while True:
            identity = process_identity(gui_pid)
            # PID 不存在、变成僵尸或启动时间变化（PID 已复用）均视为原 GUI 已退出。
            if identity is None or identity[0] == "Z" or identity[1] != gui_start_ticks:
                break
            time.sleep(0.5)
        # 保留标记：退出前 deactivate_mapping 保存 PCD 失败/超时，desk/bag 必须
        # 留下让服务端把写盘机会走完（否则整张地图连录包一起丢）。GUI 在
        # _shutdown_app 里写该文件；看到它就不再打进程组，只记录 pgid 后退出，
        # 由操作者在持久终端手动重试保存并清理。
        preserve = f"/tmp/n01_desk_preserve_{gui_pid}.flag"
        if os.path.exists(preserve):
            info = ""
            try:
                with open(preserve, "r", encoding="utf-8") as fh:
                    info = fh.read().strip()
            except OSError:
                pass
            keep_pgid = None
            for token in info.replace(";", " ").split():
                if token.startswith("pgid="):
                    try:
                        keep_pgid = int(token[5:])
                    except ValueError:
                        keep_pgid = None
            # 只保留标记里指定的那个进程组（desk）；rviz/bag 等其他受管组照常清理。
            # 标记不由本进程删除：多个看门狗共享同一 GUI pid，先读到的删掉会让
            # 后者误判为无标记而杀掉 desk。GUI 下次启动时按自身 pid 清理。
            if keep_pgid is None or keep_pgid == pgid:
                log(f"GUI pid {gui_pid} 已退出，发现保留标记（{info or '无内容'}）；"
                    f"跳过自动清理，desk 进程组 {pgid} 保持运行以重试 PCD 保存")
                # 保留期间必须给子进程的 stdout 管道提供一个读者：GUI 已退出、
                # 读端只剩这里 dup 的那份，不读走写端就会 EPIPE，被保留的节点
                # 在首次写日志时被 SIGPIPE 打死。持续读到 EOF（launch 退出）。
                try:
                    drain_fd = int(os.environ["N01_GUARD_READ_FD"])
                except (KeyError, ValueError):
                    drain_fd = -1
                if drain_fd >= 0:

                    def drain(fd: int) -> None:
                        try:
                            while os.read(fd, 65536):
                                pass
                        except (OSError, ValueError):
                            pass

                    threading.Thread(target=drain, args=(drain_fd,),
                                     daemon=True).start()
                # 不 _exit：回到主流程继续 proc.wait()，让本进程（及上面的
                # 读端 fd）活到 launch 自然结束，由操作者手动重试保存并停组。
                return
        shutdown_children(f"GUI pid {gui_pid} 已退出", wait_s=8.0)
        os._exit(0)

    threading.Thread(target=gui_watch, daemon=True).start()

    rc = proc.wait()

    # launch 自己退出：清掉可能残留的同组叶子（不依赖 launch 的收尾）
    left = group_members(pgid)
    if left:
        log(f"launch 已退出，清理同组残留 {left}")
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            pass
        time.sleep(1.0)
        left = group_members(pgid)
        if left:
            log(f"仍有残留 {left}，SIGKILL")
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    # 子进程被信号终止时 Popen 返回负数，转成 shell 惯例的 128+N
    return rc if rc >= 0 else 128 + (-rc)


if __name__ == "__main__":
    sys.exit(main())
