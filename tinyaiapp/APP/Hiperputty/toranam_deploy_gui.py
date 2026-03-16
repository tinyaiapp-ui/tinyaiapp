#!/usr/bin/env python3
"""
Toranam Deploy Tool

A small Tkinter GUI for deploying selected or changed files to a remote server via SSH/SFTP.
Workflow:
1. Connect to the server.
2. Stop the target systemd service.
3. Upload selected or changed files to the target directory.
4. Start the target systemd service again.

Requirements:
    pip install paramiko

Notes:
- Named server profiles are stored in a "profiles" subfolder next to this program.
- Passwords can be stored locally if the "Remember passwords" checkbox is enabled.
- Remote file timestamps are updated to match local timestamps after upload to improve change detection.
"""

from __future__ import annotations

import json
import os
import posixpath
import re
import stat
import subprocess
import sys
import threading
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Callable, Dict, List, Optional, Tuple
import traceback

import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk

try:
    import paramiko
except ImportError:  # pragma: no cover - handled at runtime
    paramiko = None


if TYPE_CHECKING:
    from paramiko import SFTPClient, SSHClient


APP_DIR = Path(__file__).resolve().parent
PROFILES_DIR = APP_DIR / "profiles"
DEFAULT_IGNORES = {
    ".git",
    ".idea",
    ".vscode",
    "__pycache__",
    ".venv",
    "venv",
    "node_modules",
    ".mypy_cache",
    ".pytest_cache",
}


@dataclass
class FileInfo:
    """Simple description of a local or remote file used for comparisons."""

    rel_path: str
    size: int
    mtime: float


class DeployToolApp:
    """Main GUI application for comparing and deploying files over SSH/SFTP."""

    def __init__(self, root: tk.Tk) -> None:
        """Build the interface, prepare the profile list, and initialize the file views."""
        self.root = root
        self.root.title("Toranam Deploy Tool")
        self.root.geometry("1380x860")
        self.root.minsize(1120, 720)

        PROFILES_DIR.mkdir(exist_ok=True)

        self.local_files: Dict[str, FileInfo] = {}
        self.remote_files: Dict[str, FileInfo] = {}
        self.local_scan_done = False
        self.remote_scan_done = False
        self.local_path_to_iid: Dict[str, str] = {}
        self.remote_path_to_iid: Dict[str, str] = {}
        self.local_item_to_relpath: Dict[str, str] = {}
        self.remote_item_to_relpath: Dict[str, str] = {}
        self._busy_counter = 0
        self._activity_mode = "idle"
        self._activity_total = 0
        self._activity_current = 0
        self._marquee_offset = 0
        self._marquee_job: str | None = None

        self._build_variables()
        self._build_styles()
        self._build_ui()
        self.refresh_profile_list()
        self.refresh_local_files()

    def _build_variables(self) -> None:
        """Create Tk variables used by entry fields and application options."""
        self.profile_name_var = tk.StringVar()
        self.host_var = tk.StringVar()
        self.port_var = tk.StringVar()
        self.username_var = tk.StringVar()
        self.password_var = tk.StringVar()
        self.sudo_password_var = tk.StringVar()
        self.local_root_var = tk.StringVar()
        self.remote_root_var = tk.StringVar()
        self.service_var = tk.StringVar()
        self.url_var = tk.StringVar()
        self.remember_passwords_var = tk.BooleanVar(value=True)
        self.auto_open_url_var = tk.BooleanVar(value=True)
        self.stop_start_service_var = tk.BooleanVar(value=True)
        self.status_var = tk.StringVar(value="Ready.")
        self.activity_var = tk.StringVar(value="Idle")
        self.progress_text_var = tk.StringVar(value="")
        self.ignore_names_var = tk.StringVar()

    def _build_styles(self) -> None:
        """Reserved for future UI styling tweaks."""
        return

    def _build_ui(self) -> None:
        """Construct the full Tkinter interface: settings, file panes, progress, and logs."""
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill="both", expand=True)

        top_row = ttk.Frame(main)
        top_row.pack(fill="x")

        server_frame = ttk.LabelFrame(top_row, text="Server and deployment settings", padding=10)
        server_frame.pack(side="left", fill="x", expand=True, padx=(0, 10))

        profile_frame = ttk.LabelFrame(top_row, text="Profiles", padding=10)
        profile_frame.pack(side="left", fill="y")

        server_labels = [
            ("Host / IP", self.host_var, 0, 0, 22),
            ("Port", self.port_var, 0, 2, 8),
            ("Username", self.username_var, 0, 4, 16),
            ("Password", self.password_var, 1, 0, 22),
            ("Sudo password", self.sudo_password_var, 1, 2, 16),
            ("Systemd service", self.service_var, 1, 4, 16),
            ("Test URL", self.url_var, 2, 0, 34),
            ("Ignore folder names", self.ignore_names_var, 2, 3, 20),
        ]

        for text_label, variable, row, col, width in server_labels:
            ttk.Label(server_frame, text=text_label).grid(row=row, column=col, sticky="w", padx=(0, 6), pady=4)
            show = "*" if "Password" in text_label else ""
            entry = ttk.Entry(server_frame, textvariable=variable, width=width, show=show)
            entry.grid(row=row, column=col + 1, sticky="ew", padx=(0, 12), pady=4)

        server_frame.columnconfigure(1, weight=3)
        server_frame.columnconfigure(3, weight=2)
        server_frame.columnconfigure(5, weight=2)

        options_row = ttk.Frame(server_frame)
        options_row.grid(row=3, column=0, columnspan=6, sticky="ew", pady=(8, 0))
        ttk.Checkbutton(
            options_row, text="Remember passwords locally", variable=self.remember_passwords_var
        ).pack(side="left", padx=(0, 16))
        ttk.Checkbutton(
            options_row, text="Stop / start service during deploy", variable=self.stop_start_service_var
        ).pack(side="left", padx=(0, 16))
        ttk.Checkbutton(
            options_row, text="Open test URL after upload", variable=self.auto_open_url_var
        ).pack(side="left", padx=(0, 16))

        ttk.Label(profile_frame, text="Profile name").grid(row=0, column=0, sticky="w", padx=(0, 6), pady=4)
        self.profile_combo = ttk.Combobox(
            profile_frame,
            textvariable=self.profile_name_var,
            width=28,
            state="normal",
        )
        self.profile_combo.grid(row=0, column=1, sticky="ew", padx=(0, 0), pady=4)
        ttk.Button(profile_frame, text="Save profile", command=self.save_profile).grid(
            row=1, column=0, columnspan=2, sticky="ew", pady=(8, 4)
        )
        ttk.Button(profile_frame, text="Load profile", command=self.load_profile).grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=4
        )
        ttk.Button(profile_frame, text="Refresh profiles", command=self.refresh_profile_list).grid(
            row=3, column=0, columnspan=2, sticky="ew", pady=(4, 0)
        )
        profile_frame.columnconfigure(1, weight=1)

        content_row = ttk.Frame(main, padding=(0, 10, 0, 0))
        content_row.pack(fill="both", expand=True)
        content_row.columnconfigure(0, weight=1)
        content_row.columnconfigure(1, weight=0)
        content_row.columnconfigure(2, weight=1)
        content_row.rowconfigure(0, weight=1)

        local_frame = ttk.LabelFrame(content_row, text="Local files", padding=8)
        local_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        action_frame = ttk.LabelFrame(content_row, text="Actions", padding=10)
        action_frame.grid(row=0, column=1, sticky="ns", padx=(0, 10))

        remote_frame = ttk.LabelFrame(content_row, text="Server files", padding=8)
        remote_frame.grid(row=0, column=2, sticky="nsew")

        local_controls = ttk.Frame(local_frame)
        local_controls.pack(fill="x", pady=(0, 8))
        ttk.Label(local_controls, text="Local folder").grid(row=0, column=0, sticky="w", padx=(0, 6), pady=(0, 4))
        ttk.Entry(local_controls, textvariable=self.local_root_var).grid(row=0, column=1, sticky="ew", padx=(0, 8), pady=(0, 4))
        ttk.Button(local_controls, text="Browse…", command=self.choose_local_folder).grid(row=0, column=2, sticky="w", pady=(0, 4))
        local_controls.columnconfigure(1, weight=1)

        remote_controls = ttk.Frame(remote_frame)
        remote_controls.pack(fill="x", pady=(0, 8))
        ttk.Label(remote_controls, text="Remote folder").grid(row=0, column=0, sticky="w", padx=(0, 6), pady=(0, 4))
        ttk.Entry(remote_controls, textvariable=self.remote_root_var).grid(row=0, column=1, sticky="ew", padx=(0, 8), pady=(0, 4))
        remote_controls.columnconfigure(1, weight=1)

        refresh_buttons = [
            ("Refresh local", self.refresh_local_files),
            ("Refresh remote", self.refresh_remote_files_thread),
        ]
        upload_buttons = [
            ("Upload selected", self.upload_selected_thread),
            ("Upload changed", self.upload_changed_thread),
            ("Upload all", self.upload_all_thread),
        ]
        download_buttons = [
            ("Download selected", self.download_selected_thread),
            ("Download changed", self.download_changed_thread),
            ("Download all", self.download_all_thread),
        ]
        other_buttons = [
            ("Open URL", self.open_test_url),
        ]

        row = 0

        for label, command in refresh_buttons:
            ttk.Button(action_frame, text=label, command=command, width=18).grid(
                row=row, column=0, sticky="ew", pady=(0, 8)
            )
            row += 1

        ttk.Separator(action_frame, orient="horizontal").grid(row=row, column=0, sticky="ew", pady=(0, 12))
        row += 1

        for label, command in upload_buttons:
            ttk.Button(action_frame, text=label, command=command, width=18).grid(
                row=row, column=0, sticky="ew", pady=(0, 8)
            )
            row += 1

        ttk.Separator(action_frame, orient="horizontal").grid(row=row, column=0, sticky="ew", pady=(0, 12))
        row += 1

        ttk.Button(action_frame, text="Compare", command=self.compare_and_log, width=18).grid(
            row=row, column=0, sticky="ew", pady=(0, 8)
        )
        row += 1

        ttk.Separator(action_frame, orient="horizontal").grid(row=row, column=0, sticky="ew", pady=(0, 12))
        row += 1

        for label, command in download_buttons:
            ttk.Button(action_frame, text=label, command=command, width=18).grid(
                row=row, column=0, sticky="ew", pady=(0, 8)
            )
            row += 1

        ttk.Separator(action_frame, orient="horizontal").grid(row=row, column=0, sticky="ew", pady=(0, 12))
        row += 1

        for label, command in other_buttons:
            ttk.Button(action_frame, text=label, command=command, width=18).grid(
                row=row, column=0, sticky="ew", pady=(0, 8)
            )
            row += 1
        action_frame.columnconfigure(0, weight=1)

        self.local_tree = self._build_tree(local_frame)
        self.remote_tree = self._build_tree(remote_frame)

        ttk.Label(
            local_frame,
            text="Tip: you can select individual files or whole folders before 'Upload selected'.",
        ).pack(anchor="w", pady=(6, 0))

        log_frame = ttk.LabelFrame(main, text="Log", padding=8)
        log_frame.pack(fill="both", expand=False, pady=(10, 0))
        self.log_text = tk.Text(log_frame, height=10, wrap="word")
        log_scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_scroll.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        log_scroll.pack(side="right", fill="y")

        footer = ttk.Frame(main, padding=(0, 10, 0, 0))
        footer.pack(fill="x", expand=False)

        activity_row = ttk.Frame(footer)
        activity_row.pack(fill="x")
        self.activity_label = ttk.Label(activity_row, textvariable=self.activity_var, width=28, anchor="w")
        self.activity_label.pack(side="left", padx=(0, 10))

        self.progress_canvas = tk.Canvas(
            activity_row,
            height=20,
            background="#f2f2f2",
            highlightthickness=1,
            highlightbackground="#b8b8b8",
        )
        self.progress_canvas.pack(side="left", fill="x", expand=True)
        self.progress_canvas.bind("<Configure>", lambda _event: self._render_progress_canvas())

        self.progress_bar_rect = self.progress_canvas.create_rectangle(0, 0, 0, 20, fill="#4f86f7", outline="")
        self.progress_text_item = self.progress_canvas.create_text(10, 10, anchor="w", text="", fill="#202020")

        status = ttk.Label(footer, textvariable=self.status_var, anchor="w")
        status.pack(fill="x", expand=True, pady=(6, 0))

    def _build_tree(self, parent: ttk.Frame) -> ttk.Treeview:
        """Create a reusable folder-style Treeview used for both local and remote panes."""
        container = ttk.Frame(parent)
        container.pack(fill="both", expand=True)
        container.columnconfigure(0, weight=1)
        container.rowconfigure(0, weight=1)

        columns = ("type", "size", "modified", "status")
        tree = ttk.Treeview(container, columns=columns, show="tree headings", selectmode="extended")
        tree.heading("#0", text="Name", anchor="w")
        tree.column("#0", width=200, minwidth=120, anchor="w", stretch=True)

        headings = {
            "type": "Type",
            "size": "Size",
            "modified": "Modified",
            "status": "Status",
        }
        widths = {
            "type": 70,
            "size": 60,
            "modified": 130,
            "status": 110,
        }
        for column in columns:
            tree.heading(column, text=headings[column], anchor="w")
            tree.column(column, width=widths[column], minwidth=50, anchor="w", stretch=True)

        yscroll = ttk.Scrollbar(container, orient="vertical", command=tree.yview)
        xscroll = ttk.Scrollbar(container, orient="horizontal", command=tree.xview)
        tree.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)

        tree.grid(row=0, column=0, sticky="nsew")
        yscroll.grid(row=0, column=1, sticky="ns")
        xscroll.grid(row=1, column=0, sticky="ew")
        return tree

    def choose_local_folder(self) -> None:
        """Open a folder chooser and store the selected local project root."""
        current = self.local_root_var.get().strip() or str(Path.home())
        selected = filedialog.askdirectory(initialdir=current)
        if selected:
            self.local_root_var.set(selected)
            self.refresh_local_files()

    def _collect_current_form_data(self) -> dict:
        """Collect all current form values into a serializable dictionary."""
        data = {
            "profile_name": self.profile_name_var.get().strip(),
            "host": self.host_var.get().strip(),
            "port": self.port_var.get().strip(),
            "username": self.username_var.get().strip(),
            "local_root": self.local_root_var.get().strip(),
            "remote_root": self.remote_root_var.get().strip(),
            "service_name": self.service_var.get().strip(),
            "test_url": self.url_var.get().strip(),
            "remember_passwords": self.remember_passwords_var.get(),
            "auto_open_url": self.auto_open_url_var.get(),
            "stop_start_service": self.stop_start_service_var.get(),
            "ignore_names": self.ignore_names_var.get().strip(),
        }
        if self.remember_passwords_var.get():
            data["password"] = self.password_var.get()
            data["sudo_password"] = self.sudo_password_var.get()
        return data

    def _apply_form_data(self, data: dict) -> None:
        """Populate the form fields from a saved profile dictionary."""
        self.profile_name_var.set(data.get("profile_name", self.profile_name_var.get()))
        self.host_var.set(data.get("host", ""))
        self.port_var.set(str(data.get("port", "")))
        self.username_var.set(data.get("username", ""))
        self.local_root_var.set(data.get("local_root", ""))
        self.remote_root_var.set(data.get("remote_root", ""))
        self.service_var.set(data.get("service_name", ""))
        self.url_var.set(data.get("test_url", ""))
        self.remember_passwords_var.set(bool(data.get("remember_passwords", True)))
        self.auto_open_url_var.set(bool(data.get("auto_open_url", True)))
        self.stop_start_service_var.set(bool(data.get("stop_start_service", True)))
        self.ignore_names_var.set(data.get("ignore_names", ""))

        if self.remember_passwords_var.get():
            self.password_var.set(data.get("password", ""))
            self.sudo_password_var.set(data.get("sudo_password", ""))
        else:
            self.password_var.set("")
            self.sudo_password_var.set("")

    def refresh_profile_list(self) -> None:
        """Scan the profiles folder and refresh the profile combo box choices."""
        PROFILES_DIR.mkdir(exist_ok=True)
        profile_names: List[str] = []
        for path in sorted(PROFILES_DIR.glob("*.json")):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                profile_names.append(data.get("profile_name") or path.stem)
            except Exception:
                profile_names.append(path.stem)
        self.profile_combo["values"] = profile_names

    def _sanitize_profile_name(self, name: str) -> str:
        """Convert a human profile name into a safe filename stem."""
        safe = re.sub(r"[^A-Za-z0-9._-]+", "_", name.strip())
        return safe.strip("._-") or "profile"

    def _profile_path_for_name(self, profile_name: str) -> Path:
        """Return the JSON path used to store the given profile name."""
        return PROFILES_DIR / f"{self._sanitize_profile_name(profile_name)}.json"

    def save_profile(self) -> None:
        """Save the current form values as a named server profile."""
        profile_name = self.profile_name_var.get().strip()
        if not profile_name:
            entered = simpledialog.askstring("Save profile", "Enter a profile name:", parent=self.root)
            if not entered:
                return
            profile_name = entered.strip()
            self.profile_name_var.set(profile_name)

        if not profile_name:
            messagebox.showwarning("Missing profile name", "Enter a profile name first.")
            return

        data = self._collect_current_form_data()
        data["profile_name"] = profile_name
        profile_path = self._profile_path_for_name(profile_name)

        is_effectively_empty = (
            not data.get("host")
            and not data.get("username")
            and not data.get("local_root")
            and not data.get("remote_root")
            and not data.get("service_name")
            and not data.get("test_url")
            and not data.get("ignore_names")
            and (not data.get("port") or str(data.get("port")).strip() in ("22", ""))
            and (not data.get("password") if data.get("remember_passwords") else True)
            and (not data.get("sudo_password") if data.get("remember_passwords") else True)
        )

        if is_effectively_empty:
            confirm_message = (
                "This profile looks empty (host/username/paths are blank).\n\n"
                "Save it anyway?"
            )
        elif profile_path.exists():
            confirm_message = f"Profile '{profile_name}' already exists.\n\nSave changes and overwrite it?"
        else:
            confirm_message = f"Save changes to profile '{profile_name}'?"

        if not messagebox.askyesno("Confirm save", confirm_message, parent=self.root):
            return

        try:
            profile_path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
        except Exception as exc:
            messagebox.showerror("Save profile failed", f"Could not save profile:\n{exc}")
            return

        self.refresh_profile_list()
        self.log(f"Profile saved: {profile_name} -> {profile_path}")
        self.status_var.set(f"Profile saved: {profile_name}")
        messagebox.showinfo("Profile saved", f"Profile '{profile_name}' saved successfully.")

    def load_profile(self) -> None:
        """Load a named profile from the profiles folder into the current form."""
        requested_name = self.profile_name_var.get().strip()
        profile_path: Optional[Path] = None

        if requested_name:
            candidate = self._profile_path_for_name(requested_name)
            if candidate.exists():
                profile_path = candidate

        if profile_path is None:
            profile_path_str = filedialog.askopenfilename(
                title="Load profile",
                initialdir=str(PROFILES_DIR),
                filetypes=[("JSON profile", "*.json")],
            )
            if not profile_path_str:
                return
            profile_path = Path(profile_path_str)

        try:
            data = json.loads(profile_path.read_text(encoding="utf-8"))
        except Exception as exc:
            messagebox.showerror("Load profile failed", f"Could not read profile:\n{exc}")
            return

        self._apply_form_data(data)
        self.refresh_profile_list()
        self.refresh_local_files()
        self.log(f"Profile loaded: {profile_path}")
        self.status_var.set(f"Profile loaded: {self.profile_name_var.get().strip() or profile_path.stem}")

    def _ignore_names(self) -> set[str]:
        """Return the currently configured set of folder names to skip while scanning."""
        raw = self.ignore_names_var.get().strip()
        if not raw:
            return set()
        return {name.strip() for name in raw.split(",") if name.strip()}

    def refresh_local_files(self) -> None:
        """Scan the local project folder recursively and refresh the local pane."""
        local_root = self.local_root_var.get().strip()
        if not local_root:
            self.local_files = {}
            self.local_scan_done = False
            self._populate_local_tree()
            self.status_var.set("Choose a local folder.")
            return

        root_path = Path(local_root)
        if not root_path.exists() or not root_path.is_dir():
            messagebox.showerror("Invalid folder", "The selected local folder does not exist.")
            self.local_scan_done = False
            return

        ignores = self._ignore_names()
        collected: Dict[str, FileInfo] = {}

        for current_root, dirnames, filenames in os.walk(root_path):
            dirnames[:] = [name for name in dirnames if name not in ignores]
            for filename in filenames:
                if filename in ignores:
                    continue
                path = Path(current_root) / filename
                try:
                    stat_result = path.stat()
                except OSError:
                    continue
                rel_path = path.relative_to(root_path).as_posix()
                collected[rel_path] = FileInfo(
                    rel_path=rel_path,
                    size=stat_result.st_size,
                    mtime=float(stat_result.st_mtime),
                )

        self.local_files = dict(sorted(collected.items()))
        self.local_scan_done = True
        self._populate_local_tree()
        self.compare_current_views()
        self.status_var.set(f"Local scan complete: {len(self.local_files)} files.")
        self.log(f"Local scan complete for {root_path} ({len(self.local_files)} files).")

    def refresh_remote_files_thread(self) -> None:
        """Start remote refresh on a worker thread to keep the UI responsive."""
        self._run_in_thread(self.refresh_remote_files, busy_message="Scanning server files...")

    def refresh_remote_files(self) -> None:
        """Connect to the server, list remote files recursively, and refresh the server pane."""
        self.set_status("Connecting to server and scanning remote files...")
        try:
            ssh, sftp = self._connect()
            try:
                remote_root = self.remote_root_var.get().strip()
                files = self._list_remote_files(sftp, remote_root)
                self.remote_files = dict(sorted(files.items()))
                self.remote_scan_done = True
            finally:
                sftp.close()
                ssh.close()
        except Exception as exc:
            self.log(f"Remote refresh failed: {exc}")
            self.set_status(f"Remote refresh failed: {exc}")
            self._show_error_async("Remote refresh failed", str(exc))
            self.remote_scan_done = False
            return

        self.root.after(0, self._populate_remote_tree)
        self.root.after(0, self.compare_current_views)
        self.set_status(f"Remote scan complete: {len(self.remote_files)} files.")
        self.log(f"Remote scan complete ({len(self.remote_files)} files).")

    def compare_current_views(self) -> None:
        """Update both panes with comparison status between local and remote file lists."""
        tolerance_seconds = 2.0

        for rel_path, info in self.local_files.items():
            remote = self.remote_files.get(rel_path)
            status_text = self._comparison_status(info, remote, tolerance_seconds)
            iid = self.local_path_to_iid.get(rel_path)
            if iid and self.local_tree.exists(iid):
                self.local_tree.set(iid, "status", status_text)

        for rel_path, remote in self.remote_files.items():
            local = self.local_files.get(rel_path)
            if local is None:
                status_text = "Only on server"
            else:
                status_text = self._comparison_status(local, remote, tolerance_seconds)
            iid = self.remote_path_to_iid.get(rel_path)
            if iid and self.remote_tree.exists(iid):
                self.remote_tree.set(iid, "status", status_text)

    def compare_and_log(self) -> None:
        """Run compare and write a short status summary into the log."""
        self.compare_current_views()

        tolerance_seconds = 2.0
        counts: Dict[str, int] = {
            "In sync": 0,
            "New on local": 0,
            "Only on server": 0,
            "Local newer": 0,
            "Remote newer": 0,
            "Different size": 0,
            "Different": 0,
        }

        for rel_path, local_info in self.local_files.items():
            remote_info = self.remote_files.get(rel_path)
            status = self._comparison_status(local_info, remote_info, tolerance_seconds)
            if status == "New file":
                counts["New on local"] += 1
            else:
                counts[status] = counts.get(status, 0) + 1

        for rel_path in self.remote_files:
            if rel_path not in self.local_files:
                counts["Only on server"] += 1

        summary = (
            "Compare summary: "
            f"In sync={counts['In sync']}, "
            f"New on local={counts['New on local']}, "
            f"Only on server={counts['Only on server']}, "
            f"Local newer={counts['Local newer']}, "
            f"Remote newer={counts['Remote newer']}, "
            f"Different size={counts['Different size']}, "
            f"Different={counts['Different']}"
        )
        self.log(summary)
        self.set_status("Compare complete.")

    def _comparison_status(
        self, local_info: FileInfo, remote_info: Optional[FileInfo], tolerance_seconds: float = 2.0
    ) -> str:
        """Compare local and remote metadata and return a human-readable status label."""
        if remote_info is None:
            return "New file"

        mtime_diff = local_info.mtime - remote_info.mtime
        same_time = abs(mtime_diff) <= tolerance_seconds
        same_size = local_info.size == remote_info.size

        if same_time and same_size:
            return "In sync"
        if mtime_diff > tolerance_seconds:
            return "Local newer"
        if mtime_diff < -tolerance_seconds:
            return "Remote newer"
        if not same_size:
            return "Different size"
        return "Different"

    def _populate_local_tree(self) -> None:
        """Render the current local file list into the left-hand Treeview as folders and files."""
        self._populate_hierarchical_tree(
            tree=self.local_tree,
            files=self.local_files,
            path_to_iid=self.local_path_to_iid,
            item_to_relpath=self.local_item_to_relpath,
            prefix="local",
        )

    def _populate_remote_tree(self) -> None:
        """Render the current remote file list into the right-hand Treeview as folders and files."""
        self._populate_hierarchical_tree(
            tree=self.remote_tree,
            files=self.remote_files,
            path_to_iid=self.remote_path_to_iid,
            item_to_relpath=self.remote_item_to_relpath,
            prefix="remote",
        )

    def _populate_hierarchical_tree(
        self,
        tree: ttk.Treeview,
        files: Dict[str, FileInfo],
        path_to_iid: Dict[str, str],
        item_to_relpath: Dict[str, str],
        prefix: str,
    ) -> None:
        """Render files into a folder-style tree, similar to a file explorer."""
        tree.delete(*tree.get_children())
        path_to_iid.clear()
        item_to_relpath.clear()

        folder_iids: Dict[str, str] = {}
        counter = 0

        def next_iid(kind: str) -> str:
            nonlocal counter
            counter += 1
            return f"{prefix}::{kind}::{counter}"

        def ensure_folder(folder_rel_path: str) -> str:
            folder_rel_path = folder_rel_path.strip("./")
            if not folder_rel_path:
                return ""
            if folder_rel_path in folder_iids:
                return folder_iids[folder_rel_path]

            parent_rel = posixpath.dirname(folder_rel_path)
            if parent_rel == ".":
                parent_rel = ""
            parent_iid = ensure_folder(parent_rel) if parent_rel else ""

            iid = next_iid("dir")
            tree.insert(
                parent_iid,
                "end",
                iid=iid,
                text=posixpath.basename(folder_rel_path),
                values=("Folder", "", "", ""),
                open=("/" not in folder_rel_path),
            )
            folder_iids[folder_rel_path] = iid
            return iid

        for rel_path, info in files.items():
            parent_rel = posixpath.dirname(rel_path)
            if parent_rel == ".":
                parent_rel = ""
            parent_iid = ensure_folder(parent_rel) if parent_rel else ""
            iid = next_iid("file")
            tree.insert(
                parent_iid,
                "end",
                iid=iid,
                text=posixpath.basename(rel_path),
                values=(
                    "File",
                    self._format_size(info.size),
                    self._format_mtime(info.mtime),
                    "",
                ),
            )
            path_to_iid[rel_path] = iid
            item_to_relpath[iid] = rel_path

    def upload_selected_thread(self) -> None:
        """Start deployment of manually selected local files on a worker thread."""
        if not self._require_scans(require_local=True, require_remote=True):
            return
        selected_paths = self._selected_local_paths()
        if not selected_paths:
            messagebox.showinfo("Nothing selected", "Select one or more files in the local pane first.")
            return

        if not self._confirm_overwrite_for_upload(selected_paths):
            return
        self._run_in_thread(
            lambda: self.deploy_files(selected_paths, mode_label="selected"),
            busy_message=f"Uploading {len(selected_paths)} selected file(s)...",
            mode="determinate",
            total=len(selected_paths),
        )

    def upload_changed_thread(self) -> None:
        """Start deployment of changed local files compared with the current remote scan."""
        if not self._require_scans(require_local=True, require_remote=True):
            return
        changed_paths = self._changed_local_paths()
        if not changed_paths:
            messagebox.showinfo("Nothing to upload", "No changed files detected in the current comparison.")
            return

        if not self._confirm_overwrite_for_upload(changed_paths):
            return
        self._run_in_thread(
            lambda: self.deploy_files(changed_paths, mode_label="changed"),
            busy_message=f"Uploading {len(changed_paths)} changed file(s)...",
            mode="determinate",
            total=len(changed_paths),
        )

    def deploy_files(self, relative_paths: List[str], mode_label: str = "selected") -> None:
        """Perform the stop/upload/start workflow for the provided list of relative paths."""
        if not (self.local_scan_done and self.remote_scan_done):
            self._show_error_async(
                "Refresh required",
                "Please click 'Refresh local' and 'Refresh remote' before uploading so both panes are up to date.",
            )
            return
        if not self.local_root_var.get().strip():
            self._show_error_async("Missing folder", "Choose a local folder first.")
            return

        if not relative_paths:
            self._show_error_async("Nothing to upload", "No files were selected for upload.")
            return

        self.set_status(f"Deploying {len(relative_paths)} {mode_label} file(s)...")
        self.log(f"Starting deploy of {len(relative_paths)} {mode_label} file(s).")

        ssh = None
        sftp = None
        service_stopped = False
        uploaded_count = 0

        try:
            ssh, sftp = self._connect()
            remote_root = self.remote_root_var.get().strip()
            local_root = Path(self.local_root_var.get().strip())
            service_name = self.service_var.get().strip() or "webapp"

            if self.stop_start_service_var.get():
                self.log(f"Stopping service '{service_name}'...")
                stop_result = self._run_service_command(ssh, "stop", service_name)
                self.log(stop_result)
                service_stopped = True

            total_files = len(relative_paths)
            self.update_progress(0, total_files, f"Uploading files (0/{total_files})")

            for index, rel_path in enumerate(relative_paths, start=1):
                local_path = local_root / Path(rel_path.replace("/", os.sep))
                if not local_path.exists() or not local_path.is_file():
                    self.log(f"Skipped missing local file: {local_path}")
                    self.update_progress(index, total_files, f"Skipping missing file ({index}/{total_files})")
                    continue

                remote_path = posixpath.join(remote_root, rel_path)
                remote_dir = posixpath.dirname(remote_path)
                self._ensure_remote_dirs(sftp, remote_dir)
                self.log(f"Uploading {rel_path} ...")
                self.set_status(f"Uploading {index}/{total_files}: {rel_path}")
                sftp.put(str(local_path), remote_path)
                local_mtime = float(local_path.stat().st_mtime)
                sftp.utime(remote_path, (local_mtime, local_mtime))
                uploaded_count += 1
                self.update_progress(index, total_files, f"Uploading files ({index}/{total_files})")
                self.log(f"Uploaded: {rel_path}")

            if self.stop_start_service_var.get():
                self.log(f"Starting service '{service_name}'...")
                start_result = self._run_service_command(ssh, "start", service_name)
                self.log(start_result)
                service_stopped = False

            self.log("Refreshing remote file list after deploy...")
            self.remote_files = dict(sorted(self._list_remote_files(sftp, remote_root).items()))
            self.remote_scan_done = True
            self.root.after(0, self._populate_remote_tree)
            self.root.after(0, self.compare_current_views)

            if self.auto_open_url_var.get():
                self.open_test_url()

            self.set_status(f"Deploy complete. Uploaded {uploaded_count} file(s).")
            self.log("Deploy complete.")
            self._show_info_async("Upload complete", f"Uploaded {uploaded_count} file(s) successfully.")

        except Exception as exc:
            self.log(f"Deploy failed: {exc}")
            self.set_status(f"Deploy failed: {exc}")
            self._show_error_async("Deploy failed", str(exc))

            if ssh is not None and self.stop_start_service_var.get() and service_stopped:
                try:
                    self.log("Trying to start service again after failure...")
                    result = self._run_service_command(ssh, "start", self.service_var.get().strip() or "webapp")
                    self.log(result)
                except Exception as restart_exc:
                    self.log(f"Automatic recovery start failed: {restart_exc}")
        finally:
            try:
                if sftp is not None:
                    sftp.close()
            except Exception:
                pass
            try:
                if ssh is not None:
                    ssh.close()
            except Exception:
                pass

    def _selected_local_paths(self) -> List[str]:
        """Return relative paths of selected local files, expanding selected folders recursively."""
        selected_items = self.local_tree.selection()
        selected_paths = self._collect_selected_file_paths(
            tree=self.local_tree,
            selected_items=selected_items,
            item_to_relpath=self.local_item_to_relpath,
        )
        return sorted(selected_paths)

    def _collect_selected_file_paths(
        self,
        tree: ttk.Treeview,
        selected_items: Tuple[str, ...],
        item_to_relpath: Dict[str, str],
    ) -> set[str]:
        """Collect file paths from selected file and folder nodes without duplicates."""
        collected: set[str] = set()

        def walk(item_id: str) -> None:
            rel_path = item_to_relpath.get(item_id)
            if rel_path:
                collected.add(rel_path)
            for child in tree.get_children(item_id):
                walk(child)

        for item_id in selected_items:
            walk(item_id)

        return collected

    def _changed_local_paths(self) -> List[str]:
        """Return local files that are missing remotely or differ by size or modification time."""
        changed: List[str] = []
        tolerance_seconds = 2.0

        for rel_path, local_info in self.local_files.items():
            remote_info = self.remote_files.get(rel_path)
            if remote_info is None:
                changed.append(rel_path)
                continue

            if local_info.size != remote_info.size:
                changed.append(rel_path)
                continue

            if abs(local_info.mtime - remote_info.mtime) > tolerance_seconds:
                changed.append(rel_path)

        return changed


    def upload_all_thread(self) -> None:
        """Start deployment of all currently scanned local files."""
        if not self._require_scans(require_local=True, require_remote=True):
            return
        all_paths = sorted(self.local_files.keys())
        if not all_paths:
            messagebox.showinfo("Nothing to upload", "No local files are currently loaded.")
            return

        if not self._confirm_overwrite_for_upload(all_paths):
            return
        self._run_in_thread(
            lambda: self.deploy_files(all_paths, mode_label="all"),
            busy_message=f"Uploading all {len(all_paths)} file(s)...",
            mode="determinate",
            total=len(all_paths),
        )

    def download_selected_thread(self) -> None:
        """Start download of selected remote files or folders into the local folder."""
        if not self._require_scans(require_local=True, require_remote=True):
            return
        selected_paths = self._selected_remote_paths()
        if not selected_paths:
            messagebox.showinfo("Nothing selected", "Select one or more files in the server pane first.")
            return

        if not self._confirm_overwrite_for_download(selected_paths):
            return
        self._run_in_thread(
            lambda: self.download_files(selected_paths),
            busy_message=f"Downloading {len(selected_paths)} file(s)...",
            mode="determinate",
            total=len(selected_paths),
        )

    def _changed_remote_paths(self) -> List[str]:
        """Return remote files that are missing locally or differ by size or modification time."""
        changed: List[str] = []
        tolerance_seconds = 2.0

        for rel_path, remote_info in self.remote_files.items():
            local_info = self.local_files.get(rel_path)
            if local_info is None:
                changed.append(rel_path)
                continue

            if local_info.size != remote_info.size:
                changed.append(rel_path)
                continue

            if abs(local_info.mtime - remote_info.mtime) > tolerance_seconds:
                changed.append(rel_path)

        return changed

    def download_changed_thread(self) -> None:
        """Start download of remote files that differ from local (or are missing locally)."""
        if not self._require_scans(require_local=True, require_remote=True):
            return

        changed_paths = self._changed_remote_paths()
        if not changed_paths:
            messagebox.showinfo("Nothing to download", "No changed remote files detected in the current comparison.")
            return

        if not self._confirm_overwrite_for_download(changed_paths):
            return

        self._run_in_thread(
            lambda: self.download_files(changed_paths),
            busy_message=f"Downloading {len(changed_paths)} changed file(s)...",
            mode="determinate",
            total=len(changed_paths),
        )

    def download_all_thread(self) -> None:
        """Start download of all currently scanned remote files."""
        if not self._require_scans(require_local=True, require_remote=True):
            return
        all_paths = sorted(self.remote_files.keys())
        if not all_paths:
            messagebox.showinfo("Nothing to download", "No remote files are currently loaded. Click 'Refresh remote' first.")
            return

        if not self._confirm_overwrite_for_download(all_paths):
            return
        self._run_in_thread(
            lambda: self.download_files(all_paths),
            busy_message=f"Downloading all {len(all_paths)} file(s)...",
            mode="determinate",
            total=len(all_paths),
        )

    def download_files(self, relative_paths: List[str]) -> None:
        """Download the provided remote relative paths into the local project folder."""
        if not (self.local_scan_done and self.remote_scan_done):
            self._show_error_async(
                "Refresh required",
                "Please click 'Refresh local' and 'Refresh remote' before downloading so both panes are up to date.",
            )
            return
        local_root_text = self.local_root_var.get().strip()
        if not local_root_text:
            self._show_error_async("Missing folder", "Choose a local folder first.")
            return

        local_root = Path(local_root_text)
        local_root.mkdir(parents=True, exist_ok=True)

        self.set_status(f"Downloading {len(relative_paths)} file(s)...")
        self.log(f"Starting download of {len(relative_paths)} file(s).")

        ssh = None
        sftp = None
        downloaded_count = 0

        try:
            ssh, sftp = self._connect()
            remote_root = self.remote_root_var.get().strip()
            total_files = len(relative_paths)
            self.update_progress(0, total_files, f"Downloading files (0/{total_files})")

            for index, rel_path in enumerate(relative_paths, start=1):
                remote_path = posixpath.join(remote_root, rel_path)
                local_path = local_root / Path(rel_path.replace("/", os.sep))
                local_path.parent.mkdir(parents=True, exist_ok=True)

                self.log(f"Downloading {rel_path} ...")
                self.set_status(f"Downloading {index}/{total_files}: {rel_path}")
                sftp.get(remote_path, str(local_path))
                try:
                    remote_stat = sftp.stat(remote_path)
                    os.utime(local_path, (float(remote_stat.st_mtime), float(remote_stat.st_mtime)))
                except Exception:
                    pass
                downloaded_count += 1
                self.update_progress(index, total_files, f"Downloading files ({index}/{total_files})")
                self.log(f"Downloaded: {rel_path}")

            self.refresh_local_files()
            self.remote_files = dict(sorted(self._list_remote_files(sftp, remote_root).items()))
            self.remote_scan_done = True
            self.root.after(0, self._populate_remote_tree)
            self.root.after(0, self.compare_current_views)

            self.set_status(f"Download complete. Downloaded {downloaded_count} file(s).")
            self.log("Download complete.")
            self._show_info_async("Download complete", f"Downloaded {downloaded_count} file(s) successfully.")

        except Exception as exc:
            error_trace = traceback.format_exc()
            self.log(f"Download failed: {exc}")
            self.log(error_trace)
            self.set_status(f"Download failed: {exc}")
            self._show_error_async("Download failed", f"{exc}\n\nTraceback:\n{error_trace}")
        finally:
            try:
                if sftp is not None:
                    sftp.close()
            except Exception:
                pass
            try:
                if ssh is not None:
                    ssh.close()
            except Exception:
                pass

    def _selected_remote_paths(self) -> List[str]:
        """Return relative paths of selected remote files, expanding selected folders recursively."""
        selected_items = self.remote_tree.selection()
        selected_paths = self._collect_selected_file_paths(
            tree=self.remote_tree,
            selected_items=selected_items,
            item_to_relpath=self.remote_item_to_relpath,
        )
        return sorted(selected_paths)

    def _connect(self) -> Tuple["SSHClient", "SFTPClient"]:
        """Open SSH and SFTP connections using the form values and return both handles."""
        if paramiko is None:
            raise RuntimeError("Paramiko is not installed.\n\nInstall it with:\n\npip install paramiko")

        host = self.host_var.get().strip()
        port_text = self.port_var.get().strip() or "22"
        username = self.username_var.get().strip()
        password = self.password_var.get()

        if not host or not username:
            raise ValueError("Host/IP and username are required.")

        try:
            port = int(port_text)
        except ValueError as exc:
            raise ValueError("Port must be a number.") from exc

        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(
            hostname=host,
            port=port,
            username=username,
            password=password,
            timeout=15,
            banner_timeout=15,
            auth_timeout=15,
        )
        sftp = client.open_sftp()
        return client, sftp

    def _run_service_command(self, ssh: "SSHClient", action: str, service_name: str) -> str:
        """Stop or start the configured systemd service, using sudo when needed."""
        username = self.username_var.get().strip()
        base_command = f"systemctl {action} {service_name}"

        if username == "root":
            command = base_command
            stdin, stdout, stderr = ssh.exec_command(command, get_pty=True)
        else:
            sudo_password = self.sudo_password_var.get()
            if not sudo_password:
                raise ValueError("Sudo password is required for non-root accounts.")
            command = f"sudo -S -p '' {base_command}"
            stdin, stdout, stderr = ssh.exec_command(command, get_pty=True)
            stdin.write(sudo_password + "\n")
            stdin.flush()

        exit_code = stdout.channel.recv_exit_status()
        stdout_text = stdout.read().decode("utf-8", errors="replace").strip()
        stderr_text = stderr.read().decode("utf-8", errors="replace").strip()

        if exit_code != 0:
            raise RuntimeError(
                f"Remote command failed ({exit_code}): {command}\n\nSTDOUT:\n{stdout_text}\n\nSTDERR:\n{stderr_text}"
            )

        details = "\n".join(part for part in [stdout_text, stderr_text] if part)
        return f"{action.upper()} OK: {base_command}" + (f"\n{details}" if details else "")

    def _list_remote_files(self, sftp: "SFTPClient", remote_root: str) -> Dict[str, FileInfo]:
        """Recursively walk the remote directory tree and collect file metadata."""
        files: Dict[str, FileInfo] = {}
        ignores = self._ignore_names()

        def walk(directory: str, base_root: str) -> None:
            for entry in sftp.listdir_attr(directory):
                name = entry.filename
                if name in ignores:
                    continue
                full_path = posixpath.join(directory, name)
                if stat.S_ISDIR(entry.st_mode):
                    walk(full_path, base_root)
                else:
                    rel_path = posixpath.relpath(full_path, base_root)
                    files[rel_path] = FileInfo(
                        rel_path=rel_path,
                        size=int(entry.st_size),
                        mtime=float(entry.st_mtime),
                    )

        walk(remote_root, remote_root)
        return files

    def _ensure_remote_dirs(self, sftp: "SFTPClient", remote_dir: str) -> None:
        """Create missing remote directories recursively before uploading a file."""
        if not remote_dir or remote_dir in ("/", "."):
            return

        parts: List[str] = []
        current = remote_dir
        while current not in ("", "/"):
            parts.append(current)
            current = posixpath.dirname(current)

        for path in reversed(parts):
            try:
                sftp.stat(path)
            except FileNotFoundError:
                sftp.mkdir(path)

    def open_test_url(self) -> None:
        """Open the configured test URL in the system default browser."""
        url = self.url_var.get().strip()
        if not url:
            return

        try:
            if sys.platform.startswith("win"):
                os.startfile(url)  # type: ignore[attr-defined]
            elif sys.platform == "darwin":
                subprocess.Popen(["open", url])
            else:
                subprocess.Popen(["xdg-open", url])
        except Exception as exc:
            self.log(f"Could not open URL automatically: {exc}")

    def log(self, message: str) -> None:
        """Append a timestamped message to the log pane in a thread-safe way."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        formatted = f"[{timestamp}] {message}\n"

        def append() -> None:
            self.log_text.insert("end", formatted)
            self.log_text.see("end")

        self.root.after(0, append)

    def set_status(self, message: str) -> None:
        """Update the bottom status label in a thread-safe way."""
        self.root.after(0, lambda: self.status_var.set(message))

    def _render_progress_canvas(self) -> None:
        """Draw the custom progress indicator so it stays visible on all Tk themes."""
        if not hasattr(self, "progress_canvas"):
            return

        canvas = self.progress_canvas
        width = max(canvas.winfo_width(), 40)
        height = max(canvas.winfo_height(), 20)
        canvas.coords(self.progress_bar_rect, 0, 0, 0, height)

        if self._activity_mode == "determinate":
            total = max(self._activity_total, 1)
            current = min(max(self._activity_current, 0), total)
            filled = int(width * (current / total))
            canvas.coords(self.progress_bar_rect, 0, 0, filled, height)
            canvas.itemconfigure(self.progress_bar_rect, state="normal", fill="#4f86f7")
            percent = int((current / total) * 100)
            text = f"{current}/{total}  ({percent}%)"
        elif self._activity_mode == "indeterminate":
            block_width = max(80, width // 5)
            start_x = (self._marquee_offset % (width + block_width)) - block_width
            end_x = start_x + block_width
            canvas.coords(self.progress_bar_rect, start_x, 0, end_x, height)
            canvas.itemconfigure(self.progress_bar_rect, state="normal", fill="#4f86f7")
            text = "Working..."
        else:
            canvas.coords(self.progress_bar_rect, 0, 0, 0, height)
            canvas.itemconfigure(self.progress_bar_rect, state="hidden")
            text = ""

        canvas.coords(self.progress_text_item, width / 2, height / 2)
        canvas.itemconfigure(self.progress_text_item, text=text, anchor="center")

    def _animate_marquee(self) -> None:
        """Animate the custom indeterminate progress bar."""
        if self._activity_mode != "indeterminate" or self._busy_counter <= 0:
            self._marquee_job = None
            return

        self._marquee_offset += 18
        self._render_progress_canvas()
        self._marquee_job = self.root.after(70, self._animate_marquee)

    def update_progress(self, current: int, total: Optional[int] = None, message: Optional[str] = None) -> None:
        """Update the determinate progress bar safely from any thread."""
        def apply() -> None:
            if total is not None:
                self._activity_total = max(int(total), 1)
            self._activity_current = max(int(current), 0)
            self._activity_mode = "determinate"
            if message:
                self.activity_var.set(message)
            self._render_progress_canvas()

        self.root.after(0, apply)

    def _set_busy(
        self,
        is_busy: bool,
        message: Optional[str] = None,
        mode: str = "indeterminate",
        total: Optional[int] = None,
    ) -> None:
        """Start or stop the custom bottom activity indicator in a thread-safe way."""
        def apply() -> None:
            if is_busy:
                self._busy_counter += 1
                self._activity_mode = mode
                self._activity_current = 0
                self._activity_total = max(int(total or 0), 1) if mode == "determinate" else 0
                busy_text = message or "Working..."
                self.activity_var.set(busy_text)
                if message:
                    self.status_var.set(message)
                if mode == "indeterminate":
                    if self._marquee_job is None:
                        self._animate_marquee()
                self._render_progress_canvas()
            else:
                self._busy_counter = max(0, self._busy_counter - 1)
                if self._busy_counter == 0:
                    if self._marquee_job is not None:
                        try:
                            self.root.after_cancel(self._marquee_job)
                        except Exception:
                            pass
                        self._marquee_job = None
                    self._activity_mode = "idle"
                    self._activity_current = 0
                    self._activity_total = 0
                    self.activity_var.set("Idle")
                    self._render_progress_canvas()

        self.root.after(0, apply)

    def _show_error_async(self, title: str, message: str) -> None:
        """Show an error dialog safely from any thread."""
        self.root.after(0, lambda: messagebox.showerror(title, message))

    def _show_info_async(self, title: str, message: str) -> None:
        """Show an info dialog safely from any thread."""
        self.root.after(0, lambda: messagebox.showinfo(title, message))

    def _require_scans(self, *, require_local: bool = True, require_remote: bool = True) -> bool:
        """Return True only if the requested refresh scans have been completed."""
        missing: List[str] = []
        if require_local and not self.local_scan_done:
            missing.append("Refresh local")
        if require_remote and not self.remote_scan_done:
            missing.append("Refresh remote")

        if not missing:
            return True

        messagebox.showinfo(
            "Refresh required",
            "Please refresh both panes before uploading/downloading so you can see local and server files.\n\n"
            + "Missing: "
            + ", ".join(missing),
        )
        return False

    def _confirm_overwrite_dialog(self, title: str, message: str) -> bool:
        """Show a modal dialog with an explicit 'Overwrite' button and return True if confirmed."""
        dialog = tk.Toplevel(self.root)
        dialog.title(title)
        dialog.resizable(False, False)
        dialog.transient(self.root)

        result = {"overwrite": False}

        container = ttk.Frame(dialog, padding=12)
        container.grid(row=0, column=0, sticky="nsew")
        dialog.columnconfigure(0, weight=1)

        ttk.Label(container, text=message, justify="left", wraplength=520).grid(row=0, column=0, sticky="w")

        button_row = ttk.Frame(container)
        button_row.grid(row=1, column=0, sticky="e", pady=(12, 0))

        def do_overwrite() -> None:
            result["overwrite"] = True
            dialog.destroy()

        def do_cancel() -> None:
            dialog.destroy()

        ttk.Button(button_row, text="Cancel", command=do_cancel).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(button_row, text="Overwrite", command=do_overwrite).grid(row=0, column=1)

        dialog.bind("<Escape>", lambda _event: do_cancel())
        dialog.protocol("WM_DELETE_WINDOW", do_cancel)

        dialog.update_idletasks()
        try:
            root_x = self.root.winfo_rootx()
            root_y = self.root.winfo_rooty()
            root_w = self.root.winfo_width()
            root_h = self.root.winfo_height()
            dlg_w = dialog.winfo_width()
            dlg_h = dialog.winfo_height()
            x = root_x + max((root_w - dlg_w) // 2, 0)
            y = root_y + max((root_h - dlg_h) // 2, 0)
            dialog.geometry(f"+{x}+{y}")
        except Exception:
            pass

        dialog.grab_set()
        dialog.wait_window()
        return bool(result["overwrite"])

    def _confirm_overwrite_for_upload(self, relative_paths: List[str]) -> bool:
        existing = sorted([path for path in relative_paths if path in self.remote_files])
        if not existing:
            return True

        preview = "\n".join(existing[:12])
        more = "" if len(existing) <= 12 else f"\n... (+{len(existing) - 12} more)"
        message = (
            f"{len(existing)} file(s) already exist on the server and would be overwritten:\n\n"
            f"{preview}{more}\n\n"
            "Click 'Overwrite' to continue."
        )
        return self._confirm_overwrite_dialog("Overwrite on server?", message)

    def _confirm_overwrite_for_download(self, relative_paths: List[str]) -> bool:
        local_root_text = self.local_root_var.get().strip()
        if not local_root_text:
            messagebox.showerror("Missing folder", "Choose a local folder first.")
            return False

        local_root = Path(local_root_text)
        existing: List[str] = []
        for rel_path in relative_paths:
            local_path = local_root / Path(rel_path.replace("/", os.sep))
            if local_path.exists() and local_path.is_file():
                existing.append(rel_path)

        existing.sort()
        if not existing:
            return True

        preview = "\n".join(existing[:12])
        more = "" if len(existing) <= 12 else f"\n... (+{len(existing) - 12} more)"
        message = (
            f"{len(existing)} file(s) already exist locally and would be overwritten:\n\n"
            f"{preview}{more}\n\n"
            "Click 'Overwrite' to continue."
        )
        return self._confirm_overwrite_dialog("Overwrite local files?", message)

    def _run_in_thread(
        self,
        target: Callable[[], None],
        busy_message: str = "Working...",
        mode: str = "indeterminate",
        total: Optional[int] = None,
    ) -> None:
        """Run a potentially slow operation in a daemon thread while updating the custom activity bar."""
        self._set_busy(True, busy_message, mode=mode, total=total)

        def worker() -> None:
            try:
                target()
            except Exception as exc:
                error_trace = traceback.format_exc()
                self.log(f"Unexpected error: {exc}")
                self.log(error_trace)
                self.set_status(f"Unexpected error: {exc}")
                self._show_error_async("Unexpected error", f"{exc}\n\nTraceback:\n{error_trace}")
            finally:
                self._set_busy(False)

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()

    @staticmethod
    def _format_mtime(timestamp: float) -> str:
        """Convert a POSIX timestamp into a user-friendly local time string."""
        try:
            return datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d %H:%M:%S")
        except Exception:
            return "-"

    @staticmethod
    def _format_size(size_bytes: int) -> str:
        """Convert raw bytes into a compact human-readable size label."""
        units = ["B", "KB", "MB", "GB", "TB"]
        value = float(size_bytes)
        for unit in units:
            if value < 1024 or unit == units[-1]:
                if unit == "B":
                    return f"{int(value)} {unit}"
                return f"{value:.1f} {unit}"
            value /= 1024
        return f"{size_bytes} B"


def main() -> None:
    """Launch the application after basic dependency checks."""
    if sys.version_info < (3, 10):
        print(
            "This script was written for Python 3.10+. It may still work on older versions, "
            "but 3.10 or newer is recommended."
        )

    root = tk.Tk()
    DeployToolApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
