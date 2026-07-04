"""Rosbag upload validation and the in-memory rosbag registry.

``sanitize_filename`` is the single choke point for untrusted upload names.
``RosbagStore`` tracks known bags (uploaded or pre-loaded), runs extraction in
background threads, and caches extracted odometry to disk as JSON.
"""

import json
import os
import re
import threading
import uuid

from rosbag_reader import RosbagError, extract_odometry

ALLOWED_EXTENSIONS = {".db3", ".mcap", ".zip"}
_SAFE_NAME = re.compile(r"^[A-Za-z0-9._-]+$")

# Bag lifecycle states.
PENDING = "pending"
EXTRACTING = "extracting"
READY = "ready"
ERROR = "error"


def sanitize_filename(name):
    """Return a safe basename for an uploaded file or raise ``ValueError``.

    Rejects empty names, path separators, parent-directory references, and any
    extension outside :data:`ALLOWED_EXTENSIONS`.
    """
    if not name or not name.strip():
        raise ValueError("Empty filename.")
    name = name.strip()
    base = os.path.basename(name)
    if base != name:
        raise ValueError("Path separators are not allowed in filenames.")
    if base in (".", ".."):
        raise ValueError("Invalid filename.")
    if ".." in base:
        raise ValueError("Parent-directory references are not allowed.")
    if not _SAFE_NAME.match(base):
        raise ValueError(
            "Filename may only contain letters, digits, '.', '_' and '-'."
        )
    ext = os.path.splitext(base)[1].lower()
    if ext not in ALLOWED_EXTENSIONS:
        allowed = ", ".join(sorted(ALLOWED_EXTENSIONS))
        raise ValueError(f"Unsupported file type '{ext}'. Allowed: {allowed}.")
    return base


class RosbagStore:
    """Registry of known rosbags plus async extraction and JSON caching."""

    def __init__(self, upload_dir, cache_dir, topic, heading_offset_rad=0.0):
        self._upload_dir = os.path.abspath(upload_dir)
        self._cache_dir = os.path.abspath(cache_dir)
        self._topic = topic
        self._heading_offset = heading_offset_rad
        self._bags = {}
        self._lock = threading.Lock()
        os.makedirs(self._upload_dir, exist_ok=True)
        os.makedirs(self._cache_dir, exist_ok=True)
        self._scan_existing()

    # -- registration -------------------------------------------------------

    def _scan_existing(self):
        """Register bags already present in the upload directory."""
        for entry in sorted(os.listdir(self._upload_dir)):
            full = os.path.join(self._upload_dir, entry)
            is_bag_dir = os.path.isdir(full) and self._looks_like_bag_dir(full)
            is_bag_file = os.path.isfile(full) and (
                os.path.splitext(entry)[1].lower() in ALLOWED_EXTENSIONS
            )
            if is_bag_dir or is_bag_file:
                self._register(entry, full)

    @staticmethod
    def _looks_like_bag_dir(path):
        try:
            names = {e.lower() for e in os.listdir(path)}
        except OSError:
            return False
        return "metadata.yaml" in names or any(
            n.endswith((".mcap", ".db3")) for n in names
        )

    def _register(self, filename, path):
        bag_id = self._make_id(filename)
        with self._lock:
            self._bags[bag_id] = {
                "id": bag_id,
                "filename": filename,
                "path": path,
                "size": _path_size(path),
                "status": PENDING,
                "progress": 0,
                "metadata": None,
                "error": None,
            }
        return bag_id

    @staticmethod
    def _make_id(filename):
        stem = os.path.splitext(os.path.basename(filename))[0]
        stem = re.sub(r"[^A-Za-z0-9_-]", "_", stem)[:40] or "bag"
        return f"{stem}-{uuid.uuid4().hex[:8]}"

    def register_upload(self, safe_filename, saved_path):
        """Register a freshly-saved upload; caller supplies a sanitized name."""
        return self._register(safe_filename, saved_path)

    def dest_path(self, safe_filename):
        """Collision-free destination path for a sanitized filename."""
        dest = os.path.join(self._upload_dir, safe_filename)
        if not os.path.exists(dest):
            return dest
        stem, ext = os.path.splitext(safe_filename)
        return os.path.join(self._upload_dir, f"{stem}-{uuid.uuid4().hex[:8]}{ext}")

    # -- queries ------------------------------------------------------------

    def list(self):
        with self._lock:
            return [self._summary(b) for b in self._bags.values()]

    def _get(self, bag_id):
        bag = self._bags.get(bag_id)
        if bag is None:
            raise KeyError(bag_id)
        return bag

    @staticmethod
    def _summary(bag):
        return {
            "id": bag["id"],
            "filename": bag["filename"],
            "size": bag["size"],
            "status": bag["status"],
            "progress": bag["progress"],
            "metadata": bag["metadata"],
            "error": bag["error"],
        }

    def status(self, bag_id):
        with self._lock:
            return self._summary(self._get(bag_id))

    def metadata(self, bag_id):
        with self._lock:
            bag = self._get(bag_id)
            return bag["metadata"]

    def odometry(self, bag_id):
        """Return the cached extraction result, or ``None`` if not ready."""
        with self._lock:
            bag = self._get(bag_id)
            if bag["status"] != READY:
                return None
            cache_path = self._cache_path(bag_id)
        with open(cache_path, "r", encoding="utf-8") as fh:
            return json.load(fh)

    def delete(self, bag_id):
        with self._lock:
            bag = self._bags.pop(bag_id, None)
        if bag is None:
            raise KeyError(bag_id)
        _remove_path(bag["path"])
        _remove_path(self._cache_path(bag_id))

    # -- extraction ---------------------------------------------------------

    def ensure_extraction(self, bag_id):
        """Start extraction if the bag is pending. Idempotent."""
        with self._lock:
            bag = self._get(bag_id)
            if bag["status"] != PENDING:
                return
            bag["status"] = EXTRACTING
            bag["progress"] = 0
            bag["error"] = None
        thread = threading.Thread(
            target=self._extract_worker, args=(bag_id,), daemon=True
        )
        thread.start()

    def _extract_worker(self, bag_id):
        path = self._bags[bag_id]["path"]

        def on_progress(pct):
            with self._lock:
                if bag_id in self._bags:
                    self._bags[bag_id]["progress"] = pct

        try:
            result = extract_odometry(
                path, self._topic, self._heading_offset, on_progress
            )
            cache_path = self._cache_path(bag_id)
            with open(cache_path, "w", encoding="utf-8") as fh:
                json.dump(result, fh)
            with self._lock:
                if bag_id in self._bags:
                    self._bags[bag_id]["metadata"] = result["metadata"]
                    self._bags[bag_id]["status"] = READY
                    self._bags[bag_id]["progress"] = 100
        except RosbagError as exc:
            self._fail(bag_id, str(exc))
        except Exception as exc:  # noqa: BLE001 - never let the thread die silently
            self._fail(bag_id, f"Unexpected extraction error: {exc}")

    def _fail(self, bag_id, message):
        with self._lock:
            if bag_id in self._bags:
                self._bags[bag_id]["status"] = ERROR
                self._bags[bag_id]["error"] = message

    def _cache_path(self, bag_id):
        return os.path.join(self._cache_dir, f"{bag_id}.json")


def _path_size(path):
    if os.path.isfile(path):
        return os.path.getsize(path)
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass
    return total


def _remove_path(path):
    import shutil

    if os.path.isdir(path):
        shutil.rmtree(path, ignore_errors=True)
    elif os.path.exists(path):
        try:
            os.remove(path)
        except OSError:
            pass
