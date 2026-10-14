"""Read-only provenance checks for the pinned reference repositories."""
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def git(path, *args):
    return subprocess.check_output(["git", "-C", str(path), *args], text=True).strip()


sources = json.loads((ROOT / "sources.json").read_text())
for source in sources:
    path = ROOT / source["path"]
    assert git(path, "rev-parse", "HEAD") == source["commit"], source["path"]
    assert git(path, "remote", "get-url", "origin") == source["url"], source["path"]
    assert not git(path, "status", "--porcelain", "--untracked-files=all"), source["path"]
    license_path = path / "LICENSE"
    assert hashlib.sha256(license_path.read_bytes()).hexdigest() == source["license_sha256"], license_path
    gitlink = git(ROOT, "ls-files", "--stage", "--", source["path"]).split()
    assert gitlink[:2] == ["160000", source["commit"]], source["path"]
    print(f"PASS: {source['name']} @ {source['commit']} ({source['license']})")
print(f"PASS: {len(sources)} pinned sources, original LICENSE files, clean checkouts.")
