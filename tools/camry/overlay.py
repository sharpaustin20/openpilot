#!/usr/bin/env python3
"""Lay a fetched git ref over the installed tree, under /tmp, so a branch can be benched on the comma without touching
the installed code. The result is OUT/openpilot: every file that differs between HEAD and REF is written from REF,
everything else is a symlink into the installed repo (so the prebuilt .so files are found). Symlinks in the repo are
reproduced with the same (relative) target, and the repo's openpilot/ package directory (whose entries are links
like common -> ../common) is rebuilt, so `import openpilot.x` resolves inside the overlay.

  cd /data/openpilot && git fetch origin BRANCH && python3 /tmp/overlay.py FETCH_HEAD /tmp/pr9
  PYTHONPATH=/tmp/pr9/openpilot PYTHONDONTWRITEBYTECODE=1 python3 system/hardware/test_job.py \\
      /usr/local/venv/bin/python3 /tmp/pr9/openpilot/tools/camry/follow_bench.py --quick

Writes only under OUT (reads the repo with git show, no checkout). __pycache__ entries are not linked, so with
PYTHONDONTWRITEBYTECODE=1 nothing is written into the installed tree.
"""
import os
import subprocess
import sys


def main():
  if len(sys.argv) < 3:
    sys.exit(__doc__)
  ref, out = sys.argv[1], sys.argv[2]
  repo = sys.argv[3] if len(sys.argv) > 3 else "/data/openpilot"
  if not out.startswith("/tmp/"):
    sys.exit("OUT must be under /tmp")
  root = os.path.join(out, "openpilot")
  if os.path.lexists(root):
    sys.exit(f"{root} exists; remove it first")
  changed = set(subprocess.check_output(["git", "-C", repo, "diff", "--name-only", "HEAD", ref], text=True).split())
  real_dirs = set()
  for f in changed:
    d = os.path.dirname(f)
    while d:
      real_dirs.add(d)
      d = os.path.dirname(d)
  pkg = os.path.join(repo, "openpilot")
  if os.path.isdir(pkg) and not os.path.islink(pkg):
    real_dirs.add("openpilot")   # its relative links must point into the overlay, not the installed tree
  os.makedirs(root)

  def expand(rel):
    src = os.path.join(repo, rel) if rel else repo
    for name in sorted(os.listdir(src)):
      child = os.path.join(rel, name) if rel else name
      if name == "__pycache__" or child in changed:
        continue
      src_child = os.path.join(src, name)
      if os.path.islink(src_child):
        os.symlink(os.readlink(src_child), os.path.join(root, child))   # same target: relative links stay in the overlay
      elif child in real_dirs:
        os.makedirs(os.path.join(root, child))
        expand(child)
      else:
        os.symlink(src_child, os.path.join(root, child))

  expand("")
  written = 0
  for f in sorted(changed):
    if subprocess.run(["git", "-C", repo, "cat-file", "-e", f"{ref}:{f}"], capture_output=True).returncode != 0:
      continue   # deleted in REF: left out
    data = subprocess.check_output(["git", "-C", repo, "show", f"{ref}:{f}"])
    os.makedirs(os.path.dirname(os.path.join(root, f)) or root, exist_ok=True)
    with open(os.path.join(root, f), "wb") as fh:
      fh.write(data)
    written += 1
  print(f"{root}: {written} files from {ref} over {repo} ({len(changed) - written} removed)")


if __name__ == "__main__":
  main()
