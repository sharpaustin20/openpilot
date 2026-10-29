#!/usr/bin/env python3
"""
Rebuild the longitudinal MPC solver of an overlay tree on the comma, without scons, the way
selfdrive/controls/lib/longitudinal_mpc_lib/SConscript does: generate the C code from long_mpc.py with the acados
package's acados_template, then compile libacados_ocp_solver_long.so against the acados libraries (and, with --pyx,
the Cython wrapper acados_ocp_solver_pyx.so, which only depends on the model name, not on the problem sizes).

  python3 /tmp/x/openpilot/tools/camry/build_long_solver.py /tmp/x/openpilot [--pyx] [--check /data/openpilot]

TREE must be an overlay root under /tmp (tools/camry/overlay.py). The overlay links unchanged files and directories
into the installed repo, so first the whole longitudinal_mpc_lib directory is turned into real copies inside the
overlay (a link is only ever removed, never written through), and the build refuses to run unless everything it
writes resolves under /tmp. --check REF lists which generated files differ from REF's (e.g. /data/openpilot, to see
that an unchanged long_mpc.py regenerates the installed code). The installed tree is only read.
"""
import argparse
import filecmp
import os
import shutil
import subprocess
import sys

MPC_LIB = os.path.join("selfdrive", "controls", "lib", "longitudinal_mpc_lib")
GEN = "c_generated_code"
# as in the SConscript
BUILD_FILES = [f"{GEN}/acados_solver_long.c"] + [f"{GEN}/long_model/long_expl_ode_fun.c", f"{GEN}/long_model/long_expl_vde_forw.c"] + \
  [f"{GEN}/long_cost/long_cost_y{s}_{f}.c" for s in ("", "_e", "_0") for f in ("fun", "fun_jac_ut_xt", "hess")] + \
  [f"{GEN}/long_constraints/long_constr_h_fun.c", f"{GEN}/long_constraints/long_constr_h_fun_jac_uxt_zt.c"]
ACADOS_LIBS = ["libacados.so", "libblasfeo.so", "libhpipm.so", "libqpOASES_e.so.3.1"]


def under_tmp(path: str) -> bool:
  return os.path.realpath(path).startswith("/tmp/")


def materialize(tree: str) -> str:
  """Every directory from TREE down to longitudinal_mpc_lib, and everything inside it, becomes a real copy in TREE."""
  cur = tree
  for part in MPC_LIB.split(os.sep):
    nxt = os.path.join(cur, part)
    if os.path.islink(nxt):
      target = os.path.realpath(nxt)
      os.unlink(nxt)   # removes the link only
      os.mkdir(nxt)
      for name in os.listdir(target):   # the siblings stay links, as the overlay had them
        if name != "__pycache__":
          os.symlink(os.path.join(target, name), os.path.join(nxt, name))
    cur = nxt
  lib = cur

  def real_copy(path):
    if os.path.islink(path):
      target = os.path.realpath(path)
      os.unlink(path)
      if os.path.isdir(target):
        shutil.copytree(target, path, symlinks=False, ignore=shutil.ignore_patterns("__pycache__"))
      else:
        shutil.copy2(target, path)
    if os.path.isdir(path) and not os.path.islink(path):
      for name in os.listdir(path):
        real_copy(os.path.join(path, name))
  real_copy(lib)
  for root, dirs, files in os.walk(lib):
    for name in dirs + files:
      p = os.path.join(root, name)
      if os.path.islink(p) or not under_tmp(p):
        sys.exit(f"refusing: {p} is still a link or outside /tmp")
  return lib


def run(cmd, cwd, env=None):
  print("+ " + " ".join(cmd), flush=True)
  subprocess.run(cmd, cwd=cwd, env=env, check=True, stdin=subprocess.DEVNULL)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("tree", help="overlay root under /tmp (OUT/openpilot from overlay.py)")
  ap.add_argument("--pyx", action="store_true", help="also rebuild the Cython wrapper")
  ap.add_argument("--check", metavar="REF", help="compare the generated sources with REF's")
  a = ap.parse_args()
  tree = os.path.abspath(a.tree)
  if not tree.startswith("/tmp/") or not under_tmp(tree):
    sys.exit("TREE must be under /tmp")

  import acados
  base = os.path.dirname(acados.__file__)
  inc = getattr(acados, "INCLUDE_DIR", os.path.join(base, "install", "include"))
  libdir = getattr(acados, "LIB_DIR", os.path.join(base, "install", "lib"))
  templates = getattr(acados, "TEMPLATE_DIR", os.path.join(base, "acados_template"))
  # acados_template renders the C code with its t_renderer: use the one installed on the comma, never let it fetch one
  tera = [p for p in (os.environ.get("TERA_PATH"), getattr(acados, "TERA_PATH", None), os.path.join(base, "install", "bin", "t_renderer"),
                      os.path.join(base, "bin", "t_renderer"), shutil.which("t_renderer")) if p and os.path.isfile(p)]
  if not tera:
    sys.exit("t_renderer not found next to the acados package; not downloading one")
  lib = materialize(tree)
  gen = os.path.join(lib, GEN)
  wrapper = os.path.join(gen, "acados_ocp_solver_pyx.so")
  kept = os.path.join(lib, "acados_ocp_solver_pyx.so.kept")
  shutil.copy2(wrapper, kept)

  # 1. generate (long_mpc.py writes c_generated_code/ and acados_ocp_long.json next to itself; acados_template finds its
  # t_renderer under ACADOS_SOURCE_DIR)
  env = dict(os.environ, PYTHONPATH=tree, PYTHONDONTWRITEBYTECODE="1", TERA_PATH=tera[0])
  env.setdefault("ACADOS_SOURCE_DIR", os.path.dirname(os.path.dirname(tera[0])))
  print(f"t_renderer: {tera[0]}")
  run([sys.executable, "long_mpc.py"], cwd=lib, env=env)   # stdin closed: acados can't stop to ask for a download
  if not os.path.exists(wrapper):
    shutil.copy2(kept, wrapper)
  for lib_name in ACADOS_LIBS:
    shutil.copy2(os.path.join(libdir, lib_name), os.path.join(gen, lib_name))

  # 2. the solver library
  cc = os.environ.get("CC") or shutil.which("clang") or "gcc"   # openpilot builds with clang
  includes = ["-I" + p for p in (inc, os.path.join(inc, "acados"), os.path.join(inc, "blasfeo", "include"),
                                 os.path.join(inc, "hpipm", "include"))]
  flags = ["-shared", "-fPIC", "-O2", "-g", "-DACADOS_WITH_QPOASES", "-Wno-unused"]
  out = os.path.join(gen, "libacados_ocp_solver_long.so")
  run([cc, *flags, *includes, *BUILD_FILES, "-L" + libdir, "-lacados", "-lhpipm", "-lblasfeo", "-lqpOASES_e", "-lm",
       "-Wl,--disable-new-dtags", "-Wl,-rpath,$ORIGIN", "-o", out], cwd=lib)

  # 3. the Cython wrapper (model name only: the installed one keeps working for the same model, so only on request)
  if a.pyx:
    import numpy
    import sysconfig
    pyx_c = os.path.join(gen, "acados_ocp_solver_pyx.c")
    run([sys.executable, "-m", "cython", "-o", pyx_c, "-I", gen, "-I", templates,
         os.path.join(templates, "acados_ocp_solver_pyx.pyx")], cwd=lib)
    run([cc, *flags, *includes, "-I" + sysconfig.get_paths()["include"], "-I" + numpy.get_include(), pyx_c,
         "-L" + gen, "-lacados_ocp_solver_long", "-Wl,--disable-new-dtags", "-Wl,-rpath,$ORIGIN",
         "-o", os.path.join(gen, "acados_ocp_solver_pyx.so")], cwd=lib)

  print(f"built {out}")
  if a.check:
    ref = os.path.join(a.check, MPC_LIB)
    same, diff = [], []
    for root, _, files in os.walk(gen):
      for name in sorted(files):
        if name.endswith((".so", ".so.3.1")):
          continue
        rel = os.path.relpath(os.path.join(root, name), lib)
        ref_f = os.path.join(ref, rel)
        (same if os.path.exists(ref_f) and filecmp.cmp(os.path.join(lib, rel), ref_f, shallow=False) else diff).append(rel)
    print(f"generated sources identical to {a.check}: {len(same)}; different or new: {len(diff)}")
    for rel in diff:
      print(f"  differs: {rel}")


if __name__ == "__main__":
  main()
