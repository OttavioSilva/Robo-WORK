#!/usr/bin/env python3
"""Ajuste reversivel de deadband linear no YAML de Nav2.

Aceita listas YAML inline e em bloco. Nao inicia nem reinicia o Nav2.
Modo padrao: pre-visualizacao. --apply: backup e substituicao atomica.
"""
import argparse
import copy
import datetime
import os
import re
import shutil
import tempfile
from pathlib import Path

import yaml

TARGET = [0.005, 0.005, 0.015]


def edit_yaml(text):
    parsed = yaml.safe_load(text)
    if not isinstance(parsed, dict):
        raise ValueError("YAML nao contem mapa de nos")
    node = parsed.get("velocity_smoother", {})
    params = node.get("ros__parameters", {}) if isinstance(node, dict) else {}
    old = params.get("deadband_velocity")
    if not (isinstance(old, list) and len(old) == 3
            and all(isinstance(v, (int, float)) for v in old)):
        raise ValueError("deadband_velocity nao e uma lista de tres numeros")
    if abs(float(old[2]) - TARGET[2]) > 1e-9:
        raise ValueError("deadband angular difere de 0.015: revisao manual necessaria")

    lines = text.splitlines(keepends=True)
    anchors = [i for i, line in enumerate(lines)
               if line.rstrip("\r\n").strip() == "velocity_smoother:"
               and not line.startswith((" ", "\t"))]
    if len(anchors) != 1:
        raise ValueError("Nao existe uma unica secao velocity_smoother")
    start = anchors[0] + 1
    stop = next((i for i in range(start, len(lines))
                 if lines[i].strip()
                 and not lines[i].startswith((" ", "\t", "#"))
                 ), len(lines))
    indices = [i for i in range(start, stop)
               if re.match(r"^[ \t]+deadband_velocity[ \t]*:", lines[i])]
    if len(indices) != 1:
        raise ValueError("deadband_velocity ambiguo no bloco velocity_smoother")

    i = indices[0]
    header = lines[i]
    inline = re.match(r"^([ \t]*deadband_velocity[ \t]*:[ \t]*)"
                      r"\[([^\]\n]*)\]([ \t]*(?:#.*)?)(\r?\n?)$", header)
    if inline:
        lines[i] = (inline.group(1) + "[0.005, 0.005, 0.015]" +
                    inline.group(3) + inline.group(4))
    else:
        if not re.match(r"^[ \t]*deadband_velocity[ \t]*:[ \t]*(?:#.*)?\r?\n?$", header):
            raise ValueError("Formato de deadband nao reconhecido")
        edited = 0
        j = i + 1
        while j < stop and edited < 3:
            if not lines[j].strip() or lines[j].lstrip().startswith("#"):
                j += 1
                continue
            entry = re.match(
                r"^([ \t]*-[ \t]*)([-+]?(?:\d+(?:\.\d*)?|\.\d+))"
                r"([ \t]*(?:#.*)?)(\r?\n?)$", lines[j]
            )
            if not entry:
                raise ValueError("Lista em bloco inesperada; original preservado")
            lines[j] = (
                entry.group(1) + ("0.005" if edited < 2 else "0.015")
                + entry.group(3) + entry.group(4)
            )
            edited += 1
            j += 1
        if edited != 3:
            raise ValueError("Nao encontrei os tres elementos de deadband")

    new_text = "".join(lines)
    expected = copy.deepcopy(parsed)
    expected["velocity_smoother"]["ros__parameters"]["deadband_velocity"] = TARGET
    if yaml.safe_load(new_text) != expected:
        raise ValueError("A alteracao afetaria outro parametro: arquivo preservado")
    return old, new_text


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--apply", action="store_true")
    p.add_argument("--config", default=str(
        Path.home() / "robo_jazzy_ws/src/robo_navigation/config/nav2_params.yaml"))
    args = p.parse_args()
    file = Path(args.config)
    original = file.read_text()
    old, updated = edit_yaml(original)
    print("Arquivo:", file)
    print("Atual:", old)
    print("Proposto:", TARGET)
    if updated == original:
        print("Arquivo ja configurado. Sem modificacoes.")
        return
    if not args.apply:
        print("PREVIA: nao fiz nenhuma alteracao.")
        print("Execute novamente com --apply somente com o Nav2 parado.")
        return

    backup = file.with_name(
        file.name + ".bak_deadband_" +
        datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    if backup.exists():
        raise RuntimeError("Backup ja existe; alteracao cancelada")
    shutil.copy2(file, backup)
    fd, tmp_name = tempfile.mkstemp(
        prefix=".nav2_params_deadband_", dir=str(file.parent), text=True)
    try:
        with os.fdopen(fd, "w") as out:
            out.write(updated)
            out.flush()
            os.fsync(out.fileno())
        shutil.copymode(file, tmp_name)
        os.replace(tmp_name, file)
    finally:
        if os.path.exists(tmp_name):
            os.unlink(tmp_name)
    print("BACKUP:", backup)
    print("YAML alterado e revalidado. Reinicie o Nav2 antes de testar.")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, RuntimeError) as exc:
        raise SystemExit("SEM ALTERACAO: " + str(exc))
