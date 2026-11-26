#!/bin/bash
# Desfaz FACE_FRAGMENTADA (09/11): volta os backups de 16:19 e tira os testes/dados novos.
set -e
cd ~/robo_jazzy_ws/src/robo_navigation
for f in scripts/face_ws.py scripts/alinhador_passagem.py test/test_face_proxima.py test/test_freio_cruzamento.py; do
  cp -p "$f.bak_face_frag_20261109_161912" "$f"; echo "restaurado $f"
done
mkdir -p ~/robo_jazzy_ws/docs/face_fragmentada_20261109/removidos
mv test/test_face_fragmentada.py test/dados/missao32_ws1_scans.json ~/robo_jazzy_ws/docs/face_fragmentada_20261109/removidos/ 2>/dev/null || true
grep -c FACE_FRAGMENTADA scripts/face_ws.py scripts/alinhador_passagem.py || echo "OK: nenhuma marca FACE_FRAGMENTADA restante"
