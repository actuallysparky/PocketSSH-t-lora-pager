#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
src_bin="${repo_root}/build/PocketSSH.bin"
dst_bin="${repo_root}/../PocketSSH-TPager.bin"

if [[ ! -f "${src_bin}" ]]; then
  echo "Missing source binary: ${src_bin}" >&2
  echo "Run idf.py build first." >&2
  exit 2
fi

cp "${src_bin}" "${dst_bin}"
echo "Packaged binary: ${dst_bin}"
