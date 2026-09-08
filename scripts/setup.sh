#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
THIRD_DIR="${ROOT_DIR}/3rd"
INSTALL_DIR="${THIRD_DIR}/install"

if [[ "$(uname -s)" != "Linux" || ! -x "$(command -v apt-get || true)" ]]; then
    echo "当前脚本只支持 Debian/Ubuntu 系 Linux 环境。" >&2
    exit 1
fi

for command in git cmake; do
    if ! command -v "${command}" >/dev/null 2>&1; then
        echo "未找到 ${command}，请先安装基础构建工具。" >&2
        exit 1
    fi
done

if [[ "${EUID}" -eq 0 ]]; then
    SUDO=""
else
    SUDO="sudo"
fi

${SUDO} apt-get install -y build-essential cmake libeigen3-dev

cd "${ROOT_DIR}"
git submodule update --init --recursive

if [[ ! -f "${THIRD_DIR}/osqp/CMakeLists.txt" || ! -f "${THIRD_DIR}/osqp-eigen/CMakeLists.txt" ]]; then
    echo "OSQP 或 OsqpEigen 子模块未初始化。" >&2
    exit 1
fi

cmake -S "${THIRD_DIR}/osqp" -B "${THIRD_DIR}/build-osqp" \
    -DCMAKE_BUILD_TYPE=Release \
    -DUNITTESTS=OFF \
    -DPYTHON=OFF \
    -DMATLAB=OFF \
    -DR_LANG=OFF \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}"
cmake --build "${THIRD_DIR}/build-osqp" --parallel
cmake --install "${THIRD_DIR}/build-osqp"

cmake -S "${THIRD_DIR}/osqp-eigen" -B "${THIRD_DIR}/build-osqp-eigen" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -DCMAKE_PREFIX_PATH="${INSTALL_DIR}" \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}"
cmake --build "${THIRD_DIR}/build-osqp-eigen" --parallel
cmake --install "${THIRD_DIR}/build-osqp-eigen"

echo "OSQP 和 OsqpEigen 已构建并安装到 ${INSTALL_DIR}。"
